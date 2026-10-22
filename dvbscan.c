/* DVB-C, DVB-T/T2, DVB-S/S2 scan: lock transponders, then list programs from PAT/SDT.
 *
 *   make
 *   ./dvbscan -a 0 -s t
 *
 * The frontend must be free: stop Flussonic or anything else holding
 * /dev/dvb/adapterN/frontendM. Writes dvbscan_<family>_aN.log in the current directory.
 */

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/dvb/dmx.h>
#include <linux/dvb/frontend.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#define MAX_PROG 256
#define PSI_MAX 4096
#define TS_PKT 188
#define MAX_LIST 32
#define MAX_POINTS 4096

enum { FAM_C, FAM_T, FAM_S, FAM_N };
static const char *fam_name[FAM_N] = {"dvbc", "dvbt", "dvbs"};

struct sysdef {
  const char *name; /* system name in Flussonic dvb_card */
  unsigned delsys;
  int fam;
};

static const struct sysdef systems[] = {
    {"dvbca", SYS_DVBC_ANNEX_A, FAM_C},
    {"dvbt", SYS_DVBT, FAM_T},
    {"dvbt2", SYS_DVBT2, FAM_T},
    {"dvbs", SYS_DVBS, FAM_S},
    {"dvbs2", SYS_DVBS2, FAM_S},
};
#define NSYS ((int)(sizeof(systems) / sizeof(systems[0])))

struct named {
  unsigned v;
  const char *name;
};

static const struct named mod_names[] = {
    {QAM_AUTO, "qam_auto"}, {QAM_AUTO, "auto"}, {QPSK, "qpsk"},     {QAM_16, "qam16"},
    {QAM_32, "qam32"},      {QAM_64, "qam64"},  {QAM_128, "qam128"}, {QAM_256, "qam256"},
    {PSK_8, "psk8"},        {APSK_16, "apsk16"}, {APSK_32, "apsk32"},
};
#define NMOD ((int)(sizeof(mod_names) / sizeof(mod_names[0])))

static const struct named rolloff_names[] = {
    {ROLLOFF_35, "35"}, {ROLLOFF_25, "25"}, {ROLLOFF_20, "20"}, {ROLLOFF_AUTO, "auto"},
};
#define NROLLOFF ((int)(sizeof(rolloff_names) / sizeof(rolloff_names[0])))

static const unsigned c_srs[] = {6875000, 6900000, 6952000, 6111000};
static const unsigned c_mods[] = {QAM_AUTO, QAM_256, QAM_64};
static const unsigned s_srs[] = {27500000, 30000000, 22000000};
static const unsigned s_mods[] = {QPSK};

struct program {
  int pnr;
  int pmt_pid;
  int scrambled;
  char name[96];
  char provider[96];
};

struct psi_pid {
  uint8_t buf[PSI_MAX];
  int len;
};

struct tp {
  int sys;          /* index in systems[] */
  unsigned freq;    /* kHz; for satellite the transponder frequency, not the LNB IF */
  unsigned sr;      /* symbols/s */
  unsigned mod;
  unsigned rolloff;
  unsigned bw;      /* Hz */
  int plp;          /* -1: no PLP filter */
  char pol;         /* h, v, l, r */
};

/* What to try at one frequency: zero / negative fields fall back to command line lists. */
struct point {
  int fam;
  int sys;          /* -1: every selected system of the family */
  unsigned freq;    /* kHz */
  char pol;         /* 0: every --pol */
  unsigned sr;      /* 0: every --sr */
  int mod;          /* -1: every --mod */
  unsigned bw;      /* 0: --bandwidth */
  int plp;          /* -2: --plp */
};

struct mux_info {
  struct program progs[MAX_PROG];
  int nprog;
  int tsid;
  int onid;
  int have_tsid;
  int have_onid;
  uint8_t pat_seen[32];
  int pat_last;
  int have_pat;
  uint8_t sdt_seen[32];
  int sdt_last;
  int have_sdt;
  uint8_t sdt_other_seen_bits[32];
  int sdt_other_last;
  int have_sdt_other;
  int sdt_other_seen;
  struct psi_pid pat;
  struct psi_pid sdt;
  struct tp tp;
  int status;
  int have_cnr;
  long long cnr;     /* 0.001 dB */
  int sig_scale;     /* FE_SCALE_DECIBEL: 0.001 dBm, FE_SCALE_RELATIVE: percent */
  long long sig;
};

#define MAX_NAMES 8192
struct svc_name {
  int onid, tsid, sid;
  int scrambled;
  int from_actual;
  char name[96];
  char provider[96];
};
static struct svc_name name_db[MAX_NAMES];
static int name_db_n;

static struct mux_info *saved_mux;
static int saved_n, saved_cap;

static struct {
  int adapter, frontend, demux;
  int fam;
  unsigned sys_mask;
  unsigned from, to, step; /* kHz */
  unsigned srs[MAX_LIST];
  int nsr;
  unsigned mods[MAX_LIST];
  int nmod;
  char pols[MAX_LIST];
  int npol;
  unsigned bw;
  int plp;
  unsigned lof_lo, lof_hi, slof; /* MHz; lof_hi == 0 means single LOF LNB */
  int diseqc;                    /* 0: off, 1..4: committed port */
  int wait_ms, psi_ms, do_psi;
  const char *list, *out;
} o = {
    .demux = -1, .fam = -1, .plp = -1, .wait_ms = -1, .psi_ms = 3000, .do_psi = 1,
    .lof_lo = 9750, .lof_hi = 10600, .slof = 11700,
};

static volatile sig_atomic_t stop;

static void on_sigint(int sig)
{
  (void)sig;
  stop = 1;
  signal(SIGINT, SIG_DFL);
}

static const char *name_of(const struct named *t, int n, unsigned v)
{
  int i;
  for (i = 0; i < n; i++)
    if (t[i].v == v)
      return t[i].name;
  return "?";
}

static int value_of(const struct named *t, int n, const char *s, unsigned *v)
{
  int i;
  for (i = 0; i < n; i++)
    if (!strcasecmp(t[i].name, s)) {
      *v = t[i].v;
      return 0;
    }
  return -1;
}

static int sys_by_name(const char *s)
{
  int i;
  if (!strcasecmp(s, "dvbc"))
    return 0;
  for (i = 0; i < NSYS; i++)
    if (!strcasecmp(systems[i].name, s))
      return i;
  return -1;
}

static int sys_by_delsys(unsigned delsys)
{
  int i;
  for (i = 0; i < NSYS; i++)
    if (systems[i].delsys == delsys)
      return i;
  return -1;
}

static unsigned fam_mask(int fam)
{
  unsigned m = 0;
  int i;
  for (i = 0; i < NSYS; i++)
    if (systems[i].fam == fam)
      m |= 1u << i;
  return m;
}

/* kHz -> "602", "177.5", "1.712" */
static const char *mhz_str(unsigned khz, char *buf, size_t n)
{
  if (khz % 1000 == 0) {
    snprintf(buf, n, "%u", khz / 1000);
  } else {
    char *e;
    snprintf(buf, n, "%u.%03u", khz / 1000, khz % 1000);
    e = buf + strlen(buf) - 1;
    while (*e == '0')
      *e-- = 0;
  }
  return buf;
}

static unsigned parse_mhz(const char *s)
{
  return (unsigned)(strtod(s, NULL) * 1000.0 + 0.5);
}

static unsigned parse_bw(const char *s)
{
  double d = strtod(s, NULL);
  return d >= 1e5 ? (unsigned)d : (unsigned)(d * 1e6 + 0.5);
}

static int fam_wait(int fam)
{
  if (o.wait_ms > 0)
    return o.wait_ms;
  return fam == FAM_C ? 400 : fam == FAM_T ? 1200 : 600;
}

static const unsigned *fam_srs(int fam, int *n)
{
  if (o.nsr) {
    *n = o.nsr;
    return o.srs;
  }
  if (fam == FAM_S) {
    *n = (int)(sizeof(s_srs) / sizeof(s_srs[0]));
    return s_srs;
  }
  *n = (int)(sizeof(c_srs) / sizeof(c_srs[0]));
  return c_srs;
}

static const unsigned *fam_mods(int fam, int *n)
{
  if (o.nmod) {
    *n = o.nmod;
    return o.mods;
  }
  if (fam == FAM_S) {
    *n = 1;
    return s_mods;
  }
  *n = (int)(sizeof(c_mods) / sizeof(c_mods[0]));
  return c_mods;
}

static const char *fam_pols(int *n)
{
  if (o.npol) {
    *n = o.npol;
    return o.pols;
  }
  *n = 2;
  return "hv";
}

static void fam_grid(int fam, unsigned *from, unsigned *to, unsigned *step)
{
  static const unsigned grid[FAM_N][3] = {
      {114000, 858000, 8000},
      {474000, 858000, 8000},
      {10700000, 12750000, 6000},
  };
  *from = o.from ? o.from : grid[fam][0];
  *to = o.to ? o.to : grid[fam][1];
  *step = o.step ? o.step : grid[fam][2];
}

/* LNB local oscillator for a transponder, kHz. */
static unsigned sat_lof(unsigned freq, int *hi)
{
  *hi = o.lof_hi && freq >= o.slof * 1000u;
  return (*hi ? o.lof_hi : o.lof_lo) * 1000u;
}

static unsigned sat_if(unsigned freq, int *hi)
{
  unsigned lof = sat_lof(freq, hi);
  return freq > lof ? freq - lof : lof - freq;
}

static int sec_setup(int fd, char pol, int hi)
{
  static int cur_v = -1, cur_t = -1;
  fe_sec_voltage_t v = (pol == 'h' || pol == 'l') ? SEC_VOLTAGE_18 : SEC_VOLTAGE_13;
  fe_sec_tone_mode_t t = hi ? SEC_TONE_ON : SEC_TONE_OFF;
  if ((int)v == cur_v && (int)t == cur_t)
    return 0;
  cur_v = cur_t = -1;
  if (ioctl(fd, FE_SET_TONE, SEC_TONE_OFF) < 0 || ioctl(fd, FE_SET_VOLTAGE, v) < 0)
    return -1;
  usleep(20000);
  if (o.diseqc) {
    /* DiSEqC 1.0 committed switch: port, polarization and band in one byte */
    struct dvb_diseqc_master_cmd cmd = {{0xE0, 0x10, 0x38, 0xF0, 0, 0}, 4};
    cmd.msg[3] = (uint8_t)(0xF0 | ((o.diseqc - 1) << 2) | (v == SEC_VOLTAGE_18 ? 2 : 0) | (hi ? 1 : 0));
    if (ioctl(fd, FE_DISEQC_SEND_MASTER_CMD, &cmd) < 0)
      return -1;
    usleep(20000);
    ioctl(fd, FE_DISEQC_SEND_BURST, ((o.diseqc - 1) & 1) ? SEC_MINI_B : SEC_MINI_A);
    usleep(20000);
  }
  if (ioctl(fd, FE_SET_TONE, t) < 0)
    return -1;
  usleep(20000);
  cur_v = (int)v;
  cur_t = (int)t;
  return 0;
}

static int tune(int fd, const struct tp *t)
{
  const struct sysdef *s = &systems[t->sys];
  struct dtv_property clr = {.cmd = DTV_CLEAR};
  struct dtv_properties c1 = {.num = 1, .props = &clr};
  struct dtv_property p[16];
  struct dtv_properties c2 = {.props = p};
  int n = 0;

#define PROP(c, v)          \
  do {                      \
    p[n].cmd = (c);         \
    p[n].u.data = (v);      \
    n++;                    \
  } while (0)

  memset(p, 0, sizeof(p));
  if (s->fam == FAM_S) {
    int hi;
    unsigned ifreq = sat_if(t->freq, &hi);
    if (sec_setup(fd, t->pol, hi) < 0)
      return -1;
    if (ioctl(fd, FE_SET_PROPERTY, &c1) < 0)
      return -1;
    PROP(DTV_DELIVERY_SYSTEM, s->delsys);
    PROP(DTV_FREQUENCY, ifreq);
    PROP(DTV_SYMBOL_RATE, t->sr);
    PROP(DTV_MODULATION, t->mod);
    PROP(DTV_INNER_FEC, FEC_AUTO);
    PROP(DTV_INVERSION, INVERSION_AUTO);
    if (s->delsys == SYS_DVBS2) {
      PROP(DTV_ROLLOFF, ROLLOFF_AUTO);
      PROP(DTV_PILOT, PILOT_AUTO);
    }
  } else {
    if (ioctl(fd, FE_SET_PROPERTY, &c1) < 0)
      return -1;
    PROP(DTV_DELIVERY_SYSTEM, s->delsys);
    PROP(DTV_FREQUENCY, t->freq * 1000u);
    PROP(DTV_INVERSION, INVERSION_AUTO);
    if (s->fam == FAM_C) {
      PROP(DTV_MODULATION, t->mod);
      PROP(DTV_SYMBOL_RATE, t->sr);
      PROP(DTV_INNER_FEC, FEC_AUTO);
    } else {
      PROP(DTV_BANDWIDTH_HZ, t->bw);
      PROP(DTV_MODULATION, QAM_AUTO);
      PROP(DTV_TRANSMISSION_MODE, TRANSMISSION_MODE_AUTO);
      PROP(DTV_GUARD_INTERVAL, GUARD_INTERVAL_AUTO);
      PROP(DTV_HIERARCHY, HIERARCHY_AUTO);
      PROP(DTV_CODE_RATE_HP, FEC_AUTO);
      PROP(DTV_CODE_RATE_LP, FEC_AUTO);
      if (s->delsys == SYS_DVBT2)
        PROP(DTV_STREAM_ID, t->plp >= 0 ? (unsigned)t->plp : NO_STREAM_ID_FILTER);
    }
  }
  PROP(DTV_TUNE, 0);
#undef PROP
  c2.num = (unsigned)n;
  return ioctl(fd, FE_SET_PROPERTY, &c2);
}

static int wait_lock(int fd, int wait_ms)
{
  int slept = 0;
  while (slept < wait_ms && !stop) {
    fe_status_t st = 0;
    if (ioctl(fd, FE_READ_STATUS, &st) >= 0 && (st & FE_HAS_LOCK))
      return (int)st;
    usleep(50000);
    slept += 50;
  }
  return 0;
}

/* Take what the demodulator actually locked on: delivery system, modulation, rolloff, frequency. */
static void refine(int fd, struct tp *t)
{
  struct dtv_property p[4];
  struct dtv_properties c = {.num = 4, .props = p};
  int fam = systems[t->sys].fam, si;
  unsigned v;

  memset(p, 0, sizeof(p));
  p[0].cmd = DTV_DELIVERY_SYSTEM;
  p[1].cmd = DTV_FREQUENCY;
  p[2].cmd = DTV_MODULATION;
  p[3].cmd = DTV_ROLLOFF;
  if (ioctl(fd, FE_GET_PROPERTY, &c) < 0)
    return;

  si = sys_by_delsys(p[0].u.data);
  if (si >= 0 && systems[si].fam == fam)
    t->sys = si;

  v = p[2].u.data;
  if (fam == FAM_C && (v == QAM_16 || v == QAM_32 || v == QAM_64 || v == QAM_128 || v == QAM_256))
    t->mod = v;
  if (fam == FAM_S) {
    if (systems[t->sys].delsys == SYS_DVBS)
      t->mod = QPSK;
    else if (v == QPSK || v == PSK_8 || v == APSK_16 || v == APSK_32)
      t->mod = v;
    v = p[3].u.data;
    if (systems[t->sys].delsys == SYS_DVBS2 && (v == ROLLOFF_35 || v == ROLLOFF_25 || v == ROLLOFF_20))
      t->rolloff = v;
  }

  if (fam == FAM_S && p[1].u.data) {
    int hi;
    unsigned lof = sat_lof(t->freq, &hi), real;
    unsigned ifreq = p[1].u.data;
    real = t->freq > lof ? lof + ifreq : lof - ifreq;
    if ((real > t->freq ? real - t->freq : t->freq - real) <= 10000)
      t->freq = (real + 500) / 1000 * 1000;
  }
}

static void read_stats(int fd, struct mux_info *m)
{
  struct dtv_property p[2];
  struct dtv_properties c = {.num = 2, .props = p};
  memset(p, 0, sizeof(p));
  p[0].cmd = DTV_STAT_CNR;
  p[1].cmd = DTV_STAT_SIGNAL_STRENGTH;
  if (ioctl(fd, FE_GET_PROPERTY, &c) < 0)
    return;
  if (p[0].u.st.len > 0 && p[0].u.st.stat[0].scale == FE_SCALE_DECIBEL) {
    m->have_cnr = 1;
    m->cnr = p[0].u.st.stat[0].svalue;
  }
  if (p[1].u.st.len > 0) {
    if (p[1].u.st.stat[0].scale == FE_SCALE_DECIBEL) {
      m->sig_scale = FE_SCALE_DECIBEL;
      m->sig = p[1].u.st.stat[0].svalue;
    } else if (p[1].u.st.stat[0].scale == FE_SCALE_RELATIVE) {
      m->sig_scale = FE_SCALE_RELATIVE;
      m->sig = (long long)(p[1].u.st.stat[0].uvalue * 100 / 65535);
    }
  }
}

static struct program *prog_get(struct mux_info *m, int pnr)
{
  int i;
  for (i = 0; i < m->nprog; i++)
    if (m->progs[i].pnr == pnr)
      return &m->progs[i];
  if (m->nprog >= MAX_PROG)
    return NULL;
  memset(&m->progs[m->nprog], 0, sizeof(m->progs[0]));
  m->progs[m->nprog].pnr = pnr;
  m->progs[m->nprog].pmt_pid = -1;
  return &m->progs[m->nprog++];
}

static void dvb_text(const uint8_t *s, int len, char *out, int outsz)
{
  int i = 0, o = 0, utf8 = 0, iso5 = 0;
  if (len <= 0 || outsz <= 1) {
    if (outsz > 0)
      out[0] = 0;
    return;
  }
  if (s[0] < 0x20) {
    if (s[0] == 0x01)
      iso5 = 1, i = 1;
    else if (s[0] == 0x15)
      utf8 = 1, i = 1;
    else if (s[0] == 0x10 && len >= 3) {
      if (s[2] == 0x05)
        iso5 = 1;
      i = 3;
    } else if (s[0] == 0x1F && len >= 2)
      i = 2;
    else
      i = 1;
  }
  for (; i < len && o < outsz - 1; i++) {
    unsigned char c = s[i];
    if (c == 0)
      break;
    if (iso5 && c >= 0xA1) {
      unsigned u = 0x0400 + (c - 0xA0);
      if (o + 3 >= outsz)
        break;
      out[o++] = (char)(0xC0 | (u >> 6));
      out[o++] = (char)(0x80 | (u & 0x3F));
    } else if (!utf8 && c >= 0x80) {
      if (o + 3 >= outsz)
        break;
      out[o++] = (char)(0xC0 | (c >> 6));
      out[o++] = (char)(0x80 | (c & 0x3F));
    } else if (c >= 0x20 || c == 0x0A) {
      out[o++] = (char)c;
    }
  }
  out[o] = 0;
}

static int sections_complete(const uint8_t *seen, int last)
{
  int i;
  if (last < 0 || last > 255)
    return 0;
  for (i = 0; i <= last; i++)
    if ((seen[i >> 3] & (1u << (i & 7))) == 0)
      return 0;
  return 1;
}

static void mark_section(uint8_t *seen, int sec)
{
  if (sec >= 0 && sec <= 255)
    seen[sec >> 3] |= (uint8_t)(1u << (sec & 7));
}

static struct svc_name *name_slot(int onid, int tsid, int sid)
{
  int i;
  for (i = 0; i < name_db_n; i++)
    if (name_db[i].onid == onid && name_db[i].tsid == tsid && name_db[i].sid == sid)
      return &name_db[i];
  if (name_db_n >= MAX_NAMES)
    return NULL;
  memset(&name_db[name_db_n], 0, sizeof(name_db[0]));
  name_db[name_db_n].onid = onid;
  name_db[name_db_n].tsid = tsid;
  name_db[name_db_n].sid = sid;
  return &name_db[name_db_n++];
}

static void remember_name(int onid, int tsid, int sid, const char *name, const char *prov,
                          int scrambled, int from_actual)
{
  struct svc_name *n;
  if (!name || !name[0])
    return;
  n = name_slot(onid, tsid, sid);
  if (!n)
    return;
  if (n->name[0] && n->from_actual && !from_actual)
    return;
  snprintf(n->name, sizeof(n->name), "%s", name);
  if (prov && prov[0])
    snprintf(n->provider, sizeof(n->provider), "%s", prov);
  n->scrambled = scrambled;
  n->from_actual = from_actual;
}

static struct program *prog_find(struct mux_info *m, int pnr)
{
  int i;
  for (i = 0; i < m->nprog; i++)
    if (m->progs[i].pnr == pnr)
      return &m->progs[i];
  return NULL;
}

static void apply_catalog(struct mux_info *m)
{
  int i, k;
  for (i = 0; i < m->nprog; i++) {
    struct program *p = &m->progs[i];
    struct svc_name *hit = NULL;
    if (p->name[0])
      continue;
    if (!m->have_tsid)
      continue;
    for (k = 0; k < name_db_n; k++) {
      if (name_db[k].tsid != m->tsid || name_db[k].sid != p->pnr)
        continue;
      if (m->have_onid && name_db[k].onid != m->onid)
        continue;
      if (!hit || (name_db[k].from_actual && !hit->from_actual))
        hit = &name_db[k];
    }
    if (!hit)
      continue;
    snprintf(p->name, sizeof(p->name), "%s", hit->name);
    if (!p->provider[0] && hit->provider[0])
      snprintf(p->provider, sizeof(p->provider), "%s", hit->provider);
  }
}

static void parse_pat(struct mux_info *m, const uint8_t *s, int len)
{
  int i, slen, sec, last;
  if (len < 12 || s[0] != 0x00)
    return;
  if (((s[5] >> 0) & 1) == 0)
    return;
  slen = ((s[1] & 0x0F) << 8) | s[2];
  if (3 + slen > len)
    return;
  m->tsid = (s[3] << 8) | s[4];
  m->have_tsid = 1;
  sec = s[6];
  last = s[7];
  for (i = 8; i + 4 <= 3 + slen - 4; i += 4) {
    int pnr = (s[i] << 8) | s[i + 1];
    int pid = ((s[i + 2] & 0x1F) << 8) | s[i + 3];
    if (pnr == 0)
      continue;
    struct program *p = prog_get(m, pnr);
    if (p)
      p->pmt_pid = pid;
  }
  mark_section(m->pat_seen, sec);
  m->pat_last = last;
  if (sections_complete(m->pat_seen, last))
    m->have_pat = 1;
}

static void take_service_name(const uint8_t *d, int tlen, char *name, int namesz,
                              char *prov, int provsz)
{
  int plen, nlen, name_at, content_end;
  if (tlen < 3)
    return;
  plen = d[3];
  content_end = 2 + tlen;
  if (4 + plen > content_end)
    return;
  dvb_text(d + 4, plen, prov, provsz);
  if (4 + plen >= content_end)
    return;
  nlen = d[4 + plen];
  name_at = 5 + plen;
  if (name_at + nlen > content_end)
    nlen = content_end - name_at;
  if (nlen > 0)
    dvb_text(d + name_at, nlen, name, namesz);
}

static void parse_sdt(struct mux_info *m, const uint8_t *s, int len)
{
  int i, slen, sec, last, end, tsid, onid, actual;
  if (len < 12 || (s[0] != 0x42 && s[0] != 0x46))
    return;
  if (((s[5] >> 0) & 1) == 0)
    return;
  slen = ((s[1] & 0x0F) << 8) | s[2];
  if (3 + slen > len)
    return;
  actual = s[0] == 0x42;
  tsid = (s[3] << 8) | s[4];
  onid = (s[8] << 8) | s[9];
  sec = s[6];
  last = s[7];
  if (actual) {
    m->tsid = tsid;
    m->onid = onid;
    m->have_tsid = 1;
    m->have_onid = 1;
  }
  end = 3 + slen - 4;
  i = 11;
  while (i + 5 <= end) {
    int pnr = (s[i] << 8) | s[i + 1];
    int ca = (s[i + 3] >> 4) & 1;
    int dlen = ((s[i + 3] & 0x0F) << 8) | s[i + 4];
    int j = i + 5, dend = i + 5 + dlen;
    int same = actual || (m->have_tsid && tsid == m->tsid && (!m->have_onid || onid == m->onid));
    char name[96], prov[96];
    struct program *p = NULL;
    name[0] = prov[0] = 0;
    if (dend > end)
      break;
    if (same)
      p = actual ? prog_get(m, pnr) : prog_find(m, pnr);
    if (p)
      p->scrambled = ca;
    while (j + 2 <= dend) {
      int tag = s[j], tlen = s[j + 1];
      if (j + 2 + tlen > dend)
        break;
      if (tag == 0x48)
        take_service_name(s + j, tlen, name, sizeof(name), prov, sizeof(prov));
      j += 2 + tlen;
    }
    if (p && name[0] && !p->name[0])
      snprintf(p->name, sizeof(p->name), "%s", name);
    if (p && prov[0] && !p->provider[0])
      snprintf(p->provider, sizeof(p->provider), "%s", prov);
    remember_name(onid, tsid, pnr, name, prov, ca, actual);
    i = dend;
  }
  if (actual) {
    mark_section(m->sdt_seen, sec);
    m->sdt_last = last;
    if (sections_complete(m->sdt_seen, last))
      m->have_sdt = 1;
  } else {
    m->sdt_other_seen = 1;
    mark_section(m->sdt_other_seen_bits, sec);
    m->sdt_other_last = last;
    if (sections_complete(m->sdt_other_seen_bits, last))
      m->have_sdt_other = 1;
  }
}

static int section_ready(const uint8_t *buf, int len)
{
  int slen;
  if (len < 3)
    return 0;
  slen = ((buf[1] & 0x0F) << 8) | buf[2];
  return len >= 3 + slen;
}

static void feed_psi(struct psi_pid *st, int pusi, const uint8_t *pl, int pll,
                     void (*parse)(struct mux_info *, const uint8_t *, int),
                     struct mux_info *m)
{
  if (pusi) {
    int ptr, off;
    if (pll < 1)
      return;
    ptr = pl[0];
    off = 1 + ptr;
    if (off > pll)
      return;
    if (st->len && section_ready(st->buf, st->len))
      parse(m, st->buf, st->len);
    st->len = 0;
    pl += off;
    pll -= off;
  }
  if (st->len + pll > PSI_MAX)
    st->len = 0;
  if (pll <= 0)
    return;
  memcpy(st->buf + st->len, pl, pll);
  st->len += pll;
  while (section_ready(st->buf, st->len)) {
    int slen = ((st->buf[1] & 0x0F) << 8) | st->buf[2];
    int seclen = 3 + slen;
    parse(m, st->buf, seclen);
    if (st->len > seclen)
      memmove(st->buf, st->buf + seclen, st->len - seclen);
    st->len -= seclen;
    if (st->len == 0)
      break;
  }
}

static void handle_packet(struct mux_info *m, const uint8_t *pkt)
{
  int pusi, pid, afc, off, afl;
  if (pkt[0] != 0x47 || (pkt[1] & 0x80))
    return;
  pusi = (pkt[1] >> 6) & 1;
  pid = ((pkt[1] & 0x1F) << 8) | pkt[2];
  if (pid != 0 && pid != 0x11)
    return;
  afc = (pkt[3] >> 4) & 3;
  off = 4;
  if (afc & 2) {
    afl = pkt[4];
    off += 1 + afl;
  }
  if (!(afc & 1) || off >= TS_PKT)
    return;
  if (pid == 0)
    feed_psi(&m->pat, pusi, pkt + off, TS_PKT - off, parse_pat, m);
  else
    feed_psi(&m->sdt, pusi, pkt + off, TS_PKT - off, parse_sdt, m);
}

static int start_pid_filter(uint16_t pid)
{
  char dmx_path[128];
  struct dmx_pes_filter_params pes;
  int dmx;
  snprintf(dmx_path, sizeof(dmx_path), "/dev/dvb/adapter%d/demux%d", o.adapter, o.demux);
  dmx = open(dmx_path, O_RDWR);
  if (dmx < 0)
    return -1;
  ioctl(dmx, DMX_SET_BUFFER_SIZE, 64 * 1024);
  memset(&pes, 0, sizeof(pes));
  pes.pid = pid;
  pes.input = DMX_IN_FRONTEND;
  pes.output = DMX_OUT_TS_TAP;
  pes.pes_type = DMX_PES_OTHER;
  pes.flags = DMX_IMMEDIATE_START;
  if (ioctl(dmx, DMX_SET_PES_FILTER, &pes) < 0) {
    close(dmx);
    return -1;
  }
  return dmx;
}

static int mux_named(const struct mux_info *m)
{
  int i, k;
  if (!m->have_pat || !m->have_tsid || m->nprog == 0)
    return 0;
  for (i = 0; i < m->nprog; i++) {
    int ok = m->progs[i].name[0] != 0;
    if (ok)
      continue;
    for (k = 0; k < name_db_n; k++) {
      if (name_db[k].tsid == m->tsid && name_db[k].sid == m->progs[i].pnr && name_db[k].name[0]) {
        ok = 1;
        break;
      }
    }
    if (!ok)
      return 0;
  }
  return 1;
}

static void stop_filter(int dmx)
{
  if (dmx < 0)
    return;
  ioctl(dmx, DMX_STOP);
  close(dmx);
}

static int read_programs(int psi_ms, struct mux_info *m)
{
  char dvr_path[128];
  uint8_t buf[32 * TS_PKT];
  struct pollfd pfd;
  int dmx_pat, dmx_sdt, dvr;
  int elapsed = 0;

  dmx_pat = start_pid_filter(0);
  dmx_sdt = start_pid_filter(0x11);
  if (dmx_pat < 0 && dmx_sdt < 0)
    return -1;

  snprintf(dvr_path, sizeof(dvr_path), "/dev/dvb/adapter%d/dvr%d", o.adapter, o.demux);
  dvr = open(dvr_path, O_RDONLY | O_NONBLOCK);
  if (dvr < 0) {
    stop_filter(dmx_pat);
    stop_filter(dmx_sdt);
    return -1;
  }

  pfd.fd = dvr;
  pfd.events = POLLIN;
  while (elapsed < psi_ms && !stop) {
    int n, off, pr = poll(&pfd, 1, 100);
    elapsed += 100;
    if (pr <= 0)
      continue;
    n = (int)read(dvr, buf, sizeof(buf));
    if (n <= 0)
      continue;
    for (off = 0; off + TS_PKT <= n; off += TS_PKT) {
      if (buf[off] != 0x47) {
        int j;
        for (j = off + 1; j + TS_PKT <= n; j++)
          if (buf[j] == 0x47) {
            off = j - TS_PKT;
            break;
          }
        continue;
      }
      handle_packet(m, buf + off);
    }
    if (elapsed >= 500 && mux_named(m))
      break;
  }
  if (m->pat.len && section_ready(m->pat.buf, m->pat.len))
    parse_pat(m, m->pat.buf, m->pat.len);
  if (m->sdt.len && section_ready(m->sdt.buf, m->sdt.len))
    parse_sdt(m, m->sdt.buf, m->sdt.len);
  close(dvr);
  stop_filter(dmx_pat);
  stop_filter(dmx_sdt);
  return 0;
}

static int cmp_prog(const void *a, const void *b)
{
  return ((const struct program *)a)->pnr - ((const struct program *)b)->pnr;
}

static void print_programs(FILE *out, struct mux_info *m)
{
  int i;
  if (m->nprog == 0) {
    fprintf(out, "  (no PAT/SDT)\n");
    return;
  }
  qsort(m->progs, (size_t)m->nprog, sizeof(m->progs[0]), cmp_prog);
  for (i = 0; i < m->nprog; i++) {
    const struct program *p = &m->progs[i];
    const char *name = p->name[0] ? p->name : "?";
    fprintf(out, "  program %-5d %s", p->pnr, name);
    if (p->provider[0])
      fprintf(out, "  [%s]", p->provider);
    if (p->scrambled)
      fprintf(out, "  scrambled");
    fprintf(out, "\n");
  }
}

static int named_count(const struct mux_info *m)
{
  int i, n = 0;
  for (i = 0; i < m->nprog; i++)
    if (m->progs[i].name[0])
      n++;
  return n;
}

/* "602 MHz", "11766 MHz H" */
static const char *tp_label(const struct tp *t, char pol, char *buf, size_t n)
{
  char f[32];
  mhz_str(t->freq, f, sizeof(f));
  if (pol)
    snprintf(buf, n, "%s MHz %c", f, toupper((unsigned char)pol));
  else
    snprintf(buf, n, "%s MHz", f);
  return buf;
}

/* key=value line that --list reads back */
static void print_params(FILE *out, const struct tp *t)
{
  const struct sysdef *s = &systems[t->sys];
  char f[32];
  fprintf(out, "system=%s freq=%s", s->name, mhz_str(t->freq, f, sizeof(f)));
  if (s->fam == FAM_S)
    fprintf(out, " pol=%c", t->pol);
  if (s->fam != FAM_T)
    fprintf(out, " sr=%u mod=%s", t->sr / 1000, name_of(mod_names, NMOD, t->mod));
  if (s->delsys == SYS_DVBS2)
    fprintf(out, " rolloff=%s", name_of(rolloff_names, NROLLOFF, t->rolloff));
  if (s->fam == FAM_T)
    fprintf(out, " bw=%s", mhz_str(t->bw / 1000, f, sizeof(f)));
  if (s->delsys == SYS_DVBT2 && t->plp >= 0)
    fprintf(out, " plp=%d", t->plp);
}

static void print_signal(FILE *out, const struct mux_info *m)
{
  if (m->have_cnr)
    fprintf(out, " cnr=%s%lld.%llddB", m->cnr < 0 ? "-" : "", llabs(m->cnr) / 1000,
            llabs(m->cnr) % 1000 / 100);
  if (m->sig_scale == FE_SCALE_DECIBEL)
    fprintf(out, " signal=%s%lld.%llddBm", m->sig < 0 ? "-" : "", llabs(m->sig) / 1000,
            llabs(m->sig) % 1000 / 100);
  else if (m->sig_scale == FE_SCALE_RELATIVE)
    fprintf(out, " signal=%lld%%", m->sig);
}

static void print_card(FILE *out, const struct mux_info *m)
{
  const struct tp *t = &m->tp;
  const struct sysdef *s = &systems[t->sys];
  unsigned mhz = (t->freq + 500) / 1000;

  fprintf(out, "dvb_card %c%d_%u", "cts"[s->fam], o.adapter, mhz);
  if (s->fam == FAM_S)
    fputc(t->pol, out);
  fprintf(out, " {\n  adapter %d;\n  frontend %d;\n  system %s;\n", o.adapter, o.frontend, s->name);
  switch (s->fam) {
  case FAM_C:
    fprintf(out, "  frequency %u;\n  symbol_rate %u;\n  modulation %s;\n", t->freq * 1000u, t->sr / 1000,
            name_of(mod_names, NMOD, t->mod));
    break;
  case FAM_T:
    fprintf(out, "  frequency %u;\n  bandwidth %u;\n", t->freq * 1000u, t->bw);
    if (s->delsys == SYS_DVBT2 && t->plp >= 0)
      fprintf(out, "  plp_stream_id %d;\n", t->plp);
    break;
  case FAM_S: {
    int hi;
    unsigned lof = sat_lof(t->freq, &hi) / 1000;
    int flussonic_lnb = o.lof_lo == 9750 && o.lof_hi == 10600 && o.slof == 11700 &&
                        mhz >= 10700 && mhz <= 12750;
    fprintf(out, "  frequency %u;\n  symbol_rate %u;\n  polarization %c;\n", mhz, t->sr / 1000, t->pol);
    if (s->delsys == SYS_DVBS2 && t->mod != QPSK)
      fprintf(out, "  modulation %s;\n", name_of(mod_names, NMOD, t->mod));
    if (s->delsys == SYS_DVBS2 && (t->rolloff == ROLLOFF_25 || t->rolloff == ROLLOFF_20))
      fprintf(out, "  rolloff %s;\n", name_of(rolloff_names, NROLLOFF, t->rolloff));
    if (!flussonic_lnb) {
      fprintf(out, "  int_freq %u;\n", lof);
      if (hi)
        fprintf(out, "  high_band true;\n");
    }
    break;
  }
  }
  fprintf(out, "}\n");
}

static struct mux_info *new_mux(void)
{
  if (saved_n == saved_cap) {
    int cap = saved_cap ? saved_cap * 2 : 16;
    struct mux_info *n = realloc(saved_mux, (size_t)cap * sizeof(*n));
    if (!n)
      return NULL;
    saved_mux = n;
    saved_cap = cap;
  }
  memset(&saved_mux[saved_n], 0, sizeof(saved_mux[0]));
  return &saved_mux[saved_n];
}

static const struct mux_info *find_dup(const struct tp *t)
{
  int fam = systems[t->sys].fam, i;
  for (i = 0; i < saved_n; i++) {
    const struct tp *s = &saved_mux[i].tp;
    unsigned d;
    if (systems[s->sys].fam != fam)
      continue;
    d = s->freq > t->freq ? s->freq - t->freq : t->freq - s->freq;
    if (fam != FAM_S && d == 0)
      return &saved_mux[i];
    if (fam == FAM_S && s->pol == t->pol && d < s->sr / 2000)
      return &saved_mux[i];
  }
  return NULL;
}

static void collect_mux(int fd, FILE *log, const struct tp *t, int st, unsigned idx, unsigned total)
{
  struct mux_info *m = new_mux();
  char label[48];
  if (!m) {
    fprintf(stderr, "out of memory\n");
    return;
  }
  m->pat_last = -1;
  m->sdt_last = -1;
  m->sdt_other_last = -1;
  m->tp = *t;
  m->status = st;
  if (o.do_psi)
    read_programs(o.psi_ms, m);
  else
    usleep(200000);
  read_stats(fd, m);
  saved_n++;

  tp_label(t, systems[t->sys].fam == FAM_S ? t->pol : 0, label, sizeof(label));
  fprintf(stderr, "[%u/%u] %s LOCK %s", idx, total, label, systems[t->sys].name);
  if (systems[t->sys].fam != FAM_T)
    fprintf(stderr, " sr=%u %s", t->sr / 1000, name_of(mod_names, NMOD, t->mod));
  print_signal(stderr, m);
  fprintf(stderr, "  %d programs, %d named here          \n", m->nprog, named_count(m));

  fprintf(log, "%s  LOCK  ", label);
  print_params(log, t);
  fprintf(log, "  status=0x%x", m->status);
  print_signal(log, m);
  fprintf(log, "  programs=%d named=%d tsid=%d\n", m->nprog, named_count(m), m->have_tsid ? m->tsid : -1);
  fflush(log);
}

/* Returns lock status and fills *t, 0 if nothing locked. */
static int search_point(int fd, const struct point *pt, int wait_ms, struct tp *t)
{
  int si, pi, ri, mi, npol = 1, nsr = 1, nmod = 1;
  const char *pols = "";
  const unsigned *srs = NULL, *mods = NULL;
  unsigned mask = o.sys_mask & fam_mask(pt->fam);

  if (pt->fam == FAM_S && !pt->pol)
    pols = fam_pols(&npol);
  if (pt->fam != FAM_T && !pt->sr)
    srs = fam_srs(pt->fam, &nsr);
  if (pt->fam != FAM_T && pt->mod < 0)
    mods = fam_mods(pt->fam, &nmod);

  for (pi = 0; pi < npol && !stop; pi++)
    for (si = 0; si < NSYS && !stop; si++) {
      const struct sysdef *s = &systems[si];
      if (pt->sys >= 0 ? si != pt->sys : !(mask & (1u << si)))
        continue;
      for (ri = 0; ri < nsr && !stop; ri++)
        for (mi = 0; mi < nmod && !stop; mi++) {
          int st;
          if (s->delsys == SYS_DVBS && mi > 0)
            break;
          memset(t, 0, sizeof(*t));
          t->sys = si;
          t->freq = pt->freq;
          t->pol = pt->pol ? pt->pol : pols[pi];
          t->plp = -1;
          if (s->fam == FAM_T) {
            t->mod = QAM_AUTO;
            t->bw = pt->bw ? pt->bw : o.bw ? o.bw : 8000000;
            if (s->delsys == SYS_DVBT2)
              t->plp = pt->plp != -2 ? pt->plp : o.plp;
          } else {
            t->sr = srs ? srs[ri] : pt->sr;
            t->mod = mods ? mods[mi] : (unsigned)pt->mod;
            if (s->delsys == SYS_DVBS)
              t->mod = QPSK;
            t->rolloff = s->delsys == SYS_DVBS2 ? ROLLOFF_AUTO : ROLLOFF_35;
          }
          if (tune(fd, t) < 0)
            continue;
          st = wait_lock(fd, wait_ms);
          if (st)
            return st;
        }
    }
  return 0;
}

/* Tunes one point. *next is set to the first frequency that may hold another transponder. */
static int scan_point(int fd, FILE *log, const struct point *pt, int wait_ms, unsigned idx, unsigned total,
                      unsigned *next)
{
  struct tp t;
  const struct mux_info *dup;
  char label[48], f[32];
  int st;

  memset(&t, 0, sizeof(t));
  t.freq = pt->freq;
  tp_label(&t, pt->pol, label, sizeof(label));

  if (pt->fam == FAM_S) {
    int hi;
    unsigned ifreq = sat_if(pt->freq, &hi);
    if (ifreq < 950000 || ifreq > 2150000) {
      fprintf(stderr, "[%u/%u] %s outside LNB IF range (IF %s MHz)\n", idx, total, label,
              mhz_str(ifreq, f, sizeof(f)));
      fprintf(log, "%s  outside LNB IF range\n", label);
      return 0;
    }
  }

  fprintf(stderr, "[%u/%u] %s ...\r", idx, total, label);
  fflush(stderr);
  st = search_point(fd, pt, wait_ms, &t);
  if (stop)
    return 0;
  if (!st) {
    fprintf(stderr, "[%u/%u] %s no lock                    \n", idx, total, label);
    fprintf(log, "%s  no lock\n", label);
    fflush(log);
    return 0;
  }
  refine(fd, &t);
  if (next && pt->fam == FAM_S) {
    int step = (int)((unsigned long long)t.sr * 27 / 40000) - (int)o.step;
    *next = t.freq + (step > 0 ? (unsigned)step : 0);
  }
  dup = find_dup(&t);
  if (dup) {
    char other[48];
    tp_label(&dup->tp, pt->fam == FAM_S ? dup->tp.pol : 0, other, sizeof(other));
    fprintf(stderr, "[%u/%u] %s locks %s again, skipped     \n", idx, total, label, other);
    return 0;
  }
  collect_mux(fd, log, &t, st, idx, total);
  return 1;
}

static int scan_grid(int fd, FILE *log)
{
  unsigned from, to, step, total, idx = 0;
  int npol = 1, pi, found = 0;
  const char *pols = "";

  fam_grid(o.fam, &from, &to, &step);
  o.step = step;
  if (o.fam == FAM_S)
    pols = fam_pols(&npol);
  total = ((to - from) / step + 1) * (unsigned)npol;

  for (pi = 0; pi < npol && !stop; pi++) {
    unsigned f, next = 0;
    for (f = from; f <= to && !stop; f += step) {
      struct point pt = {.fam = o.fam, .sys = -1, .freq = f, .pol = o.fam == FAM_S ? pols[pi] : 0,
                         .mod = -1, .plp = -2};
      idx++;
      if (f < next)
        continue;
      found += scan_point(fd, log, &pt, fam_wait(o.fam), idx, total, &next);
    }
  }
  return found;
}

static int parse_point(char *line, struct point *pt)
{
  char *tok, *save = NULL;
  int have_freq = 0;

  memset(pt, 0, sizeof(*pt));
  pt->fam = o.fam;
  pt->sys = -1;
  pt->mod = -1;
  pt->plp = -2;
  if (line[0] == '#')
    return 0;
  for (tok = strtok_r(line, " \t\r\n", &save); tok; tok = strtok_r(NULL, " \t\r\n", &save)) {
    char *v = strchr(tok, '=');
    if (!v)
      continue;
    *v++ = 0;
    if (!strcmp(tok, "system")) {
      int si = sys_by_name(v);
      if (si < 0)
        return -1;
      pt->sys = si;
      pt->fam = systems[si].fam;
    } else if (!strcmp(tok, "freq")) {
      double d = strtod(v, NULL);
      pt->freq = d >= 1e6 ? (unsigned)(d / 1000.0 + 0.5) : (unsigned)(d * 1000.0 + 0.5);
      have_freq = pt->freq > 0;
    } else if (!strcmp(tok, "sr")) {
      double d = strtod(v, NULL);
      pt->sr = d >= 1e5 ? (unsigned)d : (unsigned)(d * 1000.0 + 0.5);
    } else if (!strcmp(tok, "mod") || !strcmp(tok, "qam")) {
      unsigned m;
      if (value_of(mod_names, NMOD, v, &m) == 0)
        pt->mod = (int)m;
    } else if (!strcmp(tok, "pol")) {
      pt->pol = (char)tolower((unsigned char)v[0]);
    } else if (!strcmp(tok, "bw")) {
      pt->bw = parse_bw(v);
    } else if (!strcmp(tok, "plp")) {
      pt->plp = atoi(v);
    }
  }
  if (!have_freq)
    return 0;
  if (pt->fam < 0)
    return -1;
  return 1;
}

static int load_points(const char *path, struct point *out, int max)
{
  FILE *f = fopen(path, "r");
  char line[1024];
  int n = 0, lineno = 0;
  if (!f)
    return -1;
  while (fgets(line, sizeof(line), f) && n < max) {
    int r;
    lineno++;
    r = parse_point(line, &out[n]);
    if (r < 0) {
      fprintf(stderr, "%s:%d: unknown system, pass -s or system=\n", path, lineno);
      continue;
    }
    if (r > 0)
      n++;
  }
  fclose(f);
  return n;
}

static int scan_list(int fd, FILE *log, struct point *pts, int n)
{
  int i, found = 0;
  for (i = 0; i < n && !stop; i++) {
    int w = fam_wait(pts[i].fam);
    found += scan_point(fd, log, &pts[i], w < 800 ? 800 : w, (unsigned)i + 1, (unsigned)n, NULL);
  }
  return found;
}

static void emit_results(FILE *log)
{
  int i, total_prog = 0, total_named = 0;
  for (i = 0; i < saved_n; i++)
    apply_catalog(&saved_mux[i]);
  fprintf(log, "\n");
  for (i = 0; i < saved_n; i++) {
    struct mux_info *m = &saved_mux[i];
    int named = named_count(m);
    total_prog += m->nprog;
    total_named += named;

    fprintf(log, "# ");
    print_params(log, &m->tp);
    fprintf(log, "  programs=%d named=%d tsid=%d\n", m->nprog, named, m->have_tsid ? m->tsid : -1);
    print_card(log, m);
    print_programs(log, m);
    fprintf(log, "\n");

    printf("LOCK ");
    print_params(stdout, &m->tp);
    print_signal(stdout, m);
    printf("  programs=%d named=%d\n", m->nprog, named);
    print_programs(stdout, m);
    print_card(stdout, m);
    printf("\n");
  }
  fprintf(log, "# programs %d, with names %d, sdt entries %d\n", total_prog, total_named, name_db_n);
  printf("# programs %d, with names %d\n", total_prog, total_named);
  fflush(stdout);
  fflush(log);
}

static unsigned frontend_systems(int fd, char *name, size_t nsz)
{
  struct dvb_frontend_info info;
  struct dtv_property p;
  struct dtv_properties c = {.num = 1, .props = &p};
  unsigned mask = 0, two_g;
  unsigned i;
  int k;

  memset(&info, 0, sizeof(info));
  if (ioctl(fd, FE_GET_INFO, &info) == 0)
    snprintf(name, nsz, "%s", info.name);
  else
    snprintf(name, nsz, "?");

  memset(&p, 0, sizeof(p));
  p.cmd = DTV_ENUM_DELSYS;
  if (ioctl(fd, FE_GET_PROPERTY, &c) == 0) {
    for (i = 0; i < p.u.buffer.len && i < sizeof(p.u.buffer.data); i++)
      if ((k = sys_by_delsys(p.u.buffer.data[i])) >= 0)
        mask |= 1u << k;
    if (mask)
      return mask;
  }
  two_g = info.caps & FE_CAN_2G_MODULATION;
  switch (info.type) {
  case FE_QAM:
    mask = 1u << sys_by_delsys(SYS_DVBC_ANNEX_A);
    break;
  case FE_OFDM:
    mask = 1u << sys_by_delsys(SYS_DVBT);
    if (two_g)
      mask |= 1u << sys_by_delsys(SYS_DVBT2);
    break;
  case FE_QPSK:
    mask = 1u << sys_by_delsys(SYS_DVBS);
    if (two_g)
      mask |= 1u << sys_by_delsys(SYS_DVBS2);
    break;
  default:
    break;
  }
  return mask;
}

static void print_mask(FILE *out, unsigned mask)
{
  int i;
  for (i = 0; i < NSYS; i++)
    if (mask & (1u << i))
      fprintf(out, " %s", systems[i].name);
}

static int parse_systems(const char *arg)
{
  char buf[128], *tok, *save = NULL;
  snprintf(buf, sizeof(buf), "%s", arg);
  for (tok = strtok_r(buf, ",", &save); tok; tok = strtok_r(NULL, ",", &save)) {
    unsigned m;
    int fam, si;
    if (!strcasecmp(tok, "c"))
      m = fam_mask(FAM_C);
    else if (!strcasecmp(tok, "t"))
      m = fam_mask(FAM_T);
    else if (!strcasecmp(tok, "s"))
      m = fam_mask(FAM_S);
    else if ((si = sys_by_name(tok)) >= 0)
      m = 1u << si;
    else
      return -1;
    for (fam = 0; fam < FAM_N; fam++)
      if (m & fam_mask(fam))
        break;
    if (o.fam >= 0 && o.fam != fam) {
      fprintf(stderr, "-s: one family per run (dvbc, dvbt/dvbt2 or dvbs/dvbs2)\n");
      return -1;
    }
    o.fam = fam;
    o.sys_mask |= m;
  }
  return o.sys_mask ? 0 : -1;
}

static int parse_uint_list(const char *arg, unsigned *out, int max, unsigned mul)
{
  char buf[256], *tok, *save = NULL;
  int n = 0;
  snprintf(buf, sizeof(buf), "%s", arg);
  for (tok = strtok_r(buf, ",", &save); tok && n < max; tok = strtok_r(NULL, ",", &save))
    out[n++] = (unsigned)(strtod(tok, NULL) * mul + 0.5);
  return n;
}

static int parse_mods(const char *arg)
{
  char buf[256], *tok, *save = NULL;
  snprintf(buf, sizeof(buf), "%s", arg);
  o.nmod = 0;
  for (tok = strtok_r(buf, ",", &save); tok && o.nmod < MAX_LIST; tok = strtok_r(NULL, ",", &save))
    if (value_of(mod_names, NMOD, tok, &o.mods[o.nmod++]) < 0)
      return -1;
  return o.nmod ? 0 : -1;
}

static int parse_pols(const char *arg)
{
  o.npol = 0;
  for (; *arg && o.npol < MAX_LIST; arg++) {
    char c = (char)tolower((unsigned char)*arg);
    if (c == ',')
      continue;
    if (c != 'h' && c != 'v' && c != 'l' && c != 'r')
      return -1;
    o.pols[o.npol++] = c;
  }
  return o.npol ? 0 : -1;
}

static int parse_lnb(const char *arg)
{
  unsigned v[3];
  int n;
  if (!strcasecmp(arg, "universal")) {
    o.lof_lo = 9750;
    o.lof_hi = 10600;
    o.slof = 11700;
    return 0;
  }
  n = parse_uint_list(arg, v, 3, 1);
  if (n == 1) {
    o.lof_lo = v[0];
    o.lof_hi = 0;
    return 0;
  }
  if (n == 3) {
    o.lof_lo = v[0];
    o.lof_hi = v[1];
    o.slof = v[2];
    return 0;
  }
  return -1;
}

static void usage(const char *argv0)
{
  fprintf(stderr,
          "Usage: %s [-a ADAPTER] [-f FRONTEND] [-s SYSTEMS] [options]\n"
          "\n"
          "  -a, --adapter N      /dev/dvb/adapterN (0)\n"
          "  -f, --frontend N     frontendN (0)\n"
          "      --demux N        demuxN and dvrN (same as frontend if present, else 0)\n"
          "  -s, --system LIST    c, t, s, dvbc, dvbt, dvbt2, dvbs, dvbs2, comma separated;\n"
          "                       t = dvbt,dvbt2, s = dvbs,dvbs2 (default: what the frontend supports)\n"
          "      --from MHZ       grid start   (dvbc 114, dvbt 474, dvbs 10700)\n"
          "      --to MHZ         grid end     (dvbc 858, dvbt 858, dvbs 12750)\n"
          "      --step MHZ       grid step    (dvbc 8, dvbt 8, dvbs 6)\n"
          "      --sr LIST        symbol rates, ksym/s (dvbc 6875,6900,6952,6111; dvbs 27500,30000,22000)\n"
          "      --mod LIST       modulations: qam_auto, qam64, qam256, qpsk, psk8, apsk16, apsk32\n"
          "                       (dvbc qam_auto,qam256,qam64; dvbs2 qpsk)\n"
          "      --bandwidth MHZ  DVB-T/T2 channel bandwidth: 1.712, 5, 6, 7, 8, 10 (8)\n"
          "      --plp N          DVB-T2 PLP id (any)\n"
          "      --pol LIST       satellite polarizations: h, v, l, r (h,v)\n"
          "      --lnb TYPE       universal (9750,10600,11700), LOF_MHZ, or LO,HI,SWITCH (universal)\n"
          "      --diseqc N       DiSEqC 1.0 committed port 1..4\n"
          "      --wait MS        lock timeout per try (dvbc 400, dvbt 1200, dvbs 600)\n"
          "      --psi-ms MS      time to collect PAT/SDT on a locked mux (3000)\n"
          "      --no-psi         only lock, do not read programs\n"
          "      --list FILE      tune lines with freq=... from FILE instead of the grid\n"
          "                       (a previous log works; --from-log is an alias)\n"
          "  -o, --output FILE    log file (dvbscan_<family>_aN.log)\n"
          "\n"
          "Stop Flussonic before scanning. Ctrl-C stops the scan and prints what was found.\n",
          argv0);
}

int main(int argc, char **argv)
{
  char path[128], logname[256], fe_name[128], cwd[4096], logpath[4400], range[512];
  static struct point pts[MAX_POINTS];
  unsigned supported;
  int npts = 0, found, fd, i;
  time_t now;
  FILE *log;

  for (i = 1; i < argc; i++) {
    const char *a = argv[i];
    const char *v = i + 1 < argc ? argv[i + 1] : NULL;
    int bad = 0;
    if (!strcmp(a, "-h") || !strcmp(a, "--help")) {
      usage(argv[0]);
      return 0;
    } else if (!strcmp(a, "--no-psi")) {
      o.do_psi = 0;
      continue;
    } else if (!v) {
      bad = 1;
    } else if (!strcmp(a, "-a") || !strcmp(a, "--adapter"))
      o.adapter = atoi(v);
    else if (!strcmp(a, "-f") || !strcmp(a, "--frontend"))
      o.frontend = atoi(v);
    else if (!strcmp(a, "--demux"))
      o.demux = atoi(v);
    else if (!strcmp(a, "-s") || !strcmp(a, "--system"))
      bad = parse_systems(v) < 0;
    else if (!strcmp(a, "--from"))
      o.from = parse_mhz(v);
    else if (!strcmp(a, "--to"))
      o.to = parse_mhz(v);
    else if (!strcmp(a, "--step"))
      bad = (o.step = parse_mhz(v)) == 0;
    else if (!strcmp(a, "--sr"))
      bad = (o.nsr = parse_uint_list(v, o.srs, MAX_LIST, 1000)) == 0;
    else if (!strcmp(a, "--mod"))
      bad = parse_mods(v) < 0;
    else if (!strcmp(a, "--bandwidth"))
      bad = (o.bw = parse_bw(v)) == 0;
    else if (!strcmp(a, "--plp"))
      o.plp = atoi(v);
    else if (!strcmp(a, "--pol"))
      bad = parse_pols(v) < 0;
    else if (!strcmp(a, "--lnb"))
      bad = parse_lnb(v) < 0;
    else if (!strcmp(a, "--diseqc"))
      bad = (o.diseqc = atoi(v)) < 1 || o.diseqc > 4;
    else if (!strcmp(a, "--wait"))
      o.wait_ms = atoi(v);
    else if (!strcmp(a, "--psi-ms"))
      o.psi_ms = atoi(v);
    else if (!strcmp(a, "--list") || !strcmp(a, "--from-log"))
      o.list = v;
    else if (!strcmp(a, "-o") || !strcmp(a, "--output"))
      o.out = v;
    else
      bad = 1;
    if (bad) {
      fprintf(stderr, "bad option %s %s\n\n", a, v ? v : "");
      usage(argv[0]);
      return 1;
    }
    i++;
  }

  snprintf(path, sizeof(path), "/dev/dvb/adapter%d/frontend%d", o.adapter, o.frontend);
  fd = open(path, O_RDWR);
  if (fd < 0) {
    fprintf(stderr, "cannot open %s: %s\n", path, strerror(errno));
    fprintf(stderr, "Stop Flussonic or pick another -a adapter.\n");
    return 1;
  }

  if (o.demux < 0) {
    char dmx[128];
    snprintf(dmx, sizeof(dmx), "/dev/dvb/adapter%d/demux%d", o.adapter, o.frontend);
    o.demux = access(dmx, F_OK) == 0 ? o.frontend : 0;
  }

  supported = frontend_systems(fd, fe_name, sizeof(fe_name));
  fprintf(stderr, "%s: %s, systems:", path, fe_name);
  print_mask(stderr, supported);
  fprintf(stderr, "\n");

  if (!o.sys_mask) {
    int fam, nfam = 0;
    for (fam = 0; fam < FAM_N; fam++)
      if (supported & fam_mask(fam)) {
        o.fam = fam;
        nfam++;
      }
    if (nfam > 1 && !o.list) {
      fprintf(stderr, "frontend supports several families, pick one with -s\n");
      close(fd);
      return 1;
    }
    if (nfam == 0 && !o.list) {
      fprintf(stderr, "frontend reports no DVB-C/T/S systems, pass -s\n");
      close(fd);
      return 1;
    }
    if (nfam > 1)
      o.fam = -1;
    o.sys_mask = supported;
  } else if ((o.sys_mask & supported) != o.sys_mask) {
    fprintf(stderr, "warning: frontend does not report");
    print_mask(stderr, o.sys_mask & ~supported);
    fprintf(stderr, ", trying anyway\n");
  }

  if (o.list) {
    npts = load_points(o.list, pts, MAX_POINTS);
    if (npts < 0) {
      fprintf(stderr, "cannot read %s: %s\n", o.list, strerror(errno));
      close(fd);
      return 1;
    }
    if (npts == 0) {
      fprintf(stderr, "no freq= lines in %s\n", o.list);
      close(fd);
      return 1;
    }
    for (i = 0; i < npts; i++)
      if (pts[i].sys < 0 && !(o.sys_mask & fam_mask(pts[i].fam)))
        o.sys_mask |= fam_mask(pts[i].fam);
    snprintf(range, sizeof(range), "list %s, %d entries", o.list, npts);
  } else {
    unsigned from, to, step;
    char a[32], b[32], c[32];
    fam_grid(o.fam, &from, &to, &step);
    if (from > to) {
      fprintf(stderr, "bad frequency range\n");
      close(fd);
      return 1;
    }
    snprintf(range, sizeof(range), "grid %s-%s MHz step %s", mhz_str(from, a, sizeof(a)),
             mhz_str(to, b, sizeof(b)), mhz_str(step, c, sizeof(c)));
  }

  if (o.out) {
    snprintf(logname, sizeof(logname), "%s", o.out);
  } else {
    char fe[16] = "";
    if (o.frontend)
      snprintf(fe, sizeof(fe), "f%d", o.frontend);
    snprintf(logname, sizeof(logname), "dvbscan_%s_a%d%s%s.log", o.fam >= 0 ? fam_name[o.fam] : "mixed",
             o.adapter, fe, o.list ? "_rescan" : "");
  }
  if (o.list && !strcmp(o.list, logname)) {
    fprintf(stderr, "--list and the output log are the same file, pass -o\n");
    close(fd);
    return 1;
  }
  log = fopen(logname, "w");
  if (!log) {
    fprintf(stderr, "cannot write %s: %s\n", logname, strerror(errno));
    close(fd);
    return 1;
  }
  if (logname[0] == '/' || !getcwd(cwd, sizeof(cwd)))
    snprintf(logpath, sizeof(logpath), "%s", logname);
  else
    snprintf(logpath, sizeof(logpath), "%s/%s", cwd, logname);
  fprintf(stderr, "log: %s\n", logpath);

  signal(SIGINT, on_sigint);

  now = time(NULL);
  fprintf(log, "# dvbscan %s (%s)\n# systems", path, fe_name);
  print_mask(log, o.sys_mask);
  fprintf(log, "\n# started %s# %s, psi %s (%d ms)\n", ctime(&now), range, o.do_psi ? "on" : "off", o.psi_ms);
  if (o.fam == FAM_S || o.list) {
    if (o.lof_hi)
      fprintf(log, "# lnb %u/%u switch %u", o.lof_lo, o.lof_hi, o.slof);
    else
      fprintf(log, "# lnb %u", o.lof_lo);
    if (o.diseqc)
      fprintf(log, ", diseqc port %d (dvb_card blocks below do not carry it)", o.diseqc);
    fprintf(log, "\n");
  }
  fprintf(log, "\n");
  fflush(log);

  fprintf(stderr, "scan%s, %s\n", o.fam >= 0 ? "" : " mixed list", range);
  found = o.list ? scan_list(fd, log, pts, npts) : scan_grid(fd, log);

  close(fd);
  if (saved_n)
    emit_results(log);
  now = time(NULL);
  fprintf(log, "\n# %s %s# found %d transponder(s)\n", stop ? "interrupted" : "finished", ctime(&now), found);
  fclose(log);

  if (stop)
    fprintf(stderr, "\ninterrupted\n");
  if (!found) {
    fprintf(stderr, "no lock. Results: %s\n", logpath);
    return 2;
  }
  fprintf(stderr, "found %d transponder(s). Results: %s\n", found, logpath);
  return 0;
}
