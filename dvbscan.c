/* DVB-C blind scan: lock transponders, then list programs from PAT/SDT.
 *
 *   gcc -O2 -o dvbc_scan dvbc_scan.c
 *   ./dvbc_scan -a 0
 *
 * Stop Flussonic first so /dev/dvb/adapterN/frontend0 is free.
 * Writes dvbc_scan_aN.log in the current directory.
 */

#include <errno.h>
#include <fcntl.h>
#include <linux/dvb/dmx.h>
#include <linux/dvb/frontend.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#define MAX_PROG 256
#define PSI_MAX 4096
#define TS_PKT 188

static unsigned srs[] = {6875000, 6900000, 6952000, 6111000};
static unsigned qams[] = {QAM_AUTO, QAM_256, QAM_64};
static const char *qam_name[] = {"qam_auto", "qam256", "qam64"};

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
  unsigned mhz, freq, sr;
  const char *qam;
  int status;
  int adapter;
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

#define MAX_MUX 64
static struct mux_info saved_mux[MAX_MUX];
static int saved_n;

static int tune(int fd, unsigned freq, unsigned sr, unsigned qam)
{
  struct dtv_property clr = {.cmd = DTV_CLEAR};
  struct dtv_properties c1 = {.num = 1, .props = &clr};
  if (ioctl(fd, FE_SET_PROPERTY, &c1) < 0)
    return -1;

  struct dtv_property p[] = {
      {.cmd = DTV_DELIVERY_SYSTEM, .u.data = SYS_DVBC_ANNEX_A},
      {.cmd = DTV_FREQUENCY, .u.data = freq},
      {.cmd = DTV_MODULATION, .u.data = qam},
      {.cmd = DTV_INVERSION, .u.data = INVERSION_AUTO},
      {.cmd = DTV_SYMBOL_RATE, .u.data = sr},
      {.cmd = DTV_INNER_FEC, .u.data = FEC_AUTO},
      {.cmd = DTV_TUNE},
  };
  struct dtv_properties c2 = {.num = 7, .props = p};
  return ioctl(fd, FE_SET_PROPERTY, &c2);
}

struct candidate {
  unsigned freq;
  unsigned mhz;
  unsigned sr;
  unsigned qam;
  const char *qam_str;
};

static unsigned qam_from_name(const char *s)
{
  if (!strcmp(s, "qam256"))
    return QAM_256;
  if (!strcmp(s, "qam64"))
    return QAM_64;
  return QAM_AUTO;
}

static int load_locks(const char *path, struct candidate *out, int max)
{
  FILE *f = fopen(path, "r");
  char line[512];
  int n = 0;
  if (!f)
    return -1;
  while (fgets(line, sizeof(line), f) && n < max) {
    unsigned mhz = 0, freq = 0, sr = 0;
    char qam[32];
    if (sscanf(line, "%u MHz  LOCK  freq=%u Hz  sr=%u  qam=%31s",
               &mhz, &freq, &sr, qam) != 4)
      continue;
    out[n].mhz = mhz;
    out[n].freq = freq;
    out[n].sr = sr;
    out[n].qam = qam_from_name(qam);
    out[n].qam_str = qam_name[out[n].qam == QAM_256 ? 1 : out[n].qam == QAM_64 ? 2 : 0];
    n++;
  }
  fclose(f);
  return n;
}

static int wait_lock(int fd, int wait_ms)
{
  int slept = 0;
  while (slept < wait_ms) {
    fe_status_t st = 0;
    if (ioctl(fd, FE_READ_STATUS, &st) >= 0 && (st & FE_HAS_LOCK))
      return (int)st;
    usleep(50000);
    slept += 50;
  }
  return 0;
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

static int start_pid_filter(int adapter, uint16_t pid)
{
  char dmx_path[128];
  struct dmx_pes_filter_params pes;
  int dmx;
  snprintf(dmx_path, sizeof(dmx_path), "/dev/dvb/adapter%d/demux0", adapter);
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

static int read_programs(int adapter, int psi_ms, struct mux_info *m)
{
  char dvr_path[128];
  uint8_t buf[32 * TS_PKT];
  struct pollfd pfd;
  int dmx_pat, dmx_sdt, dvr = -1;
  int elapsed = 0;

  memset(m, 0, sizeof(*m));
  m->pat_last = -1;
  m->sdt_last = -1;

  dmx_pat = start_pid_filter(adapter, 0);
  dmx_sdt = start_pid_filter(adapter, 0x11);
  if (dmx_pat < 0 && dmx_sdt < 0)
    return -1;

  snprintf(dvr_path, sizeof(dvr_path), "/dev/dvb/adapter%d/dvr0", adapter);
  dvr = open(dvr_path, O_RDONLY | O_NONBLOCK);
  if (dvr < 0) {
    if (dmx_pat >= 0) {
      ioctl(dmx_pat, DMX_STOP);
      close(dmx_pat);
    }
    if (dmx_sdt >= 0) {
      ioctl(dmx_sdt, DMX_STOP);
      close(dmx_sdt);
    }
    return -1;
  }

  pfd.fd = dvr;
  pfd.events = POLLIN;
  while (elapsed < psi_ms) {
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
  if (dmx_pat >= 0) {
    ioctl(dmx_pat, DMX_STOP);
    close(dmx_pat);
  }
  if (dmx_sdt >= 0) {
    ioctl(dmx_sdt, DMX_STOP);
    close(dmx_sdt);
  }
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

static void collect_mux(int adapter, unsigned idx, unsigned nfreq,
                        unsigned mhz, unsigned freq, unsigned sr, const char *qam,
                        int st, int do_psi, int psi_ms)
{
  struct mux_info *m;
  int named;
  if (saved_n >= MAX_MUX) {
    fprintf(stderr, "too many muxes\n");
    return;
  }
  m = &saved_mux[saved_n];
  memset(m, 0, sizeof(*m));
  m->pat_last = -1;
  m->sdt_last = -1;
  m->sdt_other_last = -1;
  if (do_psi)
    read_programs(adapter, psi_ms, m);
  m->adapter = adapter;
  m->mhz = mhz;
  m->freq = freq;
  m->sr = sr;
  m->qam = qam;
  m->status = st;
  saved_n++;
  named = named_count(m);
  fprintf(stderr, "[%u/%u] %u MHz LOCK sr=%u qam=%s  %d programs, %d named here          \n",
          idx, nfreq, mhz, sr, qam, m->nprog, named);
}

static void emit_results(FILE *log)
{
  int i, total_prog = 0, total_named = 0;
  for (i = 0; i < saved_n; i++)
    apply_catalog(&saved_mux[i]);
  for (i = 0; i < saved_n; i++) {
    struct mux_info *m = &saved_mux[i];
    int named = named_count(m);
    total_prog += m->nprog;
    total_named += named;
    fprintf(log,
            "%u MHz  LOCK  freq=%u Hz  sr=%u  qam=%s  status=0x%x  programs=%d named=%d tsid=%d\n"
            "dvb_card c%d_%u {\n"
            "  adapter %d;\n"
            "  frontend 0;\n"
            "  system dvbca;\n"
            "  frequency %u;\n"
            "  symbol_rate %u;\n"
            "  modulation %s;\n"
            "}\n",
            m->mhz, m->freq, m->sr, m->qam, m->status, m->nprog, named, m->have_tsid ? m->tsid : -1,
            m->adapter, m->mhz, m->adapter, m->freq, m->sr / 1000, m->qam);
    print_programs(log, m);
    fprintf(log, "\n");
    printf("LOCK freq=%u Hz  sr=%u  qam=%s  programs=%d named=%d\n",
           m->freq, m->sr, m->qam, m->nprog, named);
    print_programs(stdout, m);
    printf("dvb_card c%d_%u {\n"
           "  adapter %d;\n"
           "  frontend 0;\n"
           "  system dvbca;\n"
           "  frequency %u;\n"
           "  symbol_rate %u;\n"
           "  modulation %s;\n"
           "}\n\n",
           m->adapter, m->mhz, m->adapter, m->freq, m->sr / 1000, m->qam);
  }
  fprintf(log, "# programs %d, with names %d, sdt entries %d\n",
          total_prog, total_named, name_db_n);
  printf("# programs %d, with names %d\n", total_prog, total_named);
  fflush(stdout);
  fflush(log);
}

static void usage(const char *argv0)
{
  fprintf(stderr,
          "Usage: %s [-a ADAPTER] [--from MHZ] [--to MHZ] [--step MHZ] [--wait MS]\n"
          "          [--psi-ms MS] [--no-psi] [--from-log FILE]\n"
          "Writes dvbc_scan_aN.log (or dvbc_scan_aN_programs.log with --from-log).\n"
          "Stop Flussonic before scanning.\n",
          argv0);
}

int main(int argc, char **argv)
{
  int adapter = 0, do_psi = 1, psi_ms = 3000;
  unsigned from_mhz = 114, to_mhz = 858, step_mhz = 8;
  int wait_ms = 400;
  const char *from_log = NULL;
  int i;

  for (i = 1; i < argc; i++) {
    if ((!strcmp(argv[i], "-a") || !strcmp(argv[i], "--adapter")) && i + 1 < argc)
      adapter = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--from") && i + 1 < argc)
      from_mhz = (unsigned)atoi(argv[++i]);
    else if (!strcmp(argv[i], "--to") && i + 1 < argc)
      to_mhz = (unsigned)atoi(argv[++i]);
    else if (!strcmp(argv[i], "--step") && i + 1 < argc)
      step_mhz = (unsigned)atoi(argv[++i]);
    else if (!strcmp(argv[i], "--wait") && i + 1 < argc)
      wait_ms = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--psi-ms") && i + 1 < argc)
      psi_ms = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--from-log") && i + 1 < argc)
      from_log = argv[++i];
    else if (!strcmp(argv[i], "--no-psi"))
      do_psi = 0;
    else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
      usage(argv[0]);
      return 0;
    } else {
      usage(argv[0]);
      return 1;
    }
  }

  if (!from_log && (step_mhz == 0 || from_mhz > to_mhz)) {
    fprintf(stderr, "bad frequency range\n");
    return 1;
  }

  struct candidate locks[128];
  int nlocks = 0;
  if (from_log) {
    nlocks = load_locks(from_log, locks, 128);
    if (nlocks < 0) {
      fprintf(stderr, "cannot read %s: %s\n", from_log, strerror(errno));
      return 1;
    }
    if (nlocks == 0) {
      fprintf(stderr, "no LOCK lines in %s\n", from_log);
      return 1;
    }
  }

  char path[128];
  snprintf(path, sizeof(path), "/dev/dvb/adapter%d/frontend0", adapter);
  int fd = open(path, O_RDWR);
  if (fd < 0) {
    fprintf(stderr, "cannot open %s: %s\n", path, strerror(errno));
    fprintf(stderr, "Stop Flussonic or pick another -a adapter.\n");
    return 1;
  }

  char logname[64];
  if (from_log)
    snprintf(logname, sizeof(logname), "dvbc_scan_a%d_programs.log", adapter);
  else
    snprintf(logname, sizeof(logname), "dvbc_scan_a%d.log", adapter);
  FILE *log = fopen(logname, "w");
  if (!log) {
    fprintf(stderr, "cannot write %s: %s\n", logname, strerror(errno));
    close(fd);
    return 1;
  }

  char cwd[4096];
  if (!getcwd(cwd, sizeof(cwd)))
    strcpy(cwd, ".");
  fprintf(stderr, "log: %s/%s\n", cwd, logname);

  time_t now = time(NULL);
  unsigned nfreq = from_log ? (unsigned)nlocks : (to_mhz - from_mhz) / step_mhz + 1;
  fprintf(log,
          "# DVB-C scan %s\n"
          "# started %s"
          "# %s, wait %d ms, psi %s (%d ms)\n\n",
          path, ctime(&now),
          from_log ? from_log : "full grid", wait_ms,
          do_psi ? "on" : "off", psi_ms);
  fflush(log);

  fprintf(stderr, "DVB-C scan %s  %u muxes\n", path, nfreq);

  int found = 0;
  unsigned idx = 0;
  if (from_log) {
    for (i = 0; i < nlocks; i++) {
      idx++;
      fprintf(stderr, "[%u/%u] %u MHz ...\r", idx, nfreq, locks[i].mhz);
      fflush(stderr);
      if (tune(fd, locks[i].freq, locks[i].sr, locks[i].qam) < 0) {
        fprintf(stderr, "[%u/%u] %u MHz tune failed\n", idx, nfreq, locks[i].mhz);
        fprintf(log, "%u MHz  tune failed\n", locks[i].mhz);
        continue;
      }
      int st = wait_lock(fd, wait_ms < 800 ? 800 : wait_ms);
      if (!st) {
        fprintf(stderr, "[%u/%u] %u MHz no lock                    \n", idx, nfreq, locks[i].mhz);
        fprintf(log, "%u MHz  no lock\n", locks[i].mhz);
        fflush(log);
        continue;
      }
      found++;
      collect_mux(adapter, idx, nfreq, locks[i].mhz, locks[i].freq,
                  locks[i].sr, locks[i].qam_str, st, do_psi, psi_ms);
    }
  } else {
    for (unsigned mhz = from_mhz; mhz <= to_mhz; mhz += step_mhz) {
      unsigned freq = mhz * 1000000u;
      int locked = 0;
      unsigned lock_sr = 0;
      const char *lock_qam = NULL;
      int lock_st = 0;
      idx++;
      fprintf(stderr, "[%u/%u] %u MHz ...\r", idx, nfreq, mhz);
      fflush(stderr);
      for (unsigned qi = 0; qi < 3 && !locked; qi++) {
        for (unsigned si = 0; si < 4 && !locked; si++) {
          if (tune(fd, freq, srs[si], qams[qi]) < 0)
            continue;
          int st = wait_lock(fd, wait_ms);
          if (!st)
            continue;
          locked = 1;
          lock_sr = srs[si];
          lock_qam = qam_name[qi];
          lock_st = st;
          found++;
        }
      }
      if (locked) {
        collect_mux(adapter, idx, nfreq, mhz, freq, lock_sr, lock_qam,
                    lock_st, do_psi, psi_ms);
      } else {
        fprintf(stderr, "[%u/%u] %u MHz no lock                    \n", idx, nfreq, mhz);
        fprintf(log, "%u MHz  no lock\n", mhz);
        fflush(log);
      }
    }
  }

  close(fd);
  if (saved_n)
    emit_results(log);
  now = time(NULL);
  fprintf(log, "\n# finished %s# found %d transponder(s)\n", ctime(&now), found);
  fclose(log);

  if (!found) {
    fprintf(stderr, "no lock. Results: %s/%s\n", cwd, logname);
    return 2;
  }
  fprintf(stderr, "found %d transponder(s). Results: %s/%s\n", found, cwd, logname);
  return 0;
}
