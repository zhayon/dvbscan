/* LD_PRELOAD stand-in for /dev/dvb/adapter0: a frontend that locks on a fixed set of
 * transponders and a dvr that plays PAT/SDT of the locked one.
 *
 *   FAKE_DVB=c|t|s LD_PRELOAD=./fakedvb.so ./dvbscan ...
 */

#define _GNU_SOURCE
#include <dlfcn.h>
#include <fcntl.h>
#include <linux/dvb/dmx.h>
#include <linux/dvb/frontend.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

struct signal {
  unsigned delsys;
  unsigned freq;   /* C/T: Hz, S: transponder kHz */
  unsigned sr;
  unsigned mod;
  unsigned bw;
  char pol;
  unsigned rolloff;
  int tsid;
  const char *name;
};

static const struct signal signals[] = {
    {SYS_DVBC_ANNEX_A, 306000000, 6875000, QAM_256, 0, 0, 0, 101, "Cable One"},
    {SYS_DVBC_ANNEX_A, 314000000, 6900000, QAM_64, 0, 0, 0, 102, "Cable Two"},
    {SYS_DVBT, 602000000, 0, QAM_64, 8000000, 0, 0, 201, "Terr One"},
    {SYS_DVBT2, 650000000, 0, QAM_256, 8000000, 0, 0, 202, "Terr Two HD"},
    {SYS_DVBS2, 11766000, 27500000, PSK_8, 0, 'h', ROLLOFF_20, 301, "Sat One HD"},
    {SYS_DVBS, 12226000, 27500000, QPSK, 0, 'v', ROLLOFF_35, 302, "Sat Two"},
};
#define NSIG ((int)(sizeof(signals) / sizeof(signals[0])))

static int fe_fd = -1, dmx_fd = -1;
static char fam;
static unsigned t_delsys, t_freq, t_sr, t_mod, t_bw;
static int voltage = -1, tone = -1;
static const struct signal *locked;

static int (*real_open)(const char *, int, ...);
static int (*real_ioctl)(int, unsigned long, ...);

static void init(void)
{
  const char *f = getenv("FAKE_DVB");
  if (real_open)
    return;
  real_open = dlsym(RTLD_NEXT, "open");
  real_ioctl = dlsym(RTLD_NEXT, "ioctl");
  fam = f ? f[0] : 't';
}

static int is_sat(unsigned d) { return d == SYS_DVBS || d == SYS_DVBS2; }

static unsigned lof(unsigned khz) { return khz >= 11700000 ? 10600000 : 9750000; }

static void evaluate(void)
{
  int i;
  locked = NULL;
  for (i = 0; i < NSIG; i++) {
    const struct signal *s = &signals[i];
    if (is_sat(s->delsys)) {
      unsigned real_if = s->freq - lof(s->freq);
      unsigned d = t_freq > real_if ? t_freq - real_if : real_if - t_freq;
      int want_v = (s->pol == 'h') ? SEC_VOLTAGE_18 : SEC_VOLTAGE_13;
      int want_t = s->freq >= 11700000 ? SEC_TONE_ON : SEC_TONE_OFF;
      if (!is_sat(t_delsys) || d > 5000 || t_sr != s->sr || voltage != want_v || tone != want_t)
        continue;
      /* DVB-S2 tuning locks DVB-S too, DVB-S tuning does not lock DVB-S2 */
      if (s->delsys == SYS_DVBS2 && t_delsys != SYS_DVBS2)
        continue;
    } else {
      if (t_delsys != s->delsys || t_freq != s->freq)
        continue;
      if (s->delsys == SYS_DVBC_ANNEX_A && (t_sr != s->sr || (t_mod != QAM_AUTO && t_mod != s->mod)))
        continue;
      if (s->delsys != SYS_DVBC_ANNEX_A && t_bw != s->bw)
        continue;
    }
    locked = s;
    return;
  }
}

static void put_section(uint8_t *out, int *pos, int pid, const uint8_t *sec, int len)
{
  uint8_t *p = out + *pos;
  memset(p, 0xFF, 188);
  p[0] = 0x47;
  p[1] = (uint8_t)(0x40 | (pid >> 8));
  p[2] = (uint8_t)pid;
  p[3] = 0x10;
  p[4] = 0;
  memcpy(p + 5, sec, (size_t)len);
  *pos += 188;
}

static int open_dvr(int flags)
{
  uint8_t ts[188 * 6], sec[184];
  int p[2], pos = 0, n, k, nlen, slen;
  const struct signal *s = locked;
  if (pipe(p) < 0)
    return -1;
  if (s) {
    for (k = 0; k < 3; k++) {
      n = 0;
      sec[n++] = 0x00;
      sec[n++] = 0xB0;
      sec[n++] = 13;
      sec[n++] = (uint8_t)(s->tsid >> 8);
      sec[n++] = (uint8_t)s->tsid;
      sec[n++] = 0xC1;
      sec[n++] = 0;
      sec[n++] = 0;
      sec[n++] = 0;
      sec[n++] = 1;
      sec[n++] = 0xE1;
      sec[n++] = 0x00;
      n += 4;
      put_section(ts, &pos, 0, sec, n);

      nlen = (int)strlen(s->name);
      slen = 8 + 5 + 2 + 3 + 4 + nlen + 4;
      n = 0;
      sec[n++] = 0x42;
      sec[n++] = (uint8_t)(0xF0 | (slen >> 8));
      sec[n++] = (uint8_t)slen;
      sec[n++] = (uint8_t)(s->tsid >> 8);
      sec[n++] = (uint8_t)s->tsid;
      sec[n++] = 0xC1;
      sec[n++] = 0;
      sec[n++] = 0;
      sec[n++] = 0x00;
      sec[n++] = 0x01;
      sec[n++] = 0xFF;
      sec[n++] = 0x00;
      sec[n++] = 0x01;
      sec[n++] = 0xFC;
      sec[n++] = (uint8_t)(0x80 | ((2 + 3 + 4 + nlen) >> 8));
      sec[n++] = (uint8_t)(2 + 3 + 4 + nlen);
      sec[n++] = 0x48;
      sec[n++] = (uint8_t)(3 + 4 + nlen);
      sec[n++] = 0x01;
      sec[n++] = 4;
      memcpy(sec + n, "Fake", 4);
      n += 4;
      sec[n++] = (uint8_t)nlen;
      memcpy(sec + n, s->name, (size_t)nlen);
      n += nlen;
      n += 4;
      put_section(ts, &pos, 0x11, sec, n);
    }
    if (write(p[1], ts, (size_t)pos) != pos)
      return -1;
  }
  close(p[1]);
  if (flags & O_NONBLOCK)
    fcntl(p[0], F_SETFL, O_NONBLOCK);
  return p[0];
}

int open(const char *path, int flags, ...)
{
  va_list ap;
  int mode;
  init();
  va_start(ap, flags);
  mode = va_arg(ap, int);
  va_end(ap);
  if (!strcmp(path, "/dev/dvb/adapter0/frontend0"))
    return fe_fd = real_open("/dev/null", O_RDWR);
  if (!strcmp(path, "/dev/dvb/adapter0/demux0"))
    return dmx_fd = real_open("/dev/null", O_RDWR);
  if (!strcmp(path, "/dev/dvb/adapter0/dvr0"))
    return open_dvr(flags);
  return real_open(path, flags, mode);
}

int open64(const char *path, int flags, ...)
{
  va_list ap;
  int mode;
  va_start(ap, flags);
  mode = va_arg(ap, int);
  va_end(ap);
  return open(path, flags, mode);
}

static void get_prop(struct dtv_property *p)
{
  const struct signal *s = locked;
  switch (p->cmd) {
  case DTV_ENUM_DELSYS:
    if (fam == 'c') {
      p->u.buffer.data[0] = SYS_DVBC_ANNEX_A;
      p->u.buffer.len = 1;
    } else if (fam == 's') {
      p->u.buffer.data[0] = SYS_DVBS;
      p->u.buffer.data[1] = SYS_DVBS2;
      p->u.buffer.len = 2;
    } else {
      p->u.buffer.data[0] = SYS_DVBT;
      p->u.buffer.data[1] = SYS_DVBT2;
      p->u.buffer.len = 2;
    }
    break;
  case DTV_DELIVERY_SYSTEM:
    p->u.data = s ? s->delsys : t_delsys;
    break;
  case DTV_FREQUENCY:
    p->u.data = s && is_sat(s->delsys) ? s->freq - lof(s->freq) : t_freq;
    break;
  case DTV_MODULATION:
    p->u.data = s ? s->mod : t_mod;
    break;
  case DTV_ROLLOFF:
    p->u.data = s ? s->rolloff : ROLLOFF_AUTO;
    break;
  case DTV_STAT_CNR:
    p->u.st.len = 1;
    p->u.st.stat[0].scale = FE_SCALE_DECIBEL;
    p->u.st.stat[0].svalue = 12300;
    break;
  case DTV_STAT_SIGNAL_STRENGTH:
    p->u.st.len = 1;
    p->u.st.stat[0].scale = FE_SCALE_DECIBEL;
    p->u.st.stat[0].svalue = -45600;
    break;
  }
}

static void set_prop(const struct dtv_property *p)
{
  switch (p->cmd) {
  case DTV_CLEAR:
    t_freq = t_sr = t_bw = 0;
    t_mod = QAM_AUTO;
    locked = NULL;
    break;
  case DTV_DELIVERY_SYSTEM: t_delsys = p->u.data; break;
  case DTV_FREQUENCY: t_freq = p->u.data; break;
  case DTV_SYMBOL_RATE: t_sr = p->u.data; break;
  case DTV_MODULATION: t_mod = p->u.data; break;
  case DTV_BANDWIDTH_HZ: t_bw = p->u.data; break;
  case DTV_TUNE: evaluate(); break;
  }
}

int ioctl(int fd, unsigned long req, ...)
{
  va_list ap;
  void *arg;
  unsigned i;
  init();
  va_start(ap, req);
  arg = va_arg(ap, void *);
  va_end(ap);
  if (fd == dmx_fd && fd >= 0)
    return 0;
  if (fd != fe_fd || fd < 0)
    return real_ioctl(fd, req, arg);

  if (req == FE_GET_INFO) {
    struct dvb_frontend_info *info = arg;
    memset(info, 0, sizeof(*info));
    snprintf(info->name, sizeof(info->name), "Fake DVB-%c", fam - 32);
    return 0;
  }
  if (req == FE_SET_PROPERTY || req == FE_GET_PROPERTY) {
    struct dtv_properties *c = arg;
    for (i = 0; i < c->num; i++) {
      if (req == FE_SET_PROPERTY)
        set_prop(&c->props[i]);
      else
        get_prop(&c->props[i]);
    }
    return 0;
  }
  if (req == FE_READ_STATUS) {
    *(fe_status_t *)arg = locked ? (FE_HAS_SIGNAL | FE_HAS_CARRIER | FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK) : 0;
    return 0;
  }
  if (req == FE_SET_VOLTAGE) {
    voltage = (int)(uintptr_t)arg;
    return 0;
  }
  if (req == FE_SET_TONE) {
    tone = (int)(uintptr_t)arg;
    return 0;
  }
  if (req == FE_DISEQC_SEND_MASTER_CMD || req == FE_DISEQC_SEND_BURST)
    return 0;
  return 0;
}
