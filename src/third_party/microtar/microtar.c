/*
 * Copyright (c) 2017 rxi
 * MIT License — see microtar.h for full license text.
 * Source: https://github.com/rxi/microtar (vendored for TopBand BMS Gateway)
 */

#include "microtar.h"

typedef struct {
  char name[100];
  char mode[8];
  char owner[8];
  char group[8];
  char size[12];
  char mtime[12];
  char checksum[8];
  char type;
  char linkname[100];
  char _padding[255];
} mtar_raw_header_t;

typedef struct {
  const unsigned char *data;
  size_t               size;
  size_t               pos;
} mtar_mem_stream_t;

static unsigned round_up(unsigned n, unsigned incr) {
  return n + (incr - n % incr) % incr;
}

static unsigned checksum(const mtar_raw_header_t* rh) {
  unsigned i;
  unsigned char *p = (unsigned char*) rh;
  unsigned res = 256;
  for (i = 0; i < offsetof(mtar_raw_header_t, checksum); i++) res += p[i];
  for (i = offsetof(mtar_raw_header_t, checksum) + 8;
       i < sizeof(*rh); i++) res += p[i];
  return res;
}

static int tread(mtar_t *tar, void *data, unsigned size) {
  int err = tar->read(tar, data, size);
  tar->pos += size;
  return err;
}

static int twrite(mtar_t *tar, const void *data, unsigned size) {
  int err = tar->write(tar, data, size);
  tar->pos += size;
  return err;
}

static int write_null_bytes(mtar_t *tar, int n) {
  int i, err;
  char nul = '\0';
  for (i = 0; i < n; i++) {
    err = twrite(tar, &nul, 1);
    if (err) return err;
  }
  return MTAR_ESUCCESS;
}

static int raw_to_header(mtar_header_t *h, const mtar_raw_header_t *rh) {
  unsigned chksum1, chksum2;
  if (*rh->checksum == '\0') return MTAR_ENULLRECORD;
  chksum1 = checksum(rh);
  sscanf(rh->checksum, "%o", &chksum2);
  if (chksum1 != chksum2) return MTAR_EBADCHKSUM;
  sscanf(rh->mode,  "%o", &h->mode);
  sscanf(rh->owner, "%o", &h->owner);
  sscanf(rh->size,  "%o", &h->size);
  sscanf(rh->mtime, "%o", &h->mtime);
  h->type = rh->type ? rh->type : MTAR_TREG;
  memcpy(h->name,     rh->name,     sizeof(h->name));
  memcpy(h->linkname, rh->linkname, sizeof(h->linkname));
  return MTAR_ESUCCESS;
}

static int header_to_raw(mtar_raw_header_t *rh, const mtar_header_t *h) {
  unsigned chksum;
  memset(rh, 0, sizeof(*rh));
  sprintf(rh->mode,  "%o",  h->mode);
  sprintf(rh->owner, "%o",  h->owner);
  sprintf(rh->size,  "%011o", h->size);
  sprintf(rh->mtime, "%011o", h->mtime);
  rh->type = h->type;
  memcpy(rh->name,     h->name,     sizeof(rh->name));
  memcpy(rh->linkname, h->linkname, sizeof(rh->linkname));
  memset(rh->checksum, ' ', sizeof(rh->checksum));
  chksum = checksum(rh);
  sprintf(rh->checksum, "%06o", chksum);
  rh->checksum[6] = '\0';
  rh->checksum[7] = ' ';
  return MTAR_ESUCCESS;
}

const char* mtar_strerror(int err) {
  switch (err) {
    case MTAR_ESUCCESS   : return "success";
    case MTAR_EFAILURE   : return "failure";
    case MTAR_EOPENFAIL  : return "could not open";
    case MTAR_EREADFAIL  : return "could not read";
    case MTAR_EWRITEFAIL : return "could not write";
    case MTAR_ESEEKFAIL  : return "could not seek";
    case MTAR_EBADCHKSUM : return "bad checksum";
    case MTAR_ENULLRECORD: return "null record";
    case MTAR_ENOTFOUND  : return "file not found";
    default              : return "unknown error";
  }
}

static int file_write(mtar_t *tar, const void *data, unsigned size) {
  unsigned res = fwrite(data, 1, size, (FILE*)tar->stream);
  return (res == size) ? MTAR_ESUCCESS : MTAR_EWRITEFAIL;
}

static int file_read(mtar_t *tar, void *data, unsigned size) {
  unsigned res = fread(data, 1, size, (FILE*)tar->stream);
  return (res == size) ? MTAR_ESUCCESS : MTAR_EREADFAIL;
}

static int file_seek(mtar_t *tar, unsigned offset) {
  int res = fseek((FILE*)tar->stream, offset, SEEK_SET);
  return (res == 0) ? MTAR_ESUCCESS : MTAR_ESEEKFAIL;
}

static int file_close(mtar_t *tar) {
  fclose((FILE*)tar->stream);
  return MTAR_ESUCCESS;
}

int mtar_open(mtar_t *tar, const char *filename, const char *mode) {
  memset(tar, 0, sizeof(*tar));
  tar->read  = file_read;
  tar->write = file_write;
  tar->seek  = file_seek;
  tar->close = file_close;
  tar->stream = fopen(filename, mode);
  if (!tar->stream) return MTAR_EOPENFAIL;
  if (*mode == 'r') {
    /* Validate by checking the first header */
    mtar_header_t h;
    int err = mtar_read_header(tar, &h);
    if (err != MTAR_ESUCCESS && err != MTAR_ENULLRECORD) {
      mtar_close(tar);
      return err;
    }
    mtar_rewind(tar);
  }
  return MTAR_ESUCCESS;
}

int mtar_close(mtar_t *tar) {
  return tar->close(tar);
}

int mtar_seek(mtar_t *tar, unsigned pos) {
  int err = tar->seek(tar, pos);
  tar->pos = pos;
  return err;
}

int mtar_rewind(mtar_t *tar) {
  tar->remaining_data = 0;
  tar->last_header = 0;
  return mtar_seek(tar, 0);
}

int mtar_next(mtar_t *tar) {
  int err, n;
  mtar_header_t h;
  err = mtar_read_header(tar, &h);
  if (err) return err;
  n = round_up(h.size, 512) + sizeof(mtar_raw_header_t);
  return mtar_seek(tar, tar->last_header + n);
}

int mtar_find(mtar_t *tar, const char *name, mtar_header_t *h) {
  int err;
  mtar_header_t header;
  err = mtar_rewind(tar);
  if (err) return err;
  while ((err = mtar_read_header(tar, &header)) == MTAR_ESUCCESS) {
    if (!strcmp(header.name, name)) {
      if (h) *h = header;
      return MTAR_ESUCCESS;
    }
    mtar_next(tar);
  }
  return MTAR_ENOTFOUND;
}

int mtar_read_header(mtar_t *tar, mtar_header_t *h) {
  int err;
  mtar_raw_header_t rh;
  tar->last_header = tar->pos;
  err = tread(tar, &rh, sizeof(rh));
  if (err) return err;
  err = raw_to_header(h, &rh);
  if (err) return err;
  tar->remaining_data = h->size;
  return MTAR_ESUCCESS;
}

int mtar_read_data(mtar_t *tar, void *ptr, unsigned size) {
  int err;
  if (size > tar->remaining_data) size = tar->remaining_data;
  err = tread(tar, ptr, size);
  if (err) return err;
  tar->remaining_data -= size;
  if (tar->remaining_data == 0) {
    /* Move to next 512-byte boundary */
    unsigned next = tar->last_header + sizeof(mtar_raw_header_t);
    mtar_header_t h;
    mtar_seek(tar, tar->last_header);
    mtar_read_header(tar, &h);
    next = tar->last_header + sizeof(mtar_raw_header_t) + round_up(h.size, 512);
    (void)next;
  }
  return MTAR_ESUCCESS;
}

int mtar_write_header(mtar_t *tar, const mtar_header_t *h) {
  mtar_raw_header_t rh;
  memset(&rh, 0, sizeof(rh));
  header_to_raw(&rh, h);
  tar->remaining_data = h->size;
  return twrite(tar, &rh, sizeof(rh));
}

int mtar_write_file_header(mtar_t *tar, const char *name, unsigned size) {
  mtar_header_t h;
  memset(&h, 0, sizeof(h));
  snprintf(h.name, sizeof(h.name), "%s", name);
  h.size  = size;
  h.type  = MTAR_TREG;
  h.mode  = 0664;
  return mtar_write_header(tar, &h);
}

int mtar_write_dir_header(mtar_t *tar, const char *name) {
  mtar_header_t h;
  memset(&h, 0, sizeof(h));
  snprintf(h.name, sizeof(h.name), "%s/", name);
  h.type = MTAR_TDIR;
  h.mode = 0755;
  return mtar_write_header(tar, &h);
}

int mtar_write_data(mtar_t *tar, const void *data, unsigned size) {
  int err = twrite(tar, data, size);
  if (err) return err;
  tar->remaining_data -= size;
  if (tar->remaining_data == 0) {
    unsigned rem = round_up(tar->pos, 512) - tar->pos;
    return write_null_bytes(tar, (int)rem);
  }
  return MTAR_ESUCCESS;
}

int mtar_finalize(mtar_t *tar) {
  return write_null_bytes(tar, 1024);
}

/* ── Memory-backed read-only tar ─────────────────────────────────────────── */

static int mem_read(mtar_t *tar, void *data, unsigned size) {
  mtar_mem_stream_t *ms = (mtar_mem_stream_t*)tar->stream;
  if (ms->pos + size > ms->size) return MTAR_EREADFAIL;
  memcpy(data, ms->data + ms->pos, size);
  ms->pos += size;
  return MTAR_ESUCCESS;
}

static int mem_write(mtar_t *tar, const void *data, unsigned size) {
  (void)tar; (void)data; (void)size;
  return MTAR_EWRITEFAIL;  /* read-only */
}

static int mem_seek(mtar_t *tar, unsigned pos) {
  mtar_mem_stream_t *ms = (mtar_mem_stream_t*)tar->stream;
  if (pos > ms->size) return MTAR_ESEEKFAIL;
  ms->pos = pos;
  return MTAR_ESUCCESS;
}

static int mem_close(mtar_t *tar) {
  free(tar->stream);
  tar->stream = NULL;
  return MTAR_ESUCCESS;
}

int mtar_open_mem(mtar_t *tar, const void *data, size_t size) {
  mtar_mem_stream_t *ms = (mtar_mem_stream_t*)malloc(sizeof(*ms));
  if (!ms) return MTAR_EFAILURE;
  ms->data = (const unsigned char*)data;
  ms->size = size;
  ms->pos  = 0;
  memset(tar, 0, sizeof(*tar));
  tar->read   = mem_read;
  tar->write  = mem_write;
  tar->seek   = mem_seek;
  tar->close  = mem_close;
  tar->stream = ms;
  return MTAR_ESUCCESS;
}
