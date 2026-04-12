/*
  fs_sim.c - Simulated filesystem backed by a host directory

  Part of grblHAL FlexiHAL Simulator

  Maps grblHAL VFS operations to standard C file I/O against a directory
  on the host.  Defaults to "./sdcard" relative to the working directory.

  Also provides fs_stream_init() (called by plugins_init.h when
  SDCARD_ENABLE is set) which bootstraps the macro file subsystem so
  that G65 P<n> can open P<n>.macro from the VFS.
*/

#include "driver.h"

#if SDCARD_ENABLE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>

#ifdef _WIN32
#include <direct.h>
#include <io.h>
#include <windows.h>
#define SIM_MKDIR(p) _mkdir(p)
#define SIM_STAT     _stat
#define SIM_STRUCT   struct _stat
#else
#include <dirent.h>
#include <unistd.h>
#define SIM_MKDIR(p) mkdir(p, 0755)
#define SIM_STAT     stat
#define SIM_STRUCT   struct stat
#endif

#include "grbl/vfs.h"
#include "grbl/protocol.h"
#include "fs_sim.h"

#define SIM_SD_ROOT  "sdcard"
#define SIM_CARD_SIZE ((uint64_t)1024 * 1024 * 1024)

static char sd_root[512] = SIM_SD_ROOT;

/* ── Helpers ───────────────────────────────────────────────────────────── */

static void make_host_path (char *buf, size_t buflen, const char *vfs_path)
{
    if (vfs_path && *vfs_path == '/')
        vfs_path++;
    snprintf(buf, buflen, "%s/%s", sd_root, vfs_path ? vfs_path : "");
}

/* ── File handle ───────────────────────────────────────────────────────── */

typedef struct { FILE *fp; } sim_file_t;

/* ── File operations ────────────────────────────────────────────────────── */

static vfs_file_t *sim_fopen (const char *filename, const char *mode)
{
    char hp[600];
    make_host_path(hp, sizeof(hp), filename);

    FILE *fp = fopen(hp, mode);
    if (!fp) return NULL;

    vfs_file_t *f = calloc(1, sizeof(vfs_file_t) + sizeof(sim_file_t));
    if (!f) { fclose(fp); return NULL; }

    ((sim_file_t *)&f->handle)->fp = fp;
    fseek(fp, 0, SEEK_END);
    f->size = (size_t)ftell(fp);
    fseek(fp, 0, SEEK_SET);
    return f;
}

static void sim_fclose (vfs_file_t *f)
{
    fclose(((sim_file_t *)&f->handle)->fp);
    free(f);
}

static size_t sim_fread (void *buf, size_t sz, size_t n, vfs_file_t *f)
{
    return fread(buf, sz, n, ((sim_file_t *)&f->handle)->fp);
}

static size_t sim_fwrite (const void *buf, size_t sz, size_t n, vfs_file_t *f)
{
    size_t w = fwrite(buf, sz, n, ((sim_file_t *)&f->handle)->fp);
    fflush(((sim_file_t *)&f->handle)->fp);
    return w;
}

static size_t sim_ftell (vfs_file_t *f)
{
    return (size_t)ftell(((sim_file_t *)&f->handle)->fp);
}

static int sim_fseek (vfs_file_t *f, size_t off)
{
    return fseek(((sim_file_t *)&f->handle)->fp, (long)off, SEEK_SET);
}

static bool sim_feof (vfs_file_t *f)
{
    return feof(((sim_file_t *)&f->handle)->fp) != 0;
}

static int sim_rename (const char *from, const char *to)
{
    char a[600], b[600];
    make_host_path(a, sizeof(a), from);
    make_host_path(b, sizeof(b), to);
    /* On some platforms rename() fails if target exists — remove first */
    remove(b);
    return rename(a, b);
}

static int sim_unlink (const char *fn)
{
    char hp[600];
    make_host_path(hp, sizeof(hp), fn);
    return remove(hp);
}

static int sim_mkdir (const char *p)
{
    char hp[600];
    make_host_path(hp, sizeof(hp), p);
    return SIM_MKDIR(hp);
}

/* ── Directory operations ───────────────────────────────────────────────── */

#ifdef _WIN32

typedef struct {
    HANDLE hFind;
    WIN32_FIND_DATAA fdata;
    bool first;
    char pattern[600];
} sim_dir_t;

static vfs_dir_t *sim_opendir (const char *path)
{
    vfs_dir_t *dir = calloc(1, sizeof(vfs_dir_t) + sizeof(sim_dir_t));
    if (!dir) return NULL;
    sim_dir_t *sd = (sim_dir_t *)&dir->handle;
    char hp[600];
    make_host_path(hp, sizeof(hp), path);
    snprintf(sd->pattern, sizeof(sd->pattern), "%s/*", hp);
    sd->hFind = FindFirstFileA(sd->pattern, &sd->fdata);
    if (sd->hFind == INVALID_HANDLE_VALUE) { free(dir); return NULL; }
    sd->first = true;
    return dir;
}

static char *sim_readdir (vfs_dir_t *dir, vfs_dirent_t *dirent)
{
    sim_dir_t *sd = (sim_dir_t *)&dir->handle;
    for (;;) {
        if (!sd->first) { if (!FindNextFileA(sd->hFind, &sd->fdata)) return NULL; }
        sd->first = false;
        if (!strcmp(sd->fdata.cFileName, ".") || !strcmp(sd->fdata.cFileName, "..")) continue;
        strncpy(dirent->name, sd->fdata.cFileName, sizeof(dirent->name) - 1);
        dirent->size = (size_t)sd->fdata.nFileSizeLow;
        dirent->st_mode.mode = 0;
        if (sd->fdata.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) dirent->st_mode.directory = 1;
        return dirent->name;
    }
}

static void sim_closedir (vfs_dir_t *dir)
{
    sim_dir_t *sd = (sim_dir_t *)&dir->handle;
    if (sd->hFind != INVALID_HANDLE_VALUE) FindClose(sd->hFind);
    free(dir);
}

#else /* POSIX */

typedef struct { DIR *dp; char base[600]; } sim_dir_t;

static vfs_dir_t *sim_opendir (const char *path)
{
    char hp[600];
    make_host_path(hp, sizeof(hp), path);
    DIR *dp = opendir(hp);
    if (!dp) return NULL;
    vfs_dir_t *dir = calloc(1, sizeof(vfs_dir_t) + sizeof(sim_dir_t));
    if (!dir) { closedir(dp); return NULL; }
    sim_dir_t *sd = (sim_dir_t *)&dir->handle;
    sd->dp = dp;
    strncpy(sd->base, hp, sizeof(sd->base) - 1);
    return dir;
}

static char *sim_readdir (vfs_dir_t *dir, vfs_dirent_t *dirent)
{
    sim_dir_t *sd = (sim_dir_t *)&dir->handle;
    struct dirent *de;
    while ((de = readdir(sd->dp))) {
        if (de->d_name[0] == '.') continue;
        strncpy(dirent->name, de->d_name, sizeof(dirent->name) - 1);
        char fp[700];
        snprintf(fp, sizeof(fp), "%s/%s", sd->base, de->d_name);
        SIM_STRUCT st;
        dirent->st_mode.mode = 0;
        if (SIM_STAT(fp, &st) == 0) {
            dirent->size = (size_t)st.st_size;
            if (S_ISDIR(st.st_mode)) dirent->st_mode.directory = 1;
        } else dirent->size = 0;
        return dirent->name;
    }
    return NULL;
}

static void sim_closedir (vfs_dir_t *dir)
{
    sim_dir_t *sd = (sim_dir_t *)&dir->handle;
    if (sd->dp) closedir(sd->dp);
    free(dir);
}

#endif

/* ── Stat / misc ────────────────────────────────────────────────────────── */

static int sim_stat (const char *fn, vfs_stat_t *st)
{
    char hp[600];
    make_host_path(hp, sizeof(hp), fn);
    SIM_STRUCT hst;
    if (SIM_STAT(hp, &hst) != 0) return -1;
    st->st_size = (size_t)hst.st_size;
    st->st_mode.mode = 0;
#ifdef _WIN32
    if (hst.st_mode & _S_IFDIR) st->st_mode.directory = 1;
#else
    if (S_ISDIR(hst.st_mode)) st->st_mode.directory = 1;
#endif
    /* Skip mtime — Linux's <sys/stat.h> #defines st_mtime as a macro
       which collides with vfs_stat_t's member.  Not needed for sim. */
    return 0;
}

static int sim_rmdir (const char *p)
{
    char hp[600];
    make_host_path(hp, sizeof(hp), p);
#ifdef _WIN32
    return _rmdir(hp);
#else
    return rmdir(hp);
#endif
}

static char sim_cwd[256] = "/";

static int sim_chdir (const char *p)
{
    if (p[0] == '/') strncpy(sim_cwd, p, sizeof(sim_cwd) - 1);
    else if (strlen(sim_cwd) + strlen(p) + 2 < sizeof(sim_cwd)) {
        if (sim_cwd[strlen(sim_cwd) - 1] != '/') strcat(sim_cwd, "/");
        strcat(sim_cwd, p);
    }
    return 0;
}

static char *sim_getcwd (char *buf, size_t sz)
{
    if (!buf) { buf = malloc(strlen(sim_cwd) + 1); if (buf) strcpy(buf, sim_cwd); return buf; }
    strncpy(buf, sim_cwd, sz - 1); buf[sz - 1] = '\0';
    return buf;
}

static bool sim_getfree (vfs_free_t *fi)
{
    fi->size = SIM_CARD_SIZE;
    fi->used = 0;
    return true;
}

/* ── VFS table ─────────────────────────────────────────────────────────── */

static const vfs_t sim_vfs = {
    .fs_name = "SimSD", .removable = false,
    .fopen = sim_fopen, .fclose = sim_fclose,
    .fread = sim_fread, .fwrite = sim_fwrite,
    .ftell = sim_ftell, .fseek = sim_fseek, .feof = sim_feof,
    .frename = sim_rename, .funlink = sim_unlink,
    .fmkdir = sim_mkdir, .fchdir = sim_chdir, .frmdir = sim_rmdir,
    .fopendir = sim_opendir, .readdir = sim_readdir, .fclosedir = sim_closedir,
    .fstat = sim_stat, .fgetcwd = sim_getcwd, .fgetfree = sim_getfree,
};

/* ── fs_stream_init — called by plugins_init.h ─────────────────────────── */
/* The real SD card plugin's fs_stream_init() registers $F commands and
   calls fs_macros_init().  We only need the macro subsystem so that
   G65 P<n> can open P<n>.macro files from the VFS. */

extern void fs_macros_init (void);

void fs_stream_init (void)
{
    fs_macros_init();
}

/* ── Public init — call from driver_init() ─────────────────────────────── */

void fs_sim_init (void)
{
    SIM_STRUCT st;
    if (SIM_STAT(sd_root, &st) != 0)
        SIM_MKDIR(sd_root);

    vfs_mount("/", &sim_vfs, (vfs_st_mode_t){ 0 });

    printf("[SIM] Simulated SD card mounted: %s/ (1 GB)\n", sd_root);
}

#endif /* SDCARD_ENABLE */
