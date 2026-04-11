/*
  uf2.c - UF2 firmware upload + plugin manager with embedded web UI

  Part of grblHAL FlexiHAL Simulator

  Serves a tabbed HTML page:
    - Tab 1: Drag-and-drop UF2 firmware upload
    - Tab 2: Plugin manager — upload .c files, rebuild simulator
    - GET /              — serves the page
    - POST /upload       — UF2 firmware upload
    - POST /plugin       — plugin .c file upload
    - GET /plugins       — JSON list of installed plugins
    - POST /rebuild      — trigger cmake rebuild + restart
    - POST /plugin/delete — remove a plugin
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#ifdef WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <direct.h>
typedef SOCKET socket_t;
#define SOCK_INVALID INVALID_SOCKET
#define sock_close closesocket
#define mkdir_p(d) _mkdir(d)
#else
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>
typedef int socket_t;
#define SOCK_INVALID (-1)
#define sock_close close
#define mkdir_p(d) mkdir(d, 0755)
#endif

#include "uf2.h"

static socket_t http_listen_fd = SOCK_INVALID;
static volatile bool firmware_uploaded = false;
static volatile bool rebuild_requested = false;
static char sim_root_dir[512] = {0};  /* path to grblhal-sim/ */
static char plugins_dir[512] = {0};   /* path to grblhal-sim/plugins/ */
static char build_dir[512] = {0};     /* path to grblhal-sim/build/ */

/* ── Embedded HTML page ─────────────────────────────────────────────────── */

static const char *page_html =
"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"
"<!DOCTYPE html><html><head><title>FlexiHAL Simulator</title><style>"
"*{box-sizing:border-box}"
"body{font-family:system-ui;background:#1a1a2e;color:#e0e0e0;"
"display:flex;justify-content:center;padding:40px 20px;margin:0}"
".wrap{max-width:600px;width:100%}"
"h1{color:#00d4ff;margin:0 0 4px;text-align:center}"
".sub{text-align:center;opacity:.5;font-size:14px;margin-bottom:24px}"
".tabs{display:flex;gap:4px;margin-bottom:0}"
".tab{padding:10px 20px;background:#252545;border:none;color:#aaa;"
"cursor:pointer;border-radius:8px 8px 0 0;font-size:14px}"
".tab.active{background:#2a2a4a;color:#00d4ff}"
".panel{background:#2a2a4a;border-radius:0 8px 8px 8px;padding:30px;display:none}"
".panel.active{display:block}"
".drop{border:3px dashed #4a4a6a;border-radius:12px;padding:50px 30px;"
"cursor:pointer;transition:all .2s;text-align:center}"
".drop:hover,.drop.over{border-color:#00d4ff;background:rgba(0,212,255,.05)}"
".drop h2{margin:0 0 6px;color:#00d4ff;font-size:18px}"
".drop p{margin:0;opacity:.6;font-size:13px}"
"#status,#pstatus{margin-top:16px;padding:10px;border-radius:8px;display:none;font-size:13px}"
".ok{background:#0a3d0a;color:#4caf50;display:block!important}"
".err{background:#3d0a0a;color:#f44336;display:block!important}"
".info{background:#0a2a3d;color:#2196f3;display:block!important}"
".plist{margin-top:20px}"
".pitem{display:flex;justify-content:space-between;align-items:center;"
"background:#1a1a2e;padding:10px 14px;border-radius:8px;margin-bottom:6px}"
".pitem .name{font-family:monospace;color:#00d4ff;font-size:14px}"
".pitem .del{background:#5a2020;color:#f44;border:none;padding:4px 10px;"
"border-radius:4px;cursor:pointer;font-size:12px}"
".pitem .del:hover{background:#7a3030}"
".btn{background:#00d4ff;color:#1a1a2e;border:none;padding:10px 24px;"
"border-radius:8px;cursor:pointer;font-size:14px;font-weight:600;margin-top:16px}"
".btn:hover{background:#00b8d4}"
".btn:disabled{opacity:.4;cursor:default}"
".empty{opacity:.4;font-size:13px;text-align:center;padding:20px}"
"</style></head><body><div class='wrap'>"
"<h1>FlexiHAL Simulator</h1>"
"<div class='sub'>grblHAL Virtual Controller</div>"
"<div class='tabs'>"
"<button class='tab active' onclick='showTab(0)'>Firmware</button>"
"<button class='tab' onclick='showTab(1)'>Plugins</button>"
"</div>"
"<div class='panel active' id='p0'>"
"<div class='drop' id='fdrop'>"
"<h2>Drop .uf2 firmware here</h2>"
"<p>or click to browse</p>"
"<input type='file' id='ffile' accept='.uf2' hidden>"
"</div>"
"<div id='status'></div>"
"</div>"
"<div class='panel' id='p1'>"
"<div class='drop' id='pdrop'>"
"<h2>Drop plugin .c or .h file here</h2>"
"<p>Upload a grblHAL plugin source file (headers are inlined automatically)</p>"
"<input type='file' id='pfile' accept='.c,.h' hidden>"
"</div>"
"<div id='pstatus'></div>"
"<div class='plist' id='plist'></div>"
"<button class='btn' id='rbtn' onclick='rebuild()' disabled>"
"Rebuild &amp; Restart Simulator</button>"
"</div>"
"</div>"
"<script>"
"const tabs=document.querySelectorAll('.tab'),"
"panels=document.querySelectorAll('.panel');"
"function showTab(i){tabs.forEach((t,j)=>{t.classList.toggle('active',j==i)});"
"panels.forEach((p,j)=>{p.classList.toggle('active',j==i)});"
"if(i==1)loadPlugins()}"
"const fd=document.getElementById('fdrop'),ff=document.getElementById('ffile'),"
"fs=document.getElementById('status');"
"fd.onclick=()=>ff.click();"
"fd.ondragover=e=>{e.preventDefault();fd.classList.add('over')};"
"fd.ondragleave=()=>fd.classList.remove('over');"
"fd.ondrop=e=>{e.preventDefault();fd.classList.remove('over');"
"if(e.dataTransfer.files[0])uploadFW(e.dataTransfer.files[0])};"
"ff.onchange=()=>{if(ff.files[0])uploadFW(ff.files[0])};"
"function uploadFW(f){"
"if(!f.name.endsWith('.uf2')){show(fs,'Not a .uf2 file','err');return}"
"show(fs,'Uploading '+f.name+'...','info');"
"fetch('/upload',{method:'POST',body:f}).then(r=>r.text())"
".then(t=>show(fs,t,t.includes('OK')?'ok':'err'))"
".catch(e=>show(fs,'Failed: '+e,'err'))}"
"const pd=document.getElementById('pdrop'),pf=document.getElementById('pfile'),"
"ps=document.getElementById('pstatus');"
"pd.onclick=()=>pf.click();"
"pd.ondragover=e=>{e.preventDefault();pd.classList.add('over')};"
"pd.ondragleave=()=>pd.classList.remove('over');"
"pd.ondrop=e=>{e.preventDefault();pd.classList.remove('over');"
"if(e.dataTransfer.files[0])uploadPlugin(e.dataTransfer.files[0])};"
"pf.onchange=()=>{if(pf.files[0])uploadPlugin(pf.files[0])};"
"function uploadPlugin(f){"
"if(!f.name.endsWith('.c')&&!f.name.endsWith('.h')){show(ps,'Must be a .c or .h file','err');return}"
"show(ps,'Uploading '+f.name+'...','info');"
"fetch('/plugin?name='+encodeURIComponent(f.name),{method:'POST',body:f})"
".then(r=>r.text()).then(t=>{show(ps,t,t.includes('OK')?'ok':'err');"
"loadPlugins()}).catch(e=>show(ps,'Failed: '+e,'err'))}"
"function loadPlugins(){"
"fetch('/plugins').then(r=>r.json()).then(list=>{"
"const el=document.getElementById('plist'),rb=document.getElementById('rbtn');"
"if(!list.length){el.innerHTML='<div class=empty>No plugins installed</div>';rb.disabled=true;return}"
"rb.disabled=false;"
"el.innerHTML=list.map(p=>"
"'<div class=pitem><span class=name>'+p+'</span>'"
"+'<button class=del onclick=\"delPlugin(\\''+p+'\\')\">&times; Remove</button></div>'"
").join('')}).catch(()=>{})}"
"function delPlugin(name){"
"fetch('/plugin/delete?name='+encodeURIComponent(name),{method:'POST'})"
".then(r=>r.text()).then(t=>{show(ps,t,t.includes('OK')?'ok':'err');"
"loadPlugins()}).catch(e=>show(ps,'Failed: '+e,'err'))}"
"function rebuild(){"
"const rb=document.getElementById('rbtn');rb.disabled=true;rb.textContent='Rebuilding...';"
"show(ps,'Rebuilding simulator with plugins... this takes a moment','info');"
"fetch('/rebuild',{method:'POST'}).then(r=>r.text()).then(t=>{"
"show(ps,t,t.includes('OK')?'ok':'err');"
"rb.disabled=false;rb.textContent='Rebuild & Restart Simulator';"
"}).catch(e=>{show(ps,'Failed: '+e,'err');"
"rb.disabled=false;rb.textContent='Rebuild & Restart Simulator'})}"
"function show(el,msg,cls){el.textContent=msg;el.className=cls}"
"</script></body></html>";

/* ── Helpers ────────────────────────────────────────────────────────────── */

static void set_nonblocking_http(socket_t fd)
{
#ifdef WIN32
    u_long mode = 1;
    ioctlsocket(fd, FIONBIO, &mode);
#else
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
#endif
}

static void send_response(socket_t fd, const char *status, const char *ctype,
                          const char *body, int body_len)
{
    char hdr[512];
    if (body_len < 0) body_len = (int)strlen(body);
    snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 %s\r\n"
        "Content-Type: %s\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Connection: close\r\n"
        "Content-Length: %d\r\n\r\n",
        status, ctype, body_len);
    send(fd, hdr, (int)strlen(hdr), 0);
    if (body_len > 0)
        send(fd, body, body_len, 0);
}

static void send_text(socket_t fd, const char *status, const char *body)
{
    send_response(fd, status, "text/plain", body, -1);
}

static size_t read_content_length(const char *header)
{
    const char *cl = strstr(header, "Content-Length: ");
    if (!cl) cl = strstr(header, "content-length: ");
    return cl ? (size_t)atol(cl + 16) : 0;
}

static const char *get_query_param(const char *header, const char *param, char *out, size_t out_sz)
{
    char needle[64];
    snprintf(needle, sizeof(needle), "%s=", param);
    const char *p = strstr(header, needle);
    if (!p) return NULL;
    p += strlen(needle);
    size_t i = 0;
    while (*p && *p != '&' && *p != ' ' && *p != '\r' && i < out_sz - 1) {
        if (*p == '%' && p[1] && p[2]) {
            char hex[3] = { p[1], p[2], 0 };
            out[i++] = (char)strtol(hex, NULL, 16);
            p += 3;
        } else {
            out[i++] = *p++;
        }
    }
    out[i] = '\0';
    return out;
}

static uint8_t *read_body(socket_t fd, const char *header, size_t *out_len)
{
    size_t content_len = read_content_length(header);
    *out_len = 0;
    if (content_len == 0 || content_len > 4 * 1024 * 1024) return NULL;

    uint8_t *body = malloc(content_len);
    if (!body) return NULL;

    size_t received = 0;
    while (received < content_len) {
        int n = recv(fd, (char *)body + received, (int)(content_len - received), 0);
        if (n <= 0) break;
        received += n;
    }
    *out_len = received;
    return body;
}

/* ── UF2 validation ─────────────────────────────────────────────────────── */

static bool validate_uf2(const uint8_t *data, size_t len)
{
    if (len < 512 || len % 512 != 0) {
        printf("[UF2] Invalid size: %zu\n", len);
        return false;
    }
    size_t num_blocks = len / 512;
    printf("[UF2] Parsing %zu blocks...\n", num_blocks);

    for (size_t i = 0; i < num_blocks; i++) {
        const uf2_block_t *blk = (const uf2_block_t *)(data + i * 512);
        if (blk->magic_start0 != UF2_MAGIC_START0 ||
            blk->magic_start1 != UF2_MAGIC_START1 ||
            blk->magic_end != UF2_MAGIC_END) {
            printf("[UF2] Block %zu: bad magic\n", i);
            return false;
        }
        if (i == 0) {
            printf("[UF2] Family: 0x%08X, addr: 0x%08X, blocks: %u\n",
                   blk->family_id, blk->target_addr, blk->num_blocks);
        }
    }
    printf("[UF2] Valid: %zu blocks\n", num_blocks);
    return true;
}

/* ── Plugin management ──────────────────────────────────────────────────── */

static void ensure_plugins_dir(void)
{
    struct stat st;
    if (stat(plugins_dir, &st) != 0)
        mkdir_p(plugins_dir);
}

/* Generate my_plugin.c that calls all plugin init functions */
/* Sanitize a filename stem into a valid C identifier */
#define SANITIZE_TO_C_IDENT(src, dst, maxlen) do { \
    size_t _i; \
    for (_i = 0; _i < (maxlen) - 1 && (src)[_i]; _i++) { \
        char _c = (src)[_i]; \
        (dst)[_i] = ((_c >= 'a' && _c <= 'z') || (_c >= 'A' && _c <= 'Z') || \
                      (_c >= '0' && _c <= '9') || _c == '_') ? _c : '_'; \
    } \
    (dst)[_i] = '\0'; \
} while(0)

/*
   Scan a plugin .c file for its init function.
   Looks for a line matching: void XXXX_init(void)
   Returns the function name (e.g. "atci_init") or NULL.
*/
static const char *find_init_func(const char *filepath, char *out, size_t out_sz)
{
    FILE *pf = fopen(filepath, "r");
    if (!pf) return NULL;

    char line[512];
    while (fgets(line, sizeof(line), pf)) {
        char *p = strstr(line, "void ");
        if (!p) continue;
        p += 5;
        while (*p == ' ') p++;

        char *end = strstr(p, "_init");
        if (!end) continue;
        end += 5;

        char *q = end;
        while (*q == ' ') q++;
        if (*q != '(') continue;

        size_t len = end - p;
        if (len >= out_sz) continue;
        memcpy(out, p, len);
        out[len] = '\0';
        fclose(pf);
        return out;
    }
    fclose(pf);
    return NULL;
}

static void regenerate_my_plugin(void)
{
    char path[600];
    snprintf(path, sizeof(path), "%s/src/my_plugin.c", sim_root_dir);

    FILE *f = fopen(path, "w");
    if (!f) {
        printf("[PLUGIN] Failed to write %s\n", path);
        return;
    }

    fprintf(f,
        "/* Auto-generated by FlexiHAL Simulator plugin manager */\n"
        "#define ADD_MY_PLUGIN\n"
        "#include \"grbl/hal.h\"\n\n");

    /* Scan plugins dir for .c files, find their init functions */
#ifndef WIN32
    DIR *dir = opendir(plugins_dir);
    if (dir) {
        struct dirent *ent;
        /* Collect init function names */
        char funcs[32][256];
        int nfuncs = 0;

        while ((ent = readdir(dir)) != NULL && nfuncs < 32) {
            size_t nlen = strlen(ent->d_name);
            if (nlen > 2 && strcmp(ent->d_name + nlen - 2, ".c") == 0) {
                char filepath[768];
                snprintf(filepath, sizeof(filepath), "%s/%s", plugins_dir, ent->d_name);

                char func[256];
                if (find_init_func(filepath, func, sizeof(func))) {
                    strncpy(funcs[nfuncs], func, sizeof(funcs[0]) - 1);
                    nfuncs++;
                    printf("[PLUGIN] Found init: %s() in %s\n", func, ent->d_name);
                } else {
                    /* Fallback: use sanitized filename */
                    char raw[256], safe[256];
                    strncpy(raw, ent->d_name, nlen - 2);
                    raw[nlen - 2] = '\0';
                    SANITIZE_TO_C_IDENT(raw, safe, sizeof(safe));
                    snprintf(funcs[nfuncs], sizeof(funcs[0]), "%.240s_init", safe);
                    nfuncs++;
                    printf("[PLUGIN] Guessing init: %s_init() for %s\n", safe, ent->d_name);
                }
            }
        }
        closedir(dir);

        /* Write extern declarations */
        for (int i = 0; i < nfuncs; i++)
            fprintf(f, "extern void %s(void);\n", funcs[i]);

        fprintf(f, "\nvoid my_plugin_init(void)\n{\n");

        /* Write init calls */
        for (int i = 0; i < nfuncs; i++)
            fprintf(f, "    %s();\n", funcs[i]);

    } else {
        fprintf(f, "\nvoid my_plugin_init(void)\n{\n");
    }
#else
    fprintf(f, "\nvoid my_plugin_init(void)\n{\n");
#endif

    fprintf(f, "}\n");
    fclose(f);
    printf("[PLUGIN] Regenerated my_plugin.c\n");
}

/* Generate a CMakeLists.txt snippet that adds plugin sources */
static void regenerate_plugin_cmake(void)
{
    char path[600];
    snprintf(path, sizeof(path), "%s/plugins/CMakeLists_plugins.txt", sim_root_dir);

    FILE *f = fopen(path, "w");
    if (!f) return;

    fprintf(f, "# Auto-generated plugin list\nset(plugin_SRC\n");

#ifndef WIN32
    DIR *dir = opendir(plugins_dir);
    if (dir) {
        struct dirent *ent;
        while ((ent = readdir(dir)) != NULL) {
            size_t nlen = strlen(ent->d_name);
            if (nlen > 2 && strcmp(ent->d_name + nlen - 2, ".c") == 0)
                fprintf(f, "    ${CMAKE_SOURCE_DIR}/plugins/%s\n", ent->d_name);
        }
        closedir(dir);
    }
#endif

    fprintf(f, ")\n");
    fclose(f);
}

/*
   Try to inline a local header file into the output.
   Searches the plugins dir for the header. If found, writes its contents
   (recursively processing any nested local includes) into `out`.
   Returns true if the header was found and inlined.
*/
static bool inline_local_header(FILE *out, const char *header_name, int depth)
{
    if (depth > 8) return false; /* prevent infinite recursion */

    char hpath[768];
    snprintf(hpath, sizeof(hpath), "%s/%s", plugins_dir, header_name);

    FILE *hf = fopen(hpath, "r");
    if (!hf) return false;

    fprintf(out, "/* ---- inlined from %s ---- */\n", header_name);

    char hline[1024];
    while (fgets(hline, sizeof(hline), hf)) {
        /* Check for nested local includes */
        char *t = hline;
        while (*t == ' ' || *t == '\t') t++;

        if (strncmp(t, "#include \"", 10) == 0 && !strstr(t, "grbl/") && !strstr(t, "driver.h")) {
            char nested[256];
            const char *start = t + 10;
            const char *end = strchr(start, '"');
            if (end && (size_t)(end - start) < sizeof(nested)) {
                memcpy(nested, start, end - start);
                nested[end - start] = '\0';
                if (!inline_local_header(out, nested, depth + 1))
                    fprintf(out, "/* stripped (not found): %s */\n", hline);
            } else {
                fputs(hline, out);
            }
        } else {
            /* Skip include guards — they cause issues when inlined */
            if (strncmp(t, "#ifndef ", 8) == 0 || strncmp(t, "#define ", 8) == 0 ||
                (strncmp(t, "#endif", 6) == 0 && strstr(t, "_H"))) {
                char *guard = strstr(t, "_H");
                if (guard) {
                    fprintf(out, "/* %s */\n", "include guard removed");
                    continue;
                }
            }
            fputs(hline, out);
        }
    }

    fprintf(out, "/* ---- end %s ---- */\n", header_name);
    fclose(hf);
    return true;
}

static void handle_plugin_upload(socket_t fd, const char *header)
{
    char filename[256] = {0};
    if (!get_query_param(header, "name", filename, sizeof(filename))) {
        send_text(fd, "400 Bad Request", "ERROR: Missing filename");
        return;
    }

    /* Sanitize: only allow alphanumeric, underscore, dash, dot */
    for (char *p = filename; *p; p++) {
        if (!(*p >= 'a' && *p <= 'z') && !(*p >= 'A' && *p <= 'Z') &&
            !(*p >= '0' && *p <= '9') && *p != '_' && *p != '-' && *p != '.') {
            send_text(fd, "400 Bad Request", "ERROR: Invalid filename characters");
            return;
        }
    }

    size_t len = strlen(filename);
    bool is_c = len >= 3 && strcmp(filename + len - 2, ".c") == 0;
    bool is_h = len >= 3 && strcmp(filename + len - 2, ".h") == 0;

    if (!is_c && !is_h) {
        send_text(fd, "400 Bad Request", "ERROR: Must be a .c or .h file");
        return;
    }

    size_t body_len = 0;
    uint8_t *body = read_body(fd, header, &body_len);
    if (!body || body_len == 0) {
        send_text(fd, "400 Bad Request", "ERROR: Empty file");
        if (body) free(body);
        return;
    }

    ensure_plugins_dir();

    char filepath[768];
    snprintf(filepath, sizeof(filepath), "%s/%s", plugins_dir, filename);

    /* Header files: save as-is (they'll be inlined into .c files at build) */
    if (is_h) {
        FILE *f = fopen(filepath, "w");
        if (!f) {
            send_text(fd, "500 Internal Server Error", "ERROR: Could not write file");
            free(body);
            return;
        }
        fwrite(body, 1, body_len, f);
        fclose(f);
        free(body);
        printf("[PLUGIN] Saved header %s (%zu bytes)\n", filename, body_len);
        char msg[512];
        snprintf(msg, sizeof(msg),
            "OK: Header '%s' uploaded. Upload the .c file next, or Rebuild if already present.",
            filename);
        send_text(fd, "200 OK", msg);
        return;
    }

    /* ── .c file preprocessing ──
       1. Strip board-specific #if guards
       2. Inline local #include "foo.h" headers (not grbl/ or system headers)
    */
    FILE *f = fopen(filepath, "w");
    if (!f) {
        send_text(fd, "500 Internal Server Error", "ERROR: Could not write file");
        free(body);
        return;
    }

    char *src = (char *)body;
    char *end_ptr = src + body_len;
    char line_buf[1024];

    while (src < end_ptr) {
        /* Extract one line */
        char *eol = memchr(src, '\n', end_ptr - src);
        size_t line_len = eol ? (size_t)(eol - src + 1) : (size_t)(end_ptr - src);
        if (line_len >= sizeof(line_buf)) line_len = sizeof(line_buf) - 1;
        memcpy(line_buf, src, line_len);
        line_buf[line_len] = '\0';
        src += line_len;

        char *trimmed = line_buf;
        while (*trimmed == ' ' || *trimmed == '\t') trimmed++;

        /* ── Board guard stripping ── */
        if (strncmp(trimmed, "#if", 3) == 0 && strstr(trimmed, "BOARD_")) {
            char *amp = strstr(trimmed, "&&");
            if (amp && strstr(amp, "BOARD_")) {
                size_t prefix_len = amp - trimmed;
                char fixed[1024];
                memcpy(fixed, trimmed, prefix_len);
                snprintf(fixed + prefix_len, sizeof(fixed) - prefix_len,
                         " // board guard removed by simulator\n");
                fputs(fixed, f);
            } else {
                fprintf(f, "#if 1 // board guard removed by simulator: %s", trimmed + 3);
            }
            continue;
        }

        /* ── Local header inlining ──
           Inline #include "foo.h" when it's not a system or grbl header.
           This makes plugins self-contained for the simulator build. */
        if (strncmp(trimmed, "#include \"", 10) == 0 &&
            strncmp(trimmed, "#include \"grbl/", 15) != 0 &&
            strncmp(trimmed, "#include \"driver.h\"", 19) != 0) {

            const char *start = trimmed + 10;
            const char *end_q = strchr(start, '"');
            if (end_q) {
                char hname[256];
                size_t hlen = end_q - start;
                if (hlen < sizeof(hname)) {
                    memcpy(hname, start, hlen);
                    hname[hlen] = '\0';

                    /* First, save the raw body to a temp location so the header
                       can be found if it was uploaded in the same source blob.
                       But normally the .h is uploaded separately or already exists. */
                    if (inline_local_header(f, hname, 0)) {
                        printf("[PLUGIN] Inlined header: %s\n", hname);
                    } else {
                        fprintf(f, "/* stripped: #include \"%s\" (not found in plugins/) */\n", hname);
                        printf("[PLUGIN] Warning: header %s not found, stripped\n", hname);
                    }
                    continue;
                }
            }
        }

        /* ── Default: pass through ── */
        fputs(line_buf, f);
    }

    fclose(f);
    free(body);

    printf("[PLUGIN] Saved %s (%zu bytes, preprocessed)\n", filename, body_len);

    /* Regenerate build files */
    regenerate_my_plugin();
    regenerate_plugin_cmake();

    char msg[512];
    snprintf(msg, sizeof(msg), "OK: Plugin '%s' uploaded. Click Rebuild to apply.", filename);
    send_text(fd, "200 OK", msg);
}

static void handle_plugin_list(socket_t fd)
{
    char json[4096] = "[";
    size_t jlen = 1;

#ifndef WIN32
    DIR *dir = opendir(plugins_dir);
    if (dir) {
        struct dirent *ent;
        bool first = true;
        while ((ent = readdir(dir)) != NULL) {
            size_t nlen = strlen(ent->d_name);
            if (nlen > 2 && strcmp(ent->d_name + nlen - 2, ".c") == 0) {
                if (!first) json[jlen++] = ',';
                jlen += snprintf(json + jlen, sizeof(json) - jlen,
                                 "\"%s\"", ent->d_name);
                first = false;
            }
        }
        closedir(dir);
    }
#endif

    json[jlen++] = ']';
    json[jlen] = '\0';
    send_response(fd, "200 OK", "application/json", json, (int)jlen);
}

static void handle_plugin_delete(socket_t fd, const char *header)
{
    char filename[256] = {0};
    if (!get_query_param(header, "name", filename, sizeof(filename))) {
        send_text(fd, "400 Bad Request", "ERROR: Missing filename");
        return;
    }

    char filepath[768];
    snprintf(filepath, sizeof(filepath), "%s/%s", plugins_dir, filename);

    if (remove(filepath) != 0) {
        send_text(fd, "404 Not Found", "ERROR: Plugin not found");
        return;
    }

    printf("[PLUGIN] Deleted %s\n", filename);
    regenerate_my_plugin();
    regenerate_plugin_cmake();

    char msg[512];
    snprintf(msg, sizeof(msg), "OK: Plugin '%s' removed. Click Rebuild to apply.", filename);
    send_text(fd, "200 OK", msg);
}

static void handle_rebuild(socket_t fd)
{
    printf("[PLUGIN] Rebuild requested...\n");

    /* Run cmake + make in the build directory */
    char cmd[2048];
    snprintf(cmd, sizeof(cmd),
        "cd \"%s\" && cmake .. 2>&1 && make -j$(nproc) 2>&1",
        build_dir);

    printf("[PLUGIN] Running: %s\n", cmd);
    FILE *proc = popen(cmd, "r");
    if (!proc) {
        send_text(fd, "500 Internal Server Error", "ERROR: Could not start build");
        return;
    }

    /* Ring-buffer approach: keep the last ~60KB of output so errors aren't truncated */
    #define BUILD_BUF_SIZE (64 * 1024)
    char *output = calloc(1, BUILD_BUF_SIZE);
    if (!output) {
        pclose(proc);
        send_text(fd, "500 Internal Server Error", "ERROR: Out of memory");
        return;
    }
    size_t olen = 0;
    char line[512];
    while (fgets(line, sizeof(line), proc)) {
        size_t llen = strlen(line);
        printf("[BUILD] %s", line);
        if (olen + llen < BUILD_BUF_SIZE - 1) {
            memcpy(output + olen, line, llen);
            olen += llen;
        } else {
            /* Buffer full — shift to keep the tail */
            size_t keep = BUILD_BUF_SIZE / 2;
            memmove(output, output + olen - keep, keep);
            olen = keep;
            memcpy(output + olen, line, llen);
            olen += llen;
        }
    }
    output[olen] = '\0';
    int ret = pclose(proc);

    if (ret == 0) {
        rebuild_requested = true;
        send_text(fd, "200 OK",
            "OK: Build succeeded. Restart the simulator to load new plugins.");
        printf("[PLUGIN] Build succeeded\n");
    } else {
        /* Send last ~8KB to the HTTP client (response size limit) */
        const size_t max_resp = 8000;
        const char *tail = (olen > max_resp) ? output + olen - max_resp : output;
        char *msg = malloc(max_resp + 128);
        if (msg) {
            snprintf(msg, max_resp + 128, "ERROR: Build failed (exit %d):\n%s", ret, tail);
            send_text(fd, "500 Internal Server Error", msg);
            free(msg);
        } else {
            send_text(fd, "500 Internal Server Error", "ERROR: Build failed (out of memory for log)");
        }
        printf("[PLUGIN] Build failed (exit %d)\n", ret);
    }
    free(output);
    #undef BUILD_BUF_SIZE
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void uf2_set_paths(const char *root, const char *build)
{
    strncpy(sim_root_dir, root, sizeof(sim_root_dir) - 1);
    snprintf(plugins_dir, sizeof(plugins_dir), "%s/plugins", root);
    strncpy(build_dir, build, sizeof(build_dir) - 1);
    ensure_plugins_dir();
}

int uf2_http_init(uint16_t port)
{
    http_listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (http_listen_fd == SOCK_INVALID) return -1;

    int opt = 1;
#ifdef WIN32
    setsockopt(http_listen_fd, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(opt));
#else
    setsockopt(http_listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#endif

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(http_listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        printf("[HTTP] Failed to bind port %d\n", port);
        return -1;
    }

    listen(http_listen_fd, 4);
    set_nonblocking_http(http_listen_fd);

    printf("[HTTP] Management page at http://localhost:%d\n", port);
    return 0;
}

bool uf2_poll(void)
{
    if (http_listen_fd == SOCK_INVALID) return false;

    bool result = firmware_uploaded;
    firmware_uploaded = false;

    struct sockaddr_in caddr;
    socklen_t clen = sizeof(caddr);
    socket_t fd = accept(http_listen_fd, (struct sockaddr *)&caddr, &clen);
    if (fd == SOCK_INVALID) return result;

#ifdef WIN32
    DWORD tv = 5000;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv));
#else
    struct timeval tv = { .tv_sec = 5, .tv_usec = 0 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif

    /* Read HTTP request header */
    char header[4096] = {0};
    int hlen = 0;
    while (hlen < (int)sizeof(header) - 1) {
        int n = recv(fd, header + hlen, 1, 0);
        if (n <= 0) break;
        hlen++;
        if (hlen >= 4 && memcmp(header + hlen - 4, "\r\n\r\n", 4) == 0)
            break;
    }

    /* Route requests */
    if (strncmp(header, "GET / ", 6) == 0 || strncmp(header, "GET /index", 10) == 0) {
        send(fd, page_html, (int)strlen(page_html), 0);

    } else if (strncmp(header, "GET /plugins", 12) == 0) {
        handle_plugin_list(fd);

    } else if (strncmp(header, "POST /upload", 12) == 0) {
        size_t body_len = 0;
        uint8_t *body = read_body(fd, header, &body_len);
        if (body && body_len > 0 && validate_uf2(body, body_len)) {
            send_text(fd, "200 OK", "OK: Firmware uploaded. Simulator restarting...");
            firmware_uploaded = true;
        } else {
            send_text(fd, "400 Bad Request", "ERROR: Invalid UF2 file");
        }
        if (body) free(body);

    } else if (strncmp(header, "POST /plugin/delete", 19) == 0) {
        handle_plugin_delete(fd, header);

    } else if (strncmp(header, "POST /plugin", 12) == 0) {
        handle_plugin_upload(fd, header);

    } else if (strncmp(header, "POST /rebuild", 13) == 0) {
        handle_rebuild(fd);

    } else if (strncmp(header, "OPTIONS", 7) == 0) {
        const char *cors = "HTTP/1.1 204 No Content\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Access-Control-Allow-Methods: GET,POST\r\n"
            "Access-Control-Allow-Headers: Content-Type\r\n"
            "Connection: close\r\n\r\n";
        send(fd, cors, (int)strlen(cors), 0);

    } else {
        send_text(fd, "404 Not Found", "Not found");
    }

    sock_close(fd);
    return result || firmware_uploaded;
}

bool uf2_rebuild_requested(void)
{
    bool r = rebuild_requested;
    rebuild_requested = false;
    return r;
}

void uf2_shutdown(void)
{
    if (http_listen_fd != SOCK_INVALID) {
        sock_close(http_listen_fd);
        http_listen_fd = SOCK_INVALID;
    }
}
