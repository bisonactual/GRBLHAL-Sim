#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <commdlg.h>

#include <cctype>
#include <cstring>
#include <cstdio>
#include <iterator>
#include <string>
#include <vector>

namespace {

constexpr UINT WM_APP_LOG = WM_APP + 1;
constexpr UINT WM_APP_STOPPED = WM_APP + 2;
constexpr UINT_PTR STATE_TIMER_ID = 1;
constexpr UINT STATE_TIMER_MS = 500;
constexpr char SIM_STDIN_REPORT_STATE = 0x12;
constexpr char SIM_STDIN_FORCE_IDLE = 'i';

enum ControlId {
    IDC_SIM_PATH = 100,
    IDC_BROWSE,
    IDC_WS_PORT,
    IDC_EEPROM,
    IDC_SPEED,
    IDC_STEP,
    IDC_NO_COMMENT,
    IDC_START,
    IDC_STOP,
    IDC_STATUS,
    IDC_CONTROLLER_STATE,
    IDC_LOG,
    IDC_INJECT_BASE = 1000
};

struct InjectionButton {
    const char *label;
    char key;
};

const InjectionButton kInjectionButtons[] = {
    {"E-Stop", 'e'},
    {"Reset", 'r'},
    {"Feed Hold", 'h'},
    {"Cycle Start", 's'},
    {"Safety Door", 'd'},
    {"Probe", 'p'},
    {"Probe Connected", 'o'},
    {"X Limit", 'x'},
    {"Y Limit", 'y'},
    {"Z Limit", 'z'},
    {"Alarm 1 Hard Limit", '1'},
    {"Alarm 2 Soft Limit", '2'},
    {"Alarm 3 Abort Cycle", '3'},
    {"Alarm 4 Probe Initial", '4'},
    {"Alarm 5 Probe Contact", '5'},
    {"Alarm 6 Homing Reset", '6'},
    {"Alarm 7 Homing Door", '7'},
    {"Alarm 8 Pull-off", '8'},
    {"Alarm 9 Approach", '9'},
    {"Alarm 10 E-Stop", '0'},
    {"Motor Fault", 'm'},
    {"Force Idle", SIM_STDIN_FORCE_IDLE},
    {"Mute Replies", 'n'},
    {"Kick Sender", 'k'},
    {"Status Report", '?'}
};

HWND g_hwnd = nullptr;
HWND g_simPath = nullptr;
HWND g_wsPort = nullptr;
HWND g_eeprom = nullptr;
HWND g_speed = nullptr;
HWND g_step = nullptr;
HWND g_noComment = nullptr;
HWND g_start = nullptr;
HWND g_stop = nullptr;
HWND g_status = nullptr;
HWND g_controllerState = nullptr;
HWND g_log = nullptr;
HFONT g_font = nullptr;

HANDLE g_process = nullptr;
HANDLE g_processThread = nullptr;
HANDLE g_stdinWrite = nullptr;
HANDLE g_stdoutRead = nullptr;
HANDLE g_readerThread = nullptr;
bool g_running = false;
std::string g_outputBuffer;

std::string get_text(HWND hwnd)
{
    int len = GetWindowTextLengthA(hwnd);
    std::string text(static_cast<size_t>(len) + 1, '\0');
    if (len > 0)
        GetWindowTextA(hwnd, text.data(), len + 1);
    text.resize(static_cast<size_t>(len));
    return text;
}

void set_text(HWND hwnd, const std::string &text)
{
    SetWindowTextA(hwnd, text.c_str());
}

std::string dirname_of(const std::string &path)
{
    size_t slash = path.find_last_of("\\/");
    if (slash == std::string::npos)
        return ".";
    return path.substr(0, slash);
}

std::string module_dir()
{
    char path[MAX_PATH] = {0};
    GetModuleFileNameA(nullptr, path, MAX_PATH);
    return dirname_of(path);
}

std::string default_sim_path()
{
    return module_dir() + "\\grblHAL_flexihal_sim.exe";
}

std::string settings_path()
{
    char local[MAX_PATH] = {0};
    DWORD n = GetEnvironmentVariableA("LOCALAPPDATA", local, MAX_PATH);
    std::string dir = (n > 0 && n < MAX_PATH) ? local : module_dir();
    dir += "\\grblHAL FlexiHAL Simulator";
    CreateDirectoryA(dir.c_str(), nullptr);
    return dir + "\\launcher.ini";
}

std::string read_ini(const char *key, const char *fallback)
{
    char value[1024] = {0};
    GetPrivateProfileStringA("launcher", key, fallback, value, sizeof(value), settings_path().c_str());
    return value;
}

void write_ini(const char *key, const std::string &value)
{
    WritePrivateProfileStringA("launcher", key, value.c_str(), settings_path().c_str());
}

std::string quote_arg(const std::string &arg)
{
    std::string out = "\"";
    for (char c : arg) {
        if (c == '"')
            out += "\\\"";
        else
            out += c;
    }
    out += "\"";
    return out;
}

std::string normalize_log_text(const char *data, DWORD len)
{
    std::string out;
    out.reserve(len * 2);
    for (DWORD i = 0; i < len; ++i) {
        if (data[i] == '\n' && (i == 0 || data[i - 1] != '\r'))
            out += "\r\n";
        else
            out += data[i];
    }
    return out;
}

void append_log(const std::string &text)
{
    int existing = GetWindowTextLengthA(g_log);
    if (existing > 256 * 1024)
        SetWindowTextA(g_log, "");

    SendMessageA(g_log, EM_SETSEL, static_cast<WPARAM>(-1), static_cast<LPARAM>(-1));
    SendMessageA(g_log, EM_REPLACESEL, FALSE, reinterpret_cast<LPARAM>(text.c_str()));
}

void handle_output_line(std::string line)
{
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n'))
        line.pop_back();

    constexpr const char *state_prefix = "[SIMSTATE]";
    constexpr size_t state_prefix_len = 10;

    if (line.rfind(state_prefix, 0) == 0) {
        set_text(g_controllerState, line.substr(state_prefix_len));
        return;
    }

    append_log(line + "\r\n");
}

void handle_output_chunk(const char *text)
{
    g_outputBuffer += text;

    size_t line_end = std::string::npos;
    while ((line_end = g_outputBuffer.find('\n')) != std::string::npos) {
        std::string line = g_outputBuffer.substr(0, line_end + 1);
        g_outputBuffer.erase(0, line_end + 1);
        handle_output_line(line);
    }

    if (g_outputBuffer.size() > 4096) {
        append_log(g_outputBuffer);
        g_outputBuffer.clear();
    }
}

void update_running_state(bool running)
{
    g_running = running;
    EnableWindow(g_start, running ? FALSE : TRUE);
    EnableWindow(g_stop, running ? TRUE : FALSE);
    SetWindowTextA(g_status, running ? "Running" : "Stopped");

    if (running) {
        set_text(g_controllerState, "<Starting>");
        SetTimer(g_hwnd, STATE_TIMER_ID, STATE_TIMER_MS, nullptr);
    } else {
        KillTimer(g_hwnd, STATE_TIMER_ID);
        set_text(g_controllerState, "<Stopped>");
    }
}

void save_settings()
{
    write_ini("sim_path", get_text(g_simPath));
    write_ini("ws_port", get_text(g_wsPort));
    write_ini("eeprom", get_text(g_eeprom));
    write_ini("speed", get_text(g_speed));
    write_ini("step", get_text(g_step));
    write_ini("no_comment", SendMessageA(g_noComment, BM_GETCHECK, 0, 0) == BST_CHECKED ? "1" : "0");
}

void load_settings()
{
    set_text(g_simPath, read_ini("sim_path", default_sim_path().c_str()));
    set_text(g_wsPort, read_ini("ws_port", "8080"));
    set_text(g_eeprom, read_ini("eeprom", "EEPROM.DAT"));
    set_text(g_speed, read_ini("speed", "1.0"));
    set_text(g_step, read_ini("step", "0"));
    SendMessageA(g_noComment, BM_SETCHECK, read_ini("no_comment", "0") == "1" ? BST_CHECKED : BST_UNCHECKED, 0);
}

void close_process_handles()
{
    if (g_readerThread) {
        WaitForSingleObject(g_readerThread, 1000);
        CloseHandle(g_readerThread);
        g_readerThread = nullptr;
    }
    if (g_stdinWrite) {
        CloseHandle(g_stdinWrite);
        g_stdinWrite = nullptr;
    }
    if (g_stdoutRead) {
        CloseHandle(g_stdoutRead);
        g_stdoutRead = nullptr;
    }
    if (g_processThread) {
        CloseHandle(g_processThread);
        g_processThread = nullptr;
    }
    if (g_process) {
        CloseHandle(g_process);
        g_process = nullptr;
    }
}

DWORD WINAPI reader_thread(void *)
{
    char buffer[2048];
    DWORD read = 0;
    while (g_stdoutRead && ReadFile(g_stdoutRead, buffer, sizeof(buffer), &read, nullptr) && read > 0) {
        std::string text = normalize_log_text(buffer, read);
        char *posted = new char[text.size() + 1];
        memcpy(posted, text.c_str(), text.size() + 1);
        PostMessageA(g_hwnd, WM_APP_LOG, 0, reinterpret_cast<LPARAM>(posted));
    }

    if (g_process)
        WaitForSingleObject(g_process, INFINITE);

    PostMessageA(g_hwnd, WM_APP_STOPPED, 0, 0);
    return 0;
}

bool create_child_pipes(HANDLE *stdin_read, HANDLE *stdout_write)
{
    SECURITY_ATTRIBUTES sa = {};
    sa.nLength = sizeof(sa);
    sa.bInheritHandle = TRUE;

    HANDLE stdin_write = nullptr;
    HANDLE stdout_read = nullptr;
    if (!CreatePipe(stdin_read, &stdin_write, &sa, 0))
        return false;
    if (!SetHandleInformation(stdin_write, HANDLE_FLAG_INHERIT, 0)) {
        CloseHandle(*stdin_read);
        CloseHandle(stdin_write);
        return false;
    }

    if (!CreatePipe(&stdout_read, stdout_write, &sa, 0)) {
        CloseHandle(*stdin_read);
        CloseHandle(stdin_write);
        return false;
    }
    if (!SetHandleInformation(stdout_read, HANDLE_FLAG_INHERIT, 0)) {
        CloseHandle(*stdin_read);
        CloseHandle(stdin_write);
        CloseHandle(stdout_read);
        CloseHandle(*stdout_write);
        return false;
    }

    g_stdinWrite = stdin_write;
    g_stdoutRead = stdout_read;
    return true;
}

void start_simulator()
{
    if (g_running)
        return;

    save_settings();

    std::string sim = get_text(g_simPath);
    std::string cmd = quote_arg(sim);
    cmd += " -w " + get_text(g_wsPort);
    cmd += " -t " + get_text(g_speed);

    std::string eeprom = get_text(g_eeprom);
    if (!eeprom.empty())
        cmd += " -e " + quote_arg(eeprom);

    std::string step = get_text(g_step);
    if (!step.empty() && step != "0")
        cmd += " -r " + step;

    if (SendMessageA(g_noComment, BM_GETCHECK, 0, 0) == BST_CHECKED)
        cmd += " -n";

    HANDLE stdin_read = nullptr;
    HANDLE stdout_write = nullptr;
    if (!create_child_pipes(&stdin_read, &stdout_write)) {
        MessageBoxA(g_hwnd, "Could not create process pipes.", "Launcher", MB_ICONERROR);
        return;
    }

    STARTUPINFOA si = {};
    PROCESS_INFORMATION pi = {};
    si.cb = sizeof(si);
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdInput = stdin_read;
    si.hStdOutput = stdout_write;
    si.hStdError = stdout_write;

    std::vector<char> mutable_cmd(cmd.begin(), cmd.end());
    mutable_cmd.push_back('\0');
    std::string workdir = dirname_of(sim);

    BOOL ok = CreateProcessA(
        nullptr,
        mutable_cmd.data(),
        nullptr,
        nullptr,
        TRUE,
        CREATE_NO_WINDOW,
        nullptr,
        workdir.c_str(),
        &si,
        &pi
    );

    CloseHandle(stdin_read);
    CloseHandle(stdout_write);

    if (!ok) {
        DWORD err = GetLastError();
        close_process_handles();
        char msg[256];
        snprintf(msg, sizeof(msg), "Could not start simulator. Windows error %lu.", static_cast<unsigned long>(err));
        MessageBoxA(g_hwnd, msg, "Launcher", MB_ICONERROR);
        return;
    }

    g_process = pi.hProcess;
    g_processThread = pi.hThread;
    g_readerThread = CreateThread(nullptr, 0, reader_thread, nullptr, 0, nullptr);
    g_outputBuffer.clear();
    update_running_state(true);
    append_log("> " + cmd + "\r\n");
}

void stop_simulator(bool closing)
{
    if (!g_running || !g_process)
        return;

    DWORD written = 0;
    const char quit = 0x06;
    if (g_stdinWrite)
        WriteFile(g_stdinWrite, &quit, 1, &written, nullptr);

    SetWindowTextA(g_status, "Stopping");
    if (closing && WaitForSingleObject(g_process, 2000) == WAIT_TIMEOUT)
        TerminateProcess(g_process, 1);
}

bool focus_is_edit()
{
    HWND focus = GetFocus();
    if (!focus)
        return false;

    char class_name[32] = {0};
    GetClassNameA(focus, class_name, sizeof(class_name));
    return strcmp(class_name, "Edit") == 0;
}

bool is_injection_key(char c)
{
    for (const auto &button : kInjectionButtons) {
        if (button.key == c)
            return true;
    }
    return false;
}

void send_injection(char c)
{
    if (!g_running || !g_stdinWrite) {
        MessageBeep(MB_ICONWARNING);
        return;
    }

    DWORD written = 0;
    WriteFile(g_stdinWrite, &c, 1, &written, nullptr);

    char msg[64];
    snprintf(msg, sizeof(msg), "> inject %c\r\n", c);
    append_log(msg);
}

void request_controller_state()
{
    if (!g_running || !g_stdinWrite)
        return;

    DWORD written = 0;
    WriteFile(g_stdinWrite, &SIM_STDIN_REPORT_STATE, 1, &written, nullptr);
}

void browse_for_simulator()
{
    char file[MAX_PATH] = {0};
    std::string current = get_text(g_simPath);
    strncpy(file, current.c_str(), sizeof(file) - 1);

    OPENFILENAMEA ofn = {};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = g_hwnd;
    ofn.lpstrFilter = "Simulator executable\0grblHAL_flexihal_sim.exe\0Executables\0*.exe\0All files\0*.*\0";
    ofn.lpstrFile = file;
    ofn.nMaxFile = sizeof(file);
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST;

    if (GetOpenFileNameA(&ofn))
        set_text(g_simPath, file);
}

HWND add_control(const char *klass, const char *text, DWORD style, int x, int y, int w, int h, int id)
{
    HWND hwnd = CreateWindowExA(
        0,
        klass,
        text,
        WS_CHILD | WS_VISIBLE | style,
        x,
        y,
        w,
        h,
        g_hwnd,
        reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)),
        GetModuleHandleA(nullptr),
        nullptr
    );
    SendMessageA(hwnd, WM_SETFONT, reinterpret_cast<WPARAM>(g_font), TRUE);
    return hwnd;
}

void create_ui()
{
    g_font = reinterpret_cast<HFONT>(GetStockObject(DEFAULT_GUI_FONT));

    add_control("STATIC", "Simulator", 0, 16, 18, 90, 20, 0);
    g_simPath = add_control("EDIT", "", WS_BORDER | ES_AUTOHSCROLL, 110, 14, 510, 24, IDC_SIM_PATH);
    add_control("BUTTON", "Browse", 0, 630, 13, 86, 26, IDC_BROWSE);

    add_control("STATIC", "WebSocket port", 0, 16, 52, 100, 20, 0);
    g_wsPort = add_control("EDIT", "", WS_BORDER | ES_AUTOHSCROLL, 120, 50, 80, 24, IDC_WS_PORT);
    add_control("STATIC", "EEPROM", 0, 220, 52, 60, 20, 0);
    g_eeprom = add_control("EDIT", "", WS_BORDER | ES_AUTOHSCROLL, 280, 50, 180, 24, IDC_EEPROM);
    add_control("STATIC", "Speed", 0, 480, 52, 45, 20, 0);
    g_speed = add_control("EDIT", "", WS_BORDER | ES_AUTOHSCROLL, 525, 50, 60, 24, IDC_SPEED);
    add_control("STATIC", "Step log", 0, 600, 52, 55, 20, 0);
    g_step = add_control("EDIT", "", WS_BORDER | ES_AUTOHSCROLL, 655, 50, 60, 24, IDC_STEP);

    g_noComment = add_control("BUTTON", "No comment prefix", BS_AUTOCHECKBOX, 16, 86, 160, 24, IDC_NO_COMMENT);
    g_start = add_control("BUTTON", "Start", 0, 520, 84, 90, 28, IDC_START);
    g_stop = add_control("BUTTON", "Stop", 0, 622, 84, 90, 28, IDC_STOP);
    g_status = add_control("STATIC", "Stopped", SS_CENTER | WS_BORDER, 400, 88, 100, 22, IDC_STATUS);
    add_control("STATIC", "Controller", 0, 16, 116, 80, 20, 0);
    g_controllerState = add_control("STATIC", "<Stopped>", SS_LEFT | WS_BORDER, 100, 114, 616, 24, IDC_CONTROLLER_STATE);

    add_control("BUTTON", "Simulator inputs", BS_GROUPBOX, 16, 148, 700, 238, 0);
    const int cols = 4;
    const int bw = 160;
    const int bh = 26;
    const int gap = 10;
    for (int i = 0; i < static_cast<int>(std::size(kInjectionButtons)); ++i) {
        int row = i / cols;
        int col = i % cols;
        add_control("BUTTON", kInjectionButtons[i].label, 0,
                    32 + col * (bw + gap),
                    174 + row * 30,
                    bw,
                    bh,
                    IDC_INJECT_BASE + i);
    }

    add_control("STATIC", "Log", 0, 16, 400, 60, 20, 0);
    g_log = add_control("EDIT", "", WS_BORDER | WS_VSCROLL | ES_MULTILINE | ES_READONLY | ES_AUTOVSCROLL,
                        16, 422, 700, 238, IDC_LOG);

    load_settings();
    update_running_state(false);
}

LRESULT CALLBACK wnd_proc(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam)
{
    switch (msg) {
        case WM_CREATE:
            g_hwnd = hwnd;
            create_ui();
            return 0;

        case WM_COMMAND: {
            int id = LOWORD(wparam);
            if (id == IDC_START) {
                start_simulator();
            } else if (id == IDC_STOP) {
                stop_simulator(false);
            } else if (id == IDC_BROWSE) {
                browse_for_simulator();
            } else if (id >= IDC_INJECT_BASE &&
                       id < IDC_INJECT_BASE + static_cast<int>(std::size(kInjectionButtons))) {
                send_injection(kInjectionButtons[id - IDC_INJECT_BASE].key);
            }
            return 0;
        }

        case WM_CHAR:
            if (!focus_is_edit()) {
                char c = static_cast<char>(wparam);
                if (std::isalpha(static_cast<unsigned char>(c)))
                    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                if (is_injection_key(c)) {
                    send_injection(c);
                    return 0;
                }
            }
            break;

        case WM_APP_LOG: {
            char *text = reinterpret_cast<char *>(lparam);
            handle_output_chunk(text);
            delete[] text;
            return 0;
        }

        case WM_APP_STOPPED:
            close_process_handles();
            update_running_state(false);
            append_log("> simulator stopped\r\n");
            return 0;

        case WM_TIMER:
            if (wparam == STATE_TIMER_ID) {
                request_controller_state();
                return 0;
            }
            break;

        case WM_CLOSE:
            save_settings();
            stop_simulator(true);
            DestroyWindow(hwnd);
            return 0;

        case WM_DESTROY:
            close_process_handles();
            PostQuitMessage(0);
            return 0;
    }

    return DefWindowProcA(hwnd, msg, wparam, lparam);
}

} // namespace

int WINAPI WinMain(HINSTANCE instance, HINSTANCE, LPSTR, int show)
{
    WNDCLASSA wc = {};
    wc.lpfnWndProc = wnd_proc;
    wc.hInstance = instance;
    wc.lpszClassName = "grblHALFlexiHALLauncher";
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_BTNFACE + 1);

    if (!RegisterClassA(&wc))
        return 1;

    HWND hwnd = CreateWindowExA(
        0,
        wc.lpszClassName,
        "grblHAL FlexiHAL Simulator",
        WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        750,
        720,
        nullptr,
        nullptr,
        instance,
        nullptr
    );

    if (!hwnd)
        return 1;

    ShowWindow(hwnd, show);
    UpdateWindow(hwnd);

    MSG msg;
    while (GetMessageA(&msg, nullptr, 0, 0)) {
        TranslateMessage(&msg);
        DispatchMessageA(&msg);
    }

    return static_cast<int>(msg.wParam);
}
