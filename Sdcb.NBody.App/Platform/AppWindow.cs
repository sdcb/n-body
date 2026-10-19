using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using static Sdcb.NBody.App.Platform.Win32;

namespace Sdcb.NBody.App.Platform;

public enum MouseButton { Left, Right, Middle }

/// <summary>Minimal native Win32 window (no WinForms, Native AOT friendly). Single instance.</summary>
internal sealed unsafe class AppWindow : IDisposable
{
    private static AppWindow? s_instance;
    private const string ClassName = "SdcbNBodyWindow";
    private WINDOWPLACEMENT _windowedPlacement;
    private nint _arrowCursor, _handCursor;

    public nint Handle { get; }
    public int Width { get; private set; }
    public int Height { get; private set; }
    public bool IsFullscreen { get; private set; }
    public bool IsMinimized { get; private set; }
    public bool Closed { get; private set; }
    public bool HandCursor { get; set; }
    public float DpiScale => GetDpiForWindow(Handle) / 96f;

    public event Action<int, int>? Resized;
    public event Action<int, bool>? KeyDown;   // vk, repeat
    public event Action<int>? KeyUp;
    public event Action<MouseButton, int, int>? MouseDown;
    public event Action<MouseButton, int, int>? MouseUp;
    public event Action<int, int>? MouseDoubleClick;
    public event Action<int, int>? MouseMove;
    public event Action<float, int, int>? MouseWheel;

    public AppWindow(string title, bool fullscreen, int width = 1600, int height = 900)
    {
        s_instance = this;
        nint hInstance = GetModuleHandle(0);
        _arrowCursor = LoadCursor(0, IDC_ARROW);
        _handCursor = LoadCursor(0, IDC_HAND);
        fixed (char* cls = ClassName)
        {
            WNDCLASSEXW wc = new()
            {
                cbSize = (uint)sizeof(WNDCLASSEXW),
                style = CS_HREDRAW | CS_VREDRAW | CS_DBLCLKS,
                lpfnWndProc = &WndProc,
                hInstance = hInstance,
                hCursor = _arrowCursor,
                hbrBackground = GetStockObject(4 /* BLACK_BRUSH */),
                hIcon = LoadIcon(hInstance, 32512),
                lpszClassName = cls,
            };
            if (RegisterClassEx(&wc) == 0) throw new InvalidOperationException("RegisterClassEx failed");
        }

        RECT rc = new() { Left = 0, Top = 0, Right = width, Bottom = height };
        AdjustWindowRectEx(ref rc, WS_OVERLAPPEDWINDOW, false, 0);
        Handle = CreateWindowEx(0, ClassName, title, WS_OVERLAPPEDWINDOW, 60, 40, rc.Width, rc.Height, 0, 0, hInstance, 0);
        if (Handle == 0) throw new InvalidOperationException("CreateWindowEx failed");

        int dark = 1;
        _ = DwmSetWindowAttribute(Handle, 20 /* DWMWA_USE_IMMERSIVE_DARK_MODE */, &dark, sizeof(int));

        _windowedPlacement.length = (uint)sizeof(WINDOWPLACEMENT);
        if (fullscreen) SetFullscreen(true);
        UpdateClientSize();
    }

    private bool _shown;

    /// <summary>Shows the window; call after the first frame has been presented to avoid a white flash.</summary>
    public void Show()
    {
        if (_shown) return;
        _shown = true;
        ShowWindow(Handle, SW_SHOW);
        SetForegroundWindow(Handle);
        UpdateClientSize();
    }

    public void SetFullscreen(bool on)
    {
        if (on == IsFullscreen) return;
        if (on)
        {
            GetWindowPlacement(Handle, ref _windowedPlacement);
            MONITORINFO mi = new() { cbSize = (uint)sizeof(MONITORINFO) };
            GetMonitorInfo(MonitorFromWindow(Handle, MONITOR_DEFAULTTONEAREST), ref mi);
            SetWindowLongPtr(Handle, GWL_STYLE, unchecked((nint)(WS_POPUP | (_shown ? WS_VISIBLE : 0))));
            SetWindowPos(Handle, 0, mi.rcMonitor.Left, mi.rcMonitor.Top, mi.rcMonitor.Width, mi.rcMonitor.Height,
                SWP_NOZORDER | SWP_NOOWNERZORDER | SWP_FRAMECHANGED | (_shown ? SWP_SHOWWINDOW : 0));
        }
        else
        {
            SetWindowLongPtr(Handle, GWL_STYLE, (nint)(WS_OVERLAPPEDWINDOW | WS_VISIBLE));
            SetWindowPlacement(Handle, in _windowedPlacement);
            SetWindowPos(Handle, 0, 0, 0, 0, 0, 0x0001 | 0x0002 | SWP_NOZORDER | SWP_NOOWNERZORDER | SWP_FRAMECHANGED);
        }
        IsFullscreen = on;
        UpdateClientSize();
    }

    /// <summary>Pumps pending messages. Returns false when the app should quit.</summary>
    public bool PumpMessages()
    {
        while (PeekMessage(out MSG msg, 0, 0, 0, PM_REMOVE))
        {
            if (msg.message == WM_QUIT) { Closed = true; return false; }
            TranslateMessage(in msg);
            DispatchMessage(in msg);
        }
        return !Closed;
    }

    public void Close() => DestroyWindow(Handle);

    public static bool IsKeyDown(int vk) => (GetKeyState(vk) & 0x8000) != 0;

    private void UpdateClientSize()
    {
        GetClientRect(Handle, out RECT rc);
        if (rc.Width > 0 && rc.Height > 0 && (rc.Width != Width || rc.Height != Height))
        {
            Width = rc.Width;
            Height = rc.Height;
            Resized?.Invoke(Width, Height);
        }
    }

    [UnmanagedCallersOnly]
    private static nint WndProc(nint hwnd, uint msg, nint wParam, nint lParam)
    {
        AppWindow? w = s_instance;
        if (w == null || (w.Handle != 0 && hwnd != w.Handle)) return DefWindowProc(hwnd, msg, wParam, lParam);
        try
        {
            return w.HandleMessage(hwnd, msg, wParam, lParam);
        }
        catch (Exception ex)
        {
            Program.ReportFatal(ex);
            PostQuitMessage(1);
            return 0;
        }
    }

    private nint HandleMessage(nint hwnd, uint msg, nint wParam, nint lParam)
    {
        int x = LoWord(lParam), y = HiWord(lParam);
        switch (msg)
        {
            case WM_SIZE:
                IsMinimized = wParam == 1;
                if (!IsMinimized && Handle != 0) UpdateClientSize();
                return 0;
            case WM_SETCURSOR when LoWord(lParam) == 1:
                SetCursor(HandCursor ? _handCursor : _arrowCursor);
                return 1;
            case WM_KEYDOWN or WM_SYSKEYDOWN:
                if ((int)wParam == 0x12) break; // let Alt pass through
                KeyDown?.Invoke((int)wParam, (lParam & (1 << 30)) != 0);
                if (msg == WM_SYSKEYDOWN && (int)wParam != 0x73) return 0; // swallow F10/Alt combos except Alt+F4
                break;
            case WM_KEYUP or WM_SYSKEYUP:
                KeyUp?.Invoke((int)wParam);
                break;
            case WM_MOUSEMOVE: MouseMove?.Invoke(x, y); return 0;
            case WM_LBUTTONDOWN: SetCapture(hwnd); MouseDown?.Invoke(MouseButton.Left, x, y); return 0;
            case WM_RBUTTONDOWN: SetCapture(hwnd); MouseDown?.Invoke(MouseButton.Right, x, y); return 0;
            case WM_MBUTTONDOWN: SetCapture(hwnd); MouseDown?.Invoke(MouseButton.Middle, x, y); return 0;
            case WM_LBUTTONUP: ReleaseCapture(); MouseUp?.Invoke(MouseButton.Left, x, y); return 0;
            case WM_RBUTTONUP: ReleaseCapture(); MouseUp?.Invoke(MouseButton.Right, x, y); return 0;
            case WM_MBUTTONUP: ReleaseCapture(); MouseUp?.Invoke(MouseButton.Middle, x, y); return 0;
            case WM_LBUTTONDBLCLK: MouseDoubleClick?.Invoke(x, y); return 0;
            case WM_MOUSEWHEEL:
            {
                // wheel coordinates are screen-relative; use last known cursor instead
                MouseWheel?.Invoke(HiWord(wParam) / 120f, x, y);
                return 0;
            }
            case WM_CLOSE:
                DestroyWindow(hwnd);
                return 0;
            case WM_DESTROY:
                Closed = true;
                PostQuitMessage(0);
                return 0;
        }
        return DefWindowProc(hwnd, msg, wParam, lParam);
    }

    public void Dispose()
    {
        if (!Closed) DestroyWindow(Handle);
        s_instance = null;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Vk(char c) => c;
}
