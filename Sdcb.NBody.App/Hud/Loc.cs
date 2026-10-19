using Sdcb.NBody.Scenarios;

namespace Sdcb.NBody.App.Hud;

internal enum Lang { Zh, En }

/// <summary>Tiny bilingual string table.</summary>
internal static class Loc
{
    // InvariantGlobalization is on, so ask Windows for the UI language directly (0x04 = Chinese).
    public static Lang Current { get; set; } = (Platform.Win32.GetUserDefaultUILanguage() & 0x3FF) == 0x04 ? Lang.Zh : Lang.En;

    private static readonly Dictionary<string, (string Zh, string En)> s_table = new()
    {
        ["app.title"] = ("N·BODY", "N·BODY"),
        ["app.subtitle"] = ("引力的交响", "A Gravitational Symphony"),
        ["attract.hint"] = ("移动鼠标或按任意键开始探索", "Move the mouse or press any key to explore"),
        ["attract.badge"] = ("自动巡游", "CINEMATIC TOUR"),
        ["gallery"] = ("场景", "Scenarios"),
        ["paused"] = ("已暂停", "PAUSED"),
        ["time"] = ("时间", "Time"),
        ["speed"] = ("速度", "Speed"),
        ["bodies"] = ("天体", "Bodies"),
        ["particles"] = ("粒子", "Particles"),
        ["integrator"] = ("积分器", "Integrator"),
        ["energy"] = ("能量误差", "Energy drift"),
        ["fps"] = ("帧率", "FPS"),
        ["gpu"] = ("GPU 计算", "GPU compute"),
        ["cpu"] = ("CPU 计算", "CPU compute"),
        ["throttled"] = ("算力不足，已降速", "Throttled"),
        ["period"] = ("周期", "Period"),
        ["follow"] = ("跟随", "Following"),
        ["trails"] = ("轨迹", "Trails"),
        ["sandbox.hint"] = ("Shift + 左键拖拽：发射天体（拖得越长越快）· C 清空", "Shift + drag to launch a body (longer = faster) · C to clear"),
        ["chaos.hint"] = ("两组初值仅相差 10⁻⁹ —— 看它们何时分道扬镳", "Initial states differ by only 10⁻⁹ — watch them diverge"),
        ["chaos.separation"] = ("相空间分离", "Separation"),
        ["screenshot.saved"] = ("截图已保存", "Screenshot saved"),
        ["muted"] = ("静音", "Muted"),
        ["unmuted"] = ("声音开启", "Sound on"),
        ["help"] = ("帮助", "Help"),
        ["help.title"] = ("操作指南", "Controls"),
        ["help.body"] = (
            "左键拖动  旋转视角\n右键/中键拖动  平移\n滚轮  缩放\n双击天体  跟随 / 取消\nSpace  暂停 / 继续\n← →  上一个 / 下一个场景\n↑ ↓  加速 / 减速\nG  场景画廊\nI  切换积分器\nT  轨迹开关\nR  重置\nTab  隐藏界面\nL  中文 / English\nM  静音\nF11  全屏 / 窗口\nF12  截图\nH  帮助\nEsc  退出",
            "Left drag  orbit camera\nRight/middle drag  pan\nWheel  zoom\nDouble-click body  follow / release\nSpace  pause / resume\n← →  previous / next scenario\n↑ ↓  faster / slower\nG  scenario gallery\nI  cycle integrator\nT  toggle trails\nR  reset\nTab  hide HUD\nL  中文 / English\nM  mute\nF11  fullscreen / window\nF12  screenshot\nH  help\nEsc  quit"),
        ["hint.bar"] = ("H 帮助 · G 场景 · Space 暂停 · ←→ 切换", "H help · G scenarios · Space pause · ←→ switch"),
        ["cat.PeriodicThreeBody"] = ("三体周期解", "Periodic Three-Body"),
        ["cat.Choreography"] = ("编舞与星环", "Choreographies"),
        ["cat.SolarSystem"] = ("行星系统", "Planetary Systems"),
        ["cat.Galaxy"] = ("星系", "Galaxies"),
        ["cat.Cluster"] = ("星团与盘", "Clusters & Disks"),
        ["cat.Chaos"] = ("混沌", "Chaos"),
        ["cat.Sandbox"] = ("沙盒", "Sandbox"),
        ["unit.years"] = ("年", "yr"),
    };

    public static string T(string key) =>
        s_table.TryGetValue(key, out var v) ? (Current == Lang.Zh ? v.Zh : v.En) : key;

    public static string Name(ScenarioInfo info) => Current == Lang.Zh ? info.NameZh : info.NameEn;
    public static string Description(ScenarioInfo info) => Current == Lang.Zh ? info.DescriptionZh : info.DescriptionEn;
    public static string Category(ScenarioCategory c) => T("cat." + c);

    public static string Integrator(IntegratorKind k) => k switch
    {
        IntegratorKind.Leapfrog => "Leapfrog (KDK)",
        IntegratorKind.Yoshida4 => "Yoshida 4",
        IntegratorKind.RK4 => "Runge–Kutta 4",
        IntegratorKind.DormandPrince45 => "Dormand–Prince 5(4)",
        _ => k.ToString(),
    };
}
