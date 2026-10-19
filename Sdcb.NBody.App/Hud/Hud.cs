using System.Numerics;
using Sdcb.NBody.Scenarios;
using Vortice.DirectWrite;
using Vortice.Mathematics;

namespace Sdcb.NBody.App.Hud;

internal enum HudAction { None, Prev, Next, TogglePause, Faster, Slower, ToggleTrails, ToggleGallery, ToggleLang, ToggleHelp, Reset, CycleIntegrator, ToggleMute, Screenshot, Fullscreen, Select, Quit }

internal readonly record struct BodyLabel(Vector2 Screen, string Text, Color4 Color, float Radius);

internal sealed class HudModel
{
    public ScenarioInfo Info = null!;
    public int ScenarioIndex, ScenarioCount;
    public bool Attract, Paused, Gpu, Throttled, Trails, Muted, ShowHud = true, Gallery, Help;
    public double SimTime, TimeScale, Fps, FrameMs, EnergyError = double.NaN;
    public double? Period, ChaosSeparation;
    public string Integrator = "";
    public int Bodies;
    public float[] EnergyHistory = [];
    public int EnergyHistoryCount;
    public string? FollowName;
    public bool Sandbox;
    public Vector2? DragStart, DragEnd;
    public List<BodyLabel> Labels = [];
    public float TitleCard;   // 0..1 alpha of the big centered title
    public string? Toast;
    public float ToastAlpha;
    public string Adapter = "";
}

/// <summary>Lays out and draws the glassmorphism HUD.</summary>
internal sealed class Hud(HudCanvas c)
{
    private float _galleryT, _helpT, _hudT = 1, _attractT;
    private float _galleryScroll;
    public int SelectedIndex { get; private set; } = -1;

    // Segoe Fluent / MDL2 glyphs
    private const string IcPrev = "\uE892", IcNext = "\uE893", IcPlay = "\uE768", IcPause = "\uE769", IcReset = "\uE72C",
        IcGallery = "\uE8A9", IcGlobe = "\uE774", IcHelp = "\uE897", IcVolume = "\uE767", IcMute = "\uE74F", IcCamera = "\uE722",
        IcMinus = "\uE738", IcPlus = "\uE710", IcFull = "\uE740", IcTrail = "\uE890", IcChip = "\uE950";

    public HudAction Draw(HudModel m, float dt)
    {
        HudAction action = HudAction.None;
        float k = 1 - MathF.Exp(-dt * 10);
        _galleryT += ((m.Gallery && !m.Attract ? 1 : 0) - _galleryT) * k;
        _helpT += ((m.Help && !m.Attract ? 1 : 0) - _helpT) * k;
        _hudT += ((m.ShowHud ? 1 : 0) - _hudT) * k;
        _attractT += ((m.Attract ? 1 : 0) - _attractT) * (1 - MathF.Exp(-dt * 3));

        DrawLabels(m);
        DrawSandboxDrag(m);
        DrawTitleCard(m);

        float uiAlpha = _hudT * (1 - _attractT);
        if (uiAlpha > 0.01f)
        {
            DrawTitleBlock(m, uiAlpha);
            DrawStats(m, uiAlpha);
            HudAction a = DrawTransport(m, uiAlpha);
            if (a != HudAction.None) action = a;
            if (_galleryT > 0.01f)
            {
                a = DrawGallery(m, _galleryT * uiAlpha);
                if (a != HudAction.None) action = a;
            }
            if (_helpT > 0.01f) DrawHelp(_helpT * uiAlpha);
        }
        if (_attractT > 0.01f) DrawAttract(m, _attractT);
        DrawToast(m);
        return action;
    }

    private void DrawTitleBlock(HudModel m, float a)
    {
        float x = 28, y = 26, w = 430;
        IDWriteTextFormat desc = c.Format(14, FontWeight.Normal, wrap: true);
        string d = Loc.Description(m.Info);
        float dh = c.Measure(d, desc, w - 44).Y;
        float h = 152 + dh;
        c.Glass(x, y, w, h, 18, a);

        // wordmark
        IDWriteTextFormat logo = c.Format(13, FontWeight.Bold, family: c.UiFont);
        c.Text("N · B O D Y", x + 22, y + 18, 200, 20, logo, HudCanvas.Accent, a);
        c.Text(Loc.T("app.subtitle"), x + 122, y + 18, w - 140, 20, c.Format(13, FontWeight.Normal), HudCanvas.TextDim, a);
        c.Line(new Vector2(x + 22, y + 46), new Vector2(x + w - 22, y + 46), new Color4(1, 1, 1, 1), 1, 0.08f * a);

        string cat = Loc.Category(m.Info.Category).ToUpperInvariant();
        c.Text(cat, x + 22, y + 58, w - 44, 18, c.Format(12, FontWeight.SemiBold), HudCanvas.Warm, a);
        c.Text($"{m.ScenarioIndex + 1:00} / {m.ScenarioCount:00}", x + 22, y + 58, w - 44, 18,
            c.Format(12, FontWeight.Normal, TextAlignment.Trailing, family: c.MonoFont), HudCanvas.TextDim, a);
        c.Text(Loc.Name(m.Info), x + 22, y + 78, w - 44, 44, c.Format(30, FontWeight.SemiBold), HudCanvas.TextPrimary, a);
        c.Text(d, x + 22, y + 128, w - 44, dh + 4, desc, HudCanvas.TextSecondary, a * 0.9f);
    }

    private void DrawStats(HudModel m, float a)
    {
        float w = 290, x = c.Width - w - 28, y = 26;
        bool showEnergy = !m.Gpu && m.EnergyHistoryCount > 1;
        int rows = 5 + (m.Period.HasValue ? 1 : 0) + (m.ChaosSeparation.HasValue ? 1 : 0) + (m.FollowName != null ? 1 : 0);
        float h = 30 + rows * 24 + (showEnergy ? 70 : 0) + 8;
        c.Glass(x, y, w, h, 18, a);

        IDWriteTextFormat key = c.Format(13, FontWeight.Normal);
        IDWriteTextFormat val = c.Format(13, FontWeight.Normal, TextAlignment.Trailing, family: c.MonoFont);
        float ry = y + 20;
        void Row(string k, string v, Color4? color = null)
        {
            c.Text(k, x + 20, ry, w - 40, 20, key, HudCanvas.TextDim, a);
            c.Text(v, x + 20, ry, w - 40, 20, val, color ?? HudCanvas.TextPrimary, a);
            ry += 24;
        }
        Row(Loc.T("fps"), $"{m.Fps:0}  ·  {m.FrameMs:0.0} ms");
        Row(Loc.T(m.Gpu ? "particles" : "bodies"), m.Bodies.ToString("N0"));
        Row(m.Gpu ? Loc.T("gpu") : Loc.T("integrator"), m.Gpu ? "Leapfrog · HLSL" : m.Integrator, HudCanvas.Accent);
        Row(Loc.T("time"), FormatTime(m.SimTime, m.Info.Category));
        Row(Loc.T("speed"), m.Paused ? Loc.T("paused") : $"×{m.TimeScale:0.###}", m.Paused ? HudCanvas.Warm : m.Throttled ? new Color4(1, 0.5f, 0.4f, 1) : null);
        if (m.Period.HasValue) Row(Loc.T("period"), $"T = {m.Period.Value:0.####}");
        if (m.ChaosSeparation.HasValue) Row(Loc.T("chaos.separation"), $"{m.ChaosSeparation.Value:0.00e+0}", HudCanvas.Warm);
        if (m.FollowName != null) Row(Loc.T("follow"), m.FollowName, HudCanvas.Accent);

        if (showEnergy)
        {
            ry += 4;
            c.Text(Loc.T("energy"), x + 20, ry, w - 40, 18, key, HudCanvas.TextDim, a);
            c.Text(double.IsNaN(m.EnergyError) ? "—" : $"{m.EnergyError:0.0e+0}", x + 20, ry, w - 40, 18, val, HudCanvas.TextPrimary, a);
            DrawSparkline(m, x + 20, ry + 24, w - 40, 34, a);
        }
    }

    private void DrawSparkline(HudModel m, float x, float y, float w, float h, float a)
    {
        c.FillRounded(x, y, w, h, 6, new Color4(0, 0, 0, 1), 0.18f * a);
        int n = m.EnergyHistoryCount;
        float lo = -16, hi = 0;
        // log10 scale: -16 (machine precision) .. 0 (100%)
        Vector2 prev = default;
        for (int i = 0; i < n; ++i)
        {
            float v = Math.Clamp(m.EnergyHistory[i], lo, hi);
            float px = x + 4 + (w - 8) * i / (m.EnergyHistory.Length - 1);
            float py = y + h - 4 - (h - 8) * (v - lo) / (hi - lo);
            Vector2 p = new(px, py);
            if (i > 0)
            {
                float t = (v - lo) / (hi - lo);
                Color4 col = new(HudCanvas.Accent.R + (1 - HudCanvas.Accent.R) * t, HudCanvas.Accent.G * (1 - t * 0.5f), HudCanvas.Accent.B * (1 - t), 1);
                c.Line(prev, p, col, 1.5f, a);
            }
            prev = p;
        }
    }

    private static string FormatTime(double t, ScenarioCategory cat) =>
        cat == ScenarioCategory.SolarSystem ? $"{t:0.000} {Loc.T("unit.years")}" : $"t = {t:0.000}";

    private HudAction DrawTransport(HudModel m, float a)
    {
        HudAction act = HudAction.None;
        const float bh = 40, gap = 4;
        string speed = m.Paused ? Loc.T("paused") : $"×{m.TimeScale:0.##}";
        float speedW = 86;
        (string icon, HudAction action, bool active)[] left =
        [
            (IcPrev, HudAction.Prev, false), (m.Paused ? IcPlay : IcPause, HudAction.TogglePause, false), (IcNext, HudAction.Next, false),
        ];
        (string icon, HudAction action, bool active)[] right =
        [
            (IcTrail, HudAction.ToggleTrails, m.Trails), (IcReset, HudAction.Reset, false), (IcChip, HudAction.CycleIntegrator, false),
            (IcGallery, HudAction.ToggleGallery, m.Gallery), (IcGlobe, HudAction.ToggleLang, false), (m.Muted ? IcMute : IcVolume, HudAction.ToggleMute, false),
            (IcCamera, HudAction.Screenshot, false), (IcFull, HudAction.Fullscreen, false), (IcHelp, HudAction.ToggleHelp, m.Help),
        ];
        float w = 16 + (left.Length + right.Length + 2) * (bh + gap) + speedW + 24 + 16;
        float x = (c.Width - w) / 2, y = c.Height - bh - 16 - 28;
        c.Glass(x, y, w, bh + 16, 20, a);
        float cx = x + 8;
        foreach (var b in left)
        {
            if (c.Button(cx, y + 8, bh, bh, b.icon, active: b.active, alpha: a)) act = b.action;
            cx += bh + gap;
        }
        cx += 8;
        if (c.Button(cx, y + 8, bh, bh, IcMinus, alpha: a, fontSize: 12)) act = HudAction.Slower;
        cx += bh;
        c.Text(speed, cx, y + 8, speedW, bh, c.Format(15, FontWeight.SemiBold, TextAlignment.Center, ParagraphAlignment.Center, c.MonoFont),
            m.Paused ? HudCanvas.Warm : HudCanvas.TextPrimary, a);
        cx += speedW;
        if (c.Button(cx, y + 8, bh, bh, IcPlus, alpha: a, fontSize: 12)) act = HudAction.Faster;
        cx += bh + 12;
        c.Line(new Vector2(cx - 6, y + 16), new Vector2(cx - 6, y + bh), new Color4(1, 1, 1, 1), 1, 0.12f * a);
        foreach (var b in right)
        {
            if (c.Button(cx, y + 8, bh, bh, b.icon, active: b.active, alpha: a)) act = b.action;
            cx += bh + gap;
        }

        string hint = m.Sandbox ? Loc.T("sandbox.hint") : m.ChaosSeparation.HasValue ? Loc.T("chaos.hint") : Loc.T("hint.bar");
        c.Text(hint, 0, y + bh + 20, c.Width, 20, c.Format(12, FontWeight.Normal, TextAlignment.Center), HudCanvas.TextDim, a * 0.9f);
        return act;
    }

    private HudAction DrawGallery(HudModel m, float t)
    {
        HudAction act = HudAction.None;
        float w = 360, x = 28 - (1 - t) * (w + 40), y = 26, h = c.Height - 26 - 120;
        c.Glass(x, y, w, h, 18, t);
        c.Text(Loc.T("gallery"), x + 22, y + 16, w - 44, 26, c.Format(18, FontWeight.SemiBold), HudCanvas.TextPrimary, t);
        c.Text("G", x + 22, y + 18, w - 44, 22, c.Format(12, FontWeight.Normal, TextAlignment.Trailing, family: c.MonoFont), HudCanvas.TextDim, t);

        float listTop = y + 56, listH = h - 66;
        bool over = HudCanvas.Contains(x, listTop, w, listH, c.Mouse);
        if (over && c.Wheel != 0) { _galleryScroll -= c.Wheel * 60; c.WheelUsed = true; }

        IReadOnlyList<Scenario> all = ScenarioRegistry.All;
        // compute content height
        float contentH = 0;
        ScenarioCategory? lastCat = null;
        foreach (Scenario s in all)
        {
            if (s.Info.Category != lastCat) { contentH += 34; lastCat = s.Info.Category; }
            contentH += 34;
        }
        _galleryScroll = Math.Clamp(_galleryScroll, 0, Math.Max(0, contentH - listH));

        float cy = listTop - _galleryScroll;
        lastCat = null;
        IDWriteTextFormat catF = c.Format(11, FontWeight.SemiBold);
        IDWriteTextFormat itemF = c.Format(14, FontWeight.Normal, TextAlignment.Leading, ParagraphAlignment.Center);
        for (int i = 0; i < all.Count; ++i)
        {
            ScenarioInfo info = all[i].Info;
            if (info.Category != lastCat)
            {
                lastCat = info.Category;
                if (cy > listTop - 30 && cy < listTop + listH)
                    c.Text(Loc.Category(info.Category).ToUpperInvariant(), x + 22, cy + 12, w - 44, 18, catF, HudCanvas.Warm, t * Fade(cy, listTop, listH));
                cy += 34;
            }
            float fa = Fade(cy, listTop, listH);
            if (fa > 0.01f)
            {
                bool current = i == m.ScenarioIndex;
                bool hot = over && HudCanvas.Contains(x + 12, cy, w - 24, 32, c.Mouse);
                if (current) c.FillRounded(x + 12, cy, w - 24, 32, 9, HudCanvas.Accent, 0.13f * t * fa);
                else if (hot) c.FillRounded(x + 12, cy, w - 24, 32, 9, new Color4(1, 1, 1, 1), 0.07f * t * fa);
                if (current) c.FillRounded(x + 14, cy + 9, 3, 14, 1.5f, HudCanvas.Accent, t * fa);
                c.Text(Loc.Name(info), x + 26, cy, w - 52, 32, itemF, current ? HudCanvas.Accent : hot ? HudCanvas.TextPrimary : HudCanvas.TextSecondary, t * fa);
                if (hot && c.MouseReleased && t > 0.8f) { SelectedIndex = i; act = HudAction.Select; }
                if (hot) c.MarkHot();
            }
            cy += 34;
        }
        return act;
    }

    private static float Fade(float y, float top, float h)
    {
        float a = Math.Clamp((y - top + 20) / 30f, 0, 1);
        float b = Math.Clamp((top + h - y - 20) / 30f, 0, 1);
        return a * b;
    }

    private void DrawHelp(float t)
    {
        string body = Loc.T("help.body");
        string[] lines = body.Split('\n');
        float w = 460, lh = 26, h = 70 + lines.Length * lh + 20;
        float x = (c.Width - w) / 2, y = (c.Height - h) / 2 - 30 + (1 - t) * 20;
        c.Glass(x, y, w, h, 20, t);
        c.Text(Loc.T("help.title"), x + 28, y + 22, w - 56, 30, c.Format(20, FontWeight.SemiBold), HudCanvas.TextPrimary, t);
        IDWriteTextFormat kf = c.Format(13, FontWeight.SemiBold, family: c.MonoFont);
        IDWriteTextFormat vf = c.Format(14, FontWeight.Normal);
        float cy = y + 70;
        foreach (string line in lines)
        {
            int sp = line.IndexOf("  ", StringComparison.Ordinal);
            string keyPart = sp > 0 ? line[..sp] : line;
            string valPart = sp > 0 ? line[(sp + 2)..] : "";
            bool keyFirst = keyPart.Length <= 12 && !keyPart.Contains("拖") && !keyPart.Contains("drag", StringComparison.OrdinalIgnoreCase) && !keyPart.Contains("click", StringComparison.OrdinalIgnoreCase) && !keyPart.Contains("击");
            if (!keyFirst)
            {
                // "action  description" lines (mouse); show the gesture as key
                c.Text(keyPart, x + 28, cy, 190, lh, vf, HudCanvas.Accent, t);
                c.Text(valPart, x + 220, cy, w - 248, lh, vf, HudCanvas.TextSecondary, t);
            }
            else
            {
                c.Text(keyPart, x + 28, cy + 2, 190, lh, kf, HudCanvas.Accent, t);
                c.Text(valPart, x + 220, cy, w - 248, lh, vf, HudCanvas.TextSecondary, t);
            }
            cy += lh;
        }
    }

    private void DrawAttract(HudModel m, float t)
    {
        // cinematic letterbox
        float bar = c.Height * 0.075f * t;
        c.FillRounded(-10, -10, c.Width + 20, bar + 10, 0, new Color4(0, 0, 0, 1), 0.85f);
        c.FillRounded(-10, c.Height - bar, c.Width + 20, bar + 10, 0, new Color4(0, 0, 0, 1), 0.85f);

        float pulse = 0.55f + 0.45f * MathF.Sin((float)Environment.TickCount64 / 1000f * 2.2f);
        IDWriteTextFormat badge = c.Format(12, FontWeight.SemiBold, TextAlignment.Center);
        c.Text("N · B O D Y   —   " + Loc.T("attract.badge"), 0, bar * 0.5f - 9, c.Width, 20, badge, HudCanvas.Accent, t * 0.9f);
        c.Text(Loc.T("attract.hint"), 0, c.Height - bar * 0.5f - 10, c.Width, 20, c.Format(14, FontWeight.Normal, TextAlignment.Center), HudCanvas.TextSecondary, t * pulse);

        // lower-left caption
        float cx = 56, cy = c.Height - bar - 120;
        c.Text(Loc.Category(m.Info.Category).ToUpperInvariant(), cx, cy, 800, 20, c.Format(13, FontWeight.SemiBold), HudCanvas.Warm, t * (1 - m.TitleCard));
        c.Text(Loc.Name(m.Info), cx, cy + 20, 900, 50, c.Format(36, FontWeight.Light), HudCanvas.TextPrimary, t * (1 - m.TitleCard));
        c.Text(Loc.Description(m.Info), cx, cy + 70, 640, 44, c.Format(14, FontWeight.Normal, wrap: true), HudCanvas.TextSecondary, t * 0.85f * (1 - m.TitleCard));
    }

    private void DrawTitleCard(HudModel m)
    {
        float a = m.TitleCard;
        if (a <= 0.01f) return;
        float y = c.Height * 0.40f;
        c.Text(Loc.Category(m.Info.Category).ToUpperInvariant(), 0, y - 34, c.Width, 24, c.Format(15, FontWeight.SemiBold, TextAlignment.Center), HudCanvas.Warm, a);
        c.Text(Loc.Name(m.Info), 0, y - 6, c.Width, 80, c.Format(56, FontWeight.Light, TextAlignment.Center), HudCanvas.TextPrimary, a);
        float lw = 120 * a;
        c.Line(new Vector2(c.Width / 2 - lw, y + 86), new Vector2(c.Width / 2 + lw, y + 86), HudCanvas.Accent, 1.5f, a * 0.8f);
    }

    private void DrawLabels(HudModel m)
    {
        if (m.Labels.Count == 0 || _hudT < 0.05f) return;
        IDWriteTextFormat f = c.Format(12, FontWeight.Normal);
        foreach (BodyLabel l in m.Labels)
        {
            Vector2 p = l.Screen + new Vector2(l.Radius + 8, -l.Radius - 14);
            if (p.X < -100 || p.Y < -40 || p.X > c.Width + 100 || p.Y > c.Height + 40) continue;
            c.Line(l.Screen + new Vector2(l.Radius * 0.7f + 2, -l.Radius * 0.7f - 2), p + new Vector2(-2, 12), l.Color, 1, 0.35f * _hudT);
            c.Text(l.Text, p.X, p.Y, 200, 18, f, HudCanvas.TextSecondary, 0.85f * _hudT);
        }
    }

    private void DrawSandboxDrag(HudModel m)
    {
        if (m.DragStart is not Vector2 s || m.DragEnd is not Vector2 e) return;
        Vector2 d = e - s;
        float len = d.Length();
        c.Circle(s, 7, HudCanvas.Warm, 0.9f);
        c.Circle(s, 14, HudCanvas.Warm, 0.5f, fill: false, width: 1.5f);
        if (len < 3) return;
        c.Line(s, e, HudCanvas.Warm, 2, 0.9f);
        Vector2 dir = d / len, nrm = new(-dir.Y, dir.X);
        c.Line(e, e - dir * 12 + nrm * 7, HudCanvas.Warm, 2, 0.9f);
        c.Line(e, e - dir * 12 - nrm * 7, HudCanvas.Warm, 2, 0.9f);
    }

    private void DrawToast(HudModel m)
    {
        if (m.ToastAlpha <= 0.01f || m.Toast == null) return;
        IDWriteTextFormat f = c.Format(14, FontWeight.SemiBold, TextAlignment.Center, ParagraphAlignment.Center);
        float w = c.Measure(m.Toast, f).X + 48, h = 40;
        float x = (c.Width - w) / 2, y = 30;
        c.Glass(x, y, w, h, 20, m.ToastAlpha);
        c.Text(m.Toast, x, y, w, h, f, HudCanvas.TextPrimary, m.ToastAlpha);
    }
}
