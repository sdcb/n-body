using System.Numerics;
using Sdcb.NBody.App.Rendering;
using Vortice;
using Vortice.Direct2D1;
using Vortice.DirectWrite;
using Vortice.Mathematics;

namespace Sdcb.NBody.App.Hud;

/// <summary>Immediate-mode Direct2D drawing helpers + input for the HUD. All coordinates in DIPs.</summary>
internal sealed class HudCanvas : IDisposable
{
    private readonly ID2D1DeviceContext _dc;
    private readonly IDWriteFactory _dw;
    private readonly ID2D1SolidColorBrush _brush;
    private readonly ID2D1StrokeStyle _roundStroke;
    private readonly Dictionary<(string, float, FontWeight, TextAlignment, ParagraphAlignment, bool), IDWriteTextFormat> _formats = [];

    public string UiFont { get; }
    public string MonoFont { get; }
    public string IconFont { get; }
    public float Scale { get; set; } = 1;
    public float Width { get; set; }
    public float Height { get; set; }

    // input (DIPs)
    public Vector2 Mouse;
    public bool MouseDown, MousePressed, MouseReleased;
    public float Wheel;
    public bool MouseOverUi { get; private set; }
    public bool WheelUsed;
    public bool AnyHot { get; private set; }

    public List<GlassPanel> Panels { get; } = [];

    public static readonly Color4 Accent = new(0.45f, 0.85f, 1.0f, 1f);
    public static readonly Color4 Warm = new(1.0f, 0.72f, 0.38f, 1f);
    public static readonly Color4 TextPrimary = new(0.95f, 0.97f, 1f, 1f);
    public static readonly Color4 TextSecondary = new(0.72f, 0.77f, 0.86f, 1f);
    public static readonly Color4 TextDim = new(0.5f, 0.55f, 0.65f, 1f);

    public HudCanvas(ID2D1DeviceContext dc, IDWriteFactory dw, ID2D1Factory1 factory)
    {
        _dc = dc;
        _dw = dw;
        _brush = dc.CreateSolidColorBrush(new Color4(1, 1, 1, 1));
        _roundStroke = factory.CreateStrokeStyle(new StrokeStyleProperties { StartCap = CapStyle.Round, EndCap = CapStyle.Round, LineJoin = LineJoin.Round });
        using IDWriteFontCollection fonts = dw.GetSystemFontCollection(false);
        UiFont = Pick(fonts, "Segoe UI Variable Display", "Segoe UI");
        MonoFont = Pick(fonts, "Cascadia Mono", "Consolas");
        IconFont = Pick(fonts, "Segoe Fluent Icons", "Segoe MDL2 Assets");
    }

    private static string Pick(IDWriteFontCollection fonts, params string[] names)
    {
        foreach (string n in names) if (fonts.FindFamilyName(n, out _)) return n;
        return names[^1];
    }

    public void BeginFrame()
    {
        Panels.Clear();
        MouseOverUi = false;
        AnyHot = false;
        WheelUsed = false;
    }

    public void EndFrame()
    {
        MousePressed = false;
        MouseReleased = false;
        Wheel = 0;
    }

    public IDWriteTextFormat Format(float size, FontWeight weight = FontWeight.Normal, TextAlignment align = TextAlignment.Leading,
        ParagraphAlignment palign = ParagraphAlignment.Near, string? family = null, bool wrap = false)
    {
        family ??= UiFont;
        var key = (family, size, weight, align, palign, wrap);
        if (_formats.TryGetValue(key, out IDWriteTextFormat? f)) return f;
        f = _dw.CreateTextFormat(family, weight, Vortice.DirectWrite.FontStyle.Normal, FontStretch.Normal, size);
        f.TextAlignment = align;
        f.ParagraphAlignment = palign;
        f.WordWrapping = wrap ? WordWrapping.Wrap : WordWrapping.NoWrap;
        return _formats[key] = f;
    }

    public ID2D1SolidColorBrush Brush(Color4 c, float alpha = 1)
    {
        _brush.Color = new Color4(c.R, c.G, c.B, c.A * alpha);
        return _brush;
    }

    public void Text(string text, float x, float y, float w, float h, IDWriteTextFormat fmt, Color4 color, float alpha = 1)
    {
        if (alpha <= 0.003f || string.IsNullOrEmpty(text)) return;
        _dc.DrawText(text, fmt, new Rect(x, y, w, h), Brush(color, alpha), DrawTextOptions.EnableColorFont);
    }

    public Vector2 Measure(string text, IDWriteTextFormat fmt, float maxWidth = 10000)
    {
        using IDWriteTextLayout layout = _dw.CreateTextLayout(text, fmt, maxWidth, 10000);
        TextMetrics m = layout.Metrics;
        return new Vector2(m.WidthIncludingTrailingWhitespace, m.Height);
    }

    public void Glass(float x, float y, float w, float h, float radius = 14, float alpha = 1)
    {
        if (alpha <= 0.01f) return;
        Panels.Add(new GlassPanel(x * Scale, y * Scale, w * Scale, h * Scale, radius * Scale, Math.Clamp(alpha, 0, 1), 1));
        RoundedRectangle rr = Rounded(x, y, w, h, radius);
        _dc.FillRoundedRectangle(rr, Brush(new Color4(0.6f, 0.75f, 1f, 1f), 0.025f * alpha));
        _dc.DrawRoundedRectangle(rr, Brush(new Color4(0.7f, 0.85f, 1f, 1f), 0.16f * alpha), 1f);
        // top highlight line
        _dc.DrawLine(new Vector2(x + radius, y + 0.5f), new Vector2(x + w - radius, y + 0.5f), Brush(new Color4(1, 1, 1, 1), 0.18f * alpha), 1f, _roundStroke);
        if (Contains(x, y, w, h, Mouse)) MouseOverUi = true;
    }

    public static RoundedRectangle Rounded(float x, float y, float w, float h, float r) =>
        new() { Rect = new RawRectF(x, y, x + w, y + h), RadiusX = r, RadiusY = r };

    public void FillRounded(float x, float y, float w, float h, float r, Color4 c, float alpha = 1) =>
        _dc.FillRoundedRectangle(Rounded(x, y, w, h, r), Brush(c, alpha));

    public void StrokeRounded(float x, float y, float w, float h, float r, Color4 c, float alpha = 1, float width = 1) =>
        _dc.DrawRoundedRectangle(Rounded(x, y, w, h, r), Brush(c, alpha), width);

    public void Line(Vector2 a, Vector2 b, Color4 c, float width = 1, float alpha = 1) =>
        _dc.DrawLine(a, b, Brush(c, alpha), width, _roundStroke);

    public void Circle(Vector2 c, float r, Color4 color, float alpha = 1, bool fill = true, float width = 1)
    {
        Ellipse e = new(c, r, r);
        if (fill) _dc.FillEllipse(e, Brush(color, alpha));
        else _dc.DrawEllipse(e, Brush(color, alpha), width);
    }

    public void MarkHot() { AnyHot = true; MouseOverUi = true; }

    public static bool Contains(float x, float y, float w, float h, Vector2 p) => p.X >= x && p.X < x + w && p.Y >= y && p.Y < y + h;

    /// <summary>Icon/text button. Returns true when clicked.</summary>
    public bool Button(float x, float y, float w, float h, string label, bool icon = true, bool active = false, float alpha = 1, float fontSize = 16)
    {
        bool hot = Contains(x, y, w, h, Mouse);
        if (hot) { MouseOverUi = true; AnyHot = true; }
        float bg = active ? 0.16f : hot ? (MouseDown ? 0.18f : 0.1f) : 0f;
        if (bg > 0) FillRounded(x, y, w, h, h * 0.3f, active ? Accent : new Color4(1, 1, 1, 1), bg * alpha);
        IDWriteTextFormat f = Format(fontSize, FontWeight.Normal, TextAlignment.Center, ParagraphAlignment.Center, icon ? IconFont : UiFont);
        Text(label, x, y, w, h, f, active ? Accent : hot ? TextPrimary : TextSecondary, alpha);
        return hot && MouseReleased && alpha > 0.5f;
    }

    public void Dispose()
    {
        foreach (IDWriteTextFormat f in _formats.Values) f.Dispose();
        _brush.Dispose();
        _roundStroke.Dispose();
    }
}
