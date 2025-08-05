using FlysEngine;
using FlysEngine.Desktop;
using Sdcb.NBody;
using Sdcb.NBody.Direct2D;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Windows.Forms;
using Vortice.Direct2D1;
using Vortice.Direct2D1.Effects;
using Vortice.DXGI;
using Vortice.Mathematics;
using Vortice.UIAnimation;

class NBodyWindow : RenderWindow
{
    IEnumerator<SystemSnapshot> _system;
    SystemSnapshot _lastSnapshot;
    BodyUIProps[] _uiBodies;
    float _speed = 1.0f, _acc = 0, initialScale = 1f;
    IUIAnimationVariable2 _scale;
    const float RefDt = 0.015625f;
    ID2D1StrokeStyle1 _stroke;
    ID2D1Bitmap1? _bitmap;
    ID2D1Effect? _effect;

    public NBodyWindow(IEnumerable<SystemSnapshot> sys)
    {
        _system = sys.GetEnumerator();
        _system.MoveNext();
        _lastSnapshot = _system.Current;
        _uiBodies = new BodyUIProps[_lastSnapshot.Bodies.Length];
        foreach ((Color4 color, int i) in GenerateColorMap().Take(_uiBodies.Length).Select((color, i) => (color, i)))
        {
            _uiBodies[i] = new()
            {
                Color = color
            };
        }
        _scale = XResource.CreateAnimation(ClientSize.Height * initialScale, ClientSize.Height * initialScale, 0);
        _stroke = XResource.Direct2DFactory.CreateStrokeStyle(new StrokeStyleProperties1 { StartCap = CapStyle.Flat, EndCap = CapStyle.Flat });
    }

    protected override void OnMouseWheel(MouseEventArgs e)
    {
        double finalValue = _scale.Value * (1 + 0.2f * e.Delta / 120);
        using IUIAnimationTransition2 transition = XResource.TransitionLibrary.CreateAccelerateDecelerateTransition(0.25f, finalValue, 0.2, 0.8);
        XResource.Animation.ScheduleTransition(_scale, transition, XResource.DurationSinceStart.TotalSeconds);
    }

    protected override void OnKeyUp(KeyEventArgs e)
    {
        if (e.KeyCode == Keys.Up) _speed *= 1.2f;
        else if (e.KeyCode == Keys.Down) _speed /= 1.2f;
    }

    protected override void OnCreateDeviceResources()
    {
        _effect = new ID2D1Effect(XResource.RenderTarget.CreateEffect(EffectGuids.GaussianBlur));
    }

    protected override void OnReleaseDeviceResources()
    {
        _effect?.Dispose(); _effect = null;
    }

    protected override void OnCreateDeviceSizeResources()
    {
        ID2D1DeviceContext dc = XResource.RenderTarget;
        _bitmap = dc.CreateBitmap(new SizeI(ClientSize.Width, ClientSize.Height), new BitmapProperties1
        {
            BitmapOptions = BitmapOptions.Target,
            PixelFormat = new Vortice.DCommon.PixelFormat(Format.B8G8R8A8_UNorm, Vortice.DCommon.AlphaMode.Premultiplied),
            DpiX = 96,
            DpiY = 96
        });
    }

    protected override void OnReleaseDeviceSizeResources()
    {
        _bitmap?.Dispose(); _bitmap = null;
    }

    protected override void OnUpdateLogic(float dt)
    {
        float unit = _speed * RefDt;

        try
        {
            while (_acc < unit)
            {
                if (!_system.MoveNext())
                {
                    // End of simulation data
                    break;
                }

                // Calculate the time delta for this simulation step.
                float snapshotDt = _system.Current.Timestamp - _lastSnapshot.Timestamp;
                _acc += snapshotDt;

                for (int i = 0; i < _system.Current.Bodies.Length; ++i)
                {
                    BodySnapshot star = _system.Current.Bodies[i];
                    BodyUIProps props = _uiBodies[i];

                    Vector2 now = new(star.Px, star.Py);
                    // Add the new point along with the time delta that led to it.
                    props.TrackingHistory.Add(new TimedVector2(snapshotDt, now));
                }
                _lastSnapshot = _system.Current;
            }
        }
        catch (OperationCanceledException) // device lose
        {
        }

        if (_acc >= unit) _acc -= unit;
    }

    protected override void OnDraw(ID2D1DeviceContext ctx)
    {
        //ctx.Clear(new Color4(0.05f));
        //DrawCore(ctx);
        using ID2D1Image oldBmp = ctx.Target;
        ctx.Target = _bitmap;
        DrawCore(ctx);
        ctx.Transform = Matrix3x2.Identity;

        ctx.Target = oldBmp;
        ctx.Clear(new Color4(0.05f));
        ctx.UnitMode = UnitMode.Pixels;
        _effect!.SetInput(0, _bitmap, invalidate: true);
        _effect!.SetValue((int)GaussianBlurProperties.StandardDeviation, 15.0f);

        ctx.DrawImage(_effect!);
        ctx.DrawImage(_bitmap!);
        ctx.UnitMode = UnitMode.Dips;
    }

    private void DrawCore(ID2D1DeviceContext ctx)
    {
        ctx.Clear(Colors.Transparent);

        float allHeight = ctx.Size.Height;
        float allWidth = ctx.Size.Width;
        ctx.Transform =
            Matrix3x2.CreateTranslation(allWidth / (float)_scale.Value * 0.5f, allHeight / (float)_scale.Value * 0.5f) *
            Matrix3x2.CreateScale((float)_scale.Value, (float)_scale.Value);

        // --- Draw trails first, so bodies are on top ---
        DrawPathsInBatches(ctx);

        // --- Then draw the bodies themselves ---
        DrawBodies(ctx);
    }

    /// <summary>
    /// Renders fading trails for all celestial bodies using time-based chunking.
    /// Each body's trail is divided into segments with gradient opacity and stroke width
    /// to create a visually appealing fade effect from old to new positions.
    /// </summary>
    private void DrawPathsInBatches(ID2D1DeviceContext ctx)
    {
        // Trail rendering configuration
        const float maxTrailDuration = 5.0f;  // Total duration of trail history to display (simulation time)
        const int numChunks = 50;             // Number of segments for gradient effect

        const float maxStrokeWidth = 0.02f;   // Stroke width for newest trail segments
        const float minStrokeWidth = 0.005f;  // Stroke width for oldest trail segments
        const float maxAlpha = 1.0f;          // Alpha for newest trail segments
        const float minAlpha = 0.1f;          // Alpha for oldest trail segments

        // Render each chunk with progressive gradient styling
        for (int i = 0; i < numChunks; i++)
        {
            // Render trails for each celestial body
            foreach (BodyUIProps props in _uiBodies)
            {
                if (props.TrackingHistory.Count < 2)
                {
                    continue; // Need at least 2 points to form a line segment
                }

                // Group trajectory history into time-based chunks using extension method
                // GroupedByTimeFromOldest returns chunks ordered from oldest to newest
                IReadOnlyList<IReadOnlyList<Vector2>> trailChunks = props.TrackingHistory.GroupedByTimeFromOldest(maxTrailDuration, numChunks);
                if (i >= trailChunks.Count)
                {
                    continue; // No valid chunks to render
                }

                IReadOnlyList<Vector2> chunk = trailChunks[i];
                if (chunk.Count == 0)
                {
                    continue;
                }

                // Calculate gradient progression from oldest (0.0) to newest (1.0)
                float progress = (trailChunks.Count > 1) ? (float)i / (trailChunks.Count - 1) : 1.0f;

                float currentStrokeWidth = minStrokeWidth + (maxStrokeWidth - minStrokeWidth) * progress;
                float currentAlpha = minAlpha + (maxAlpha - minAlpha) * progress;
                Color4 finalColor = new(props.Color.R, props.Color.G, props.Color.B, currentAlpha);

                // Create path geometry on-demand for current chunk
                using ID2D1PathGeometry1 path = XResource.Direct2DFactory.CreatePathGeometry();
                using (ID2D1GeometrySink sink = path.Open())
                {
                    // Connect all points in the chunk to form a continuous line
                    if (i == 0)
                    {
                        // First chunk: start from the first point
                        sink.BeginFigure(chunk[0], FigureBegin.Hollow);
                        for (int j = 1; j < chunk.Count; j++)
                        {
                            sink.AddLine(chunk[j]);
                        }
                    }
                    else
                    {
                        // Subsequent chunks: connect from the last point of previous chunk
                        sink.BeginFigure(trailChunks[i - 1][^1], FigureBegin.Hollow);
                        for (int j = 0; j < chunk.Count; j++)
                        {
                            sink.AddLine(chunk[j]);
                        }
                    }
                    sink.EndFigure(FigureEnd.Open);
                    sink.Close();
                }

                // Render the path geometry with calculated style
                ctx.DrawGeometry(path, XResource.GetColor(finalColor), currentStrokeWidth, _stroke);
            }
        }
    }

    private void DrawBodies(ID2D1DeviceContext ctx)
    {
        for (int i = 0; i < _lastSnapshot.Bodies.Length; ++i)
        {
            BodySnapshot star = _lastSnapshot.Bodies[i];
            BodyUIProps prop = _uiBodies[i];
            Vector2 center = new(star.Px, star.Py);
            float radius = star.Size;

            using ID2D1GradientStopCollection collection = ctx.CreateGradientStopCollection(
            [
                new GradientStop{ Color = Colors.White, Position = 0 },
                new GradientStop{ Color = star.BodyType switch
                {
                    BodyType.Solar => prop.Color,
                    BodyType.BlackHole => Colors.Black,
                    _ => Colors.Gray,
                }, Position = 1 },
            ]);
            using ID2D1RadialGradientBrush radialBrush = ctx.CreateRadialGradientBrush(new RadialGradientBrushProperties
            {
                Center = center,
                RadiusX = radius,
                RadiusY = radius,
            }, collection);

            ctx.FillEllipse(new Ellipse(center, radius, radius), radialBrush);
        }
    }

    protected override void OnClosed(EventArgs e) => _system.Dispose();

    static IEnumerable<Color4> GenerateColorMap()
    {
        for (int i = 1; ; ++i)
        {
            int j = 0;
            int lab = i;
            int r = 0, g = 0, b = 0;
            while (lab != 0)
            {
                r |= (((lab >> 0) & 1) << (7 - j));
                g |= (((lab >> 1) & 1) << (7 - j));
                b |= (((lab >> 2) & 1) << (7 - j));
                ++j;
                lab >>= 3;
            }

            yield return new Color(r, g, b).ToColor4();
        }
    }
}
