using FlysEngine.Desktop;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Windows.Forms;
using Vortice.Direct2D1;
using Vortice.DXGI;
using Vortice.Mathematics;
using Vortice.UIAnimation;

namespace Sdcb.NBody.Direct2D;

class NBodyWindow : RenderWindow
{
    IEnumerator<SystemSnapshot> _system;
    SystemSnapshot _lastSnapshot;
    BodyUIProps[] _uiProps;
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
        _uiProps = new BodyUIProps[_lastSnapshot.Bodies.Length];
        foreach ((Color4 color, int i) in GenerateColorMap().Take(_uiProps.Length).Select((color, i) => (color, i)))
        {
            _uiProps[i] = new()
            {
                Color = color
            };
        }
        _scale = XResource.CreateAnimation(ClientSize.Height * initialScale, ClientSize.Height * initialScale, 0);
        _stroke = XResource.Direct2DFactory.CreateStrokeStyle(new StrokeStyleProperties1 { StartCap = CapStyle.Triangle, EndCap = CapStyle.Triangle });
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
                _system.MoveNext();
                _acc += _system.Current.Timestamp - _lastSnapshot.Timestamp;

                for (int i = 0; i < _system.Current.Bodies.Length; ++i)
                {
                    BodySnapshot star = _system.Current.Bodies[i];
                    BodyUIProps props = _uiProps[i];

                    Vector2 now = new(star.Px, star.Py);
                    if (props.TrackingHistory.Count > 0)
                    {
                        Vector2 old = props.TrackingHistory.Last;
                        float dist = Vector2.Distance(old, now);
                        if (dist > 2 / _scale.Value)
                        {
                            props.TrackingHistory.Add(now);
                        }
                    }
                    else
                    {
                        props.TrackingHistory.Add(now);
                    }
                }
            }
        }
        catch (OperationCanceledException) // device lose
        {
        }

        if (_acc >= unit) _acc -= unit;

        _lastSnapshot = _system.Current;
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

        // draw paths first
        for (int i = 0; i < _lastSnapshot.Bodies.Length; ++i)
        {
            BodySnapshot star = _lastSnapshot.Bodies[i];
            BodyUIProps prop = _uiProps[i];

            if (prop.TrackingHistory.Count < 2) continue;

            using ID2D1PathGeometry1 path = XResource.Direct2DFactory.CreatePathGeometry();
            using ID2D1GeometrySink sink = path.Open();
            sink.BeginFigure(prop.TrackingHistory.First, FigureBegin.Hollow);
            foreach ((int index, Vector2 pt) in prop.TrackingHistory.Index())
            {
                if (index > 0) { sink.AddLine(pt); }
            }
            sink.EndFigure(FigureEnd.Open);
            sink.Close();
            ctx.DrawGeometry(path, XResource.GetColor(prop.Color), 0.02f);
        }

        for (int i = 0; i < _lastSnapshot.Bodies.Length; ++i)
        {
            BodySnapshot star = _lastSnapshot.Bodies[i];
            BodyUIProps prop = _uiProps[i];
            using ID2D1GradientStopCollection collection = ctx.CreateGradientStopCollection(new[]
            {
                new GradientStop{ Color = Colors.White, Position = 0 },
                new GradientStop{ Color = star.BodyType switch
                {
                    BodyType.Solar => prop.Color,
                    BodyType.BlackHole => Colors.Black,
                    _ => Colors.Gray,
                }, Position = 1 },
            });
            using ID2D1RadialGradientBrush radialBrush = ctx.CreateRadialGradientBrush(new RadialGradientBrushProperties
            {
                Center = new Vector2(star.Px, star.Py),
                RadiusX = star.Size,
                RadiusY = star.Size,
            }, collection);

            ctx.FillEllipse(new Ellipse(new Vector2(star.Px, star.Py), star.Size, star.Size), radialBrush);
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
