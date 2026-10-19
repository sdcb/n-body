using System.Diagnostics;
using System.Numerics;
using Sdcb.NBody.App.Audio;
using Sdcb.NBody.App.Camera;
using Sdcb.NBody.App.Graphics;
using Sdcb.NBody.App.Hud;
using Sdcb.NBody.App.Platform;
using Sdcb.NBody.App.Rendering;
using Sdcb.NBody.App.Simulation;
using Sdcb.NBody.App.Util;
using Sdcb.NBody.Scenarios;
using Vortice.Mathematics;
using Sim = Sdcb.NBody.Simulation;

namespace Sdcb.NBody.App;

internal sealed class NBodyApp : IDisposable
{
    private readonly AppWindow _window;
    private readonly GraphicsDevice _gfx;
    private readonly SceneRenderer _renderer;
    private readonly HudCanvas _canvas;
    private readonly Hud.Hud _hud;
    private readonly AmbientAudio _audio;
    private readonly QualityTier _tier;
    private readonly OrbitCamera _camera = new();
    private readonly RenderSettings _settings = new();
    private readonly HudModel _model = new();
    private readonly Stopwatch _clock = Stopwatch.StartNew();

    // scenario state
    private int _scenarioIndex = -1;
    private ScenarioSetup _setup = null!;
    private CpuSimHost? _cpu;
    private bool _gpuScene;
    private double _gpuAccumulator;
    private double _speed = 1.0;
    private bool _paused;
    private int _pendingIndex = -1;
    private float _fade = 0f, _titleTimer = 99;

    // attract mode
    private bool _attract = true;
    private double _attractTimer, _idleTimer, _attractElapsed;
    private Vector2 _attractMouseOrigin;
    private bool _attractMouseInit;
    private int _attractCursor;
    private static readonly string[] AttractPlaylist =
        ["galaxy-collision", "figure-8", "protoplanetary-disk", "butterfly-1", "solar-system", "chaos-twins", "spiral-galaxy", "moth-1", "milky-way-andromeda", "ring-6", "cold-collapse", "binary-star-planets"];

    // input
    private Vector2 _mouse;
    private bool _orbiting, _panning, _pressOnUi, _launching;
    private Vector2 _lastDrag;
    private Vector3 _launchStart;
    private Vector2 _launchStartScreen;
    private int _followIndex = -1;  // index into CPU bodies, or into GPU specials
    private bool _autoFrame = true;  // GPU scenes: keep the nuclei framed until the user takes the camera
    private string? _toast;
    private float _toastTimer;
    private bool _screenshotRequested;

    // per-frame scratch
    private TrailPoint[] _trailScratch = new TrailPoint[4096];
    private BodyInstance[] _bodyScratch = new BodyInstance[64];
    private readonly List<GlassPanel> _lastPanels = [];
    private double _fps, _frameMs;
    private float _time;

    private readonly AppOptions _options;
    private readonly bool _diag = Environment.GetEnvironmentVariable("NBODY_DIAG") == "1";

    public NBodyApp(AppOptions options)
    {
        _options = options;
        _window = new AppWindow("N·Body — Gravitational Symphony", !options.Windowed, options.Width, options.Height);
        _gfx = new GraphicsDevice(_window.Handle, _window.Width, _window.Height);
        // present a dark splash before the window becomes visible (no white flash while shaders load)
        DrawSplash();
        _window.Show();
        _window.PumpMessages();
        _tier = QualityTier.Detect(_gfx);
        _renderer = new SceneRenderer(_gfx, _tier.BloomLevels);
        _renderer.Resize(_gfx.Width, _gfx.Height);
        _canvas = new HudCanvas(_gfx.D2D, _gfx.DWriteFactory, _gfx.D2DFactory);
        _hud = new Hud.Hud(_canvas);
        _audio = new AmbientAudio();
        _model.Adapter = $"{_gfx.AdapterName} · {_tier.Name}";

        _window.Resized += (w, h) =>
        {
            _gfx.Resize(w, h);
            _renderer.Resize(_gfx.Width, _gfx.Height);
        };
        _window.KeyDown += OnKeyDown;
        _window.MouseDown += OnMouseDown;
        _window.MouseUp += OnMouseUp;
        _window.MouseMove += OnMouseMove;
        _window.MouseWheel += OnMouseWheel;
        _window.MouseDoubleClick += OnDoubleClick;

        int start = ScenarioRegistry.IndexOf(options.Scenario ?? AttractPlaylist[0]);
        if (options.NoAttract) _attract = false;
        LoadScenario(start >= 0 ? start : 0);
        _fade = 0;
    }

    private void DrawSplash()
    {
        _gfx.Context.ClearRenderTargetView(_gfx.BackBufferRtv, new Color4(0.008f, 0.009f, 0.014f, 1));
        float scale = _window.DpiScale;
        Vortice.Direct2D1.ID2D1DeviceContext dc = _gfx.D2D;
        dc.BeginDraw();
        dc.SetDpi(96 * scale, 96 * scale);
        float w = _gfx.Width / scale, h = _gfx.Height / scale;
        using Vortice.DirectWrite.IDWriteTextFormat fmt = _gfx.DWriteFactory.CreateTextFormat("Segoe UI",
            Vortice.DirectWrite.FontWeight.SemiLight, Vortice.DirectWrite.FontStyle.Normal, Vortice.DirectWrite.FontStretch.Normal, 22);
        fmt.TextAlignment = Vortice.DirectWrite.TextAlignment.Center;
        fmt.ParagraphAlignment = Vortice.DirectWrite.ParagraphAlignment.Center;
        using Vortice.Direct2D1.ID2D1SolidColorBrush brush = dc.CreateSolidColorBrush(new Color4(0.45f, 0.85f, 1f, 0.55f));
        dc.DrawText("N · B O D Y", fmt, new Rect(0, 0, w, h), brush);
        dc.EndDraw();
        _gfx.Present(vsync: false);
    }

    public void Run()
    {
        double last = _clock.Elapsed.TotalSeconds;
        while (_window.PumpMessages())
        {
            if (_window.IsMinimized)
            {
                Thread.Sleep(30);
                last = _clock.Elapsed.TotalSeconds;
                continue;
            }
            double now = _clock.Elapsed.TotalSeconds;
            float dt = (float)Math.Clamp(now - last, 1e-4, 0.1);
            last = now;
            Frame(dt);
        }
    }

    // ───────────────────────────── scenario management ─────────────────────────────

    private void RequestScenario(int index)
    {
        int n = ScenarioRegistry.All.Count;
        index = ((index % n) + n) % n;
        if (_pendingIndex >= 0 || index == _scenarioIndex) return;
        _pendingIndex = index;
        _audio.PlayWhoosh();
    }

    private void LoadScenario(int index)
    {
        _cpu?.Dispose();
        _cpu = null;
        _scenarioIndex = index;
        Scenario sc = ScenarioRegistry.All[index];
        _setup = sc.Create(_tier.ParticleBudget);
        _gpuScene = _setup.GpuSimulated;
        _gpuAccumulator = 0;
        _followIndex = -1;
        _autoFrame = true;
        _speed = 1.0;
        if (_gpuScene)
        {
            _renderer.GpuSim.Load(_setup.Bodies, _setup.G, _setup.Softening, _setup.Halos, _setup.HaloFriction);
        }
        else
        {
            _renderer.GpuSim.Load(new BodySet(1), 1, 0);
            _cpu = new CpuSimHost(_setup);
        }
        ConfigureLook();

        float vr = (float)_setup.ViewRadius;
        _camera.GoalTarget = Vector3.Zero;
        _camera.GoalDistance = vr / MathF.Tan(_camera.FovY / 2) * 1.12f;
        _camera.GoalPitch = _setup.CameraPitchDegrees * MathF.PI / 180f;
        if (!_attract) _camera.GoalYaw = -MathF.PI / 2;
        _camera.SnapToGoal();
        _camera.Distance *= 1.5f;   // gentle dolly-in
        _camera.Pitch += 0.25f;
        _titleTimer = 0;
        _attractTimer = 0;
    }

    private void ConfigureLook()
    {
        float vr = (float)_setup.ViewRadius;
        ScenarioCategory cat = ScenarioRegistry.All[_scenarioIndex].Info.Category;
        _settings.ParticleRadius = vr * 0.0028f;
        int n = Math.Max(1, _setup.Bodies.Count);
        _settings.ParticleBrightness = Math.Clamp(0.4f * MathF.Sqrt(32768f / n), 0.2f, 1.4f);
        _settings.ParticleStreak = 0;
        _settings.StarGlow = cat == ScenarioCategory.SolarSystem ? 0.8f : 1f;
        _settings.TrailIntensity = cat switch
        {
            ScenarioCategory.SolarSystem => 1.1f,
            ScenarioCategory.Choreography => 1.3f,
            _ => 1.6f,
        };
        _settings.BloomStrength = _gpuScene ? 0.16f : 0.12f;
        _settings.Exposure = _gpuScene ? 1.1f : 1.0f;
        Random rng = new(_scenarioIndex * 7919 + 3);
        Vector3[] tintsA = [new(0.10f, 0.05f, 0.22f), new(0.18f, 0.06f, 0.10f), new(0.04f, 0.10f, 0.18f), new(0.12f, 0.08f, 0.02f)];
        Vector3[] tintsB = [new(0.02f, 0.14f, 0.20f), new(0.04f, 0.06f, 0.20f), new(0.16f, 0.04f, 0.14f), new(0.02f, 0.12f, 0.12f)];
        _settings.Sky.NebulaTintA = tintsA[rng.Next(tintsA.Length)];
        _settings.Sky.NebulaTintB = tintsB[rng.Next(tintsB.Length)];
        _settings.Sky.Seed = (float)(rng.NextDouble() * 100);
        _settings.Sky.BandNormal = Vector3.Normalize(new Vector3((float)rng.NextDouble() - 0.5f, (float)rng.NextDouble() - 0.5f, 1.2f));
        _settings.Sky.NebulaIntensity = 0.55f;
    }

    // ───────────────────────────── frame ─────────────────────────────

    private void Frame(float dt)
    {
        _time += dt;
        _fps += (1.0 / dt - _fps) * Math.Min(1, dt * 3);
        _frameMs += (dt * 1000 - _frameMs) * Math.Min(1, dt * 3);

        UpdateTransitions(dt);
        UpdateAttract(dt);

        // 1) collect last physics job (CPU) – state is now safe to read/modify
        _cpu?.Wait();

        // 2) follow target / camera
        UpdateFollow();
        float aspect = _gfx.Width / (float)Math.Max(1, _gfx.Height);
        _camera.Smoothing = _attract ? 1.6f : 7f;
        _camera.Update(dt, aspect);

        // 3) build render data from the current state
        BuildScene(out int trailCount, out int bodyCount, out Vector3? light);

        // 4) kick next physics step (runs in parallel with rendering)
        double simDt = _paused || _pendingIndex >= 0 ? 0 : dt * _setup.TimeScale * _speed;
        if (_cpu != null) _cpu.Kick(simDt);

        // GPU physics
        if (_gpuScene)
        {
            int steps = 0;
            if (simDt > 0)
            {
                _gpuAccumulator += simDt;
                steps = (int)(_gpuAccumulator / _setup.Dt);
                long n = _renderer.GpuSim.PaddedCount;
                double budget = (double)_tier.ParticleBudget * _tier.ParticleBudget * _tier.MaxGpuStepsPerFrame;
                int maxSteps = (int)Math.Clamp(budget / Math.Max(1.0, (double)n * n), 1, 64);
                if (steps > maxSteps) { steps = maxSteps; _gpuAccumulator = 0; }
                else _gpuAccumulator -= steps * _setup.Dt;
            }
            _renderer.GpuSim.Step(_gfx.Context, _setup.Dt, steps);
            if (_diag && (int)(_time * 2) != (int)((_time - dt) * 2))
            {
                string log = Path.Combine(AppContext.BaseDirectory, "nbody-diag.log");
                File.AppendAllText(log, _renderer.GpuSim.Stats(_gfx.Context) + Environment.NewLine);
                if (Environment.GetEnvironmentVariable("NBODY_DIAG_HDR") == "1") _renderer.PassProbe = pass => File.AppendAllText(log, $"  {pass}: {_renderer.ProbeHdr()}{Environment.NewLine}");
            }
            else
            {
                _renderer.PassProbe = null;
            }
        }

        // 5) render scene
        FrameConstants fc = BuildFrameConstants(light);
        _settings.Fade = _fade;
        if (Environment.GetEnvironmentVariable("NBODY_DIAG_FRAMES") == "1")
        {
            string log = Path.Combine(AppContext.BaseDirectory, "nbody-frames.log");
            _renderer.PassProbe = pass => { if (pass == "bodies") File.AppendAllText(log, $"{_clock.Elapsed.TotalSeconds:F3} fade={_fade:F2} cam={_camera.Distance:F2} {_renderer.ProbeHdr()}{Environment.NewLine}"); };
        }
        _renderer.Render(fc, _settings, _trailScratch.AsSpan(0, trailCount), _bodyScratch.AsSpan(0, bodyCount), _gpuScene, _lastPanels);

        // 6) HUD
        FillModel(dt);
        float scale = _window.DpiScale;
        _canvas.Scale = scale;
        _canvas.Width = _gfx.Width / scale;
        _canvas.Height = _gfx.Height / scale;
        _canvas.BeginFrame();
        _gfx.D2D.BeginDraw();
        _gfx.D2D.SetDpi(96 * scale, 96 * scale);
        HudAction action = _hud.Draw(_model, dt);
        _gfx.D2D.EndDraw();
        _lastPanels.Clear();
        _lastPanels.AddRange(_canvas.Panels);
        _window.HandCursor = _canvas.AnyHot;
        _canvas.EndFrame();
        HandleAction(action);

        if (_options.CapturePath != null && _clock.Elapsed.TotalSeconds >= _options.CaptureAt)
        {
            Screenshot.Save(_gfx.Device, _gfx.Context, _gfx.BackBuffer, Path.GetFullPath(_options.CapturePath));
            _window.Close();
        }

        if (_screenshotRequested)
        {
            _screenshotRequested = false;
            try
            {
                string path = Screenshot.Capture(_gfx.Device, _gfx.Context, _gfx.BackBuffer);
                Toast($"{Loc.T("screenshot.saved")} · {Path.GetFileName(path)}");
            }
            catch (Exception ex)
            {
                Toast(ex.Message);
            }
        }

        _gfx.Present(vsync: true);

        _audio.Update(dt, _gpuScene ? 0.6f : (float)Math.Clamp(0.25 + 0.15 * Math.Log10(Math.Max(1, _speed * 10)), 0, 1) * (_paused ? 0.3f : 1f));
    }

    private void UpdateTransitions(float dt)
    {
        if (_pendingIndex >= 0)
        {
            _fade = MathF.Max(0, _fade - dt * 3.2f);
            if (_fade <= 0)
            {
                int idx = _pendingIndex;
                _pendingIndex = -1;
                LoadScenario(idx);
            }
        }
        else
        {
            _fade = MathF.Min(1, _fade + dt * 1.2f);
        }
        _titleTimer += dt;
        if (_toastTimer > 0) _toastTimer -= dt;
    }

    private void UpdateAttract(float dt)
    {
        _idleTimer += dt;
        _attractElapsed += dt;
        if (!_attract)
        {
            if (_idleTimer > 120 && !_paused) EnterAttract();
            return;
        }
        _attractTimer += dt;
        _camera.GoalYaw += dt * 0.07f;
        float basePitch = _setup.CameraPitchDegrees * MathF.PI / 180f;
        _camera.GoalPitch = basePitch + 0.18f * MathF.Sin(_time * 0.11f);
        float vr = AutoFrameRadius();
        _camera.GoalDistance = vr / MathF.Tan(_camera.FovY / 2) * (1.05f + 0.12f * MathF.Sin(_time * 0.07f));
        double dwell = _gpuScene ? 40 : 30;
        if (_attractTimer > dwell && _pendingIndex < 0)
        {
            _attractCursor = (_attractCursor + 1) % AttractPlaylist.Length;
            int idx = ScenarioRegistry.IndexOf(AttractPlaylist[_attractCursor]);
            RequestScenario(idx >= 0 ? idx : _scenarioIndex + 1);
            _attractTimer = 0;
        }
    }

    private void EnterAttract()
    {
        _attract = true;
        _attractMouseInit = false;
        _attractTimer = 0;
        _model.Gallery = false;
        _model.Help = false;
    }

    private void ExitAttract()
    {
        _idleTimer = 0;
        if (!_attract || _attractElapsed < 1.2) return;
        _attract = false;
    }

    /// <summary>GPU galaxy scenes: radius that keeps all galactic nuclei (plus their disks) in view.</summary>
    private float AutoFrameRadius()
    {
        float vr = (float)_setup.ViewRadius;
        if (!_gpuScene || _renderer.GpuSim.SpecialCount < 2) return vr;
        Vector3[] sp = _renderer.GpuSim.SpecialPositions;
        Vector3 mean = AutoFrameCenter();
        float extent = 0;
        foreach (Vector3 p in sp) extent = MathF.Max(extent, Vector3.Distance(p, mean));
        return MathF.Max(vr, extent + vr * 0.6f);
    }

    private Vector3 AutoFrameCenter()
    {
        Vector3[] sp = _renderer.GpuSim.SpecialPositions;
        if (!_gpuScene || sp.Length < 2) return Vector3.Zero;
        Vector3 sum = Vector3.Zero;
        foreach (Vector3 p in sp) sum += p;
        return sum / sp.Length;
    }

    private void UpdateFollow()
    {
        if (_followIndex < 0)
        {
            if (_autoFrame && _gpuScene)
            {
                _camera.GoalTarget = AutoFrameCenter();
                if (!_attract) _camera.GoalDistance = AutoFrameRadius() / MathF.Tan(_camera.FovY / 2) * 1.12f;
            }
            return;
        }
        Vector3? p = FollowPosition();
        if (p is Vector3 v) _camera.GoalTarget = v;
        else _followIndex = -1;
    }

    private Vector3? FollowPosition()
    {
        if (_gpuScene)
        {
            Vector3[] sp = _renderer.GpuSim.SpecialPositions;
            return _followIndex < sp.Length ? sp[_followIndex] : null;
        }
        if (_cpu == null) return null;
        BodySet b = _cpu.Main.Bodies;
        return _followIndex < b.Count ? new Vector3((float)b.Px[_followIndex], (float)b.Py[_followIndex], (float)b.Pz[_followIndex]) : null;
    }

    private string? FollowName()
    {
        if (_followIndex < 0) return null;
        if (_gpuScene)
        {
            BodyKind[] k = _renderer.GpuSim.SpecialKinds;
            return _followIndex < k.Length ? $"{k[_followIndex]} #{_followIndex + 1}" : null;
        }
        if (_cpu == null || _followIndex >= _cpu.Main.Bodies.Count) return null;
        return _cpu.Main.Bodies.Name[_followIndex] ?? $"#{_followIndex + 1}";
    }

    // ───────────────────────────── scene building ─────────────────────────────

    private void BuildScene(out int trailCount, out int bodyCount, out Vector3? light)
    {
        trailCount = 0;
        bodyCount = 0;
        light = null;
        if (_cpu == null)
        {
            if (_gpuScene)
            {
                Vector3[] sp = _renderer.GpuSim.SpecialPositions;
                BodyKind[] kinds = _renderer.GpuSim.SpecialKinds;
                for (int i = 0; i < sp.Length; ++i) if (kinds[i] == BodyKind.Star) { light = sp[i]; break; }
            }
            return;
        }

        AppendBodies(_cpu.Main, _cpu.Trails, twin: false, ref trailCount, ref bodyCount);
        if (_cpu.Twin != null) AppendBodies(_cpu.Twin, _cpu.TwinTrails, twin: true, ref trailCount, ref bodyCount);

        // light source: most massive star if the scene has planets
        BodySet b = _cpu.Main.Bodies;
        int star = -1;
        bool planets = false;
        for (int i = 0; i < b.Count; ++i)
        {
            if (b.Kind[i] == BodyKind.Star && (star < 0 || b.Mass[i] > b.Mass[star])) star = i;
            if (b.Kind[i] is BodyKind.Planet or BodyKind.Moon) planets = true;
        }
        if (star >= 0 && planets) light = new Vector3((float)b.Px[star], (float)b.Py[star], (float)b.Pz[star]);

        // back-to-front for premultiplied blending
        Vector3 cam = _camera.Position;
        Array.Sort(_bodyScratch, 0, bodyCount, Comparer<BodyInstance>.Create((x, y) =>
            Vector3.DistanceSquared(y.Pos, cam).CompareTo(Vector3.DistanceSquared(x.Pos, cam))));
    }

    private void AppendBodies(Sim sim, List<TrailRing> trails, bool twin, ref int trailCount, ref int bodyCount)
    {
        BodySet b = sim.Bodies;
        double now = sim.Time;
        int n = Math.Min(b.Count, trails.Count);
        if (_bodyScratch.Length < bodyCount + b.Count) Array.Resize(ref _bodyScratch, (bodyCount + b.Count) * 2);

        for (int i = 0; i < b.Count; ++i)
        {
            Rgb c = b.Color[i];
            _bodyScratch[bodyCount++] = new BodyInstance
            {
                Pos = new((float)b.Px[i], (float)b.Py[i], (float)b.Pz[i]),
                Radius = b.Radius[i] * (twin ? 0.85f : 1f),
                Color = new(c.R, c.G, c.B),
                Kind = (uint)b.Kind[i],
            };
        }

        if (!_settings.ShowTrails || _setup.TrailDuration <= 0) return;
        for (int i = 0; i < n; ++i)
        {
            TrailRing ring = trails[i];
            int needed = trailCount + ring.Count + 1;
            if (_trailScratch.Length < needed) Array.Resize(ref _trailScratch, (int)BitOperations.RoundUpToPowerOf2((uint)needed));
            Rgb c = b.Color[i];
            Vector3 col = new(c.R, c.G, c.B);
            float width = b.Kind[i] switch
            {
                BodyKind.Star => 2.4f,
                BodyKind.BlackHole => 2.6f,
                BodyKind.Planet => 1.7f,
                BodyKind.Moon => 1.1f,
                _ => 0.8f,
            } * (twin ? 0.8f : 1f);
            double dur = ring.Duration;
            bool first = true;
            for (int k = 0; k < ring.Count; ++k)
            {
                var (p, t) = ring[k];
                double age = now - t;
                if (age > dur) continue;
                float fade = (float)(1 - age / dur);
                _trailScratch[trailCount++] = new TrailPoint { Pos = p, Fade = fade, Color = col, Width = first ? -width : width };
                first = false;
            }
            // live head
            _trailScratch[trailCount++] = new TrailPoint
            {
                Pos = new((float)b.Px[i], (float)b.Py[i], (float)b.Pz[i]),
                Fade = 1,
                Color = col,
                Width = first ? -width : width,
            };
        }
    }

    private FrameConstants BuildFrameConstants(Vector3? light)
    {
        Matrix4x4 vp = _camera.View * _camera.Proj;
        if (!Matrix4x4.Invert(vp, out Matrix4x4 inv)) inv = Matrix4x4.Identity;
        return new FrameConstants
        {
            View = _camera.View,
            Proj = _camera.Proj,
            ViewProj = vp,
            InvViewProj = inv,
            CamPos = _camera.Position,
            Time = _time,
            CamRight = _camera.Right,
            PixelsPerUnit = _camera.Proj.M22 * _gfx.Height * 0.5f,
            CamUp = _camera.Up,
            DpiScale = _window.DpiScale,
            CamForward = _camera.Forward,
            SceneScale = (float)_setup.ViewRadius,
            Screen = new Vector2(_gfx.Width, _gfx.Height),
            InvScreen = new Vector2(1f / _gfx.Width, 1f / _gfx.Height),
            LightPos = light ?? Vector3.Zero,
            HasLight = light.HasValue ? 1 : 0,
        };
    }

    // ───────────────────────────── HUD ─────────────────────────────

    private void FillModel(float dt)
    {
        Scenario sc = ScenarioRegistry.All[_scenarioIndex];
        HudModel m = _model;
        m.Info = sc.Info;
        m.ScenarioIndex = _scenarioIndex;
        m.ScenarioCount = ScenarioRegistry.All.Count;
        m.Attract = _attract;
        m.Paused = _paused;
        m.Gpu = _gpuScene;
        m.Trails = _settings.ShowTrails;
        m.Muted = _audio.Muted;
        m.Fps = _fps;
        m.FrameMs = _frameMs;
        m.TimeScale = _speed;
        m.Period = _setup.Period;
        m.Sandbox = sc.Info.Category == ScenarioCategory.Sandbox && sc.Info.Id == "sandbox";
        m.FollowName = FollowName();
        m.Labels.Clear();
        if (_cpu != null)
        {
            Sim s = _cpu.Main;
            m.SimTime = s.Time;
            m.Bodies = s.Bodies.Count + (_cpu.Twin?.Bodies.Count ?? 0);
            m.Integrator = Loc.Integrator(s.Integrator.Kind);
            m.EnergyError = s.RelativeEnergyError;
            m.EnergyHistory = _cpu.EnergyHistory;
            m.EnergyHistoryCount = _cpu.EnergyHistoryCount;
            m.Throttled = _cpu.Throttled;
            m.ChaosSeparation = _cpu.Twin != null ? Separation(s.Bodies, _cpu.Twin.Bodies) : null;

            BodySet b = s.Bodies;
            float scale = _window.DpiScale;
            for (int i = 0; i < b.Count; ++i)
            {
                if (b.Name[i] is not string name || b.Count > 16) continue;
                Vector3 p = new((float)b.Px[i], (float)b.Py[i], (float)b.Pz[i]);
                if (!_camera.WorldToScreen(p, _gfx.Width, _gfx.Height, out Vector2 sp)) continue;
                float depth = Vector3.Dot(p - _camera.Position, _camera.Forward);
                float rpx = MathF.Max(b.Radius[i] * _camera.Proj.M22 * _gfx.Height * 0.5f / MathF.Max(depth, 1e-5f), 2.5f * scale) / scale;
                Rgb c = b.Color[i];
                m.Labels.Add(new BodyLabel(sp / scale, LocalizeBodyName(name), new Color4(c.R, c.G, c.B, 1), rpx));
            }
        }
        else
        {
            m.SimTime = _renderer.GpuSim.Time;
            m.Bodies = _renderer.GpuSim.Count;
            m.EnergyHistoryCount = 0;
            m.Throttled = false;
            m.ChaosSeparation = null;
        }
        if (!m.Sandbox || !_launching) { m.DragStart = null; m.DragEnd = null; }

        // title card envelope
        float t = _titleTimer;
        float card = t < 0.5f ? t / 0.5f : t < 2.4f ? 1 : t < 3.2f ? 1 - (t - 2.4f) / 0.8f : 0;
        m.TitleCard = card * _fade * (_attract ? 1f : 0.85f);
        m.Toast = _toast;
        m.ToastAlpha = Math.Clamp(_toastTimer / 0.4f, 0, 1);
    }

    private static string LocalizeBodyName(string name) => Loc.Current == Lang.En ? name : name switch
    {
        "Sun" => "太阳", "Mercury" => "水星", "Venus" => "金星", "Earth" => "地球", "Moon" => "月球", "Mars" => "火星",
        "Jupiter" => "木星", "Saturn" => "土星", "Uranus" => "天王星", "Neptune" => "海王星", "Star" => "恒星",
        _ => name,
    };

    private static double Separation(BodySet a, BodySet b)
    {
        int n = Math.Min(a.Count, b.Count);
        double s = 0;
        for (int i = 0; i < n; ++i)
        {
            double dx = a.Px[i] - b.Px[i], dy = a.Py[i] - b.Py[i], dz = a.Pz[i] - b.Pz[i];
            double dvx = a.Vx[i] - b.Vx[i], dvy = a.Vy[i] - b.Vy[i], dvz = a.Vz[i] - b.Vz[i];
            s += dx * dx + dy * dy + dz * dz + dvx * dvx + dvy * dvy + dvz * dvz;
        }
        return Math.Sqrt(s);
    }

    private void Toast(string text)
    {
        _toast = text;
        _toastTimer = 2.4f;
    }

    private void HandleAction(HudAction a)
    {
        switch (a)
        {
            case HudAction.None: return;
            case HudAction.Prev: RequestScenario(_scenarioIndex - 1); break;
            case HudAction.Next: RequestScenario(_scenarioIndex + 1); break;
            case HudAction.TogglePause: _paused = !_paused; break;
            case HudAction.Faster: _speed = Math.Min(_speed * 1.5, 200); break;
            case HudAction.Slower: _speed = Math.Max(_speed / 1.5, 0.01); break;
            case HudAction.ToggleTrails: _settings.ShowTrails = !_settings.ShowTrails; break;
            case HudAction.ToggleGallery: _model.Gallery = !_model.Gallery; break;
            case HudAction.ToggleHelp: _model.Help = !_model.Help; break;
            case HudAction.ToggleLang: Loc.Current = Loc.Current == Lang.Zh ? Lang.En : Lang.Zh; break;
            case HudAction.Reset: ResetScenario(); break;
            case HudAction.CycleIntegrator: CycleIntegrator(); break;
            case HudAction.ToggleMute:
                _audio.Muted = !_audio.Muted;
                Toast(Loc.T(_audio.Muted ? "muted" : "unmuted"));
                break;
            case HudAction.Screenshot: _screenshotRequested = true; break;
            case HudAction.Fullscreen: _window.SetFullscreen(!_window.IsFullscreen); break;
            case HudAction.Select:
                RequestScenario(_hud.SelectedIndex);
                break;
            case HudAction.Quit: _window.Close(); break;
        }
    }

    private void ResetScenario()
    {
        if (_gpuScene)
        {
            _renderer.GpuSim.Load(_setup.Bodies, _setup.G, _setup.Softening, _setup.Halos, _setup.HaloFriction);
            _gpuAccumulator = 0;
        }
        else
        {
            _cpu?.ResetSimulation();
        }
        _followIndex = -1;
    }

    private void CycleIntegrator()
    {
        if (_cpu == null) return;
        _cpu.Wait();
        IntegratorKind next = (IntegratorKind)(((int)_cpu.Main.Integrator.Kind + 1) % Enum.GetValues<IntegratorKind>().Length);
        _cpu.Main.Integrator = Integrators.Create(next);
        if (_cpu.Twin != null) _cpu.Twin.Integrator = Integrators.Create(next);
        _cpu.Main.RefreshDiagnostics();
        Toast($"{Loc.T("integrator")}: {Loc.Integrator(next)}");
    }

    // ───────────────────────────── input ─────────────────────────────

    private void OnKeyDown(int vk, bool repeat)
    {
        bool wasAttract = _attract;
        ExitAttract();
        if (wasAttract && !_attract && vk != 0x1B) return; // first key only wakes the UI
        switch (vk)
        {
            case 0x20: HandleAction(HudAction.TogglePause); break;              // Space
            case 0x25: HandleAction(HudAction.Prev); break;                     // Left
            case 0x27: HandleAction(HudAction.Next); break;                     // Right
            case 0x26: HandleAction(HudAction.Faster); break;                   // Up
            case 0x28: HandleAction(HudAction.Slower); break;                   // Down
            case 0x09: _model.ShowHud = !_model.ShowHud; break;                 // Tab
            case 0x1B:                                                          // Esc
                if (_model.Help || _model.Gallery) { _model.Help = false; _model.Gallery = false; }
                else _window.Close();
                break;
            case 0x7A: HandleAction(HudAction.Fullscreen); break;               // F11
            case 0x7B: HandleAction(HudAction.Screenshot); break;               // F12
            case 'G': HandleAction(HudAction.ToggleGallery); break;
            case 'H': HandleAction(HudAction.ToggleHelp); break;
            case 'T': HandleAction(HudAction.ToggleTrails); break;
            case 'R': HandleAction(HudAction.Reset); break;
            case 'L': HandleAction(HudAction.ToggleLang); break;
            case 'M': HandleAction(HudAction.ToggleMute); break;
            case 'I': if (!repeat) HandleAction(HudAction.CycleIntegrator); break;
            case 'C': if (_model.Sandbox) ResetScenario(); break;
            case 'F': _followIndex = -1; _autoFrame = true; _camera.GoalTarget = Vector3.Zero; break;
        }
    }

    private Vector2 ToDip(int x, int y) => new Vector2(x, y) / _window.DpiScale;

    private void OnMouseDown(MouseButton button, int x, int y)
    {
        ExitAttract();
        _mouse = new Vector2(x, y);
        _canvas.Mouse = ToDip(x, y);
        if (button == MouseButton.Left && _canvas.MouseOverUi)
        {
            _pressOnUi = true;
            _canvas.MouseDown = true;
            _canvas.MousePressed = true;
            return;
        }
        _pressOnUi = false;
        _lastDrag = _mouse;
        if (button == MouseButton.Left && _model.Sandbox && AppWindow.IsKeyDown(0x10) && _cpu != null)
        {
            if (PickPlane(x, y, out Vector3 p))
            {
                _launching = true;
                _launchStart = p;
                _launchStartScreen = ToDip(x, y);
                _model.DragStart = _launchStartScreen;
                _model.DragEnd = _launchStartScreen;
            }
            return;
        }
        if (button == MouseButton.Left) _orbiting = true;
        else _panning = true;
    }

    private void OnMouseUp(MouseButton button, int x, int y)
    {
        _canvas.Mouse = ToDip(x, y);
        if (_pressOnUi && button == MouseButton.Left)
        {
            _canvas.MouseDown = false;
            _canvas.MouseReleased = true;
            _pressOnUi = false;
            return;
        }
        if (_launching && button == MouseButton.Left)
        {
            _launching = false;
            if (PickPlane(x, y, out Vector3 end)) LaunchBody(_launchStart, end);
            _model.DragStart = null;
            _model.DragEnd = null;
        }
        if (button == MouseButton.Left) _orbiting = false;
        else _panning = false;
    }

    private void OnMouseMove(int x, int y)
    {
        Vector2 p = new(x, y);
        _canvas.Mouse = ToDip(x, y);
        if (_attract)
        {
            if (!_attractMouseInit) { _attractMouseOrigin = p; _attractMouseInit = true; }
            else if (Vector2.Distance(p, _attractMouseOrigin) > 40 * _window.DpiScale) ExitAttract();
        }
        else
        {
            _idleTimer = 0;
        }
        Vector2 d = p - _lastDrag;
        _lastDrag = p;
        if (_orbiting) _camera.Orbit(-d.X * 0.006f, d.Y * 0.006f);
        if (_panning) { _camera.Pan(d.X, d.Y, _gfx.Height); _followIndex = -1; _autoFrame = false; }
        if (_launching) _model.DragEnd = ToDip(x, y);
        _mouse = p;
    }

    private void OnMouseWheel(float delta, int x, int y)
    {
        ExitAttract();
        if (_canvas.MouseOverUi && _model.Gallery)
        {
            _canvas.Wheel += delta;
            return;
        }
        _camera.Zoom(MathF.Pow(0.85f, delta));
        _autoFrame = false;
    }

    private void OnDoubleClick(int x, int y)
    {
        if (_canvas.MouseOverUi) return;
        // pick nearest body on screen
        int best = -1;
        float bestD = 40 * _window.DpiScale;
        if (_gpuScene)
        {
            Vector3[] sp = _renderer.GpuSim.SpecialPositions;
            for (int i = 0; i < sp.Length; ++i) Consider(sp[i], i);
        }
        else if (_cpu != null)
        {
            BodySet b = _cpu.Main.Bodies;
            for (int i = 0; i < b.Count; ++i) Consider(new Vector3((float)b.Px[i], (float)b.Py[i], (float)b.Pz[i]), i);
        }
        if (best >= 0 && best != _followIndex)
        {
            _followIndex = best;
            if (FollowName() is string n) Toast($"{Loc.T("follow")} · {LocalizeBodyName(n)}");
        }
        else
        {
            _followIndex = -1;
            _camera.GoalTarget = Vector3.Zero;
        }

        void Consider(Vector3 p, int i)
        {
            if (!_camera.WorldToScreen(p, _gfx.Width, _gfx.Height, out Vector2 s)) return;
            float d = Vector2.Distance(s, new Vector2(x, y));
            if (d < bestD) { bestD = d; best = i; }
        }
    }

    private bool PickPlane(int x, int y, out Vector3 hit)
    {
        var (o, dir) = _camera.ScreenRay(x, y, _gfx.Width, _gfx.Height);
        float planeZ = _camera.Target.Z;
        hit = default;
        if (MathF.Abs(dir.Z) < 1e-4f) return false;
        float t = (planeZ - o.Z) / dir.Z;
        if (t <= 0) return false;
        hit = o + dir * t;
        return true;
    }

    private void LaunchBody(Vector3 start, Vector3 end)
    {
        if (_cpu == null) return;
        _cpu.Wait();
        Sim s = _cpu.Main;
        BodySet b = s.Bodies;
        double mMax = 0;
        for (int i = 0; i < b.Count; ++i) mMax = Math.Max(mMax, b.Mass[i]);
        double vr = _setup.ViewRadius;
        double vScale = Math.Sqrt(s.Force.G * Math.Max(mMax, 1e-9) / vr) / (vr * 0.5);
        Vector3 v = (end - start) * (float)vScale;
        Random rng = new();
        Rgb color = Palette(rng.Next(6));
        s.AddBody(start.X, start.Y, start.Z, v.X, v.Y, v.Z, mMax * 1e-3 * (0.3 + rng.NextDouble()), BodyKind.Planet, (float)(vr * 0.012 + rng.NextDouble() * vr * 0.006), color);
        _cpu.SyncTrails();
    }

    private static Rgb Palette(int i) => i switch
    {
        0 => new(0.4f, 0.75f, 1.3f),
        1 => new(1.3f, 0.6f, 0.3f),
        2 => new(0.6f, 1.2f, 0.6f),
        3 => new(1.2f, 0.45f, 0.9f),
        4 => new(1.2f, 1.0f, 0.5f),
        _ => new(0.7f, 0.6f, 1.3f),
    };

    public void Dispose()
    {
        _cpu?.Dispose();
        _audio.Dispose();
        _canvas.Dispose();
        _renderer.Dispose();
        _gfx.Dispose();
        _window.Dispose();
    }
}
