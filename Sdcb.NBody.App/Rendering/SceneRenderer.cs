using System.Numerics;
using Sdcb.NBody.App.Graphics;
using Vortice.Direct3D;
using Vortice.Direct3D11;
using Vortice.DXGI;
using Vortice.Mathematics;

namespace Sdcb.NBody.App.Rendering;

internal sealed class RenderSettings
{
    public float Exposure = 1.0f;
    public float BloomStrength = 0.06f;
    public float Vignette = 0.35f;
    public float Grain = 0.012f;
    public float Aberration = 0.006f;
    public float Fade = 1f;
    public float TrailIntensity = 1.6f;
    public float TrailWidth = 2.0f;
    public float BodyMinPixels = 2.5f;
    public float StarGlow = 1f;
    public float ParticleRadius = 0.004f;
    public float ParticleBrightness = 0.5f;
    public float ParticleStreak = 0.02f;
    public bool ShowTrails = true;
    public SkyConstants Sky = new()
    {
        NebulaTintA = new(0.10f, 0.05f, 0.22f),
        NebulaTintB = new(0.02f, 0.14f, 0.20f),
        NebulaIntensity = 0.5f,
        StarIntensity = 1f,
        BandNormal = Vector3.Normalize(new(0.3f, 0.5f, 0.8f)),
        Seed = 3.7f,
    };
}

internal sealed class SceneRenderer : IDisposable
{
    private readonly GraphicsDevice _g;
    private readonly ID3D11Device _d;
    private readonly int _bloomLevels;

    private readonly ID3D11VertexShader _skyVS, _trailVS, _bodyVS, _particleVS, _fullscreenVS;
    private readonly ID3D11PixelShader _skyPS, _trailPS, _bodyPS, _particlePS, _downFirstPS, _downPS, _upPS, _compositePS;
    private readonly ID3D11BlendState _opaque, _additive, _premultiplied;
    private readonly ID3D11RasterizerState _raster;
    private readonly ID3D11DepthStencilState _noDepth;
    private readonly ID3D11SamplerState _linearClamp;

    private readonly ConstantBuffer<FrameConstants> _frameCb;
    private readonly ConstantBuffer<SkyConstants> _skyCb;
    private readonly ConstantBuffer<TrailConstants> _trailCb;
    private readonly ConstantBuffer<BodyConstants> _bodyCb;
    private readonly ConstantBuffer<ParticleConstants> _particleCb;
    private readonly ConstantBuffer<PostConstants> _postCb;

    private StructuredBuffer<TrailPoint>? _trailBuffer;
    private StructuredBuffer<BodyInstance>? _bodyBuffer;

    private RenderTexture? _hdr;
    private RenderTexture[] _bloom = [];

    public GpuNBody GpuSim { get; }

    public SceneRenderer(GraphicsDevice g, int bloomLevels)
    {
        _g = g;
        _d = g.Device;
        _bloomLevels = bloomLevels;
        _skyVS = ShaderLibrary.VS(_d, "starfield.hlsl"); _skyPS = ShaderLibrary.PS(_d, "starfield.hlsl");
        _trailVS = ShaderLibrary.VS(_d, "trail.hlsl"); _trailPS = ShaderLibrary.PS(_d, "trail.hlsl");
        _bodyVS = ShaderLibrary.VS(_d, "body.hlsl"); _bodyPS = ShaderLibrary.PS(_d, "body.hlsl");
        _particleVS = ShaderLibrary.VS(_d, "particles.hlsl"); _particlePS = ShaderLibrary.PS(_d, "particles.hlsl");
        _fullscreenVS = ShaderLibrary.VS(_d, "post.hlsl", "VSFullscreen");
        _downFirstPS = ShaderLibrary.PS(_d, "post.hlsl", "PSDownsampleFirst");
        _downPS = ShaderLibrary.PS(_d, "post.hlsl", "PSDownsample");
        _upPS = ShaderLibrary.PS(_d, "post.hlsl", "PSUpsample");
        _compositePS = ShaderLibrary.PS(_d, "post.hlsl", "PSComposite");

        _opaque = _d.CreateBlendState(BlendDescription.Opaque);
        BlendDescription add = new(Blend.One, Blend.One, Blend.One, Blend.One);
        _additive = _d.CreateBlendState(add);
        _premultiplied = _d.CreateBlendState(new BlendDescription(Blend.One, Blend.InverseSourceAlpha, Blend.One, Blend.InverseSourceAlpha));
        _raster = _d.CreateRasterizerState(RasterizerDescription.CullNone);
        _noDepth = _d.CreateDepthStencilState(DepthStencilDescription.None);
        _linearClamp = _d.CreateSamplerState(SamplerDescription.LinearClamp);

        _frameCb = new(_d); _skyCb = new(_d); _trailCb = new(_d); _bodyCb = new(_d); _particleCb = new(_d); _postCb = new(_d);
        GpuSim = new GpuNBody(_d);
    }

    public void Resize(int width, int height)
    {
        _hdr?.Dispose();
        foreach (RenderTexture t in _bloom) t.Dispose();
        _hdr = new RenderTexture(_d, width, height, Format.R16G16B16A16_Float);
        List<RenderTexture> mips = [];
        int w = width, h = height;
        for (int i = 0; i < _bloomLevels && w > 4 && h > 4; ++i)
        {
            w = Math.Max(1, w / 2); h = Math.Max(1, h / 2);
            mips.Add(new RenderTexture(_d, w, h, Format.R11G11B10_Float));
        }
        _bloom = [.. mips];
    }

    private void EnsureTrailCapacity(int n)
    {
        if (_trailBuffer != null && _trailBuffer.Capacity >= n) return;
        _trailBuffer?.Dispose();
        _trailBuffer = new StructuredBuffer<TrailPoint>(_d, Math.Max(1024, (int)BitOperations.RoundUpToPowerOf2((uint)n)), dynamic: true);
    }

    private void EnsureBodyCapacity(int n)
    {
        if (_bodyBuffer != null && _bodyBuffer.Capacity >= n) return;
        _bodyBuffer?.Dispose();
        _bodyBuffer = new StructuredBuffer<BodyInstance>(_d, Math.Max(64, (int)BitOperations.RoundUpToPowerOf2((uint)n)), dynamic: true);
    }

    public void Render(in FrameConstants frame, RenderSettings s, ReadOnlySpan<TrailPoint> trails, ReadOnlySpan<BodyInstance> bodies,
        bool drawGpuParticles, IReadOnlyList<GlassPanel> panels)
    {
        ID3D11DeviceContext ctx = _g.Context;
        RenderTexture hdr = _hdr!;

        _frameCb.Update(ctx, frame);
        ctx.RSSetState(_raster);
        ctx.OMSetDepthStencilState(_noDepth);
        ctx.IASetInputLayout(null);
        ctx.VSSetConstantBuffer(0, _frameCb.Buffer);
        ctx.PSSetConstantBuffer(0, _frameCb.Buffer);
        ctx.PSSetSampler(0, _linearClamp);

        // --- Scene into HDR target ---
        ctx.OMSetRenderTargets(hdr.Rtv);
        ctx.SetViewport(hdr.Width, hdr.Height);

        // sky
        _skyCb.Update(ctx, s.Sky);
        ctx.OMSetBlendState(_opaque);
        ctx.IASetPrimitiveTopology(PrimitiveTopology.TriangleList);
        ctx.VSSetShader(_skyVS);
        ctx.PSSetShader(_skyPS);
        ctx.PSSetConstantBuffer(1, _skyCb.Buffer);
        ctx.Draw(3, 0);
        PassProbe?.Invoke("sky");

        // trails
        if (s.ShowTrails && trails.Length > 1)
        {
            EnsureTrailCapacity(trails.Length);
            _trailBuffer!.Write(ctx, trails);
            _trailCb.Update(ctx, new TrailConstants { Intensity = s.TrailIntensity, WidthScale = s.TrailWidth, PointCount = (uint)trails.Length });
            ctx.OMSetBlendState(_additive);
            ctx.VSSetShader(_trailVS);
            ctx.PSSetShader(_trailPS);
            ctx.VSSetConstantBuffer(1, _trailCb.Buffer);
            ctx.VSSetShaderResource(0, _trailBuffer.Srv);
            ctx.DrawInstanced(6, (uint)(trails.Length - 1), 0, 0);
            ctx.VSSetShaderResource(0, null!);
        }

        // GPU particles
        if (drawGpuParticles && GpuSim.Count > 0)
        {
            _particleCb.Update(ctx, new ParticleConstants
            {
                ParticleRadius = s.ParticleRadius,
                Brightness = s.ParticleBrightness,
                StreakScale = s.ParticleStreak,
                MinPixels = 0.9f,
            });
            ctx.OMSetBlendState(_additive);
            ctx.IASetPrimitiveTopology(PrimitiveTopology.TriangleStrip);
            ctx.VSSetShader(_particleVS);
            ctx.PSSetShader(_particlePS);
            ctx.VSSetConstantBuffer(1, _particleCb.Buffer);
            ctx.VSSetShaderResource(0, GpuSim.ParticleSrv!);
            ctx.VSSetShaderResource(1, GpuSim.ColorSrv!);
            ctx.DrawInstanced(4, (uint)GpuSim.Count, 0, 0);
            ctx.VSSetShaderResource(0, null!);
            ctx.VSSetShaderResource(1, null!);
            PassProbe?.Invoke("particles");
        }

        // bodies (CPU list) + black holes gathered on the GPU
        _bodyCb.Update(ctx, new BodyConstants { MinPixelRadius = s.BodyMinPixels, GlowScale = s.StarGlow });
        ctx.OMSetBlendState(_premultiplied);
        ctx.IASetPrimitiveTopology(PrimitiveTopology.TriangleStrip);
        ctx.VSSetShader(_bodyVS);
        ctx.PSSetShader(_bodyPS);
        ctx.VSSetConstantBuffer(1, _bodyCb.Buffer);
        ctx.PSSetConstantBuffer(1, _bodyCb.Buffer);
        if (drawGpuParticles && GpuSim.SpecialCount > 0)
        {
            ctx.VSSetShaderResource(0, GpuSim.BodySrv!);
            ctx.DrawInstanced(4, (uint)GpuSim.SpecialCount, 0, 0);
        }
        if (bodies.Length > 0)
        {
            EnsureBodyCapacity(bodies.Length);
            _bodyBuffer!.Write(ctx, bodies);
            ctx.VSSetShaderResource(0, _bodyBuffer.Srv);
            ctx.DrawInstanced(4, (uint)bodies.Length, 0, 0);
        }
        ctx.VSSetShaderResource(0, null!);
        PassProbe?.Invoke("bodies");

        // --- Bloom chain ---
        ctx.IASetPrimitiveTopology(PrimitiveTopology.TriangleList);
        ctx.VSSetShader(_fullscreenVS);
        ctx.PSSetConstantBuffer(1, _postCb.Buffer);
        ctx.OMSetBlendState(_opaque);
        PostConstants pc = default;
        RenderTexture src = hdr;
        for (int i = 0; i < _bloom.Length; ++i)
        {
            RenderTexture dst = _bloom[i];
            pc.SrcTexel = new Vector2(1f / src.Width, 1f / src.Height);
            _postCb.Update(ctx, pc);
            ctx.OMSetRenderTargets(dst.Rtv);
            ctx.SetViewport(dst.Width, dst.Height);
            ctx.PSSetShader(i == 0 ? _downFirstPS : _downPS);
            ctx.PSSetShaderResource(0, src.Srv);
            ctx.Draw(3, 0);
            ctx.PSSetShaderResource(0, null!);
            src = dst;
        }
        ctx.OMSetBlendState(_additive);
        ctx.PSSetShader(_upPS);
        for (int i = _bloom.Length - 1; i > 0; --i)
        {
            RenderTexture from = _bloom[i], to = _bloom[i - 1];
            pc.SrcTexel = new Vector2(1f / from.Width, 1f / from.Height);
            _postCb.Update(ctx, pc);
            ctx.OMSetRenderTargets(to.Rtv);
            ctx.SetViewport(to.Width, to.Height);
            ctx.PSSetShaderResource(0, from.Srv);
            ctx.Draw(3, 0);
            ctx.PSSetShaderResource(0, null!);
        }

        // --- Composite to back buffer ---
        ctx.OMSetBlendState(_opaque);
        ctx.OMSetRenderTargets(_g.BackBufferRtv);
        ctx.SetViewport(_g.Width, _g.Height);
        pc.SrcTexel = new Vector2(1f / hdr.Width, 1f / hdr.Height);
        pc.BloomStrength = s.BloomStrength;
        pc.Exposure = s.Exposure;
        pc.Fade = s.Fade;
        pc.Vignette = s.Vignette;
        pc.Grain = s.Grain;
        pc.Aberration = s.Aberration;
        pc.Pad0 = new Vector3(1f / Math.Max(1, _bloom.Length), 0, 0);
        int pn = Math.Min(panels.Count, PostConstants.MaxPanels);
        pc.PanelCount = (uint)pn;
        unsafe
        {
            for (int i = 0; i < pn; ++i)
            {
                GlassPanel p = panels[i];
                pc.Panels[i * 4 + 0] = p.X; pc.Panels[i * 4 + 1] = p.Y; pc.Panels[i * 4 + 2] = p.W; pc.Panels[i * 4 + 3] = p.H;
                pc.PanelStyle[i * 4 + 0] = p.Radius; pc.PanelStyle[i * 4 + 1] = p.Opacity; pc.PanelStyle[i * 4 + 2] = p.Highlight;
            }
        }
        _postCb.Update(ctx, pc);
        ctx.PSSetShader(_compositePS);
        ctx.PSSetShaderResource(0, hdr.Srv);
        ctx.PSSetShaderResource(1, _bloom.Length > 0 ? _bloom[0].Srv : hdr.Srv);
        ctx.PSSetShaderResource(2, _bloom.Length > 0 ? _bloom[Math.Min(2, _bloom.Length - 1)].Srv : hdr.Srv);
        ctx.Draw(3, 0);
        ctx.PSSetShaderResource(0, null!);
        ctx.PSSetShaderResource(1, null!);
        ctx.PSSetShaderResource(2, null!);
    }

    /// <summary>Diagnostics hook: when set, called after each scene pass with the pass name.</summary>
    public Action<string>? PassProbe { get; set; }

    /// <summary>Diagnostics only: counts non-finite and negative texels in the HDR target.</summary>
    public unsafe string ProbeHdr()
    {
        ID3D11DeviceContext ctx = _g.Context;
        Texture2DDescription d = _hdr!.Texture.Description;
        d.Usage = ResourceUsage.Staging; d.BindFlags = BindFlags.None; d.CPUAccessFlags = CpuAccessFlags.Read; d.MiscFlags = ResourceOptionFlags.None;
        using ID3D11Texture2D staging = _d.CreateTexture2D(d);
        ctx.CopyResource(staging, _hdr.Texture);
        MappedSubresource m = ctx.Map(staging, 0, MapMode.Read, Vortice.Direct3D11.MapFlags.None);
        int nan = 0, inf = 0, neg = 0;
        float max = 0;
        double sum = 0;
        for (int y = 0; y < d.Height; ++y)
        {
            Half* row = (Half*)((byte*)m.DataPointer + y * m.RowPitch);
            for (int x = 0; x < d.Width * 4; ++x)
            {
                if ((x & 3) == 3) continue;
                float v = (float)row[x];
                if (float.IsNaN(v)) nan++;
                else if (float.IsInfinity(v)) inf++;
                else { if (v < 0) neg++; max = MathF.Max(max, v); sum += v; }
            }
        }
        ctx.Unmap(staging, 0);
        return $"nan={nan} inf={inf} neg={neg} max={max:F1} mean={sum / (d.Width * d.Height * 3.0):F4}";
    }

    public void Dispose()
    {
        GpuSim.Dispose();
        _trailBuffer?.Dispose();
        _bodyBuffer?.Dispose();
        _hdr?.Dispose();
        foreach (RenderTexture t in _bloom) t.Dispose();
        _frameCb.Dispose(); _skyCb.Dispose(); _trailCb.Dispose(); _bodyCb.Dispose(); _particleCb.Dispose(); _postCb.Dispose();
        _opaque.Dispose(); _additive.Dispose(); _premultiplied.Dispose(); _raster.Dispose(); _noDepth.Dispose(); _linearClamp.Dispose();
        _skyVS.Dispose(); _skyPS.Dispose(); _trailVS.Dispose(); _trailPS.Dispose(); _bodyVS.Dispose(); _bodyPS.Dispose();
        _particleVS.Dispose(); _particlePS.Dispose(); _fullscreenVS.Dispose(); _downFirstPS.Dispose(); _downPS.Dispose(); _upPS.Dispose(); _compositePS.Dispose();
    }
}
