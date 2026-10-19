using System.Numerics;
using Sdcb.NBody.App.Graphics;
using Vortice.Direct3D11;

namespace Sdcb.NBody.App.Rendering;

/// <summary>Massive N-body simulation running entirely on the GPU (tiled O(N²) leapfrog).</summary>
internal sealed class GpuNBody : IDisposable
{
    public const int Tile = 256;
    private readonly ID3D11Device _device;
    private readonly ID3D11ComputeShader _kickDrift, _accel, _kick, _gather;
    private readonly ConstantBuffer<SimConstants> _cb;

    private StructuredBuffer<GpuParticle>? _particles;
    private StructuredBuffer<Vector4>? _accelBuf;
    private StructuredBuffer<Vector4>? _colors;
    private StructuredBuffer<UInt2>? _specials;
    private StructuredBuffer<GpuHalo>? _halos;
    private int _haloCount;
    private float _friction;
    private StructuredBuffer<BodyInstance>? _bodyOut;
    private ID3D11Buffer? _staging;
    private bool _stagingPending;

    public int Count { get; private set; }
    public int PaddedCount { get; private set; }
    public int SpecialCount { get; private set; }
    public double G { get; private set; } = 1;
    public double Softening { get; private set; }
    public double Time { get; private set; }
    public Vector3[] SpecialPositions { get; private set; } = [];
    public BodyKind[] SpecialKinds { get; private set; } = [];
    public ID3D11ShaderResourceView? ParticleSrv => _particles?.Srv;
    public ID3D11ShaderResourceView? ColorSrv => _colors?.Srv;
    public ID3D11ShaderResourceView? BodySrv => _bodyOut?.Srv;
    private bool _needsInitialAccel;

    public GpuNBody(ID3D11Device device)
    {
        _device = device;
        _kickDrift = ShaderLibrary.CS(device, "nbody.hlsl", "CSKickDrift");
        _accel = ShaderLibrary.CS(device, "nbody.hlsl", "CSAccel");
        _kick = ShaderLibrary.CS(device, "nbody.hlsl", "CSKick");
        _gather = ShaderLibrary.CS(device, "nbody.hlsl", "CSGatherBodies");
        _cb = new ConstantBuffer<SimConstants>(device);
    }

    public void Load(BodySet bodies, double g, double softening, IReadOnlyList<Sdcb.NBody.Scenarios.Halo>? halos = null, double friction = 0)
    {
        ReleaseBuffers();
        Count = bodies.Count;
        PaddedCount = (Count + Tile - 1) / Tile * Tile;
        G = g;
        Softening = softening;
        Time = 0;

        GpuParticle[] p = new GpuParticle[PaddedCount];
        Vector4[] c = new Vector4[PaddedCount];
        List<UInt2> specials = [];
        for (int i = 0; i < Count; ++i)
        {
            BodyKind kind = bodies.Kind[i];
            p[i] = new GpuParticle
            {
                Pos = new((float)bodies.Px[i], (float)bodies.Py[i], (float)bodies.Pz[i]),
                Vel = new((float)bodies.Vx[i], (float)bodies.Vy[i], (float)bodies.Vz[i]),
                Mass = (float)bodies.Mass[i],
                Kind = (float)kind,
            };
            Rgb col = bodies.Color[i];
            c[i] = new Vector4(col.R, col.G, col.B, 1);
            if (kind != BodyKind.Particle) specials.Add(new UInt2((uint)i, BitConverter.SingleToUInt32Bits(bodies.Radius[i])));
        }
        for (int i = Count; i < PaddedCount; ++i) p[i].Kind = (float)BodyKind.Particle;
        SpecialCount = specials.Count;
        SpecialPositions = new Vector3[specials.Count];
        SpecialKinds = new BodyKind[specials.Count];
        for (int i = 0; i < specials.Count; ++i)
        {
            SpecialPositions[i] = p[specials[i].X].Pos;
            SpecialKinds[i] = bodies.Kind[(int)specials[i].X];
        }

        _particles = new StructuredBuffer<GpuParticle>(_device, PaddedCount, dynamic: false, uav: true, p);
        _accelBuf = new StructuredBuffer<Vector4>(_device, PaddedCount, dynamic: false, uav: true);
        _colors = new StructuredBuffer<Vector4>(_device, PaddedCount, dynamic: false, uav: false, c);
        _specials = new StructuredBuffer<UInt2>(_device, Math.Max(1, specials.Count), dynamic: false, uav: false, specials.Count > 0 ? [.. specials] : [default]);
        _bodyOut = new StructuredBuffer<BodyInstance>(_device, Math.Max(1, specials.Count), dynamic: false, uav: true);
        GpuHalo[] h = halos is { Count: > 0 }
            ? [.. halos.Select(x => new GpuHalo { Index = (uint)x.BodyIndex, Mass = (float)x.Mass, Scale2 = (float)(x.Scale * x.Scale) })]
            : [default];
        _haloCount = halos?.Count ?? 0;
        _friction = (float)friction;
        _halos = new StructuredBuffer<GpuHalo>(_device, h.Length, dynamic: false, uav: false, h);
        unsafe
        {
            _staging = _device.CreateBuffer(new BufferDescription((uint)(Math.Max(1, specials.Count) * sizeof(BodyInstance)),
                BindFlags.None, ResourceUsage.Staging, CpuAccessFlags.Read, ResourceOptionFlags.None, 0));
        }
        _stagingPending = false;
        _needsInitialAccel = true;
    }

    public void Step(ID3D11DeviceContext ctx, double dt, int steps)
    {
        if (_particles == null || Count == 0) return;
        uint groups = (uint)(PaddedCount / Tile);
        ctx.CSSetUnorderedAccessView(0, _particles.Uav!);
        ctx.CSSetUnorderedAccessView(1, _accelBuf!.Uav!);
        ctx.CSSetConstantBuffer(1, _cb.Buffer);
        ctx.CSSetShaderResource(2, _halos!.Srv);

        SimConstants sc = new()
        {
            Count = (uint)PaddedCount,
            Dt = (float)dt,
            Eps2 = (float)(Softening * Softening),
            G = (float)G,
            KickScale = 0.5f,
            BlackHoleCount = (uint)SpecialCount,
            HaloCount = (uint)_haloCount,
            Friction = _friction,

        };

        _cb.Update(ctx, sc);
        if (_needsInitialAccel)
        {
            ctx.CSSetShader(_accel);
            ctx.Dispatch(groups, 1, 1);
            _needsInitialAccel = false;
        }

        // kick-drift-kick leapfrog: v += a dt/2; x += v dt; a = a(x); v += a dt/2
        for (int s = 0; s < steps; ++s)
        {
            ctx.CSSetShader(_kickDrift);
            ctx.Dispatch(groups, 1, 1);
            ctx.CSSetShader(_accel);
            ctx.Dispatch(groups, 1, 1);
            ctx.CSSetShader(_kick);
            ctx.Dispatch(groups, 1, 1);
        }
        Time += dt * steps;

        if (SpecialCount > 0)
        {
            ctx.CSSetUnorderedAccessView(1, null!);
            ctx.CSSetUnorderedAccessView(2, _bodyOut!.Uav!);
            ctx.CSSetShaderResource(0, _specials!.Srv);
            ctx.CSSetShaderResource(1, _colors!.Srv);
            ctx.CSSetShader(_gather);
            ctx.Dispatch((uint)((SpecialCount + 63) / 64), 1, 1);
            ctx.CSSetUnorderedAccessView(2, null!);
            ctx.CSSetShaderResource(0, null!);
            ctx.CSSetShaderResource(1, null!);
            ReadBackSpecials(ctx);
        }

        ctx.CSSetUnorderedAccessView(0, null!);
        ctx.CSSetUnorderedAccessView(1, null!);
        ctx.CSSetShaderResource(2, null!);
        ctx.CSSetShader(null!);
    }

    private unsafe void ReadBackSpecials(ID3D11DeviceContext ctx)
    {
        // read last frame's copy without stalling, then queue a new copy
        if (_stagingPending)
        {
            if (ctx.Map(_staging!, 0, MapMode.Read, MapFlags.DoNotWait, out MappedSubresource m).Success && m.DataPointer != 0)
            {
                BodyInstance* src = (BodyInstance*)m.DataPointer;
                for (int i = 0; i < SpecialCount; ++i) SpecialPositions[i] = src[i].Pos;
                ctx.Unmap(_staging!, 0);
                _stagingPending = false;
            }
        }
        if (!_stagingPending)
        {
            ctx.CopyResource(_staging!, _bodyOut!.Buffer);
            _stagingPending = true;
        }
    }

    /// <summary>Diagnostics only: reads the whole particle buffer back (stalls the GPU).</summary>
    public unsafe string Stats(ID3D11DeviceContext ctx)
    {
        if (_particles == null) return "empty";
        BufferDescription d = _particles.Buffer.Description;
        d.Usage = ResourceUsage.Staging; d.BindFlags = BindFlags.None; d.CPUAccessFlags = CpuAccessFlags.Read; d.MiscFlags = ResourceOptionFlags.None;
        using ID3D11Buffer staging = _device.CreateBuffer(d);
        ctx.CopyResource(staging, _particles.Buffer);
        MappedSubresource m = ctx.Map(staging, 0, MapMode.Read, MapFlags.None);
        GpuParticle* p = (GpuParticle*)m.DataPointer;
        int nan = 0; float maxR = 0, maxV = 0;
        float[] radii = new float[Count];
        for (int i = 0; i < Count; ++i)
        {
            if (!float.IsFinite(p[i].Pos.X) || !float.IsFinite(p[i].Vel.X)) { nan++; continue; }
            radii[i] = p[i].Pos.Length();
            maxR = MathF.Max(maxR, radii[i]);
            maxV = MathF.Max(maxV, p[i].Vel.Length());
        }
        int padNan = 0;
        for (int i = Count; i < PaddedCount; ++i) if (!float.IsFinite(p[i].Pos.X)) padNan++;
        ctx.Unmap(staging, 0);
        Array.Sort(radii);
        BufferDescription ad = _accelBuf!.Buffer.Description;
        ad.Usage = ResourceUsage.Staging; ad.BindFlags = BindFlags.None; ad.CPUAccessFlags = CpuAccessFlags.Read; ad.MiscFlags = ResourceOptionFlags.None;
        using ID3D11Buffer accStaging = _device.CreateBuffer(ad);
        ctx.CopyResource(accStaging, _accelBuf.Buffer);
        MappedSubresource am = ctx.Map(accStaging, 0, MapMode.Read, MapFlags.None);
        Vector4* acc = (Vector4*)am.DataPointer;
        float maxA = 0; for (int i = 0; i < Count; ++i) maxA = MathF.Max(maxA, new Vector3(acc[i].X, acc[i].Y, acc[i].Z).Length());
        string a0 = $"a0=({acc[0].X:e3},{acc[0].Y:e3},{acc[0].Z:e3}) w={acc[0].W} p0={p[0].Pos}";
        ctx.Unmap(accStaging, 0);
        return $"t={Time:F2} nan={nan} padNan={padNan} median={radii[Count / 2]:F3} p90={radii[Count * 9 / 10]:F3} maxR={maxR:F2} maxV={maxV:F2} maxA={maxA:e2} m0={p[0].Mass:e2} m1={p[1].Mass:e2} {a0}";
    }

    private void ReleaseBuffers()
    {
        _particles?.Dispose(); _particles = null;
        _accelBuf?.Dispose(); _accelBuf = null;
        _colors?.Dispose(); _colors = null;
        _specials?.Dispose(); _specials = null;
        _halos?.Dispose(); _halos = null;
        _haloCount = 0;
        _bodyOut?.Dispose(); _bodyOut = null;
        _staging?.Dispose(); _staging = null;
        Count = 0;
        SpecialCount = 0;
    }

    public void Dispose()
    {
        ReleaseBuffers();
        _cb.Dispose();
        _kickDrift.Dispose();
        _accel.Dispose();
        _kick.Dispose();
        _gather.Dispose();
    }
}

internal readonly record struct UInt2(uint X, uint Y);
