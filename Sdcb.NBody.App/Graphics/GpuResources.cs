using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Vortice.D3DCompiler;
using Vortice.Direct3D;
using Vortice.Direct3D11;
using Vortice.DXGI;
using Vortice.Mathematics;

namespace Sdcb.NBody.App.Graphics;

internal static class ShaderLibrary
{
    private static readonly Dictionary<string, string> s_sources = [];

    public static string Source(string name)
    {
        if (s_sources.TryGetValue(name, out string? s)) return s;
        Assembly asm = typeof(ShaderLibrary).Assembly;
        using Stream stream = asm.GetManifestResourceStream("Shaders." + name)
            ?? throw new FileNotFoundException("Embedded shader not found: " + name);
        using StreamReader reader = new(stream);
        string text = reader.ReadToEnd();
        // poor man's #include for the shared header
        if (name != "common.hlsli") text = text.Replace("#include \"common.hlsli\"", Source("common.hlsli"));
        return s_sources[name] = text;
    }

    private static readonly string s_cacheDir = Path.Combine(
        Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "Sdcb.NBody", "ShaderCache");

    /// <summary>Compiles (or loads from the on-disk cache, keyed by a hash of source + entry + profile).</summary>
    public static ReadOnlyMemory<byte> Compile(string file, string entry, string profile)
    {
        string src = Source(file);
        string key = Convert.ToHexString(System.Security.Cryptography.SHA256.HashData(
            System.Text.Encoding.UTF8.GetBytes($"{profile}|{entry}|{src}")))[..32];
        string cached = Path.Combine(s_cacheDir, $"{Path.GetFileNameWithoutExtension(file)}.{entry}.{key}.cso");
        try
        {
            if (File.Exists(cached)) return File.ReadAllBytes(cached);
        }
        catch (IOException)
        {
        }
        byte[] bytes = CompileNow(src, file, entry, profile);
        try
        {
            Directory.CreateDirectory(s_cacheDir);
            File.WriteAllBytes(cached, bytes);
        }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException)
        {
        }
        return bytes;
    }

    private static byte[] CompileNow(string src, string file, string entry, string profile)
    {
        SharpGen.Runtime.Result r = Compiler.Compile(src, null!, null!, entry, file, profile,
            ShaderFlags.OptimizationLevel3, out Blob blob, out Blob error);
        if (r.Failure || blob == null)
        {
            string msg = error?.AsString() ?? r.ToString();
            error?.Dispose();
            throw new InvalidOperationException($"Shader compile failed: {file}:{entry}\n{msg}");
        }
        error?.Dispose();
        byte[] bytes = blob.AsBytes();
        blob.Dispose();
        return bytes;
    }

    public static ID3D11VertexShader VS(ID3D11Device d, string file, string entry = "VSMain") => d.CreateVertexShader(Compile(file, entry, "vs_5_0").Span);
    public static ID3D11PixelShader PS(ID3D11Device d, string file, string entry = "PSMain") => d.CreatePixelShader(Compile(file, entry, "ps_5_0").Span);
    public static ID3D11ComputeShader CS(ID3D11Device d, string file, string entry) => d.CreateComputeShader(Compile(file, entry, "cs_5_0").Span);
}

internal sealed class ConstantBuffer<T> : IDisposable where T : unmanaged
{
    public ID3D11Buffer Buffer { get; }

    public unsafe ConstantBuffer(ID3D11Device device)
    {
        uint size = (uint)((sizeof(T) + 15) & ~15);
        Buffer = device.CreateBuffer(size, BindFlags.ConstantBuffer, ResourceUsage.Dynamic, CpuAccessFlags.Write);
    }

    public unsafe void Update(ID3D11DeviceContext ctx, in T value)
    {
        MappedSubresource m = ctx.Map(Buffer, MapMode.WriteDiscard);
        Unsafe.Write((void*)m.DataPointer, value);
        ctx.Unmap(Buffer);
    }

    public void Dispose() => Buffer.Dispose();
}

/// <summary>Structured buffer, optionally CPU-writable (dynamic) or GPU-writable (UAV).</summary>
internal sealed class StructuredBuffer<T> : IDisposable where T : unmanaged
{
    public ID3D11Buffer Buffer { get; }
    public ID3D11ShaderResourceView Srv { get; }
    public ID3D11UnorderedAccessView? Uav { get; }
    public int Capacity { get; }
    public bool Dynamic { get; }

    public unsafe StructuredBuffer(ID3D11Device device, int capacity, bool dynamic, bool uav = false, ReadOnlySpan<T> initial = default)
    {
        Capacity = Math.Max(1, capacity);
        Dynamic = dynamic;
        BufferDescription desc = new(
            (uint)(Capacity * sizeof(T)),
            BindFlags.ShaderResource | (uav ? BindFlags.UnorderedAccess : BindFlags.None),
            dynamic ? ResourceUsage.Dynamic : ResourceUsage.Default,
            dynamic ? CpuAccessFlags.Write : CpuAccessFlags.None,
            ResourceOptionFlags.BufferStructured,
            (uint)sizeof(T));
        if (!initial.IsEmpty)
        {
            T[] tmp = new T[Capacity];
            initial.CopyTo(tmp);
            fixed (T* p = tmp)
            {
                Buffer = device.CreateBuffer(in desc, (nint)p);
            }
        }
        else
        {
            Buffer = device.CreateBuffer(desc);
        }
        Srv = device.CreateShaderResourceView(Buffer);
        if (uav) Uav = device.CreateUnorderedAccessView(Buffer);
    }

    public unsafe void Write(ID3D11DeviceContext ctx, ReadOnlySpan<T> data)
    {
        int n = Math.Min(data.Length, Capacity);
        if (Dynamic)
        {
            MappedSubresource m = ctx.Map(Buffer, MapMode.WriteDiscard);
            data[..n].CopyTo(new Span<T>((void*)m.DataPointer, n));
            ctx.Unmap(Buffer);
        }
        else
        {
            fixed (T* p = data)
            {
                Box box = new(0, 0, 0, n * sizeof(T), 1, 1);
                ctx.UpdateSubresource(new MappedSubresource((nint)p), Buffer, 0, box);
            }
        }
    }

    public void Dispose()
    {
        Uav?.Dispose();
        Srv.Dispose();
        Buffer.Dispose();
    }
}

internal sealed class RenderTexture : IDisposable
{
    public ID3D11Texture2D Texture { get; }
    public ID3D11RenderTargetView Rtv { get; }
    public ID3D11ShaderResourceView Srv { get; }
    public int Width { get; }
    public int Height { get; }

    public RenderTexture(ID3D11Device device, int width, int height, Format format)
    {
        Width = Math.Max(1, width);
        Height = Math.Max(1, height);
        Texture2DDescription desc = new()
        {
            Width = (uint)Width,
            Height = (uint)Height,
            MipLevels = 1,
            ArraySize = 1,
            Format = format,
            SampleDescription = new SampleDescription(1, 0),
            Usage = ResourceUsage.Default,
            BindFlags = BindFlags.RenderTarget | BindFlags.ShaderResource,
        };
        Texture = device.CreateTexture2D(desc);
        Rtv = device.CreateRenderTargetView(Texture);
        Srv = device.CreateShaderResourceView(Texture);
    }

    public void Dispose()
    {
        Srv.Dispose();
        Rtv.Dispose();
        Texture.Dispose();
    }
}

internal static class GpuExtensions
{
    public static unsafe void SetViewport(this ID3D11DeviceContext ctx, int w, int h) => ctx.RSSetViewport(0, 0, w, h);

    public static int SizeOf<T>() where T : unmanaged => Marshal.SizeOf<T>();
}
