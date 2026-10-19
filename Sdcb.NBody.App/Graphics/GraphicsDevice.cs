using Vortice.Direct2D1;
using Vortice.Direct3D;
using Vortice.Direct3D11;
using Vortice.DXGI;
using D2DFactoryType = Vortice.Direct2D1.FactoryType;
using DWFactoryType = Vortice.DirectWrite.FactoryType;
using Vortice.DirectWrite;
using FeatureLevel = Vortice.Direct3D.FeatureLevel;

namespace Sdcb.NBody.App.Graphics;

/// <summary>Owns the D3D11 device, swap chain and the Direct2D context that draws onto the back buffer.</summary>
internal sealed class GraphicsDevice : IDisposable
{
    public ID3D11Device Device { get; }
    public ID3D11DeviceContext Context { get; }
    public IDXGISwapChain1 SwapChain { get; }
    public ID2D1Factory1 D2DFactory { get; }
    public IDWriteFactory DWriteFactory { get; }
    public ID2D1DeviceContext D2D { get; }
    public ID3D11RenderTargetView BackBufferRtv { get; private set; } = null!;
    public ID3D11Texture2D BackBuffer { get; private set; } = null!;
    public int Width { get; private set; }
    public int Height { get; private set; }
    public string AdapterName { get; }
    public ulong DedicatedVideoMemory { get; }
    public uint VendorId { get; }
    public FeatureLevel FeatureLevel { get; }

    private readonly ID2D1Device _d2dDevice;
    private ID2D1Bitmap1? _d2dTarget;
    private readonly bool _allowTearing;
    private const Format BackBufferFormat = Format.B8G8R8A8_UNorm;

    public GraphicsDevice(nint hwnd, int width, int height)
    {
        using IDXGIFactory2 factory = DXGI.CreateDXGIFactory2<IDXGIFactory2>(false);
        IDXGIAdapter1? adapter = PickAdapter(factory);

        FeatureLevel[] levels = [FeatureLevel.Level_11_1, FeatureLevel.Level_11_0, FeatureLevel.Level_10_1, FeatureLevel.Level_10_0];
        DeviceCreationFlags flags = DeviceCreationFlags.BgraSupport;
        D3D11.D3D11CreateDevice(adapter, adapter == null ? DriverType.Hardware : DriverType.Unknown, flags, levels,
            out ID3D11Device device, out FeatureLevel fl, out ID3D11DeviceContext ctx).CheckError();
        Device = device;
        Context = ctx;
        FeatureLevel = fl;

        using (IDXGIDevice dxgiDevice = Device.QueryInterface<IDXGIDevice>())
        using (IDXGIAdapter a = dxgiDevice.GetAdapter())
        using (IDXGIAdapter1 a1 = a.QueryInterface<IDXGIAdapter1>())
        {
            AdapterDescription1 desc = a1.Description1;
            AdapterName = desc.Description;
            DedicatedVideoMemory = desc.DedicatedVideoMemory;
            VendorId = desc.VendorId;
        }
        adapter?.Dispose();

        using (IDXGIFactory5? f5 = factory.QueryInterfaceOrNull<IDXGIFactory5>())
        {
            _allowTearing = f5 != null && f5.PresentAllowTearing;
        }

        Width = Math.Max(width, 8);
        Height = Math.Max(height, 8);
        SwapChainDescription1 scd = new()
        {
            Width = (uint)Width,
            Height = (uint)Height,
            Format = BackBufferFormat,
            BufferCount = 2,
            BufferUsage = Usage.RenderTargetOutput,
            SampleDescription = new SampleDescription(1, 0),
            Scaling = Scaling.Stretch,
            SwapEffect = SwapEffect.FlipDiscard,
            AlphaMode = AlphaMode.Ignore,
            Flags = _allowTearing ? SwapChainFlags.AllowTearing : SwapChainFlags.None,
        };
        SwapChain = factory.CreateSwapChainForHwnd(Device, hwnd, scd);
        factory.MakeWindowAssociation(hwnd, WindowAssociationFlags.IgnoreAltEnter);

        D2DFactory = D2D1.D2D1CreateFactory<ID2D1Factory1>(D2DFactoryType.SingleThreaded);
        DWriteFactory = DWrite.DWriteCreateFactory<IDWriteFactory>(DWFactoryType.Shared);
        using (IDXGIDevice dxgiDevice = Device.QueryInterface<IDXGIDevice>())
        {
            _d2dDevice = D2DFactory.CreateDevice(dxgiDevice);
        }
        D2D = _d2dDevice.CreateDeviceContext(DeviceContextOptions.None);
        D2D.TextAntialiasMode = Vortice.Direct2D1.TextAntialiasMode.Grayscale;

        CreateSizeDependent();
    }

    private static IDXGIAdapter1? PickAdapter(IDXGIFactory2 factory)
    {
        // Prefer the high-performance GPU on hybrid laptops.
        using IDXGIFactory6? f6 = factory.QueryInterfaceOrNull<IDXGIFactory6>();
        if (f6 != null && f6.EnumAdapterByGpuPreference(0, GpuPreference.HighPerformance, out IDXGIAdapter1? best).Success)
        {
            return best;
        }
        return null;
    }

    private void CreateSizeDependent()
    {
        BackBuffer = SwapChain.GetBuffer<ID3D11Texture2D>(0);
        BackBufferRtv = Device.CreateRenderTargetView(BackBuffer);
        using IDXGISurface surface = BackBuffer.QueryInterface<IDXGISurface>();
        _d2dTarget = D2D.CreateBitmapFromDxgiSurface(surface, new BitmapProperties1(
            new Vortice.DCommon.PixelFormat(BackBufferFormat, Vortice.DCommon.AlphaMode.Ignore), 96, 96,
            BitmapOptions.Target | BitmapOptions.CannotDraw));
        D2D.Target = _d2dTarget;
    }

    private void ReleaseSizeDependent()
    {
        D2D.Target = null;
        _d2dTarget?.Dispose(); _d2dTarget = null;
        BackBufferRtv?.Dispose();
        BackBuffer?.Dispose();
        Context.ClearState();
        Context.Flush();
    }

    public void Resize(int width, int height)
    {
        width = Math.Max(width, 8);
        height = Math.Max(height, 8);
        if (width == Width && height == Height) return;
        ReleaseSizeDependent();
        Width = width;
        Height = height;
        SwapChain.ResizeBuffers(2, (uint)width, (uint)height, BackBufferFormat,
            _allowTearing ? SwapChainFlags.AllowTearing : SwapChainFlags.None).CheckError();
        CreateSizeDependent();
    }

    public void Present(bool vsync)
    {
        SharpGen.Runtime.Result r = vsync
            ? SwapChain.Present(1, PresentFlags.None)
            : SwapChain.Present(0, _allowTearing ? PresentFlags.AllowTearing : PresentFlags.None);
        if (r.Failure) r.CheckError();
    }

    public void Dispose()
    {
        ReleaseSizeDependent();
        D2D.Dispose();
        _d2dDevice.Dispose();
        DWriteFactory.Dispose();
        D2DFactory.Dispose();
        SwapChain.Dispose();
        Context.Dispose();
        Device.Dispose();
    }
}
