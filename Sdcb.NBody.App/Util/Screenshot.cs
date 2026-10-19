using System.Buffers.Binary;
using System.IO.Compression;
using Vortice.Direct3D11;

namespace Sdcb.NBody.App.Util;

internal static class Screenshot
{
    /// <summary>Copies the back buffer (BGRA8) and saves it as PNG in Pictures\N-Body. Returns the path.</summary>
    public static string Capture(ID3D11Device device, ID3D11DeviceContext ctx, ID3D11Texture2D backBuffer)
    {
        string dir = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyPictures), "N-Body");
        Directory.CreateDirectory(dir);
        string path = Path.Combine(dir, $"nbody-{DateTime.Now:yyyyMMdd-HHmmss}.png");
        Save(device, ctx, backBuffer, path);
        return path;
    }

    public static unsafe void Save(ID3D11Device device, ID3D11DeviceContext ctx, ID3D11Texture2D backBuffer, string path)
    {
        Texture2DDescription desc = backBuffer.Description;
        desc.Usage = ResourceUsage.Staging;
        desc.BindFlags = BindFlags.None;
        desc.CPUAccessFlags = CpuAccessFlags.Read;
        desc.MiscFlags = ResourceOptionFlags.None;
        using ID3D11Texture2D staging = device.CreateTexture2D(desc);
        ctx.CopyResource(staging, backBuffer);
        MappedSubresource m = ctx.Map(staging, 0, MapMode.Read, MapFlags.None);
        int w = (int)desc.Width, h = (int)desc.Height;
        byte[] rgb = new byte[w * h * 3];
        try
        {
            for (int y = 0; y < h; ++y)
            {
                byte* row = (byte*)m.DataPointer + y * m.RowPitch;
                for (int x = 0; x < w; ++x)
                {
                    int o = (y * w + x) * 3;
                    rgb[o] = row[x * 4 + 2];
                    rgb[o + 1] = row[x * 4 + 1];
                    rgb[o + 2] = row[x * 4];
                }
            }
        }
        finally
        {
            ctx.Unmap(staging, 0);
        }

        Directory.CreateDirectory(Path.GetDirectoryName(path)!);
        File.WriteAllBytes(path, EncodePng(rgb, w, h));
    }

    public static byte[] EncodePng(byte[] rgb, int w, int h)
    {
        using MemoryStream raw = new();
        using (ZLibStream z = new(raw, CompressionLevel.Fastest, leaveOpen: true))
        {
            for (int y = 0; y < h; ++y)
            {
                z.WriteByte(0);
                z.Write(rgb, y * w * 3, w * 3);
            }
        }

        using MemoryStream png = new();
        png.Write([0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A]);
        byte[] ihdr = new byte[13];
        BinaryPrimitives.WriteInt32BigEndian(ihdr, w);
        BinaryPrimitives.WriteInt32BigEndian(ihdr.AsSpan(4), h);
        ihdr[8] = 8; ihdr[9] = 2; // 8-bit RGB
        Chunk(png, "IHDR", ihdr);
        Chunk(png, "IDAT", raw.ToArray());
        Chunk(png, "IEND", []);
        return png.ToArray();
    }

    private static void Chunk(Stream s, string type, byte[] data)
    {
        Span<byte> b4 = stackalloc byte[4];
        BinaryPrimitives.WriteInt32BigEndian(b4, data.Length);
        s.Write(b4);
        byte[] typeBytes = System.Text.Encoding.ASCII.GetBytes(type);
        s.Write(typeBytes);
        s.Write(data);
        uint crc = Crc32(typeBytes, 0xFFFFFFFF);
        crc = Crc32(data, crc) ^ 0xFFFFFFFF;
        BinaryPrimitives.WriteUInt32BigEndian(b4, crc);
        s.Write(b4);
    }

    private static readonly uint[] s_crcTable = BuildCrcTable();

    private static uint[] BuildCrcTable()
    {
        uint[] t = new uint[256];
        for (uint n = 0; n < 256; ++n)
        {
            uint c = n;
            for (int k = 0; k < 8; ++k) c = (c & 1) != 0 ? 0xEDB88320 ^ (c >> 1) : c >> 1;
            t[n] = c;
        }
        return t;
    }

    private static uint Crc32(byte[] data, uint crc)
    {
        foreach (byte b in data) crc = s_crcTable[(crc ^ b) & 0xFF] ^ (crc >> 8);
        return crc;
    }
}
