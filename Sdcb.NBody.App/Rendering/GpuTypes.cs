using System.Numerics;
using System.Runtime.InteropServices;

namespace Sdcb.NBody.App.Rendering;

[StructLayout(LayoutKind.Sequential)]
internal struct FrameConstants
{
    public Matrix4x4 View, Proj, ViewProj, InvViewProj;
    public Vector3 CamPos; public float Time;
    public Vector3 CamRight; public float PixelsPerUnit;
    public Vector3 CamUp; public float DpiScale;
    public Vector3 CamForward; public float SceneScale;
    public Vector2 Screen, InvScreen;
    public Vector3 LightPos; public float HasLight;
}

[StructLayout(LayoutKind.Sequential)]
internal struct SkyConstants
{
    public Vector3 NebulaTintA; public float NebulaIntensity;
    public Vector3 NebulaTintB; public float StarIntensity;
    public Vector3 BandNormal; public float Seed;
}

[StructLayout(LayoutKind.Sequential)]
internal struct TrailConstants
{
    public float Intensity, WidthScale;
    public uint PointCount;
    public float Pad;
}

[StructLayout(LayoutKind.Sequential)]
internal struct BodyConstants
{
    public float MinPixelRadius, GlowScale, Pad0, Pad1;
}

[StructLayout(LayoutKind.Sequential)]
internal struct ParticleConstants
{
    public float ParticleRadius, Brightness, StreakScale, MinPixels;
}

[StructLayout(LayoutKind.Sequential)]
internal struct SimConstants
{
    public uint Count;
    public float Dt, Eps2, G, KickScale;
    public uint BlackHoleCount;
    public float Friction;
    public uint HaloCount;
}

[StructLayout(LayoutKind.Sequential)]
internal unsafe struct PostConstants
{
    public const int MaxPanels = 16;
    public Vector2 SrcTexel; public float BloomStrength, Exposure;
    public float Fade, Vignette, Grain, Aberration;
    public uint PanelCount; public Vector3 Pad0;
    public fixed float Panels[MaxPanels * 4];
    public fixed float PanelStyle[MaxPanels * 4];
}

[StructLayout(LayoutKind.Sequential)]
internal struct TrailPoint
{
    public Vector3 Pos;
    public float Fade;
    public Vector3 Color;
    public float Width;
}

[StructLayout(LayoutKind.Sequential)]
internal struct BodyInstance
{
    public Vector3 Pos;
    public float Radius;
    public Vector3 Color;
    public uint Kind;
}

[StructLayout(LayoutKind.Sequential)]
internal struct GpuParticle
{
    public Vector3 Pos;
    public float Mass;
    public Vector3 Vel;
    public float Kind;
}

[StructLayout(LayoutKind.Sequential)]
internal struct GpuHalo
{
    public uint Index;
    public float Mass, Scale2, Pad;
}

/// <summary>Rounded glass panel the post pass blurs behind (pixel coordinates).</summary>
internal readonly record struct GlassPanel(float X, float Y, float W, float H, float Radius, float Opacity = 1f, float Highlight = 1f);
