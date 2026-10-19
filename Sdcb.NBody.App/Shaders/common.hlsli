// Shared frame constants. Must match Rendering/FrameConstants.cs.
cbuffer Frame : register(b0)
{
    row_major float4x4 View;
    row_major float4x4 Proj;
    row_major float4x4 ViewProj;
    row_major float4x4 InvViewProj;
    float3 CamPos;      float Time;
    float3 CamRight;    float PixelsPerUnit;   // projected pixels per world unit at distance 1
    float3 CamUp;       float DpiScale;
    float3 CamForward;  float SceneScale;      // world size that frames the scene (ViewRadius)
    float2 Screen;      float2 InvScreen;
    float3 LightPos;    float HasLight;
};

static const float PI = 3.14159265;

float Hash11(float p) { p = frac(p * 0.1031); p *= p + 33.33; p *= p + p; return frac(p); }
float3 Hash33(float3 p)
{
    p = frac(p * float3(0.1031, 0.1030, 0.0973));
    p += dot(p, p.yxz + 33.33);
    return frac((p.xxy + p.yxx) * p.zyx);
}
float Hash31(float3 p) { p = frac(p * 0.1031); p += dot(p, p.zyx + 31.32); return frac((p.x + p.y) * p.z); }

float Noise3(float3 p)
{
    float3 i = floor(p);
    float3 f = frac(p);
    f = f * f * (3.0 - 2.0 * f);
    return lerp(lerp(lerp(Hash31(i + float3(0, 0, 0)), Hash31(i + float3(1, 0, 0)), f.x),
                     lerp(Hash31(i + float3(0, 1, 0)), Hash31(i + float3(1, 1, 0)), f.x), f.y),
                lerp(lerp(Hash31(i + float3(0, 0, 1)), Hash31(i + float3(1, 0, 1)), f.x),
                     lerp(Hash31(i + float3(0, 1, 1)), Hash31(i + float3(1, 1, 1)), f.x), f.y), f.z);
}

float Fbm(float3 p, int octaves)
{
    float v = 0, a = 0.5;
    for (int i = 0; i < octaves; ++i) { v += a * Noise3(p); p = p * 2.03 + 17.1; a *= 0.5; }
    return v;
}

// Approximate black-body colour for temperature t in [0,1] (cool red -> hot blue-white)
float3 Blackbody(float t)
{
    float3 c1 = float3(1.0, 0.35, 0.12);
    float3 c2 = float3(1.0, 0.85, 0.65);
    float3 c3 = float3(0.75, 0.85, 1.0);
    return t < 0.5 ? lerp(c1, c2, t * 2) : lerp(c2, c3, t * 2 - 1);
}

struct BodyInstance
{
    float3 Pos;
    float Radius;
    float3 Color;
    uint Kind;      // 0 Star, 1 Planet, 2 Moon, 3 BlackHole, 4 Particle
};

struct Particle
{
    float3 Pos;
    float Mass;
    float3 Vel;
    float Kind;     // BodyKind as float (4 = plain particle)
};
