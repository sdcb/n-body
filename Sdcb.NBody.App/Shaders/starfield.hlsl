#include "common.hlsli"

cbuffer Sky : register(b1)
{
    float3 NebulaTintA; float NebulaIntensity;
    float3 NebulaTintB; float StarIntensity;
    float3 BandNormal;  float Seed;
};

struct VSOut { float4 Pos : SV_Position; float2 Ndc : TEXCOORD0; };

VSOut VSMain(uint id : SV_VertexID)
{
    VSOut o;
    float2 uv = float2((id << 1) & 2, id & 2);
    o.Ndc = uv * float2(2, -2) + float2(-1, 1);
    o.Pos = float4(o.Ndc, 0, 1);
    return o;
}

float3 StarLayer(float3 dir, float density, float sizePx, float seed, float population)
{
    float3 p = dir * density;
    float3 cell = floor(p);
    float3 col = 0;
    [unroll] for (int z = -1; z <= 1; ++z)
    [unroll] for (int y = -1; y <= 1; ++y)
    [unroll] for (int x = -1; x <= 1; ++x)
    {
        float3 c = cell + float3(x, y, z);
        float3 h = Hash33(c + seed);
        if (h.z > population) continue;
        float3 sp = normalize(c + h) * density;
        float3 d = p - sp;
        float dist = length(d - dot(d, dir) * dir); // distance perpendicular to the view ray
        float pxPerUnit = PixelsPerUnit / density;
        float r = dist * pxPerUnit / max(sizePx, 0.5);
        float mag = pow(Hash31(c * 1.7 + seed), 14.0) * 5.0 + 0.035;   // heavy-tailed brightness
        float tw = 0.85 + 0.15 * sin(Time * (1.5 + h.x * 3.0) + h.y * 40.0);
        float3 tint = Blackbody(Hash31(c * 3.1 + seed));
        col += tint * mag * tw * exp(-r * r * 1.6);
    }
    return col;
}

float4 PSMain(VSOut i) : SV_Target
{
    // view ray from the camera basis (no matrix inverse: it is ill-conditioned with a huge far/near ratio)
    float3 dir = normalize(CamForward + CamRight * (i.Ndc.x / Proj[0][0]) + CamUp * (i.Ndc.y / Proj[1][1]));

    // Galactic band
    float band = dot(dir, BandNormal);
    float bandMask = exp(-band * band * 18.0);
    float3 q = dir * 3.0 + Seed;
    float dust = Fbm(q * 2.5, 5);
    float neb = Fbm(q + Fbm(q * 0.7, 3) * 1.5, 5);

    float3 col = 0;
    col += lerp(NebulaTintA, NebulaTintB, saturate(neb * 1.6 - 0.3)) * pow(saturate(neb - 0.25), 2.0) * NebulaIntensity;
    col += float3(0.85, 0.8, 0.95) * bandMask * saturate(dust * 1.4 - 0.35) * 0.09 * NebulaIntensity * 3.0;
    col *= 1.0 - bandMask * smoothstep(0.45, 0.7, Fbm(q * 6.0, 4)) * 0.8; // dust lanes

    float starsBoost = 1.0 + bandMask * 1.5;
    col += StarLayer(dir, 70, 1.1 * DpiScale, Seed, 0.07) * StarIntensity;
    col += StarLayer(dir, 200, 0.8 * DpiScale, Seed + 11.3, 0.035 + 0.08 * bandMask) * StarIntensity * 0.5 * starsBoost;
    return float4(col, 1);
}
