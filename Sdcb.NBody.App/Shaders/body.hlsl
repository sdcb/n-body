#include "common.hlsli"

StructuredBuffer<BodyInstance> Bodies : register(t0);

cbuffer BodyParams : register(b1)
{
    float MinPixelRadius;
    float GlowScale;
    float2 _pad;
};

struct VSOut
{
    float4 Pos : SV_Position;
    float2 Uv : TEXCOORD0;           // in units of the effective radius
    nointerpolation float3 Center : TEXCOORD1;
    nointerpolation float Radius : TEXCOORD2;
    nointerpolation float3 Color : TEXCOORD3;
    nointerpolation uint Kind : TEXCOORD4;
    nointerpolation float PixelRadius : TEXCOORD5;
};

float Extent(uint kind)
{
    switch (kind)
    {
        case 0: return 9.0;   // star corona + spikes
        case 1: return 1.8;   // planet atmosphere halo
        case 2: return 1.6;
        case 3: return 8.0;   // black hole accretion disk
        default: return 3.5;
    }
}

VSOut VSMain(uint vid : SV_VertexID, uint iid : SV_InstanceID)
{
    BodyInstance b = Bodies[iid];
    float depth = max(dot(b.Pos - CamPos, CamForward), 1e-4);
    float pixelRadius = b.Radius * PixelsPerUnit / depth;
    float minPx = MinPixelRadius * DpiScale * (b.Kind == 4 ? 0.5 : 1.0);
    float r = b.Radius * max(1.0, minPx / max(pixelRadius, 1e-5));

    static const float2 corners[4] = { float2(-1, 1), float2(1, 1), float2(-1, -1), float2(1, -1) };
    float ext = Extent(b.Kind) * (b.Kind == 0 ? GlowScale : 1.0);
    float2 c = corners[vid] * ext;
    float3 world = b.Pos + (c.x * CamRight + c.y * CamUp) * r;

    VSOut o;
    o.Pos = mul(float4(world, 1), ViewProj);
    o.Uv = c;
    o.Center = b.Pos;
    o.Radius = r;
    o.Color = b.Color;
    o.Kind = b.Kind;
    o.PixelRadius = max(pixelRadius, minPx);
    return o;
}

float4 ShadeStar(VSOut i)
{
    float d = length(i.Uv);
    float aa = 1.5 / i.PixelRadius;
    float core = smoothstep(1.0 + aa, 1.0 - aa, d);
    float3 hot = lerp(i.Color, float3(1, 1, 1), 0.75);
    float limb = sqrt(saturate(1 - d * d));
    float flicker = 0.93 + 0.07 * Noise3(float3(i.Uv * 3.0, Time * 1.5));
    float3 col = hot * core * (2.5 + 3.5 * limb) * flicker;

    float halo = exp(-max(d - 1.0, 0) * 1.4) * 0.9 + 0.35 / (1.0 + d * d);
    col += i.Color * halo * 1.2 * (1 - core * 0.5);

    // diffraction spikes (screen aligned)
    float2 a = abs(i.Uv);
    float spikes = exp(-a.x * 18.0 / max(1, a.y * 0.6)) * exp(-a.y * 0.45) + exp(-a.y * 18.0 / max(1, a.x * 0.6)) * exp(-a.x * 0.45);
    col += lerp(i.Color, 1, 0.5) * spikes * 0.45 * saturate(i.PixelRadius / 3.0);

    float edgeFade = saturate(1 - d / Extent(0) / GlowScale);
    col *= edgeFade * edgeFade;
    return float4(col, core);
}

float3 LightDirFor(float3 center)
{
    if (HasLight > 0.5) return normalize(LightPos - center);
    return normalize(CamUp * 0.6 + CamRight * 0.4 - CamForward);
}

float4 ShadePlanet(VSOut i)
{
    float d = length(i.Uv);
    float3 L = LightDirFor(i.Center);
    float aa = 1.5 / i.PixelRadius;
    float alpha = smoothstep(1.0 + aa, 1.0 - aa, d);

    float3 col = 0;
    if (d < 1.0 + aa)
    {
        float z = sqrt(saturate(1 - d * d));
        float3 N = normalize(i.Uv.x * CamRight + i.Uv.y * CamUp - z * CamForward);
        float ndl = dot(N, L);
        float lit = smoothstep(-0.12, 0.35, ndl);
        // latitude bands + noise to give planets some texture
        float lat = N.z;
        float bands = 0.82 + 0.18 * sin(lat * 18.0 + Fbm(N * 3.0 + i.Center * 7.0, 3) * 4.0);
        float3 albedo = i.Color * bands;
        float rim = pow(1 - z, 3.0);
        col = albedo * (0.02 + 1.4 * lit) + i.Color * rim * lit * 1.2;
    }
    // atmosphere halo
    float halo = exp(-max(d - 1.0, 0) * 9.0) * (1 - alpha);
    float side = saturate(dot(normalize(i.Uv.x * CamRight + i.Uv.y * CamUp), L) * 0.5 + 0.5);
    col += i.Color * halo * (0.15 + 0.8 * side);

    // tiny on screen: add a glow so the planet never disappears
    float small = saturate(1.0 - i.PixelRadius / 6.0);
    col += i.Color * exp(-d * d * 1.5) * small * 1.5;
    return float4(col, alpha * (1 - small * 0.5));
}

float4 ShadeBlackHole(VSOut i)
{
    float3 world = i.Center + (i.Uv.x * CamRight + i.Uv.y * CamUp) * i.Radius;
    float3 rd = normalize(world - CamPos);
    float3 n = float3(0, 0, 1);
    float d = length(i.Uv);

    // event-horizon shadow via ray-sphere intersection
    float3 oc = CamPos - i.Center;
    float bq = dot(oc, rd);
    float cq = dot(oc, oc) - i.Radius * i.Radius;
    float disc = bq * bq - cq;
    float tSphere = disc > 0 ? -bq - sqrt(disc) : 1e30;

    float3 col = 0;
    float alpha = disc > 0 ? 1.0 : 0.0;

    float denom = dot(rd, n);
    if (abs(denom) > 1e-4)
    {
        float t = dot(i.Center - CamPos, n) / denom;
        if (t > 0 && t < tSphere)
        {
            float3 hit = CamPos + rd * t;
            float3 rel = (hit - i.Center) / i.Radius;
            float rr = length(rel);
            if (rr > 1.6 && rr < 7.5)
            {
                float ang = atan2(rel.y, rel.x);
                float omega = 1.6 / pow(rr, 1.5);
                float swirl = Fbm(float3(cos(ang - Time * omega) * rr, sin(ang - Time * omega) * rr, rr * 0.7) * 1.3, 4);
                float temp = pow(1.6 / rr, 0.75);
                float3 tang = normalize(cross(n, hit - i.Center));
                float doppler = 1.0 + 0.75 * dot(tang, -rd);
                float edge = smoothstep(1.6, 2.1, rr) * smoothstep(7.5, 4.5, rr);
                float3 diskCol = lerp(float3(1.0, 0.35, 0.08), float3(1.0, 0.9, 0.75), temp) * temp * 5.0;
                col += diskCol * edge * (0.35 + swirl * 1.3) * doppler * doppler;
            }
        }
    }

    // photon ring + soft lensing glow around the shadow
    float rq = (d - 1.08) / 0.05;
    float ring = exp(-rq * rq);
    col += float3(1.0, 0.75, 0.45) * ring * 3.0;
    col += float3(1.0, 0.55, 0.25) * exp(-max(d - 1.0, 0) * 1.2) * 0.25 * (1 - alpha);
    col *= saturate(1 - d / Extent(3));
    return float4(col * (1 - alpha * 0.0), alpha);
}

float4 ShadeParticle(VSOut i)
{
    float d2 = dot(i.Uv, i.Uv);
    return float4(i.Color * exp(-d2 * 1.2) * 2.0, 0);
}

float4 PSMain(VSOut i) : SV_Target
{
    switch (i.Kind)
    {
        case 0: return ShadeStar(i);
        case 1:
        case 2: return ShadePlanet(i);
        case 3: return ShadeBlackHole(i);
        default: return ShadeParticle(i);
    }
}
