#include "common.hlsli"

StructuredBuffer<Particle> Particles : register(t0);
StructuredBuffer<float4> Colors : register(t1);

cbuffer ParticleParams : register(b1)
{
    float ParticleRadius;   // world units
    float Brightness;
    float StreakScale;      // seconds of motion used for velocity streaks
    float MinPixels;
};

struct VSOut
{
    float4 Pos : SV_Position;
    float2 Uv : TEXCOORD0;
    float3 Color : COLOR0;
};

VSOut VSMain(uint vid : SV_VertexID, uint iid : SV_InstanceID)
{
    Particle p = Particles[iid];
    float4 tint = Colors[iid];
    VSOut o;
    if (p.Kind < 3.5) { o.Pos = float4(0, 0, -2, 1); o.Uv = 0; o.Color = 0; return o; } // stars/planets/black holes: body pass

    float depth = max(dot(p.Pos - CamPos, CamForward), 1e-4);
    float px = ParticleRadius * PixelsPerUnit / depth;
    float minPx = MinPixels * DpiScale;
    float scale = max(1.0, minPx / max(px, 1e-5));
    float r = ParticleRadius * scale;
    float energy = 1.0 / (scale * scale);   // conserve flux when enlarged to minimum size

    // velocity streak in screen space
    float3 v = p.Vel - dot(p.Vel, CamForward) * CamForward;
    float3 streak = v * StreakScale;
    float sl = length(streak);
    float3 axisA = sl > r ? streak / sl : CamRight;
    float3 axisB = normalize(cross(CamForward, axisA));
    float lenA = max(r, sl);

    static const float2 corners[4] = { float2(-1, 1), float2(1, 1), float2(-1, -1), float2(1, -1) };
    float2 c = corners[vid] * 2.0;
    float3 world = p.Pos + axisA * c.x * lenA + axisB * c.y * r;

    o.Pos = mul(float4(world, 1), ViewProj);
    o.Uv = c;
    o.Color = tint.rgb * Brightness * energy * (r / lenA);
    return o;
}

float4 PSMain(VSOut i) : SV_Target
{
    float d2 = dot(i.Uv, i.Uv);
    float f = exp(-d2 * 2.2);
    return float4(i.Color * f, 0);
}
