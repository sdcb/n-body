#include "common.hlsli"

#define TILE 256

RWStructuredBuffer<Particle> Particles : register(u0);
RWStructuredBuffer<float4> Accel : register(u1);
StructuredBuffer<uint2> Specials : register(t0);   // (particle index, asuint(radius))
StructuredBuffer<float4> Colors : register(t1);

struct HaloGpu { uint Index; float Mass; float Scale2; float Pad; };
StructuredBuffer<HaloGpu> Halos : register(t2);   // analytic Plummer dark-matter halos following a body
RWStructuredBuffer<BodyInstance> BodyOut : register(u2);

cbuffer SimParams : register(b1)
{
    uint Count;         // padded to multiple of TILE
    float Dt;
    float Eps2;
    float G;
    float KickScale;    // 0.5 for a regular kick, 0 for init
    uint BlackHoleCount;
    float Friction;
    uint HaloCount;
};

groupshared float4 tile[TILE];

[numthreads(TILE, 1, 1)]
void CSKickDrift(uint3 id : SV_DispatchThreadID)
{
    Particle p = Particles[id.x];
    float3 a = Accel[id.x].xyz;
    p.Vel += a * (Dt * 0.5);
    p.Pos += p.Vel * Dt;
    Particles[id.x] = p;
}

[numthreads(TILE, 1, 1)]
void CSAccel(uint3 id : SV_DispatchThreadID, uint3 lid : SV_GroupThreadID)
{
    Particle p = Particles[id.x];
    float3 acc = 0;
    for (uint start = 0; start < Count; start += TILE)
    {
        Particle q = Particles[start + lid.x];
        tile[lid.x] = float4(q.Pos, q.Mass);
        GroupMemoryBarrierWithGroupSync();
        [loop]
        for (uint j = 0; j < TILE; j += 4)
        {
            [unroll]
            for (uint k = 0; k < 4; ++k)
            {
                float4 o = tile[j + k];
                float3 r = o.xyz - p.Pos;
                float d2 = dot(r, r) + Eps2;
                float inv = rsqrt(d2);
                acc += r * (o.w * inv * inv * inv);
            }
        }
        GroupMemoryBarrierWithGroupSync();
    }
    for (uint h = 0; h < HaloCount; ++h)
    {
        HaloGpu hh = Halos[h];
        if (hh.Index == id.x) continue;
        float3 r = Particles[hh.Index].Pos - p.Pos;
        float inv = rsqrt(dot(r, r) + hh.Scale2);
        acc += r * (hh.Mass * inv * inv * inv);
        if (Friction > 0 && p.Kind > 2.5 && p.Kind < 3.5)
        {
            // Chandrasekhar-like drag of a galactic nucleus through another halo: a = -k ρ(r) v_rel
            float s2 = hh.Scale2;
            float rho = 0.2387 * hh.Mass / (s2 * sqrt(s2)) * pow(s2 * inv * inv, 2.5);
            acc -= Friction * rho * (p.Vel - Particles[hh.Index].Vel);
        }
    }
    Accel[id.x] = float4(acc * G, 0);
}

[numthreads(TILE, 1, 1)]
void CSKick(uint3 id : SV_DispatchThreadID)
{
    Particles[id.x].Vel += Accel[id.x].xyz * (Dt * KickScale);
}

[numthreads(64, 1, 1)]
void CSGatherBodies(uint3 id : SV_DispatchThreadID)
{
    if (id.x >= BlackHoleCount) return;
    uint2 s = Specials[id.x];
    Particle p = Particles[s.x];
    BodyInstance b;
    b.Pos = p.Pos;
    b.Radius = asfloat(s.y);
    b.Color = Colors[s.x].rgb;
    b.Kind = (uint)(p.Kind + 0.5);
    BodyOut[id.x] = b;
}
