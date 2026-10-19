#include "common.hlsli"

struct TrailPoint
{
    float3 Pos;
    float Fade;     // 0 = oldest, 1 = newest
    float3 Color;
    float Width;    // pixel width; negative marks the first point of a new strip
};

StructuredBuffer<TrailPoint> Points : register(t0);

cbuffer TrailParams : register(b1)
{
    float Intensity;
    float WidthScale;
    uint PointCount;
    float _pad;
};

struct VSOut
{
    float4 Pos : SV_Position;
    float3 Color : COLOR0;
    float Across : TEXCOORD0;
};

VSOut VSMain(uint vid : SV_VertexID, uint iid : SV_InstanceID)
{
    VSOut o = (VSOut)0;
    if (iid + 1 >= PointCount) { o.Pos = float4(0, 0, -2, 1); return o; }
    TrailPoint a = Points[iid];
    TrailPoint b = Points[iid + 1];
    if (b.Width < 0) { o.Pos = float4(0, 0, -2, 1); return o; } // strip break

    float4 ca = mul(float4(a.Pos, 1), ViewProj);
    float4 cb = mul(float4(b.Pos, 1), ViewProj);
    if (ca.w <= 0.001 || cb.w <= 0.001) { o.Pos = float4(0, 0, -2, 1); return o; }

    float2 sa = ca.xy / ca.w * Screen * 0.5;
    float2 sb = cb.xy / cb.w * Screen * 0.5;
    float2 dir = sb - sa;
    float len = length(dir);
    dir = len > 1e-4 ? dir / len : float2(1, 0);
    float2 nrm = float2(-dir.y, dir.x);

    // vertex 0..5 -> two triangles
    static const float2 corners[6] = { float2(0, -1), float2(1, -1), float2(0, 1), float2(0, 1), float2(1, -1), float2(1, 1) };
    float2 c = corners[vid];
    bool atB = c.x > 0.5;
    TrailPoint p = a;
    float4 cp = ca;
    if (atB) { p = b; cp = cb; }
    float fade = p.Fade;
    float widthPx = max(abs(p.Width) * WidthScale * DpiScale * (0.25 + 0.75 * fade), 0.8 * DpiScale) + 1.0;
    float2 offset = nrm * c.y * widthPx * 0.5 + dir * (atB ? 0.5 : -0.5);

    o.Pos = cp;
    o.Pos.xy += offset * 2.0 / Screen * cp.w;
    float a2 = fade * fade;
    o.Color = p.Color * Intensity * (0.04 + 0.96 * a2) * (1.0 + 2.5 * pow(fade, 24.0));
    o.Across = c.y;
    return o;
}

float4 PSMain(VSOut i) : SV_Target
{
    float f = exp(-i.Across * i.Across * 3.0);
    return float4(i.Color * f, 0);
}
