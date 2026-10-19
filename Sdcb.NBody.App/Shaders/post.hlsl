#include "common.hlsli"

Texture2D Src : register(t0);
Texture2D Bloom : register(t1);
Texture2D Glass : register(t2);
SamplerState LinearClamp : register(s0);

#define MAX_PANELS 16

cbuffer PostParams : register(b1)
{
    float2 SrcTexel;    float BloomStrength;  float Exposure;
    float Fade;         float Vignette;       float Grain;   float Aberration;
    uint PanelCount;    float BloomNorm; float2 _pad0;
    float4 Panels[MAX_PANELS];      // x, y, w, h in pixels
    float4 PanelStyle[MAX_PANELS];  // corner radius px, opacity, highlight, unused
};

struct VSOut { float4 Pos : SV_Position; float2 Uv : TEXCOORD0; };

VSOut VSFullscreen(uint id : SV_VertexID)
{
    VSOut o;
    o.Uv = float2((id << 1) & 2, id & 2);
    o.Pos = float4(o.Uv * float2(2, -2) + float2(-1, 1), 0, 1);
    return o;
}

float3 Sample(float2 uv) { return Src.SampleLevel(LinearClamp, uv, 0).rgb; }
float3 Sanitize(float3 c) { return any(isnan(c)) ? 0 : clamp(c, 0, 3e4); }
float3 SafeSample(float2 uv) { return Sanitize(Sample(uv)); }

float Luma(float3 c) { return dot(c, float3(0.2126, 0.7152, 0.0722)); }
float3 KarisWeight(float3 c) { return c / (1.0 + Luma(c)); }

// 13-tap downsample (Call of Duty: Advanced Warfare)
float3 Downsample13(float2 uv, bool karis)
{
    float2 t = SrcTexel;
    float3 a = SafeSample(uv + t * float2(-2, -2)), b = SafeSample(uv + t * float2(0, -2)), c = SafeSample(uv + t * float2(2, -2));
    float3 d = SafeSample(uv + t * float2(-2, 0)), e = SafeSample(uv), f = SafeSample(uv + t * float2(2, 0));
    float3 g = SafeSample(uv + t * float2(-2, 2)), h = SafeSample(uv + t * float2(0, 2)), i = SafeSample(uv + t * float2(2, 2));
    float3 j = SafeSample(uv + t * float2(-1, -1)), k = SafeSample(uv + t * float2(1, -1));
    float3 l = SafeSample(uv + t * float2(-1, 1)), m = SafeSample(uv + t * float2(1, 1));
    if (karis)
    {
        // Karis average: weight each 2x2 group by 1/(1+luma) to suppress fireflies, then renormalise
        float3 g0 = (a + b + d + e) * 0.25, g1 = (b + c + e + f) * 0.25;
        float3 g2 = (d + e + g + h) * 0.25, g3 = (e + f + h + i) * 0.25;
        float3 g4 = (j + k + l + m) * 0.25;
        float w0 = 0.125 / (1 + Luma(g0)), w1 = 0.125 / (1 + Luma(g1)), w2 = 0.125 / (1 + Luma(g2)), w3 = 0.125 / (1 + Luma(g3)), w4 = 0.5 / (1 + Luma(g4));
        return (g0 * w0 + g1 * w1 + g2 * w2 + g3 * w3 + g4 * w4) / (w0 + w1 + w2 + w3 + w4);
    }
    return e * 0.125 + (a + c + g + i) * 0.03125 + (b + d + f + h) * 0.0625 + (j + k + l + m) * 0.125;
}

float4 PSDownsampleFirst(VSOut i) : SV_Target { return float4(min(Downsample13(i.Uv, true), 6e4), 1); }
float4 PSDownsample(VSOut i) : SV_Target { return float4(Downsample13(i.Uv, false), 1); }

// 3x3 tent upsample; additively blended onto the larger mip
float4 PSUpsample(VSOut i) : SV_Target
{
    float2 t = SrcTexel;
    float3 s = SafeSample(i.Uv) * 4.0;
    s += (SafeSample(i.Uv + t * float2(-1, 0)) + SafeSample(i.Uv + t * float2(1, 0)) + SafeSample(i.Uv + t * float2(0, -1)) + SafeSample(i.Uv + t * float2(0, 1))) * 2.0;
    s += SafeSample(i.Uv + t * float2(-1, -1)) + SafeSample(i.Uv + t * float2(1, -1)) + SafeSample(i.Uv + t * float2(-1, 1)) + SafeSample(i.Uv + t * float2(1, 1));
    return float4(s / 16.0, 1);
}

float3 ACESFitted(float3 x)
{
    const float3x3 inM = { 0.59719, 0.35458, 0.04823, 0.07600, 0.90834, 0.01566, 0.02840, 0.13383, 0.83777 };
    const float3x3 outM = { 1.60475, -0.53108, -0.07367, -0.10208, 1.10813, -0.00605, -0.00327, -0.07276, 1.07602 };
    x = mul(inM, x);
    float3 a = x * (x + 0.0245786) - 0.000090537;
    float3 b = x * (0.983729 * x + 0.4329510) + 0.238081;
    return saturate(mul(outM, a / b));
}

float3 LinearToSrgb(float3 c) { return c <= 0.0031308 ? c * 12.92 : 1.055 * pow(c, 1.0 / 2.4) - 0.055; }

float RoundedRectSdf(float2 p, float2 center, float2 half, float r)
{
    float2 q = abs(p - center) - half + r;
    return length(max(q, 0)) + min(max(q.x, q.y), 0) - r;
}

float4 PSComposite(VSOut i) : SV_Target
{
    float2 uv = i.Uv;
    float2 fromCenter = uv - 0.5;
    float r2 = dot(fromCenter, fromCenter);

    float3 hdr;
    float ab = Aberration * r2;
    hdr.r = Src.SampleLevel(LinearClamp, uv - fromCenter * ab, 0).r;
    hdr.g = Src.SampleLevel(LinearClamp, uv, 0).g;
    hdr.b = Src.SampleLevel(LinearClamp, uv + fromCenter * ab, 0).b;
    float3 bloom = Sanitize(Bloom.SampleLevel(LinearClamp, uv, 0).rgb) * BloomNorm;
    hdr = lerp(hdr, bloom, BloomStrength);
    hdr = Sanitize(hdr);
    hdr *= Exposure;

    float3 col = ACESFitted(hdr);
    col *= 1.0 - Vignette * smoothstep(0.15, 0.85, r2 * 1.6);

    // frosted glass panels behind the HUD
    float2 px = i.Pos.xy;
    for (uint k = 0; k < PanelCount; ++k)
    {
        float4 rc = Panels[k];
        float4 st = PanelStyle[k];
        float d = RoundedRectSdf(px, rc.xy + rc.zw * 0.5, rc.zw * 0.5, st.x);
        float inside = saturate(0.5 - d);
        if (inside > 0)
        {
            float3 blurred = ACESFitted(Sanitize(Glass.SampleLevel(LinearClamp, uv, 0).rgb) * BloomNorm * 2.0 * Exposure);
            float3 glass = blurred * 0.45 + float3(0.012, 0.016, 0.028);
            float sheen = st.z * saturate(1 - (px.y - rc.y) / max(rc.w, 1)) * 0.05;
            col = lerp(col, glass + sheen, inside * st.y);
        }
    }

    col = LinearToSrgb(col);
    float n = Hash31(float3(px, frac(Time) * 100.0));
    col += (n - 0.5) * Grain;
    col *= Fade;
    return float4(col, 1);
}
