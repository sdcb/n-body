using Vortice.Multimedia;
using Vortice.XAudio2;

namespace Sdcb.NBody.App.Audio;

/// <summary>
/// Fully procedural ambient soundtrack (no audio assets): a slowly breathing Am9 pad with
/// shimmering partials, filtered "solar wind" noise and a simple feedback reverb, rendered
/// once into a seamless loop. Also synthesizes a short whoosh for scene transitions.
/// </summary>
internal sealed class AmbientAudio : IDisposable
{
    private const int SampleRate = 48000;
    private IXAudio2? _xa;
    private IXAudio2MasteringVoice? _master;
    private IXAudio2SourceVoice? _pad, _whoosh;
    private AudioBuffer? _padBuffer, _whooshBuffer;
    private float _targetVolume = 0.55f, _volume;
    private bool _muted;
    private float _cutoffTarget = 0.5f, _cutoff = 0.5f;
    private volatile bool _ready;
    private float[]? _padData, _whooshData;

    public bool Muted
    {
        get => _muted;
        set { _muted = value; _master?.SetVolume(value ? 0 : 1); }
    }

    public AmbientAudio()
    {
        try
        {
            _xa = XAudio2.XAudio2Create();
            _master = _xa.CreateMasteringVoice(2, SampleRate);
            Task.Run(() =>
            {
                _padData = RenderPad(24);
                _whooshData = RenderWhoosh(2.2f);
                _ready = true;
            });
        }
        catch
        {
            _xa = null; // no audio device; stay silent
        }
    }

    /// <summary>Call once per frame. activity in [0,1] opens the filter and raises the level.</summary>
    public void Update(float dt, float activity)
    {
        if (_xa == null || !_ready) return;
        if (_pad == null) Start();
        _cutoffTarget = 0.18f + 0.55f * Math.Clamp(activity, 0, 1);
        float k = 1 - MathF.Exp(-dt * 0.8f);
        _cutoff += (_cutoffTarget - _cutoff) * k;
        _volume += (_targetVolume - _volume) * (1 - MathF.Exp(-dt * 0.5f));
        _pad!.SetVolume(_volume);
        _pad.SetFilterParameters(new FilterParameters { Type = FilterType.LowPassFilter, Frequency = _cutoff, OneOverQ = 1.2f }, 0);
    }

    public void PlayWhoosh()
    {
        if (_whoosh == null || _whooshBuffer == null) return;
        _whoosh.Stop();
        _whoosh.FlushSourceBuffers();
        _whoosh.SubmitSourceBuffer(_whooshBuffer);
        _whoosh.Start();
    }

    private void Start()
    {
        WaveFormat fmt = WaveFormat.CreateIeeeFloatWaveFormat(SampleRate, 2);
        _pad = _xa!.CreateSourceVoice(fmt, VoiceFlags.UseFilter);
        _padBuffer = AudioBuffer.Create<float>(_padData!, BufferFlags.EndOfStream);
        _padBuffer.LoopCount = XAudio2.LoopInfinite;
        _pad.SetVolume(0);
        _pad.SubmitSourceBuffer(_padBuffer);
        _pad.Start();

        _whoosh = _xa.CreateSourceVoice(fmt);
        _whoosh.SetVolume(0.45f);
        _whooshBuffer = AudioBuffer.Create<float>(_whooshData!, BufferFlags.EndOfStream);
    }

    private static float[] RenderPad(int seconds)
    {
        const int Xfade = SampleRate * 4;
        int loop = seconds * SampleRate;
        int total = loop + Xfade;
        float[] l = new float[total], r = new float[total];
        Random rng = new(7);

        const int TableSize = 4096;
        float[] sine = new float[TableSize + 1];
        for (int i = 0; i <= TableSize; ++i) sine[i] = MathF.Sin(2 * MathF.PI * i / TableSize);

        // A minor 9 voicing spread across octaves
        double[] notes = [55.0, 82.41, 110.0, 130.81, 164.81, 196.0, 246.94, 329.63];
        foreach (double f0 in notes)
        {
            for (int p = 0; p < 2; ++p)
            {
                double f = f0 * (1 + (p == 0 ? -1 : 1) * 0.0012);
                double pan = 0.5 + (rng.NextDouble() - 0.5) * 0.8;
                double lfoRate = 0.02 + rng.NextDouble() * 0.06;
                double lfoPhase = rng.NextDouble() * Math.PI * 2;
                double amp = 0.05 / Math.Sqrt(f0 / 55.0);
                double gl = Math.Sqrt(1 - pan) * amp, gr = Math.Sqrt(pan) * amp;
                double ph1 = rng.NextDouble(), ph2 = rng.NextDouble(), ph3 = rng.NextDouble();
                double inc = f / SampleRate;
                for (int i = 0; i < total; ++i)
                {
                    double t = (double)i / SampleRate;
                    double env = 0.55 + 0.45 * Math.Sin(2 * Math.PI * lfoRate * t + lfoPhase);
                    env *= env;
                    float s = Lookup(sine, ph1) + 0.35f * Lookup(sine, ph2) + 0.12f * Lookup(sine, ph3);
                    ph1 += inc; ph2 += inc * 2; ph3 += inc * 3.0;
                    if (ph1 >= 1) ph1 -= 1; if (ph2 >= 1) ph2 -= 1; if (ph3 >= 1) ph3 -= 1;
                    l[i] += (float)(s * env * gl);
                    r[i] += (float)(s * env * gr);
                }
            }
        }

        // shimmering high partials that fade in and out
        for (int k = 0; k < 6; ++k)
        {
            double f = notes[3 + rng.Next(5)] * 4 * (1 + (rng.NextDouble() - 0.5) * 0.002);
            double center = rng.NextDouble() * seconds, width = 2 + rng.NextDouble() * 4;
            double pan = rng.NextDouble();
            double ph = 0, inc = f / SampleRate;
            for (int i = 0; i < total; ++i)
            {
                double t = (double)i / SampleRate;
                double d = (t - center) / width;
                double env = Math.Exp(-d * d) * 0.012;
                float s = Lookup(sine, ph);
                ph += inc; if (ph >= 1) ph -= 1;
                l[i] += (float)(s * env * (1 - pan));
                r[i] += (float)(s * env * pan);
            }
        }

        // filtered noise "solar wind"
        double lpL = 0, lpR = 0, lp2L = 0, lp2R = 0;
        for (int i = 0; i < total; ++i)
        {
            double t = (double)i / SampleRate;
            double c = 0.004 + 0.003 * (0.5 + 0.5 * Math.Sin(2 * Math.PI * 0.045 * t));
            lpL += c * ((rng.NextDouble() * 2 - 1) - lpL);
            lpR += c * ((rng.NextDouble() * 2 - 1) - lpR);
            lp2L += c * (lpL - lp2L);
            lp2R += c * (lpR - lp2R);
            l[i] += (float)(lp2L * 0.9);
            r[i] += (float)(lp2R * 0.9);
        }

        Reverb(l, r);

        // seamless loop: crossfade the tail into the head (equal power)
        float[] outBuf = new float[loop * 2];
        float peak = 1e-6f;
        for (int i = 0; i < loop; ++i)
        {
            float a = l[i], b = r[i];
            if (i < Xfade)
            {
                float x = (float)i / Xfade;
                float gIn = MathF.Sin(x * MathF.PI / 2), gOut = MathF.Cos(x * MathF.PI / 2);
                a = a * gIn + l[loop + i] * gOut;
                b = b * gIn + r[loop + i] * gOut;
            }
            outBuf[i * 2] = a;
            outBuf[i * 2 + 1] = b;
            peak = MathF.Max(peak, MathF.Max(MathF.Abs(a), MathF.Abs(b)));
        }
        float norm = 0.6f / peak;
        for (int i = 0; i < outBuf.Length; ++i) outBuf[i] *= norm;
        return outBuf;
    }

    private static float Lookup(float[] table, double phase)
    {
        double idx = phase * (table.Length - 1);
        int i = (int)idx;
        float f = (float)(idx - i);
        return table[i] + (table[i + 1] - table[i]) * f;
    }

    private static void Reverb(float[] l, float[] r)
    {
        int[] combs = [1557, 1617, 1491, 1422, 1277, 1356];
        float[] wetL = new float[l.Length], wetR = new float[r.Length];
        foreach (int baseLen in combs)
        {
            int len = baseLen * 3;
            float[] bufL = new float[len], bufR = new float[len + 23 * 3];
            int il = 0, ir = 0;
            float fbL = 0, fbR = 0;
            for (int i = 0; i < l.Length; ++i)
            {
                float yl = bufL[il];
                fbL = yl * 0.6f + fbL * 0.4f;
                bufL[il] = l[i] + fbL * 0.84f;
                il = (il + 1) % bufL.Length;
                wetL[i] += yl;

                float yr = bufR[ir];
                fbR = yr * 0.6f + fbR * 0.4f;
                bufR[ir] = r[i] + fbR * 0.84f;
                ir = (ir + 1) % bufR.Length;
                wetR[i] += yr;
            }
        }
        for (int i = 0; i < l.Length; ++i)
        {
            l[i] = l[i] * 0.7f + wetL[i] * 0.08f;
            r[i] = r[i] * 0.7f + wetR[i] * 0.08f;
        }
    }

    private static float[] RenderWhoosh(float seconds)
    {
        int n = (int)(seconds * SampleRate);
        float[] data = new float[n * 2];
        Random rng = new(3);
        double bpL = 0, bpR = 0, lowL = 0, lowR = 0;
        for (int i = 0; i < n; ++i)
        {
            double t = (double)i / n;
            double env = Math.Sin(Math.PI * Math.Pow(t, 0.6)) * Math.Exp(-t * 1.5);
            double c = 0.01 + 0.08 * Math.Sin(Math.PI * t);
            double nl = rng.NextDouble() * 2 - 1, nr = rng.NextDouble() * 2 - 1;
            lowL += c * (nl - lowL); bpL += c * (lowL - bpL);
            lowR += c * (nr - lowR); bpR += c * (lowR - bpR);
            double sub = Math.Sin(2 * Math.PI * (40 + 30 * (1 - t)) * i / SampleRate) * 0.25;
            data[i * 2] = (float)((lowL - bpL) * 3.5 * env + sub * env);
            data[i * 2 + 1] = (float)((lowR - bpR) * 3.5 * env + sub * env);
        }
        return data;
    }

    public void Dispose()
    {
        _pad?.DestroyVoice();
        _whoosh?.DestroyVoice();
        _padBuffer?.Dispose();
        _whooshBuffer?.Dispose();
        _master?.DestroyVoice();
        _xa?.Dispose();
    }
}
