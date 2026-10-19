namespace Sdcb.NBody.Scenarios;

internal static class GalaxyScenarios
{
    public static IEnumerable<Scenario> All()
    {
        yield return new DelegateScenario(new ScenarioInfo("galaxy-collision", ScenarioCategory.Galaxy, "星系碰撞", "Galaxy Collision",
            "两个盘状星系沿抛物线轨道相遇。潮汐力撕出长长的星流与潮汐尾，两颗中心黑洞最终走向合并。全部引力在 GPU 上逐对计算。",
            "Two disk galaxies meet on a parabolic orbit. Tides rip out long streams and tails while the central black holes spiral together. Every pairwise force is computed on the GPU."),
            (budget, seed) => Collision(budget, seed, massRatio: 1.0, q: 3.0, r0: 11, viewRadius: 8, tiltB: -62));
        yield return new DelegateScenario(new ScenarioInfo("milky-way-andromeda", ScenarioCategory.Galaxy, "银河系与仙女座", "Milky Way × Andromeda",
            "约 45 亿年后的预演：质量更大的仙女座星系（右）与银河系迎面相撞，最终融合为一个椭圆星系。",
            "A preview of 4.5 billion years from now: the more massive Andromeda meets the Milky Way, and they merge into an elliptical."),
            (budget, seed) => Collision(budget, seed + 7, massRatio: 1.5, q: 3.4, r0: 13, viewRadius: 10, tiltB: 77));
        yield return new DelegateScenario(new ScenarioInfo("spiral-galaxy", ScenarioCategory.Galaxy, "旋涡星系", "Spiral Galaxy",
            "一个带有中心黑洞的冷盘星系。自引力会自发激发棒结构与旋臂——这些都不是预设的动画，而是引力的产物。",
            "A cold disk around a central black hole. Self-gravity spontaneously grows a bar and spiral arms — not an animation, just gravity."),
            (budget, seed) => Spiral(budget, seed));
    }

    internal readonly record struct DiskParams(double BhMass, double DiskMass, double Rd, double RMax, double Arms, Rgb Warm, Rgb Cool, double HaloMass = 0, double HaloScale = 1, double Dispersion = 0.08)
    {
        public double TotalMass => BhMass + DiskMass + HaloMass;
    }

    /// <summary>Appends an exponential disk + bulge galaxy around a central black hole.</summary>
    internal static Halo? AddDiskGalaxy(BodySet b, int n, DiskParams p, double softening, Random rng,
        (double x, double y, double z) center, (double x, double y, double z) velocity, double tiltDeg, double azimuthDeg, int spin)
    {
        double tilt = tiltDeg * Math.PI / 180, az = azimuthDeg * Math.PI / 180;
        double ct = Math.Cos(tilt), st = Math.Sin(tilt), ca = Math.Cos(az), sa = Math.Sin(az);
        (double, double, double) Rot(double x, double y, double z)
        {
            // tilt about X, then rotate about Z
            double y1 = y * ct - z * st, z1 = y * st + z * ct;
            return (x * ca - y1 * sa, x * sa + y1 * ca, z1);
        }

        int bh = b.Add(center.x, center.y, center.z, velocity.x, velocity.y, velocity.z, p.BhMass, BodyKind.BlackHole, (float)(p.Rd * 0.09), new Rgb(1.2f, 0.7f, 0.4f));
        Halo? halo = p.HaloMass > 0 ? new Halo(bh, p.HaloMass, p.HaloScale) : null;

        double eps2 = softening * softening;
        double MassWithin(double r)
        {
            double x = r / p.Rd;
            return p.BhMass + p.DiskMass * (1 - (1 + x) * Math.Exp(-x)) + (halo?.MassWithin(r) ?? 0);
        }

        int bulge = n / 10;
        double m = p.DiskMass / n;
        for (int i = 0; i < n; ++i)
        {
            double x, y, z, vx, vy, vz;
            Rgb color;
            if (i < bulge)
            {
                // Hernquist bulge (scale 0.2 Rd), truncated by rejection so nothing piles up at the cut
                double r;
                do
                {
                    double s = Math.Sqrt(rng.NextDouble());
                    r = p.Rd * 0.2 * s / Math.Max(1 - s, 1e-9);
                } while (r > p.Rd * 1.6);
                double cz = rng.NextDouble() * 2 - 1, phi = rng.NextDouble() * 2 * Math.PI, sz = Math.Sqrt(1 - cz * cz);
                x = r * sz * Math.Cos(phi); y = r * sz * Math.Sin(phi); z = r * cz * 0.6;
                double sigma = Math.Sqrt(MassWithin(r) * r * r / Math.Pow(r * r + eps2, 1.5)) * 0.55;
                double vc = Math.Sqrt(MassWithin(r) * r * r / Math.Pow(r * r + eps2, 1.5));
                vx = sigma * rng.NextGaussian() - spin * 0.4 * vc * Math.Sin(phi);
                vy = sigma * rng.NextGaussian() + spin * 0.4 * vc * Math.Cos(phi);
                vz = sigma * rng.NextGaussian() * 0.7;
                color = Rgb.Lerp(p.Warm, new Rgb(1.0f, 0.8f, 0.55f), (float)rng.NextDouble() * 0.5f) * (float)(0.45 + 0.3 * rng.NextDouble());
            }
            else
            {
                double r, theta;
                while (true)
                {
                    r = -p.Rd * Math.Log(Math.Max(rng.NextDouble() * rng.NextDouble(), 1e-12));
                    if (r > p.RMax || r < p.Rd * 0.08) continue;
                    theta = rng.NextDouble() * 2 * Math.PI;
                    if (p.Arms <= 0) break;
                    double arm = Math.Cos(2 * (theta - Math.Log(r / p.Rd) / Math.Tan(0.3)));
                    if (rng.NextDouble() * (1 + p.Arms) < 1 + p.Arms * arm) break;
                }
                double h = 0.05 * p.Rd * (1 + 0.4 * r / p.Rd);
                x = r * Math.Cos(theta); y = r * Math.Sin(theta); z = h * rng.NextGaussian();
                double vc = Math.Sqrt(MassWithin(r) * r * r / Math.Pow(r * r + eps2, 1.5));
                double vr = p.Dispersion * vc * rng.NextGaussian(), vt = vc * (1 + 0.05 * rng.NextGaussian());
                vx = vr * Math.Cos(theta) - spin * vt * Math.Sin(theta);
                vy = vr * Math.Sin(theta) + spin * vt * Math.Cos(theta);
                vz = 0.03 * vc * rng.NextGaussian();

                double t = Math.Clamp(r / p.RMax * 1.3 + 0.15 * rng.NextGaussian(), 0, 1);
                color = Rgb.Lerp(p.Warm, p.Cool, (float)t);
                double roll = rng.NextDouble();
                if (roll < 0.03) color = new Rgb(1.5f, 0.45f, 0.75f);          // HII regions
                else if (roll < 0.10) color = new Rgb(0.6f, 0.8f, 1.6f);       // young blue stars
                color *= (float)(0.75 + 0.5 * rng.NextDouble());
            }
            var (rx, ry, rz) = Rot(x, y, z);
            var (rvx, rvy, rvz) = Rot(vx, vy, vz);
            b.Add(center.x + rx, center.y + ry, center.z + rz, velocity.x + rvx, velocity.y + rvy, velocity.z + rvz, m, BodyKind.Particle, 0.01f, color);
        }
        return halo;
    }

    private static ScenarioSetup Collision(int budget, int seed, double massRatio, double q, double r0, double viewRadius, double tiltB)
    {
        Random rng = new(seed);
        int nA = (int)(budget / (1 + massRatio)) - 1, nB = budget - nA - 2;
        DiskParams a = new(0.15, 1.0, 0.5, 2.6, 0.35, new Rgb(1.25f, 0.95f, 0.65f), new Rgb(0.55f, 0.7f, 1.25f), HaloMass: 5.0, HaloScale: 1.5);
        DiskParams bp = new(0.15 * massRatio, 1.0 * massRatio, 0.5 * Math.Sqrt(massRatio), 2.6 * Math.Sqrt(massRatio), 0.35,
            new Rgb(1.3f, 0.85f, 0.55f), new Rgb(0.75f, 0.6f, 1.3f), HaloMass: 5.0 * massRatio, HaloScale: 1.5 * Math.Sqrt(massRatio));
        double mA = a.TotalMass, mB = bp.TotalMass, mt = mA + mB;
        // parabolic relative orbit with pericentre q, starting at separation r0 (approaching)
        double f0 = -Math.Acos(2 * q / r0 - 1);
        double pp = 2 * q, k = Math.Sqrt(mt / pp) * 0.72; // bound orbit: they come back for a second passage and merge
        double rx = r0 * Math.Cos(f0), ry = r0 * Math.Sin(f0);
        double vx = -k * Math.Sin(f0), vy = k * (1 + Math.Cos(f0));
        // rotate the orbit so the encounter happens across the screen
        double rot = -Math.PI / 2 - f0 * 0.55;
        (rx, ry) = (rx * Math.Cos(rot) - ry * Math.Sin(rot), rx * Math.Sin(rot) + ry * Math.Cos(rot));
        (vx, vy) = (vx * Math.Cos(rot) - vy * Math.Sin(rot), vx * Math.Sin(rot) + vy * Math.Cos(rot));

        const double soft = 0.035;
        BodySet b = new(budget);
        Halo? hA = AddDiskGalaxy(b, nA, a, soft, rng, (-rx * mB / mt, -ry * mB / mt, 0), (-vx * mB / mt, -vy * mB / mt, 0), 25, 10, 1);
        Halo? hB = AddDiskGalaxy(b, nB, bp, soft, rng, (rx * mA / mt, ry * mA / mt, 0), (vx * mA / mt, vy * mA / mt, 0), tiltB, 120, -1);
        b.ShiftToCenterOfMassFrame();
        return new ScenarioSetup
        {
            Bodies = b,
            Softening = soft,
            Integrator = IntegratorKind.Leapfrog,
            Dt = 0.015,
            TimeScale = 0.9,
            Halos = [hA!, hB!],
            HaloFriction = 15,
            ViewRadius = viewRadius,
            TrailDuration = 0,
            GpuSimulated = true,
            CameraPitchDegrees = 48,
        };
    }

    private static ScenarioSetup Spiral(int budget, int seed)
    {
        Random rng = new(seed);
        const double soft = 0.03;
        BodySet b = new(budget);
        DiskParams p = new(0.2, 1.0, 0.6, 3.0, 0.6, new Rgb(1.3f, 0.92f, 0.6f), new Rgb(0.5f, 0.68f, 1.35f), HaloMass: 4.0, HaloScale: 1.5, Dispersion: 0.16);
        Halo? halo = AddDiskGalaxy(b, budget - 1, p, soft, rng, (0, 0, 0), (0, 0, 0), 0, 0, 1);
        return new ScenarioSetup
        {
            Bodies = b,
            Softening = soft,
            Integrator = IntegratorKind.Leapfrog,
            Halos = [halo!],
            Dt = 0.012,
            TimeScale = 0.7,
            ViewRadius = 3.6,
            TrailDuration = 0,
            GpuSimulated = true,
            CameraPitchDegrees = 58,
        };
    }
}
