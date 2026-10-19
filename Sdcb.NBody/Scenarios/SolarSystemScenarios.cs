namespace Sdcb.NBody.Scenarios;

/// <summary>Solar system from J2000 mean orbital elements (units: AU, years, solar masses → G = 4π²).</summary>
internal static class SolarSystemScenarios
{
    public const double G = 4 * Math.PI * Math.PI;

    private sealed record Planet(string Name, double A, double E, double I, double L, double LongPeri, double Node, double Mass, float Radius, Rgb Color, BodyKind Kind = BodyKind.Planet);

    // Standish (JPL) J2000 elements: a [AU], e, i [deg], L [deg], ϖ [deg], Ω [deg]; masses in solar masses.
    private static readonly Planet[] Planets =
    [
        new("Mercury", 0.38709927, 0.20563593, 7.00497902, 252.25032350, 77.45779628, 48.33076593, 1.6601e-7, 0.022f, new(0.75f, 0.68f, 0.62f)),
        new("Venus", 0.72333566, 0.00677672, 3.39467605, 181.97909950, 131.60246718, 76.67984255, 2.4478e-6, 0.034f, new(1.05f, 0.85f, 0.55f)),
        new("Earth", 1.00000261, 0.01671123, -0.00001531, 100.46457166, 102.93768193, 0.0, 3.0404e-6, 0.036f, new(0.35f, 0.62f, 1.2f)),
        new("Mars", 1.52371034, 0.09339410, 1.84969142, -4.55343205, -23.94362959, 49.55953891, 3.2272e-7, 0.028f, new(1.1f, 0.45f, 0.25f)),
        new("Jupiter", 5.20288700, 0.04838624, 1.30439695, 34.39644051, 14.72847983, 100.47390909, 9.5479e-4, 0.20f, new(1.0f, 0.78f, 0.55f)),
        new("Saturn", 9.53667594, 0.05386179, 2.48599187, 49.95424423, 92.59887831, 113.66242448, 2.8589e-4, 0.17f, new(1.0f, 0.88f, 0.62f)),
        new("Uranus", 19.18916464, 0.04725744, 0.77263783, 313.23810451, 170.95427630, 74.01692503, 4.3662e-5, 0.13f, new(0.55f, 0.92f, 1.0f)),
        new("Neptune", 30.06992276, 0.00859048, 1.77004347, -55.12002969, 44.96476227, 131.78422574, 5.1514e-5, 0.13f, new(0.35f, 0.52f, 1.2f)),
    ];

    public static IEnumerable<Scenario> All()
    {
        yield return new DelegateScenario(new ScenarioInfo("solar-system", ScenarioCategory.SolarSystem, "太阳系", "The Solar System",
            "基于 JPL J2000 轨道根数的真实太阳系：八大行星的位置、质量与倾角均为实测值。1 秒 ≈ 4 年。",
            "The real Solar System from JPL J2000 elements — true masses, positions and inclinations. 1 s ≈ 4 years."),
            (_, _) => Create(Planets.Length, scale: 1.0, viewRadius: 32, timeScale: 4, sunRadius: 0.5f, radiusScale: 2.2f, pitch: 28));
        yield return new DelegateScenario(new ScenarioInfo("inner-solar-system", ScenarioCategory.SolarSystem, "内太阳系", "Inner Solar System",
            "水星、金星、地球与火星。注意水星的偏心轨道和地球一年一圈的节奏。",
            "Mercury, Venus, Earth and Mars. Watch Mercury's eccentric orbit and Earth's yearly lap."),
            (_, _) => Create(4, scale: 1.0, viewRadius: 1.75, timeScale: 0.25, sunRadius: 0.09f, radiusScale: 0.55f, pitch: 32));
        yield return new DelegateScenario(new ScenarioInfo("sun-earth-moon", ScenarioCategory.SolarSystem, "日-地-月（夸张模型）", "Sun, Earth & Moon (toy)",
            "原项目中的日地月玩具模型：放大的质量比让月球的回旋轨迹清晰可见。",
            "The original project's toy Sun–Earth–Moon: exaggerated masses make the Moon's looping path visible."),
            (_, _) => SunEarthMoon());
    }

    private static ScenarioSetup Create(int planetCount, double scale, double viewRadius, double timeScale, float sunRadius, float radiusScale, float pitch)
    {
        BodySet b = new(planetCount + 1);
        b.Add(0, 0, 0, 0, 0, 0, 1.0, BodyKind.Star, sunRadius, new Rgb(1.7f, 1.15f, 0.55f), "Sun");
        for (int i = 0; i < planetCount; ++i)
        {
            Planet p = Planets[i];
            var (x, y, z, vx, vy, vz) = ElementsToState(p.A * scale, p.E, p.I, p.L, p.LongPeri, p.Node, G * (1 + p.Mass));
            b.Add(x, y, z, vx, vy, vz, p.Mass, p.Kind, p.Radius * radiusScale, p.Color, p.Name);
        }
        b.ShiftToCenterOfMassFrame();
        double outerPeriod = Math.Pow(Planets[planetCount - 1].A, 1.5);
        return new ScenarioSetup
        {
            Bodies = b,
            G = G,
            Integrator = IntegratorKind.Yoshida4,
            Dt = 0.0015,
            TimeScale = timeScale,
            ViewRadius = viewRadius,
            TrailDuration = outerPeriod,
            Period = null,
            CameraPitchDegrees = pitch,
        };
    }

    /// <summary>Converts Keplerian elements (angles in degrees) to heliocentric ecliptic state.</summary>
    public static (double X, double Y, double Z, double Vx, double Vy, double Vz) ElementsToState(
        double a, double e, double iDeg, double lDeg, double longPeriDeg, double nodeDeg, double mu)
    {
        const double D = Math.PI / 180;
        double i = iDeg * D, node = nodeDeg * D, argPeri = (longPeriDeg - nodeDeg) * D;
        double meanAnomaly = ((lDeg - longPeriDeg) % 360) * D;
        double ecc = meanAnomaly + e * Math.Sin(meanAnomaly);
        for (int k = 0; k < 50; ++k)
        {
            double d = (ecc - e * Math.Sin(ecc) - meanAnomaly) / (1 - e * Math.Cos(ecc));
            ecc -= d;
            if (Math.Abs(d) < 1e-15) break;
        }
        double cosE = Math.Cos(ecc), sinE = Math.Sin(ecc);
        double n = Math.Sqrt(mu / (a * a * a));
        double xp = a * (cosE - e), yp = a * Math.Sqrt(1 - e * e) * sinE;
        double denom = 1 - e * cosE;
        double vxp = -a * n * sinE / denom, vyp = a * n * Math.Sqrt(1 - e * e) * cosE / denom;

        double cO = Math.Cos(node), sO = Math.Sin(node), cw = Math.Cos(argPeri), sw = Math.Sin(argPeri), ci = Math.Cos(i), si = Math.Sin(i);
        double r11 = cO * cw - sO * sw * ci, r12 = -cO * sw - sO * cw * ci;
        double r21 = sO * cw + cO * sw * ci, r22 = -sO * sw + cO * cw * ci;
        double r31 = sw * si, r32 = cw * si;
        return (r11 * xp + r12 * yp, r21 * xp + r22 * yp, r31 * xp + r32 * yp,
                r11 * vxp + r12 * vyp, r21 * vxp + r22 * vyp, r31 * vxp + r32 * vyp);
    }

    private static ScenarioSetup SunEarthMoon()
    {
        BodySet b = new(3);
        b.Add(0.3534, 0, 0, 0, -0.2658, 0, 1.0, BodyKind.Star, 0.12f, new Rgb(1.7f, 1.15f, 0.55f), "Sun");
        b.Add(-1.1466, 0, 0, 0, 0.8183, 0, 0.3, BodyKind.Planet, 0.06f, new Rgb(0.35f, 0.62f, 1.2f), "Earth");
        b.Add(-0.9466, 0, 0, 0, 2.0430, 0, 0.01, BodyKind.Moon, 0.03f, new Rgb(0.85f, 0.85f, 0.9f), "Moon");
        b.ShiftToCenterOfMassFrame();
        return new ScenarioSetup
        {
            Bodies = b,
            Integrator = IntegratorKind.DormandPrince45,
            Dt = 0.004,
            TimeScale = 0.8,
            ViewRadius = 1.9,
            TrailDuration = 9,
            CameraPitchDegrees = 50,
        };
    }
}
