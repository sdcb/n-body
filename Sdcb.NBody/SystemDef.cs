namespace Sdcb.NBody;

public record SystemDef(BodyDef[] Bodies, double G = 1, double Dt = 1.0 / 80)
{
    public Body[] ToBodies() => [.. Bodies.Select((x, i) => x.Create(i))];

    public static SystemDef CreateStableRing(int N, double scale = 1.0, double G = 1.0, double M = 1.0, double R = 1.0)
    {
        if (N < 2)
        {
            throw new ArgumentException("星体数量 N 必须大于等于 2", nameof(N));
        }

        // 步骤 1: 计算引力求和因子
        // 这个因子源于 F_grav = (G*M^2 / (4*R^2)) * sum_factor
        // sum_{k=1}^{N-1} (1 / sin(pi*k/N))
        double forceSumFactor = 0;
        for (int k = 1; k < N; ++k)
        {
            forceSumFactor += 1.0 / Math.Sin(Math.PI * k / N);
        }

        // 步骤 2: 计算稳定轨道所需的速度的平方
        // v^2 = (G * M / (4 * R)) * sum_factor
        double v_squared = (G * M / (4.0 * R)) * forceSumFactor;
        double v = Math.Sqrt(v_squared) * scale;

        // 步骤 3: 创建星体数组
        var stars = new BodyDef[N];
        for (var i = 0; i < N; ++i)
        {
            double angle = 2.0 * Math.PI * i / N;

            // 位置：(R*cos(angle), R*sin(angle)) 是标准数学坐标
            // 你的代码中使用 (R*sin, -R*cos) 只是旋转了90度并翻转了y轴，物理上是等价的
            double px = R * Math.Sin(angle);
            double py = R * -Math.Cos(angle);

            // 速度：方向与位置向量垂直（即切向）
            double vx = v * Math.Cos(angle);
            double vy = v * Math.Sin(angle);

            stars[i] = new BodyDef(new BodyState(px, py, vx, vy), BodyType.Solar, M);
        }

        // 使用你的StarSystem构造函数创建实例
        // 这里的 dt 只是一个初始值，因为 Ode45 是自适应的
        return new SystemDef(stars, G);
    }

    public static SystemDef CreateSolarEarthMoon1() => new([
        new(0.3534,  0, 0, -0.2658, BodyType.Solar, 1.0),
        new(-1.1466, 0, 0, 0.8183 , BodyType.Planet, 0.3),
        new(-0.9466, 0, 0, 2.0430 , BodyType.Moon, 0.01)
    ]);


    public static SystemDef CreateSolarEarthMoon2(double dt = 0.001953125) => new([
        new(-0.2013, 0, 0, 0.16041, BodyType.Solar, 2.0),
        new(1.2987, 0, 0, -1.0744, BodyType.Planet, 0.3),
        new(1.4987, 0, 0, 0.1503, BodyType.Moon, 0.01)
    ]);

    public static SystemDef CreateBrouckeA15(double dt = 0.001953125) => new([
        new(-1.1889693067, 0, 0, 0.8042120498),
        new(3.8201881837, 0, 0, 0.0212794833),
        new(-2.631218877, 0, 0, -0.8254915331)
    ]);

    public static SystemDef CreateHenon2(double dt = 0.0001) => new([
        new(-1.0207041786, 0, 0, 9.1265693140),
        new(2.0532718983, 0, 0, 0.0660238922),
        new(-1.0325677197, 0, 0, -9.1925932061)
    ]);

    public static SystemDef CreateHenon3(double dt = 0.001953125) => new([
        new(-0.9738300580, 0, 0, 4.3072892019),
        new(1.9988948637, 0, 0, 0.1333821680),
        new(-1.0250648057, 0, 0, -4.4406713699)
    ]);

    public static SystemDef CreateHenon4(double dt = 0.001953125) => new([
        new(-1.1889693067, 0, 0, 0.8042120498),
        new(3.8201881837, 0, 0, 0.0212794833),
        new(-2.631218877, 0, 0, -0.8254915331)
    ]);

    public static SystemDef CreateHenon5(double dt = 0.001953125) => new([
        new(-0.9353825545, 0, 0, 3.3166932522),
        new(1.9545571553, 0, 0, 0.1654488998),
        new(-1.0191746008, 0, 0, -3.4821421520)
    ]);

    public static SystemDef CreateHenon42(double dt = 0.001953125) => new([
        new(1.1593879407, 0, 0, 1.1787714143),
        new(1.7740754142, 0, 0, -0.6271771385),
        new(-2.9334633549, 0, 0, -0.5515942758)
    ]);

    public static SystemDef CreateFigure8(double dt = 0.001953125) => new([
        new(-1, 0, 0.347111, 0.532728),
        new(1, 0, 0.347111, 0.532728),
        new(0, 0, -2 * 0.347111, -2 * 0.532728)]);

    public static SystemDef CreateFreeFallF1(double dt = 0.001953125 / 128) => new([
        new(-0.5, 0, 0.3471168881, 0.5327249454),
        new(0.5, 0, 0.3471168881, 0.5327249454),
        new(-0.0009114239, 0.3019805958, -0.6942337762, -1.0654498908)]);
}