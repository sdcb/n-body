namespace Sdcb.NBody.Solvers;

/// <summary>
/// 使用经典的 6 阶段、5 阶龙格-库塔方法的固定步长求解器。
/// </summary>
public class RK5 : Solver
{
    // 存储中间计算结果的数组
    private readonly BodyState[] k1, k2, k3, k4, k5, k6;
    private readonly BodyState[] w; // 用于存储中间状态

    // RK5 系数 (一个常见的 6 阶段、5 阶方法)
    // 权重 b_i
    private const double b1 = 7.0 / 90.0;
    // b2 is 0
    private const double b3 = 32.0 / 90.0; // 16.0 / 45.0
    private const double b4 = 12.0 / 90.0; // 2.0 / 15.0
    private const double b5 = 32.0 / 90.0; // 16.0 / 45.0
    private const double b6 = 7.0 / 90.0;

    public RK5(Body[] stars, double initialDt) : base(stars, initialDt)
    {
        int starCount = stars.Length;
        k1 = new BodyState[starCount];
        k2 = new BodyState[starCount];
        k3 = new BodyState[starCount];
        k4 = new BodyState[starCount];
        k5 = new BodyState[starCount];
        k6 = new BodyState[starCount];
        w = new BodyState[starCount];
    }

    public override double Step(ReadOnlySpan<BodyState> current)
    {
        // k1 = f(y_n)
        NewtonsLaw(current, k1);

        // k2 = f(y_n + dt * (1/4 * k1))
        for (int i = 0; i < _stars.Length; ++i)
        {
            w[i] = current[i] + 1.0 / 4.0 * dt * k1[i];
        }
        NewtonsLaw(w, k2);

        // k3 = f(y_n + dt * (1/8 * k1 + 1/8 * k2))
        for (int i = 0; i < _stars.Length; ++i)
        {
            w[i] = current[i] + 1.0 / 8.0 * dt * k1[i] + 1.0 / 8.0 * dt * k2[i];
        }
        NewtonsLaw(w, k3);

        // k4 = f(y_n + dt * (-1/2 * k2 + 1 * k3))
        for (int i = 0; i < _stars.Length; ++i)
        {
            // 注意这里 k2 的系数是负数
            w[i] = current[i] - 1.0 / 2.0 * dt * k2[i] + 1.0 * dt * k3[i];
        }
        NewtonsLaw(w, k4);

        // k5 = f(y_n + dt * (3/16 * k1 + 9/16 * k4))
        for (int i = 0; i < _stars.Length; ++i)
        {
            w[i] = current[i] + 3.0 / 16.0 * dt * k1[i] + 9.0 / 16.0 * dt * k4[i];
        }
        NewtonsLaw(w, k5);

        // k6 = f(y_n + dt * (-3/7 * k1 + 2/7 * k2 + 12/7 * k3 - 12/7 * k4 + 8/7 * k5))
        for (int i = 0; i < _stars.Length; ++i)
        {
            w[i] = current[i] +
                -3.0 / 7.0 * dt * k1[i] +
                2.0 / 7.0 * dt * k2[i] +
                12.0 / 7.0 * dt * k3[i] -
                12.0 / 7.0 * dt * k4[i] +
                8.0 / 7.0 * dt * k5[i];
        }
        NewtonsLaw(w, k6);

        // 使用加权平均更新最终状态
        // y_{n+1} = y_n + dt * (b1*k1 + b2*k2 + b3*k3 + b4*k4 + b5*k5 + b6*k6)
        // 注意 b2 的权重为 0
        for (int i = 0; i < _stars.Length; ++i)
        {
            BodyState delta = b1 * k1[i] + b3 * k3[i] + b4 * k4[i] + b5 * k5[i] + b6 * k6[i];
            _stars[i].State += delta * dt;
        }

        // 返回固定的步长
        return dt;
    }
}
