namespace Sdcb.NBody.Solvers;

/// <summary>
/// 使用 Dormand-Prince 5(4) 方法的自适应步长求解器。
/// 当所需步长小于设定的最小值时，将抛出异常。
/// </summary>
public class Ode45 : Solver
{
    // 自适应步长控制参数
    private readonly double _tolerance;
    private readonly double _minDt;
    private readonly double _maxDt;

    // 安全系数和步长调整因子
    private const double SafetyFactor = 0.9;
    private const double MinShrinkFactor = 0.2;
    private const double MaxGrowFactor = 5.0;

    // Dormand-Prince 5(4) 的 Butcher Tableau 系数 (与之前相同)
    #region Coefficients
    private const double c2 = 1.0 / 5.0, c3 = 3.0 / 10.0, c4 = 4.0 / 5.0, c5 = 8.0 / 9.0, c6 = 1.0, c7 = 1.0;
    private const double a21 = 1.0 / 5.0;
    private const double a31 = 3.0 / 40.0, a32 = 9.0 / 40.0;
    private const double a41 = 44.0 / 45.0, a42 = -56.0 / 15.0, a43 = 32.0 / 9.0;
    private const double a51 = 19372.0 / 6561.0, a52 = -25360.0 / 2187.0, a53 = 64448.0 / 6561.0, a54 = -212.0 / 729.0;
    private const double a61 = 9017.0 / 3168.0, a62 = -355.0 / 33.0, a63 = 46732.0 / 5247.0, a64 = 49.0 / 176.0, a65 = -5103.0 / 18656.0;
    private const double a71 = 35.0 / 384.0, a73 = 500.0 / 1113.0, a74 = 125.0 / 192.0, a75 = -2187.0 / 6784.0, a76 = 11.0 / 84.0;
    private const double b1 = 35.0 / 384.0, b3 = 500.0 / 1113.0, b4 = 125.0 / 192.0, b5 = -2187.0 / 6784.0, b6 = 11.0 / 84.0;
    private const double e1 = 71.0 / 57600.0, e3 = -71.0 / 16695.0, e4 = 71.0 / 1920.0, e5 = -17253.0 / 339200.0, e6 = 22.0 / 525.0, e7 = -1.0 / 40.0;
    #endregion

    private readonly BodyState[] k1, k2, k3, k4, k5, k6, k7, tempState, errorState;

    public Ode45(Body[] stars, double initialDt, double tolerance = 1e-6, double minDt = 1e-9, double maxDt = 0.1)
        : base(stars, initialDt)
    {
        _tolerance = tolerance;
        _minDt = minDt;
        _maxDt = maxDt;
        int starCount = stars.Length;
        k1 = new BodyState[starCount]; k2 = new BodyState[starCount]; k3 = new BodyState[starCount]; k4 = new BodyState[starCount];
        k5 = new BodyState[starCount]; k6 = new BodyState[starCount]; k7 = new BodyState[starCount];
        tempState = new BodyState[starCount]; errorState = new BodyState[starCount];
    }

    public override double Step(ReadOnlySpan<BodyState> current)
    {
        while (true)
        {
            #region Stage Calculations
            NewtonsLaw(current, k1);
            for (int i = 0; i < _stars.Length; i++) tempState[i] = current[i] + dt * a21 * k1[i];
            NewtonsLaw(tempState, k2);
            for (int i = 0; i < _stars.Length; i++) tempState[i] = current[i] + dt * (a31 * k1[i] + a32 * k2[i]);
            NewtonsLaw(tempState, k3);
            for (int i = 0; i < _stars.Length; i++) tempState[i] = current[i] + dt * (a41 * k1[i] + a42 * k2[i] + a43 * k3[i]);
            NewtonsLaw(tempState, k4);
            for (int i = 0; i < _stars.Length; i++) tempState[i] = current[i] + dt * (a51 * k1[i] + a52 * k2[i] + a53 * k3[i] + a54 * k4[i]);
            NewtonsLaw(tempState, k5);
            for (int i = 0; i < _stars.Length; i++) tempState[i] = current[i] + dt * (a61 * k1[i] + a62 * k2[i] + a63 * k3[i] + a64 * k4[i] + a65 * k5[i]);
            NewtonsLaw(tempState, k6);
            for (int i = 0; i < _stars.Length; i++)
            {
                tempState[i] = current[i] + dt * (a71 * k1[i] + a73 * k3[i] + a74 * k4[i] + a75 * k5[i] + a76 * k6[i]);
                NewtonsLaw(tempState, k7);
                errorState[i] = k1[i] * e1 + k3[i] * e3 + k4[i] * e4 + k5[i] * e5 + k6[i] * e6 + k7[i] * e7;
            }
            double errorNorm = 0;
            for (int i = 0; i < _stars.Length; i++)
            {
                errorNorm += Math.Pow(errorState[i].Px / _tolerance, 2) + Math.Pow(errorState[i].Py / _tolerance, 2) +
                             Math.Pow(errorState[i].Vx / _tolerance, 2) + Math.Pow(errorState[i].Vy / _tolerance, 2);
            }
            errorNorm = dt * Math.Sqrt(errorNorm / (4.0 * _stars.Length));
            #endregion

            if (errorNorm <= 1.0) // 步长可接受
            {
                for (int i = 0; i < _stars.Length; i++)
                {
                    _stars[i].State += dt * (b1 * k1[i] + b3 * k3[i] + b4 * k4[i] + b5 * k5[i] + b6 * k6[i]);
                }
                double newDt = errorNorm == 0.0 ? dt * MaxGrowFactor : dt * SafetyFactor * Math.Pow(1.0 / errorNorm, 0.2);
                dt = Math.Clamp(Math.Min(newDt, dt * MaxGrowFactor), _minDt, _maxDt);
                return dt;
            }
            else // 步长过大，拒绝本次计算
            {
                double shrinkPower = 0.25;
                double newDt = dt * SafetyFactor * Math.Pow(1.0 / errorNorm, shrinkPower);
                newDt = Math.Max(newDt, dt * MinShrinkFactor);

                // 检查计算出的新步长是否低于设定的绝对最小值
                if (newDt < _minDt)
                {
                    throw new InvalidOperationException(
                        $"Ode45: Adaptive step size ({newDt:E2}) has fallen below the minimum allowed value ({_minDt:E2}). " +
                        $"The solver cannot meet the required error tolerance ({_tolerance:E2}). " +
                        "Consider increasing tolerance, decreasing minDt, or checking for simulation instability (e.g., a near-collision)."
                    );
                }

                // 更新步长以供下一次重试
                dt = newDt;
            }
        }
    }
}
