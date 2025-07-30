namespace Sdcb.NBody.Solvers;

/// <summary>
/// 使用 Cash-Karp 4(5) 方法的自适应步长求解器。
/// 当所需步长小于设定的最小值时，将抛出异常。
/// </summary>
public class CashKarp : Solver
{
    // 自适应步长控制参数 (与Ode45相同)
    private readonly double _tolerance;
    private readonly double _minDt;
    private readonly double _maxDt;

    private const double SafetyFactor = 0.9;
    private const double MinShrinkFactor = 0.2;
    private const double MaxGrowFactor = 5.0;

    // Cash-Karp RK45 Butcher Tableau 系数 (与之前相同)
    #region Coefficients
    private const double c2 = 1.0 / 5.0, c3 = 3.0 / 10.0, c4 = 3.0 / 5.0, c5 = 1.0, c6 = 7.0 / 8.0;
    private const double a21 = 1.0 / 5.0;
    private const double a31 = 3.0 / 40.0, a32 = 9.0 / 40.0;
    private const double a41 = 3.0 / 10.0, a42 = -9.0 / 10.0, a43 = 6.0 / 5.0;
    private const double a51 = -11.0 / 54.0, a52 = 5.0 / 2.0, a53 = -70.0 / 27.0, a54 = 35.0 / 27.0;
    private const double a61 = 1631.0 / 55296.0, a62 = 175.0 / 512.0, a63 = 575.0 / 13824.0, a64 = 44275.0 / 110592.0, a65 = 253.0 / 4096.0;
    private const double b1 = 37.0 / 378.0, b3 = 250.0 / 621.0, b4 = 125.0 / 594.0, b6 = 512.0 / 1771.0;
    private const double e1 = 2825.0 / 27648.0 - 37.0 / 378.0, e3 = 18575.0 / 48384.0 - 250.0 / 621.0, e4 = 13525.0 / 55296.0 - 125.0 / 594.0, e5 = 277.0 / 14336.0, e6 = 1.0 / 4.0 - 512.0 / 1771.0;
    #endregion

    private readonly BodyState[] k1, k2, k3, k4, k5, k6, tempState, errorState;

    public CashKarp(Body[] stars, double initialDt, double tolerance = 1e-6, double minDt = 1e-9, double maxDt = 0.1)
        : base(stars, initialDt)
    {
        _tolerance = tolerance;
        _minDt = minDt;
        _maxDt = maxDt;
        int starCount = stars.Length;
        k1 = new BodyState[starCount]; k2 = new BodyState[starCount]; k3 = new BodyState[starCount];
        k4 = new BodyState[starCount]; k5 = new BodyState[starCount]; k6 = new BodyState[starCount];
        tempState = new BodyState[starCount]; errorState = new BodyState[starCount];
    }

    public override double Step(ReadOnlySpan<BodyState> current)
    {
        while (true)
        {
            // ... [阶段计算部分与之前完全相同] ...
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
                errorState[i] = e1 * k1[i] + e3 * k3[i] + e4 * k4[i] + e5 * k5[i] + e6 * k6[i];
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
                    _stars[i].State += dt * (b1 * k1[i] + b3 * k3[i] + b4 * k4[i] + b6 * k6[i]);
                }
                double newDt = errorNorm < 1e-10 ? dt * MaxGrowFactor : dt * SafetyFactor * Math.Pow(1.0 / errorNorm, 0.2);
                dt = Math.Clamp(Math.Min(newDt, dt * MaxGrowFactor), _minDt, _maxDt);
                return dt;
            }
            else // 步长过大，拒绝本次计算
            {
                double shrinkPower = 0.25;
                double newDt = dt * SafetyFactor * Math.Pow(1.0 / errorNorm, shrinkPower);
                newDt = Math.Max(newDt, dt * MinShrinkFactor);

                // --- 核心修改 ---
                if (newDt < _minDt)
                {
                    throw new InvalidOperationException(
                        $"CashKarpSolver: Adaptive step size ({newDt:E2}) has fallen below the minimum allowed value ({_minDt:E2}). " +
                        $"The solver cannot meet the required error tolerance ({_tolerance:E2}). " +
                        "Consider increasing tolerance, decreasing minDt, or checking for simulation instability (e.g., a near-collision)."
                    );
                }

                dt = newDt;
            }
        }
    }
}
