namespace Sdcb.NBody.Solvers;

public abstract class Solver
{
    protected Body[] _stars;
    public double dt;
    public Solver(Body[] stars, double initialDt) { _stars = stars; dt = initialDt; }

    public abstract double Step(ReadOnlySpan<BodyState> current); // 可以返回自适应dt

    protected void NewtonsLaw(ReadOnlySpan<BodyState> current, Span<BodyState> result)
    {
        const double G = 1.0;
        // 先初始化 result 数组
        for (int i = 0; i < _stars.Length; ++i)
        {
            result[i] = new BodyState { Px = current[i].Vx, Py = current[i].Vy }; // Vx, Vy 默认为 0
        }

        // 只计算上三角矩阵，即 j > i
        for (int i = 0; i < _stars.Length; ++i)
        {
            for (int j = i + 1; j < _stars.Length; ++j)
            {
                double rx = current[j].Px - current[i].Px;
                double ry = current[j].Py - current[i].Py;
                double r2 = rx * rx + ry * ry;
                if (r2 < 1e-12) continue; // 使用一个小的阈值避免奇异点
                double r3 = r2 * Math.Sqrt(r2);

                double forceX = G * rx / r3;
                double forceY = G * ry / r3;

                // 对 i 施加力 (乘以 j 的质量)
                result[i].Vx += _stars[j].Mass * forceX;
                result[i].Vy += _stars[j].Mass * forceY;

                // 对 j 施加大小相等、方向相反的力 (乘以 i 的质量)
                result[j].Vx -= _stars[i].Mass * forceX;
                result[j].Vy -= _stars[i].Mass * forceY;
            }
        }
    }
}
