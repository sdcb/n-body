namespace Sdcb.NBody.Solvers;

/// <summary>
/// 使用蛙跳法（Leapfrog）的辛积分器。
/// 这种方法非常适合于需要长期保持能量守恒的N体模拟。
/// 它采用 "Kick-Drift-Kick" (KDK) 结构实现。
/// </summary>
public class Leapfrog : Solver
{
    // 用于存储计算出的导数（加速度）
    private readonly BodyState[] derivatives;

    // 用于在第二次Kick时传递更新后的状态
    private readonly BodyState[] nextState;

    public Leapfrog(Body[] stars, double initialDt) : base(stars, initialDt)
    {
        derivatives = new BodyState[stars.Length];
        nextState = new BodyState[stars.Length];
    }

    public override double Step(ReadOnlySpan<BodyState> current)
    {
        // --- 1. First Kick (速度推进 dt/2) ---
        // 使用当前位置计算初始加速度
        NewtonsLaw(current, derivatives);

        for (int i = 0; i < _stars.Length; i++)
        {
            // 注意：这里我们直接修改 _stars[i].State
            // v(t + dt/2) = v(t) + a(t) * dt/2
            _stars[i].State.Vx = current[i].Vx + derivatives[i].Vx * (dt / 2.0);
            _stars[i].State.Vy = current[i].Vy + derivatives[i].Vy * (dt / 2.0);
        }

        // --- 2. Drift (位置推进 dt) ---
        for (int i = 0; i < _stars.Length; i++)
        {
            // x(t + dt) = x(t) + v(t + dt/2) * dt
            // 使用刚刚更新的“半步速度”来更新位置
            _stars[i].State.Px = current[i].Px + _stars[i].State.Vx * dt;
            _stars[i].State.Py = current[i].Py + _stars[i].State.Vy * dt;
        }

        // --- 3. Second Kick (速度再推进 dt/2) ---
        // 为了计算新位置的加速度，我们需要从 _stars 数组中获取更新后的状态
        for (int i = 0; i < _stars.Length; i++)
        {
            nextState[i] = _stars[i].State;
        }

        // 使用新位置 x(t + dt) 计算新加速度 a(t + dt)
        NewtonsLaw(nextState, derivatives);

        for (int i = 0; i < _stars.Length; i++)
        {
            // v(t + dt) = v(t + dt/2) + a(t + dt) * dt/2
            // 在半步速度的基础上，完成剩余的半步更新
            _stars[i].State.Vx += derivatives[i].Vx * (dt / 2.0);
            _stars[i].State.Vy += derivatives[i].Vy * (dt / 2.0);
        }

        return dt;
    }
}
