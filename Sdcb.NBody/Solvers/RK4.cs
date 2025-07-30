namespace Sdcb.NBody.Solvers;

public class RK4(Body[] stars, double initialDt) : Solver(stars, initialDt)
{
    BodyState[] k1 = new BodyState[stars.Length];
    BodyState[] k2 = new BodyState[stars.Length];
    BodyState[] k3 = new BodyState[stars.Length];
    BodyState[] k4 = new BodyState[stars.Length];
    BodyState[] w = new BodyState[stars.Length];

    public override double Step(ReadOnlySpan<BodyState> current)
    {
        NewtonsLaw(current, k1);

        for (int i = 0; i < _stars.Length; ++i)
        {
            w[i].Px = current[i].Px + k1[i].Px * dt / 2;
            w[i].Py = current[i].Py + k1[i].Py * dt / 2;
            w[i].Vx = current[i].Vx + k1[i].Vx * dt / 2;
            w[i].Vy = current[i].Vy + k1[i].Vy * dt / 2;
        }
        NewtonsLaw(w, k2);

        for (int i = 0; i < _stars.Length; ++i)
        {
            w[i].Px = current[i].Px + k2[i].Px * dt / 2;
            w[i].Py = current[i].Py + k2[i].Py * dt / 2;
            w[i].Vx = current[i].Vx + k2[i].Vx * dt / 2;
            w[i].Vy = current[i].Vy + k2[i].Vy * dt / 2;
        }
        NewtonsLaw(w, k3);

        for (int i = 0; i < _stars.Length; ++i)
        {
            w[i].Px = current[i].Px + k3[i].Px * dt;
            w[i].Py = current[i].Py + k3[i].Py * dt;
            w[i].Vx = current[i].Vx + k3[i].Vx * dt;
            w[i].Vy = current[i].Vy + k3[i].Vy * dt;
        }
        NewtonsLaw(w, k4);

        for (int i = 0; i < _stars.Length; ++i)
        {
            _stars[i].State.Px += (k1[i].Px + 2 * k2[i].Px + 2 * k3[i].Px + k4[i].Px) / 6 * dt;
            _stars[i].State.Py += (k1[i].Py + 2 * k2[i].Py + 2 * k3[i].Py + k4[i].Py) / 6 * dt;
            _stars[i].State.Vx += (k1[i].Vx + 2 * k2[i].Vx + 2 * k3[i].Vx + k4[i].Vx) / 6 * dt;
            _stars[i].State.Vy += (k1[i].Vy + 2 * k2[i].Vy + 2 * k3[i].Vy + k4[i].Vy) / 6 * dt;
        }
        return dt;
    }
}
