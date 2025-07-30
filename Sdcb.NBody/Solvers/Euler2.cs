namespace Sdcb.NBody.Solvers;

public class Euler2(Body[] stars, double initialDt) : Solver(stars, initialDt)
{
    public override double Step(ReadOnlySpan<BodyState> current)
    {
        BodyState[] k1 = new BodyState[_stars.Length];
        BodyState[] k2 = new BodyState[_stars.Length];
        BodyState[] w = new BodyState[_stars.Length];

        NewtonsLaw(current, k1);

        for (int i = 0; i < _stars.Length; ++i)
        {
            w[i].Px = current[i].Px + k1[i].Px * dt;
            w[i].Py = current[i].Py + k1[i].Py * dt;
            w[i].Vx = current[i].Vx + k1[i].Vx * dt;
            w[i].Vy = current[i].Vy + k1[i].Vy * dt;
        }

        NewtonsLaw(w, k2);

        for (int i = 0; i < _stars.Length; ++i)
        {
            _stars[i].State.Px += (k1[i].Px + k2[i].Px) / 2 * dt;
            _stars[i].State.Py += (k1[i].Py + k2[i].Py) / 2 * dt;
            _stars[i].State.Vx += (k1[i].Vx + k2[i].Vx) / 2 * dt;
            _stars[i].State.Vy += (k1[i].Vy + k2[i].Vy) / 2 * dt;
        }

        return dt;
    }
}
