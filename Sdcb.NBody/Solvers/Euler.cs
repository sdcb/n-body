namespace Sdcb.NBody.Solvers;

public class Euler(Body[] stars, double initialDt) : Solver(stars, initialDt)
{
    public override double Step(ReadOnlySpan<BodyState> current)
    {
        BodyState[] delta = new BodyState[_stars.Length];

        NewtonsLaw(current, delta);

        for (int i = 0; i < _stars.Length; ++i)
        {
            _stars[i].State.Px += delta[i].Px * dt;
            _stars[i].State.Py += delta[i].Py * dt;
            _stars[i].State.Vx += delta[i].Vx * dt;
            _stars[i].State.Vy += delta[i].Vy * dt;
        }
        return dt;
    }
}
