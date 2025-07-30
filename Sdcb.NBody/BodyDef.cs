namespace Sdcb.NBody;

public record BodyDef(BodyState BodyState, BodyType BodyType = BodyType.Solar, double Mass = 1)
{
    public BodyDef(double px, double py, double vx, double vy, BodyType starType = BodyType.Solar, double mass = 1)
        : this(new BodyState(px, py, vx, vy), starType, mass)
    {
    }

    public Body Create(int id) => new(id, BodyType, Mass) { State = BodyState };
}
