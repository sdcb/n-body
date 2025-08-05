namespace Sdcb.NBody;

public record Body(int Id, BodyType BodyType, double Mass) : IDisposable
{
    public BodyState State;
    public double Size => BodyType switch
    {
        BodyType.BlackHole => Math.Log(Math.Log(Mass)),
        _ => Math.Log(Mass)
    };

    public void Dispose()
    {
    }

    public BodySnapshot GetSnapshot() => new(Id, (float)State.Px, (float)State.Py, BodyType, (float)Mass);
}
