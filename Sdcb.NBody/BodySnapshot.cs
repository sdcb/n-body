namespace Sdcb.NBody;

public record BodySnapshot(int Id, float Px, float Py, BodyType BodyType, float Mass)
{
    public float Size => BodyType switch
    {
        BodyType.BlackHole => MathF.Log(MathF.Log(Mass) + 1) + 1,
        _ => 0.05f
    };
}
