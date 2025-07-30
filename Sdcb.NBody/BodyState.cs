namespace Sdcb.NBody;

public record struct BodyState(double Px, double Py, double Vx, double Vy)
{
    public readonly bool Crashed => Px > 50 || Px < -50 || Py > 50 || Py < -50;

    // 矢量加法
    public static BodyState operator +(BodyState a, BodyState b)
        => new(a.Px + b.Px, a.Py + b.Py, a.Vx + b.Vx, a.Vy + b.Vy);

    public static BodyState operator -(BodyState a, BodyState b)
        => new(a.Px - b.Px, a.Py - b.Py, a.Vx - b.Vx, a.Vy - b.Vy);

    // 标量乘法
    public static BodyState operator *(BodyState s, double scalar)
        => new(s.Px * scalar, s.Py * scalar, s.Vx * scalar, s.Vy * scalar);

    public static BodyState operator *(double scalar, BodyState s)
        => s * scalar;
}
