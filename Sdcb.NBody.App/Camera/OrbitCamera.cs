using System.Numerics;

namespace Sdcb.NBody.App.Camera;

/// <summary>Spherical orbit camera with critically-damped smoothing toward goal values.</summary>
internal sealed class OrbitCamera
{
    public Vector3 Target, GoalTarget;
    public float Yaw, GoalYaw;
    public float Pitch, GoalPitch;
    public float Distance = 5, GoalDistance = 5;
    public float FovY = MathF.PI / 3.6f;
    public float Smoothing = 6f;   // higher = snappier

    public Vector3 Position { get; private set; }
    public Vector3 Forward { get; private set; }
    public Vector3 Right { get; private set; }
    public Vector3 Up { get; private set; }
    public Matrix4x4 View { get; private set; }
    public Matrix4x4 Proj { get; private set; }

    // Z is "up" in world space (ecliptic / galactic plane = XY).
    private static readonly Vector3 WorldUp = Vector3.UnitZ;

    public void SnapToGoal()
    {
        Target = GoalTarget; Yaw = GoalYaw; Pitch = GoalPitch; Distance = GoalDistance;
    }

    public void Orbit(float dYaw, float dPitch)
    {
        GoalYaw += dYaw;
        GoalPitch = Math.Clamp(GoalPitch + dPitch, -1.55f, 1.55f);
    }

    public void Zoom(float factor) => GoalDistance = Math.Clamp(GoalDistance * factor, 1e-3f, 1e5f);

    public void Pan(float dxPixels, float dyPixels, float viewportHeight)
    {
        float worldPerPixel = 2f * Distance * MathF.Tan(FovY / 2) / viewportHeight;
        GoalTarget += (-Right * dxPixels + Up * dyPixels) * worldPerPixel;
    }

    public void Update(float dt, float aspect)
    {
        float k = 1f - MathF.Exp(-Smoothing * dt);
        Target = Vector3.Lerp(Target, GoalTarget, k);
        Yaw += (GoalYaw - Yaw) * k;
        Pitch += (GoalPitch - Pitch) * k;
        Distance = MathF.Exp(MathF.Log(Distance) + (MathF.Log(GoalDistance) - MathF.Log(Distance)) * k);

        Vector3 offset = new(MathF.Cos(Pitch) * MathF.Cos(Yaw), MathF.Cos(Pitch) * MathF.Sin(Yaw), MathF.Sin(Pitch));
        Position = Target + offset * Distance;
        Forward = Vector3.Normalize(Target - Position);
        Right = Vector3.Normalize(Vector3.Cross(Forward, WorldUp));
        if (float.IsNaN(Right.X)) Right = Vector3.UnitX;
        Up = Vector3.Cross(Right, Forward);

        View = Matrix4x4.CreateLookAt(Position, Target, Up);
        float near = Math.Max(Distance * 2e-3f, 1e-6f);
        Proj = Matrix4x4.CreatePerspectiveFieldOfView(FovY, aspect, near, Distance * 500f);
    }

    /// <summary>Projects a world point to pixel coordinates. Returns false if behind the camera.</summary>
    public bool WorldToScreen(Vector3 p, float width, float height, out Vector2 screen)
    {
        Vector4 c = Vector4.Transform(new Vector4(p, 1), View * Proj);
        screen = default;
        if (c.W <= 1e-6f) return false;
        screen = new Vector2((c.X / c.W * 0.5f + 0.5f) * width, (0.5f - c.Y / c.W * 0.5f) * height);
        return true;
    }

    /// <summary>Ray through a pixel.</summary>
    public (Vector3 Origin, Vector3 Dir) ScreenRay(float x, float y, float width, float height)
    {
        float ndcX = x / width * 2 - 1, ndcY = 1 - y / height * 2;
        float t = MathF.Tan(FovY / 2);
        Vector3 dir = Vector3.Normalize(Forward + Right * ndcX * t * (width / height) + Up * ndcY * t);
        return (Position, dir);
    }
}
