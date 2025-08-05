using Sdcb.NBody.Common;
using Vortice.Mathematics;

namespace Sdcb.NBody.Direct2D;

record BodyUIProps
{
    public CircularBuffer<TimedVector2> TrackingHistory { get; } = new(capacity: 100000);
    public Color4 Color { get; set; }
}
