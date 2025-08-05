using FlysEngine.Desktop;
using System.Windows.Forms;
using Vortice.DXGI;

namespace Sdcb.NBody.Direct2D;

internal class Program
{
    static void Main(string[] args)
    {
        NBodySystem _sys = new(SystemDef.CreateStableRing(3, scale: 0.9));
        using NBodyWindow sw = new(_sys.AutoStep(boundedCapacity: 512))
        {
            StartPosition = FormStartPosition.CenterScreen,
            Size = new System.Drawing.Size(1024, 768),
            Text = "星体运动模拟"
        };
        RenderLoop.Run(sw, () =>
        {
            sw.Render(1, PresentFlags.None);
        });
    }
}
