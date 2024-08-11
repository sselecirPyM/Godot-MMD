using Godot;

namespace Mmd.addons.MMDImport
{
    [GlobalClass]
    [Tool]
    public partial class VMDCameraResource:Resource
    {
        [Export]
        public byte[] Data { get; set; }
        [Export]
        public double Duration { get; set; }
    }
}
