using Godot;

namespace Mmd.addons.MMDImport
{
    [GlobalClass]
    public partial class VMDCameraResource:Resource
    {
        [Export]
        public byte[] Data { get; set; }
    }
}
