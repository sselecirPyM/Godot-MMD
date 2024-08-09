using Godot;

namespace Mmd.addons.MMDImport
{
    [GlobalClass]
    public partial class VMDResource : Resource
    {
        [Export]
        public byte[] Data { get; set; }
    }
}
