#if TOOLS
using Godot;

namespace Mmd.addons.MMDImport
{
    [Tool]
    public partial class MMDImport : EditorPlugin
    {
        PMXImporter pmxImporter;
        VMDImporter vmdImporter;

        public override void _EnterTree()
        {
            pmxImporter = new PMXImporter();
            AddSceneFormatImporterPlugin(pmxImporter);
            vmdImporter = new VMDImporter();
            AddImportPlugin(vmdImporter);
        }

        public override void _ExitTree()
        {
            RemoveImportPlugin(vmdImporter);
            RemoveSceneFormatImporterPlugin(pmxImporter);
        }
    }
}
#endif
