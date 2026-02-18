#if TOOLS
using Godot;
using System.Collections.Generic;

namespace Mmd.addons.MMDImport
{
    [Tool]
    public partial class MMDImport : EditorPlugin
    {
        PMXImporter pmxImporter;
        VMDImporter vmdImporter;

        public static MMDImport currentPlugin;

        public MMDImport()
        {
            currentPlugin ??= this;
        }

        public override void _EnterTree()
        {
            MMDImport.currentPlugin = this;
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
