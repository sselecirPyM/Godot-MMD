#if TOOLS
using Godot;
using Mmd.addons.MMDImport.Inspectors;

namespace Mmd.addons.MMDImport
{
    [Tool]
    public partial class MMDImport : EditorPlugin
    {
        PMXImporter pmxImporter;
        VMDImporter vmdImporter;

        MMDInspectorPlugin mmdInspectorPlugin;
        Mesh3DInspectorPlugin mesh3DInspectorPlugin;

        public override void _EnterTree()
        {
            pmxImporter = new PMXImporter();
            AddSceneFormatImporterPlugin(pmxImporter);
            vmdImporter = new VMDImporter();
            AddImportPlugin(vmdImporter);

            mmdInspectorPlugin = new MMDInspectorPlugin
            {
                currentPlugin = this
            };
            AddInspectorPlugin(mmdInspectorPlugin);

            mesh3DInspectorPlugin = new Mesh3DInspectorPlugin()
            {
                currentPlugin = this
            };
            AddInspectorPlugin(mesh3DInspectorPlugin);
        }

        public override void _ExitTree()
        {
            RemoveImportPlugin(vmdImporter);
            RemoveSceneFormatImporterPlugin(pmxImporter);

            RemoveInspectorPlugin(mesh3DInspectorPlugin);
            RemoveInspectorPlugin(mmdInspectorPlugin);
        }
    }
}
#endif
