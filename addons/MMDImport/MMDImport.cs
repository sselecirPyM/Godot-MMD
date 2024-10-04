#if TOOLS
using Godot;
using Mmd.addons.MMDImport.Inspectors;
using System.Collections.Generic;

namespace Mmd.addons.MMDImport
{
    [Tool]
    public partial class MMDImport : EditorPlugin
    {
        PMXImporter pmxImporter;
        VMDImporter vmdImporter;

        public static MMDImport currentPlugin;

        public List<EditorInspectorPlugin> editorInspectorPlugins = new List<EditorInspectorPlugin>();

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

            editorInspectorPlugins = new List<EditorInspectorPlugin>()
            {
                 new MMDInspectorPlugin(),
                 new Mesh3DInspectorPlugin(),
                 new MusicInspectorPlugin(),
            };
            foreach (var a in editorInspectorPlugins)
            {
                AddInspectorPlugin(a);
            }
        }

        public override void _ExitTree()
        {
            RemoveImportPlugin(vmdImporter);
            RemoveSceneFormatImporterPlugin(pmxImporter);

            foreach (var a in editorInspectorPlugins)
            {
                RemoveInspectorPlugin(a);
            }
        }
    }
}
#endif
