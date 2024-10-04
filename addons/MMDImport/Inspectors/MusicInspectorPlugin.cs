using Godot;
using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace Mmd.addons.MMDImport.Inspectors
{
    public partial class MusicInspectorPlugin : EditorInspectorPlugin
    {
        AudioStream stream;

        public override bool _CanHandle(GodotObject @object)
        {
            if (@object == null)
            {
                return false;
            }
            if (@object is AudioStreamMP3 || @object is AudioStreamWav || @object is AudioStreamOggVorbis)
            {
                stream = (AudioStream)@object;
                return true;
            }
            return false;
        }

        public override void _ParseCategory(GodotObject @object, string category)
        {
            if (category == "AudioStreamWAV" || category == "AudioStreamMP3")
            {
                var b1 = AddButton("生成频谱图");
                b1.Pressed += () =>
                {
                    try
                    {
                        var fileName = ProjectSettings.GlobalizePath(stream.ResourcePath);
                        double length = stream.GetLength();
                        List<string> args = new List<string>()
                        {
                            "-y",
                            "-i", fileName,
                            "-lavfi", $"showspectrumpic=legend=0:stop=16k:scale=lin:limit=0:drange=100:color=channel:win_func=hanning:s={(int)(length*30)}x128",
                            $"{fileName.Substring(0, fileName.LastIndexOf('.'))}.exr"
                        };
                        ProcessStartInfo info = new ProcessStartInfo();
                        info.FileName = "ffmpeg";
                        info.UseShellExecute = true;
                        foreach (var a in args)
                        {
                            info.ArgumentList.Add(a);
                        }

                        Process.Start(info);
                    }
                    catch (Exception ex)
                    {
                        GD.Print("需要FFmpeg");
                    }
                };
            }
        }

        Button AddButton(string text, string tooltip = "")
        {
            var button = new Button();
            button.Text = text;
            button.TooltipText = tooltip;
            AddCustomControl(button);
            return button;
        }
    }
}
