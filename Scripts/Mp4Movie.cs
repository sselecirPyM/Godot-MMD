using Godot;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;

namespace Mmd.Scripts
{
    public enum VideoEncoderOption
    {
        X264,
        H264_NVEnc,
        HEVC_NVEnc,
        QSV,
    }
    public partial class Mp4Movie : Node
    {
        [Export(PropertyHint.SaveFile)]
        public string movie;
        [Export(hintString: "编码器")]
        public VideoEncoderOption encoder;

        [Export(hintString: "录制时间")]
        public float endTime = 240;
        [Export(hintString: "录制时移除的节点")]
        public Node[] removeOnRecord;
        [Export(hintString: "不录制时移除的节点")]
        public Node[] removeOnPlay;
        [Export(hintString: "录制开始时间")]
        public float startTime = 0;

        string absolutePath;
        Stream pipe;
        Process ffmpegProcess;


        int frameCount = -1;
        float frameRate;
        bool recording;
        bool stop = false;

        RenderingDevice renderingDevice;


        public override void _Ready()
        {
            if (OS.HasFeature("movie"))
            {
                renderingDevice = RenderingServer.GetRenderingDevice();
                recording = true;
                absolutePath = ProjectSettings.GlobalizePath(movie);
                Directory.CreateDirectory(Path.GetDirectoryName(absolutePath));
                StartFFMpeg();
                if (pipe != null)
                {
                    RenderingServer.FramePostDraw += RenderingServer_FramePostDraw;
                }
                //pipe = System.IO.File.Create(absolutePath);
                //RenderingServer.FramePostDraw += RenderingServer_FramePostDraw;
                if (removeOnRecord != null)
                    foreach (var node in removeOnRecord)
                    {
                        node.QueueFree();
                    }

                //var t1 = GetViewport().GetTexture();
                //renderingDevice.TextureCreate(new RDTextureFormat()
                //{
                //    Width = (uint)t1.GetWidth(),
                //    Height = (uint)t1.GetHeight(),
                //    Depth = 1,
                //    Mipmaps = 0,
                //    Format = RenderingDevice.DataFormat.R8G8B8Unorm,
                //    Samples = RenderingDevice.TextureSamples.Samples1,
                //    ArrayLayers = 0,
                //    UsageBits = RenderingDevice.TextureUsageBits.CanCopyToBit,
                //    TextureType = RenderingDevice.TextureType.Type2D,
                //}, new RDTextureView() { });
            }
            else
            {
                if (removeOnPlay != null)
                    foreach (var node in removeOnPlay)
                    {
                        node.QueueFree();
                    }
            }
        }

        void StartFFMpeg()
        {
            var fps = ProjectSettings.GetSetting("editor/movie_writer/fps");
            var texture = GetViewport().GetTexture();
            frameRate = (float)fps;
            List<string> args = new List<string>()
            {
                "-y",
                "-f","rawvideo",
                "-pixel_format","rgb24",
                "-r", fps.ToString(),
                "-colorspace","bt709",
                "-color_trc","iec61966-2-1",
                "-s", texture.GetWidth() + "X" + texture.GetHeight(),
                "-i", @"pipe:0",
                "-color_primaries","bt709",
                "-color_trc"," bt709",
                "-colorspace","bt709",
                "-color_range", "tv"
            };

            switch (encoder)
            {
                case VideoEncoderOption.H264_NVEnc:
                    args.AddRange(new string[]
                    {
                        "-vf", "format=yuv420p",
                        "-c:v", "h264_nvenc",
                        "-cq","26",
                        absolutePath,
                    });
                    break;
                case VideoEncoderOption.HEVC_NVEnc:
                    args.AddRange(new string[]
                    {
                        "-vf", "format=yuv420p",
                        "-c:v", "hevc_nvenc",
                        "-cq","26",
                        absolutePath,
                    });
                    break;
                case VideoEncoderOption.QSV:
                    args.AddRange(new string[]
                    {
                        "-vf", "format=yuv420p",
                        "-c:v", "h264_qsv",
                        "-global_quality","17",
                        absolutePath,
                    });
                    break;
                default:
                    args.AddRange(new string[]
                    {
                        "-vf", "format=yuv420p",
                        "-c:v", "libx264",
                        //"-preset", "medium",
                        "-crf", "17",
                        absolutePath,
                    });
                    break;
            }
            try
            {
                var processStartInfo = new ProcessStartInfo();
                processStartInfo.FileName = "ffmpeg";
                processStartInfo.RedirectStandardInput = true;
                processStartInfo.RedirectStandardOutput = true;
                foreach (var arg in args)
                    processStartInfo.ArgumentList.Add(arg);
                ffmpegProcess = Process.Start(processStartInfo);
                this.pipe = ffmpegProcess.StandardInput.BaseStream;

            }
            catch
            {
                GD.Print("无法录制，启动FFMpeg失败");
                QuitGame();
            }
        }

        private void RenderingServer_FramePostDraw()
        {
            if (frameCount < startTime * frameRate)
            {
                frameCount++;
                return;
            }
            var texture = GetViewport().GetTexture();
            using var image = texture.GetImage();
            var data = image.GetData();
            pipe.Write(data);
            pipe.Flush();
            frameCount++;
            if (frameCount >= endTime * frameRate)
            {
                QuitGame();
            }
        }

        public override void _Notification(int what)
        {
            if (what == NotificationWMCloseRequest)
            {
                GD.Print("mp4movie quit");
                pipe?.Close();
                pipe = null;
            }
        }

        void QuitGame()
        {
            GetTree().Root.PropagateNotification((int)NotificationWMCloseRequest);
            GetTree().Quit();
        }
    }
}
