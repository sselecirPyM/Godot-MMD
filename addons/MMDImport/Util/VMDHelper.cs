namespace Mmd.addons.MMDImport.Util
{
    public static class VMDHelper
    {
        public static void Scale(this VMDFormat vmd, float scale)
        {
            foreach (var pair in vmd.BoneKeyFrameSet)
            {
                for (int i = 0; i < pair.Value.Count; i++)
                {
                    var v = pair.Value[i];
                    v.translation *= scale;
                    pair.Value[i] = v;
                }
            }
            for (int i = 0; i < vmd.CameraKeyFrames.Count; i++)
            {
                var frame = vmd.CameraKeyFrames[i];
                frame.position *= scale;
                frame.distance *= scale;

                vmd.CameraKeyFrames[i] = frame;
            }
        }
    }
}
