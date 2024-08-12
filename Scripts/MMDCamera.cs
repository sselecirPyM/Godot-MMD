using Godot;
using Mmd.addons.MMDImport;
using Mmd.addons.MMDImport.Util;
using System.Collections.Generic;
using System;

namespace Mmd.Scripts
{
    [Tool]
    public partial class MMDCamera : Camera3D
    {
        [Export]
        public VMDCameraResource vmdResource
        {
            get => _vmdResource;
            set
            {
                _vmdResource = value;
                vmd = null;
            }
        }

        [Export]
        public double currentTime;

        [Export]
        public float motionScale = 0.1f;

        [Export]
        public bool AutoPlay = true;
        [Export]
        public bool Playing;

        VMDCameraResource _vmdResource;

        VMDFormat vmd;

        public override void _Process(double delta)
        {
            if (vmd == null && _vmdResource != null)
            {
                vmd = VMDFormat.Load(_vmdResource.Data);
                vmd.Scale(motionScale);
            }
            if (AutoPlay && !Engine.IsEditorHint())
            {
                Playing = true;
            }
            if (Playing)
            {
                currentTime += delta;
            }

            if (vmd != null)
            {
                var cam = GetCameraKeyFrame(vmd.CameraKeyFrames);

                Rotation = GetVector3Rotation(cam.rotation);
                Position = GetVector3(cam.position) + Quaternion * new Vector3(0, 0, -cam.distance);
                Fov = cam.FOV;
            }

        }


        static Vector3 GetVector3Rotation(System.Numerics.Vector3 vec3)
        {
            return new Vector3(vec3.X, vec3.Y, vec3.Z);
        }

        static Vector3 GetVector3(System.Numerics.Vector3 vec3)
        {
            return new Vector3(vec3.X, vec3.Y, vec3.Z);
        }

        CameraKeyFrame GetCameraKeyFrame(List<CameraKeyFrame> list)
        {
            int l = 0;
            int r = list.Count - 1;
            CameraKeyFrame lf = list[l];
            CameraKeyFrame rf = list[r];

            float f1 = (float)currentTime * 30;
            while (l + 1 < r)
            {
                var mid = (l + r) / 2;
                var mf = list[mid];
                if (mf.Frame > f1)
                {
                    rf = mf;
                    r = mid;
                }
                else
                {
                    lf = mf;
                    l = mid;
                }
            }
            if (lf.Frame == rf.Frame)
            {
                return rf;
            }
            float factor = ((float)currentTime * 30 - lf.Frame) / (rf.Frame - lf.Frame);
            factor = Math.Clamp(factor, 0, 1);

            float fx = CubicBezierCurve.Get(GetA(rf.mxInterpolator), GetB(rf.mxInterpolator)).Sample(factor);
            float fy = CubicBezierCurve.Get(GetA(rf.myInterpolator), GetB(rf.myInterpolator)).Sample(factor);
            float fz = CubicBezierCurve.Get(GetA(rf.mzInterpolator), GetB(rf.mzInterpolator)).Sample(factor);
            float fr = CubicBezierCurve.Get(GetA(rf.rInterpolator), GetB(rf.rInterpolator)).Sample(factor);
            float fd = CubicBezierCurve.Get(GetA(rf.dInterpolator), GetB(rf.dInterpolator)).Sample(factor);
            float ff = CubicBezierCurve.Get(GetA(rf.fInterpolator), GetB(rf.fInterpolator)).Sample(factor);

            var position = new System.Numerics.Vector3(lf.position.X * (1 - fx) + rf.position.X * fx,
                lf.position.Y * (1 - fy) + rf.position.Y * fy,
                lf.position.Z * (1 - fz) + rf.position.Z * fz);
            return new CameraKeyFrame()
            {
                position = position,
                rotation = System.Numerics.Vector3.Lerp(lf.rotation, rf.rotation, fr),
                FOV = Mathf.Lerp(lf.FOV, rf.FOV, ff),
                distance = Mathf.Lerp(lf.distance, rf.distance, fd),
            };
        }

        static Vector2 GetA(Interpolator interpolator)
        {
            return new Vector2(interpolator.ax, interpolator.ay);
        }

        static Vector2 GetB(Interpolator interpolator)
        {
            return new Vector2(interpolator.bx, interpolator.by);
        }
    }
}
