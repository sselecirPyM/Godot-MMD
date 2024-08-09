using Godot;
using Mmd.addons.MMDImport.Util;
using System;
using System.Collections.Generic;
using Matrix = System.Numerics.Matrix4x4;

namespace Mmd.addons.MMDImport
{
    public partial class MMDModel : Node3D
    {
        public enum IKTransformOrder
        {
            Yzx = 0,
            Zxy = 1,
            Xyz = 2,
        }

        public enum AxisFixType
        {
            FixNone,
            FixX,
            FixY,
            FixZ,
            FixAll
        }
        class AppendBone
        {
            public int bone;
            public bool rotate;
            public bool transition;
            public float ratio;
            public int index;
        }

        class IKLink
        {
            public int index;
            public Vector3 min;
            public Vector3 max;
            public bool limited;


            public IKTransformOrder TransformOrder;
            public AxisFixType FixTypes;
        }
        class IKBone
        {
            public string name;
            public bool enabled;
            public int bone;
            public int target;
            public int iterateLimit;
            public float angleLimit;
            public List<IKLink> links;
        }
        class MMDBone
        {
            public Vector3 restPosition;
            public Quaternion restRotation;
            public Transform3D restTransform;
            public Transform3D globalRestTransform;

            public Transform3D invertRestTransform;

            public Vector3 position;
            public Quaternion rotation;

            public Quaternion ikRotation;

            public Vector3 appendPosition;
            public Quaternion appendRotation;
            public int index;
            public MMDBone parent;
            public string name;

            public Transform3D transform3D;

            public bool isPhysicsBone;

            public Vector3 GetPosition()
            {
                return transform3D * new Vector3(0, 0, 0);
            }

            public void ComputeTransform()
            {
                if (parent != null)
                {
                    transform3D = parent.transform3D * restTransform * new Transform3D(new Basis(rotation * appendRotation), position + appendPosition);
                }
                else
                {
                    transform3D = restTransform * new Transform3D(new Basis(rotation * appendRotation), position + appendPosition);
                }
            }

            public void ComputeBaseTransform()
            {
                if (parent != null)
                {
                    transform3D = parent.transform3D * restTransform * new Transform3D(new Basis(rotation), position);
                }
                else
                {
                    transform3D = restTransform * new Transform3D(new Basis(rotation), position);
                }
            }
        }
        class PhysicsBone
        {
            public int index;
            public int type;
            public string name;
            public BulletSharp.RigidBody rigidBody;
            public Matrix offset;
            public Matrix invertOffset;

            public Node3D visuallizer;

            public void SetTransform(Transform3D transform)
            {
                rigidBody.MotionState.WorldTransform = offset * MMDPhysicsHelper.GetMatrix(transform);
            }

            public void SetTransform2(Transform3D transform)
            {
                rigidBody.WorldTransform = offset * MMDPhysicsHelper.GetMatrix(transform);
                rigidBody.MotionState.WorldTransform = rigidBody.WorldTransform;
            }

            public Transform3D GetTransform()
            {
                return MMDPhysicsHelper.GetTransform3D(invertOffset * rigidBody.MotionState.WorldTransform);
            }
        }
        class MMDJoint
        {
            public BulletSharp.TypedConstraint joint;
            public string name;
        }

        [Export]
        public double currentTime;
        [Export]
        public VMDResource vmdResource
        {
            get => _vmdResource;
            set
            {
                _vmdResource = value;
                vmd = null;
            }
        }
        VMDFormat vmd;
        [Export]
        public bool UseBulletPhysics = true;
        [Export]
        public bool PhysicsDebug;

        VMDResource _vmdResource;
        public Skeleton3D skeleton;
        public MeshInstance3D morphMesh;
        public float modelScale;

        List<AppendBone> appendBones = new List<AppendBone>();
        List<IKBone> ikBones = new List<IKBone>();
        List<PhysicsBone> physicsBones = new List<PhysicsBone>();

        List<MMDBone> mmdBones = new List<MMDBone>();
        List<MMDJoint> mmdJoints = new List<MMDJoint>();

        MMDBulletWorld bulletWorld = null;
        void CreatePhysics()
        {
            bulletWorld = new MMDBulletWorld();

            bulletWorld.Initialize();

            var physicsBoneMeta = (Godot.Collections.Array<Godot.Collections.Dictionary>)GetMeta("physics_bone");
            var jointMeta = (Godot.Collections.Array<Godot.Collections.Dictionary>)GetMeta("joint");
            foreach (var meta in physicsBoneMeta)
            {
                var rigidBody = bulletWorld.AddRigidBody(meta);
                int type = (int)meta["type"];
                int index = (int)meta["bone_index"];
                string name = (string)meta["name"];

                var mmdBone = mmdBones[index];
                mmdBone.isPhysicsBone = type != 0;
                var offset = (rigidBody.WorldTransform * MMDPhysicsHelper.GetMatrix(mmdBone.globalRestTransform).Invert());

                //var offset = Transform3D.Identity;
                var physicsBone = new PhysicsBone()
                {
                    name = name,
                    rigidBody = rigidBody,
                    index = index,
                    type = type,
                    offset = offset,
                    invertOffset = offset.Invert(),
                };

                physicsBones.Add(physicsBone);

                if (PhysicsDebug)
                {
                    int shape = (int)meta["shape"];
                    var dimensions = (Godot.Vector3)meta["dimensions"];
                    var position = (Godot.Vector3)meta["position"];
                    var rotation = (Godot.Vector3)meta["rotation"];
                    MeshInstance3D collisionVisualizer = new MeshInstance3D();
                    collisionVisualizer.Transform = MMDPhysicsHelper.GetTransform3D(rigidBody.MotionState.WorldTransform);

                    switch (shape)
                    {
                        case 0:
                            var sphere = new SphereMesh();
                            sphere.Radius = dimensions.X;
                            sphere.Height = dimensions.X;
                            collisionVisualizer.Mesh = sphere;
                            break;
                        case 1:
                            var box = new BoxMesh();
                            box.Size = dimensions;
                            collisionVisualizer.Mesh = box;
                            break;
                        case 2:
                            var capsule = new CapsuleMesh();
                            capsule.Radius = dimensions.X;
                            capsule.Height = dimensions.Y;
                            collisionVisualizer.Mesh = capsule;
                            break;
                    }
                    physicsBone.visuallizer = collisionVisualizer;
                    AddChild(collisionVisualizer);
                }
            }

            foreach (var meta in jointMeta)
            {
                int r1 = (int)meta["bone1"];
                int r2 = (int)meta["bone2"];
                string name = (string)meta["name"];
                var joint = bulletWorld.AddJoint(physicsBones[r1].rigidBody, physicsBones[r2].rigidBody, meta);

                mmdJoints.Add(new MMDJoint()
                {
                    joint = joint,
                    name = name,
                });
            }
        }

        public override void _Ready()
        {
            skeleton = GetChild<Skeleton3D>(0);
            morphMesh = skeleton.GetChild<MeshInstance3D>(0);
            modelScale = (float)GetMeta("model_scale", 0.1f);

            for (int i = 0; i < skeleton.GetBoneCount(); i++)
            {
                var bone = new MMDBone()
                {
                    restPosition = skeleton.GetBonePosePosition(i),
                    restRotation = Quaternion.Identity,
                    rotation = Quaternion.Identity,
                    appendRotation = Quaternion.Identity,
                    index = i,
                    name = skeleton.GetBoneName(i),
                    globalRestTransform = skeleton.GetBoneGlobalRest(i)
                };

                bone.restTransform = new Transform3D(Basis.Identity, bone.restPosition);
                bone.invertRestTransform = bone.restTransform.AffineInverse();
                int parentIndex = skeleton.GetBoneParent(i);
                if (parentIndex != -1)
                {
                    bone.parent = mmdBones[skeleton.GetBoneParent(i)];
                }
                mmdBones.Add(bone);
            }

            var appendBones1 = (Godot.Collections.Array<Godot.Collections.Dictionary>)GetMeta("append_bone");
            foreach (var dict in appendBones1)
            {
                var appendBone = new AppendBone()
                {
                    bone = (int)dict["bone"],
                    rotate = (bool)dict["ar"],
                    transition = (bool)dict["at"],
                    index = (int)dict["index"],
                    ratio = (float)dict["ratio"]
                };
                appendBones.Add(appendBone);
            }
            var ikBones1 = (Godot.Collections.Array<Godot.Collections.Dictionary>)GetMeta("ik_bone");
            foreach (var dict in ikBones1)
            {
                var links1 = (Godot.Collections.Array)dict["links"];
                List<IKLink> links = new List<IKLink>();
                foreach (var link1 in links1)
                {
                    var link = (Godot.Collections.Dictionary)link1;
                    var ikLink = new IKLink()
                    {
                        index = (int)link["index"],
                        min = (Vector3)link["min"],
                        max = (Vector3)link["max"],
                        limited = (bool)link["limited"]
                    };

                    if (ikLink.min.X > -Math.PI * 0.5 && ikLink.max.X < Math.PI * 0.5)
                        ikLink.TransformOrder = IKTransformOrder.Zxy;
                    else if (ikLink.min.Y > -Math.PI * 0.5 && ikLink.max.Y < Math.PI * 0.5)
                        ikLink.TransformOrder = IKTransformOrder.Xyz;
                    else
                        ikLink.TransformOrder = IKTransformOrder.Yzx;
                    const float epsilon = 1e-6f;
                    if (ikLink.limited)
                    {
                        uint a = 0;
                        if (Math.Abs(ikLink.min.X) < epsilon &&
                                Math.Abs(ikLink.max.X) < epsilon)
                        {
                            a |= 1;
                        }
                        if (Math.Abs(ikLink.min.Y) < epsilon &&
                                Math.Abs(ikLink.max.Y) < epsilon)
                        {
                            a |= 2;
                        }
                        if (Math.Abs(ikLink.min.Z) < epsilon &&
                                Math.Abs(ikLink.max.Z) < epsilon)
                        {
                            a |= 4;
                        }
                        ikLink.FixTypes = a switch
                        {
                            7 => AxisFixType.FixAll,
                            6 => AxisFixType.FixX,
                            5 => AxisFixType.FixY,
                            3 => AxisFixType.FixZ,
                            _ => AxisFixType.FixNone
                        };
                    }
                    links.Add(ikLink);
                }
                var ikBone = new IKBone()
                {
                    bone = (int)dict["bone"],
                    target = (int)dict["target"],
                    angleLimit = (float)dict["angle_limit"],
                    iterateLimit = (int)dict["iterate_limit"],
                    links = links,
                    enabled = true,
                };
                ikBone.name = skeleton.GetBoneName(ikBone.bone);
                ikBones.Add(ikBone);
            }

            if (UseBulletPhysics)
            {
                CreatePhysics();
            }
        }

        public override void _Process(double delta)
        {
            if (vmdResource != null && vmd == null)
            {
                var bytes = vmdResource.Data;

                vmd = VMDFormat.Load(bytes);
                vmd.Scale(modelScale);
            }

            if (vmd != null)
            {
                currentTime += delta;
                SetAnimation(delta);
            }
        }
        public override void _ExitTree()
        {
            bulletWorld?.Dispose();
            bulletWorld = null;
        }

        void SetAnimation(double delta)
        {
            if (vmd == null)
            {
                return;
            }

            foreach (var pair in vmd.MorphKeyFrameSet)
            {
                int index = morphMesh.FindBlendShapeByName(pair.Key);
                if (index != -1)
                {
                    morphMesh.SetBlendShapeValue(index, GetWeight(pair.Value));
                }
            }
            for (int i = 0; i < mmdBones.Count; i++)
            {
                var bone = mmdBones[i];
                if (!vmd.BoneKeyFrameSet.TryGetValue(bone.name, out var list))
                    continue;
                var frame = GetBoneKeyFrame(list);
                //var rest = skeleton.GetBoneRest(i);
                //skeleton.SetBonePosePosition(i, rest.Origin + GetVector3(frame.Translation));
                //skeleton.SetBonePoseRotation(i, GetQuaternion(frame.Rotation));

                bone.position = GetVector3(frame.Translation);
                bone.rotation = GetQuaternion(frame.Rotation);
            }

            UpdateBonesBase();
            SolveIKs();

            UpdateBonesBase();
            UpdateAppendBones();
            UpdateBones();

            if (UseBulletPhysics)
            {
                UpdateKinematicBones();
                bulletWorld.Simulation(delta);
                UpdatePhysicsBones();
                if (PhysicsDebug)
                {
                    foreach (var bone in physicsBones)
                    {
                        if (bone.visuallizer != null)
                            bone.visuallizer.Transform = MMDPhysicsHelper.GetTransform3D(bone.rigidBody.MotionState.WorldTransform);
                    }
                }
            }
        }


        public void ResetPhysics()
        {
            UpdateBonesBase();

            foreach (var physicsBone in physicsBones)
            {
                var bone = mmdBones[physicsBone.index];
                physicsBone.SetTransform2(bone.transform3D);
            }
            bulletWorld.Simulation(1 / 60.0);
        }
        void UpdateKinematicBones()
        {
            foreach (var physicsBone in physicsBones)
            {
                if (physicsBone.type == 0)
                {
                    var bone = mmdBones[physicsBone.index];
                    physicsBone.SetTransform(bone.transform3D);
                }
            }
        }

        void UpdatePhysicsBones()
        {
            foreach (var physicsBone in physicsBones)
            {
                if (physicsBone.type != 0)
                {
                    var bone = mmdBones[physicsBone.index];

                    bone.transform3D = physicsBone.GetTransform();
                    skeleton.SetBoneGlobalPoseOverride(bone.index, bone.transform3D, 1, true);
                }
            }
        }

        void UpdateBonesBase()
        {
            foreach (var bone in mmdBones)
            {
                bone.ComputeBaseTransform();
            }
        }
        void UpdateBones()
        {
            foreach (var bone in mmdBones)
            {
                if (!bone.isPhysicsBone || !UseBulletPhysics)
                    bone.ComputeTransform();
                skeleton.SetBoneGlobalPoseOverride(bone.index, bone.transform3D, 1, true);
            }
        }

        void UpdateAppendBones()
        {
            foreach (var append in appendBones)
            {
                //mmdBones[append.bone].appendRotation = mmdBones[append.index].rotation;
                //mmdBones[append.bone].transform3D.Basis *= new Basis(Quaternion.Identity.Slerp(mmdBones[append.bone].appendRotation, append.ratio));
                if (append.rotate)
                    mmdBones[append.bone].appendRotation = Quaternion.Identity.Slerp(mmdBones[append.index].rotation.Normalized(), append.ratio);
            }
        }

        float GetWeight(List<MorphKeyFrame> list)
        {
            int l = 0;
            int r = list.Count - 1;
            MorphKeyFrame lf = list[l];
            MorphKeyFrame rf = list[r];

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
                return rf.Weight;
            }
            float factor = ((float)currentTime * 30 - lf.Frame) / (rf.Frame - lf.Frame);
            factor = Math.Clamp(factor, 0, 1);
            return lf.Weight * (1 - factor) + rf.Weight * factor;
        }

        BoneKeyFrame GetBoneKeyFrame(List<BoneKeyFrame> list)
        {
            int l = 0;
            int r = list.Count - 1;
            BoneKeyFrame lf = list[l];
            BoneKeyFrame rf = list[r];

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

            float fx = CubicBezierCurve.Get(GetA(rf.xInterpolator), GetB(rf.xInterpolator)).Sample(factor);
            float fy = CubicBezierCurve.Get(GetA(rf.yInterpolator), GetB(rf.yInterpolator)).Sample(factor);
            float fz = CubicBezierCurve.Get(GetA(rf.zInterpolator), GetB(rf.zInterpolator)).Sample(factor);
            float fr = CubicBezierCurve.Get(GetA(rf.rInterpolator), GetB(rf.rInterpolator)).Sample(factor);

            var translation = new System.Numerics.Vector3(lf.translation.X * (1 - fx) + rf.translation.X * fx,
                lf.translation.Y * (1 - fy) + rf.translation.Y * fy,
                lf.translation.Z * (1 - fz) + rf.translation.Z * fz);
            return new BoneKeyFrame()
            {
                translation = translation,
                rotation = System.Numerics.Quaternion.Normalize(System.Numerics.Quaternion.Slerp(lf.rotation, rf.rotation, fr)),
            };
        }

        bool GetIKEnable(List<IKKeyFrame> list)
        {
            int l = 0;
            int r = list.Count - 1;
            IKKeyFrame lf = list[l];
            IKKeyFrame rf = list[r];

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
            if (f1 > rf.Frame)
            {
                return rf.enable;
            }
            return lf.enable;
        }

        void SolveIKs()
        {
            foreach (var ik in ikBones)
            {
                if (vmd.IKKeyFrameSet.TryGetValue(ik.name, out var list))
                {
                    ik.enabled = GetIKEnable(list);
                }
                else
                {
                    ik.enabled = true;
                }
                SolveIK(ik);
            }
        }
        void SolveIK(IKBone ikBone)
        {
            if (!ikBone.enabled)
                return;
            var ikTip = mmdBones[ikBone.target];
            var ikTargetPosition = mmdBones[ikBone.bone].GetPosition();

            var halfLimit = ikBone.iterateLimit / 2;

            Vector3 tipPosition = ikTip.GetPosition();
            if (ikTargetPosition.DistanceSquaredTo(tipPosition) < 1e-6f)
                return;
            for (int i = 0; i < ikBone.iterateLimit; i++)
            {
                bool axis_limit = i < halfLimit;
                for (int j = 0; j < ikBone.links.Count; j++)
                {
                    var link = ikBone.links[j];
                    var itBone = mmdBones[link.index];
                    var itPosition = itBone.GetPosition();
                    var targetDirection = itPosition.DirectionTo(ikTargetPosition);
                    var ikDirection = itPosition.DirectionTo(tipPosition);

                    float dotV = Mathf.Clamp(targetDirection.Dot(ikDirection), -1, 1);
                    float angle1 = Mathf.Acos(dotV);
                    if (Math.Abs(angle1) < 6e-4f)
                        continue;
                    var invertTransform = itBone.transform3D.Basis.Transposed();
                    var rotateAxis = (invertTransform * targetDirection.Cross(ikDirection)).Normalized();

                    if (axis_limit)
                    {
                        switch (link.FixTypes)
                        {
                            case AxisFixType.FixX:
                                rotateAxis = new Vector3(rotateAxis.X >= 0 ? 1 : -1, 0, 0);
                                break;
                            case AxisFixType.FixY:
                                rotateAxis = new Vector3(0, rotateAxis.Y >= 0 ? 1 : -1, 0);
                                break;
                            case AxisFixType.FixZ:
                                rotateAxis = new Vector3(0, 0, rotateAxis.Z >= 0 ? 1 : -1);
                                break;
                        }
                    }
                    var limit = ikBone.angleLimit * (i + 1);
                    //try
                    //{
                    //    var itResult1 = itBone.rotation * new Quaternion(rotateAxis.Normalized(), -Mathf.Clamp(angle1, -limit, limit)).Normalized();
                    //}
                    //catch
                    //{
                    //    GD.Print($"rot: {rotateAxis}, tar: {targetDirection}, ik: {ikDirection}, angle: {angle1}");
                    //}
                    var itResult = itBone.rotation * new Quaternion(rotateAxis, -Mathf.Clamp(angle1, -limit, limit)).Normalized();
                    if (link.limited)
                    {
                        Vector3 angle = Vector3.Zero;
                        switch (link.TransformOrder)
                        {
                            case IKTransformOrder.Zxy:
                                {
                                    Vector3 cachedE = QuaternionToZxy(itResult);
                                    angle = LimitAngle(cachedE, axis_limit, link.min, link.max);
                                    itResult = new Quaternion(Vector3.Back, angle.Z) * new Quaternion(Vector3.Right, angle.X) * new Quaternion(Vector3.Up, angle.Y);
                                    break;
                                }
                            case IKTransformOrder.Xyz:
                                {
                                    Vector3 cachedE = QuaternionToXyz(itResult);
                                    angle = LimitAngle(cachedE, axis_limit, link.min, link.max);
                                    itResult = new Quaternion(Vector3.Right, angle.X) * new Quaternion(Vector3.Up, angle.Y) * new Quaternion(Vector3.Back, angle.Z);
                                    break;
                                }
                            case IKTransformOrder.Yzx:
                                {
                                    Vector3 cachedE = QuaternionToYzx(itResult);
                                    angle = LimitAngle(cachedE, axis_limit, link.min, link.max);
                                    itResult = new Quaternion(Vector3.Up, angle.Y) * new Quaternion(Vector3.Back, angle.Z) * new Quaternion(Vector3.Right, angle.X);
                                    break;
                                }
                            default:
                                throw new NotImplementedException();
                        }
                    }
                    itResult = itResult.Normalized();
                    itBone.rotation = itResult;
                    var parent = itBone.parent;
                    var it2Tip = invertTransform * (tipPosition - itPosition);
                    var rotatedVec = parent.transform3D.Basis * (itResult * it2Tip);
                    tipPosition = rotatedVec + itPosition;
                }
                UpdateIKBones(ikBone);
                tipPosition = ikTip.GetPosition();
                if (ikTargetPosition.DistanceSquaredTo(tipPosition) < 1e-6f)
                    return;
            }
        }

        void UpdateIKBones(IKBone ikBone)
        {
            for (int i = ikBone.links.Count - 1; i >= 0; i--)
            {
                var link = ikBone.links[i];
                mmdBones[link.index].ComputeBaseTransform();
            }
            mmdBones[ikBone.target].ComputeBaseTransform();
        }


        private Vector3 LimitAngle(Vector3 angle, bool axis_lim, Vector3 low, Vector3 high)
        {
            if (!axis_lim)
            {
                return angle.Clamp(low, high);
            }
            Vector3 vecL1 = 2.0f * low - angle;
            Vector3 vecH1 = 2.0f * high - angle;
            if (angle.X < low.X)
            {
                angle.X = (vecL1.X <= high.X) ? vecL1.X : low.X;
            }
            else if (angle.X > high.X)
            {
                angle.X = (vecH1.X >= low.X) ? vecH1.X : high.X;
            }
            if (angle.Y < low.Y)
            {
                angle.Y = (vecL1.Y <= high.Y) ? vecL1.Y : low.Y;
            }
            else if (angle.Y > high.Y)
            {
                angle.Y = (vecH1.Y >= low.Y) ? vecH1.Y : high.Y;
            }
            if (angle.Z < low.Z)
            {
                angle.Z = (vecL1.Z <= high.Z) ? vecL1.Z : low.Z;
            }
            else if (angle.Z > high.Z)
            {
                angle.Z = (vecH1.Z >= low.Z) ? vecH1.Z : high.Z;
            }
            return angle;
        }

        static Vector2 GetA(Interpolator interpolator)
        {
            return new Vector2(interpolator.ax, interpolator.ay);
        }

        static Vector2 GetB(Interpolator interpolator)
        {
            return new Vector2(interpolator.bx, interpolator.by);
        }

        static Vector3 GetVector3(System.Numerics.Vector3 vec3)
        {
            return new Vector3(vec3.X, vec3.Y, vec3.Z);
        }

        static Quaternion GetQuaternion(System.Numerics.Quaternion q)
        {
            return new Quaternion(q.X, q.Y, q.Z, q.W);
        }



        public static Vector3 QuaternionToXyz(Quaternion quaternion)
        {
            float ii = quaternion.X * quaternion.X;
            float jj = quaternion.Y * quaternion.Y;
            float kk = quaternion.Z * quaternion.Z;
            float ei = quaternion.W * quaternion.X;
            float ej = quaternion.W * quaternion.Y;
            float ek = quaternion.W * quaternion.Z;
            float ij = quaternion.X * quaternion.Y;
            float ik = quaternion.X * quaternion.Z;
            float jk = quaternion.Y * quaternion.Z;
            Vector3 result = new Vector3();
            result.X = MathF.Atan2(2.0f * (ei - jk), 1 - 2.0f * (ii + jj));
            result.Y = MathF.Asin(2.0f * (ej + ik));
            result.Z = MathF.Atan2(2.0f * (ek - ij), 1 - 2.0f * (jj + kk));
            return result;
        }
        public static Vector3 QuaternionToXzy(Quaternion quaternion)
        {
            float ii = quaternion.X * quaternion.X;
            float jj = quaternion.Y * quaternion.Y;
            float kk = quaternion.Z * quaternion.Z;
            float ei = quaternion.W * quaternion.X;
            float ej = quaternion.W * quaternion.Y;
            float ek = quaternion.W * quaternion.Z;
            float ij = quaternion.X * quaternion.Y;
            float ik = quaternion.X * quaternion.Z;
            float jk = quaternion.Y * quaternion.Z;
            Vector3 result = new Vector3();
            result.X = MathF.Atan2(2.0f * (ei + jk), 1 - 2.0f * (ii + kk));
            result.Y = MathF.Atan2(2.0f * (ej + ik), 1 - 2.0f * (jj + kk));
            result.Z = MathF.Asin(2.0f * (ek - ij));
            return result;
        }
        public static Vector3 QuaternionToYxz(Quaternion quaternion)
        {
            float ii = quaternion.X * quaternion.X;
            float jj = quaternion.Y * quaternion.Y;
            float kk = quaternion.Z * quaternion.Z;
            float ei = quaternion.W * quaternion.X;
            float ej = quaternion.W * quaternion.Y;
            float ek = quaternion.W * quaternion.Z;
            float ij = quaternion.X * quaternion.Y;
            float ik = quaternion.X * quaternion.Z;
            float jk = quaternion.Y * quaternion.Z;
            Vector3 result = new Vector3();
            result.X = MathF.Asin(2.0f * (ei - jk));
            result.Y = MathF.Atan2(2.0f * (ej + ik), 1 - 2.0f * (ii + jj));
            result.Z = MathF.Atan2(2.0f * (ek + ij), 1 - 2.0f * (ii + kk));
            return result;
        }
        public static Vector3 QuaternionToYzx(Quaternion quaternion)
        {
            float ii = quaternion.X * quaternion.X;
            float jj = quaternion.Y * quaternion.Y;
            float kk = quaternion.Z * quaternion.Z;
            float ei = quaternion.W * quaternion.X;
            float ej = quaternion.W * quaternion.Y;
            float ek = quaternion.W * quaternion.Z;
            float ij = quaternion.X * quaternion.Y;
            float ik = quaternion.X * quaternion.Z;
            float jk = quaternion.Y * quaternion.Z;
            Vector3 result = new Vector3();
            result.X = MathF.Atan2(2.0f * (ei - jk), 1 - 2.0f * (ii + kk));
            result.Y = MathF.Atan2(2.0f * (ej - ik), 1 - 2.0f * (jj + kk));
            result.Z = MathF.Asin(2.0f * (ek + ij));
            return result;
        }
        public static Vector3 QuaternionToZxy(Quaternion quaternion)
        {
            float ii = quaternion.X * quaternion.X;
            float jj = quaternion.Y * quaternion.Y;
            float kk = quaternion.Z * quaternion.Z;
            float ei = quaternion.W * quaternion.X;
            float ej = quaternion.W * quaternion.Y;
            float ek = quaternion.W * quaternion.Z;
            float ij = quaternion.X * quaternion.Y;
            float ik = quaternion.X * quaternion.Z;
            float jk = quaternion.Y * quaternion.Z;
            Vector3 result = new Vector3();
            result.X = MathF.Asin(2.0f * (ei + jk));
            result.Y = MathF.Atan2(2.0f * (ej - ik), 1 - 2.0f * (ii + jj));
            result.Z = MathF.Atan2(2.0f * (ek - ij), 1 - 2.0f * (ii + kk));
            return result;
        }
        public static Vector3 QuaternionToZyx(Quaternion quaternion)
        {
            float ii = quaternion.X * quaternion.X;
            float jj = quaternion.Y * quaternion.Y;
            float kk = quaternion.Z * quaternion.Z;
            float ei = quaternion.W * quaternion.X;
            float ej = quaternion.W * quaternion.Y;
            float ek = quaternion.W * quaternion.Z;
            float ij = quaternion.X * quaternion.Y;
            float ik = quaternion.X * quaternion.Z;
            float jk = quaternion.Y * quaternion.Z;
            Vector3 result = new Vector3();
            result.X = MathF.Atan2(2.0f * (ei + jk), 1 - 2.0f * (ii + jj));
            result.Y = MathF.Asin(2.0f * (ej - ik));
            result.Z = MathF.Atan2(2.0f * (ek + ij), 1 - 2.0f * (jj + kk));
            return result;
        }

        public static Quaternion XyzToQuaternion(Vector3 euler)
        {
            float cx = MathF.Cos(euler.X * 0.5f);
            float sx = MathF.Sin(euler.X * 0.5f);
            float cy = MathF.Cos(euler.Y * 0.5f);
            float sy = MathF.Sin(euler.Y * 0.5f);
            float cz = MathF.Cos(euler.Z * 0.5f);
            float sz = MathF.Sin(euler.Z * 0.5f);
            Quaternion result;
            result.W = (cx * cy * cz - sx * sy * sz);
            result.X = (sx * cy * cz + cx * sy * sz);
            result.Y = (cx * sy * cz - sx * cy * sz);
            result.Z = (sx * sy * cz + cx * cy * sz);
            return result;
        }
        public static Quaternion XzyToQuaternion(Vector3 euler)
        {
            float cx = MathF.Cos(euler.X * 0.5f);
            float sx = MathF.Sin(euler.X * 0.5f);
            float cy = MathF.Cos(euler.Y * 0.5f);
            float sy = MathF.Sin(euler.Y * 0.5f);
            float cz = MathF.Cos(euler.Z * 0.5f);
            float sz = MathF.Sin(euler.Z * 0.5f);
            Quaternion result;
            result.W = (cx * cy * cz + sx * sy * sz);
            result.X = (sx * cy * cz - cx * sy * sz);
            result.Y = (cx * sy * cz - sx * cy * sz);
            result.Z = (cx * cy * sz + sx * sy * cz);
            return result;
        }
        public static Quaternion YxzToQuaternion(Vector3 euler)
        {
            float cx = MathF.Cos(euler.X * 0.5f);
            float sx = MathF.Sin(euler.X * 0.5f);
            float cy = MathF.Cos(euler.Y * 0.5f);
            float sy = MathF.Sin(euler.Y * 0.5f);
            float cz = MathF.Cos(euler.Z * 0.5f);
            float sz = MathF.Sin(euler.Z * 0.5f);
            Quaternion result;
            result.W = (cx * cy * cz + sx * sy * sz);
            result.X = (sx * cy * cz + cx * sy * sz);
            result.Y = (cx * sy * cz - sx * cy * sz);
            result.Z = (cx * cy * sz - sx * sy * cz);
            return result;
        }
        public static Quaternion YzxToQuaternion(Vector3 euler)
        {
            float cx = MathF.Cos(euler.X * 0.5f);
            float sx = MathF.Sin(euler.X * 0.5f);
            float cy = MathF.Cos(euler.Y * 0.5f);
            float sy = MathF.Sin(euler.Y * 0.5f);
            float cz = MathF.Cos(euler.Z * 0.5f);
            float sz = MathF.Sin(euler.Z * 0.5f);
            Quaternion result;
            result.W = (cx * cy * cz - sx * sy * sz);
            result.X = (sx * cy * cz + cx * sy * sz);
            result.Y = (cx * sy * cz + sx * cy * sz);
            result.Z = (cx * cy * sz - sx * sy * cz);
            return result;
        }
        public static Quaternion ZxyToQuaternion(Vector3 euler)
        {
            float cx = MathF.Cos(euler.X * 0.5f);
            float sx = MathF.Sin(euler.X * 0.5f);
            float cy = MathF.Cos(euler.Y * 0.5f);
            float sy = MathF.Sin(euler.Y * 0.5f);
            float cz = MathF.Cos(euler.Z * 0.5f);
            float sz = MathF.Sin(euler.Z * 0.5f);
            Quaternion result;
            result.W = (cx * cy * cz - sx * sy * sz);
            result.X = (sx * cy * cz - cx * sy * sz);
            result.Y = (cx * sy * cz + sx * cy * sz);
            result.Z = (cx * cy * sz + sx * sy * cz);
            return result;
        }
        public static Quaternion ZYXToQuaternion(Vector3 euler)
        {
            float cx = MathF.Cos(euler.X * 0.5f);
            float sx = MathF.Sin(euler.X * 0.5f);
            float cy = MathF.Cos(euler.Y * 0.5f);
            float sy = MathF.Sin(euler.Y * 0.5f);
            float cz = MathF.Cos(euler.Z * 0.5f);
            float sz = MathF.Sin(euler.Z * 0.5f);
            Quaternion result;
            result.W = (cx * cy * cz + sx * sy * sz);
            result.X = (sx * cy * cz - cx * sy * sz);
            result.Y = (cx * sy * cz + sx * cy * sz);
            result.Z = (cx * cy * sz - sx * sy * cz);
            return result;
        }
    }
}