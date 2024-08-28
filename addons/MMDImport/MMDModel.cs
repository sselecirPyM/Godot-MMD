using Godot;
using Mmd.addons.MMDImport.Util;
using System;
using System.Collections.Generic;
using Matrix = System.Numerics.Matrix4x4;
using Vector3 = System.Numerics.Vector3;
using Quaternion = System.Numerics.Quaternion;

namespace Mmd.addons.MMDImport
{
    [Tool]
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
            public Matrix restTransform;
            public Matrix globalRestTransform;

            public Matrix invertRestTransform;

            public Vector3 position;
            public Quaternion rotation;

            public Quaternion ikRotation;

            public Vector3 appendPosition;
            public Quaternion appendRotation;
            public int index;
            public MMDBone parent;
            public string name;

            public Matrix transform;

            public bool isPhysicsBone;

            public Vector3 GetPosition()
            {
                return transform.Translation;
            }

            public void ComputeTransform()
            {
                if (parent != null)
                {
                    transform = TRS(position + appendPosition, rotation * appendRotation) * restTransform * parent.transform;
                }
                else
                {
                    transform = TRS(position + appendPosition, rotation * appendRotation) * restTransform;
                }
            }

            public void ComputeBaseTransform()
            {
                if (parent != null)
                {
                    transform = TRS(position, rotation) * restTransform * parent.transform;
                }
                else
                {
                    transform = TRS(position, rotation) * restTransform;
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

            public void SetTransform(Matrix transform)
            {
                rigidBody.MotionState.WorldTransform = offset * transform;
            }

            public void SetTransform2(Matrix transform)
            {
                rigidBody.WorldTransform = offset * transform;
                rigidBody.MotionState.WorldTransform = rigidBody.WorldTransform;
            }

            public Matrix GetTransform2()
            {
                return invertOffset * rigidBody.MotionState.WorldTransform;
            }
        }
        class MMDJoint
        {
            public BulletSharp.TypedConstraint joint;
            public string name;
        }

        class MorphDesc
        {
            public string type;
            public int Index;
            public Vector3 Translation;
            public Quaternion Rotation;
            public string MorphName;
            public float Rate;
        }

        double _currentTime;

        [Export]
        public double currentTime
        {
            get => _currentTime;
            set { _currentTime = value; }
        }
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
        //[Export]
        //public bool PhysicsDebug;
        [Export]
        public bool AutoPlay = true;
        [Export]
        public bool Playing;

        VMDResource _vmdResource;
        public Skeleton3D skeleton;
        public MeshInstance3D morphMesh;
        public float modelScale;

        List<AppendBone> appendBones = new List<AppendBone>();
        List<IKBone> ikBones = new List<IKBone>();
        List<PhysicsBone> physicsBones = new List<PhysicsBone>();

        List<MMDBone> mmdBones = new List<MMDBone>();
        List<MMDJoint> mmdJoints = new List<MMDJoint>();

        Dictionary<string, MorphDesc[]> morphDescs;

        MMDBulletWorld bulletWorld = null;
        void CreatePhysics()
        {
            if (bulletWorld != null)
                return;
            bulletWorld = new MMDBulletWorld();
            bulletWorld.Initialize();

            var physicsBoneMeta = (Godot.Collections.Array<Godot.Collections.Dictionary>)GetMeta("physics_bone");
            foreach (var meta in physicsBoneMeta)
            {
                var rigidBody = bulletWorld.AddRigidBody(meta);
                int type = (int)meta["type"];
                int index = (int)meta["bone_index"];
                string name = (string)meta["name"];

                var mmdBone = mmdBones[index];
                mmdBone.isPhysicsBone = type != 0;
                var offset = rigidBody.WorldTransform * mmdBone.globalRestTransform.Invert();

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

                //if (PhysicsDebug)
                //{
                //    int shape = (int)meta["shape"];
                //    var dimensions = (Godot.Vector3)meta["dimensions"];
                //    var position = (Godot.Vector3)meta["position"];
                //    var rotation = (Godot.Vector3)meta["rotation"];
                //    MeshInstance3D collisionVisualizer = new MeshInstance3D();
                //    collisionVisualizer.Transform = MMDPhysicsHelper.GetTransform3D(rigidBody.MotionState.WorldTransform);

                //    switch (shape)
                //    {
                //        case 0:
                //            var sphere = new SphereMesh();
                //            sphere.Radius = dimensions.X;
                //            sphere.Height = dimensions.X;
                //            collisionVisualizer.Mesh = sphere;
                //            break;
                //        case 1:
                //            var box = new BoxMesh();
                //            box.Size = dimensions;
                //            collisionVisualizer.Mesh = box;
                //            break;
                //        case 2:
                //            var capsule = new CapsuleMesh();
                //            capsule.Radius = dimensions.X;
                //            capsule.Height = dimensions.Y;
                //            collisionVisualizer.Mesh = capsule;
                //            break;
                //    }
                //    physicsBone.visuallizer = collisionVisualizer;
                //    AddChild(collisionVisualizer);
                //}
            }

            var jointMeta = (Godot.Collections.Array<Godot.Collections.Dictionary>)GetMeta("joint");
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

        void FreePhysicsResources()
        {
            if (bulletWorld == null)
                return;
            foreach (var joint in mmdJoints)
            {
                bulletWorld.world.RemoveConstraint(joint.joint);
                joint.joint.Dispose();
                joint.joint = null;
            }
            mmdJoints.Clear();
            foreach (var rigidBody in physicsBones)
            {
                bulletWorld.world.RemoveRigidBody(rigidBody.rigidBody);
                rigidBody.rigidBody.Dispose();
                rigidBody.rigidBody = null;
            }
            physicsBones.Clear();
            bulletWorld?.Dispose();
            bulletWorld = null;
        }

        void CreateGenericResource()
        {
            if (mmdBones.Count > 0)
                return;
            skeleton = GetChild<Skeleton3D>(0);
            morphMesh = skeleton.GetChild<MeshInstance3D>(0);
            modelScale = (float)GetMeta("model_scale", 0.1f);

            for (int i = 0; i < skeleton.GetBoneCount(); i++)
            {
                var bone = new MMDBone()
                {
                    restPosition = GetVector3S(skeleton.GetBonePosePosition(i)),
                    restRotation = System.Numerics.Quaternion.Identity,
                    rotation = System.Numerics.Quaternion.Identity,
                    appendRotation = System.Numerics.Quaternion.Identity,
                    index = i,
                    name = skeleton.GetBoneName(i),
                    globalRestTransform = MMDPhysicsHelper.GetMatrix(skeleton.GetBoneGlobalRest(i))
                };

                bone.restTransform = Matrix.CreateTranslation(bone.restPosition);
                Matrix.Invert(bone.restTransform, out bone.invertRestTransform);

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
                        min = GetVector3S((Godot.Vector3)link["min"]),
                        max = GetVector3S((Godot.Vector3)link["max"]),
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

        }

        public override void _Process(double delta)
        {
            CreateGenericResource();
            if (UseBulletPhysics)
            {
                CreatePhysics();
            }
            if (!UseBulletPhysics)
            {
                FreePhysicsResources();
            }
            if (vmdResource != null && vmd == null)
            {
                var bytes = vmdResource.Data;

                vmd = VMDFormat.Load(bytes);
                vmd.Scale(modelScale);
                ClearStates();
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
                SetAnimation(delta);
            }
        }
        public override void _ExitTree()
        {
            FreePhysicsResources();
        }

        void SetAnimation(double delta)
        {
            if (vmd == null)
            {
                return;
            }
            if (morphDescs == null)
            {
                morphDescs = new Dictionary<string, MorphDesc[]>();
                var boneMorphMeta = (Godot.Collections.Dictionary)GetMeta("morph");
                foreach (var meta in boneMorphMeta)
                {
                    var arr = (Godot.Collections.Array)meta.Value;
                    var arr1 = new MorphDesc[arr.Count];
                    for (int i = 0; i < arr.Count; i++)
                    {
                        var dict = (Godot.Collections.Dictionary)arr[i];
                        if (dict["type"].AsString() == "bone")
                        {
                            arr1[i] = new MorphDesc()
                            {
                                type = dict["type"].AsString(),
                                Index = (int)dict["bone"],
                                Rotation = GetQuaternionS((Godot.Quaternion)dict["rotation"]),
                                Translation = GetVector3S((Godot.Vector3)dict["translation"]),
                            };
                        }
                        else
                        {
                            arr1[i] = new MorphDesc()
                            {
                                type = dict["type"].AsString(),
                                MorphName = (string)dict["morph"],
                                Rate = (float)dict["rate"],
                            };
                        }
                    }
                    morphDescs[meta.Key.AsString()] = arr1;
                }
            }


            for (int i = 0; i < mmdBones.Count; i++)
            {
                var bone = mmdBones[i];
                bone.appendRotation = System.Numerics.Quaternion.Identity;
                if (!vmd.BoneKeyFrameSet.TryGetValue(bone.name, out var list))
                {
                    bone.position = System.Numerics.Vector3.Zero;
                    bone.rotation = System.Numerics.Quaternion.Identity;
                    continue;
                }
                var frame = GetBoneKeyFrame(list);

                bone.position = frame.Translation;
                bone.rotation = frame.Rotation;
            }
            foreach (var pair in vmd.MorphKeyFrameSet)
            {
                var weight = GetWeight(pair.Value);
                _SetMorph(pair.Key, weight);
            }

            UpdateBonesBase();
            UpdateAppendBones();
            UpdateBones();
            SolveIKs();

            UpdateBonesBase();
            UpdateAppendBones();
            UpdateBones();

            if (bulletWorld != null)
            {
                UpdateKinematicBones();
                bulletWorld.Simulation(delta);
                UpdatePhysicsBones();
                //if (PhysicsDebug)
                //{
                //    foreach (var bone in physicsBones)
                //    {
                //        if (bone.visuallizer != null)
                //            bone.visuallizer.Transform = MMDPhysicsHelper.GetTransform3D(bone.rigidBody.MotionState.WorldTransform);
                //    }
                //}
            }
        }


        void ClearStates()
        {
            for (int i = 0; i < morphMesh.GetBlendShapeCount(); i++)
            {
                morphMesh.SetBlendShapeValue(i, 0.0f);
            }
        }

        //void _SetMorph(string name, float weight)
        //{
        //    _SetMorph1(name, weight);
        //}
        void _SetMorph(string name, float weight)
        {
            int index = morphMesh.FindBlendShapeByName(name);
            if (index != -1)
            {
                morphMesh.SetBlendShapeValue(index, weight);
            }
            if (morphDescs.TryGetValue(name, out var ms))
            {
                foreach (var b in ms)
                {
                    if (b.type == "bone")
                    {
                        mmdBones[b.Index].position += b.Translation * weight;
                        mmdBones[b.Index].rotation *= System.Numerics.Quaternion.Slerp(System.Numerics.Quaternion.Identity, b.Rotation, weight);
                    }
                    else if (b.type == "group")
                    {
                        _SetMorph(b.MorphName, b.Rate * weight);
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
                physicsBone.SetTransform2(bone.transform);
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
                    physicsBone.SetTransform(bone.transform);
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

                    bone.transform = physicsBone.GetTransform2();
                    skeleton.SetBoneGlobalPoseOverride(bone.index, MMDPhysicsHelper.GetTransform3D(bone.transform), 1, true);
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
                skeleton.SetBoneGlobalPoseOverride(bone.index, MMDPhysicsHelper.GetTransform3D(bone.transform), 1, true);
            }
        }

        void UpdateAppendBones()
        {
            foreach (var append in appendBones)
            {
                if (append.rotate)
                {
                    mmdBones[append.bone].appendRotation = System.Numerics.Quaternion.Normalize(System.Numerics.Quaternion.Slerp(System.Numerics.Quaternion.Identity,
                        mmdBones[append.index].rotation, append.ratio));
                }
                if (append.transition)
                {
                    mmdBones[append.bone].appendPosition = mmdBones[append.index].position * append.ratio;
                }
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
            if (Vector3.DistanceSquared(ikTargetPosition, tipPosition) < 1e-6f)
                return;
            for (int i = 0; i < ikBone.iterateLimit; i++)
            {
                bool axis_limit = i < halfLimit;
                for (int j = 0; j < ikBone.links.Count; j++)
                {
                    var link = ikBone.links[j];
                    var itBone = mmdBones[link.index];
                    var itPosition = itBone.GetPosition();
                    var targetDirection = Vector3.Normalize(ikTargetPosition - itPosition);
                    var ikDirection = Vector3.Normalize(tipPosition - itPosition);

                    float dotV = Mathf.Clamp(Vector3.Dot(targetDirection, ikDirection), -1, 1);
                    float angle1 = Mathf.Acos(dotV);
                    if (Math.Abs(angle1) < 6e-4f)
                        continue;
                    var invertTransform = Matrix.Transpose(itBone.transform);
                    var rotateAxis = Vector3.TransformNormal(Vector3.Cross(targetDirection, ikDirection), invertTransform);

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

                    var itResult = System.Numerics.Quaternion.Normalize(itBone.rotation * QAxisAngle(Vector3.Normalize(rotateAxis), -Math.Clamp(angle1, -limit, limit)));
                    if (link.limited)
                    {
                        Vector3 angle = Vector3.Zero;
                        switch (link.TransformOrder)
                        {
                            case IKTransformOrder.Zxy:
                                {
                                    Vector3 cachedE = QuaternionToZxy(itResult);
                                    angle = LimitAngle(cachedE, axis_limit, link.min, link.max);
                                    itResult = QAxisAngle(Vector3.UnitZ, angle.Z) * QAxisAngle(Vector3.UnitX, angle.X) * QAxisAngle(Vector3.UnitY, angle.Y);
                                    break;
                                }
                            case IKTransformOrder.Xyz:
                                {
                                    Vector3 cachedE = QuaternionToXyz(itResult);
                                    angle = LimitAngle(cachedE, axis_limit, link.min, link.max);
                                    itResult = QAxisAngle(Vector3.UnitX, angle.X) * QAxisAngle(Vector3.UnitY, angle.Y) * QAxisAngle(Vector3.UnitZ, angle.Z);
                                    break;
                                }
                            case IKTransformOrder.Yzx:
                                {
                                    Vector3 cachedE = QuaternionToYzx(itResult);
                                    angle = LimitAngle(cachedE, axis_limit, link.min, link.max);
                                    itResult = QAxisAngle(Vector3.UnitY, angle.Y) * QAxisAngle(Vector3.UnitZ, angle.Z) * QAxisAngle(Vector3.UnitX, angle.X);
                                    break;
                                }
                            default:
                                throw new NotImplementedException();
                        }
                    }
                    itResult = System.Numerics.Quaternion.Normalize(itResult);
                    itBone.rotation = itResult;
                    var parent = itBone.parent;
                    var it2Tip = Vector3.TransformNormal(tipPosition - itPosition, invertTransform);
                    var rotatedVec = Vector3.TransformNormal(Vector3.Transform(it2Tip, itResult), parent.transform);
                    tipPosition = rotatedVec + itPosition;
                }
                UpdateIKBones(ikBone);
                tipPosition = ikTip.GetPosition();
                if (Vector3.DistanceSquared(ikTargetPosition, tipPosition) < 1e-6f)
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
                return Vector3.Clamp(angle, low, high);
                //return angle.Clamp(low, high);
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

        static Godot.Vector3 GetVector3(System.Numerics.Vector3 vec3)
        {
            return new Godot.Vector3(vec3.X, vec3.Y, vec3.Z);
        }

        static System.Numerics.Vector3 GetVector3S(Godot.Vector3 vec3)
        {
            return new System.Numerics.Vector3(vec3.X, vec3.Y, vec3.Z);
        }

        static Godot.Quaternion GetQuaternion(System.Numerics.Quaternion q)
        {
            return new Godot.Quaternion(q.X, q.Y, q.Z, q.W);
        }

        static System.Numerics.Quaternion GetQuaternionS(Godot.Quaternion q)
        {
            return new System.Numerics.Quaternion(q.X, q.Y, q.Z, q.W);
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
        static Quaternion QAxisAngle(Vector3 axis, float angle) => System.Numerics.Quaternion.CreateFromAxisAngle(axis, angle);

        static Matrix TRS(Vector3 position, Quaternion rotation)
        {
            return Matrix.CreateFromQuaternion(rotation) * Matrix.CreateTranslation(position);
        }
    }
}