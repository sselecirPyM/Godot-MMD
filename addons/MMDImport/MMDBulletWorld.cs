using BulletSharp;
using Godot.Collections;
using System;

using Matrix = System.Numerics.Matrix4x4;
using Vector3 = System.Numerics.Vector3;

namespace Mmd.addons.MMDImport
{
    public class MMDBulletWorld : IDisposable
    {
        public DbvtBroadphase broadphase = new DbvtBroadphase();
        public DefaultCollisionConfiguration defaultCollisionConfiguration = new DefaultCollisionConfiguration();
        public SequentialImpulseConstraintSolver sequentialImpulseConstraintSolver = new SequentialImpulseConstraintSolver();
        public Dispatcher dispatcher;
        public DiscreteDynamicsWorld world;

        public Godot.Vector3 gravity = new Godot.Vector3(0, -9.81f, 0);

        public void Initialize()
        {
            dispatcher = new CollisionDispatcher(defaultCollisionConfiguration);
            world = new DiscreteDynamicsWorld(dispatcher, broadphase, sequentialImpulseConstraintSolver, defaultCollisionConfiguration);
            Vector3 gravity = GetVector3(this.gravity);
            world.SetGravity(ref gravity);
        }

        public void Simulation(double step)
        {
            world.StepSimulation((float)step);

        }

        public void SetGravity(Godot.Vector3 g)
        {
            this.gravity = g;
            Vector3 gravity = GetVector3(this.gravity);
            world.SetGravity(ref gravity);
        }

        public RigidBody AddRigidBody(Dictionary meta)
        {
            int shape = (int)meta["shape"];
            int type = (int)meta["type"];
            var collisionGroup = (byte)meta["collision_group"];
            var collisionMask = (ushort)meta["collision_mask"];

            var position = (Godot.Vector3)meta["position"];
            var rotation = (Godot.Vector3)meta["rotation"];
            var dimensions = (Godot.Vector3)meta["dimensions"];

            float mass = (float)meta["mass"];
            float linearDamping = (float)meta["linear_damp"];
            float angularDamping = (float)meta["angular_damp"];
            float friction = (float)meta["friction"];
            float restitution = (float)meta["bounce"];
            var worldTransform = MMDPhysicsHelper.GetMatrix(position, rotation);
            MotionState motionState = new DefaultMotionState(worldTransform);
            motionState.WorldTransform = worldTransform;
            CollisionShape collisionShape;
            switch (shape)
            {
                case 0://sphere
                    collisionShape = new SphereShape(dimensions.X);
                    break;
                case 2://capsule
                    collisionShape = new CapsuleShape(dimensions.X, dimensions.Y);
                    break;
                case 1://box
                default:
                    collisionShape = new BoxShape(GetVector3(dimensions));
                    break;
            }

            var localInertia = new Vector3();
            if (type == 0)
            {
                mass = 0;
            }
            else
            {
                collisionShape.CalculateLocalInertia(mass, out localInertia);
            }
            var rigidbodyInfo = new RigidBodyConstructionInfo(mass, motionState, collisionShape, localInertia);
            rigidbodyInfo.Friction = friction;
            rigidbodyInfo.LinearDamping = linearDamping;
            rigidbodyInfo.AngularDamping = angularDamping;
            rigidbodyInfo.Restitution = restitution;
            var rigidBody = new RigidBody(rigidbodyInfo);
            rigidBody.ActivationState = ActivationState.DisableDeactivation;
            rigidBody.SetSleepingThresholds(0, 0);

            //see pmx define. Kinematic = 0, Physics = 1
            if (type == 0)
            {
                rigidBody.CollisionFlags |= CollisionFlags.KinematicObject;
            }
            world.AddRigidBody(rigidBody, 1 << collisionGroup, collisionMask);

            return rigidBody;
        }

        public TypedConstraint AddJoint(RigidBody r1, RigidBody r2, Dictionary meta)
        {
            var position = (Godot.Vector3)meta["position"];
            var rotation = (Godot.Vector3)meta["rotation"];
            var linearMin = (Godot.Vector3)meta["linear_min"];
            var linearMax = (Godot.Vector3)meta["linear_max"];
            var linearSpring = (Godot.Vector3)meta["linear_spring"];
            var angularMin = (Godot.Vector3)meta["angular_min"];
            var angularMax = (Godot.Vector3)meta["angular_max"];
            var angularSpring = (Godot.Vector3)meta["angular_spring"];



            Matrix t0 = MMDPhysicsHelper.GetMatrix(position, rotation);
            var t1 = t0 * r1.WorldTransform.Invert();
            var t2 = t0 * r2.WorldTransform.Invert();
            //var t1 = t0 * invertR1;
            //var t2 = t0 * invertR2;

            var joint = new Generic6DofSpringConstraint(r1, r2, t1, t2, true);

            joint.LinearLowerLimit = GetVector3(linearMin);
            joint.LinearUpperLimit = GetVector3(linearMax);
            joint.AngularLowerLimit = GetVector3(angularMin);
            joint.AngularUpperLimit = GetVector3(angularMax);


            S(0, linearSpring.X);
            S(1, linearSpring.Y);
            S(2, linearSpring.Z);
            S(3, angularSpring.X);
            S(4, angularSpring.Y);
            S(5, angularSpring.Z);
            void S(int index, float f)
            {
                if (f != 0.0f)
                {
                    joint.EnableSpring(index, true);
                    joint.SetStiffness(index, f);
                }
                else
                {
                    joint.EnableSpring(index, false);
                }
            }

            world.AddConstraint(joint);
            return joint;
        }

        public void RemoveJoint(TypedConstraint joint)
        {
            world.RemoveConstraint(joint);
        }

        public void RemoveRigidBody(RigidBody rigidBody)
        {
            world.RemoveRigidBody(rigidBody);
        }

        static Vector3 GetVector3(Godot.Vector3 vector3)
        {
            return new Vector3(vector3.X, vector3.Y, vector3.Z);
        }

        public void Dispose()
        {
            defaultCollisionConfiguration?.Dispose();
            broadphase?.Dispose();
            world?.Dispose();
            dispatcher?.Dispose();
            sequentialImpulseConstraintSolver?.Dispose();
        }


    }
}
