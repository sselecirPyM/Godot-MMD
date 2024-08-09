using Godot;
using Mmd.addons.MMDImport;
using System;
using System.Collections.Generic;

namespace Mmd.Scripts
{
    public partial class FreeLookCamera : Camera3D
    {
        [Export]
        float speed = 0.5f;
        [Export]
        float sensitiveH = 0.008f;
        [Export]
        float sensitiveV = 0.008f;

        Vector3 velocity;

        public override void _Ready()
        {
            Input.UseAccumulatedInput = false;
        }

        public override void _UnhandledInput(InputEvent @event)
        {
            if (@event is InputEventMouseMotion mouse)
            {
                if (mouse.ButtonMask.HasFlag(MouseButtonMask.Left) || mouse.ButtonMask.HasFlag(MouseButtonMask.Middle))
                {
                    Vector3 angle = Rotation;
                    angle.Y += -mouse.Relative.X * sensitiveH;
                    angle.X += -mouse.Relative.Y * sensitiveV;
                    Rotation = angle;
                }
            }
        }
        public override void _UnhandledKeyInput(InputEvent @event)
        {
            if (@event is InputEventKey key)
            {
                if (key.Keycode == Key.J && key.Pressed)
                {
                    foreach (var mmdModel in GetMMDModels())
                    {
                        mmdModel.currentTime -= 5;
                    }
                }
                if (key.Keycode == Key.K && key.Pressed)
                {
                    foreach (var mmdModel in GetMMDModels())
                    {
                        mmdModel.currentTime += 5;
                    }
                }
                if (key.Keycode == Key.L && key.Pressed)
                {
                    foreach (var mmdModel in GetMMDModels())
                    {
                        mmdModel.currentTime = 0;
                    }
                }
                if (key.Keycode == Key.R && key.Pressed)
                {
                    foreach (var mmdModel in GetMMDModels())
                    {
                        GD.Print("reset physics");
                        mmdModel.ResetPhysics();
                    }
                }
            }
        }

        public override void _Process(double delta)
        {
            float forward = 0;
            float right = 0;
            float up = 0;
            if (Input.IsKeyPressed(Key.W))
                forward = 1;
            else if (Input.IsKeyPressed(Key.S))
                forward = -1;

            if (Input.IsKeyPressed(Key.D))
                right = 1;
            else if (Input.IsKeyPressed(Key.A))
                right = -1;
            if (Input.IsKeyPressed(Key.E))
                up = 1;
            else if (Input.IsKeyPressed(Key.Q))
                up = -1;

            Vector3 t = Quaternion * (new Vector3(right, up, -forward) * speed);

            velocity += t * (float)delta;
            velocity *= MathF.Pow(0.001f, (float)delta);

            Position += velocity;
        }

        public IEnumerable<MMDModel> GetMMDModels()
        {
            var nodes = GetParent().FindChildren("*", recursive: true);

            foreach (var node in nodes)
            {
                if (node is MMDModel mmdModel)
                {
                    yield return mmdModel;
                }
            }
        }
    }
}