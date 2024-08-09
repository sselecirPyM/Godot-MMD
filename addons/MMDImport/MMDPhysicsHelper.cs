using Matrix = System.Numerics.Matrix4x4;
using Godot;

namespace Mmd.addons.MMDImport
{
    public static class MMDPhysicsHelper
    {
        public static Transform3D GetTransform3D(this Matrix matrix)
        {
            //return new Transform3D(
            //    (float)matrix[0, 0], (float)matrix[1, 0], (float)matrix[2, 0],
            //    (float)matrix[0, 1], (float)matrix[1, 1], (float)matrix[2, 1],
            //    (float)matrix[0, 2], (float)matrix[1, 2], (float)matrix[2, 2],
            //    (float)matrix[3, 0], (float)matrix[3, 1], (float)matrix[3, 2]);
            return new Transform3D(
                (float)matrix.M11, (float)matrix.M21, (float)matrix.M31,
                (float)matrix.M12, (float)matrix.M22, (float)matrix.M32,
                (float)matrix.M13, (float)matrix.M23, (float)matrix.M33,
                (float)matrix.M41, (float)matrix.M42, (float)matrix.M43);
        }

        public static Matrix GetMatrix(this Transform3D transform)
        {
            return new Matrix(
                transform[0][0], transform[0][1], transform[0][2], 0,
                transform[1][0], transform[1][1], transform[1][2], 0,
                transform[2][0], transform[2][1], transform[2][2], 0,
                transform[3][0], transform[3][1], transform[3][2], 1);
        }

        public static Matrix GetMatrix(Godot.Vector3 position, Godot.Vector3 rotation)
        {
            return Matrix.CreateFromYawPitchRoll(rotation.Y, rotation.X, rotation.Z) * Matrix.CreateTranslation(position.X, position.Y, position.Z);
        }

        public static Matrix Invert(this Matrix matrix)
        {
            Matrix.Invert(matrix, out var result);
            return result;
        }

    }
}
