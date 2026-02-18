using System.Numerics;

namespace Mmd.addons.MMDImport.Util
{
    public static class CurveExtension
    {
        public static float Sample(this Interpolator interpolator,float factor)
        {
            return CubicBezierCurve.Get(GetA(interpolator), GetB(interpolator)).Sample(factor);
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
