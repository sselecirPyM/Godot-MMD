using Godot;

namespace Mmd.Scripts
{
    public partial class ShowFPS : Label
    {
        double accumulate;
        int count;

        public override void _Process(double delta)
        {
            accumulate += delta;
            count++;
            if (accumulate >= 1)
            {
                Text = $"FPS:{count / accumulate:F2}";
                accumulate = 0;
                count = 0;
            }
        }
    }
}
