#include "model/chassis_model.hpp"
#include "sim/viewer.hpp"
#include "sim/world.hpp"
#include "types/types.hpp"

#include <chrono>
#include <thread>

int main()
{
    T1::sim::WorldConfig cfg;
    cfg.mjcf_path = T1_TEST_MJCF;
    cfg.timestep = 0.001;
    cfg.effort_limits << 35.0, 35.0, 200.0, 12.0, 12.0, 12.0;

    T1::sim::MujocoWorld world(cfg);
    world.reset();

    T1::MitCommand cmd;
    cmd.q[0] = 0.3;
    cmd.kp << 20.0, 80.0, 5000.0, 80.0, 80.0, 80.0;
    cmd.kd << 8.0, 5.0, 200.0, 2.0, 2.0, 2.0;
    world.set_arm_command(cmd);

    T1::WheelSpeeds wheels;
    wheels.left = 5.0;
    wheels.right = -5.0;
    world.set_wheel_command(wheels);

    T1::sim::SimViewer viewer(world);
    viewer.start();
    while (viewer.alive())
    {
        world.step();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    viewer.stop();

    return 0;
}