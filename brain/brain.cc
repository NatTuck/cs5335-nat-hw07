
#include <iostream>
#include <thread>
#include <math.h>

#include "robot.hh"
#include "grid.hh"
#include "viz.hh"

using std::cout;
using std::endl;

void
callback(Robot* robot)
{
    if (robot->ranges.size() < 5) {
        return;
    }

    Pose pose(robot->pos_x, robot->pos_y, robot->pos_t);

    for (auto hit : robot->ranges) {
        if (hit.range < 100) {
            grid_apply_hit(hit, pose);
        }
    }

    GridView view = grid_view(pose);
    viz_show(view);

    //robot->set_vel(0, 0);

    float lft = clamp(0.0, robot->ranges[2].range, 2.0);
    float fwd = clamp(0.0, robot->ranges[3].range, 2.0);
    float rgt = clamp(0.0, robot->ranges[4].range, 2.0);

    float spd = fwd - 1.0;
    float trn = clamp(-1.0, lft - rgt, 1.0);

    if (fwd < 1.2) {
      spd = 0;
      trn = 1;
    }

    //cout << "spd,trn = " << spd << "," << trn << endl;
    robot->set_vel(spd + trn, spd - trn);
}

void
robot_thread(Robot* robot)
{
    robot->do_stuff();
}

int
main(int argc, char* argv[])
{
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    std::thread rthr(robot_thread, &robot);

    return viz_run(argc, argv);
}
