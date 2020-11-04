#ifndef GRID_HH
#define GRID_HH

#include "robot.hh"
#include "viz.hh"
#include "pose.hh"

static const float CELL_SIZE = 0.25;
static const float VIEW_SIZE = 41;

void grid_apply_hit(LaserHit hit, Pose pose);
GridView grid_view(Pose pose);

#endif
