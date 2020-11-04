
#include <map>
#include <utility>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdio.h>

#include "grid.hh"

using namespace std;

typedef pair<int,int> coords;


static std::map<coords, int> grid;

static
int
iclamp(int xmin, int xx, int xmax)
{
    if (xx < xmin) return xmin;
    if (xx > xmax) return xmax;
    return xx;
}

static
string
coords_to_s(coords cc)
{
    char temp[100];
    float xx = cc.first * CELL_SIZE;
    float yy = cc.second * CELL_SIZE;
    snprintf(temp, 100, "(k:%.02f,%.02f)", xx, yy);
    return string(temp);
}

static
int
grid_get(coords kk)
{
    try {
        return grid.at(kk);
    }
    catch(const std::out_of_range& _ee) {
        return 0;
    }
}

static
void
grid_put(coords kk, int xx)
{
    //cout << "put " << coords_to_s(kk) << " " << xx << endl;
    grid[kk] = xx;
}

static
void
grid_inc(coords kk, int dv)
{
    int vv = grid_get(kk);
    grid_put(kk, iclamp(0, vv + dv, 100));
}

static
int
c2k(float cc)
{
    return int(round(cc / CELL_SIZE));
}

static
coords
key(float xx, float yy)
{
    return make_pair(c2k(xx), c2k(yy));
}


void
grid_apply_hit(LaserHit hit, Pose pose)
{
    set<coords> cells;

    for (float ds = 0.0f; ds < (hit.range - CELL_SIZE); ds += 0.1f) {
        float xx = pose.x + ds * cos(pose.t + hit.angle);
        float yy = pose.y + ds * sin(pose.t + hit.angle);
        cells.insert(key(xx, yy));
    }

    for (auto cell : cells) {
        grid_inc(cell, -2);
    }

    float hx = pose.x + hit.range * cos(pose.t + hit.angle);
    float hy = pose.y + hit.range * sin(pose.t + hit.angle);
    coords hk = key(hx, hy);
    grid_inc(hk, +5);

    /*
    cout << "Hit @ " << hx << "," << hy
         << " => " << coords_to_s(hk) << endl;
    */
}

GridView
grid_view(Pose pose)
{
    //cout << "get grid_view @ " << pose.to_s() << endl;
    //cout << fixed << setprecision(2) << endl;

    GridView gv(VIEW_SIZE, VIEW_SIZE);

    for (int ii = 0; ii < VIEW_SIZE; ++ii) {
        float dx = CELL_SIZE * (ii - (VIEW_SIZE/2));
        float xx = pose.x + dx;

        for (int jj = 0; jj < VIEW_SIZE; ++jj) {
            float dy = CELL_SIZE * (jj - (VIEW_SIZE/2));
            float yy = pose.y + dy;

            coords kk = key(xx, yy);
            int vv = grid_get(kk);

            /*
            cout << coords_to_s(kk) << " "
                 << vv << " ";
            */

            float vf = vv / 100.0f;
            gv.put(jj, ii, vf);
        }
        //cout << endl;
    }

    //cout << endl;

    return gv;
}
