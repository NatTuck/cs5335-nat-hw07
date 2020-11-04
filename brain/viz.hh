#ifndef VIZ_H
#define VIZ_H

#include <cassert>
#include <vector>
using std::vector;

class GridView {
  public:
    int ww;
    int hh;
    vector<float> data;

    GridView(int w, int h)
        : ww(w), hh(h), data(w*h, 0.0f)
    {}

    void
    put(int xx, int yy, float vv)
    {
        assert(xx >= 0 && yy >= 0 && xx < ww && yy < hh);
        data.at(yy*ww+xx) = vv;
    }

    float
    get(int xx, int yy)
    {
        assert(xx >= 0 && yy >= 0 && xx < ww && yy < hh);
        return data.at(yy*ww+xx);
    }
};

int viz_run(int argc, char **argv);
int viz_show(GridView view);

#endif
