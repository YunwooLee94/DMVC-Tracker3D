//
// Created by larr-planning on 24. 3. 3.
//

#ifndef DMVC_TRACKER_BERNSTEIN_UTIL3D_H
#define DMVC_TRACKER_BERNSTEIN_UTIL3D_H

#include <cmath>
#include <dmvc_utils3d/Utils.h>

namespace dmvc3d{
    int factorial(int num);

    int nchoosek(int n, int r);

    int nchooser(int n, int r);

    double getBernsteinValue(double bern_ctrl_pts[], double t, double t0, double tf, int poly_order);

}



#endif //DMVC_TRACKER_BERNSTEIN_UTIL_H
