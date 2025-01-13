//
// Created by larr-planning on 24. 3. 3.
//

#ifndef DMVC_TRACKER_UTILS3D_H
#define DMVC_TRACKER_UTILS3D_H
#include <vector>
#include <thread>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace dmvc3d{
    using namespace std;

    const vector<int> n_equals_0{1};
    const vector<int> n_equals_1{1, 1};
    const vector<int> n_equals_2{1, 2, 1};
    const vector<int> n_equals_3{1, 3, 3, 1};
    const vector<int> n_equals_4{1, 4, 6, 4, 1};
    const vector<int> n_equals_5{1, 5, 10, 10, 5, 1};
    const vector<int> n_equals_6{1, 6, 15, 20, 15, 6, 1};
    const vector<int> n_equals_7{1, 7, 21, 35, 35, 21, 7, 1};
    const vector<int> n_equals_8{1, 8, 28, 56, 70, 56, 28, 8, 1};
    const vector<int> n_equals_9{1, 9, 36, 84, 126, 126, 84, 36, 9, 1};
    const vector<int> n_equals_10{1, 10, 45, 120, 210, 252, 210, 120, 45, 10, 1};
    const vector<int> n_equals_11{1, 11, 55, 165, 330, 462, 462, 330, 165, 55, 11, 1};
    const vector<int> n_equals_12{1, 12, 66, 220, 495, 792, 924, 792, 495, 220, 66, 12, 1};

    const vector<vector<int>> n_choose_r{n_equals_0, n_equals_1, n_equals_2, n_equals_3, n_equals_4,
                                         n_equals_5, n_equals_6, n_equals_7, n_equals_8, n_equals_9,
                                         n_equals_10, n_equals_11, n_equals_12};

    typedef unsigned int uint;
    typedef Eigen::Vector3d AffineCoeff2D;
    typedef Eigen::Vector4d AffineCoeff3D;
    struct Point{
        double x;
        double y;
        double z;
    };
    struct PrimitiveTarget{
        double t0{-1.0};
        double tf;
        double ctrl_x[4];
        double ctrl_y[4];
        double ctrl_z[4];
    };
    struct PrimitivePlanning{
        double t0;
        double tf;
        double ctrl_x[4];
        double ctrl_y[4];
        double ctrl_z[4];
    };
    struct ControlInput{
        double px;
        double py;
        double pz;
        double vx;
        double vy;
        double vz;
        double ax;
        double ay;
        double az;
    };

    struct State{
        double px;
        double py;
        double pz;
        double vx;
        double vy;
        double vz;
    };
    struct StateHistory{
        vector<double> t;
        vector<double> px;
        vector<double> py;
        vector<double> pz;
        vector<double> vx;
        vector<double> vy;
        vector<double> vz;
    };
    double interpolate(const vector<double> &xData, const vector<double> &yData, const double &x);

}


#endif //DMVC_TRACKER_UTILS3D_H
