//
// Created by larr-planning on 24. 3. 3.
//
#include <dmvc_utils3d/bernstein_util.h>

int dmvc3d::factorial(int num) {
    if (num<=1) return 1;
    return num*factorial(num-1);
}

int dmvc3d::nchoosek(int n, int r) {
    long long int number = 1;
    if (n - r > r) {
        for (int i = n; i > n - r; i--) {
            number = number * i;
        }
        return (int) (number / factorial(r));
    } else {
        for (int i = n; i > r; i--) {
            number = number * i;
        }
        return (int) (number / factorial(n - r));
    }}

int dmvc3d::nchooser(int n, int r) {
    if (n < 0 or r < 0)
        return 0;
    if (n == r)
        return 1;
    return dmvc3d::n_choose_r[n][r];
}

double dmvc3d::getBernsteinValue(double *bern_ctrl_pts, double t, double t0, double tf, int poly_order) {
    double value = 0.0;
    for (int i = 0; i < poly_order + 1; i++) {
        value += bern_ctrl_pts[i] * double(nchooser(poly_order, i)) * std::pow(t - t0, i) *
                 std::pow(tf - t, poly_order - i) / std::pow(tf - t0, poly_order);
    }
    return value;
}