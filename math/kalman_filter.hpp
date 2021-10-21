#pragma once

#include "matrix.hpp"

class KalmanFilter
{
public:
    // dlib::matrix<double, 6, 1> x;
    // dlib::matrix<double, 6, 1> u;
    // dlib::matrix<double, 6, 6> P;     // trust prev_state(next_state?) more with lower values
    // dlib::matrix<double, 6, 6> P_add; // matrix added to P to prevent certainty going too low
    // dlib::matrix<double, 6, 6> F;
    // dlib::matrix<double, 5, 6> H;
    // dlib::matrix<double, 5, 5> R; // trust measuremnts more with lower number
    // dlib::matrix<double, 6, 6> I;
    KalmanFilter(double x_pos, double y_pos, double orientation, double steering_angle, double speed); // Initializes matrices with default/starting  values

    // http://www.swarthmore.edu/NatSci/echeeve1/Ref/Kalman/MatrixKalman.html - matrix dimensions
    math::matrix x {5, 1}; // x_pos, y_pos, orientation, steering_angle, speed
    math::matrix u {5, 1};
    math::matrix P {5, 5};
    math::matrix P_add {5, 5};
    math::matrix F {5, 5};
    math::matrix H {5, 5};
    math::matrix R {5, 5};
    math::matrix I {5, 5};

    void Predict(double dt);
    void UpdateMeasurements(math::matrix& measurements); // update state x - which can be accesed after calling update on KalmanFilter (KalmanFilter.x)
                                                                       //dlib::matrix<float, 6, 1> update(float x_pos, float y_pos, float orientation, float steering_angle, float speed, float gear);
};

class ExtendedKalmanFilter : public KalmanFilter
{
public:
    ExtendedKalmanFilter(double x_pos, double y_pos, double orientation, double steering_angle, double speed);
    const std::vector<double> jacobian_deltas{0.1, 0.1, 0.1, 0.1, 0.1}; // x_pos, y_pos, orientation, steering_angle, speed
    void Predict(double dt);
};
