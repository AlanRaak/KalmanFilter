#include "kalman_filter.hpp"
#include "../model/model.hpp"

#include <iostream>

enum class gear
{
    forward,
    reverse
};

struct state
{
    struct
    {
        double x;
        double y;
    } position{};
    double orientation;
    double steering_angle;
    double speed;
    enum gear gear;
};

// const auto move_direction{[] (const enum gear gear) -> double { return (gear == gear::forward) * 2.0 - 1.0; }};

/*
KalmanFilter::KalmanFilter(double x_pos, double y_pos, double orientation, double steering_angle, double speed, double gear){
    x = x_pos,
        y_pos,
        orientation,
        steering_angle,
        speed,
        gear;
}; // Initializes matrices with default/starting  values
*/
KalmanFilter::KalmanFilter(double x_pos, double y_pos, double orientation, double steering_angle, double speed)
{
     // state matrix
    x.array = {{x_pos},
            {y_pos},
            {orientation},
            {steering_angle},
            {speed}};

    // input/command matrix
    u.array = {{0.0},
               {0.0},
               {0.0},
               {0.0},
               {0.0}};

    // uncertainty matrix - (trust more previous state with smaller numbers [trusts predicted/simulated values more])
    P.array = {{100.0,   0.0,    0.0,    0.0,    0.0},
               {0.0, 100.0,    0.0,    0.0,    0.0},
               {0.0,   0.0,  100.0,    0.0,    0.0},
               {0.0,   0.0,    0.0,  100.0,    0.0},
               {0.0,   0.0,    0.0,    0.0,  100.0}};

    // matrix to add to P - won't allow craintiy to go too small
    P_add.array = {{0.1,    0.0,    0.0,    0.0,   0.0},
            {0.0,    0.1,    0.0,    0.0,    0.0},
            {0.0,    0.0,    0.1,    0.0,    0.0},
            {0.0,    0.0,    0.0,    0.1,    0.0},
            {0.0,    0.0,    0.0,    0.0,    0.1}};

    // state transrom matrix
    F.array = {{1.0,   0.0,    0.0,    0.0,    0.0},
        {0.0,   1.0,    0.0,    0.0,    0.0},
        {0.0,   0.0,    1.0,    0.0,    0.0},
        {0.0,   0.0,    0.0,    1.0,    0.0},
        {0.0,   0.0,    0.0,    0.0,    1.0}};

    // measurment matrix (m x n -> m number of parameters measured; n number of total parameters used)
    H.array = {{1.0,   0.0,    0.0,    0.0,    0.0},
        {0.0,   1.0,    0.0,    0.0,    0.0},
        {0.0,   0.0,    1.0,    0.0,    0.0},
        {0.0,   0.0,    0.0,    1.0,    0.0},
        {0.0,   0.0,    0.0,    0.0,    1.0}}; // mesurment matrix

    // measurement uncertainty - (trust measurements more with smaller numbers)
    R.array = {{1.0,   0.0,    0.0,    0.0,    0.0,},
        {0.0,   1.0,    0.0,    0.0,    0.0,},
        {0.0,   0.0,    1.0,    0.0,    0.0,},
        {0.0,   0.0,    0.0,    1.0,    0.0,},
        {0.0,   0.0,    0.0,    0.0,    1.0}};

    // eye() matrix
    I.array = {{1.0,   0.0,    0.0,    0.0,    0.0},
        {0.0,   1.0,    0.0,    0.0,    0.0},
        {0.0,   0.0,    1.0,    0.0,    0.0},
        {0.0,   0.0,    0.0,    1.0,    0.0},
        {0.0,   0.0,    0.0,    0.0,    1.0}};
}

void KalmanFilter::Predict(double dt)
{ // use dt for next_state calculation
    P = P + P_add;

    //F(); // TODO convert nextState.cpp to F matrix
    // x_new = F * x_old  ->  F = x_new * inv(x_old); F includes u - migth be bad
    // OR use trivial F matrix for P calculation and next_state for x - positon prediction; i think this is better option
    F.array[0][4] = cos(x.array[2][0]) * dt;
    F.array[1][4] = sin(x.array[2][0]) * dt;
    F.array[2][4] = tan(x.array[3][0]) / 1 * dt; // 1 m wheelbase, orientation change
    x = (F * x);// + u; // figure out F/transform matrix, needed for kalman filter uncertainty calculation

    P = F * P * math::trans(F);

    // F(0, 4) = cos(x(2)) * dt;
    // F(1, 4) = sin(x(2)) * dt;
    // F(2, 4) = tan(x(3)) / 1 * dt; // 1 m wheelbase, orientation change
    // x = (F * x);// + u; // figure out F/transform matrix, needed for kalman filter uncertainty calculation
    // std::cout << "predictes: " << x(0) << std::endl;
    // P = F * P * dlib::trans(F);
}

void KalmanFilter::UpdateMeasurements(math::matrix &measurements)
{

    math::matrix y{measurements - (H * x)};
    math::matrix S{H * P * math::trans(H) + R};
    math::matrix K{P * math::trans(H) * math::inv(S)};

    //y = dlib::trans(measurements) - (H * x); // if measurements.shape == (5 , 1), then transpose might be unnecessary
    // y = measurements - (H * x); // (5 ,1)
    // S = H * P * math::trans(H) + R; // (5, 5)
    // K = P * math::trans(H) * math::inv(S); // (6, 5) // this here dumped core, because inv(S) wasnt definde, inverse of S didn't exist; pinv(S) return pseudo inverse

    x = x + K * y;
    P = (I - K * H) * P;
}

ExtendedKalmanFilter::ExtendedKalmanFilter(double x_pos, double y_pos, double orientation, double steering_angle, double speed) : KalmanFilter(x_pos, y_pos, orientation, steering_angle, speed)
{
};

void ExtendedKalmanFilter::Predict(double dt)
{
    P = P + P_add;

    math::matrix F{x.rows, x.rows}; // jacobian
    // constexpr double dt{0.1}; // used for jacobian's derivatives; i think it should be equal to time between 2 consecutive measurements
    math::matrix initial_state_prediction{model::next_state(x, dt)};

    for (int row_num{0}; row_num < F.array.size(); row_num++)
    {
        math::matrix x_delta{x};
        x_delta.array[row_num][0] = x_delta.array[row_num][0] + jacobian_deltas[row_num];
        math::matrix new_state_delta = model::next_state(x_delta, dt);

        for (int column_num{0}; column_num < F.array[0].size(); column_num++)
        {
            F.array[row_num][column_num] = (new_state_delta.array[column_num][0] - initial_state_prediction.array[column_num][0]) / (2 * jacobian_deltas[row_num]); // based on some YT video; only god knows if this is actually correct
        }
    }

    // x = (F * x) + u; // use some function with u
    x = initial_state_prediction;

    P = F * P * math::trans(F);
}
