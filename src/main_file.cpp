#include "../math/kalman_filter.hpp"
#include "../util/file.hpp"
#include "../model/model.hpp"

#include <iostream>
#include <fstream>
#include <string.h>

int main()
{
    // math::matrix mat{{{1, 2, 4}, {1, 3, 3}, {1, 2, 3}}};
    // mat.print();
    // (math::inv(mat)*mat).print();

    math::matrix mat_t{{{1, 2, 3}, {1, 2, 3}, {1, 2, 3}}};
    // mat_t.print();
    // math::trans(mat_t).print();

    // (mat_t - math::matrix{{{1,0,0},{0,1,0},{0,0,1}}}).print();

    // kf.x.print();


    std::ofstream true_state_file;
    std::ofstream noisy_state_file;
    std::ofstream filtered_state_file;
    true_state_file.open("../data/true_state_file.csv");
    noisy_state_file.open("../data/noisy_state_file.csv");
    filtered_state_file.open("../data/filtered_state_file.csv");
    true_state_file << "x_pos, y_pos, orientation, steering_angle, speed\n";
    noisy_state_file << "x_pos, y_pos, orientation, steering_angle, speed\n";
    filtered_state_file << "x_pos, y_pos, orientation, steering_angle, speed\n";

    ExtendedKalmanFilter ekf{10.0, 10.0, 0.0, 0.0, 1.0};
    math::matrix measurements{{{10.0}, {10.0}, {0.0}, {0.0}, {1.0}}};

    math::matrix true_next_state{{{10.0}, {10.0}, {0.0}, {0.0}, {1.0}}};
    for (int iteration{}; iteration < 1000; iteration++)
    {
        // ekf.Predict(0.1);
        // math::matrix true_next_state{};
        // measurements.array[0][0] = 10.0 + iteration*0.2001;
        // ekf.UpdateMeasurements(measurements);

        ekf.Predict(0.1);
        true_next_state = model::next_state(true_next_state, 0.1); // ground truth (this also has noise actually; i think even more than measeurement on our ATV)
        math::matrix noisy_next_state{model::noisify_state(true_next_state)}; // aka noisy measurement

        ekf.UpdateMeasurements(noisy_next_state);
        ekf.x; // predicted new state

        util::state_to_file(true_state_file, true_next_state);
        util::state_to_file(noisy_state_file, noisy_next_state);
        util::state_to_file(filtered_state_file, ekf.x);
    }

    true_state_file.close();
    noisy_state_file.close();
    filtered_state_file.close();
}