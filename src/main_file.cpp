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

    ExtendedKalmanFilter ekf_gps{10.0, 10.0, 0.0, 0.2, 2.0};
    ExtendedKalmanFilter ekf_odo{10.0, 10.0, 0.0, 0.2, 2.0};

    math::matrix gps_measurement{{{10.0}, {10.0}, {0.0}, {0.2}, {2.0}}};

    math::matrix true_next_state{{{10.0}, {10.0}, {0.0}, {0.2}, {1.0}}};

    double system_time{}; // in milliseconds
    double gps_time{};
    double odo_time{};
    for (int i{}; i < 200; i++)
    {
        system_time++;

        if (system_time - gps_time > 100) // make random interval instead of 100
        {
            ekf_gps.Predict(0.1); // use time diff
            true_next_state = model::next_state(true_next_state, 0.1); // ground truth (this also has noise actually; i think even more than measeurement on our ATV);
                                                                    // TODO also test with adding noise here
            gps_measurement{model::noisify_state(true_next_state)}; // aka noisy measurement
            // true_next_state = model::noisify_state(model::next_state(true_next_state, 0.1)); // vehicle mechanical noise; i think it makes sense adding this here

            // Sensors measurement with noise
            ekf_gps.UpdateMeasurements(gps_measurement);
            ekf_gps.x; // predicted new state

            gps_time = system_time;
        }

        util::state_to_file(true_state_file, true_next_state);
        util::state_to_file(noisy_state_file, gps_measurement);
        util::state_to_file(filtered_state_file, ekf_gps.x);
        if(i == 100) {true_next_state.array[3][0] *= -1;}
    }

    true_state_file.close();
    noisy_state_file.close();
    filtered_state_file.close();
}