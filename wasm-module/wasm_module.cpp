// wasm-module.cpp : Defines the entry point for the application.
//

#include "wasm_module.h"
#include <iostream>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <emscripten.h>
#include <math.h>

// Speed of sound in meters per second
static double c = 0.0;

// Time of arrivals for a specific sound
//double toa1 = 52.805 / c;
//double toa2 = 55.000 / c;
//double toa3 = 58.661 / c;

// Cost function to compute the TDOA residuals
struct TDOACostFunctor {
    TDOACostFunctor(double toa1, double toa2, double toa3, Eigen::Vector2d mic1, Eigen::Vector2d mic2, Eigen::Vector2d mic3)
        : toa1_(toa1), toa2_(toa2), toa3_(toa3), mic1_(mic1), mic2_(mic2), mic3_(mic3) {}
    template <typename T>
    bool operator()(const T* const x, T* residuals) const {
        Eigen::Matrix<T, 2, 1> pos(x[0], x[1]);

        T d1 = (pos - mic1_.cast<T>()).norm();
        T d2 = (pos - mic2_.cast<T>()).norm();
        T d3 = (pos - mic3_.cast<T>()).norm();

        T tdoa1 = d1 / T(c);
        T tdoa2 = d2 / T(c);
        T tdoa3 = d3 / T(c);

        residuals[0] = tdoa1 - T(toa1_);
        residuals[1] = tdoa2 - T(toa2_);
        residuals[2] = tdoa3 - T(toa3_);

        return true;
    }

private:
    const double toa1_, toa2_, toa3_;
    const Eigen::Vector2d mic1_, mic2_, mic3_;
};

void getLoudestSampleTimes(int64_t* results[3], int64_t* tracks_start_times[3], int16_t* tracks[], int64_t tracks_size) {
    // audio has 3 arrays of 16-bit samples of size audio_size:
    int64_t loudest_samples[3] = { 0, 0, 0 };
    for (int i = 0; i < 3; i++) {
        for (int64_t j = 0; j < tracks_size; j++) {
            if (tracks[i][j] >= tracks[i][loudest_samples[i]]) {
                loudest_samples[i] = j;
            }
        }
        *results[i] = (int64_t)(((double)loudest_samples[i] / (double)44100) * 1000000);
    }
}

extern "C" {
    EMSCRIPTEN_KEEPALIVE
        void getLocation(int16_t temperature, int64_t toas[3], double mloc[3][2], double* result_x, double* result_y) {

        c = 331 + (0.61 * temperature);

        static Eigen::Vector2d mic1((double)mloc[0][0], (double)mloc[0][1]);
        static Eigen::Vector2d mic2((double)mloc[1][0], (double)mloc[1][1]);
        static Eigen::Vector2d mic3((double)mloc[2][0], (double)mloc[2][1]);

        // Initial guess for the sound source location
        double x[2] = { 1.0, 1.0 };

        // Build the problem
        ceres::Problem problem;
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<TDOACostFunctor, 3, 2>(
                new TDOACostFunctor(
                    (double)(toas[0] / 1000000.0),
                    (double)(toas[1] / 1000000.0),
                    (double)(toas[2] / 1000000.0),
                    mic1, mic2, mic3)),
            nullptr, x);

        // Configure solver options
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false; // true for debugging

        // Solve the problem
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        // Output the results
        // std::cout << summary.FullReport() << std::endl;
        /*EM_ASM({
            console.log('Location:', $0, $1);
        }, x[0], x[1]);*/

        *result_x = x[0];
        *result_y = x[1];

    }

    EMSCRIPTEN_KEEPALIVE
        void getLocationFromAudio(int64_t* tracks_start_times[3], int16_t* tracks[], int64_t tracks_size,
            int16_t temperature, double mloc[3][2], double* result_x, double* result_y) {
        int64_t sample_times_values[3] = { 0 };
        int64_t* sample_times[3] = { &sample_times_values[0], &sample_times_values[1], &sample_times_values[2] };
        getLoudestSampleTimes(sample_times, tracks_start_times, tracks, tracks_size);

        int64_t* actualTime1 = new int64_t;
        int64_t* actualTime2 = new int64_t;
        int64_t* actualTime3 = new int64_t;

        *actualTime1 = *sample_times[0];
        *actualTime2 = *sample_times[1] - (*tracks_start_times[1] - *tracks_start_times[0]);
        *actualTime3 = *sample_times[2] - (*tracks_start_times[2] - *tracks_start_times[0]);

        int64_t toas[3] = { *actualTime1, *actualTime2, *actualTime3 };

        int64_t largest_toa = 0;
        for (int i = 0; i < 3; i++) {
            if (toas[i] > largest_toa) {
                largest_toa = toas[i];
            }
        }
        for (int i = 0; i < 3; i++) {
            toas[i] = largest_toa - toas[i];
        }

        getLocation(temperature, toas, mloc, result_x, result_y);

        delete actualTime1;
        delete actualTime2;
        delete actualTime3;
        return;
    }

    EMSCRIPTEN_KEEPALIVE
        void getSyncTime(int64_t* tracks_start_times[3], int16_t* tracks[], int64_t tracks_size, int64_t* actualTime1, int64_t* actualTime2, int64_t* actualTime3) {
        int64_t sample_times_values[3] = { 0 };
        int64_t* sample_times[3] = { &sample_times_values[0], &sample_times_values[1], &sample_times_values[2] };
        getLoudestSampleTimes(sample_times, tracks_start_times, tracks, tracks_size);

        *actualTime1 = *sample_times[0];
        *actualTime2 = *sample_times[1] - (*tracks_start_times[1] - *tracks_start_times[0]);
        *actualTime3 = *sample_times[2] - (*tracks_start_times[2] - *tracks_start_times[0]);
    }
}
