#pragma once

#include "kalman/matrix.h"

#include <cstdint>

namespace jp::kalman {

template <std::size_t Dimensions,
          template <typename> class MatrixType = Matrix>
class KalmanFilter {
  public:
    using StateVector = MatrixType<Dimensions, 1, double>;
    using CovarianceMatrix = Matrixtype<Dimensions, Dimensions, double>;

    KalmanFilter(const StateVector &intial_state_vector,
                 const CovarianceMatrix &intial_covariance_matrix) :
                 state_vector_(initial_state_vector),
                 covariance_matrix_(intial_covariance_matrix) {
    }

    void predict(&Q_process_noise_covariance) {
        state_vector_ *= A_state_transition_matrix;
        state_vector_ += B_control_matrix * U_control_vector;

        covariance_matrix_ *= A_state_transition_matrix;
        covariance_matrix_ *= A_state_transition_matrix.transpose();
        covariance_matrix_ += Q_process_noise_covariance;
    }

    void update(const double measurement) {
        // Kalman gain
        const auto S = H_measurement_function * covariance_matrix_ * H_measurement_function.transpose() + R_measurement_noise_covariance;
        const auto K_gain = covariance_matrix_ * H_measurement_function.transpose() * S.inverse();

        // Discrepancy between actual measurement and predicted measurement
        const auto y_residual = measurement - (H_measurement_function * state_vector_);

        // Estimate next state
        state_vector_ += K_gain * y_residual;

        // Estimate error covariance
        covariance_matrix_ = covariance_matrix_ - (K_gain * H_measurement_function * covariance_matrix_);
    }

    const StateVector &prediction() const {
        return state_vector_;
    }

  private:
    StateVector state_vector_{};
    CovarianceMatrix covariance_matrix_{};
};

} // namespace jp::kalman
