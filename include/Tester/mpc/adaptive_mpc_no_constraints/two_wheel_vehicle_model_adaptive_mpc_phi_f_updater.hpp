#ifndef __TWO_WHEEL_VEHICLE_MODEL_ADAPTIVE_MPC_PHI_F_UPDATER_HPP__
#define __TWO_WHEEL_VEHICLE_MODEL_ADAPTIVE_MPC_PHI_F_UPDATER_HPP__

#include "two_wheel_vehicle_model_mpc_embedded_integrator_state_space_updater.hpp"
#include "two_wheel_vehicle_model_prediction_matrices_phi_f_updater.hpp"

namespace two_wheel_vehicle_model_adaptive_mpc_phi_f_updater {

using namespace two_wheel_vehicle_model_mpc_embedded_integrator_state_space_updater;
using namespace two_wheel_vehicle_model_prediction_matrices_phi_f_updater;

class Adaptive_MPC_Phi_F_Updater {
public:
  template <typename X_Type, typename U_Type, typename Parameter_Type,
            typename Phi_Type, typename F_Type, typename StateSpace_Type>
  static inline void update(const X_Type &X, const U_Type &U,
                            const Parameter_Type &parameter, Phi_Type &Phi,
                            F_Type &F) {

    StateSpace_Type state_space;
    EmbeddedIntegrator_Updater::update(X, U, parameter, state_space);

    PredictionMatricesPhiF_Updater::update(state_space.A, state_space.B,
                                           state_space.C, Phi, F);
  }
};

} // namespace two_wheel_vehicle_model_adaptive_mpc_phi_f_updater

#endif // __TWO_WHEEL_VEHICLE_MODEL_ADAPTIVE_MPC_PHI_F_UPDATER_HPP__
