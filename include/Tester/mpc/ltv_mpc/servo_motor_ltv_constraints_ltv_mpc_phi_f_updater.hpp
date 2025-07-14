#ifndef __SERVO_MOTOR_LTV_CONSTRAINTS_LTV_MPC_PHI_F_UPDATER_HPP__
#define __SERVO_MOTOR_LTV_CONSTRAINTS_LTV_MPC_PHI_F_UPDATER_HPP__

#include "servo_motor_ltv_constraints_mpc_embedded_integrator_state_space_updater.hpp"
#include "servo_motor_ltv_constraints_prediction_matrices_phi_f_updater.hpp"

namespace servo_motor_ltv_constraints_ltv_mpc_phi_f_updater {

using namespace servo_motor_ltv_constraints_mpc_embedded_integrator_state_space_updater;
using namespace servo_motor_ltv_constraints_prediction_matrices_phi_f_updater;

class LTV_MPC_Phi_F_Updater {
public:
  template <typename StateSpace_Type, typename Parameter_Type,
            typename Phi_Type, typename F_Type>
  static inline void update(const Parameter_Type &parameter, Phi_Type &Phi,
                            F_Type &F) {

    StateSpace_Type state_space;
    EmbeddedIntegrator_Updater::update(parameter, state_space);

    PredictionMatricesPhiF_Updater::update(state_space.A, state_space.B,
                                           state_space.C, Phi, F);
  }
};

} // namespace servo_motor_ltv_constraints_ltv_mpc_phi_f_updater

#endif // __SERVO_MOTOR_LTV_CONSTRAINTS_LTV_MPC_PHI_F_UPDATER_HPP__
