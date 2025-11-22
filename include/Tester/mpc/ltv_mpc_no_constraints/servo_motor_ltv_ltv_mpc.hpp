#ifndef __SERVO_MOTOR_LTV_LTV_MPC_HPP__
#define __SERVO_MOTOR_LTV_LTV_MPC_HPP__

#include "servo_motor_ltv_ltv_mpc_F.hpp"
#include "servo_motor_ltv_ltv_mpc_Phi.hpp"
#include "servo_motor_ltv_ltv_mpc_Weight_U_Nc.hpp"
#include "servo_motor_ltv_ltv_mpc_lkf.hpp"
#include "servo_motor_ltv_ltv_mpc_solver_factor.hpp"
#include "servo_motor_ltv_mpc_phi_f_updater.hpp"
#include "servo_motor_ltv_parameters.hpp"
#include "servo_motor_mpc_state_space_updater.hpp"

#include "python_mpc.hpp"

namespace servo_motor_ltv_ltv_mpc {

using namespace PythonNumpy;
using namespace PythonControl;
using namespace PythonMPC;

constexpr std::size_t NP = 32;
constexpr std::size_t NC = 2;

constexpr std::size_t INPUT_SIZE = servo_motor_ltv_ltv_mpc_lkf::INPUT_SIZE;
constexpr std::size_t STATE_SIZE = servo_motor_ltv_ltv_mpc_lkf::STATE_SIZE;
constexpr std::size_t OUTPUT_SIZE = servo_motor_ltv_ltv_mpc_lkf::OUTPUT_SIZE;

constexpr std::size_t AUGMENTED_STATE_SIZE = STATE_SIZE + OUTPUT_SIZE;

constexpr std::size_t NUMBER_OF_DELAY =
    servo_motor_ltv_ltv_mpc_lkf::NUMBER_OF_DELAY;

using LKF_Type = servo_motor_ltv_ltv_mpc_lkf::type;

using A_Type = typename LKF_Type::DiscreteStateSpace_Type::A_Type;

using B_Type = typename LKF_Type::DiscreteStateSpace_Type::B_Type;

using C_Type = typename LKF_Type::DiscreteStateSpace_Type::C_Type;

using F_Type = servo_motor_ltv_ltv_mpc_F::type;

using Phi_Type = servo_motor_ltv_ltv_mpc_Phi::type;

using SolverFactor_Type = servo_motor_ltv_ltv_mpc_solver_factor::type;

using PredictionMatrices_Type =
    MPC_PredictionMatrices_Type<F_Type, Phi_Type, NP, NC, INPUT_SIZE,
                                AUGMENTED_STATE_SIZE, OUTPUT_SIZE>;

using Reference_Type = DenseMatrix_Type<float, OUTPUT_SIZE, 1>;

using ReferenceTrajectory_Type =
    MPC_ReferenceTrajectory_Type<Reference_Type, NP>;

using Parameter_Type = servo_motor_ltv_parameters::Parameter;

using Weight_U_Nc_Type = servo_motor_ltv_ltv_mpc_Weight_U_Nc::type;

using EmbeddedIntegratorSateSpace_Type =
    typename EmbeddedIntegratorTypes<A_Type, B_Type, C_Type>::StateSpace_Type;

using type = LTV_MPC_NoConstraints_Type<LKF_Type, PredictionMatrices_Type,
                                        ReferenceTrajectory_Type,
                                        Parameter_Type, SolverFactor_Type>;

inline auto make() -> type {

  auto kalman_filter = servo_motor_ltv_ltv_mpc_lkf::make();

  auto F = servo_motor_ltv_ltv_mpc_F::make();

  auto Phi = servo_motor_ltv_ltv_mpc_Phi::make();

  auto solver_factor = servo_motor_ltv_ltv_mpc_solver_factor::make();

  PredictionMatrices_Type prediction_matrices(F, Phi);

  ReferenceTrajectory_Type reference_trajectory;

  Weight_U_Nc_Type Weight_U_Nc = servo_motor_ltv_ltv_mpc_Weight_U_Nc::make();

  MPC_StateSpace_Updater_Function_Object<
      Parameter_Type, typename LKF_Type::DiscreteStateSpace_Type>
      MPC_StateSpace_Updater_Function =
          servo_motor_mpc_state_space_updater::MPC_StateSpace_Updater::update<
              Parameter_Type, typename LKF_Type::DiscreteStateSpace_Type>;

  LTV_MPC_Phi_F_Updater_Function_Object<EmbeddedIntegratorSateSpace_Type,
                                        Parameter_Type, Phi_Type, F_Type>
      LTV_MPC_Phi_F_Updater_Function =
          servo_motor_ltv_mpc_phi_f_updater::LTV_MPC_Phi_F_Updater::update<
              EmbeddedIntegratorSateSpace_Type, Parameter_Type, Phi_Type,
              F_Type>;

  auto ltv_mpc_nc = make_LTV_MPC_NoConstraints(
      kalman_filter, prediction_matrices, reference_trajectory, solver_factor,
      Weight_U_Nc, MPC_StateSpace_Updater_Function,
      LTV_MPC_Phi_F_Updater_Function);

  return ltv_mpc_nc;
}

} // namespace servo_motor_ltv_ltv_mpc

#endif // __SERVO_MOTOR_LTV_LTV_MPC_HPP__
