#ifndef __SERVO_MOTOR_LTV_CONSTRAINTS_LTV_MPC_HPP__
#define __SERVO_MOTOR_LTV_CONSTRAINTS_LTV_MPC_HPP__

#include "servo_motor_ltv_constraints_ltv_mpc_lkf.hpp"
#include "servo_motor_ltv_constraints_ltv_mpc_F.hpp"
#include "servo_motor_ltv_constraints_ltv_mpc_Phi.hpp"
#include "servo_motor_ltv_constraints_ltv_mpc_solver_factor.hpp"
#include "servo_motor_ltv_constraints_ltv_mpc_Weight_U_Nc.hpp"
#include "servo_motor_ltv_constraints_parameters.hpp"
#include "servo_motor_ltv_constraints_mpc_state_space_updater.hpp"
#include "servo_motor_ltv_constraints_ltv_mpc_phi_f_updater.hpp"

#include "servo_motor_ltv_constraints_delta_U_min.hpp"
#include "servo_motor_ltv_constraints_delta_U_max.hpp"
#include "servo_motor_ltv_constraints_U_min.hpp"
#include "servo_motor_ltv_constraints_U_max.hpp"
#include "servo_motor_ltv_constraints_Y_min.hpp"
#include "servo_motor_ltv_constraints_Y_max.hpp"

#include "python_mpc.hpp"

namespace servo_motor_ltv_constraints_ltv_mpc {

using namespace PythonNumpy;
using namespace PythonControl;
using namespace PythonMPC;

constexpr std::size_t NP = 32;
constexpr std::size_t NC = 2;

constexpr std::size_t INPUT_SIZE = servo_motor_ltv_constraints_ltv_mpc_lkf::INPUT_SIZE;
constexpr std::size_t STATE_SIZE = servo_motor_ltv_constraints_ltv_mpc_lkf::STATE_SIZE;
constexpr std::size_t OUTPUT_SIZE = servo_motor_ltv_constraints_ltv_mpc_lkf::OUTPUT_SIZE;

constexpr std::size_t AUGMENTED_STATE_SIZE = STATE_SIZE + OUTPUT_SIZE;

constexpr std::size_t NUMBER_OF_DELAY = servo_motor_ltv_constraints_ltv_mpc_lkf::NUMBER_OF_DELAY;

using LKF_Type = servo_motor_ltv_constraints_ltv_mpc_lkf::type;

using A_Type = typename LKF_Type::DiscreteStateSpace_Type::A_Type;

using B_Type = typename LKF_Type::DiscreteStateSpace_Type::B_Type;

using C_Type = typename LKF_Type::DiscreteStateSpace_Type::C_Type;

using F_Type = servo_motor_ltv_constraints_ltv_mpc_F::type;

using Phi_Type = servo_motor_ltv_constraints_ltv_mpc_Phi::type;

using SolverFactor_Type = servo_motor_ltv_constraints_ltv_mpc_solver_factor::type;

using Delta_U_Min_Type = servo_motor_ltv_constraints_delta_U_min::type;

using Delta_U_Max_Type = servo_motor_ltv_constraints_delta_U_max::type;

using U_Min_Type = servo_motor_ltv_constraints_U_min::type;

using U_Max_Type = servo_motor_ltv_constraints_U_max::type;

using Y_Min_Type = servo_motor_ltv_constraints_Y_min::type;

using Y_Max_Type = servo_motor_ltv_constraints_Y_max::type;

using PredictionMatrices_Type = MPC_PredictionMatrices_Type<
  F_Type, Phi_Type, NP, NC, INPUT_SIZE, AUGMENTED_STATE_SIZE, OUTPUT_SIZE>;

using Ref_Type = DenseMatrix_Type<double, OUTPUT_SIZE, 1>;

using ReferenceTrajectory_Type = MPC_ReferenceTrajectory_Type<
  Ref_Type, NP>;

using Parameter_Type = servo_motor_ltv_constraints_parameters::Parameter;

using Weight_U_Nc_Type = servo_motor_ltv_constraints_ltv_mpc_Weight_U_Nc::type;

using EmbeddedIntegratorSateSpace_Type =
  typename EmbeddedIntegratorTypes<A_Type, B_Type, C_Type>::StateSpace_Type;

using type = LTV_MPC_Type<
  LKF_Type, PredictionMatrices_Type, ReferenceTrajectory_Type,
  Parameter_Type,  Delta_U_Min_Type, Delta_U_Max_Type,
  U_Min_Type, U_Max_Type, Y_Min_Type, Y_Max_Type,
  SolverFactor_Type>;

inline auto make() -> type {

  auto kalman_filter = servo_motor_ltv_constraints_ltv_mpc_lkf::make();

  auto F = servo_motor_ltv_constraints_ltv_mpc_F::make();

  auto Phi = servo_motor_ltv_constraints_ltv_mpc_Phi::make();

  auto solver_factor = servo_motor_ltv_constraints_ltv_mpc_solver_factor::make();

  auto Weight_U_Nc = servo_motor_ltv_constraints_ltv_mpc_Weight_U_Nc::make();

  auto delta_U_min = servo_motor_ltv_constraints_delta_U_min::make();

  delta_U_min.template set<0, 0>(static_cast<double>(-100.0));

  auto delta_U_max = servo_motor_ltv_constraints_delta_U_max::make();

  delta_U_max.template set<0, 0>(static_cast<double>(100.0));

  auto U_min = servo_motor_ltv_constraints_U_min::make();

  U_min.template set<0, 0>(static_cast<double>(-180.0));

  auto U_max = servo_motor_ltv_constraints_U_max::make();

  U_max.template set<0, 0>(static_cast<double>(180.0));

  auto Y_min = servo_motor_ltv_constraints_Y_min::make();

  auto Y_max = servo_motor_ltv_constraints_Y_max::make();

  PredictionMatrices_Type prediction_matrices(F, Phi);

  ReferenceTrajectory_Type reference_trajectory;

  MPC_StateSpace_Updater_Function_Object<
    Parameter_Type, typename LKF_Type::DiscreteStateSpace_Type>
    MPC_StateSpace_Updater_Function =
    servo_motor_ltv_constraints_mpc_state_space_updater::MPC_StateSpace_Updater::update<
      Parameter_Type, typename LKF_Type::DiscreteStateSpace_Type>;

  LTV_MPC_Phi_F_Updater_Function_Object<
    EmbeddedIntegratorSateSpace_Type, Parameter_Type, Phi_Type, F_Type>
    LTV_MPC_Phi_F_Updater_Function =
    servo_motor_ltv_constraints_ltv_mpc_phi_f_updater::LTV_MPC_Phi_F_Updater::update<
      EmbeddedIntegratorSateSpace_Type, Parameter_Type, Phi_Type, F_Type>;

  auto ltv_mpc = make_LTV_MPC(
    kalman_filter, prediction_matrices, reference_trajectory, Weight_U_Nc,
    MPC_StateSpace_Updater_Function,
    LTV_MPC_Phi_F_Updater_Function,
    delta_U_min, delta_U_max, U_min, U_max, Y_min, Y_max, solver_factor);

  return ltv_mpc;

}

} // namespace servo_motor_ltv_constraints_ltv_mpc

#endif // __SERVO_MOTOR_LTV_CONSTRAINTS_LTV_MPC_HPP__
