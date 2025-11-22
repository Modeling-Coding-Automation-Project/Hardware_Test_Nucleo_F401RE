#ifndef __TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_ADA_MPC_HPP__
#define __TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_ADA_MPC_HPP__

#include "two_wheel_vehicle_model_constraints_ada_mpc_B.hpp"
#include "two_wheel_vehicle_model_constraints_ada_mpc_F.hpp"
#include "two_wheel_vehicle_model_constraints_ada_mpc_Phi.hpp"
#include "two_wheel_vehicle_model_constraints_ada_mpc_Weight_U_Nc.hpp"
#include "two_wheel_vehicle_model_constraints_ada_mpc_ekf.hpp"
#include "two_wheel_vehicle_model_constraints_ada_mpc_ekf_parameter.hpp"
#include "two_wheel_vehicle_model_constraints_ada_mpc_solver_factor.hpp"
#include "two_wheel_vehicle_model_constraints_adaptive_mpc_phi_f_updater.hpp"

#include "two_wheel_vehicle_model_constraints_U_max.hpp"
#include "two_wheel_vehicle_model_constraints_U_min.hpp"
#include "two_wheel_vehicle_model_constraints_Y_max.hpp"
#include "two_wheel_vehicle_model_constraints_Y_min.hpp"
#include "two_wheel_vehicle_model_constraints_delta_U_max.hpp"
#include "two_wheel_vehicle_model_constraints_delta_U_min.hpp"

#include "python_mpc.hpp"

namespace two_wheel_vehicle_model_constraints_ada_mpc {

using namespace PythonNumpy;
using namespace PythonControl;
using namespace PythonMPC;

constexpr std::size_t NP = 16;
constexpr std::size_t NC = 1;

constexpr std::size_t INPUT_SIZE =
    two_wheel_vehicle_model_constraints_ada_mpc_ekf::INPUT_SIZE;
constexpr std::size_t STATE_SIZE =
    two_wheel_vehicle_model_constraints_ada_mpc_ekf::STATE_SIZE;
constexpr std::size_t OUTPUT_SIZE =
    two_wheel_vehicle_model_constraints_ada_mpc_ekf::OUTPUT_SIZE;

constexpr std::size_t AUGMENTED_STATE_SIZE = STATE_SIZE + OUTPUT_SIZE;

constexpr std::size_t NUMBER_OF_DELAY =
    two_wheel_vehicle_model_constraints_ada_mpc_ekf::NUMBER_OF_DELAY;

using EKF_Type = two_wheel_vehicle_model_constraints_ada_mpc_ekf::type;

using A_Type = typename EKF_Type::A_Type;

using B_Type = two_wheel_vehicle_model_constraints_ada_mpc_B::type;

using C_Type = typename EKF_Type::C_Type;

using X_Type = StateSpaceState_Type<float, STATE_SIZE>;

using U_Type = StateSpaceOutput_Type<float, INPUT_SIZE>;

using F_Type = two_wheel_vehicle_model_constraints_ada_mpc_F::type;

using Phi_Type = two_wheel_vehicle_model_constraints_ada_mpc_Phi::type;

using SolverFactor_Type =
    two_wheel_vehicle_model_constraints_ada_mpc_solver_factor::type;

using Delta_U_Min_Type = two_wheel_vehicle_model_constraints_delta_U_min::type;

using Delta_U_Max_Type = two_wheel_vehicle_model_constraints_delta_U_max::type;

using U_Min_Type = two_wheel_vehicle_model_constraints_U_min::type;

using U_Max_Type = two_wheel_vehicle_model_constraints_U_max::type;

using Y_Min_Type = two_wheel_vehicle_model_constraints_Y_min::type;

using Y_Max_Type = two_wheel_vehicle_model_constraints_Y_max::type;

using PredictionMatrices_Type =
    MPC_PredictionMatrices_Type<F_Type, Phi_Type, NP, NC, INPUT_SIZE,
                                AUGMENTED_STATE_SIZE, OUTPUT_SIZE>;

using Reference_Type = DenseMatrix_Type<float, OUTPUT_SIZE, 1>;

using ReferenceTrajectory_Type =
    MPC_ReferenceTrajectory_Type<Reference_Type, NP>;

using Parameter_Type =
    two_wheel_vehicle_model_constraints_ada_mpc_ekf_parameter::Parameter_Type;

using Weight_U_Nc_Type =
    two_wheel_vehicle_model_constraints_ada_mpc_Weight_U_Nc::type;

using EmbeddedIntegratorStateSpace_Type =
    typename EmbeddedIntegratorTypes<A_Type, B_Type, C_Type>::StateSpace_Type;

using type =
    AdaptiveMPC_Type<B_Type, EKF_Type, PredictionMatrices_Type,
                     ReferenceTrajectory_Type, Parameter_Type, Delta_U_Min_Type,
                     Delta_U_Max_Type, U_Min_Type, U_Max_Type, Y_Min_Type,
                     Y_Max_Type, SolverFactor_Type>;

inline auto make() -> type {

  auto kalman_filter = two_wheel_vehicle_model_constraints_ada_mpc_ekf::make();

  kalman_filter.X_hat.template set<0, 0>(static_cast<float>(0.0));
  kalman_filter.X_hat.template set<1, 0>(static_cast<float>(0.0));
  kalman_filter.X_hat.template set<2, 0>(static_cast<float>(0.0));
  kalman_filter.X_hat.template set<3, 0>(static_cast<float>(0.0));
  kalman_filter.X_hat.template set<4, 0>(static_cast<float>(0.0));
  kalman_filter.X_hat.template set<5, 0>(static_cast<float>(10.0));

  auto F = two_wheel_vehicle_model_constraints_ada_mpc_F::make();

  auto Phi = two_wheel_vehicle_model_constraints_ada_mpc_Phi::make();

  auto solver_factor =
      two_wheel_vehicle_model_constraints_ada_mpc_solver_factor::make();

  auto delta_U_min = two_wheel_vehicle_model_constraints_delta_U_min::make();

  auto delta_U_max = two_wheel_vehicle_model_constraints_delta_U_max::make();

  auto U_min = two_wheel_vehicle_model_constraints_U_min::make();

  U_min.template set<0, 0>(static_cast<float>(-1.0));
  U_min.template set<1, 0>(static_cast<float>(-10.0));

  auto U_max = two_wheel_vehicle_model_constraints_U_max::make();

  U_max.template set<0, 0>(static_cast<float>(1.0));
  U_max.template set<1, 0>(static_cast<float>(10.0));

  auto Y_min = two_wheel_vehicle_model_constraints_Y_min::make();

  auto Y_max = two_wheel_vehicle_model_constraints_Y_max::make();

  PredictionMatrices_Type prediction_matrices(F, Phi);

  ReferenceTrajectory_Type reference_trajectory;

  Weight_U_Nc_Type Weight_U_Nc =
      two_wheel_vehicle_model_constraints_ada_mpc_Weight_U_Nc::make();

  Adaptive_MPC_Phi_F_Updater_Function_Object<X_Type, U_Type, Parameter_Type,
                                             Phi_Type, F_Type,
                                             EmbeddedIntegratorStateSpace_Type>
      Adaptive_MPC_Phi_F_Updater_Function =
          two_wheel_vehicle_model_constraints_adaptive_mpc_phi_f_updater::
              Adaptive_MPC_Phi_F_Updater::update<
                  X_Type, U_Type, Parameter_Type, Phi_Type, F_Type,
                  EmbeddedIntegratorStateSpace_Type>;

  auto adaptive_mpc = make_AdaptiveMPC<
      B_Type, EKF_Type, PredictionMatrices_Type, ReferenceTrajectory_Type,
      Parameter_Type, Delta_U_Min_Type, Delta_U_Max_Type, U_Min_Type,
      U_Max_Type, Y_Min_Type, Y_Max_Type, SolverFactor_Type, Weight_U_Nc_Type,
      X_Type, U_Type, EmbeddedIntegratorStateSpace_Type>(
      kalman_filter, prediction_matrices, reference_trajectory, Weight_U_Nc,
      Adaptive_MPC_Phi_F_Updater_Function, delta_U_min, delta_U_max, U_min,
      U_max, Y_min, Y_max, solver_factor);

  return adaptive_mpc;
}

} // namespace two_wheel_vehicle_model_constraints_ada_mpc

#endif // __TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_ADA_MPC_HPP__
