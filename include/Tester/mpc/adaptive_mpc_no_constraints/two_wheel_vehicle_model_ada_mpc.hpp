#ifndef __TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_HPP__
#define __TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_HPP__

#include "two_wheel_vehicle_model_ada_mpc_B.hpp"
#include "two_wheel_vehicle_model_ada_mpc_F.hpp"
#include "two_wheel_vehicle_model_ada_mpc_Phi.hpp"
#include "two_wheel_vehicle_model_ada_mpc_Weight_U_Nc.hpp"
#include "two_wheel_vehicle_model_ada_mpc_ekf.hpp"
#include "two_wheel_vehicle_model_ada_mpc_ekf_parameter.hpp"
#include "two_wheel_vehicle_model_ada_mpc_solver_factor.hpp"
#include "two_wheel_vehicle_model_adaptive_mpc_phi_f_updater.hpp"

#include "python_mpc.hpp"

namespace two_wheel_vehicle_model_ada_mpc {

using namespace PythonNumpy;
using namespace PythonControl;
using namespace PythonMPC;

constexpr std::size_t NP = 16;
constexpr std::size_t NC = 1;

constexpr std::size_t INPUT_SIZE =
    two_wheel_vehicle_model_ada_mpc_ekf::INPUT_SIZE;
constexpr std::size_t STATE_SIZE =
    two_wheel_vehicle_model_ada_mpc_ekf::STATE_SIZE;
constexpr std::size_t OUTPUT_SIZE =
    two_wheel_vehicle_model_ada_mpc_ekf::OUTPUT_SIZE;

constexpr std::size_t AUGMENTED_STATE_SIZE = STATE_SIZE + OUTPUT_SIZE;

constexpr std::size_t NUMBER_OF_DELAY =
    two_wheel_vehicle_model_ada_mpc_ekf::NUMBER_OF_DELAY;

using EKF_Type = two_wheel_vehicle_model_ada_mpc_ekf::type;

using A_Type = typename EKF_Type::A_Type;

using B_Type = two_wheel_vehicle_model_ada_mpc_B::type;

using C_Type = typename EKF_Type::C_Type;

using X_Type = StateSpaceState_Type<float, STATE_SIZE>;

using U_Type = StateSpaceOutput_Type<float, INPUT_SIZE>;

using F_Type = two_wheel_vehicle_model_ada_mpc_F::type;

using Phi_Type = two_wheel_vehicle_model_ada_mpc_Phi::type;

using SolverFactor_Type = two_wheel_vehicle_model_ada_mpc_solver_factor::type;

using PredictionMatrices_Type =
    MPC_PredictionMatrices_Type<F_Type, Phi_Type, NP, NC, INPUT_SIZE,
                                AUGMENTED_STATE_SIZE, OUTPUT_SIZE>;

using Reference_Type = DenseMatrix_Type<float, OUTPUT_SIZE, 1>;

using ReferenceTrajectory_Type =
    MPC_ReferenceTrajectory_Type<Reference_Type, NP>;

using Parameter_Type =
    two_wheel_vehicle_model_ada_mpc_ekf_parameter::Parameter_Type;

using Weight_U_Nc_Type = two_wheel_vehicle_model_ada_mpc_Weight_U_Nc::type;

using EmbeddedIntegratorStateSpace_Type =
    typename EmbeddedIntegratorTypes<A_Type, B_Type, C_Type>::StateSpace_Type;

using type =
    AdaptiveMPC_NoConstraints_Type<B_Type, EKF_Type, PredictionMatrices_Type,
                                   ReferenceTrajectory_Type, Parameter_Type,
                                   SolverFactor_Type>;

inline auto make() -> type {

  auto kalman_filter = two_wheel_vehicle_model_ada_mpc_ekf::make();

  kalman_filter.X_hat.template set<0, 0>(static_cast<float>(0.0));
  kalman_filter.X_hat.template set<1, 0>(static_cast<float>(0.0));
  kalman_filter.X_hat.template set<2, 0>(static_cast<float>(0.0));
  kalman_filter.X_hat.template set<3, 0>(static_cast<float>(0.0));
  kalman_filter.X_hat.template set<4, 0>(static_cast<float>(0.0));
  kalman_filter.X_hat.template set<5, 0>(static_cast<float>(10.0));

  auto F = two_wheel_vehicle_model_ada_mpc_F::make();

  auto Phi = two_wheel_vehicle_model_ada_mpc_Phi::make();

  auto solver_factor = two_wheel_vehicle_model_ada_mpc_solver_factor::make();

  PredictionMatrices_Type prediction_matrices(F, Phi);

  ReferenceTrajectory_Type reference_trajectory;

  Weight_U_Nc_Type Weight_U_Nc =
      two_wheel_vehicle_model_ada_mpc_Weight_U_Nc::make();

  Adaptive_MPC_Phi_F_Updater_Function_Object<X_Type, U_Type, Parameter_Type,
                                             Phi_Type, F_Type,
                                             EmbeddedIntegratorStateSpace_Type>
      Adaptive_MPC_Phi_F_Updater_Function =
          two_wheel_vehicle_model_adaptive_mpc_phi_f_updater::
              Adaptive_MPC_Phi_F_Updater::update<
                  X_Type, U_Type, Parameter_Type, Phi_Type, F_Type,
                  EmbeddedIntegratorStateSpace_Type>;

  auto adaptive_mpc_nc = make_AdaptiveMPC_NoConstraints<
      B_Type, EKF_Type, PredictionMatrices_Type, ReferenceTrajectory_Type,
      Parameter_Type, SolverFactor_Type, Weight_U_Nc_Type, X_Type, U_Type,
      EmbeddedIntegratorStateSpace_Type>(
      kalman_filter, prediction_matrices, reference_trajectory, solver_factor,
      Weight_U_Nc, Adaptive_MPC_Phi_F_Updater_Function);

  return adaptive_mpc_nc;
}

} // namespace two_wheel_vehicle_model_ada_mpc

#endif // __TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_HPP__
