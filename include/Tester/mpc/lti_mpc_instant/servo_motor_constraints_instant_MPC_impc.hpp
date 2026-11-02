#ifndef SERVO_MOTOR_CONSTRAINTS_INSTANT_MPC_IMPC_HPP_
#define SERVO_MOTOR_CONSTRAINTS_INSTANT_MPC_IMPC_HPP_

#include "servo_motor_constraints_instant_MPC_impc_Ag.hpp"
#include "servo_motor_constraints_instant_MPC_impc_AgKT.hpp"
#include "servo_motor_constraints_instant_MPC_impc_K.hpp"
#include "servo_motor_constraints_instant_MPC_impc_L.hpp"
#include "servo_motor_constraints_instant_MPC_impc_M_sub_inv.hpp"
#include "servo_motor_constraints_instant_MPC_impc_U_max.hpp"
#include "servo_motor_constraints_instant_MPC_impc_U_min.hpp"
#include "servo_motor_constraints_instant_MPC_impc_WCzT_Qk.hpp"
#include "servo_motor_constraints_instant_MPC_impc_bg.hpp"
#include "servo_motor_constraints_instant_MPC_impc_delta_U_max.hpp"
#include "servo_motor_constraints_instant_MPC_impc_delta_U_min.hpp"
#include "servo_motor_constraints_instant_MPC_impc_lkf.hpp"
#include "servo_motor_constraints_instant_MPC_impc_q_K_matrix.hpp"
#include "servo_motor_constraints_instant_MPC_impc_w_unc_map.hpp"
#include "servo_motor_constraints_instant_MPC_impc_w_unc_traj_map.hpp"


#include "python_mpc.hpp"

namespace servo_motor_constraints_instant_MPC_impc {

using namespace PythonNumpy;
using namespace PythonControl;
using namespace PythonMPC;

constexpr std::size_t NP = 20;
constexpr std::size_t NC = 2;

constexpr std::size_t INPUT_SIZE =
    servo_motor_constraints_instant_MPC_impc_lkf::INPUT_SIZE;
constexpr std::size_t STATE_SIZE =
    servo_motor_constraints_instant_MPC_impc_lkf::STATE_SIZE;
constexpr std::size_t OUTPUT_SIZE =
    servo_motor_constraints_instant_MPC_impc_lkf::OUTPUT_SIZE;

constexpr std::size_t NUMBER_OF_DELAY =
    servo_motor_constraints_instant_MPC_impc_lkf::NUMBER_OF_DELAY;

constexpr std::size_t N_SUB = 3;
constexpr float DTZETA_SUB = static_cast<float>(0.8333333333333334);

using LKF_Type = servo_motor_constraints_instant_MPC_impc_lkf::type;

using WCzT_Qk_Type = servo_motor_constraints_instant_MPC_impc_WCzT_Qk::type;

using Delta_U_Min_Type =
    servo_motor_constraints_instant_MPC_impc_delta_U_min::type;

using Delta_U_Max_Type =
    servo_motor_constraints_instant_MPC_impc_delta_U_max::type;

using U_Min_Type = servo_motor_constraints_instant_MPC_impc_U_min::type;

using U_Max_Type = servo_motor_constraints_instant_MPC_impc_U_max::type;

using Ag_Type = servo_motor_constraints_instant_MPC_impc_Ag::type;

using Bg_Type = servo_motor_constraints_instant_MPC_impc_bg::type;

using K_Type = servo_motor_constraints_instant_MPC_impc_K::type;

using L_Type = servo_motor_constraints_instant_MPC_impc_L::type;

using q_K_matrix_Type =
    servo_motor_constraints_instant_MPC_impc_q_K_matrix::type;

using W_Unc_Map_Type = servo_motor_constraints_instant_MPC_impc_w_unc_map::type;

using W_Unc_Traj_Map_Type =
    servo_motor_constraints_instant_MPC_impc_w_unc_traj_map::type;

using M_Sub_Inv_Type = servo_motor_constraints_instant_MPC_impc_M_sub_inv::type;

using AgKT_Type = servo_motor_constraints_instant_MPC_impc_AgKT::type;

using Reference_Type = DenseMatrix_Type<float, OUTPUT_SIZE, 1>;

using ReferenceTrajectory_Type = Reference_Type;

using type =
    InstantMPC_LTI_Type<NP, NC, LKF_Type, ReferenceTrajectory_Type,
                        Delta_U_Min_Type, Delta_U_Max_Type, U_Min_Type,
                        U_Max_Type, WCzT_Qk_Type, Ag_Type, Bg_Type, K_Type,
                        L_Type, q_K_matrix_Type, W_Unc_Map_Type,
                        W_Unc_Traj_Map_Type, M_Sub_Inv_Type, AgKT_Type>;

inline auto make(void) -> type {

  auto kalman_filter = servo_motor_constraints_instant_MPC_impc_lkf::make();

  auto WCzT_Qk = servo_motor_constraints_instant_MPC_impc_WCzT_Qk::make();

  auto delta_U_min =
      servo_motor_constraints_instant_MPC_impc_delta_U_min::make();

  auto delta_U_max =
      servo_motor_constraints_instant_MPC_impc_delta_U_max::make();

  auto U_min = servo_motor_constraints_instant_MPC_impc_U_min::make();

  U_min.template set<0, 0>(static_cast<float>(-180.0));

  auto U_max = servo_motor_constraints_instant_MPC_impc_U_max::make();

  U_max.template set<0, 0>(static_cast<float>(180.0));

  auto Ag = servo_motor_constraints_instant_MPC_impc_Ag::make();

  auto bg = servo_motor_constraints_instant_MPC_impc_bg::make();

  constexpr std::size_t N_sub = N_SUB;
  constexpr float dtzeta_sub = DTZETA_SUB;

  auto K = servo_motor_constraints_instant_MPC_impc_K::make();

  auto L = servo_motor_constraints_instant_MPC_impc_L::make();

  auto q_K_matrix = servo_motor_constraints_instant_MPC_impc_q_K_matrix::make();

  auto w_unc_map = servo_motor_constraints_instant_MPC_impc_w_unc_map::make();

  auto w_unc_traj_map =
      servo_motor_constraints_instant_MPC_impc_w_unc_traj_map::make();

  auto M_sub_inv = servo_motor_constraints_instant_MPC_impc_M_sub_inv::make();

  auto AgKT = servo_motor_constraints_instant_MPC_impc_AgKT::make();

  auto impc =
      make_InstantMPC_LTI<NP, NC, LKF_Type, ReferenceTrajectory_Type,
                          Delta_U_Min_Type, Delta_U_Max_Type, U_Min_Type,
                          U_Max_Type, WCzT_Qk_Type, Ag_Type, Bg_Type, K_Type,
                          L_Type, q_K_matrix_Type, W_Unc_Map_Type,
                          W_Unc_Traj_Map_Type, M_Sub_Inv_Type, AgKT_Type>(
          kalman_filter, WCzT_Qk, delta_U_min, delta_U_max, U_min, U_max, Ag,
          bg, N_sub, dtzeta_sub, K, L, q_K_matrix, w_unc_map, w_unc_traj_map,
          M_sub_inv, AgKT);

  return impc;
}

} // namespace servo_motor_constraints_instant_MPC_impc

#endif // SERVO_MOTOR_CONSTRAINTS_INSTANT_MPC_IMPC_HPP_
