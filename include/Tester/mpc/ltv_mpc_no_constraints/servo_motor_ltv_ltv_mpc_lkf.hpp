#ifndef __SERVO_MOTOR_LTV_LTV_MPC_LKF_HPP__
#define __SERVO_MOTOR_LTV_LTV_MPC_LKF_HPP__

#include "servo_motor_ltv_ltv_mpc_lkf_ss.hpp"

#include "python_control.hpp"

namespace servo_motor_ltv_ltv_mpc_lkf {

using namespace PythonNumpy;
using namespace PythonControl;

constexpr std::size_t NUMBER_OF_DELAY = 0;

using LkfStateSpace_Type = servo_motor_ltv_ltv_mpc_lkf_ss::type;

constexpr std::size_t STATE_SIZE = LkfStateSpace_Type::A_Type::COLS;
constexpr std::size_t INPUT_SIZE = LkfStateSpace_Type::B_Type::ROWS;
constexpr std::size_t OUTPUT_SIZE = LkfStateSpace_Type::C_Type::COLS;

using Q_Type = KalmanFilter_Q_Type<float, STATE_SIZE>;

using R_Type = KalmanFilter_R_Type<float, OUTPUT_SIZE>;

using type = LinearKalmanFilter_Type<LkfStateSpace_Type, Q_Type, R_Type>;

inline auto make() -> type {

  auto lkf_state_space = servo_motor_ltv_ltv_mpc_lkf_ss::make();

  auto Q = make_KalmanFilter_Q<STATE_SIZE>(
      static_cast<float>(1.0), static_cast<float>(1.0), static_cast<float>(1.0),
      static_cast<float>(1.0));

  auto R = make_KalmanFilter_R<OUTPUT_SIZE>(static_cast<float>(1.0),
                                            static_cast<float>(1.0));

  auto lkf = make_LinearKalmanFilter(lkf_state_space, Q, R);

  lkf.P = make_DenseMatrix<STATE_SIZE, STATE_SIZE>(
      static_cast<float>(0.04893928924912184),
      static_cast<float>(0.008278739813038703),
      static_cast<float>(0.9787742027878747),
      static_cast<float>(0.002865103798186428),
      static_cast<float>(0.008278739813038703),
      static_cast<float>(8.588725388823203),
      static_cast<float>(0.1655700352338051),
      static_cast<float>(0.00111561715961999),
      static_cast<float>(0.978774202787875),
      static_cast<float>(0.16557003523380448),
      static_cast<float>(19.575496476307546),
      static_cast<float>(0.05730212969415684),
      static_cast<float>(0.0028651037981862566),
      static_cast<float>(0.0011156171596189693),
      static_cast<float>(0.0573021296941534),
      static_cast<float>(1.3159809338068813));

  lkf.G = make_DenseMatrix<STATE_SIZE, OUTPUT_SIZE>(
      static_cast<float>(0.048939289249122116),
      static_cast<float>(0.0007413756985857118),
      static_cast<float>(0.00827873981303891),
      static_cast<float>(0.0003047530998279957),
      static_cast<float>(0.9787742027878751),
      static_cast<float>(-0.0007950387916633945),
      static_cast<float>(0.002865103798186256),
      static_cast<float>(-3.439282045136472e-06));

  return lkf;
}

} // namespace servo_motor_ltv_ltv_mpc_lkf

#endif // __SERVO_MOTOR_LTV_LTV_MPC_LKF_HPP__
