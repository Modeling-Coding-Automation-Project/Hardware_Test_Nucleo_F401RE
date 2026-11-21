#ifndef TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_ADA_MPC_PHI_HPP_
#define TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_ADA_MPC_PHI_HPP_

#include "python_numpy.hpp"

namespace two_wheel_vehicle_model_constraints_ada_mpc_Phi {

using namespace PythonNumpy;

using SparseAvailable_ada_mpc_Phi =
    SparseAvailable<ColumnAvailable<false, false>,
                    ColumnAvailable<false, false>,
                    ColumnAvailable<false, false>, ColumnAvailable<true, false>,
                    ColumnAvailable<false, true>, ColumnAvailable<false, true>,
                    ColumnAvailable<false, true>, ColumnAvailable<true, false>,
                    ColumnAvailable<true, true>, ColumnAvailable<false, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<false, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<false, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<false, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<false, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<false, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<false, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<false, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<false, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<false, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<false, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<false, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<false, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<false, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<false, true>>;

using type = SparseMatrix_Type<float, SparseAvailable_ada_mpc_Phi>;

inline auto make(void) -> type {

  auto result = make_SparseMatrixZeros<float, SparseAvailable_ada_mpc_Phi>();

  result.unsafe_access(0) = static_cast<float>(0.0008399999999999999);
  result.unsafe_access(1) = static_cast<float>(0.01);
  result.unsafe_access(2) = static_cast<float>(0.0001);
  result.unsafe_access(3) = static_cast<float>(0.0);
  result.unsafe_access(4) = static_cast<float>(4.2000000000000004e-05);
  result.unsafe_access(5) = static_cast<float>(0.0016587744000000001);
  result.unsafe_access(6) = static_cast<float>(0.0);
  result.unsafe_access(7) = static_cast<float>(0.02);
  result.unsafe_access(8) = static_cast<float>(0.0);
  result.unsafe_access(9) = static_cast<float>(0.0003);
  result.unsafe_access(10) = static_cast<float>(8.400000000000001e-05);
  result.unsafe_access(11) = static_cast<float>(0.0);
  result.unsafe_access(12) = static_cast<float>(0.00012493872);
  result.unsafe_access(13) = static_cast<float>(0.0);
  result.unsafe_access(14) = static_cast<float>(0.0024568272983040004);
  result.unsafe_access(15) = static_cast<float>(0.0);
  result.unsafe_access(16) = static_cast<float>(0.03);
  result.unsafe_access(17) = static_cast<float>(0.0);
  result.unsafe_access(18) = static_cast<float>(0.0006000000000000001);
  result.unsafe_access(19) = static_cast<float>(0.0003338774400000001);
  result.unsafe_access(20) = static_cast<float>(0.0);
  result.unsafe_access(21) = static_cast<float>(0.00024778008491520005);
  result.unsafe_access(22) = static_cast<float>(0.0);
  result.unsafe_access(23) = static_cast<float>(0.0032346516300852334);
  result.unsafe_access(24) = static_cast<float>(0.0);
  result.unsafe_access(25) = static_cast<float>(0.04);
  result.unsafe_access(26) = static_cast<float>(0.0);
  result.unsafe_access(27) = static_cast<float>(0.001);
  result.unsafe_access(28) = static_cast<float>(0.0008294376098304003);
  result.unsafe_access(29) = static_cast<float>(0.0);
  result.unsafe_access(30) = static_cast<float>(0.0004095126664194618);
  result.unsafe_access(31) = static_cast<float>(0.0);
  result.unsafe_access(32) = static_cast<float>(0.003992729392995146);
  result.unsafe_access(33) = static_cast<float>(0.0);
  result.unsafe_access(34) = static_cast<float>(0.05);
  result.unsafe_access(35) = static_cast<float>(0.0);
  result.unsafe_access(36) = static_cast<float>(0.0015);
  result.unsafe_access(37) = static_cast<float>(0.0016484629426693235);
  result.unsafe_access(38) = static_cast<float>(0.0);
  result.unsafe_access(39) = static_cast<float>(0.0006091491360692191);
  result.unsafe_access(40) = static_cast<float>(0.0);
  result.unsafe_access(41) = static_cast<float>(0.004731531868831065);
  result.unsafe_access(42) = static_cast<float>(0.0);
  result.unsafe_access(43) = static_cast<float>(0.060000000000000005);
  result.unsafe_access(44) = static_cast<float>(0.0);
  result.unsafe_access(45) = static_cast<float>(0.0021000000000000003);
  result.unsafe_access(46) = static_cast<float>(0.0028667612148077616);
  result.unsafe_access(47) = static_cast<float>(0.0);
  result.unsafe_access(48) = static_cast<float>(0.0008457257295107723);
  result.unsafe_access(49) = static_cast<float>(0.0);
  result.unsafe_access(50) = static_cast<float>(0.005451519841311999);
  result.unsafe_access(51) = static_cast<float>(0.0);
  result.unsafe_access(52) = static_cast<float>(0.06999999999999999);
  result.unsafe_access(53) = static_cast<float>(0.0);
  result.unsafe_access(54) = static_cast<float>(0.0028000000000000004);
  result.unsafe_access(55) = static_cast<float>(0.0045582126738293064);
  result.unsafe_access(56) = static_cast<float>(0.0);
  result.unsafe_access(57) = static_cast<float>(0.0011183017215763724);
  result.unsafe_access(58) = static_cast<float>(0.0);
  result.unsafe_access(59) = static_cast<float>(0.006153143810012125);
  result.unsafe_access(60) = static_cast<float>(0.0);
  result.unsafe_access(61) = static_cast<float>(0.08);
  result.unsafe_access(62) = static_cast<float>(0.0);
  result.unsafe_access(63) = static_cast<float>(0.0036000000000000003);
  result.unsafe_access(64) = static_cast<float>(0.006794816116982051);
  result.unsafe_access(65) = static_cast<float>(0.0);
  result.unsafe_access(66) = static_cast<float>(0.0014259589120769784);
  result.unsafe_access(67) = static_cast<float>(0.0);
  result.unsafe_access(68) = static_cast<float>(0.006836844200501545);
  result.unsafe_access(69) = static_cast<float>(0.0);
  result.unsafe_access(70) = static_cast<float>(0.09);
  result.unsafe_access(71) = static_cast<float>(0.0);
  result.unsafe_access(72) = static_cast<float>(0.0045000000000000005);
  result.unsafe_access(73) = static_cast<float>(0.009646733941136009);
  result.unsafe_access(74) = static_cast<float>(0.0);
  result.unsafe_access(75) = static_cast<float>(0.0017678011221020557);
  result.unsafe_access(76) = static_cast<float>(0.0);
  result.unsafe_access(77) = static_cast<float>(0.007503051570743916);
  result.unsafe_access(78) = static_cast<float>(0.0);
  result.unsafe_access(79) = static_cast<float>(0.09999999999999999);
  result.unsafe_access(80) = static_cast<float>(0.0);
  result.unsafe_access(81) = static_cast<float>(0.0055000000000000005);
  result.unsafe_access(82) = static_cast<float>(0.013182336185340121);
  result.unsafe_access(83) = static_cast<float>(0.0);
  result.unsafe_access(84) = static_cast<float>(0.0021429537006392514);
  result.unsafe_access(85) = static_cast<float>(0.0);
  result.unsafe_access(86) = static_cast<float>(0.008152186813800531);
  result.unsafe_access(87) = static_cast<float>(0.0);
  result.unsafe_access(88) = static_cast<float>(0.11);
  result.unsafe_access(89) = static_cast<float>(0.0);
  result.unsafe_access(90) = static_cast<float>(0.006600000000000001);
  result.unsafe_access(91) = static_cast<float>(0.017468243586618623);
  result.unsafe_access(92) = static_cast<float>(0.0);
  result.unsafe_access(93) = static_cast<float>(0.0025505630413292777);
  result.unsafe_access(94) = static_cast<float>(0.0);
  result.unsafe_access(95) = static_cast<float>(0.008784661356890409);
  result.unsafe_access(96) = static_cast<float>(0.0);
  result.unsafe_access(97) = static_cast<float>(0.12);
  result.unsafe_access(98) = static_cast<float>(0.0);
  result.unsafe_access(99) = static_cast<float>(0.0078000000000000005);
  result.unsafe_access(100) = static_cast<float>(0.022569369669277178);
  result.unsafe_access(101) = static_cast<float>(0.0);
  result.unsafe_access(102) = static_cast<float>(0.0029897961091737983);
  result.unsafe_access(103) = static_cast<float>(0.0);
  result.unsafe_access(104) = static_cast<float>(0.009400877356855828);
  result.unsafe_access(105) = static_cast<float>(0.0);
  result.unsafe_access(106) = static_cast<float>(0.13);
  result.unsafe_access(107) = static_cast<float>(0.0);
  result.unsafe_access(108) = static_cast<float>(0.0091);
  result.unsafe_access(109) = static_cast<float>(0.02854896188762478);
  result.unsafe_access(110) = static_cast<float>(0.0);
  result.unsafe_access(111) = static_cast<float>(0.0034598399770165897);
  result.unsafe_access(112) = static_cast<float>(0.0);
  result.unsafe_access(113) = static_cast<float>(0.010001227892082664);
  result.unsafe_access(114) = static_cast<float>(0.0);
  result.unsafe_access(115) = static_cast<float>(0.14);
  result.unsafe_access(116) = static_cast<float>(0.0);
  result.unsafe_access(117) = static_cast<float>(0.0105);
  result.unsafe_access(118) = static_cast<float>(0.03546864184165796);
  result.unsafe_access(119) = static_cast<float>(0.0);
  result.unsafe_access(120) = static_cast<float>(0.003959901371620723);
  result.unsafe_access(121) = static_cast<float>(0.0);
  result.unsafe_access(122) = static_cast<float>(0.01058609715092478);
  result.unsafe_access(123) = static_cast<float>(0.0);
  result.unsafe_access(124) = static_cast<float>(0.15000000000000002);
  result.unsafe_access(125) = static_cast<float>(0.0);
  result.unsafe_access(126) = static_cast<float>(0.012);
  result.unsafe_access(127) = static_cast<float>(0.04338844458489941);
  result.unsafe_access(128) = static_cast<float>(0.0);
  result.unsafe_access(129) = static_cast<float>(0.004489206229166961);
  result.unsafe_access(130) = static_cast<float>(0.0);
  result.unsafe_access(131) = static_cast<float>(0.011155860616681465);
  result.unsafe_access(132) = static_cast<float>(0.0);
  result.unsafe_access(133) = static_cast<float>(0.16);

  return result;
}

} // namespace two_wheel_vehicle_model_constraints_ada_mpc_Phi

#endif // TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_ADA_MPC_PHI_HPP_
