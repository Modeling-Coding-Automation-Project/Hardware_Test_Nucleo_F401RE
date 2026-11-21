#ifndef TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_SOLVER_FACTOR_HPP_
#define TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_SOLVER_FACTOR_HPP_

#include "python_numpy.hpp"

namespace two_wheel_vehicle_model_ada_mpc_solver_factor {

using namespace PythonNumpy;

using SparseAvailable_ada_mpc_solver_factor = SparseAvailable<
    ColumnAvailable<
        false, false, false, true, true, true, true, true, true, true, true,
        true, true, true, true, true, true, true, true, true, true, true, true,
        true, true, true, true, true, true, true, true, true, true, true, true,
        true, true, true, true, true, true, true, true, true, true, true, true,
        true, true, true, true, true, true, true, true, true, true, true, true,
        true, true, true, true, true, true, true, true, true, true, true, true,
        true, true, true, true, true, true, true, true, true>,
    ColumnAvailable<
        false, false, false, true, true, true, true, true, true, true, true,
        true, true, true, true, true, true, true, true, true, true, true, true,
        true, true, true, true, true, true, true, true, true, true, true, true,
        true, true, true, true, true, true, true, true, true, true, true, true,
        true, true, true, true, true, true, true, true, true, true, true, true,
        true, true, true, true, true, true, true, true, true, true, true, true,
        true, true, true, true, true, true, true, true, true>>;

using type = SparseMatrix_Type<float, SparseAvailable_ada_mpc_solver_factor>;

inline auto make(void) -> type {

  auto result =
      make_SparseMatrixZeros<float, SparseAvailable_ada_mpc_solver_factor>();

  result.unsafe_access(0) = static_cast<float>(0.007925373319758792);
  result.unsafe_access(1) = static_cast<float>(-0.0);
  result.unsafe_access(2) = static_cast<float>(-0.0);
  result.unsafe_access(3) = static_cast<float>(-0.0);
  result.unsafe_access(4) = static_cast<float>(0.0003962686659879396);
  result.unsafe_access(5) = static_cast<float>(0.015650483777689166);
  result.unsafe_access(6) = static_cast<float>(-0.0);
  result.unsafe_access(7) = static_cast<float>(-0.0);
  result.unsafe_access(8) = static_cast<float>(0.0007925373319758792);
  result.unsafe_access(9) = static_cast<float>(0.0011787928548723978);
  result.unsafe_access(10) = static_cast<float>(0.023180087525278094);
  result.unsafe_access(11) = static_cast<float>(-0.0);
  result.unsafe_access(12) = static_cast<float>(-0.0);
  result.unsafe_access(13) = static_cast<float>(0.003150123041720676);
  result.unsafe_access(14) = static_cast<float>(0.0023377972311363028);
  result.unsafe_access(15) = static_cast<float>(0.030518835390228323);
  result.unsafe_access(16) = static_cast<float>(-0.0);
  result.unsafe_access(17) = static_cast<float>(-0.0);
  result.unsafe_access(18) = static_cast<float>(0.007825717503993282);
  result.unsafe_access(19) = static_cast<float>(0.00386373900064772);
  result.unsafe_access(20) = static_cast<float>(0.03767127500507197);
  result.unsafe_access(21) = static_cast<float>(-0.0);
  result.unsafe_access(22) = static_cast<float>(-0.0);
  result.unsafe_access(23) = static_cast<float>(0.015553195505288718);
  result.unsafe_access(24) = static_cast<float>(0.005747302750901318);
  result.unsafe_access(25) = static_cast<float>(0.04464185289859783);
  result.unsafe_access(26) = static_cast<float>(-0.0);
  result.unsafe_access(27) = static_cast<float>(-0.0);
  result.unsafe_access(28) = static_cast<float>(0.02704780100709135);
  result.unsafe_access(29) = static_cast<float>(0.007979395395831208);
  result.unsafe_access(30) = static_cast<float>(0.05143491655055928);
  result.unsafe_access(31) = static_cast<float>(-0.0);
  result.unsafe_access(32) = static_cast<float>(-0.0);
  result.unsafe_access(33) = static_cast<float>(0.04300659179875377);
  result.unsafe_access(34) = static_cast<float>(0.010551141223359175);
  result.unsafe_access(35) = static_cast<float>(0.058054716410129825);
  result.unsafe_access(36) = static_cast<float>(-0.0);
  result.unsafe_access(37) = static_cast<float>(-0.0);
  result.unsafe_access(38) = static_cast<float>(0.06410887424547212);
  result.unsafe_access(39) = static_cast<float>(0.013453877043865664);
  result.unsafe_access(40) = static_cast<float>(0.06450540787857449);
  result.unsafe_access(41) = static_cast<float>(-0.0);
  result.unsafe_access(42) = static_cast<float>(-0.0);
  result.unsafe_access(43) = static_cast<float>(0.09101662833320345);
  result.unsafe_access(44) = static_cast<float>(0.01667914743779439);
  result.unsafe_access(45) = static_cast<float>(0.07079105325660491);
  result.unsafe_access(46) = static_cast<float>(-0.0);
  result.unsafe_access(47) = static_cast<float>(-0.0);
  result.unsafe_access(48) = static_cast<float>(0.12437492320879225);
  result.unsafe_access(49) = static_cast<float>(0.020218700100624633);
  result.unsafe_access(50) = static_cast<float>(0.07691562365688591);
  result.unsafe_access(51) = static_cast<float>(-0.0);
  result.unsafe_access(52) = static_cast<float>(-0.0);
  result.unsafe_access(53) = static_cast<float>(0.1648123234100415);
  result.unsafe_access(54) = static_cast<float>(0.024064481283468928);
  result.unsafe_access(55) = static_cast<float>(0.08288300088216108);
  result.unsafe_access(56) = static_cast<float>(-0.0);
  result.unsafe_access(57) = static_cast<float>(-0.0);
  result.unsafe_access(58) = static_cast<float>(0.21294128597697937);
  result.unsafe_access(59) = static_cast<float>(0.02820863132757698);
  result.unsafe_access(60) = static_cast<float>(0.08869697926946396);
  result.unsafe_access(61) = static_cast<float>(-0.0);
  result.unsafe_access(62) = static_cast<float>(-0.0);
  result.unsafe_access(63) = static_cast<float>(0.2693585486321334);
  result.unsafe_access(64) = static_cast<float>(0.03264348029105018);
  result.unsafe_access(65) = static_cast<float>(0.09436126750088025);
  result.unsafe_access(66) = static_cast<float>(-0.0);
  result.unsafe_access(67) = static_cast<float>(-0.0);
  result.unsafe_access(68) = static_cast<float>(0.33464550921423375);
  result.unsafe_access(69) = static_cast<float>(0.03736154366609419);
  result.unsafe_access(70) = static_cast<float>(0.09987949038132596);
  result.unsafe_access(71) = static_cast<float>(-0.0);
  result.unsafe_access(72) = static_cast<float>(-0.0);
  result.unsafe_access(73) = static_cast<float>(0.40936859654642216);
  result.unsafe_access(74) = static_cast<float>(0.042355518185160486);
  result.unsafe_access(75) = static_cast<float>(0.10525519058380374);
  result.unsafe_access(76) = static_cast<float>(-0.0);
  result.unsafe_access(77) = static_cast<float>(-0.0);
  result.unsafe_access(78) = static_cast<float>(0.03998066854714409);
  result.unsafe_access(79) = static_cast<float>(0.00039980668547144095);
  result.unsafe_access(80) = static_cast<float>(-0.0);
  result.unsafe_access(81) = static_cast<float>(-0.0);
  result.unsafe_access(82) = static_cast<float>(-0.0);
  result.unsafe_access(83) = static_cast<float>(0.07996133709428818);
  result.unsafe_access(84) = static_cast<float>(0.0011994200564143227);
  result.unsafe_access(85) = static_cast<float>(-0.0);
  result.unsafe_access(86) = static_cast<float>(-0.0);
  result.unsafe_access(87) = static_cast<float>(-0.0);
  result.unsafe_access(88) = static_cast<float>(0.11994200564143227);
  result.unsafe_access(89) = static_cast<float>(0.002398840112828646);
  result.unsafe_access(90) = static_cast<float>(-0.0);
  result.unsafe_access(91) = static_cast<float>(-0.0);
  result.unsafe_access(92) = static_cast<float>(-0.0);
  result.unsafe_access(93) = static_cast<float>(0.15992267418857636);
  result.unsafe_access(94) = static_cast<float>(0.003998066854714408);
  result.unsafe_access(95) = static_cast<float>(-0.0);
  result.unsafe_access(96) = static_cast<float>(-0.0);
  result.unsafe_access(97) = static_cast<float>(-0.0);
  result.unsafe_access(98) = static_cast<float>(0.19990334273572047);
  result.unsafe_access(99) = static_cast<float>(0.005997100282071614);
  result.unsafe_access(100) = static_cast<float>(-0.0);
  result.unsafe_access(101) = static_cast<float>(-0.0);
  result.unsafe_access(102) = static_cast<float>(-0.0);
  result.unsafe_access(103) = static_cast<float>(0.23988401128286457);
  result.unsafe_access(104) = static_cast<float>(0.00839594039490026);
  result.unsafe_access(105) = static_cast<float>(-0.0);
  result.unsafe_access(106) = static_cast<float>(-0.0);
  result.unsafe_access(107) = static_cast<float>(-0.0);
  result.unsafe_access(108) = static_cast<float>(0.27986467983000857);
  result.unsafe_access(109) = static_cast<float>(0.011194587193200347);
  result.unsafe_access(110) = static_cast<float>(-0.0);
  result.unsafe_access(111) = static_cast<float>(-0.0);
  result.unsafe_access(112) = static_cast<float>(-0.0);
  result.unsafe_access(113) = static_cast<float>(0.3198453483771527);
  result.unsafe_access(114) = static_cast<float>(0.014393040676971874);
  result.unsafe_access(115) = static_cast<float>(-0.0);
  result.unsafe_access(116) = static_cast<float>(-0.0);
  result.unsafe_access(117) = static_cast<float>(-0.0);
  result.unsafe_access(118) = static_cast<float>(0.3598260169242968);
  result.unsafe_access(119) = static_cast<float>(0.017991300846214843);
  result.unsafe_access(120) = static_cast<float>(-0.0);
  result.unsafe_access(121) = static_cast<float>(-0.0);
  result.unsafe_access(122) = static_cast<float>(-0.0);
  result.unsafe_access(123) = static_cast<float>(0.3998066854714409);
  result.unsafe_access(124) = static_cast<float>(0.02198936770092925);
  result.unsafe_access(125) = static_cast<float>(-0.0);
  result.unsafe_access(126) = static_cast<float>(-0.0);
  result.unsafe_access(127) = static_cast<float>(-0.0);
  result.unsafe_access(128) = static_cast<float>(0.439787354018585);
  result.unsafe_access(129) = static_cast<float>(0.026387241241115102);
  result.unsafe_access(130) = static_cast<float>(-0.0);
  result.unsafe_access(131) = static_cast<float>(-0.0);
  result.unsafe_access(132) = static_cast<float>(-0.0);
  result.unsafe_access(133) = static_cast<float>(0.4797680225657291);
  result.unsafe_access(134) = static_cast<float>(0.031184921466772392);
  result.unsafe_access(135) = static_cast<float>(-0.0);
  result.unsafe_access(136) = static_cast<float>(-0.0);
  result.unsafe_access(137) = static_cast<float>(-0.0);
  result.unsafe_access(138) = static_cast<float>(0.5197486911128733);
  result.unsafe_access(139) = static_cast<float>(0.03638240837790112);
  result.unsafe_access(140) = static_cast<float>(-0.0);
  result.unsafe_access(141) = static_cast<float>(-0.0);
  result.unsafe_access(142) = static_cast<float>(-0.0);
  result.unsafe_access(143) = static_cast<float>(0.5597293596600172);
  result.unsafe_access(144) = static_cast<float>(0.04197970197450129);
  result.unsafe_access(145) = static_cast<float>(-0.0);
  result.unsafe_access(146) = static_cast<float>(-0.0);
  result.unsafe_access(147) = static_cast<float>(-0.0);
  result.unsafe_access(148) = static_cast<float>(0.5997100282071615);
  result.unsafe_access(149) = static_cast<float>(0.047976802256572915);
  result.unsafe_access(150) = static_cast<float>(-0.0);
  result.unsafe_access(151) = static_cast<float>(-0.0);
  result.unsafe_access(152) = static_cast<float>(-0.0);
  result.unsafe_access(153) = static_cast<float>(0.6396906967543055);

  return result;
}

} // namespace two_wheel_vehicle_model_ada_mpc_solver_factor

#endif // TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_SOLVER_FACTOR_HPP_
