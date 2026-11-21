#ifndef SERVO_MOTOR_LTV_LTV_MPC_SOLVER_FACTOR_HPP_
#define SERVO_MOTOR_LTV_LTV_MPC_SOLVER_FACTOR_HPP_

#include "python_numpy.hpp"

namespace servo_motor_ltv_ltv_mpc_solver_factor {

using namespace PythonNumpy;

using SparseAvailable_ltv_mpc_solver_factor = SparseAvailable<
    ColumnAvailable<false, false, false, true, false, true, true, true, true,
                    true, true, true, true, true, true, true, true, true, true,
                    true, true, true, true, true, true, true, true, true, true,
                    true, true, true, true, true, true, true, true, true, true,
                    true>,
    ColumnAvailable<false, false, false, true, false, true, true, true, true,
                    true, true, true, true, true, true, true, true, true, true,
                    true, true, true, true, true, true, true, true, true, true,
                    true, true, true, true, true, true, true, true, true, true,
                    true>>;

using type = SparseMatrix_Type<float, SparseAvailable_ltv_mpc_solver_factor>;

inline auto make(void) -> type {

  auto result =
      make_SparseMatrixZeros<float, SparseAvailable_ltv_mpc_solver_factor>();

  result.unsafe_access(0) = static_cast<float>(-0.6171258210073844);
  result.unsafe_access(1) = static_cast<float>(-1.3725300408027512);
  result.unsafe_access(2) = static_cast<float>(0.012342455543448438);
  result.unsafe_access(3) = static_cast<float>(-2.1068111311062823);
  result.unsafe_access(4) = static_cast<float>(0.05151825677565598);
  result.unsafe_access(5) = static_cast<float>(-2.6856053872170063);
  result.unsafe_access(6) = static_cast<float>(0.13087129240218684);
  result.unsafe_access(7) = static_cast<float>(-3.003448947895235);
  result.unsafe_access(8) = static_cast<float>(0.2599685386383381);
  result.unsafe_access(9) = static_cast<float>(-2.9917674710448123);
  result.unsafe_access(10) = static_cast<float>(0.4426796370809581);
  result.unsafe_access(11) = static_cast<float>(-2.626372534604199);
  result.unsafe_access(12) = static_cast<float>(0.6760902799569568);
  result.unsafe_access(13) = static_cast<float>(-1.9314765844112665);
  result.unsafe_access(14) = static_cast<float>(0.9503576398636349);
  result.unsafe_access(15) = static_cast<float>(-0.9784672133802993);
  result.unsafe_access(16) = static_cast<float>(1.2495410405698046);
  result.unsafe_access(17) = static_cast<float>(0.1212385384813903);
  result.unsafe_access(18) = static_cast<float>(1.5533345927696194);
  result.unsafe_access(19) = static_cast<float>(1.2289357009836985);
  result.unsafe_access(20) = static_cast<float>(1.839513783469139);
  result.unsafe_access(21) = static_cast<float>(2.195741503780159);
  result.unsafe_access(22) = static_cast<float>(2.086805492418753);
  result.unsafe_access(23) = static_cast<float>(2.8821132785806065);
  result.unsafe_access(24) = static_cast<float>(2.2778180634309013);
  result.unsafe_access(25) = static_cast<float>(3.17732870245003);
  result.unsafe_access(26) = static_cast<float>(2.4016380717347383);
  result.unsafe_access(27) = static_cast<float>(3.0163134148393946);
  result.unsafe_access(28) = static_cast<float>(2.4557208495393574);
  result.unsafe_access(29) = static_cast<float>(2.3913818689311643);
  result.unsafe_access(30) = static_cast<float>(2.4467735310403302);
  result.unsafe_access(31) = static_cast<float>(1.3569826246257768);
  result.unsafe_access(32) = static_cast<float>(2.390446174780214);
  result.unsafe_access(33) = static_cast<float>(0.026358394407458602);
  result.unsafe_access(34) = static_cast<float>(2.3097956538097444);
  result.unsafe_access(35) = static_cast<float>(-1.439948675023278);
  result.unsafe_access(36) = static_cast<float>(0.16411325350563521);
  result.unsafe_access(37) = static_cast<float>(-0.21734289051299538);
  result.unsafe_access(38) = static_cast<float>(-0.003282248881075012);
  result.unsafe_access(39) = static_cast<float>(-0.8897639183998537);
  result.unsafe_access(40) = static_cast<float>(-0.0020535497571873246);
  result.unsafe_access(41) = static_cast<float>(-1.6594842036050335);
  result.unsafe_access(42) = static_cast<float>(0.01690890531029031);
  result.unsafe_access(43) = static_cast<float>(-2.366755564387623);
  result.unsafe_access(44) = static_cast<float>(0.06811276267221113);
  result.unsafe_access(45) = static_cast<float>(-2.88035416200542);
  result.unsafe_access(46) = static_cast<float>(0.16409131761152665);
  result.unsafe_access(47) = static_cast<float>(-3.1022571451598133);
  result.unsafe_access(48) = static_cast<float>(0.31287776757957925);
  result.unsafe_access(49) = static_cast<float>(-2.9754226305369036);
  result.unsafe_access(50) = static_cast<float>(0.5162697686214406);
  result.unsafe_access(51) = static_cast<float>(-2.4902571872646218);
  result.unsafe_access(52) = static_cast<float>(0.7690003788691016);
  result.unsafe_access(53) = static_cast<float>(-1.686992696254027);
  result.unsafe_access(54) = static_cast<float>(1.058899419023808);
  result.unsafe_access(55) = static_cast<float>(-0.6524585249352299);
  result.unsafe_access(56) = static_cast<float>(1.3680432661746886);
  result.unsafe_access(57) = static_cast<float>(0.48915115542905496);
  result.unsafe_access(58) = static_cast<float>(1.6747791033437591);
  result.unsafe_access(59) = static_cast<float>(1.5908979572245063);
  result.unsafe_access(60) = static_cast<float>(1.9563952494436585);
  result.unsafe_access(61) = static_cast<float>(2.5016050294251673);
  result.unsafe_access(62) = static_cast<float>(2.1921128554794627);
  result.unsafe_access(63) = static_cast<float>(3.0860632163894928);
  result.unsafe_access(64) = static_cast<float>(2.366012785528325);
  result.unsafe_access(65) = static_cast<float>(3.2443514343007354);
  result.unsafe_access(66) = static_cast<float>(2.469496802059399);
  result.unsafe_access(67) = static_cast<float>(2.9276220419821275);
  result.unsafe_access(68) = static_cast<float>(2.502919934639234);
  result.unsafe_access(69) = static_cast<float>(2.148000282486028);
  result.unsafe_access(70) = static_cast<float>(2.476119766789847);
  result.unsafe_access(71) = static_cast<float>(0.9808857657430733);

  return result;
}

} // namespace servo_motor_ltv_ltv_mpc_solver_factor

#endif // SERVO_MOTOR_LTV_LTV_MPC_SOLVER_FACTOR_HPP_
