#ifndef SERVO_MOTOR_LTV_LTV_MPC_PHI_HPP_
#define SERVO_MOTOR_LTV_LTV_MPC_PHI_HPP_

#include "python_numpy.hpp"

namespace servo_motor_ltv_ltv_mpc_Phi {

using namespace PythonNumpy;

using SparseAvailable_ltv_mpc_Phi =
    SparseAvailable<ColumnAvailable<false, false>,
                    ColumnAvailable<false, false>,
                    ColumnAvailable<false, false>, ColumnAvailable<true, false>,
                    ColumnAvailable<false, false>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, false>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>,
                    ColumnAvailable<true, true>, ColumnAvailable<true, true>>;

using type = SparseMatrix_Type<float, SparseAvailable_ltv_mpc_Phi>;

inline auto make(void) -> type {

  auto result = make_SparseMatrixZeros<float, SparseAvailable_ltv_mpc_Phi>();

  result.unsafe_access(0) = static_cast<float>(-0.0008001243789611515);
  result.unsafe_access(1) = static_cast<float>(-0.001992309703613266);
  result.unsafe_access(2) = static_cast<float>(-0.0008001243789611515);
  result.unsafe_access(3) = static_cast<float>(1.6002408650535354e-05);
  result.unsafe_access(4) = static_cast<float>(-0.0032613695729506557);
  result.unsafe_access(5) = static_cast<float>(-0.001992309703613266);
  result.unsafe_access(6) = static_cast<float>(7.105069835479187e-05);
  result.unsafe_access(7) = static_cast<float>(1.6002408650535354e-05);
  result.unsafe_access(8) = static_cast<float>(-0.004349278903452054);
  result.unsafe_access(9) = static_cast<float>(-0.0032613695729506557);
  result.unsafe_access(10) = static_cast<float>(0.00018857365688901043);
  result.unsafe_access(11) = static_cast<float>(7.105069835479187e-05);
  result.unsafe_access(12) = static_cast<float>(-0.005050683230750509);
  result.unsafe_access(13) = static_cast<float>(-0.004349278903452054);
  result.unsafe_access(14) = static_cast<float>(0.0003872056455114522);
  result.unsafe_access(15) = static_cast<float>(0.00018857365688901043);
  result.unsafe_access(16) = static_cast<float>(-0.005222063374256645);
  result.unsafe_access(17) = static_cast<float>(-0.005050683230750509);
  result.unsafe_access(18) = static_cast<float>(0.000676919250075885);
  result.unsafe_access(19) = static_cast<float>(0.0003872056455114522);
  result.unsafe_access(20) = static_cast<float>(-0.004793891946869751);
  result.unsafe_access(21) = static_cast<float>(-0.005222063374256645);
  result.unsafe_access(22) = static_cast<float>(0.0010565879982114327);
  result.unsafe_access(23) = static_cast<float>(0.000676919250075885);
  result.unsafe_access(24) = static_cast<float>(-0.003779071669904123);
  result.unsafe_access(25) = static_cast<float>(-0.004793891946869751);
  result.unsafe_access(26) = static_cast<float>(0.0015131507686131757);
  result.unsafe_access(27) = static_cast<float>(0.0010565879982114327);
  result.unsafe_access(28) = static_cast<float>(-0.002273590199366996);
  result.unsafe_access(29) = static_cast<float>(-0.003779071669904123);
  result.unsafe_access(30) = static_cast<float>(0.0020224665736990966);
  result.unsafe_access(31) = static_cast<float>(0.0015131507686131757);
  result.unsafe_access(32) = static_cast<float>(-0.0004474296246385053);
  result.unsafe_access(33) = static_cast<float>(-0.002273590199366996);
  result.unsafe_access(34) = static_cast<float>(0.002551788293842877);
  result.unsafe_access(35) = static_cast<float>(0.0020224665736990966);
  result.unsafe_access(36) = static_cast<float>(0.0014743707881697133);
  result.unsafe_access(37) = static_cast<float>(-0.0004474296246385053);
  result.unsafe_access(38) = static_cast<float>(0.0030635926068733536);
  result.unsafe_access(39) = static_cast<float>(0.002551788293842877);
  result.unsafe_access(40) = static_cast<float>(0.0032389344700059245);
  result.unsafe_access(41) = static_cast<float>(0.0014743707881697133);
  result.unsafe_access(42) = static_cast<float>(0.003520319560146999);
  result.unsafe_access(43) = static_cast<float>(0.0030635926068733536);
  result.unsafe_access(44) = static_cast<float>(0.004598091789859274);
  result.unsafe_access(45) = static_cast<float>(0.0032389344700059245);
  result.unsafe_access(46) = static_cast<float>(0.0038894319084984003);
  result.unsafe_access(47) = static_cast<float>(0.003520319560146999);
  result.unsafe_access(48) = static_cast<float>(0.005342291387148386);
  result.unsafe_access(49) = static_cast<float>(0.004598091789859274);
  result.unsafe_access(50) = static_cast<float>(0.004148127348244415);
  result.unsafe_access(51) = static_cast<float>(0.0038894319084984003);
  result.unsafe_access(52) = static_cast<float>(0.005331435837751546);
  result.unsafe_access(53) = static_cast<float>(0.005342291387148386);
  result.unsafe_access(54) = static_cast<float>(0.004287042779051156);
  result.unsafe_access(55) = static_cast<float>(0.004148127348244415);
  result.unsafe_access(56) = static_cast<float>(0.004518304240415324);
  result.unsafe_access(57) = static_cast<float>(0.005331435837751546);
  result.unsafe_access(58) = static_cast<float>(0.004312384281743267);
  result.unsafe_access(59) = static_cast<float>(0.004287042779051156);
  result.unsafe_access(60) = static_cast<float>(0.002960933454428857);
  result.unsafe_access(61) = static_cast<float>(0.004518304240415324);
  result.unsafe_access(62) = static_cast<float>(0.004246093076452513);
  result.unsafe_access(63) = static_cast<float>(0.004312384281743267);
  result.unsafe_access(64) = static_cast<float>(0.0008215802990515499);
  result.unsafe_access(65) = static_cast<float>(0.002960933454428857);
  result.unsafe_access(66) = static_cast<float>(0.004123898038072222);
  result.unsafe_access(67) = static_cast<float>(0.004246093076452513);
  result.unsafe_access(68) = static_cast<float>(-0.0016484577193429048);
  result.unsafe_access(69) = static_cast<float>(0.0008215802990515499);

  return result;
}

} // namespace servo_motor_ltv_ltv_mpc_Phi

#endif // SERVO_MOTOR_LTV_LTV_MPC_PHI_HPP_
