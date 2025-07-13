#ifndef __SERVO_MOTOR_LTV_CONSTRAINTS_LTV_MPC_PHI_HPP__
#define __SERVO_MOTOR_LTV_CONSTRAINTS_LTV_MPC_PHI_HPP__

#include "python_numpy.hpp"

namespace servo_motor_ltv_constraints_ltv_mpc_Phi {

using namespace PythonNumpy;

using SparseAvailable_ltv_mpc_Phi = SparseAvailable<
    ColumnAvailable<false, false>,
    ColumnAvailable<false, false>,
    ColumnAvailable<false, false>,
    ColumnAvailable<true, false>,
    ColumnAvailable<false, false>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, false>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>,
    ColumnAvailable<true, true>
>;

using type = SparseMatrix_Type<double, SparseAvailable_ltv_mpc_Phi>;

inline auto make(void) -> type {

  return make_SparseMatrix<SparseAvailable_ltv_mpc_Phi>(
    static_cast<double>(-0.0008001243789611515),
    static_cast<double>(-0.001992309703613266),
    static_cast<double>(-0.0008001243789611515),
    static_cast<double>(1.6002408650535354e-05),
    static_cast<double>(-0.0032613695729506557),
    static_cast<double>(-0.001992309703613266),
    static_cast<double>(7.105069835479187e-05),
    static_cast<double>(1.6002408650535354e-05),
    static_cast<double>(-0.004349278903452054),
    static_cast<double>(-0.0032613695729506557),
    static_cast<double>(0.00018857365688901043),
    static_cast<double>(7.105069835479187e-05),
    static_cast<double>(-0.005050683230750509),
    static_cast<double>(-0.004349278903452054),
    static_cast<double>(0.0003872056455114522),
    static_cast<double>(0.00018857365688901043),
    static_cast<double>(-0.005222063374256645),
    static_cast<double>(-0.005050683230750509),
    static_cast<double>(0.000676919250075885),
    static_cast<double>(0.0003872056455114522),
    static_cast<double>(-0.004793891946869751),
    static_cast<double>(-0.005222063374256645),
    static_cast<double>(0.0010565879982114327),
    static_cast<double>(0.000676919250075885),
    static_cast<double>(-0.003779071669904123),
    static_cast<double>(-0.004793891946869751),
    static_cast<double>(0.0015131507686131757),
    static_cast<double>(0.0010565879982114327),
    static_cast<double>(-0.002273590199366996),
    static_cast<double>(-0.003779071669904123),
    static_cast<double>(0.0020224665736990966),
    static_cast<double>(0.0015131507686131757),
    static_cast<double>(-0.0004474296246385053),
    static_cast<double>(-0.002273590199366996),
    static_cast<double>(0.002551788293842877),
    static_cast<double>(0.0020224665736990966),
    static_cast<double>(0.0014743707881697133),
    static_cast<double>(-0.0004474296246385053),
    static_cast<double>(0.0030635926068733536),
    static_cast<double>(0.002551788293842877),
    static_cast<double>(0.0032389344700059245),
    static_cast<double>(0.0014743707881697133),
    static_cast<double>(0.003520319560146999),
    static_cast<double>(0.0030635926068733536),
    static_cast<double>(0.004598091789859274),
    static_cast<double>(0.0032389344700059245),
    static_cast<double>(0.0038894319084984003),
    static_cast<double>(0.003520319560146999),
    static_cast<double>(0.005342291387148386),
    static_cast<double>(0.004598091789859274),
    static_cast<double>(0.004148127348244415),
    static_cast<double>(0.0038894319084984003),
    static_cast<double>(0.005331435837751546),
    static_cast<double>(0.005342291387148386),
    static_cast<double>(0.004287042779051156),
    static_cast<double>(0.004148127348244415),
    static_cast<double>(0.004518304240415324),
    static_cast<double>(0.005331435837751546),
    static_cast<double>(0.004312384281743267),
    static_cast<double>(0.004287042779051156),
    static_cast<double>(0.002960933454428857),
    static_cast<double>(0.004518304240415324),
    static_cast<double>(0.004246093076452513),
    static_cast<double>(0.004312384281743267),
    static_cast<double>(0.0008215802990515499),
    static_cast<double>(0.002960933454428857),
    static_cast<double>(0.004123898038072222),
    static_cast<double>(0.004246093076452513),
    static_cast<double>(-0.0016484577193429048),
    static_cast<double>(0.0008215802990515499)
  );

}

} // namespace servo_motor_ltv_constraints_ltv_mpc_Phi

#endif // __SERVO_MOTOR_LTV_CONSTRAINTS_LTV_MPC_PHI_HPP__
