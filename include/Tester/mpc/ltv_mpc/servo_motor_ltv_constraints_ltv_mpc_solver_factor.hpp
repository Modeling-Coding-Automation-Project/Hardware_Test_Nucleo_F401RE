#ifndef __SERVO_MOTOR_LTV_CONSTRAINTS_LTV_MPC_SOLVER_FACTOR_HPP__
#define __SERVO_MOTOR_LTV_CONSTRAINTS_LTV_MPC_SOLVER_FACTOR_HPP__

#include "python_numpy.hpp"

namespace servo_motor_ltv_constraints_ltv_mpc_solver_factor {

using namespace PythonNumpy;

using SparseAvailable_ltv_mpc_solver_factor = SparseAvailable<
    ColumnAvailable<false, false, false, true, false, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true>,
    ColumnAvailable<false, false, false, true, false, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true>
>;

using type = SparseMatrix_Type<double, SparseAvailable_ltv_mpc_solver_factor>;

inline auto make(void) -> type {

  return make_SparseMatrix<SparseAvailable_ltv_mpc_solver_factor>(
    static_cast<double>(-0.6171258210073843),
    static_cast<double>(-1.372530040802751),
    static_cast<double>(0.012342455543448434),
    static_cast<double>(-2.1068111311062823),
    static_cast<double>(0.051518256775655975),
    static_cast<double>(-2.6856053872170054),
    static_cast<double>(0.13087129240218684),
    static_cast<double>(-3.003448947895235),
    static_cast<double>(0.259968538638338),
    static_cast<double>(-2.9917674710448123),
    static_cast<double>(0.442679637080958),
    static_cast<double>(-2.626372534604199),
    static_cast<double>(0.6760902799569567),
    static_cast<double>(-1.9314765844112662),
    static_cast<double>(0.9503576398636346),
    static_cast<double>(-0.9784672133802994),
    static_cast<double>(1.2495410405698046),
    static_cast<double>(0.12123853848139025),
    static_cast<double>(1.5533345927696194),
    static_cast<double>(1.2289357009836983),
    static_cast<double>(1.8395137834691393),
    static_cast<double>(2.195741503780159),
    static_cast<double>(2.086805492418753),
    static_cast<double>(2.882113278580606),
    static_cast<double>(2.277818063430901),
    static_cast<double>(3.17732870245003),
    static_cast<double>(2.4016380717347374),
    static_cast<double>(3.0163134148393946),
    static_cast<double>(2.455720849539357),
    static_cast<double>(2.391381868931164),
    static_cast<double>(2.4467735310403302),
    static_cast<double>(1.3569826246257766),
    static_cast<double>(2.3904461747802137),
    static_cast<double>(0.026358394407458657),
    static_cast<double>(2.3097956538097444),
    static_cast<double>(-1.439948675023278),
    static_cast<double>(0.16411325350563521),
    static_cast<double>(-0.21734289051299524),
    static_cast<double>(-0.0032822488810750126),
    static_cast<double>(-0.8897639183998535),
    static_cast<double>(-0.002053549757187325),
    static_cast<double>(-1.659484203605033),
    static_cast<double>(0.016908905310290302),
    static_cast<double>(-2.3667555643876224),
    static_cast<double>(0.0681127626722111),
    static_cast<double>(-2.880354162005419),
    static_cast<double>(0.1640913176115266),
    static_cast<double>(-3.102257145159813),
    static_cast<double>(0.3128777675795792),
    static_cast<double>(-2.975422630536903),
    static_cast<double>(0.5162697686214406),
    static_cast<double>(-2.4902571872646213),
    static_cast<double>(0.7690003788691016),
    static_cast<double>(-1.6869926962540267),
    static_cast<double>(1.058899419023808),
    static_cast<double>(-0.6524585249352299),
    static_cast<double>(1.3680432661746884),
    static_cast<double>(0.48915115542905474),
    static_cast<double>(1.6747791033437585),
    static_cast<double>(1.590897957224506),
    static_cast<double>(1.956395249443658),
    static_cast<double>(2.501605029425167),
    static_cast<double>(2.1921128554794627),
    static_cast<double>(3.0860632163894923),
    static_cast<double>(2.366012785528325),
    static_cast<double>(3.244351434300735),
    static_cast<double>(2.4694968020593984),
    static_cast<double>(2.927622041982127),
    static_cast<double>(2.5029199346392335),
    static_cast<double>(2.1480002824860276),
    static_cast<double>(2.4761197667898465),
    static_cast<double>(0.9808857657430731)
  );

}

} // namespace servo_motor_ltv_constraints_ltv_mpc_solver_factor

#endif // __SERVO_MOTOR_LTV_CONSTRAINTS_LTV_MPC_SOLVER_FACTOR_HPP__
