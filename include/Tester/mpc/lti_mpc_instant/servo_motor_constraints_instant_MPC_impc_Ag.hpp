#ifndef SERVO_MOTOR_CONSTRAINTS_INSTANT_MPC_IMPC_AG_HPP_
#define SERVO_MOTOR_CONSTRAINTS_INSTANT_MPC_IMPC_AG_HPP_

#include "python_numpy.hpp"

namespace servo_motor_constraints_instant_MPC_impc_Ag {

using namespace PythonNumpy;

using SparseAvailable_impc_Ag = SparseAvailable<
    ColumnAvailable<
        true, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false>,
    ColumnAvailable<
        true, true, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false>,
    ColumnAvailable<
        true, true, true, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false>,
    ColumnAvailable<
        true, true,
        true, true,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false>,
    ColumnAvailable<
        true, true,
        true, true,
        true, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false>,
    ColumnAvailable<true, true, true, true, true, true, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false>,
    ColumnAvailable<true, true, true, true, true, true, true, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false>,
    ColumnAvailable<true, true, true, true, true, true, true, true, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false>,
    ColumnAvailable<true, true, true, true, true, true, true, true, true, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false>,
    ColumnAvailable<true, true, true, true, true, true, true, true, true, true,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false>,
    ColumnAvailable<true, true, true, true, true, true, true, true, true, true,
                    true, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false>,
    ColumnAvailable<true, true, true, true, true, true, true, true, true, true,
                    true, true, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false>,
    ColumnAvailable<true, true, true, true, true, true, true, true, true, true,
                    true, true, true, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false>,
    ColumnAvailable<true, true, true, true, true, true, true, true, true, true,
                    true, true, true, true, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false>,
    ColumnAvailable<true, true, true, true, true, true, true, true, true, true,
                    true, true, true, true, true, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false>,
    ColumnAvailable<true, true, true, true, true, true, true, true, true, true,
                    true, true, true, true, true, true, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false>,
    ColumnAvailable<true, true, true, true, true, true, true, true, true, true,
                    true, true, true, true, true, true, true, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false>,
    ColumnAvailable<true, true, true, true, true, true, true, true, true, true,
                    true, true, true, true, true, true, true, true, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false>,
    ColumnAvailable<true, true, true, true, true, true, true, true, true, true,
                    true, true, true, true, true, true, true, true, true, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false>,
    ColumnAvailable<true, true, true, true, true, true, true, true, true, true,
                    true, true, true, true, true, true, true, true, true, true,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false>,
    ColumnAvailable<
        true, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false>,
    ColumnAvailable<
        true, true,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false>,
    ColumnAvailable<
        true, true,
        true, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false>,
    ColumnAvailable<
        true, true,
        true, true,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false>,
    ColumnAvailable<
        true, true,
        true, true,
        true, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false>,
    ColumnAvailable<true, true, true, true, true, true, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false>,
    ColumnAvailable<true, true, true, true, true, true, true, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false>,
    ColumnAvailable<true, true, true, true, true, true, true, true, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false>,
    ColumnAvailable<true, true, true, true, true, true, true, true, true, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false>,
    ColumnAvailable<true, true, true, true, true, true, true, true, true, true,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false>,
    ColumnAvailable<true, true, true, true, true, true, true, true, true, true,
                    true, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false>,
    ColumnAvailable<true, true, true, true, true, true, true, true, true, true,
                    true, true, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false>,
    ColumnAvailable<true, true, true, true, true, true, true, true, true, true,
                    true, true, true, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false>,
    ColumnAvailable<true, true, true, true, true, true, true, true, true, true,
                    true, true, true, true, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false>,
    ColumnAvailable<true, true, true, true, true, true, true, true, true, true,
                    true, true, true, true, true, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false>,
    ColumnAvailable<true, true, true, true, true, true, true, true, true, true,
                    true, true, true, true, true, true, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false>,
    ColumnAvailable<true, true, true, true, true, true, true, true, true, true,
                    true, true, true, true, true, true, true, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false>,
    ColumnAvailable<true, true, true, true, true, true, true, true, true, true,
                    true, true, true, true, true, true, true, true, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false>,
    ColumnAvailable<true, true, true, true, true, true, true, true, true, true,
                    true, true, true, true, true, true, true, true, true, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false>,
    ColumnAvailable<true, true, true, true, true, true, true, true, true, true,
                    true, true, true, true, true, true, true, true, true, true,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false>>;

using type = SparseMatrix_Type<float, SparseAvailable_impc_Ag>;

inline auto make(void) -> type {

  auto result = make_SparseMatrixZeros<float, SparseAvailable_impc_Ag>();

  result.unsafe_access(0) = static_cast<float>(1.0);
  result.unsafe_access(1) = static_cast<float>(1.0);
  result.unsafe_access(2) = static_cast<float>(1.0);
  result.unsafe_access(3) = static_cast<float>(1.0);
  result.unsafe_access(4) = static_cast<float>(1.0);
  result.unsafe_access(5) = static_cast<float>(1.0);
  result.unsafe_access(6) = static_cast<float>(1.0);
  result.unsafe_access(7) = static_cast<float>(1.0);
  result.unsafe_access(8) = static_cast<float>(1.0);
  result.unsafe_access(9) = static_cast<float>(1.0);
  result.unsafe_access(10) = static_cast<float>(1.0);
  result.unsafe_access(11) = static_cast<float>(1.0);
  result.unsafe_access(12) = static_cast<float>(1.0);
  result.unsafe_access(13) = static_cast<float>(1.0);
  result.unsafe_access(14) = static_cast<float>(1.0);
  result.unsafe_access(15) = static_cast<float>(1.0);
  result.unsafe_access(16) = static_cast<float>(1.0);
  result.unsafe_access(17) = static_cast<float>(1.0);
  result.unsafe_access(18) = static_cast<float>(1.0);
  result.unsafe_access(19) = static_cast<float>(1.0);
  result.unsafe_access(20) = static_cast<float>(1.0);
  result.unsafe_access(21) = static_cast<float>(1.0);
  result.unsafe_access(22) = static_cast<float>(1.0);
  result.unsafe_access(23) = static_cast<float>(1.0);
  result.unsafe_access(24) = static_cast<float>(1.0);
  result.unsafe_access(25) = static_cast<float>(1.0);
  result.unsafe_access(26) = static_cast<float>(1.0);
  result.unsafe_access(27) = static_cast<float>(1.0);
  result.unsafe_access(28) = static_cast<float>(1.0);
  result.unsafe_access(29) = static_cast<float>(1.0);
  result.unsafe_access(30) = static_cast<float>(1.0);
  result.unsafe_access(31) = static_cast<float>(1.0);
  result.unsafe_access(32) = static_cast<float>(1.0);
  result.unsafe_access(33) = static_cast<float>(1.0);
  result.unsafe_access(34) = static_cast<float>(1.0);
  result.unsafe_access(35) = static_cast<float>(1.0);
  result.unsafe_access(36) = static_cast<float>(1.0);
  result.unsafe_access(37) = static_cast<float>(1.0);
  result.unsafe_access(38) = static_cast<float>(1.0);
  result.unsafe_access(39) = static_cast<float>(1.0);
  result.unsafe_access(40) = static_cast<float>(1.0);
  result.unsafe_access(41) = static_cast<float>(1.0);
  result.unsafe_access(42) = static_cast<float>(1.0);
  result.unsafe_access(43) = static_cast<float>(1.0);
  result.unsafe_access(44) = static_cast<float>(1.0);
  result.unsafe_access(45) = static_cast<float>(1.0);
  result.unsafe_access(46) = static_cast<float>(1.0);
  result.unsafe_access(47) = static_cast<float>(1.0);
  result.unsafe_access(48) = static_cast<float>(1.0);
  result.unsafe_access(49) = static_cast<float>(1.0);
  result.unsafe_access(50) = static_cast<float>(1.0);
  result.unsafe_access(51) = static_cast<float>(1.0);
  result.unsafe_access(52) = static_cast<float>(1.0);
  result.unsafe_access(53) = static_cast<float>(1.0);
  result.unsafe_access(54) = static_cast<float>(1.0);
  result.unsafe_access(55) = static_cast<float>(1.0);
  result.unsafe_access(56) = static_cast<float>(1.0);
  result.unsafe_access(57) = static_cast<float>(1.0);
  result.unsafe_access(58) = static_cast<float>(1.0);
  result.unsafe_access(59) = static_cast<float>(1.0);
  result.unsafe_access(60) = static_cast<float>(1.0);
  result.unsafe_access(61) = static_cast<float>(1.0);
  result.unsafe_access(62) = static_cast<float>(1.0);
  result.unsafe_access(63) = static_cast<float>(1.0);
  result.unsafe_access(64) = static_cast<float>(1.0);
  result.unsafe_access(65) = static_cast<float>(1.0);
  result.unsafe_access(66) = static_cast<float>(1.0);
  result.unsafe_access(67) = static_cast<float>(1.0);
  result.unsafe_access(68) = static_cast<float>(1.0);
  result.unsafe_access(69) = static_cast<float>(1.0);
  result.unsafe_access(70) = static_cast<float>(1.0);
  result.unsafe_access(71) = static_cast<float>(1.0);
  result.unsafe_access(72) = static_cast<float>(1.0);
  result.unsafe_access(73) = static_cast<float>(1.0);
  result.unsafe_access(74) = static_cast<float>(1.0);
  result.unsafe_access(75) = static_cast<float>(1.0);
  result.unsafe_access(76) = static_cast<float>(1.0);
  result.unsafe_access(77) = static_cast<float>(1.0);
  result.unsafe_access(78) = static_cast<float>(1.0);
  result.unsafe_access(79) = static_cast<float>(1.0);
  result.unsafe_access(80) = static_cast<float>(1.0);
  result.unsafe_access(81) = static_cast<float>(1.0);
  result.unsafe_access(82) = static_cast<float>(1.0);
  result.unsafe_access(83) = static_cast<float>(1.0);
  result.unsafe_access(84) = static_cast<float>(1.0);
  result.unsafe_access(85) = static_cast<float>(1.0);
  result.unsafe_access(86) = static_cast<float>(1.0);
  result.unsafe_access(87) = static_cast<float>(1.0);
  result.unsafe_access(88) = static_cast<float>(1.0);
  result.unsafe_access(89) = static_cast<float>(1.0);
  result.unsafe_access(90) = static_cast<float>(1.0);
  result.unsafe_access(91) = static_cast<float>(1.0);
  result.unsafe_access(92) = static_cast<float>(1.0);
  result.unsafe_access(93) = static_cast<float>(1.0);
  result.unsafe_access(94) = static_cast<float>(1.0);
  result.unsafe_access(95) = static_cast<float>(1.0);
  result.unsafe_access(96) = static_cast<float>(1.0);
  result.unsafe_access(97) = static_cast<float>(1.0);
  result.unsafe_access(98) = static_cast<float>(1.0);
  result.unsafe_access(99) = static_cast<float>(1.0);
  result.unsafe_access(100) = static_cast<float>(1.0);
  result.unsafe_access(101) = static_cast<float>(1.0);
  result.unsafe_access(102) = static_cast<float>(1.0);
  result.unsafe_access(103) = static_cast<float>(1.0);
  result.unsafe_access(104) = static_cast<float>(1.0);
  result.unsafe_access(105) = static_cast<float>(1.0);
  result.unsafe_access(106) = static_cast<float>(1.0);
  result.unsafe_access(107) = static_cast<float>(1.0);
  result.unsafe_access(108) = static_cast<float>(1.0);
  result.unsafe_access(109) = static_cast<float>(1.0);
  result.unsafe_access(110) = static_cast<float>(1.0);
  result.unsafe_access(111) = static_cast<float>(1.0);
  result.unsafe_access(112) = static_cast<float>(1.0);
  result.unsafe_access(113) = static_cast<float>(1.0);
  result.unsafe_access(114) = static_cast<float>(1.0);
  result.unsafe_access(115) = static_cast<float>(1.0);
  result.unsafe_access(116) = static_cast<float>(1.0);
  result.unsafe_access(117) = static_cast<float>(1.0);
  result.unsafe_access(118) = static_cast<float>(1.0);
  result.unsafe_access(119) = static_cast<float>(1.0);
  result.unsafe_access(120) = static_cast<float>(1.0);
  result.unsafe_access(121) = static_cast<float>(1.0);
  result.unsafe_access(122) = static_cast<float>(1.0);
  result.unsafe_access(123) = static_cast<float>(1.0);
  result.unsafe_access(124) = static_cast<float>(1.0);
  result.unsafe_access(125) = static_cast<float>(1.0);
  result.unsafe_access(126) = static_cast<float>(1.0);
  result.unsafe_access(127) = static_cast<float>(1.0);
  result.unsafe_access(128) = static_cast<float>(1.0);
  result.unsafe_access(129) = static_cast<float>(1.0);
  result.unsafe_access(130) = static_cast<float>(1.0);
  result.unsafe_access(131) = static_cast<float>(1.0);
  result.unsafe_access(132) = static_cast<float>(1.0);
  result.unsafe_access(133) = static_cast<float>(1.0);
  result.unsafe_access(134) = static_cast<float>(1.0);
  result.unsafe_access(135) = static_cast<float>(1.0);
  result.unsafe_access(136) = static_cast<float>(1.0);
  result.unsafe_access(137) = static_cast<float>(1.0);
  result.unsafe_access(138) = static_cast<float>(1.0);
  result.unsafe_access(139) = static_cast<float>(1.0);
  result.unsafe_access(140) = static_cast<float>(1.0);
  result.unsafe_access(141) = static_cast<float>(1.0);
  result.unsafe_access(142) = static_cast<float>(1.0);
  result.unsafe_access(143) = static_cast<float>(1.0);
  result.unsafe_access(144) = static_cast<float>(1.0);
  result.unsafe_access(145) = static_cast<float>(1.0);
  result.unsafe_access(146) = static_cast<float>(1.0);
  result.unsafe_access(147) = static_cast<float>(1.0);
  result.unsafe_access(148) = static_cast<float>(1.0);
  result.unsafe_access(149) = static_cast<float>(1.0);
  result.unsafe_access(150) = static_cast<float>(1.0);
  result.unsafe_access(151) = static_cast<float>(1.0);
  result.unsafe_access(152) = static_cast<float>(1.0);
  result.unsafe_access(153) = static_cast<float>(1.0);
  result.unsafe_access(154) = static_cast<float>(1.0);
  result.unsafe_access(155) = static_cast<float>(1.0);
  result.unsafe_access(156) = static_cast<float>(1.0);
  result.unsafe_access(157) = static_cast<float>(1.0);
  result.unsafe_access(158) = static_cast<float>(1.0);
  result.unsafe_access(159) = static_cast<float>(1.0);
  result.unsafe_access(160) = static_cast<float>(1.0);
  result.unsafe_access(161) = static_cast<float>(1.0);
  result.unsafe_access(162) = static_cast<float>(1.0);
  result.unsafe_access(163) = static_cast<float>(1.0);
  result.unsafe_access(164) = static_cast<float>(1.0);
  result.unsafe_access(165) = static_cast<float>(1.0);
  result.unsafe_access(166) = static_cast<float>(1.0);
  result.unsafe_access(167) = static_cast<float>(1.0);
  result.unsafe_access(168) = static_cast<float>(1.0);
  result.unsafe_access(169) = static_cast<float>(1.0);
  result.unsafe_access(170) = static_cast<float>(1.0);
  result.unsafe_access(171) = static_cast<float>(1.0);
  result.unsafe_access(172) = static_cast<float>(1.0);
  result.unsafe_access(173) = static_cast<float>(1.0);
  result.unsafe_access(174) = static_cast<float>(1.0);
  result.unsafe_access(175) = static_cast<float>(1.0);
  result.unsafe_access(176) = static_cast<float>(1.0);
  result.unsafe_access(177) = static_cast<float>(1.0);
  result.unsafe_access(178) = static_cast<float>(1.0);
  result.unsafe_access(179) = static_cast<float>(1.0);
  result.unsafe_access(180) = static_cast<float>(1.0);
  result.unsafe_access(181) = static_cast<float>(1.0);
  result.unsafe_access(182) = static_cast<float>(1.0);
  result.unsafe_access(183) = static_cast<float>(1.0);
  result.unsafe_access(184) = static_cast<float>(1.0);
  result.unsafe_access(185) = static_cast<float>(1.0);
  result.unsafe_access(186) = static_cast<float>(1.0);
  result.unsafe_access(187) = static_cast<float>(1.0);
  result.unsafe_access(188) = static_cast<float>(1.0);
  result.unsafe_access(189) = static_cast<float>(1.0);
  result.unsafe_access(190) = static_cast<float>(1.0);
  result.unsafe_access(191) = static_cast<float>(1.0);
  result.unsafe_access(192) = static_cast<float>(1.0);
  result.unsafe_access(193) = static_cast<float>(1.0);
  result.unsafe_access(194) = static_cast<float>(1.0);
  result.unsafe_access(195) = static_cast<float>(1.0);
  result.unsafe_access(196) = static_cast<float>(1.0);
  result.unsafe_access(197) = static_cast<float>(1.0);
  result.unsafe_access(198) = static_cast<float>(1.0);
  result.unsafe_access(199) = static_cast<float>(1.0);
  result.unsafe_access(200) = static_cast<float>(1.0);
  result.unsafe_access(201) = static_cast<float>(1.0);
  result.unsafe_access(202) = static_cast<float>(1.0);
  result.unsafe_access(203) = static_cast<float>(1.0);
  result.unsafe_access(204) = static_cast<float>(1.0);
  result.unsafe_access(205) = static_cast<float>(1.0);
  result.unsafe_access(206) = static_cast<float>(1.0);
  result.unsafe_access(207) = static_cast<float>(1.0);
  result.unsafe_access(208) = static_cast<float>(1.0);
  result.unsafe_access(209) = static_cast<float>(1.0);
  result.unsafe_access(210) = static_cast<float>(-1.0);
  result.unsafe_access(211) = static_cast<float>(-1.0);
  result.unsafe_access(212) = static_cast<float>(-1.0);
  result.unsafe_access(213) = static_cast<float>(-1.0);
  result.unsafe_access(214) = static_cast<float>(-1.0);
  result.unsafe_access(215) = static_cast<float>(-1.0);
  result.unsafe_access(216) = static_cast<float>(-1.0);
  result.unsafe_access(217) = static_cast<float>(-1.0);
  result.unsafe_access(218) = static_cast<float>(-1.0);
  result.unsafe_access(219) = static_cast<float>(-1.0);
  result.unsafe_access(220) = static_cast<float>(-1.0);
  result.unsafe_access(221) = static_cast<float>(-1.0);
  result.unsafe_access(222) = static_cast<float>(-1.0);
  result.unsafe_access(223) = static_cast<float>(-1.0);
  result.unsafe_access(224) = static_cast<float>(-1.0);
  result.unsafe_access(225) = static_cast<float>(-1.0);
  result.unsafe_access(226) = static_cast<float>(-1.0);
  result.unsafe_access(227) = static_cast<float>(-1.0);
  result.unsafe_access(228) = static_cast<float>(-1.0);
  result.unsafe_access(229) = static_cast<float>(-1.0);
  result.unsafe_access(230) = static_cast<float>(-1.0);
  result.unsafe_access(231) = static_cast<float>(-1.0);
  result.unsafe_access(232) = static_cast<float>(-1.0);
  result.unsafe_access(233) = static_cast<float>(-1.0);
  result.unsafe_access(234) = static_cast<float>(-1.0);
  result.unsafe_access(235) = static_cast<float>(-1.0);
  result.unsafe_access(236) = static_cast<float>(-1.0);
  result.unsafe_access(237) = static_cast<float>(-1.0);
  result.unsafe_access(238) = static_cast<float>(-1.0);
  result.unsafe_access(239) = static_cast<float>(-1.0);
  result.unsafe_access(240) = static_cast<float>(-1.0);
  result.unsafe_access(241) = static_cast<float>(-1.0);
  result.unsafe_access(242) = static_cast<float>(-1.0);
  result.unsafe_access(243) = static_cast<float>(-1.0);
  result.unsafe_access(244) = static_cast<float>(-1.0);
  result.unsafe_access(245) = static_cast<float>(-1.0);
  result.unsafe_access(246) = static_cast<float>(-1.0);
  result.unsafe_access(247) = static_cast<float>(-1.0);
  result.unsafe_access(248) = static_cast<float>(-1.0);
  result.unsafe_access(249) = static_cast<float>(-1.0);
  result.unsafe_access(250) = static_cast<float>(-1.0);
  result.unsafe_access(251) = static_cast<float>(-1.0);
  result.unsafe_access(252) = static_cast<float>(-1.0);
  result.unsafe_access(253) = static_cast<float>(-1.0);
  result.unsafe_access(254) = static_cast<float>(-1.0);
  result.unsafe_access(255) = static_cast<float>(-1.0);
  result.unsafe_access(256) = static_cast<float>(-1.0);
  result.unsafe_access(257) = static_cast<float>(-1.0);
  result.unsafe_access(258) = static_cast<float>(-1.0);
  result.unsafe_access(259) = static_cast<float>(-1.0);
  result.unsafe_access(260) = static_cast<float>(-1.0);
  result.unsafe_access(261) = static_cast<float>(-1.0);
  result.unsafe_access(262) = static_cast<float>(-1.0);
  result.unsafe_access(263) = static_cast<float>(-1.0);
  result.unsafe_access(264) = static_cast<float>(-1.0);
  result.unsafe_access(265) = static_cast<float>(-1.0);
  result.unsafe_access(266) = static_cast<float>(-1.0);
  result.unsafe_access(267) = static_cast<float>(-1.0);
  result.unsafe_access(268) = static_cast<float>(-1.0);
  result.unsafe_access(269) = static_cast<float>(-1.0);
  result.unsafe_access(270) = static_cast<float>(-1.0);
  result.unsafe_access(271) = static_cast<float>(-1.0);
  result.unsafe_access(272) = static_cast<float>(-1.0);
  result.unsafe_access(273) = static_cast<float>(-1.0);
  result.unsafe_access(274) = static_cast<float>(-1.0);
  result.unsafe_access(275) = static_cast<float>(-1.0);
  result.unsafe_access(276) = static_cast<float>(-1.0);
  result.unsafe_access(277) = static_cast<float>(-1.0);
  result.unsafe_access(278) = static_cast<float>(-1.0);
  result.unsafe_access(279) = static_cast<float>(-1.0);
  result.unsafe_access(280) = static_cast<float>(-1.0);
  result.unsafe_access(281) = static_cast<float>(-1.0);
  result.unsafe_access(282) = static_cast<float>(-1.0);
  result.unsafe_access(283) = static_cast<float>(-1.0);
  result.unsafe_access(284) = static_cast<float>(-1.0);
  result.unsafe_access(285) = static_cast<float>(-1.0);
  result.unsafe_access(286) = static_cast<float>(-1.0);
  result.unsafe_access(287) = static_cast<float>(-1.0);
  result.unsafe_access(288) = static_cast<float>(-1.0);
  result.unsafe_access(289) = static_cast<float>(-1.0);
  result.unsafe_access(290) = static_cast<float>(-1.0);
  result.unsafe_access(291) = static_cast<float>(-1.0);
  result.unsafe_access(292) = static_cast<float>(-1.0);
  result.unsafe_access(293) = static_cast<float>(-1.0);
  result.unsafe_access(294) = static_cast<float>(-1.0);
  result.unsafe_access(295) = static_cast<float>(-1.0);
  result.unsafe_access(296) = static_cast<float>(-1.0);
  result.unsafe_access(297) = static_cast<float>(-1.0);
  result.unsafe_access(298) = static_cast<float>(-1.0);
  result.unsafe_access(299) = static_cast<float>(-1.0);
  result.unsafe_access(300) = static_cast<float>(-1.0);
  result.unsafe_access(301) = static_cast<float>(-1.0);
  result.unsafe_access(302) = static_cast<float>(-1.0);
  result.unsafe_access(303) = static_cast<float>(-1.0);
  result.unsafe_access(304) = static_cast<float>(-1.0);
  result.unsafe_access(305) = static_cast<float>(-1.0);
  result.unsafe_access(306) = static_cast<float>(-1.0);
  result.unsafe_access(307) = static_cast<float>(-1.0);
  result.unsafe_access(308) = static_cast<float>(-1.0);
  result.unsafe_access(309) = static_cast<float>(-1.0);
  result.unsafe_access(310) = static_cast<float>(-1.0);
  result.unsafe_access(311) = static_cast<float>(-1.0);
  result.unsafe_access(312) = static_cast<float>(-1.0);
  result.unsafe_access(313) = static_cast<float>(-1.0);
  result.unsafe_access(314) = static_cast<float>(-1.0);
  result.unsafe_access(315) = static_cast<float>(-1.0);
  result.unsafe_access(316) = static_cast<float>(-1.0);
  result.unsafe_access(317) = static_cast<float>(-1.0);
  result.unsafe_access(318) = static_cast<float>(-1.0);
  result.unsafe_access(319) = static_cast<float>(-1.0);
  result.unsafe_access(320) = static_cast<float>(-1.0);
  result.unsafe_access(321) = static_cast<float>(-1.0);
  result.unsafe_access(322) = static_cast<float>(-1.0);
  result.unsafe_access(323) = static_cast<float>(-1.0);
  result.unsafe_access(324) = static_cast<float>(-1.0);
  result.unsafe_access(325) = static_cast<float>(-1.0);
  result.unsafe_access(326) = static_cast<float>(-1.0);
  result.unsafe_access(327) = static_cast<float>(-1.0);
  result.unsafe_access(328) = static_cast<float>(-1.0);
  result.unsafe_access(329) = static_cast<float>(-1.0);
  result.unsafe_access(330) = static_cast<float>(-1.0);
  result.unsafe_access(331) = static_cast<float>(-1.0);
  result.unsafe_access(332) = static_cast<float>(-1.0);
  result.unsafe_access(333) = static_cast<float>(-1.0);
  result.unsafe_access(334) = static_cast<float>(-1.0);
  result.unsafe_access(335) = static_cast<float>(-1.0);
  result.unsafe_access(336) = static_cast<float>(-1.0);
  result.unsafe_access(337) = static_cast<float>(-1.0);
  result.unsafe_access(338) = static_cast<float>(-1.0);
  result.unsafe_access(339) = static_cast<float>(-1.0);
  result.unsafe_access(340) = static_cast<float>(-1.0);
  result.unsafe_access(341) = static_cast<float>(-1.0);
  result.unsafe_access(342) = static_cast<float>(-1.0);
  result.unsafe_access(343) = static_cast<float>(-1.0);
  result.unsafe_access(344) = static_cast<float>(-1.0);
  result.unsafe_access(345) = static_cast<float>(-1.0);
  result.unsafe_access(346) = static_cast<float>(-1.0);
  result.unsafe_access(347) = static_cast<float>(-1.0);
  result.unsafe_access(348) = static_cast<float>(-1.0);
  result.unsafe_access(349) = static_cast<float>(-1.0);
  result.unsafe_access(350) = static_cast<float>(-1.0);
  result.unsafe_access(351) = static_cast<float>(-1.0);
  result.unsafe_access(352) = static_cast<float>(-1.0);
  result.unsafe_access(353) = static_cast<float>(-1.0);
  result.unsafe_access(354) = static_cast<float>(-1.0);
  result.unsafe_access(355) = static_cast<float>(-1.0);
  result.unsafe_access(356) = static_cast<float>(-1.0);
  result.unsafe_access(357) = static_cast<float>(-1.0);
  result.unsafe_access(358) = static_cast<float>(-1.0);
  result.unsafe_access(359) = static_cast<float>(-1.0);
  result.unsafe_access(360) = static_cast<float>(-1.0);
  result.unsafe_access(361) = static_cast<float>(-1.0);
  result.unsafe_access(362) = static_cast<float>(-1.0);
  result.unsafe_access(363) = static_cast<float>(-1.0);
  result.unsafe_access(364) = static_cast<float>(-1.0);
  result.unsafe_access(365) = static_cast<float>(-1.0);
  result.unsafe_access(366) = static_cast<float>(-1.0);
  result.unsafe_access(367) = static_cast<float>(-1.0);
  result.unsafe_access(368) = static_cast<float>(-1.0);
  result.unsafe_access(369) = static_cast<float>(-1.0);
  result.unsafe_access(370) = static_cast<float>(-1.0);
  result.unsafe_access(371) = static_cast<float>(-1.0);
  result.unsafe_access(372) = static_cast<float>(-1.0);
  result.unsafe_access(373) = static_cast<float>(-1.0);
  result.unsafe_access(374) = static_cast<float>(-1.0);
  result.unsafe_access(375) = static_cast<float>(-1.0);
  result.unsafe_access(376) = static_cast<float>(-1.0);
  result.unsafe_access(377) = static_cast<float>(-1.0);
  result.unsafe_access(378) = static_cast<float>(-1.0);
  result.unsafe_access(379) = static_cast<float>(-1.0);
  result.unsafe_access(380) = static_cast<float>(-1.0);
  result.unsafe_access(381) = static_cast<float>(-1.0);
  result.unsafe_access(382) = static_cast<float>(-1.0);
  result.unsafe_access(383) = static_cast<float>(-1.0);
  result.unsafe_access(384) = static_cast<float>(-1.0);
  result.unsafe_access(385) = static_cast<float>(-1.0);
  result.unsafe_access(386) = static_cast<float>(-1.0);
  result.unsafe_access(387) = static_cast<float>(-1.0);
  result.unsafe_access(388) = static_cast<float>(-1.0);
  result.unsafe_access(389) = static_cast<float>(-1.0);
  result.unsafe_access(390) = static_cast<float>(-1.0);
  result.unsafe_access(391) = static_cast<float>(-1.0);
  result.unsafe_access(392) = static_cast<float>(-1.0);
  result.unsafe_access(393) = static_cast<float>(-1.0);
  result.unsafe_access(394) = static_cast<float>(-1.0);
  result.unsafe_access(395) = static_cast<float>(-1.0);
  result.unsafe_access(396) = static_cast<float>(-1.0);
  result.unsafe_access(397) = static_cast<float>(-1.0);
  result.unsafe_access(398) = static_cast<float>(-1.0);
  result.unsafe_access(399) = static_cast<float>(-1.0);
  result.unsafe_access(400) = static_cast<float>(-1.0);
  result.unsafe_access(401) = static_cast<float>(-1.0);
  result.unsafe_access(402) = static_cast<float>(-1.0);
  result.unsafe_access(403) = static_cast<float>(-1.0);
  result.unsafe_access(404) = static_cast<float>(-1.0);
  result.unsafe_access(405) = static_cast<float>(-1.0);
  result.unsafe_access(406) = static_cast<float>(-1.0);
  result.unsafe_access(407) = static_cast<float>(-1.0);
  result.unsafe_access(408) = static_cast<float>(-1.0);
  result.unsafe_access(409) = static_cast<float>(-1.0);
  result.unsafe_access(410) = static_cast<float>(-1.0);
  result.unsafe_access(411) = static_cast<float>(-1.0);
  result.unsafe_access(412) = static_cast<float>(-1.0);
  result.unsafe_access(413) = static_cast<float>(-1.0);
  result.unsafe_access(414) = static_cast<float>(-1.0);
  result.unsafe_access(415) = static_cast<float>(-1.0);
  result.unsafe_access(416) = static_cast<float>(-1.0);
  result.unsafe_access(417) = static_cast<float>(-1.0);
  result.unsafe_access(418) = static_cast<float>(-1.0);
  result.unsafe_access(419) = static_cast<float>(-1.0);

  return result;
}

} // namespace servo_motor_constraints_instant_MPC_impc_Ag

#endif // SERVO_MOTOR_CONSTRAINTS_INSTANT_MPC_IMPC_AG_HPP_
