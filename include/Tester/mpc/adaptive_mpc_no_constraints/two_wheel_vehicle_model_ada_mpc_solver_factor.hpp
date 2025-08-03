#ifndef __TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_SOLVER_FACTOR_HPP__
#define __TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_SOLVER_FACTOR_HPP__

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

  return make_SparseMatrix<SparseAvailable_ada_mpc_solver_factor>(
      static_cast<float>(0.007925373319758794), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(-0.0),
      static_cast<float>(0.0003962686659879397),
      static_cast<float>(0.01565048377768917), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(0.0007925373319758794),
      static_cast<float>(0.0011787928548723983),
      static_cast<float>(0.023180087525278097), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(0.0031501230417206763),
      static_cast<float>(0.0023377972311363036),
      static_cast<float>(0.030518835390228326), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(0.007825717503993284),
      static_cast<float>(0.0038637390006477203),
      static_cast<float>(0.03767127500507197), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(0.015553195505288723),
      static_cast<float>(0.005747302750901319),
      static_cast<float>(0.04464185289859784), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(0.02704780100709136),
      static_cast<float>(0.00797939539583121),
      static_cast<float>(0.05143491655055928), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(0.043006591798753775),
      static_cast<float>(0.010551141223359175),
      static_cast<float>(0.05805471641012983), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(0.06410887424547212),
      static_cast<float>(0.013453877043865666),
      static_cast<float>(0.06450540787857448), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(0.09101662833320347),
      static_cast<float>(0.01667914743779439),
      static_cast<float>(0.07079105325660488), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(0.12437492320879223),
      static_cast<float>(0.020218700100624636),
      static_cast<float>(0.07691562365688585), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(0.16481232341004148),
      static_cast<float>(0.024064481283468924),
      static_cast<float>(0.08288300088216105), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(0.21294128597697934),
      static_cast<float>(0.02820863132757698),
      static_cast<float>(0.08869697926946392), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(0.2693585486321333),
      static_cast<float>(0.03264348029105017),
      static_cast<float>(0.09436126750088022), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(0.33464550921423364),
      static_cast<float>(0.037361543666094184),
      static_cast<float>(0.09987949038132593), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(0.40936859654642205),
      static_cast<float>(0.042355518185160486),
      static_cast<float>(0.10525519058380371), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(0.03998066854714409),
      static_cast<float>(0.00039980668547144095), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(-0.0),
      static_cast<float>(0.07996133709428818),
      static_cast<float>(0.0011994200564143227), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(-0.0),
      static_cast<float>(0.11994200564143227),
      static_cast<float>(0.002398840112828646), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(-0.0),
      static_cast<float>(0.15992267418857636),
      static_cast<float>(0.003998066854714408), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(-0.0),
      static_cast<float>(0.19990334273572047),
      static_cast<float>(0.005997100282071614), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(-0.0),
      static_cast<float>(0.23988401128286457),
      static_cast<float>(0.00839594039490026), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(-0.0),
      static_cast<float>(0.27986467983000857),
      static_cast<float>(0.011194587193200347), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(-0.0),
      static_cast<float>(0.3198453483771527),
      static_cast<float>(0.014393040676971874), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(-0.0),
      static_cast<float>(0.3598260169242968),
      static_cast<float>(0.017991300846214843), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(-0.0),
      static_cast<float>(0.3998066854714409),
      static_cast<float>(0.02198936770092925), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(-0.0),
      static_cast<float>(0.439787354018585),
      static_cast<float>(0.026387241241115102), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(-0.0),
      static_cast<float>(0.4797680225657291),
      static_cast<float>(0.031184921466772392), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(-0.0),
      static_cast<float>(0.5197486911128733),
      static_cast<float>(0.03638240837790112), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(-0.0),
      static_cast<float>(0.5597293596600172),
      static_cast<float>(0.04197970197450129), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(-0.0),
      static_cast<float>(0.5997100282071615),
      static_cast<float>(0.047976802256572915), static_cast<float>(-0.0),
      static_cast<float>(-0.0), static_cast<float>(-0.0),
      static_cast<float>(0.6396906967543055));
}

} // namespace two_wheel_vehicle_model_ada_mpc_solver_factor

#endif // __TWO_WHEEL_VEHICLE_MODEL_ADA_MPC_SOLVER_FACTOR_HPP__
