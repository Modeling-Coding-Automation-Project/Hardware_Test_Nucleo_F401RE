#ifndef TWO_WHEEL_VEHICLE_MODEL_MPC_EMBEDDED_INTEGRATOR_STATE_SPACE_UPDATER_HPP_
#define TWO_WHEEL_VEHICLE_MODEL_MPC_EMBEDDED_INTEGRATOR_STATE_SPACE_UPDATER_HPP_

#include "python_math.hpp"
#include "python_numpy.hpp"

namespace two_wheel_vehicle_model_mpc_embedded_integrator_state_space_updater {

using namespace PythonMath;
using namespace PythonNumpy;

template <typename A_Updater_Output_Type> class A_Updater {
public:
  static inline auto update(float m, float l_f, float l_r, float I, float K_f,
                            float K_r, float theta, float beta, float px,
                            float delta, float V, float accel, float r,
                            float py) -> A_Updater_Output_Type {
    static_cast<void>(px);
    static_cast<void>(accel);
    static_cast<void>(py);

    return A_Updater::sympy_function(K_r, m, beta, r, I, V, l_r, theta, delta,
                                     K_f, l_f);
  }

  static inline auto sympy_function(float K_r, float m, float beta, float r,
                                    float I, float V, float l_r, float theta,
                                    float delta, float K_f, float l_f)
      -> A_Updater_Output_Type {
    A_Updater_Output_Type result;

    float x0 = 0.01 * PythonMath::sin(theta);

    float x1 = -V * x0;

    float x2 = 0.01 * PythonMath::cos(theta);

    float x3 = V * x2;

    float x4 = 1 / I;

    float x5 = 1 / V;

    float x6 = K_f * (l_f * l_f);

    float x7 = K_r * (l_r * l_r);

    float x8 = x4 * x5 * (I * V - 0.02 * x6 - 0.02 * x7);

    float x9 = K_f * l_f;

    float x10 = x4 * (K_r * l_r - x9);

    float x11 = V * V;

    float x12 = 1 / x11;

    float x13 = r * x12 * x4 * (x6 + x7);

    float x14 = 1 / m;

    float x15 = 0.02 * K_f;

    float x16 = 0.02 * K_r;

    float x17 = V * beta;

    float x18 = 0.04 * r;

    result.template set<0, 0>(static_cast<float>(1));
    result.template set<0, 1>(static_cast<float>(0));
    result.template set<0, 2>(static_cast<float>(x1));
    result.template set<0, 3>(static_cast<float>(0));
    result.template set<0, 4>(static_cast<float>(0));
    result.template set<0, 5>(static_cast<float>(x2));
    result.template set<0, 6>(static_cast<float>(0.0));
    result.template set<0, 7>(static_cast<float>(0.0));
    result.template set<0, 8>(static_cast<float>(0.0));
    result.template set<0, 9>(static_cast<float>(0.0));
    result.template set<0, 10>(static_cast<float>(0.0));
    result.template set<1, 0>(static_cast<float>(0));
    result.template set<1, 1>(static_cast<float>(1));
    result.template set<1, 2>(static_cast<float>(x3));
    result.template set<1, 3>(static_cast<float>(0));
    result.template set<1, 4>(static_cast<float>(0));
    result.template set<1, 5>(static_cast<float>(x0));
    result.template set<1, 6>(static_cast<float>(0.0));
    result.template set<1, 7>(static_cast<float>(0.0));
    result.template set<1, 8>(static_cast<float>(0.0));
    result.template set<1, 9>(static_cast<float>(0.0));
    result.template set<1, 10>(static_cast<float>(0.0));
    result.template set<2, 0>(static_cast<float>(0));
    result.template set<2, 1>(static_cast<float>(0));
    result.template set<2, 2>(static_cast<float>(1));
    result.template set<2, 3>(static_cast<float>(0.01));
    result.template set<2, 4>(static_cast<float>(0));
    result.template set<2, 5>(static_cast<float>(0));
    result.template set<2, 6>(static_cast<float>(0.0));
    result.template set<2, 7>(static_cast<float>(0.0));
    result.template set<2, 8>(static_cast<float>(0.0));
    result.template set<2, 9>(static_cast<float>(0.0));
    result.template set<2, 10>(static_cast<float>(0.0));
    result.template set<3, 0>(static_cast<float>(0));
    result.template set<3, 1>(static_cast<float>(0));
    result.template set<3, 2>(static_cast<float>(0));
    result.template set<3, 3>(static_cast<float>(x8));
    result.template set<3, 4>(static_cast<float>(0.02 * x10));
    result.template set<3, 5>(static_cast<float>(0.02 * x13));
    result.template set<3, 6>(static_cast<float>(0.0));
    result.template set<3, 7>(static_cast<float>(0.0));
    result.template set<3, 8>(static_cast<float>(0.0));
    result.template set<3, 9>(static_cast<float>(0.0));
    result.template set<3, 10>(static_cast<float>(0.0));
    result.template set<4, 0>(static_cast<float>(0));
    result.template set<4, 1>(static_cast<float>(0));
    result.template set<4, 2>(static_cast<float>(0));
    result.template set<4, 3>(static_cast<float>(
        x12 * x14 * (0.02 * K_r * l_r - 0.01 * m * x11 - 0.02 * x9)));
    result.template set<4, 4>(
        static_cast<float>(x14 * x5 * (V * m - x15 - x16)));
    result.template set<4, 5>(
        static_cast<float>(1.0 * x14 *
                           (-K_r * l_r * x18 - V * delta * x15 + x15 * x17 +
                            x16 * x17 + x18 * x9) /
                           (V * V * V)));
    result.template set<4, 6>(static_cast<float>(0.0));
    result.template set<4, 7>(static_cast<float>(0.0));
    result.template set<4, 8>(static_cast<float>(0.0));
    result.template set<4, 9>(static_cast<float>(0.0));
    result.template set<4, 10>(static_cast<float>(0.0));
    result.template set<5, 0>(static_cast<float>(0));
    result.template set<5, 1>(static_cast<float>(0));
    result.template set<5, 2>(static_cast<float>(0));
    result.template set<5, 3>(static_cast<float>(0));
    result.template set<5, 4>(static_cast<float>(0));
    result.template set<5, 5>(static_cast<float>(1));
    result.template set<5, 6>(static_cast<float>(0.0));
    result.template set<5, 7>(static_cast<float>(0.0));
    result.template set<5, 8>(static_cast<float>(0.0));
    result.template set<5, 9>(static_cast<float>(0.0));
    result.template set<5, 10>(static_cast<float>(0.0));
    result.template set<6, 0>(static_cast<float>(1.0));
    result.template set<6, 1>(static_cast<float>(0));
    result.template set<6, 2>(static_cast<float>(x1));
    result.template set<6, 3>(static_cast<float>(0));
    result.template set<6, 4>(static_cast<float>(0));
    result.template set<6, 5>(static_cast<float>(x2));
    result.template set<6, 6>(static_cast<float>(1));
    result.template set<6, 7>(static_cast<float>(0));
    result.template set<6, 8>(static_cast<float>(0));
    result.template set<6, 9>(static_cast<float>(0));
    result.template set<6, 10>(static_cast<float>(0));
    result.template set<7, 0>(static_cast<float>(0));
    result.template set<7, 1>(static_cast<float>(1.0));
    result.template set<7, 2>(static_cast<float>(x3));
    result.template set<7, 3>(static_cast<float>(0));
    result.template set<7, 4>(static_cast<float>(0));
    result.template set<7, 5>(static_cast<float>(x0));
    result.template set<7, 6>(static_cast<float>(0));
    result.template set<7, 7>(static_cast<float>(1));
    result.template set<7, 8>(static_cast<float>(0));
    result.template set<7, 9>(static_cast<float>(0));
    result.template set<7, 10>(static_cast<float>(0));
    result.template set<8, 0>(static_cast<float>(0));
    result.template set<8, 1>(static_cast<float>(0));
    result.template set<8, 2>(static_cast<float>(0.05));
    result.template set<8, 3>(static_cast<float>(0.0005));
    result.template set<8, 4>(static_cast<float>(0));
    result.template set<8, 5>(static_cast<float>(0));
    result.template set<8, 6>(static_cast<float>(0));
    result.template set<8, 7>(static_cast<float>(0));
    result.template set<8, 8>(static_cast<float>(1));
    result.template set<8, 9>(static_cast<float>(0));
    result.template set<8, 10>(static_cast<float>(0));
    result.template set<9, 0>(static_cast<float>(0));
    result.template set<9, 1>(static_cast<float>(0));
    result.template set<9, 2>(static_cast<float>(0));
    result.template set<9, 3>(static_cast<float>(0.01 * x8));
    result.template set<9, 4>(static_cast<float>(0.0002 * x10));
    result.template set<9, 5>(static_cast<float>(0.0002 * x13));
    result.template set<9, 6>(static_cast<float>(0));
    result.template set<9, 7>(static_cast<float>(0));
    result.template set<9, 8>(static_cast<float>(0));
    result.template set<9, 9>(static_cast<float>(1));
    result.template set<9, 10>(static_cast<float>(0));
    result.template set<10, 0>(static_cast<float>(0));
    result.template set<10, 1>(static_cast<float>(0));
    result.template set<10, 2>(static_cast<float>(0));
    result.template set<10, 3>(static_cast<float>(0));
    result.template set<10, 4>(static_cast<float>(0));
    result.template set<10, 5>(static_cast<float>(1.0));
    result.template set<10, 6>(static_cast<float>(0));
    result.template set<10, 7>(static_cast<float>(0));
    result.template set<10, 8>(static_cast<float>(0));
    result.template set<10, 9>(static_cast<float>(0));
    result.template set<10, 10>(static_cast<float>(1));

    return result;
  }
};

template <typename B_Updater_Output_Type> class B_Updater {
public:
  static inline auto update(float m, float l_f, float l_r, float I, float K_f,
                            float K_r, float theta, float beta, float px,
                            float delta, float V, float accel, float r,
                            float py) -> B_Updater_Output_Type {
    static_cast<void>(l_r);
    static_cast<void>(K_r);
    static_cast<void>(theta);
    static_cast<void>(beta);
    static_cast<void>(px);
    static_cast<void>(delta);
    static_cast<void>(accel);
    static_cast<void>(r);
    static_cast<void>(py);

    return B_Updater::sympy_function(l_f, I, V, K_f, m);
  }

  static inline auto sympy_function(float l_f, float I, float V, float K_f,
                                    float m) -> B_Updater_Output_Type {
    B_Updater_Output_Type result;

    float x0 = 0.02 * K_f;

    float x1 = l_f / I;

    result.template set<0, 0>(static_cast<float>(0));
    result.template set<0, 1>(static_cast<float>(0));
    result.template set<1, 0>(static_cast<float>(0));
    result.template set<1, 1>(static_cast<float>(0));
    result.template set<2, 0>(static_cast<float>(0));
    result.template set<2, 1>(static_cast<float>(0));
    result.template set<3, 0>(static_cast<float>(x0 * x1));
    result.template set<3, 1>(static_cast<float>(0));
    result.template set<4, 0>(static_cast<float>(x0 / (V * m)));
    result.template set<4, 1>(static_cast<float>(0));
    result.template set<5, 0>(static_cast<float>(0));
    result.template set<5, 1>(static_cast<float>(0.01));
    result.template set<6, 0>(static_cast<float>(0));
    result.template set<6, 1>(static_cast<float>(0));
    result.template set<7, 0>(static_cast<float>(0));
    result.template set<7, 1>(static_cast<float>(0));
    result.template set<8, 0>(static_cast<float>(0));
    result.template set<8, 1>(static_cast<float>(0));
    result.template set<9, 0>(static_cast<float>(0.0002 * K_f * x1));
    result.template set<9, 1>(static_cast<float>(0));
    result.template set<10, 0>(static_cast<float>(0));
    result.template set<10, 1>(static_cast<float>(0.01));

    return result;
  }
};

template <typename C_Updater_Output_Type> class C_Updater {
public:
  static inline auto update(float m, float l_f, float l_r, float I, float K_f,
                            float K_r, float theta, float beta, float px,
                            float delta, float V, float accel, float r,
                            float py) -> C_Updater_Output_Type {
    static_cast<void>(m);
    static_cast<void>(l_f);
    static_cast<void>(l_r);
    static_cast<void>(I);
    static_cast<void>(K_f);
    static_cast<void>(K_r);
    static_cast<void>(theta);
    static_cast<void>(beta);
    static_cast<void>(px);
    static_cast<void>(delta);
    static_cast<void>(V);
    static_cast<void>(accel);
    static_cast<void>(r);
    static_cast<void>(py);

    return C_Updater::sympy_function();
  }

  static inline auto sympy_function() -> C_Updater_Output_Type {
    C_Updater_Output_Type result;

    result.template set<0, 0>(static_cast<float>(0.0));
    result.template set<0, 1>(static_cast<float>(0.0));
    result.template set<0, 2>(static_cast<float>(0.0));
    result.template set<0, 3>(static_cast<float>(0.0));
    result.template set<0, 4>(static_cast<float>(0.0));
    result.template set<0, 5>(static_cast<float>(0.0));
    result.template set<0, 6>(static_cast<float>(1));
    result.template set<0, 7>(static_cast<float>(0));
    result.template set<0, 8>(static_cast<float>(0));
    result.template set<0, 9>(static_cast<float>(0));
    result.template set<0, 10>(static_cast<float>(0));
    result.template set<1, 0>(static_cast<float>(0.0));
    result.template set<1, 1>(static_cast<float>(0.0));
    result.template set<1, 2>(static_cast<float>(0.0));
    result.template set<1, 3>(static_cast<float>(0.0));
    result.template set<1, 4>(static_cast<float>(0.0));
    result.template set<1, 5>(static_cast<float>(0.0));
    result.template set<1, 6>(static_cast<float>(0));
    result.template set<1, 7>(static_cast<float>(1));
    result.template set<1, 8>(static_cast<float>(0));
    result.template set<1, 9>(static_cast<float>(0));
    result.template set<1, 10>(static_cast<float>(0));
    result.template set<2, 0>(static_cast<float>(0.0));
    result.template set<2, 1>(static_cast<float>(0.0));
    result.template set<2, 2>(static_cast<float>(0.0));
    result.template set<2, 3>(static_cast<float>(0.0));
    result.template set<2, 4>(static_cast<float>(0.0));
    result.template set<2, 5>(static_cast<float>(0.0));
    result.template set<2, 6>(static_cast<float>(0));
    result.template set<2, 7>(static_cast<float>(0));
    result.template set<2, 8>(static_cast<float>(1));
    result.template set<2, 9>(static_cast<float>(0));
    result.template set<2, 10>(static_cast<float>(0));
    result.template set<3, 0>(static_cast<float>(0.0));
    result.template set<3, 1>(static_cast<float>(0.0));
    result.template set<3, 2>(static_cast<float>(0.0));
    result.template set<3, 3>(static_cast<float>(0.0));
    result.template set<3, 4>(static_cast<float>(0.0));
    result.template set<3, 5>(static_cast<float>(0.0));
    result.template set<3, 6>(static_cast<float>(0));
    result.template set<3, 7>(static_cast<float>(0));
    result.template set<3, 8>(static_cast<float>(0));
    result.template set<3, 9>(static_cast<float>(1));
    result.template set<3, 10>(static_cast<float>(0));
    result.template set<4, 0>(static_cast<float>(0.0));
    result.template set<4, 1>(static_cast<float>(0.0));
    result.template set<4, 2>(static_cast<float>(0.0));
    result.template set<4, 3>(static_cast<float>(0.0));
    result.template set<4, 4>(static_cast<float>(0.0));
    result.template set<4, 5>(static_cast<float>(0.0));
    result.template set<4, 6>(static_cast<float>(0));
    result.template set<4, 7>(static_cast<float>(0));
    result.template set<4, 8>(static_cast<float>(0));
    result.template set<4, 9>(static_cast<float>(0));
    result.template set<4, 10>(static_cast<float>(1));

    return result;
  }
};

class EmbeddedIntegrator_Updater {
public:
  template <typename X_Type, typename U_Type, typename Parameter_Type,
            typename EmbeddedIntegrator_Updater_Output_Type>
  static inline void update(const X_Type &X, const U_Type &U,
                            const Parameter_Type &parameter,
                            EmbeddedIntegrator_Updater_Output_Type &output) {
    float m = parameter.m;
    float l_f = parameter.l_f;
    float l_r = parameter.l_r;
    float I = parameter.I;
    float K_f = parameter.K_f;
    float K_r = parameter.K_r;
    float px = X.template get<0, 0>();
    float py = X.template get<1, 0>();
    float theta = X.template get<2, 0>();
    float r = X.template get<3, 0>();
    float beta = X.template get<4, 0>();
    float V = X.template get<5, 0>();
    float delta = U.template get<0, 0>();
    float accel = U.template get<1, 0>();

    auto A =
        A_Updater<typename EmbeddedIntegrator_Updater_Output_Type::A_Type>::
            update(m, l_f, l_r, I, K_f, K_r, theta, beta, px, delta, V, accel,
                   r, py);

    auto B =
        B_Updater<typename EmbeddedIntegrator_Updater_Output_Type::B_Type>::
            update(m, l_f, l_r, I, K_f, K_r, theta, beta, px, delta, V, accel,
                   r, py);

    auto C =
        C_Updater<typename EmbeddedIntegrator_Updater_Output_Type::C_Type>::
            update(m, l_f, l_r, I, K_f, K_r, theta, beta, px, delta, V, accel,
                   r, py);

    output.A = A;
    output.B = B;
    output.C = C;
  }
};

} // namespace
  // two_wheel_vehicle_model_mpc_embedded_integrator_state_space_updater

#endif // TWO_WHEEL_VEHICLE_MODEL_MPC_EMBEDDED_INTEGRATOR_STATE_SPACE_UPDATER_HPP_
