#ifndef __TWO_WHEEL_VEHICLE_MODEL_MPC_EMBEDDED_INTEGRATOR_STATE_SPACE_UPDATER_HPP__
#define __TWO_WHEEL_VEHICLE_MODEL_MPC_EMBEDDED_INTEGRATOR_STATE_SPACE_UPDATER_HPP__

namespace two_wheel_vehicle_model_mpc_embedded_integrator_state_space_updater {

template <typename A_Updater_Output_Type> class A_Updater {
public:
  static inline auto update(float m, float l_f, float l_r, float I, float K_f,
                            float K_r, float r, float delta, float theta,
                            float beta, float accel, float px, float V,
                            float py) -> A_Updater_Output_Type {
    static_cast<void>(accel);
    static_cast<void>(px);
    static_cast<void>(py);

    return A_Updater::sympy_function(delta, r, l_f, I, K_r, K_f, l_r, m, theta,
                                     beta, V);
  }

  static inline auto sympy_function(float delta, float r, float l_f, float I,
                                    float K_r, float K_f, float l_r, float m,
                                    float theta, float beta, float V)
      -> A_Updater_Output_Type {
    A_Updater_Output_Type result;

    float x0 = 0.01 * sin(theta);

    float x1 = -V * x0;

    float x2 = 0.01 * cos(theta);

    float x3 = V * x2;

    float x4 = K_f * (l_f * l_f);

    float x5 = K_r * (l_r * l_r);

    float x6 = -x4 - x5;

    float x7 = 1 / V;

    float x8 = 1 / I;

    float x9 = 0.02 * x8;

    float x10 = x7 * x9;

    float x11 = K_f * l_f;

    float x12 = V * x11;

    float x13 = K_r * V * l_r - x12;

    float x14 = K_r * beta * l_r - beta * x11 + delta * x11;

    float x15 = V * V;

    float x16 = 1 / x15;

    float x17 = x16 * (K_f * V * delta * l_f + K_r * V * beta * l_r -
                       beta * x12 - r * x4 - r * x5);

    float x18 = 2 * x11;

    float x19 = m * x15;

    float x20 = 1 / m;

    float x21 = 0.01 * x16 * x20;

    float x22 = 2 * V;

    float x23 = K_f * x22;

    float x24 = K_r * x22;

    float x25 = 2 * beta;

    float x26 = 0.0002 * x8;

    float x27 = x26 * x7;

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
    result.template set<3, 3>(static_cast<float>(x10 * x6 + 1));
    result.template set<3, 4>(static_cast<float>(x10 * x13));
    result.template set<3, 5>(static_cast<float>(x10 * x14 - x17 * x9));
    result.template set<3, 6>(static_cast<float>(0.0));
    result.template set<3, 7>(static_cast<float>(0.0));
    result.template set<3, 8>(static_cast<float>(0.0));
    result.template set<3, 9>(static_cast<float>(0.0));
    result.template set<3, 10>(static_cast<float>(0.0));
    result.template set<4, 0>(static_cast<float>(0));
    result.template set<4, 1>(static_cast<float>(0));
    result.template set<4, 2>(static_cast<float>(0));
    result.template set<4, 3>(
        static_cast<float>(x21 * (2 * K_r * l_r - x18 - x19)));
    result.template set<4, 4>(static_cast<float>(x21 * (-x23 - x24) + 1));
    result.template set<4, 5>(static_cast<float>(
        x21 * (2 * K_f * delta - K_f * x25 - K_r * x25 - m * r * x22) -
        0.02 * x20 *
            (2 * K_f * V * delta + 2 * K_r * l_r * r - beta * x23 - beta * x24 -
             r * x18 - r * x19) /
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
    result.template set<9, 3>(static_cast<float>(x27 * x6 + 0.01));
    result.template set<9, 4>(static_cast<float>(x13 * x27));
    result.template set<9, 5>(static_cast<float>(x14 * x27 - x17 * x26));
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
                            float K_r, float r, float delta, float theta,
                            float beta, float accel, float px, float V,
                            float py) -> B_Updater_Output_Type {
    static_cast<void>(l_r);
    static_cast<void>(K_r);
    static_cast<void>(r);
    static_cast<void>(delta);
    static_cast<void>(theta);
    static_cast<void>(beta);
    static_cast<void>(accel);
    static_cast<void>(px);
    static_cast<void>(py);

    return B_Updater::sympy_function(m, l_f, K_f, V, I);
  }

  static inline auto sympy_function(float m, float l_f, float K_f, float V,
                                    float I) -> B_Updater_Output_Type {
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
                            float K_r, float r, float delta, float theta,
                            float beta, float accel, float px, float V,
                            float py) -> C_Updater_Output_Type {
    static_cast<void>(m);
    static_cast<void>(l_f);
    static_cast<void>(l_r);
    static_cast<void>(I);
    static_cast<void>(K_f);
    static_cast<void>(K_r);
    static_cast<void>(r);
    static_cast<void>(delta);
    static_cast<void>(theta);
    static_cast<void>(beta);
    static_cast<void>(accel);
    static_cast<void>(px);
    static_cast<void>(V);
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
            update(m, l_f, l_r, I, K_f, K_r, r, delta, theta, beta, accel, px,
                   V, py);

    auto B =
        B_Updater<typename EmbeddedIntegrator_Updater_Output_Type::B_Type>::
            update(m, l_f, l_r, I, K_f, K_r, r, delta, theta, beta, accel, px,
                   V, py);

    auto C =
        C_Updater<typename EmbeddedIntegrator_Updater_Output_Type::C_Type>::
            update(m, l_f, l_r, I, K_f, K_r, r, delta, theta, beta, accel, px,
                   V, py);

    output.A = A;
    output.B = B;
    output.C = C;
  }
};

} // namespace
  // two_wheel_vehicle_model_mpc_embedded_integrator_state_space_updater

#endif // __TWO_WHEEL_VEHICLE_MODEL_MPC_EMBEDDED_INTEGRATOR_STATE_SPACE_UPDATER_HPP__
