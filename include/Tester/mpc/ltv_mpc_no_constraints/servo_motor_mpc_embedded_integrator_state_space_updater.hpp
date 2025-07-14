#ifndef __SERVO_MOTOR_MPC_EMBEDDED_INTEGRATOR_STATE_SPACE_UPDATER_HPP__
#define __SERVO_MOTOR_MPC_EMBEDDED_INTEGRATOR_STATE_SPACE_UPDATER_HPP__

namespace servo_motor_mpc_embedded_integrator_state_space_updater {

template <typename A_Updater_Output_Type> class A_Updater {
public:
  static inline auto update(float Lshaft, float dshaft, float shaftrho, float G,
                            float Mmotor, float Rmotor, float Bmotor, float R,
                            float Kt, float Bload) -> A_Updater_Output_Type {

    return A_Updater::sympy_function(Bmotor, Lshaft, R, dshaft, Bload, shaftrho,
                                     Kt, Mmotor, G, Rmotor);
  }

  static inline auto sympy_function(float Bmotor, float Lshaft, float R,
                                    float dshaft, float Bload, float shaftrho,
                                    float Kt, float Mmotor, float G,
                                    float Rmotor) -> A_Updater_Output_Type {
    A_Updater_Output_Type result;

    float x0 = Rmotor * Rmotor;

    float x1 = Mmotor * x0;

    float x2 = dshaft * dshaft * dshaft * dshaft;

    float x3 = 0.098174770424681 * x2;

    float x4 = Lshaft * shaftrho;

    float x5 = 1 / (25.0 * x1 + x3 * x4);

    float x6 = G / Lshaft;

    float x7 = x2 * x6;

    float x8 = 0.00490873852123405 * x7;

    float x9 = 1 / (Mmotor * x0);

    float x10 = x7 * x9;

    float x11 = 0.1 * x9;

    result.template set<0, 0>(static_cast<float>(1));
    result.template set<0, 1>(static_cast<float>(0.05));
    result.template set<0, 2>(static_cast<float>(0));
    result.template set<0, 3>(static_cast<float>(0));
    result.template set<0, 4>(static_cast<float>(0.0));
    result.template set<0, 5>(static_cast<float>(0.0));
    result.template set<1, 0>(static_cast<float>(-x5 * x8));
    result.template set<1, 1>(static_cast<float>(-0.05 * Bload * x5 + 1));
    result.template set<1, 2>(
        static_cast<float>(x8 / (500.0 * x1 + 1.96349540849362 * x2 * x4)));
    result.template set<1, 3>(static_cast<float>(0));
    result.template set<1, 4>(static_cast<float>(0.0));
    result.template set<1, 5>(static_cast<float>(0.0));
    result.template set<2, 0>(static_cast<float>(0));
    result.template set<2, 1>(static_cast<float>(0));
    result.template set<2, 2>(static_cast<float>(1));
    result.template set<2, 3>(static_cast<float>(0.05));
    result.template set<2, 4>(static_cast<float>(0.0));
    result.template set<2, 5>(static_cast<float>(0.0));
    result.template set<3, 0>(static_cast<float>(0.000490873852123405 * x10));
    result.template set<3, 1>(static_cast<float>(0));
    result.template set<3, 2>(static_cast<float>(-2.45436926061703e-05 * x10));
    result.template set<3, 3>(
        static_cast<float>(-Bmotor * x11 - Kt * Kt * x11 / R + 1));
    result.template set<3, 4>(static_cast<float>(0.0));
    result.template set<3, 5>(static_cast<float>(0.0));
    result.template set<4, 0>(static_cast<float>(1.0));
    result.template set<4, 1>(static_cast<float>(0.05));
    result.template set<4, 2>(static_cast<float>(0));
    result.template set<4, 3>(static_cast<float>(0));
    result.template set<4, 4>(static_cast<float>(1));
    result.template set<4, 5>(static_cast<float>(0));
    result.template set<5, 0>(static_cast<float>(x3 * x6));
    result.template set<5, 1>(static_cast<float>(x8));
    result.template set<5, 2>(static_cast<float>(-x8));
    result.template set<5, 3>(static_cast<float>(-0.000245436926061703 * x7));
    result.template set<5, 4>(static_cast<float>(0));
    result.template set<5, 5>(static_cast<float>(1));

    return result;
  }
};

template <typename B_Updater_Output_Type> class B_Updater {
public:
  static inline auto update(float Lshaft, float dshaft, float shaftrho, float G,
                            float Mmotor, float Rmotor, float Bmotor, float R,
                            float Kt, float Bload) -> B_Updater_Output_Type {
    static_cast<void>(Lshaft);
    static_cast<void>(dshaft);
    static_cast<void>(shaftrho);
    static_cast<void>(G);
    static_cast<void>(Bmotor);
    static_cast<void>(Bload);

    return B_Updater::sympy_function(Mmotor, R, Rmotor, Kt);
  }

  static inline auto sympy_function(float Mmotor, float R, float Rmotor,
                                    float Kt) -> B_Updater_Output_Type {
    B_Updater_Output_Type result;

    result.template set<0, 0>(static_cast<float>(0));
    result.template set<1, 0>(static_cast<float>(0));
    result.template set<2, 0>(static_cast<float>(0));
    result.template set<3, 0>(
        static_cast<float>(0.1 * Kt / (Mmotor * R * (Rmotor * Rmotor))));
    result.template set<4, 0>(static_cast<float>(0));
    result.template set<5, 0>(static_cast<float>(0));

    return result;
  }
};

template <typename C_Updater_Output_Type> class C_Updater {
public:
  static inline auto update(float Lshaft, float dshaft, float shaftrho, float G,
                            float Mmotor, float Rmotor, float Bmotor, float R,
                            float Kt, float Bload) -> C_Updater_Output_Type {
    static_cast<void>(Lshaft);
    static_cast<void>(dshaft);
    static_cast<void>(shaftrho);
    static_cast<void>(G);
    static_cast<void>(Mmotor);
    static_cast<void>(Rmotor);
    static_cast<void>(Bmotor);
    static_cast<void>(R);
    static_cast<void>(Kt);
    static_cast<void>(Bload);

    return C_Updater::sympy_function();
  }

  static inline auto sympy_function() -> C_Updater_Output_Type {
    C_Updater_Output_Type result;

    result.template set<0, 0>(static_cast<float>(0));
    result.template set<0, 1>(static_cast<float>(0));
    result.template set<0, 2>(static_cast<float>(0));
    result.template set<0, 3>(static_cast<float>(0));
    result.template set<0, 4>(static_cast<float>(1.0));
    result.template set<0, 5>(static_cast<float>(0));
    result.template set<1, 0>(static_cast<float>(0));
    result.template set<1, 1>(static_cast<float>(0));
    result.template set<1, 2>(static_cast<float>(0));
    result.template set<1, 3>(static_cast<float>(0));
    result.template set<1, 4>(static_cast<float>(0));
    result.template set<1, 5>(static_cast<float>(0.005));

    return result;
  }
};

class EmbeddedIntegrator_Updater {
public:
  template <typename Parameter_Type,
            typename EmbeddedIntegrator_Updater_Output_Type>
  static inline void update(const Parameter_Type &parameter,
                            EmbeddedIntegrator_Updater_Output_Type &output) {
    float Lshaft = parameter.Lshaft;
    float dshaft = parameter.dshaft;
    float shaftrho = parameter.shaftrho;
    float G = parameter.G;
    float Mmotor = parameter.Mmotor;
    float Rmotor = parameter.Rmotor;
    float Bmotor = parameter.Bmotor;
    float R = parameter.R;
    float Kt = parameter.Kt;
    float Bload = parameter.Bload;

    auto A =
        A_Updater<typename EmbeddedIntegrator_Updater_Output_Type::A_Type>::
            update(Lshaft, dshaft, shaftrho, G, Mmotor, Rmotor, Bmotor, R, Kt,
                   Bload);

    auto B =
        B_Updater<typename EmbeddedIntegrator_Updater_Output_Type::B_Type>::
            update(Lshaft, dshaft, shaftrho, G, Mmotor, Rmotor, Bmotor, R, Kt,
                   Bload);

    auto C =
        C_Updater<typename EmbeddedIntegrator_Updater_Output_Type::C_Type>::
            update(Lshaft, dshaft, shaftrho, G, Mmotor, Rmotor, Bmotor, R, Kt,
                   Bload);

    output.A = A;
    output.B = B;
    output.C = C;
  }
};

} // namespace servo_motor_mpc_embedded_integrator_state_space_updater

#endif // __SERVO_MOTOR_MPC_EMBEDDED_INTEGRATOR_STATE_SPACE_UPDATER_HPP__
