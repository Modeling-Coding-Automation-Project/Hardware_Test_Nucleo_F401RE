#ifndef __SERVO_MOTOR_LTV_CONSTRAINTS_MPC_STATE_SPACE_UPDATER_HPP__
#define __SERVO_MOTOR_LTV_CONSTRAINTS_MPC_STATE_SPACE_UPDATER_HPP__

namespace servo_motor_ltv_constraints_mpc_state_space_updater {

template <typename A_Updater_Output_Type> class A_Updater {
public:
  static inline auto update(float Lshaft, float dshaft, float shaftrho, float G,
                            float Mmotor, float Rmotor, float Bmotor, float R,
                            float Kt, float Bload) -> A_Updater_Output_Type {

    return A_Updater::sympy_function(Rmotor, Lshaft, Bmotor, dshaft, R,
                                     shaftrho, Mmotor, G, Bload, Kt);
  }

  static inline auto sympy_function(float Rmotor, float Lshaft, float Bmotor,
                                    float dshaft, float R, float shaftrho,
                                    float Mmotor, float G, float Bload,
                                    float Kt) -> A_Updater_Output_Type {
    A_Updater_Output_Type result;

    float x0 = Rmotor * Rmotor;

    float x1 = Mmotor * x0;

    float x2 = dshaft * dshaft * dshaft * dshaft;

    float x3 = Lshaft * shaftrho * x2;

    float x4 = 1 / (25.0 * x1 + 0.098174770424681 * x3);

    float x5 = G * x2 / Lshaft;

    float x6 = 0.00490873852123405 * x5;

    float x7 = 1 / (Mmotor * x0);

    float x8 = x5 * x7;

    float x9 = 0.1 * x7;

    result.template set<0, 0>(static_cast<float>(1));
    result.template set<0, 1>(static_cast<float>(0.05));
    result.template set<0, 2>(static_cast<float>(0));
    result.template set<0, 3>(static_cast<float>(0));
    result.template set<1, 0>(static_cast<float>(-x4 * x6));
    result.template set<1, 1>(static_cast<float>(-0.05 * Bload * x4 + 1));
    result.template set<1, 2>(
        static_cast<float>(x6 / (500.0 * x1 + 1.96349540849362 * x3)));
    result.template set<1, 3>(static_cast<float>(0));
    result.template set<2, 0>(static_cast<float>(0));
    result.template set<2, 1>(static_cast<float>(0));
    result.template set<2, 2>(static_cast<float>(1));
    result.template set<2, 3>(static_cast<float>(0.05));
    result.template set<3, 0>(static_cast<float>(0.000490873852123405 * x8));
    result.template set<3, 1>(static_cast<float>(0));
    result.template set<3, 2>(static_cast<float>(-2.45436926061703e-05 * x8));
    result.template set<3, 3>(
        static_cast<float>(-Bmotor * x9 - Kt * Kt * x9 / R + 1));

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

    return B_Updater::sympy_function(Mmotor, Rmotor, Kt, R);
  }

  static inline auto sympy_function(float Mmotor, float Rmotor, float Kt,
                                    float R) -> B_Updater_Output_Type {
    B_Updater_Output_Type result;

    result.template set<0, 0>(static_cast<float>(0));
    result.template set<1, 0>(static_cast<float>(0));
    result.template set<2, 0>(static_cast<float>(0));
    result.template set<3, 0>(
        static_cast<float>(0.1 * Kt / (Mmotor * R * (Rmotor * Rmotor))));

    return result;
  }
};

template <typename C_Updater_Output_Type> class C_Updater {
public:
  static inline auto update(float Lshaft, float dshaft, float shaftrho, float G,
                            float Mmotor, float Rmotor, float Bmotor, float R,
                            float Kt, float Bload) -> C_Updater_Output_Type {
    static_cast<void>(shaftrho);
    static_cast<void>(Mmotor);
    static_cast<void>(Rmotor);
    static_cast<void>(Bmotor);
    static_cast<void>(R);
    static_cast<void>(Kt);
    static_cast<void>(Bload);

    return C_Updater::sympy_function(G, Lshaft, dshaft);
  }

  static inline auto sympy_function(float G, float Lshaft, float dshaft)
      -> C_Updater_Output_Type {
    C_Updater_Output_Type result;

    float x0 = G * (dshaft * dshaft * dshaft * dshaft) / Lshaft;

    result.template set<0, 0>(static_cast<float>(1.0));
    result.template set<0, 1>(static_cast<float>(0.0));
    result.template set<0, 2>(static_cast<float>(0.0));
    result.template set<0, 3>(static_cast<float>(0.0));
    result.template set<1, 0>(static_cast<float>(0.098174770424681 * x0));
    result.template set<1, 1>(static_cast<float>(0.0));
    result.template set<1, 2>(static_cast<float>(-0.00490873852123405 * x0));
    result.template set<1, 3>(static_cast<float>(0.0));

    return result;
  }
};

class MPC_StateSpace_Updater {
public:
  template <typename Parameter_Type,
            typename MPC_StateSpace_Updater_Output_Type>
  static inline void update(const Parameter_Type &parameter,
                            MPC_StateSpace_Updater_Output_Type &output) {
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
        A_Updater<typename MPC_StateSpace_Updater_Output_Type::A_Type>::update(
            Lshaft, dshaft, shaftrho, G, Mmotor, Rmotor, Bmotor, R, Kt, Bload);

    auto B =
        B_Updater<typename MPC_StateSpace_Updater_Output_Type::B_Type>::update(
            Lshaft, dshaft, shaftrho, G, Mmotor, Rmotor, Bmotor, R, Kt, Bload);

    auto C =
        C_Updater<typename MPC_StateSpace_Updater_Output_Type::C_Type>::update(
            Lshaft, dshaft, shaftrho, G, Mmotor, Rmotor, Bmotor, R, Kt, Bload);

    output.A = A;
    output.B = B;
    output.C = C;
  }
};

} // namespace servo_motor_ltv_constraints_mpc_state_space_updater

#endif // __SERVO_MOTOR_LTV_CONSTRAINTS_MPC_STATE_SPACE_UPDATER_HPP__
