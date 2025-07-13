#ifndef __MPC_EMBEDDED_INTEGRATOR_STATE_SPACE_UPDATER_HPP__
#define __MPC_EMBEDDED_INTEGRATOR_STATE_SPACE_UPDATER_HPP__

namespace mpc_embedded_integrator_state_space_updater {

template <typename A_Updater_Output_Type>
class A_Updater {
public:
static inline auto update(double Lshaft, double dshaft, double shaftrho, double G, double Mmotor, double Rmotor, double Bmotor, double R, double Kt, double Bload) -> A_Updater_Output_Type {

        return A_Updater::sympy_function(Mmotor, G, Bmotor, Bload, shaftrho, Lshaft, Kt,
    dshaft, Rmotor, R);
}

static inline auto sympy_function(double Mmotor, double G, double Bmotor, double Bload, double shaftrho, double Lshaft, double Kt, double dshaft, double Rmotor, double R) -> A_Updater_Output_Type {
    A_Updater_Output_Type result;

    double x0 = Rmotor * Rmotor;

    double x1 = Mmotor * x0;

    double x2 = dshaft * dshaft * dshaft * dshaft;

    double x3 = 0.098174770424681 * x2;

    double x4 = Lshaft * shaftrho;

    double x5 = 1 / (25.0 * x1 + x3 * x4);

    double x6 = G / Lshaft;

    double x7 = x2 * x6;

    double x8 = 0.00490873852123405 * x7;

    double x9 = 1 / (Mmotor * x0);

    double x10 = x7 * x9;

    double x11 = 0.1 * x9;

        result.template set<0, 0>(static_cast<double>(1));
        result.template set<0, 1>(static_cast<double>(0.05));
        result.template set<0, 2>(static_cast<double>(0));
        result.template set<0, 3>(static_cast<double>(0));
        result.template set<0, 4>(static_cast<double>(0.0));
        result.template set<0, 5>(static_cast<double>(0.0));
        result.template set<1, 0>(static_cast<double>(-x5 * x8));
        result.template set<1, 1>(static_cast<double>(-0.05 * Bload * x5 + 1));
        result.template set<1, 2>(static_cast<double>(x8 / (500.0 * x1 + 1.96349540849362 * x2 * x4)));
        result.template set<1, 3>(static_cast<double>(0));
        result.template set<1, 4>(static_cast<double>(0.0));
        result.template set<1, 5>(static_cast<double>(0.0));
        result.template set<2, 0>(static_cast<double>(0));
        result.template set<2, 1>(static_cast<double>(0));
        result.template set<2, 2>(static_cast<double>(1));
        result.template set<2, 3>(static_cast<double>(0.05));
        result.template set<2, 4>(static_cast<double>(0.0));
        result.template set<2, 5>(static_cast<double>(0.0));
        result.template set<3, 0>(static_cast<double>(0.000490873852123405 * x10));
        result.template set<3, 1>(static_cast<double>(0));
        result.template set<3, 2>(static_cast<double>(-2.45436926061703e-05 * x10));
        result.template set<3, 3>(static_cast<double>(-Bmotor * x11 - Kt * Kt * x11 / R + 1));
        result.template set<3, 4>(static_cast<double>(0.0));
        result.template set<3, 5>(static_cast<double>(0.0));
        result.template set<4, 0>(static_cast<double>(1.0));
        result.template set<4, 1>(static_cast<double>(0.05));
        result.template set<4, 2>(static_cast<double>(0));
        result.template set<4, 3>(static_cast<double>(0));
        result.template set<4, 4>(static_cast<double>(1));
        result.template set<4, 5>(static_cast<double>(0));
        result.template set<5, 0>(static_cast<double>(x3 * x6));
        result.template set<5, 1>(static_cast<double>(x8));
        result.template set<5, 2>(static_cast<double>(-x8));
        result.template set<5, 3>(static_cast<double>(-0.000245436926061703 * x7));
        result.template set<5, 4>(static_cast<double>(0));
        result.template set<5, 5>(static_cast<double>(1));
        
        return result;
}


};

template <typename B_Updater_Output_Type>
class B_Updater {
public:
static inline auto update(double Lshaft, double dshaft, double shaftrho, double G, double Mmotor, double Rmotor, double Bmotor, double R, double Kt, double Bload) -> B_Updater_Output_Type {
static_cast<void>(Lshaft);
static_cast<void>(dshaft);
static_cast<void>(shaftrho);
static_cast<void>(G);
static_cast<void>(Bmotor);
static_cast<void>(Bload);

        return B_Updater::sympy_function(Rmotor, R, Kt, Mmotor);
}

static inline auto sympy_function(double Rmotor, double R, double Kt, double Mmotor) -> B_Updater_Output_Type {
    B_Updater_Output_Type result;

        result.template set<0, 0>(static_cast<double>(0));
        result.template set<1, 0>(static_cast<double>(0));
        result.template set<2, 0>(static_cast<double>(0));
        result.template set<3, 0>(static_cast<double>(0.1 * Kt / (Mmotor * R * (Rmotor * Rmotor))));
        result.template set<4, 0>(static_cast<double>(0));
        result.template set<5, 0>(static_cast<double>(0));
        
        return result;
}


};

template <typename C_Updater_Output_Type>
class C_Updater {
public:
static inline auto update(double Lshaft, double dshaft, double shaftrho, double G, double Mmotor, double Rmotor, double Bmotor, double R, double Kt, double Bload) -> C_Updater_Output_Type {
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

        result.template set<0, 0>(static_cast<double>(0));
        result.template set<0, 1>(static_cast<double>(0));
        result.template set<0, 2>(static_cast<double>(0));
        result.template set<0, 3>(static_cast<double>(0));
        result.template set<0, 4>(static_cast<double>(1.0));
        result.template set<0, 5>(static_cast<double>(0));
        result.template set<1, 0>(static_cast<double>(0));
        result.template set<1, 1>(static_cast<double>(0));
        result.template set<1, 2>(static_cast<double>(0));
        result.template set<1, 3>(static_cast<double>(0));
        result.template set<1, 4>(static_cast<double>(0));
        result.template set<1, 5>(static_cast<double>(0.005));
        
        return result;
}


};

class EmbeddedIntegrator_Updater {
public:
template <typename Parameter_Type, typename EmbeddedIntegrator_Updater_Output_Type>
static inline void update(const Parameter_Type& parameter, EmbeddedIntegrator_Updater_Output_Type& output) {
    double Lshaft = parameter.Lshaft;
    double dshaft = parameter.dshaft;
    double shaftrho = parameter.shaftrho;
    double G = parameter.G;
    double Mmotor = parameter.Mmotor;
    double Rmotor = parameter.Rmotor;
    double Bmotor = parameter.Bmotor;
    double R = parameter.R;
    double Kt = parameter.Kt;
    double Bload = parameter.Bload;

    auto A = A_Updater<typename EmbeddedIntegrator_Updater_Output_Type::A_Type>::update(Lshaft, dshaft, shaftrho, G, Mmotor, Rmotor, Bmotor, R, Kt, Bload);

    auto B = B_Updater<typename EmbeddedIntegrator_Updater_Output_Type::B_Type>::update(Lshaft, dshaft, shaftrho, G, Mmotor, Rmotor, Bmotor, R, Kt, Bload);

    auto C = C_Updater<typename EmbeddedIntegrator_Updater_Output_Type::C_Type>::update(Lshaft, dshaft, shaftrho, G, Mmotor, Rmotor, Bmotor, R, Kt, Bload);

    output.A = A;
    output.B = B;
    output.C = C;
}
};

} // namespace mpc_embedded_integrator_state_space_updater

#endif // __MPC_EMBEDDED_INTEGRATOR_STATE_SPACE_UPDATER_HPP__
