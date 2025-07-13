#ifndef __MPC_STATE_SPACE_UPDATER_HPP__
#define __MPC_STATE_SPACE_UPDATER_HPP__

namespace mpc_state_space_updater {

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

    double x3 = Lshaft * shaftrho * x2;

    double x4 = 1 / (25.0 * x1 + 0.098174770424681 * x3);

    double x5 = G * x2 / Lshaft;

    double x6 = 0.00490873852123405 * x5;

    double x7 = 1 / (Mmotor * x0);

    double x8 = x5 * x7;

    double x9 = 0.1 * x7;

        result.template set<0, 0>(static_cast<double>(1));
        result.template set<0, 1>(static_cast<double>(0.05));
        result.template set<0, 2>(static_cast<double>(0));
        result.template set<0, 3>(static_cast<double>(0));
        result.template set<1, 0>(static_cast<double>(-x4 * x6));
        result.template set<1, 1>(static_cast<double>(-0.05 * Bload * x4 + 1));
        result.template set<1, 2>(static_cast<double>(x6 / (500.0 * x1 + 1.96349540849362 * x3)));
        result.template set<1, 3>(static_cast<double>(0));
        result.template set<2, 0>(static_cast<double>(0));
        result.template set<2, 1>(static_cast<double>(0));
        result.template set<2, 2>(static_cast<double>(1));
        result.template set<2, 3>(static_cast<double>(0.05));
        result.template set<3, 0>(static_cast<double>(0.000490873852123405 * x8));
        result.template set<3, 1>(static_cast<double>(0));
        result.template set<3, 2>(static_cast<double>(-2.45436926061703e-05 * x8));
        result.template set<3, 3>(static_cast<double>(-Bmotor * x9 - Kt * Kt * x9 / R + 1));
        
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
        
        return result;
}


};

template <typename C_Updater_Output_Type>
class C_Updater {
public:
static inline auto update(double Lshaft, double dshaft, double shaftrho, double G, double Mmotor, double Rmotor, double Bmotor, double R, double Kt, double Bload) -> C_Updater_Output_Type {
static_cast<void>(shaftrho);
static_cast<void>(Mmotor);
static_cast<void>(Rmotor);
static_cast<void>(Bmotor);
static_cast<void>(R);
static_cast<void>(Kt);
static_cast<void>(Bload);

        return C_Updater::sympy_function(G, dshaft, Lshaft);
}

static inline auto sympy_function(double G, double dshaft, double Lshaft) -> C_Updater_Output_Type {
    C_Updater_Output_Type result;

    double x0 = G * (dshaft * dshaft * dshaft * dshaft) / Lshaft;

        result.template set<0, 0>(static_cast<double>(1.0));
        result.template set<0, 1>(static_cast<double>(0.0));
        result.template set<0, 2>(static_cast<double>(0.0));
        result.template set<0, 3>(static_cast<double>(0.0));
        result.template set<1, 0>(static_cast<double>(0.098174770424681 * x0));
        result.template set<1, 1>(static_cast<double>(0.0));
        result.template set<1, 2>(static_cast<double>(-0.00490873852123405 * x0));
        result.template set<1, 3>(static_cast<double>(0.0));
        
        return result;
}


};

class MPC_StateSpace_Updater {
public:
template <typename Parameter_Type, typename MPC_StateSpace_Updater_Output_Type>
static inline void update(const Parameter_Type& parameter, MPC_StateSpace_Updater_Output_Type& output) {
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

    auto A = A_Updater<typename MPC_StateSpace_Updater_Output_Type::A_Type>::update(Lshaft, dshaft, shaftrho, G, Mmotor, Rmotor, Bmotor, R, Kt, Bload);

    auto B = B_Updater<typename MPC_StateSpace_Updater_Output_Type::B_Type>::update(Lshaft, dshaft, shaftrho, G, Mmotor, Rmotor, Bmotor, R, Kt, Bload);

    auto C = C_Updater<typename MPC_StateSpace_Updater_Output_Type::C_Type>::update(Lshaft, dshaft, shaftrho, G, Mmotor, Rmotor, Bmotor, R, Kt, Bload);

    output.A = A;
    output.B = B;
    output.C = C;
}
};

} // namespace mpc_state_space_updater

#endif // __MPC_STATE_SPACE_UPDATER_HPP__
