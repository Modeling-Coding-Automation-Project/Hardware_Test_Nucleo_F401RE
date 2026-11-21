#ifndef SERVO_MOTOR_LTV_CONSTRAINTS_LTV_MPC_F_HPP_
#define SERVO_MOTOR_LTV_CONSTRAINTS_LTV_MPC_F_HPP_

#include "python_numpy.hpp"

namespace servo_motor_ltv_constraints_ltv_mpc_F {

using namespace PythonNumpy;

using SparseAvailable_ltv_mpc_F =
    SparseAvailable<ColumnAvailable<true, true, false, false, true, false>,
                    ColumnAvailable<true, true, true, true, false, true>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, true, false, true>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, true, true, true, false, true>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, true, true, true, false, true>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, true, true, true, false, true>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, true, true, true, false, true>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, true, true, true, false, true>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, true, true, true, false, true>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, true, true, true, false, true>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, true, true, true, false, true>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, true, true, true, false, true>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, true, true, true, false, true>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, true, true, true, false, true>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, true, true, true, false, true>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, true, true, true, false, true>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, true, true, true, false, true>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, true, true, true, false, true>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, true, true, true, false, true>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, true, true, true, false, true>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, true, true, true, false, true>>;

using type = SparseMatrix_Type<float, SparseAvailable_ltv_mpc_F>;

inline auto make(void) -> type {

  auto result = make_SparseMatrixZeros<float, SparseAvailable_ltv_mpc_F>();

  result.unsafe_access(0) = static_cast<float>(1.0);
  result.unsafe_access(1) = static_cast<float>(0.05);
  result.unsafe_access(2) = static_cast<float>(1.0);
  result.unsafe_access(3) = static_cast<float>(6.400995031689201);
  result.unsafe_access(4) = static_cast<float>(0.3200497515844601);
  result.unsafe_access(5) = static_cast<float>(-0.3200497515844601);
  result.unsafe_access(6) = static_cast<float>(-0.016002487579223033);
  result.unsafe_access(7) = static_cast<float>(0.005);
  result.unsafe_access(8) = static_cast<float>(1.871980730795717);
  result.unsafe_access(9) = static_cast<float>(0.14750001233069035);
  result.unsafe_access(10) = static_cast<float>(0.006400963460214142);
  result.unsafe_access(11) = static_cast<float>(1.0);
  result.unsafe_access(12) = static_cast<float>(11.880107513752028);
  result.unsafe_access(13) = static_cast<float>(0.9441468461028448);
  result.unsafe_access(14) = static_cast<float>(-0.5940053756876016);
  result.unsafe_access(15) = static_cast<float>(-0.03984619407226533);
  result.unsafe_access(16) = static_cast<float>(0.005);
  result.unsafe_access(17) = static_cast<float>(2.4943238550717632);
  result.unsafe_access(18) = static_cast<float>(0.28372408462948123);
  result.unsafe_access(19) = static_cast<float>(0.025283807246411827);
  result.unsafe_access(20) = static_cast<float>(0.00032004817301070713);
  result.unsafe_access(21) = static_cast<float>(1.0);
  result.unsafe_access(22) = static_cast<float>(15.608667469960642);
  result.unsafe_access(23) = static_cast<float>(1.810994863909412);
  result.unsafe_access(24) = static_cast<float>(-0.7804333734980323);
  result.unsafe_access(25) = static_cast<float>(-0.06522739145901313);
  result.unsafe_access(26) = static_cast<float>(0.005);
  result.unsafe_access(27) = static_cast<float>(2.7699294824387004);
  result.unsafe_access(28) = static_cast<float>(0.44425414312187195);
  result.unsafe_access(29) = static_cast<float>(0.06150352587806497);
  result.unsafe_access(30) = static_cast<float>(0.001421013967095838);
  result.unsafe_access(31) = static_cast<float>(1.0);
  result.unsafe_access(32) = static_cast<float>(16.9552975127828);
  result.unsafe_access(33) = static_cast<float>(2.820928692412771);
  result.unsafe_access(34) = static_cast<float>(-0.8477648756391404);
  result.unsafe_access(35) = static_cast<float>(-0.0869855780690411);
  result.unsafe_access(36) = static_cast<float>(0.005);
  result.unsafe_access(37) = static_cast<float>(2.6415635709133034);
  result.unsafe_access(38) = static_cast<float>(0.6105380196469189);
  result.unsafe_access(39) = static_cast<float>(0.11792182145433483);
  result.unsafe_access(40) = static_cast<float>(0.0037714731377802096);
  result.unsafe_access(41) = static_cast<float>(1.0);
  result.unsafe_access(42) = static_cast<float>(15.576833697821932);
  result.unsafe_access(43) = static_cast<float>(3.847697580695696);
  result.unsafe_access(44) = static_cast<float>(-0.7788416848910971);
  result.unsafe_access(45) = static_cast<float>(-0.10101366461501021);
  result.unsafe_access(46) = static_cast<float>(0.005);
  result.unsafe_access(47) = static_cast<float>(2.1024921297978114);
  result.unsafe_access(48) = static_cast<float>(0.7620894477773434);
  result.unsafe_access(49) = static_cast<float>(0.19487539351010943);
  result.unsafe_access(50) = static_cast<float>(0.007744112910229045);
  result.unsafe_access(51) = static_cast<float>(1.0);
  result.unsafe_access(52) = static_cast<float>(11.47965211618273);
  result.unsafe_access(53) = static_cast<float>(4.754205087031816);
  result.unsafe_access(54) = static_cast<float>(-0.5739826058091371);
  result.unsafe_access(55) = static_cast<float>(-0.10444126748513291);
  result.unsafe_access(56) = static_cast<float>(0.005);
  result.unsafe_access(57) = static_cast<float>(1.2008194746060081);
  result.unsafe_access(58) = static_cast<float>(0.8791097698201468);
  result.unsafe_access(59) = static_cast<float>(0.2899590262696996);
  result.unsafe_access(60) = static_cast<float>(0.013538385001517704);
  result.unsafe_access(61) = static_cast<float>(1.0);
  result.unsafe_access(62) = static_cast<float>(5.039521895814333);
  result.unsafe_access(63) = static_cast<float>(5.4105283625264375);
  result.unsafe_access(64) = static_cast<float>(-0.2519760947907172);
  result.unsafe_access(65) = static_cast<float>(-0.09587783893739503);
  result.unsafe_access(66) = static_cast<float>(0.005);
  result.unsafe_access(67) = static_cast<float>(0.036618804083408785);
  result.unsafe_access(68) = static_cast<float>(0.9451954718600469);
  result.unsafe_access(69) = static_cast<float>(0.39816905979582956);
  result.unsafe_access(70) = static_cast<float>(0.02113175996422866);
  result.unsafe_access(71) = static_cast<float>(1.0);
  result.unsafe_access(72) = static_cast<float>(-3.0262343827774227);
  result.unsafe_access(73) = static_cast<float>(5.712029125086289);
  result.unsafe_access(74) = static_cast<float>(0.15131171913887054);
  result.unsafe_access(75) = static_cast<float>(-0.07558143339808247);
  result.unsafe_access(76) = static_cast<float>(0.005);
  result.unsafe_access(77) = static_cast<float>(-1.2481815766291247);
  result.unsafe_access(78) = static_cast<float>(0.9497668715694686);
  result.unsafe_access(79) = static_cast<float>(0.5124090788314563);
  result.unsafe_access(80) = static_cast<float>(0.030263015372263523);
  result.unsafe_access(81) = static_cast<float>(1.0);
  result.unsafe_access(82) = static_cast<float>(-11.734031616099811);
  result.unsafe_access(83) = static_cast<float>(5.595167109942811);
  result.unsafe_access(84) = static_cast<float>(0.5867015808049899);
  result.unsafe_access(85) = static_cast<float>(-0.04547180398733993);
  result.unsafe_access(86) = static_cast<float>(0.005);
  result.unsafe_access(87) = static_cast<float>(-2.4862373818425585);
  result.unsafe_access(88) = static_cast<float>(0.8898696833851628);
  result.unsafe_access(89) = static_cast<float>(0.6243118690921279);
  result.unsafe_access(90) = static_cast<float>(0.04044933147398194);
  result.unsafe_access(91) = static_cast<float>(1.0);
  result.unsafe_access(92) = static_cast<float>(-19.949885465629876);
  result.unsafe_access(93) = static_cast<float>(5.048758305070602);
  result.unsafe_access(94) = static_cast<float>(0.9974942732814932);
  result.unsafe_access(95) = static_cast<float>(-0.008948592492770108);
  result.unsafe_access(96) = static_cast<float>(0.005);
  result.unsafe_access(97) = static_cast<float>(-3.5057307431227525);
  result.unsafe_access(98) = static_cast<float>(0.771064549577927);
  result.unsafe_access(99) = static_cast<float>(0.7252865371561376);
  result.unsafe_access(100) = static_cast<float>(0.05103576587685755);
  result.unsafe_access(101) = static_cast<float>(1.0);
  result.unsafe_access(102) = static_cast<float>(-26.53293730211135);
  result.unsafe_access(103) = static_cast<float>(4.118877113213544);
  result.unsafe_access(104) = static_cast<float>(1.326646865105567);
  result.unsafe_access(105) = static_cast<float>(0.029487415763394274);
  result.unsafe_access(106) = static_cast<float>(0.005);
  result.unsafe_access(107) = static_cast<float>(-4.153273462232449);
  result.unsafe_access(108) = static_cast<float>(0.6072249750980571);
  result.unsafe_access(109) = static_cast<float>(0.8076636731116222);
  result.unsafe_access(110) = static_cast<float>(0.06127185213746709);
  result.unsafe_access(111) = static_cast<float>(1.0);
  result.unsafe_access(112) = static_cast<float>(-30.489106228140194);
  result.unsafe_access(113) = static_cast<float>(2.906337159803724);
  result.unsafe_access(114) = static_cast<float>(1.524455311407009);
  result.unsafe_access(115) = static_cast<float>(0.06477868940011851);
  result.unsafe_access(116) = static_cast<float>(0.005);
  result.unsafe_access(117) = static_cast<float>(-4.315802592210969);
  result.unsafe_access(118) = static_cast<float>(0.41920020298159444);
  result.unsafe_access(119) = static_cast<float>(0.8657901296105481);
  result.unsafe_access(120) = static_cast<float>(0.07040639120294);
  result.unsafe_access(121) = static_cast<float>(1.0);
  result.unsafe_access(122) = static_cast<float>(-31.114806312627977);
  result.unsafe_access(123) = static_cast<float>(1.5566154587338592);
  result.unsafe_access(124) = static_cast<float>(1.5557403156313978);
  result.unsafe_access(125) = static_cast<float>(0.09196183579718549);
  result.unsafe_access(126) = static_cast<float>(0.005);
  result.unsafe_access(127) = static_cast<float>(-3.938445704641599);
  result.unsafe_access(128) = static_cast<float>(0.23245016660252416);
  result.unsafe_access(129) = static_cast<float>(0.8969222852320796);
  result.unsafe_access(130) = static_cast<float>(0.07778863816996802);
  result.unsafe_access(131) = static_cast<float>(1.0);
  result.unsafe_access(132) = static_cast<float>(-28.11069949607994);
  result.unsafe_access(133) = static_cast<float>(0.24309450563309157);
  result.unsafe_access(134) = static_cast<float>(1.4055349748039956);
  result.unsafe_access(135) = static_cast<float>(0.10684582774296775);
  result.unsafe_access(136) = static_cast<float>(0.005);
  result.unsafe_access(137) = static_cast<float>(-3.035683027291144);
  result.unsafe_access(138) = static_cast<float>(0.07390543036573848);
  result.unsafe_access(139) = static_cast<float>(0.9017841513645568);
  result.unsafe_access(140) = static_cast<float>(0.08296254696488832);
  result.unsafe_access(141) = static_cast<float>(1.0);
  result.unsafe_access(142) = static_cast<float>(-21.64820047102578);
  result.unsafe_access(143) = static_cast<float>(-0.8545453829176383);
  result.unsafe_access(144) = static_cast<float>(1.0824100235512875);
  result.unsafe_access(145) = static_cast<float>(0.10662871675503094);
  result.unsafe_access(146) = static_cast<float>(0.005);
  result.unsafe_access(147) = static_cast<float>(-1.6938665600656093);
  result.unsafe_access(148) = static_cast<float>(-0.03157397429100614);
  result.unsafe_access(149) = static_cast<float>(0.88469332800328);
  result.unsafe_access(150) = static_cast<float>(0.08574085558102314);
  result.unsafe_access(151) = static_cast<float>(1.0);
  result.unsafe_access(152) = static_cast<float>(-12.37671004469203);
  result.unsafe_access(153) = static_cast<float>(-1.5741785964812753);
  result.unsafe_access(154) = static_cast<float>(0.6188355022345998);
  result.unsafe_access(155) = static_cast<float>(0.0903660848083065);
  result.unsafe_access(156) = static_cast<float>(0.005);
  result.unsafe_access(157) = static_cast<float>(-0.06419822718651069);
  result.unsafe_access(158) = static_cast<float>(-0.0646886113663143);
  result.unsafe_access(159) = static_cast<float>(0.853209911359325);
  result.unsafe_access(160) = static_cast<float>(0.08624768563486535);
  result.unsafe_access(161) = static_cast<float>(1.0);
  result.unsafe_access(162) = static_cast<float>(-1.3667782827405264);
  result.unsafe_access(163) = static_cast<float>(-1.7942558055215296);
  result.unsafe_access(164) = static_cast<float>(0.06833891413702448);
  result.unsafe_access(165) = static_cast<float>(0.059218669088577154);
  result.unsafe_access(166) = static_cast<float>(0.005);
  result.unsafe_access(167) = static_cast<float>(1.6535005551160626);
  result.unsafe_access(168) = static_cast<float>(-0.014664108110428828);
  result.unsafe_access(169) = static_cast<float>(0.8173249722441963);
  result.unsafe_access(170) = static_cast<float>(0.08492186152905028);
  result.unsafe_access(171) = static_cast<float>(1.0);
  result.unsafe_access(172) = static_cast<float>(10.007261495336063);
  result.unsafe_access(173) = static_cast<float>(-1.4528326202862742);
  result.unsafe_access(174) = static_cast<float>(-0.5003630747668052);
  result.unsafe_access(175) = static_cast<float>(0.016431605981031);
  result.unsafe_access(176) = static_cast<float>(0.005);
  result.unsafe_access(177) = static_cast<float>(3.2346307369219054);
  result.unsafe_access(178) = static_cast<float>(0.11874412143452423);
  result.unsafe_access(179) = static_cast<float>(0.7882684631539041);
  result.unsafe_access(180) = static_cast<float>(0.08247796076144447);
  result.unsafe_access(181) = static_cast<float>(1.0);
  result.unsafe_access(182) = static_cast<float>(20.23324656177636);
  result.unsafe_access(183) = static_cast<float>(-0.5597785212092805);
  result.unsafe_access(184) = static_cast<float>(-1.0116623280888202);
  result.unsafe_access(185) = static_cast<float>(-0.0329691543868581);
  result.unsafe_access(186) = static_cast<float>(0.005);
  result.unsafe_access(187) = static_cast<float>(4.458541041012721);
  result.unsafe_access(188) = static_cast<float>(0.32453848149283315);
  result.unsafe_access(189) = static_cast<float>(0.7770729479493632);
  result.unsafe_access(190) = static_cast<float>(0.079827623930803);
  result.unsafe_access(191) = static_cast<float>(1.0);
  result.unsafe_access(192) = static_cast<float>(27.85645494406561);
  result.unsafe_access(193) = static_cast<float>(0.7999223464753497);
  result.unsafe_access(194) = static_cast<float>(-1.3928227472032828);
  result.unsafe_access(195) = static_cast<float>(-0.08274048963322453);
  result.unsafe_access(196) = static_cast<float>(0.005);

  return result;
}

} // namespace servo_motor_ltv_constraints_ltv_mpc_F

#endif // SERVO_MOTOR_LTV_CONSTRAINTS_LTV_MPC_F_HPP_
