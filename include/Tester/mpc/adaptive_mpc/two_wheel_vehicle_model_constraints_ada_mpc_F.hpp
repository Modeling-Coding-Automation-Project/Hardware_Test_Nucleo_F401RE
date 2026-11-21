#ifndef TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_ADA_MPC_F_HPP_
#define TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_ADA_MPC_F_HPP_

#include "python_numpy.hpp"

namespace two_wheel_vehicle_model_constraints_ada_mpc_F {

using namespace PythonNumpy;

using SparseAvailable_ada_mpc_F =
    SparseAvailable<ColumnAvailable<true, false, true, false, false, true, true,
                                    false, false, false, false>,
                    ColumnAvailable<false, true, true, false, false, true,
                                    false, true, false, false, false>,
                    ColumnAvailable<false, false, true, true, false, false,
                                    false, false, true, false, false>,
                    ColumnAvailable<false, false, false, true, true, true,
                                    false, false, false, true, false>,
                    ColumnAvailable<false, false, false, false, false, true,
                                    false, false, false, false, true>,
                    ColumnAvailable<true, false, true, true, false, true, true,
                                    false, false, false, false>,
                    ColumnAvailable<false, true, true, true, false, true, false,
                                    true, false, false, false>,
                    ColumnAvailable<false, false, true, true, true, true, false,
                                    false, true, false, false>,
                    ColumnAvailable<false, false, false, true, true, true,
                                    false, false, false, true, false>,
                    ColumnAvailable<false, false, false, false, false, true,
                                    false, false, false, false, true>,
                    ColumnAvailable<true, false, true, true, true, true, true,
                                    false, false, false, false>,
                    ColumnAvailable<false, true, true, true, true, true, false,
                                    true, false, false, false>,
                    ColumnAvailable<false, false, true, true, true, true, false,
                                    false, true, false, false>,
                    ColumnAvailable<false, false, false, true, true, true,
                                    false, false, false, true, false>,
                    ColumnAvailable<false, false, false, false, false, true,
                                    false, false, false, false, true>,
                    ColumnAvailable<true, false, true, true, true, true, true,
                                    false, false, false, false>,
                    ColumnAvailable<false, true, true, true, true, true, false,
                                    true, false, false, false>,
                    ColumnAvailable<false, false, true, true, true, true, false,
                                    false, true, false, false>,
                    ColumnAvailable<false, false, false, true, true, true,
                                    false, false, false, true, false>,
                    ColumnAvailable<false, false, false, false, false, true,
                                    false, false, false, false, true>,
                    ColumnAvailable<true, false, true, true, true, true, true,
                                    false, false, false, false>,
                    ColumnAvailable<false, true, true, true, true, true, false,
                                    true, false, false, false>,
                    ColumnAvailable<false, false, true, true, true, true, false,
                                    false, true, false, false>,
                    ColumnAvailable<false, false, false, true, true, true,
                                    false, false, false, true, false>,
                    ColumnAvailable<false, false, false, false, false, true,
                                    false, false, false, false, true>,
                    ColumnAvailable<true, false, true, true, true, true, true,
                                    false, false, false, false>,
                    ColumnAvailable<false, true, true, true, true, true, false,
                                    true, false, false, false>,
                    ColumnAvailable<false, false, true, true, true, true, false,
                                    false, true, false, false>,
                    ColumnAvailable<false, false, false, true, true, true,
                                    false, false, false, true, false>,
                    ColumnAvailable<false, false, false, false, false, true,
                                    false, false, false, false, true>,
                    ColumnAvailable<true, false, true, true, true, true, true,
                                    false, false, false, false>,
                    ColumnAvailable<false, true, true, true, true, true, false,
                                    true, false, false, false>,
                    ColumnAvailable<false, false, true, true, true, true, false,
                                    false, true, false, false>,
                    ColumnAvailable<false, false, false, true, true, true,
                                    false, false, false, true, false>,
                    ColumnAvailable<false, false, false, false, false, true,
                                    false, false, false, false, true>,
                    ColumnAvailable<true, false, true, true, true, true, true,
                                    false, false, false, false>,
                    ColumnAvailable<false, true, true, true, true, true, false,
                                    true, false, false, false>,
                    ColumnAvailable<false, false, true, true, true, true, false,
                                    false, true, false, false>,
                    ColumnAvailable<false, false, false, true, true, true,
                                    false, false, false, true, false>,
                    ColumnAvailable<false, false, false, false, false, true,
                                    false, false, false, false, true>,
                    ColumnAvailable<true, false, true, true, true, true, true,
                                    false, false, false, false>,
                    ColumnAvailable<false, true, true, true, true, true, false,
                                    true, false, false, false>,
                    ColumnAvailable<false, false, true, true, true, true, false,
                                    false, true, false, false>,
                    ColumnAvailable<false, false, false, true, true, true,
                                    false, false, false, true, false>,
                    ColumnAvailable<false, false, false, false, false, true,
                                    false, false, false, false, true>,
                    ColumnAvailable<true, false, true, true, true, true, true,
                                    false, false, false, false>,
                    ColumnAvailable<false, true, true, true, true, true, false,
                                    true, false, false, false>,
                    ColumnAvailable<false, false, true, true, true, true, false,
                                    false, true, false, false>,
                    ColumnAvailable<false, false, false, true, true, true,
                                    false, false, false, true, false>,
                    ColumnAvailable<false, false, false, false, false, true,
                                    false, false, false, false, true>,
                    ColumnAvailable<true, false, true, true, true, true, true,
                                    false, false, false, false>,
                    ColumnAvailable<false, true, true, true, true, true, false,
                                    true, false, false, false>,
                    ColumnAvailable<false, false, true, true, true, true, false,
                                    false, true, false, false>,
                    ColumnAvailable<false, false, false, true, true, true,
                                    false, false, false, true, false>,
                    ColumnAvailable<false, false, false, false, false, true,
                                    false, false, false, false, true>,
                    ColumnAvailable<true, false, true, true, true, true, true,
                                    false, false, false, false>,
                    ColumnAvailable<false, true, true, true, true, true, false,
                                    true, false, false, false>,
                    ColumnAvailable<false, false, true, true, true, true, false,
                                    false, true, false, false>,
                    ColumnAvailable<false, false, false, true, true, true,
                                    false, false, false, true, false>,
                    ColumnAvailable<false, false, false, false, false, true,
                                    false, false, false, false, true>,
                    ColumnAvailable<true, false, true, true, true, true, true,
                                    false, false, false, false>,
                    ColumnAvailable<false, true, true, true, true, true, false,
                                    true, false, false, false>,
                    ColumnAvailable<false, false, true, true, true, true, false,
                                    false, true, false, false>,
                    ColumnAvailable<false, false, false, true, true, true,
                                    false, false, false, true, false>,
                    ColumnAvailable<false, false, false, false, false, true,
                                    false, false, false, false, true>,
                    ColumnAvailable<true, false, true, true, true, true, true,
                                    false, false, false, false>,
                    ColumnAvailable<false, true, true, true, true, true, false,
                                    true, false, false, false>,
                    ColumnAvailable<false, false, true, true, true, true, false,
                                    false, true, false, false>,
                    ColumnAvailable<false, false, false, true, true, true,
                                    false, false, false, true, false>,
                    ColumnAvailable<false, false, false, false, false, true,
                                    false, false, false, false, true>,
                    ColumnAvailable<true, false, true, true, true, true, true,
                                    false, false, false, false>,
                    ColumnAvailable<false, true, true, true, true, true, false,
                                    true, false, false, false>,
                    ColumnAvailable<false, false, true, true, true, true, false,
                                    false, true, false, false>,
                    ColumnAvailable<false, false, false, true, true, true,
                                    false, false, false, true, false>,
                    ColumnAvailable<false, false, false, false, false, true,
                                    false, false, false, false, true>,
                    ColumnAvailable<true, false, true, true, true, true, true,
                                    false, false, false, false>,
                    ColumnAvailable<false, true, true, true, true, true, false,
                                    true, false, false, false>,
                    ColumnAvailable<false, false, true, true, true, true, false,
                                    false, true, false, false>,
                    ColumnAvailable<false, false, false, true, true, true,
                                    false, false, false, true, false>,
                    ColumnAvailable<false, false, false, false, false, true,
                                    false, false, false, false, true>>;

using type = SparseMatrix_Type<float, SparseAvailable_ada_mpc_F>;

inline auto make(void) -> type {

  auto result = make_SparseMatrixZeros<float, SparseAvailable_ada_mpc_F>();

  result.unsafe_access(0) = static_cast<float>(1.0);
  result.unsafe_access(1) = static_cast<float>(0.0);
  result.unsafe_access(2) = static_cast<float>(0.01);
  result.unsafe_access(3) = static_cast<float>(1.0);
  result.unsafe_access(4) = static_cast<float>(1.0);
  result.unsafe_access(5) = static_cast<float>(0.1);
  result.unsafe_access(6) = static_cast<float>(0.0);
  result.unsafe_access(7) = static_cast<float>(1.0);
  result.unsafe_access(8) = static_cast<float>(0.05);
  result.unsafe_access(9) = static_cast<float>(0.0005);
  result.unsafe_access(10) = static_cast<float>(1.0);
  result.unsafe_access(11) = static_cast<float>(0.009741600000000001);
  result.unsafe_access(12) = static_cast<float>(4e-05);
  result.unsafe_access(13) = static_cast<float>(0.0);
  result.unsafe_access(14) = static_cast<float>(1.0);
  result.unsafe_access(15) = static_cast<float>(1.0);
  result.unsafe_access(16) = static_cast<float>(1.0);
  result.unsafe_access(17) = static_cast<float>(2.0);
  result.unsafe_access(18) = static_cast<float>(0.0);
  result.unsafe_access(19) = static_cast<float>(0.0);
  result.unsafe_access(20) = static_cast<float>(0.03);
  result.unsafe_access(21) = static_cast<float>(1.0);
  result.unsafe_access(22) = static_cast<float>(2.0);
  result.unsafe_access(23) = static_cast<float>(0.30000000000000004);
  result.unsafe_access(24) = static_cast<float>(0.001);
  result.unsafe_access(25) = static_cast<float>(0.0);
  result.unsafe_access(26) = static_cast<float>(1.0);
  result.unsafe_access(27) = static_cast<float>(0.1);
  result.unsafe_access(28) = static_cast<float>(0.00148708);
  result.unsafe_access(29) = static_cast<float>(2e-06);
  result.unsafe_access(30) = static_cast<float>(0.0);
  result.unsafe_access(31) = static_cast<float>(1.0);
  result.unsafe_access(32) = static_cast<float>(0.019231080256000004);
  result.unsafe_access(33) = static_cast<float>(0.00011804640000000002);
  result.unsafe_access(34) = static_cast<float>(0.0);
  result.unsafe_access(35) = static_cast<float>(1.0);
  result.unsafe_access(36) = static_cast<float>(2.0);
  result.unsafe_access(37) = static_cast<float>(1.0);
  result.unsafe_access(38) = static_cast<float>(3.0);
  result.unsafe_access(39) = static_cast<float>(0.0);
  result.unsafe_access(40) = static_cast<float>(0.0);
  result.unsafe_access(41) = static_cast<float>(0.0);
  result.unsafe_access(42) = static_cast<float>(0.060000000000000005);
  result.unsafe_access(43) = static_cast<float>(1.0);
  result.unsafe_access(44) = static_cast<float>(3.0);
  result.unsafe_access(45) = static_cast<float>(0.6);
  result.unsafe_access(46) = static_cast<float>(0.003974160000000001);
  result.unsafe_access(47) = static_cast<float>(4e-06);
  result.unsafe_access(48) = static_cast<float>(0.0);
  result.unsafe_access(49) = static_cast<float>(1.0);
  result.unsafe_access(50) = static_cast<float>(0.15000000000000002);
  result.unsafe_access(51) = static_cast<float>(0.0029486340128);
  result.unsafe_access(52) = static_cast<float>(7.90232e-06);
  result.unsafe_access(53) = static_cast<float>(0.0);
  result.unsafe_access(54) = static_cast<float>(1.0);
  result.unsafe_access(55) = static_cast<float>(0.028474578121896965);
  result.unsafe_access(56) = static_cast<float>(0.00023225565382400005);
  result.unsafe_access(57) = static_cast<float>(0.0);
  result.unsafe_access(58) = static_cast<float>(1.0);
  result.unsafe_access(59) = static_cast<float>(3.0);
  result.unsafe_access(60) = static_cast<float>(1.0);
  result.unsafe_access(61) = static_cast<float>(4.0);
  result.unsafe_access(62) = static_cast<float>(0.0);
  result.unsafe_access(63) = static_cast<float>(0.0);
  result.unsafe_access(64) = static_cast<float>(0.0);
  result.unsafe_access(65) = static_cast<float>(0.09999999999999999);
  result.unsafe_access(66) = static_cast<float>(1.0);
  result.unsafe_access(67) = static_cast<float>(4.0);
  result.unsafe_access(68) = static_cast<float>(1.0);
  result.unsafe_access(69) = static_cast<float>(0.009871428025600003);
  result.unsafe_access(70) = static_cast<float>(1.9804640000000003e-05);
  result.unsafe_access(71) = static_cast<float>(0.0);
  result.unsafe_access(72) = static_cast<float>(1.0);
  result.unsafe_access(73) = static_cast<float>(0.2);
  result.unsafe_access(74) = static_cast<float>(0.004872362918894849);
  result.unsafe_access(75) = static_cast<float>(1.95151026912e-05);
  result.unsafe_access(76) = static_cast<float>(0.0);
  result.unsafe_access(77) = static_cast<float>(1.0);
  result.unsafe_access(78) = static_cast<float>(0.03747809104714122);
  result.unsafe_access(79) = static_cast<float>(0.0003808120862736359);
  result.unsafe_access(80) = static_cast<float>(0.0);
  result.unsafe_access(81) = static_cast<float>(1.0);
  result.unsafe_access(82) = static_cast<float>(4.0);
  result.unsafe_access(83) = static_cast<float>(1.0);
  result.unsafe_access(84) = static_cast<float>(5.0);
  result.unsafe_access(85) = static_cast<float>(0.0);
  result.unsafe_access(86) = static_cast<float>(0.0);
  result.unsafe_access(87) = static_cast<float>(0.0);
  result.unsafe_access(88) = static_cast<float>(0.15);
  result.unsafe_access(89) = static_cast<float>(1.0);
  result.unsafe_access(90) = static_cast<float>(5.0);
  result.unsafe_access(91) = static_cast<float>(1.5);
  result.unsafe_access(92) = static_cast<float>(0.019616153863389697);
  result.unsafe_access(93) = static_cast<float>(5.883484538240002e-05);
  result.unsafe_access(94) = static_cast<float>(0.0);
  result.unsafe_access(95) = static_cast<float>(1.0);
  result.unsafe_access(96) = static_cast<float>(0.25);
  result.unsafe_access(97) = static_cast<float>(0.00724626747125191);
  result.unsafe_access(98) = static_cast<float>(3.85557070048818e-05);
  result.unsafe_access(99) = static_cast<float>(0.0);
  result.unsafe_access(100) = static_cast<float>(1.0);
  result.unsafe_access(101) = static_cast<float>(0.046247479518587264);
  result.unsafe_access(102) = static_cast<float>(0.0005619657724779072);
  result.unsafe_access(103) = static_cast<float>(0.0);
  result.unsafe_access(104) = static_cast<float>(1.0);
  result.unsafe_access(105) = static_cast<float>(5.0);
  result.unsafe_access(106) = static_cast<float>(1.0);
  result.unsafe_access(107) = static_cast<float>(6.0);
  result.unsafe_access(108) = static_cast<float>(0.0);
  result.unsafe_access(109) = static_cast<float>(0.0);
  result.unsafe_access(110) = static_cast<float>(0.0);
  result.unsafe_access(111) = static_cast<float>(0.21000000000000002);
  result.unsafe_access(112) = static_cast<float>(1.0);
  result.unsafe_access(113) = static_cast<float>(6.0);
  result.unsafe_access(114) = static_cast<float>(2.1);
  result.unsafe_access(115) = static_cast<float>(0.03410868880589352);
  result.unsafe_access(116) = static_cast<float>(0.0001359462593921636);
  result.unsafe_access(117) = static_cast<float>(0.0);
  result.unsafe_access(118) = static_cast<float>(1.0);
  result.unsafe_access(119) = static_cast<float>(0.3);
  result.unsafe_access(120) = static_cast<float>(0.010058641447181273);
  result.unsafe_access(121) = static_cast<float>(6.665399562877716e-05);
  result.unsafe_access(122) = static_cast<float>(0.0);
  result.unsafe_access(123) = static_cast<float>(1.0);
  result.unsafe_access(124) = static_cast<float>(0.054788469947364);
  result.unsafe_access(125) = static_cast<float>(0.0007740304777852644);
  result.unsafe_access(126) = static_cast<float>(0.0);
  result.unsafe_access(127) = static_cast<float>(1.0);
  result.unsafe_access(128) = static_cast<float>(6.0);
  result.unsafe_access(129) = static_cast<float>(1.0);
  result.unsafe_access(130) = static_cast<float>(7.0);
  result.unsafe_access(131) = static_cast<float>(0.0);
  result.unsafe_access(132) = static_cast<float>(0.0);
  result.unsafe_access(133) = static_cast<float>(0.0);
  result.unsafe_access(134) = static_cast<float>(0.28);
  result.unsafe_access(135) = static_cast<float>(1.0);
  result.unsafe_access(136) = static_cast<float>(7.0);
  result.unsafe_access(137) = static_cast<float>(2.8000000000000003);
  result.unsafe_access(138) = static_cast<float>(0.054225971700256066);
  result.unsafe_access(139) = static_cast<float>(0.00026925425064971796);
  result.unsafe_access(140) = static_cast<float>(0.0);
  result.unsafe_access(141) = static_cast<float>(1.0);
  result.unsafe_access(142) = static_cast<float>(0.35);
  result.unsafe_access(143) = static_cast<float>(0.013298064944549473);
  result.unsafe_access(144) = static_cast<float>(0.00010535551951804039);
  result.unsafe_access(145) = static_cast<float>(0.0);
  result.unsafe_access(146) = static_cast<float>(1.0);
  result.unsafe_access(147) = static_cast<float>(0.06310665750158449);
  result.unsafe_access(148) = static_cast<float>(0.0010153816565856594);
  result.unsafe_access(149) = static_cast<float>(0.0);
  result.unsafe_access(150) = static_cast<float>(1.0);
  result.unsafe_access(151) = static_cast<float>(7.0);
  result.unsafe_access(152) = static_cast<float>(1.0);
  result.unsafe_access(153) = static_cast<float>(8.0);
  result.unsafe_access(154) = static_cast<float>(0.0);
  result.unsafe_access(155) = static_cast<float>(0.0);
  result.unsafe_access(156) = static_cast<float>(0.0);
  result.unsafe_access(157) = static_cast<float>(0.36000000000000004);
  result.unsafe_access(158) = static_cast<float>(1.0);
  result.unsafe_access(159) = static_cast<float>(8.0);
  result.unsafe_access(160) = static_cast<float>(3.6000000000000005);
  result.unsafe_access(161) = static_cast<float>(0.08082210158935502);
  result.unsafe_access(162) = static_cast<float>(0.00047996528968579874);
  result.unsafe_access(163) = static_cast<float>(0.0);
  result.unsafe_access(164) = static_cast<float>(1.0);
  result.unsafe_access(165) = static_cast<float>(0.39999999999999997);
  result.unsafe_access(166) = static_cast<float>(0.016953397819628697);
  result.unsafe_access(167) = static_cast<float>(0.00015612460234732336);
  result.unsafe_access(168) = static_cast<float>(0.0);
  result.unsafe_access(169) = static_cast<float>(1.0);
  result.unsafe_access(170) = static_cast<float>(0.07120750888571023);
  result.unsafe_access(171) = static_cast<float>(0.0012844545084905274);
  result.unsafe_access(172) = static_cast<float>(0.0);
  result.unsafe_access(173) = static_cast<float>(1.0);
  result.unsafe_access(174) = static_cast<float>(8.0);
  result.unsafe_access(175) = static_cast<float>(1.0);
  result.unsafe_access(176) = static_cast<float>(9.0);
  result.unsafe_access(177) = static_cast<float>(0.0);
  result.unsafe_access(178) = static_cast<float>(0.0);
  result.unsafe_access(179) = static_cast<float>(0.0);
  result.unsafe_access(180) = static_cast<float>(0.45000000000000007);
  result.unsafe_access(181) = static_cast<float>(1.0);
  result.unsafe_access(182) = static_cast<float>(9.0);
  result.unsafe_access(183) = static_cast<float>(4.5);
  result.unsafe_access(184) = static_cast<float>(0.11472889722861242);
  result.unsafe_access(185) = static_cast<float>(0.0007922144943804455);
  result.unsafe_access(186) = static_cast<float>(0.0);
  result.unsafe_access(187) = static_cast<float>(1.0);
  result.unsafe_access(188) = static_cast<float>(0.44999999999999996);
  result.unsafe_access(189) = static_cast<float>(0.02101377326391421);
  result.unsafe_access(190) = static_cast<float>(0.00022034732777184972);
  result.unsafe_access(191) = static_cast<float>(0.0);
  result.unsafe_access(192) = static_cast<float>(1.0);
  result.unsafe_access(193) = static_cast<float>(0.07909636506737927);
  result.unsafe_access(194) = static_cast<float>(0.0015797420903380865);
  result.unsafe_access(195) = static_cast<float>(0.0);
  result.unsafe_access(196) = static_cast<float>(1.0);
  result.unsafe_access(197) = static_cast<float>(9.0);
  result.unsafe_access(198) = static_cast<float>(1.0);
  result.unsafe_access(199) = static_cast<float>(10.0);
  result.unsafe_access(200) = static_cast<float>(0.0);
  result.unsafe_access(201) = static_cast<float>(0.0);
  result.unsafe_access(202) = static_cast<float>(0.0);
  result.unsafe_access(203) = static_cast<float>(0.55);
  result.unsafe_access(204) = static_cast<float>(1.0);
  result.unsafe_access(205) = static_cast<float>(10.0);
  result.unsafe_access(206) = static_cast<float>(5.5);
  result.unsafe_access(207) = static_cast<float>(0.15675644375644082);
  result.unsafe_access(208) = static_cast<float>(0.001232909149924145);
  result.unsafe_access(209) = static_cast<float>(0.0);
  result.unsafe_access(210) = static_cast<float>(1.0);
  result.unsafe_access(211) = static_cast<float>(0.49999999999999994);
  result.unsafe_access(212) = static_cast<float>(0.02546859151728317);
  result.unsafe_access(213) = static_cast<float>(0.00029933443228875406);
  result.unsafe_access(214) = static_cast<float>(0.0);
  result.unsafe_access(215) = static_cast<float>(1.0);
  result.unsafe_access(216) = static_cast<float>(0.08677844395250205);
  result.unsafe_access(217) = static_cast<float>(0.0018997934825298278);
  result.unsafe_access(218) = static_cast<float>(0.0);
  result.unsafe_access(219) = static_cast<float>(1.0);
  result.unsafe_access(220) = static_cast<float>(10.0);
  result.unsafe_access(221) = static_cast<float>(1.0);
  result.unsafe_access(222) = static_cast<float>(11.0);
  result.unsafe_access(223) = static_cast<float>(0.0);
  result.unsafe_access(224) = static_cast<float>(0.0);
  result.unsafe_access(225) = static_cast<float>(0.0);
  result.unsafe_access(226) = static_cast<float>(0.66);
  result.unsafe_access(227) = static_cast<float>(1.0);
  result.unsafe_access(228) = static_cast<float>(11.0);
  result.unsafe_access(229) = static_cast<float>(6.6);
  result.unsafe_access(230) = static_cast<float>(0.20769362679100717);
  result.unsafe_access(231) = static_cast<float>(0.001831578014501653);
  result.unsafe_access(232) = static_cast<float>(0.0);
  result.unsafe_access(233) = static_cast<float>(1.0);
  result.unsafe_access(234) = static_cast<float>(0.5499999999999999);
  result.unsafe_access(235) = static_cast<float>(0.030307513714908273);
  result.unsafe_access(236) = static_cast<float>(0.0003943241064152454);
  result.unsafe_access(237) = static_cast<float>(0.0);
  result.unsafe_access(238) = static_cast<float>(1.0);
  result.unsafe_access(239) = static_cast<float>(0.09425884300942271);
  result.unsafe_access(240) = static_cast<float>(0.0022432120082416505);
  result.unsafe_access(241) = static_cast<float>(0.0);
  result.unsafe_access(242) = static_cast<float>(1.0);
  result.unsafe_access(243) = static_cast<float>(11.0);
  result.unsafe_access(244) = static_cast<float>(1.0);
  result.unsafe_access(245) = static_cast<float>(12.0);
  result.unsafe_access(246) = static_cast<float>(0.0);
  result.unsafe_access(247) = static_cast<float>(0.0);
  result.unsafe_access(248) = static_cast<float>(0.0);
  result.unsafe_access(249) = static_cast<float>(0.78);
  result.unsafe_access(250) = static_cast<float>(1.0);
  result.unsafe_access(251) = static_cast<float>(12.0);
  result.unsafe_access(252) = static_cast<float>(7.799999999999999);
  result.unsafe_access(253) = static_cast<float>(0.26830865422082373);
  result.unsafe_access(254) = static_cast<float>(0.002620226227332144);
  result.unsafe_access(255) = static_cast<float>(0.0);
  result.unsafe_access(256) = static_cast<float>(1.0);
  result.unsafe_access(257) = static_cast<float>(0.6);
  result.unsafe_access(258) = static_cast<float>(0.035520455865379406);
  result.unsafe_access(259) = static_cast<float>(0.0005064847068273279);
  result.unsafe_access(260) = static_cast<float>(0.0);
  result.unsafe_access(261) = static_cast<float>(1.0);
  result.unsafe_access(262) = static_cast<float>(0.10154254184293748);
  result.unsafe_access(263) = static_cast<float>(0.0026086535040897836);
  result.unsafe_access(264) = static_cast<float>(0.0);
  result.unsafe_access(265) = static_cast<float>(1.0);
  result.unsafe_access(266) = static_cast<float>(12.0);
  result.unsafe_access(267) = static_cast<float>(1.0);
  result.unsafe_access(268) = static_cast<float>(13.0);
  result.unsafe_access(269) = static_cast<float>(0.0);
  result.unsafe_access(270) = static_cast<float>(0.0);
  result.unsafe_access(271) = static_cast<float>(0.0);
  result.unsafe_access(272) = static_cast<float>(0.91);
  result.unsafe_access(273) = static_cast<float>(1.0);
  result.unsafe_access(274) = static_cast<float>(13.0);
  result.unsafe_access(275) = static_cast<float>(9.1);
  result.unsafe_access(276) = static_cast<float>(0.33934956595158255);
  result.unsafe_access(277) = static_cast<float>(0.0036331956409867998);
  result.unsafe_access(278) = static_cast<float>(0.0);
  result.unsafe_access(279) = static_cast<float>(1.0);
  result.unsafe_access(280) = static_cast<float>(0.65);
  result.unsafe_access(281) = static_cast<float>(0.04109758295752628);
  result.unsafe_access(282) = static_cast<float>(0.000636917382031817);
  result.unsafe_access(283) = static_cast<float>(0.0);
  result.unsafe_access(284) = static_cast<float>(1.0);
  result.unsafe_access(285) = static_cast<float>(0.10863440471895541);
  result.unsafe_access(286) = static_cast<float>(0.002994824640867469);
  result.unsafe_access(287) = static_cast<float>(0.0);
  result.unsafe_access(288) = static_cast<float>(1.0);
  result.unsafe_access(289) = static_cast<float>(13.0);
  result.unsafe_access(290) = static_cast<float>(1.0);
  result.unsafe_access(291) = static_cast<float>(14.0);
  result.unsafe_access(292) = static_cast<float>(0.0);
  result.unsafe_access(293) = static_cast<float>(0.0);
  result.unsafe_access(294) = static_cast<float>(0.0);
  result.unsafe_access(295) = static_cast<float>(1.05);
  result.unsafe_access(296) = static_cast<float>(1.0);
  result.unsafe_access(297) = static_cast<float>(14.0);
  result.unsafe_access(298) = static_cast<float>(10.5);
  result.unsafe_access(299) = static_cast<float>(0.42154473186663516);
  result.unsafe_access(300) = static_cast<float>(0.004907030405050434);
  result.unsafe_access(301) = static_cast<float>(0.0);
  result.unsafe_access(302) = static_cast<float>(1.0);
  result.unsafe_access(303) = static_cast<float>(0.7000000000000001);
  result.unsafe_access(304) = static_cast<float>(0.04702930319347405);
  result.unsafe_access(305) = static_cast<float>(0.0007866586140751904);
  result.unsafe_access(306) = static_cast<float>(0.0);
  result.unsafe_access(307) = static_cast<float>(1.0);
  result.unsafe_access(308) = static_cast<float>(0.11553918304058022);
  result.unsafe_access(309) = static_cast<float>(0.0034004812930033394);
  result.unsafe_access(310) = static_cast<float>(0.0);
  result.unsafe_access(311) = static_cast<float>(1.0);
  result.unsafe_access(312) = static_cast<float>(14.0);
  result.unsafe_access(313) = static_cast<float>(1.0);
  result.unsafe_access(314) = static_cast<float>(15.0);
  result.unsafe_access(315) = static_cast<float>(0.0);
  result.unsafe_access(316) = static_cast<float>(0.0);
  result.unsafe_access(317) = static_cast<float>(0.0);
  result.unsafe_access(318) = static_cast<float>(1.2);
  result.unsafe_access(319) = static_cast<float>(1.0);
  result.unsafe_access(320) = static_cast<float>(15.0);
  result.unsafe_access(321) = static_cast<float>(12.0);
  result.unsafe_access(322) = static_cast<float>(0.5156033382535833);
  result.unsafe_access(323) = static_cast<float>(0.006480347633200815);
  result.unsafe_access(324) = static_cast<float>(0.0);
  result.unsafe_access(325) = static_cast<float>(1.0);
  result.unsafe_access(326) = static_cast<float>(0.7500000000000001);
  result.unsafe_access(327) = static_cast<float>(0.053306262345503064);
  result.unsafe_access(328) = static_cast<float>(0.0009566826787253573);
  result.unsafe_access(329) = static_cast<float>(0.0);
  result.unsafe_access(330) = static_cast<float>(1.0);
  result.unsafe_access(331) = static_cast<float>(0.12226151777638507);
  result.unsafe_access(332) = static_cast<float>(0.003824426955426584);
  result.unsafe_access(333) = static_cast<float>(0.0);
  result.unsafe_access(334) = static_cast<float>(1.0);
  result.unsafe_access(335) = static_cast<float>(15.0);
  result.unsafe_access(336) = static_cast<float>(1.0);
  result.unsafe_access(337) = static_cast<float>(16.0);
  result.unsafe_access(338) = static_cast<float>(0.0);
  result.unsafe_access(339) = static_cast<float>(0.0);
  result.unsafe_access(340) = static_cast<float>(0.0);
  result.unsafe_access(341) = static_cast<float>(1.3599999999999999);
  result.unsafe_access(342) = static_cast<float>(1.0);
  result.unsafe_access(343) = static_cast<float>(16.0);
  result.unsafe_access(344) = static_cast<float>(13.6);
  result.unsafe_access(345) = static_cast<float>(0.6222158629445893);
  result.unsafe_access(346) = static_cast<float>(0.00839371299065153);
  result.unsafe_access(347) = static_cast<float>(0.0);
  result.unsafe_access(348) = static_cast<float>(1.0);
  result.unsafe_access(349) = static_cast<float>(0.8000000000000002);
  result.unsafe_access(350) = static_cast<float>(0.05991933823432232);
  result.unsafe_access(351) = static_cast<float>(0.0011479040264966865);
  result.unsafe_access(352) = static_cast<float>(0.0);
  result.unsafe_access(353) = static_cast<float>(1.0);
  result.unsafe_access(354) = static_cast<float>(0.12880594184164545);
  result.unsafe_access(355) = static_cast<float>(0.004265511206557313);
  result.unsafe_access(356) = static_cast<float>(0.0);
  result.unsafe_access(357) = static_cast<float>(1.0);
  result.unsafe_access(358) = static_cast<float>(16.0);
  result.unsafe_access(359) = static_cast<float>(1.0);

  return result;
}

} // namespace two_wheel_vehicle_model_constraints_ada_mpc_F

#endif // TWO_WHEEL_VEHICLE_MODEL_CONSTRAINTS_ADA_MPC_F_HPP_
