#ifndef SERVO_MOTOR_CONSTRAINTS_INSTANT_MPC_IMPC_Q_K_MATRIX_HPP_
#define SERVO_MOTOR_CONSTRAINTS_INSTANT_MPC_IMPC_Q_K_MATRIX_HPP_

#include "python_numpy.hpp"

namespace servo_motor_constraints_instant_MPC_impc_q_K_matrix {

using namespace PythonNumpy;

using SparseAvailable_impc_q_K_matrix =
    SparseAvailable<ColumnAvailable<true, true, true, true, true, true>,
                    ColumnAvailable<true, true, true, true, true, true>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, false, true, false, false, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, false, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, false, false, false, false, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, false, false, false, false, false>,
                    ColumnAvailable<true, false, true, false, false, false>,
                    ColumnAvailable<true, true, false, false, true, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, false, false, false, false, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, false, false, false, false, false>,
                    ColumnAvailable<true, false, true, false, false, false>,
                    ColumnAvailable<true, true, false, false, true, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, false, false, false, false, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, false, false, false, false, false>,
                    ColumnAvailable<true, true, false, false, false, false>,
                    ColumnAvailable<true, true, false, false, true, false>,
                    ColumnAvailable<true, false, true, false, false, false>,
                    ColumnAvailable<true, false, false, false, false, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, false, false, false, false, false>,
                    ColumnAvailable<true, true, false, false, false, false>,
                    ColumnAvailable<true, true, false, false, true, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, false, true, false, false, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, false, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, false, false, true, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, false, true, false, false, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, false, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, false, false, true, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, false, true, false, false, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, false, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, false, false, true, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, false, true, false, false, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, false, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, false, false, true, false>,
                    ColumnAvailable<true, false, true, false, false, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<false, false, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, false, false, true, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, true, false, false, false, false>,
                    ColumnAvailable<true, false, true, false, false, false>,
                    ColumnAvailable<true, true, false, false, true, false>,
                    ColumnAvailable<true, true, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, true, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, false, false, true, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, false, false, false, false, false>,
                    ColumnAvailable<true, true, false, false, false, false>,
                    ColumnAvailable<true, true, false, false, true, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, false, false, false, false, false>,
                    ColumnAvailable<true, true, false, false, false, false>,
                    ColumnAvailable<true, true, false, false, true, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, false, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, false, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, false, false, true, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, false, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, false, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, false, false, true, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, false, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, false, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, false, false, true, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<false, true, false, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, false>>;

using type = SparseMatrix_Type<float, SparseAvailable_impc_q_K_matrix>;

inline auto make(void) -> type {

  auto result =
      make_SparseMatrixZeros<float, SparseAvailable_impc_q_K_matrix>();

  result.unsafe_access(0) = static_cast<float>(1.5262257676456104);
  result.unsafe_access(1) = static_cast<float>(-0.051542670370197);
  result.unsafe_access(2) = static_cast<float>(-0.0742303063486768);
  result.unsafe_access(3) = static_cast<float>(-0.0036349935855898344);
  result.unsafe_access(4) = static_cast<float>(0.003007427106549494);
  result.unsafe_access(5) = static_cast<float>(2.831635540507505e-05);
  result.unsafe_access(6) = static_cast<float>(-1.4815539626187135);
  result.unsafe_access(7) = static_cast<float>(0.07681297979087821);
  result.unsafe_access(8) = static_cast<float>(0.07488774874680698);
  result.unsafe_access(9) = static_cast<float>(0.0033194463018423226);
  result.unsafe_access(10) = static_cast<float>(0.0007079156821007335);
  result.unsafe_access(11) = static_cast<float>(-2.5206407142160825e-05);
  result.unsafe_access(12) = static_cast<float>(0.07631128838214615);
  result.unsafe_access(13) = static_cast<float>(-0.0025771335185326466);
  result.unsafe_access(14) = static_cast<float>(-0.003711515317427033);
  result.unsafe_access(15) = static_cast<float>(-0.00018174967927864859);
  result.unsafe_access(16) = static_cast<float>(0.0001503713553279326);
  result.unsafe_access(17) = static_cast<float>(0.003815564419109675);
  result.unsafe_access(18) = static_cast<float>(-0.00012885667592843283);
  result.unsafe_access(19) = static_cast<float>(-0.00018557576587132217);
  result.unsafe_access(20) = static_cast<float>(-0.036685166830820674);
  result.unsafe_access(21) = static_cast<float>(0.002577853566114653);
  result.unsafe_access(22) = static_cast<float>(0.0019257449322123315);
  result.unsafe_access(23) = static_cast<float>(7.691497226330734e-05);
  result.unsafe_access(24) = static_cast<float>(0.00010907774829297854);
  result.unsafe_access(25) = static_cast<float>(-0.24423409166120558);
  result.unsafe_access(26) = static_cast<float>(0.008248109574709696);
  result.unsafe_access(27) = static_cast<float>(0.011878695688322125);
  result.unsafe_access(28) = static_cast<float>(0.0005816894033424487);
  result.unsafe_access(29) = static_cast<float>(-0.00048126315392490554);
  result.unsafe_access(30) = static_cast<float>(0.0004884657844600165);
  result.unsafe_access(31) = static_cast<float>(-1.6496138992889653e-05);
  result.unsafe_access(32) = static_cast<float>(-2.3757274770626692e-05);
  result.unsafe_access(33) = static_cast<float>(0.0019813060777339945);
  result.unsafe_access(34) = static_cast<float>(-8.928851926879959e-05);
  result.unsafe_access(35) = static_cast<float>(-0.019196902218046836);
  result.unsafe_access(36) = static_cast<float>(0.0013043887972605946);
  result.unsafe_access(37) = static_cast<float>(0.0010030084959904164);
  result.unsafe_access(38) = static_cast<float>(4.059678344311194e-05);
  result.unsafe_access(39) = static_cast<float>(5.104178108230231e-05);
  result.unsafe_access(40) = static_cast<float>(-0.37105740422302563);
  result.unsafe_access(41) = static_cast<float>(0.008245805504264259);
  result.unsafe_access(42) = static_cast<float>(0.017594049811888744);
  result.unsafe_access(43) = static_cast<float>(0.0009172126435644027);
  result.unsafe_access(44) = static_cast<float>(-0.0013116293747267801);
  result.unsafe_access(45) = static_cast<float>(2.4423272768020214e-05);
  result.unsafe_access(46) = static_cast<float>(0.0007176879954757959);
  result.unsafe_access(47) = static_cast<float>(-1.566672855609756e-05);
  result.unsafe_access(48) = static_cast<float>(-3.4000069064985386e-05);
  result.unsafe_access(49) = static_cast<float>(0.0010214609666738582);
  result.unsafe_access(50) = static_cast<float>(6.52554422230685e-05);
  result.unsafe_access(51) = static_cast<float>(-3.913809446670082e-05);
  result.unsafe_access(52) = static_cast<float>(1.5524544228404884e-05);
  result.unsafe_access(53) = static_cast<float>(-0.010040598675688574);
  result.unsafe_access(54) = static_cast<float>(0.0006391389909934232);
  result.unsafe_access(55) = static_cast<float>(0.0005200509348721376);
  result.unsafe_access(56) = static_cast<float>(2.157004015457508e-05);
  result.unsafe_access(57) = static_cast<float>(2.0858641519635085e-05);
  result.unsafe_access(58) = static_cast<float>(2.4423265401835894e-05);
  result.unsafe_access(59) = static_cast<float>(-0.4051744103007931);
  result.unsafe_access(60) = static_cast<float>(0.0030128913204899386);
  result.unsafe_access(61) = static_cast<float>(0.018578575844905926);
  result.unsafe_access(62) = static_cast<float>(0.001048338531120475);
  result.unsafe_access(63) = static_cast<float>(-0.0022437437519228177);
  result.unsafe_access(64) = static_cast<float>(6.030767888740222e-05);
  result.unsafe_access(65) = static_cast<float>(0.0007500374591620457);
  result.unsafe_access(66) = static_cast<float>(-3.426911682774713e-05);
  result.unsafe_access(67) = static_cast<float>(0.0005194310333286995);
  result.unsafe_access(68) = static_cast<float>(9.721239174689111e-05);
  result.unsafe_access(69) = static_cast<float>(1.6567476306722165e-05);
  result.unsafe_access(70) = static_cast<float>(-0.005090478420016944);
  result.unsafe_access(71) = static_cast<float>(0.0002870135366213957);
  result.unsafe_access(72) = static_cast<float>(0.00025974759018922795);
  result.unsafe_access(73) = static_cast<float>(8.473094013137059e-05);
  result.unsafe_access(74) = static_cast<float>(-0.3612173371872754);
  result.unsafe_access(75) = static_cast<float>(-0.005268411944554549);
  result.unsafe_access(76) = static_cast<float>(0.01572233765671401);
  result.unsafe_access(77) = static_cast<float>(0.0009967320307114082);
  result.unsafe_access(78) = static_cast<float>(-0.0030777828736372726);
  result.unsafe_access(79) = static_cast<float>(9.780954897407009e-05);
  result.unsafe_access(80) = static_cast<float>(0.0006246220592213134);
  result.unsafe_access(81) = static_cast<float>(-2.6843221199932246e-05);
  result.unsafe_access(82) = static_cast<float>(0.00026490711183745617);
  result.unsafe_access(83) = static_cast<float>(0.00011156306857898108);
  result.unsafe_access(84) = static_cast<float>(1.684548442744711e-05);
  result.unsafe_access(85) = static_cast<float>(-0.0022745490689503717);
  result.unsafe_access(86) = static_cast<float>(9.923011590818725e-05);
  result.unsafe_access(87) = static_cast<float>(0.00011299512854557686);
  result.unsafe_access(88) = static_cast<float>(0.00018254048847167703);
  result.unsafe_access(89) = static_cast<float>(-0.25295834016329727);
  result.unsafe_access(90) = static_cast<float>(-0.01475107320320253);
  result.unsafe_access(91) = static_cast<float>(0.009841213454634474);
  result.unsafe_access(92) = static_cast<float>(0.0007844007785429801);
  result.unsafe_access(93) = static_cast<float>(-0.0036536970536445454);
  result.unsafe_access(94) = static_cast<float>(0.00012904064303630386);
  result.unsafe_access(95) = static_cast<float>(0.0003768741808350653);
  result.unsafe_access(96) = static_cast<float>(3.0712728993272465e-05);
  result.unsafe_access(97) = static_cast<float>(0.00015117965724048497);
  result.unsafe_access(98) = static_cast<float>(0.00011652457424616155);
  result.unsafe_access(99) = static_cast<float>(1.6773198588860922e-05);
  result.unsafe_access(100) = static_cast<float>(-0.0005732340699918878);
  result.unsafe_access(101) = static_cast<float>(2.5961992525029362e-05);
  result.unsafe_access(102) = static_cast<float>(0.000311581141551531);
  result.unsafe_access(103) = static_cast<float>(-0.09743762778304146);
  result.unsafe_access(104) = static_cast<float>(-0.023759786894335132);
  result.unsafe_access(105) = static_cast<float>(0.001880215373349094);
  result.unsafe_access(106) = static_cast<float>(0.00044333913183268594);
  result.unsafe_access(107) = static_cast<float>(-0.003856086817622974);
  result.unsafe_access(108) = static_cast<float>(0.0001478843658243959);
  result.unsafe_access(109) = static_cast<float>(4.6990669762350434e-05);
  result.unsafe_access(110) = static_cast<float>(4.719443886783504e-05);
  result.unsafe_access(111) = static_cast<float>(0.00012251795362302933);
  result.unsafe_access(112) = static_cast<float>(0.00011658504676527534);
  result.unsafe_access(113) = static_cast<float>(1.65937999911727e-05);
  result.unsafe_access(114) = static_cast<float>(0.0004967188404363714);
  result.unsafe_access(115) = static_cast<float>(-4.445093587495363e-05);
  result.unsafe_access(116) = static_cast<float>(-2.7083612684789613e-05);
  result.unsafe_access(117) = static_cast<float>(0.00045946550021205016);
  result.unsafe_access(118) = static_cast<float>(-2.1250962223128248e-05);
  result.unsafe_access(119) = static_cast<float>(0.08404141533522629);
  result.unsafe_access(120) = static_cast<float>(-0.030806451107923908);
  result.unsafe_access(121) = static_cast<float>(-0.007043298727818713);
  result.unsafe_access(122) = static_cast<float>(1.8281611552608254e-05);
  result.unsafe_access(123) = static_cast<float>(-0.0036228125337518817);
  result.unsafe_access(124) = static_cast<float>(0.00015023389127691858);
  result.unsafe_access(125) = static_cast<float>(-0.0003183151467327664);
  result.unsafe_access(126) = static_cast<float>(5.892798743890671e-05);
  result.unsafe_access(127) = static_cast<float>(2.0573423833090183e-05);
  result.unsafe_access(128) = static_cast<float>(0.00014735389391886013);
  result.unsafe_access(129) = static_cast<float>(0.00011436249996672466);
  result.unsafe_access(130) = static_cast<float>(1.6455297227692363e-05);
  result.unsafe_access(131) = static_cast<float>(0.0011507874383719957);
  result.unsafe_access(132) = static_cast<float>(-5.7014279341989874e-05);
  result.unsafe_access(133) = static_cast<float>(-5.788854029634361e-05);
  result.unsafe_access(134) = static_cast<float>(0.000609699395949574);
  result.unsafe_access(135) = static_cast<float>(-2.7737890495120302e-05);
  result.unsafe_access(136) = static_cast<float>(0.26693857553851247);
  result.unsafe_access(137) = static_cast<float>(-0.0346899362302661);
  result.unsafe_access(138) = static_cast<float>(-0.01569642408727462);
  result.unsafe_access(139) = static_cast<float>(-0.0004352330236032628);
  result.unsafe_access(140) = static_cast<float>(-0.002951791088798687);
  result.unsafe_access(141) = static_cast<float>(0.00013431813501827157);
  result.unsafe_access(142) = static_cast<float>(-0.0006681919950781049);
  result.unsafe_access(143) = static_cast<float>(6.374853504937572e-05);
  result.unsafe_access(144) = static_cast<float>(3.68509236310815e-05);
  result.unsafe_access(145) = static_cast<float>(0.00020489326616401308);
  result.unsafe_access(146) = static_cast<float>(0.00011151178600220946);
  result.unsafe_access(147) = static_cast<float>(1.6445749448672693e-05);
  result.unsafe_access(148) = static_cast<float>(0.0014783716602764776);
  result.unsafe_access(149) = static_cast<float>(-4.735442322988024e-05);
  result.unsafe_access(150) = static_cast<float>(-7.163101226363681e-05);
  result.unsafe_access(151) = static_cast<float>(0.0007440175297925193);
  result.unsafe_access(152) = static_cast<float>(-3.319614735108572e-05);
  result.unsafe_access(153) = static_cast<float>(0.42577731343730457);
  result.unsafe_access(154) = static_cast<float>(-0.03461897007656407);
  result.unsafe_access(155) = static_cast<float>(-0.022847373819515723);
  result.unsafe_access(156) = static_cast<float>(-0.0008559878758365298);
  result.unsafe_access(157) = static_cast<float>(-0.0019025844605233722);
  result.unsafe_access(158) = static_cast<float>(0.00010090853123429748);
  result.unsafe_access(159) = static_cast<float>(-0.0009524584653506003);
  result.unsafe_access(160) = static_cast<float>(6.041919232317072e-05);
  result.unsafe_access(161) = static_cast<float>(4.9310215294188856e-05);
  result.unsafe_access(162) = static_cast<float>(0.00027881184875362946);
  result.unsafe_access(163) = static_cast<float>(0.0001091440648640018);
  result.unsafe_access(164) = static_cast<float>(1.6608826411199978e-05);
  result.unsafe_access(165) = static_cast<float>(0.0015185957867400032);
  result.unsafe_access(166) = static_cast<float>(-2.2848836133511267e-05);
  result.unsafe_access(167) = static_cast<float>(-7.085394390273263e-05);
  result.unsafe_access(168) = static_cast<float>(0.00084492606631712);
  result.unsafe_access(169) = static_cast<float>(-3.681185814760243e-05);
  result.unsafe_access(170) = static_cast<float>(0.5371135852494063);
  result.unsafe_access(171) = static_cast<float>(-0.0303159054835235);
  result.unsafe_access(172) = static_cast<float>(-0.027410242651932003);
  result.unsafe_access(173) = static_cast<float>(-0.0011841313276247279);
  result.unsafe_access(174) = static_cast<float>(-0.0005910246428286379);
  result.unsafe_access(175) = static_cast<float>(5.328561395512975e-05);
  result.unsafe_access(176) = static_cast<float>(-0.0011275072281787205);
  result.unsafe_access(177) = static_cast<float>(4.879216121515936e-05);
  result.unsafe_access(178) = static_cast<float>(5.5970409652397515e-05);
  result.unsafe_access(179) = static_cast<float>(0.0003547416385328142);
  result.unsafe_access(180) = static_cast<float>(0.00010800162300070621);
  result.unsafe_access(181) = static_cast<float>(1.695103576591477e-05);
  result.unsafe_access(182) = static_cast<float>(0.0013007933110988116);
  result.unsafe_access(183) = static_cast<float>(-5.7532777506734665e-05);
  result.unsafe_access(184) = static_cast<float>(0.0008982116795827054);
  result.unsafe_access(185) = static_cast<float>(2.3825700119474225e-05);
  result.unsafe_access(186) = static_cast<float>(-3.796205818041429e-05);
  result.unsafe_access(187) = static_cast<float>(0.5826227778207106);
  result.unsafe_access(188) = static_cast<float>(-0.02207228385780114);
  result.unsafe_access(189) = static_cast<float>(-0.028589999279674823);
  result.unsafe_access(190) = static_cast<float>(-0.0013689094152893075);
  result.unsafe_access(191) = static_cast<float>(0.0008234639290694398);
  result.unsafe_access(192) = static_cast<float>(-0.001162150086993218);
  result.unsafe_access(193) = static_cast<float>(2.9865403429788914e-05);
  result.unsafe_access(194) = static_cast<float>(5.553140487818142e-05);
  result.unsafe_access(195) = static_cast<float>(0.00041978130438813236);
  result.unsafe_access(196) = static_cast<float>(0.00010851759266116009);
  result.unsafe_access(197) = static_cast<float>(1.744660830579468e-05);
  result.unsafe_access(198) = static_cast<float>(0.0008649347007879451);
  result.unsafe_access(199) = static_cast<float>(4.6274609988479864e-05);
  result.unsafe_access(200) = static_cast<float>(-3.408984485907657e-05);
  result.unsafe_access(201) = static_cast<float>(0.0008951219325650575);
  result.unsafe_access(202) = static_cast<float>(3.81047174877159e-05);
  result.unsafe_access(203) = static_cast<float>(-3.6313737792423045e-05);
  result.unsafe_access(204) = static_cast<float>(0.5517971050957817);
  result.unsafe_access(205) = static_cast<float>(-0.010738505701856724);
  result.unsafe_access(206) = static_cast<float>(-0.026002959361131874);
  result.unsafe_access(207) = static_cast<float>(-0.0013758823441775505);
  result.unsafe_access(208) = static_cast<float>(0.002156917916840434);
  result.unsafe_access(209) = static_cast<float>(-6.119725213106783e-05);
  result.unsafe_access(210) = static_cast<float>(1.5772287556488726e-05);
  result.unsafe_access(211) = static_cast<float>(-0.0010423918303487665);
  result.unsafe_access(212) = static_cast<float>(4.7580794143680126e-05);
  result.unsafe_access(213) = static_cast<float>(0.00046302803951849003);
  result.unsafe_access(214) = static_cast<float>(0.00011083132320244404);
  result.unsafe_access(215) = static_cast<float>(1.8043061000820124e-05);
  result.unsafe_access(216) = static_cast<float>(0.000269689651495412);
  result.unsafe_access(217) = static_cast<float>(7.934344887574381e-05);
  result.unsafe_access(218) = static_cast<float>(0.000833924681068934);
  result.unsafe_access(219) = static_cast<float>(5.3877005015765926e-05);
  result.unsafe_access(220) = static_cast<float>(-3.18888471551089e-05);
  result.unsafe_access(221) = static_cast<float>(0.44381404409556696);
  result.unsafe_access(222) = static_cast<float>(0.0023588534740151858);
  result.unsafe_access(223) = static_cast<float>(-0.019752252646078217);
  result.unsafe_access(224) = static_cast<float>(-0.00119251314150629);
  result.unsafe_access(225) = static_cast<float>(0.0032242638135921453);
  result.unsafe_access(226) = static_cast<float>(-0.00011331684218366544);
  result.unsafe_access(227) = static_cast<float>(1.6057522306177657e-05);
  result.unsafe_access(228) = static_cast<float>(-0.0007743074390680455);
  result.unsafe_access(229) = static_cast<float>(-2.0775126467385006e-05);
  result.unsafe_access(230) = static_cast<float>(3.270041437196156e-05);
  result.unsafe_access(231) = static_cast<float>(0.00047651252302934987);
  result.unsafe_access(232) = static_cast<float>(0.00011479849558495672);
  result.unsafe_access(233) = static_cast<float>(1.8668686312820646e-05);
  result.unsafe_access(234) = static_cast<float>(-0.00040776737382084);
  result.unsafe_access(235) = static_cast<float>(0.0001043650861937714);
  result.unsafe_access(236) = static_cast<float>(2.9406809282581107e-05);
  result.unsafe_access(237) = static_cast<float>(0.0007206078384361331);
  result.unsafe_access(238) = static_cast<float>(6.99345273413385e-05);
  result.unsafe_access(239) = static_cast<float>(-2.508491683138948e-05);
  result.unsafe_access(240) = static_cast<float>(0.2682443921003338);
  result.unsafe_access(241) = static_cast<float>(0.015567431582121764);
  result.unsafe_access(242) = static_cast<float>(-0.010443839501707292);
  result.unsafe_access(243) = static_cast<float>(-0.000831215378570594);
  result.unsafe_access(244) = static_cast<float>(0.0038643229125702185);
  result.unsafe_access(245) = static_cast<float>(-0.00015203221318727588);
  result.unsafe_access(246) = static_cast<float>(-0.0003844546869785888);
  result.unsafe_access(247) = static_cast<float>(-4.615340114300498e-05);
  result.unsafe_access(248) = static_cast<float>(0.00045612415338504173);
  result.unsafe_access(249) = static_cast<float>(0.00012001674993995727);
  result.unsafe_access(250) = static_cast<float>(1.9242079187701585e-05);
  result.unsafe_access(251) = static_cast<float>(-0.0010776542752667351);
  result.unsafe_access(252) = static_cast<float>(0.00011718178292133093);
  result.unsafe_access(253) = static_cast<float>(6.095140238862636e-05);
  result.unsafe_access(254) = static_cast<float>(0.0005685756240248962);
  result.unsafe_access(255) = static_cast<float>(8.495329334677515e-05);
  result.unsafe_access(256) = static_cast<float>(-1.6645965797851033e-05);
  result.unsafe_access(257) = static_cast<float>(0.04441641738350747);
  result.unsafe_access(258) = static_cast<float>(0.027112174708486418);
  result.unsafe_access(259) = static_cast<float>(0.0008636091488335312);
  result.unsafe_access(260) = static_cast<float>(-0.0003292572993018688);
  result.unsafe_access(261) = static_cast<float>(0.003963159686076686);
  result.unsafe_access(262) = static_cast<float>(-0.0001712549454917181);
  result.unsafe_access(263) = static_cast<float>(8.242168894910077e-05);
  result.unsafe_access(264) = static_cast<float>(-6.693511463314364e-05);
  result.unsafe_access(265) = static_cast<float>(0.0004022414394694439);
  result.unsafe_access(266) = static_cast<float>(0.0001258758392696588);
  result.unsafe_access(267) = static_cast<float>(1.9683056510238188e-05);
  result.unsafe_access(268) = static_cast<float>(-0.0016471904630142207);
  result.unsafe_access(269) = static_cast<float>(0.00011514278907223074);
  result.unsafe_access(270) = static_cast<float>(8.640343015470255e-05);
  result.unsafe_access(271) = static_cast<float>(0.0003973206758285258);
  result.unsafe_access(272) = static_cast<float>(9.76643892409266e-05);
  result.unsafe_access(273) = static_cast<float>(1.582949338617881e-05);
  result.unsafe_access(274) = static_cast<float>(-0.20057145212944286);
  result.unsafe_access(275) = static_cast<float>(0.0353276008843266);
  result.unsafe_access(276) = static_cast<float>(0.012772821523219516);
  result.unsafe_access(277) = static_cast<float>(0.00025468145590875767);
  result.unsafe_access(278) = static_cast<float>(0.003472535996979586);
  result.unsafe_access(279) = static_cast<float>(-0.00016713386431710929);
  result.unsafe_access(280) = static_cast<float>(0.0005682739449308838);
  result.unsafe_access(281) = static_cast<float>(-8.001914661289265e-05);
  result.unsafe_access(282) = static_cast<float>(-3.406743088803773e-05);
  result.unsafe_access(283) = static_cast<float>(0.0003198819152713952);
  result.unsafe_access(284) = static_cast<float>(0.00013163297865114332);
  result.unsafe_access(285) = static_cast<float>(1.992384452789098e-05);
  result.unsafe_access(286) = static_cast<float>(-0.002032062674277935);
  result.unsafe_access(287) = static_cast<float>(9.749709751406633e-05);
  result.unsafe_access(288) = static_cast<float>(0.00010188374264240647);
  result.unsafe_access(289) = static_cast<float>(0.00023018681264356004);
  result.unsafe_access(290) = static_cast<float>(0.00010702872942333264);
  result.unsafe_access(291) = static_cast<float>(1.6004046551162862e-05);
  result.unsafe_access(292) = static_cast<float>(-0.43501168428838155);
  result.unsafe_access(293) = static_cast<float>(0.03888999947288085);
  result.unsafe_access(294) = static_cast<float>(0.023714925473571707);
  result.unsafe_access(295) = static_cast<float>(0.0008470587179253972);
  result.unsafe_access(296) = static_cast<float>(0.002420674475089652);
  result.unsafe_access(297) = static_cast<float>(-0.00013872016689317636);
  result.unsafe_access(298) = static_cast<float>(0.0010087385379837445);
  result.unsafe_access(299) = static_cast<float>(-8.314297102316293e-05);
  result.unsafe_access(300) = static_cast<float>(-5.424816763401144e-05);
  result.unsafe_access(301) = static_cast<float>(0.00021827878262999163);
  result.unsafe_access(302) = static_cast<float>(0.00013650783349454558);
  result.unsafe_access(303) = static_cast<float>(1.9919174766787786e-05);
  result.unsafe_access(304) = static_cast<float>(-0.002167911869001006);
  result.unsafe_access(305) = static_cast<float>(6.558557068704344e-05);
  result.unsafe_access(306) = static_cast<float>(0.00010463355354177035);
  result.unsafe_access(307) = static_cast<float>(9.146664565991676e-05);
  result.unsafe_access(308) = static_cast<float>(0.00011239211228878637);
  result.unsafe_access(309) = static_cast<float>(1.5822620203236755e-05);
  result.unsafe_access(310) = static_cast<float>(-0.6265731173116149);
  result.unsafe_access(311) = static_cast<float>(0.03701833722786499);
  result.unsafe_access(312) = static_cast<float>(0.03215029617008011);
  result.unsafe_access(313) = static_cast<float>(0.001368444030870446);
  result.unsafe_access(314) = static_cast<float>(0.0009133872665047975);
  result.unsafe_access(315) = static_cast<float>(-8.828323993922387e-05);
  result.unsafe_access(316) = static_cast<float>(0.001341422837628652);
  result.unsafe_access(317) = static_cast<float>(-7.524253701141057e-05);
  result.unsafe_access(318) = static_cast<float>(-6.840643088225306e-05);
  result.unsafe_access(319) = static_cast<float>(0.00010988318816128333);
  result.unsafe_access(320) = static_cast<float>(0.00013978711202463297);
  result.unsafe_access(321) = static_cast<float>(1.9653921190128453e-05);
  result.unsafe_access(322) = static_cast<float>(-0.0020200839832632623);
  result.unsafe_access(323) = static_cast<float>(2.277861856180491e-05);
  result.unsafe_access(324) = static_cast<float>(9.344729483916314e-05);
  result.unsafe_access(325) = static_cast<float>(0.00011359834673545715);
  result.unsafe_access(326) = static_cast<float>(1.5408198897460776e-05);
  result.unsafe_access(327) = static_cast<float>(-0.7466268506589606);
  result.unsafe_access(328) = static_cast<float>(0.029614791114783122);
  result.unsafe_access(329) = static_cast<float>(0.03677836495208431);
  result.unsafe_access(330) = static_cast<float>(0.0017438649866966297);
  result.unsafe_access(331) = static_cast<float>(-0.0008752010129011237);

  return result;
}

} // namespace servo_motor_constraints_instant_MPC_impc_q_K_matrix

#endif // SERVO_MOTOR_CONSTRAINTS_INSTANT_MPC_IMPC_Q_K_MATRIX_HPP_
