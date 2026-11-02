#ifndef SERVO_MOTOR_CONSTRAINTS_INSTANT_MPC_IMPC_W_UNC_MAP_HPP_
#define SERVO_MOTOR_CONSTRAINTS_INSTANT_MPC_IMPC_W_UNC_MAP_HPP_

#include "python_numpy.hpp"

namespace servo_motor_constraints_instant_MPC_impc_w_unc_map {

using namespace PythonNumpy;

using SparseAvailable_impc_w_unc_map =
    SparseAvailable<ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
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
                    ColumnAvailable<true, false, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<false, false, true, false, false, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, false, false, false, true, false>,
                    ColumnAvailable<true, true, true, true, false, true>,
                    ColumnAvailable<true, true, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, true, false, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, true>,
                    ColumnAvailable<true, true, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, true, false, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, true>,
                    ColumnAvailable<true, true, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, true>,
                    ColumnAvailable<true, true, false, false, false, false>,
                    ColumnAvailable<true, false, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, true>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, true>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, true>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, false, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, true>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, true>,
                    ColumnAvailable<true, false, true, false, false, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, true>,
                    ColumnAvailable<true, false, true, false, false, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, false, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, true>,
                    ColumnAvailable<true, false, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, true>,
                    ColumnAvailable<true, false, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, true>,
                    ColumnAvailable<true, true, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, false, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, true>,
                    ColumnAvailable<true, false, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, true>,
                    ColumnAvailable<true, false, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, true>,
                    ColumnAvailable<true, false, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, true>,
                    ColumnAvailable<true, true, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, true>,
                    ColumnAvailable<true, true, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, true>,
                    ColumnAvailable<true, true, false, false, false, false>,
                    ColumnAvailable<true, true, true, false, false, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, false, true, false>,
                    ColumnAvailable<true, true, true, true, true, true>>;

using type = SparseMatrix_Type<float, SparseAvailable_impc_w_unc_map>;

inline auto make(void) -> type {

  auto result = make_SparseMatrixZeros<float, SparseAvailable_impc_w_unc_map>();

  result.unsafe_access(0) = static_cast<float>(755.9171493861298);
  result.unsafe_access(1) = static_cast<float>(106.16713961493673);
  result.unsafe_access(2) = static_cast<float>(-63.245994009547594);
  result.unsafe_access(3) = static_cast<float>(-6.84680758954051);
  result.unsafe_access(4) = static_cast<float>(-33.800988010577875);
  result.unsafe_access(5) = static_cast<float>(-144.06235147058908);
  result.unsafe_access(6) = static_cast<float>(152.82115439858808);
  result.unsafe_access(7) = static_cast<float>(-14.659900492284457);
  result.unsafe_access(8) = static_cast<float>(-4.52751930454531);
  result.unsafe_access(9) = static_cast<float>(-27.003395485751415);
  result.unsafe_access(10) = static_cast<float>(1.0000000000000053);
  result.unsafe_access(11) = static_cast<float>(-2.560385384085661);
  result.unsafe_access(12) = static_cast<float>(0.9500002466138071);
  result.unsafe_access(13) = static_cast<float>(0.12801926920428275);
  result.unsafe_access(14) = static_cast<float>(1.0);
  result.unsafe_access(15) = static_cast<float>(44.19685250099569);
  result.unsafe_access(16) = static_cast<float>(5.308356980746836);
  result.unsafe_access(17) = static_cast<float>(-3.482349452061839);
  result.unsafe_access(18) = static_cast<float>(0.14765962052297466);
  result.unsafe_access(19) = static_cast<float>(-1.6900494005288935);
  result.unsafe_access(20) = static_cast<float>(0.9999999999999982);
  result.unsafe_access(21) = static_cast<float>(0.9999999999999982);
  result.unsafe_access(22) = static_cast<float>(1280.1990063378407);
  result.unsafe_access(23) = static_cast<float>(64.00995031689204);
  result.unsafe_access(24) = static_cast<float>(-64.00995031689203);
  result.unsafe_access(25) = static_cast<float>(-3.2004975158446016);
  result.unsafe_access(26) = static_cast<float>(1.0);
  result.unsafe_access(27) = static_cast<float>(0.8719807307957212);
  result.unsafe_access(28) = static_cast<float>(0.09750001233068892);
  result.unsafe_access(29) = static_cast<float>(-4.992752130393436);
  result.unsafe_access(30) = static_cast<float>(0.7744811993620149);
  result.unsafe_access(31) = static_cast<float>(0.24963760651967162);
  result.unsafe_access(32) = static_cast<float>(2.2098426250497845);
  result.unsafe_access(33) = static_cast<float>(0.2654178490373418);
  result.unsafe_access(34) = static_cast<float>(0.825882527396908);
  result.unsafe_access(35) = static_cast<float>(-0.08450247002644468);
  result.unsafe_access(36) = static_cast<float>(20.854335183647677);
  result.unsafe_access(37) = static_cast<float>(10.562202392079804);
  result.unsafe_access(38) = static_cast<float>(-2.7593960077089865);
  result.unsafe_access(39) = static_cast<float>(-0.17002523875023082);
  result.unsafe_access(40) = static_cast<float>(-2.1782939805467336);
  result.unsafe_access(41) = static_cast<float>(1.8719807307957177);
  result.unsafe_access(42) = static_cast<float>(0.1475000123306964);
  result.unsafe_access(43) = static_cast<float>(0.9999999999999964);
  result.unsafe_access(44) = static_cast<float>(2255.0559548106817);
  result.unsafe_access(45) = static_cast<float>(171.83998589047067);
  result.unsafe_access(46) = static_cast<float>(-108.68014280178677);
  result.unsafe_access(47) = static_cast<float>(-6.873579280363533);
  result.unsafe_access(48) = static_cast<float>(5.40899890804638);
  result.unsafe_access(49) = static_cast<float>(0.9959048909829048);
  result.unsafe_access(50) = static_cast<float>(0.6223431242760497);
  result.unsafe_access(51) = static_cast<float>(0.13622407229878966);
  result.unsafe_access(52) = static_cast<float>(-6.692820035573722);
  result.unsafe_access(53) = static_cast<float>(0.520098322939537);
  result.unsafe_access(54) = static_cast<float>(0.32649573207610905);
  result.unsafe_access(55) = static_cast<float>(3.252559384232168);
  result.unsafe_access(56) = static_cast<float>(0.793527968641332);
  result.unsafe_access(57) = static_cast<float>(0.6879127270114587);
  result.unsafe_access(58) = static_cast<float>(-0.19341716905378137);
  result.unsafe_access(59) = static_cast<float>(15.092908982351561);
  result.unsafe_access(60) = static_cast<float>(5.714629349986999);
  result.unsafe_access(61) = static_cast<float>(-1.5754550062018773);
  result.unsafe_access(62) = static_cast<float>(-0.10167777581020776);
  result.unsafe_access(63) = static_cast<float>(-1.0403190559276678);
  result.unsafe_access(64) = static_cast<float>(2.4943238550717695);
  result.unsafe_access(65) = static_cast<float>(0.2837240846294793);
  result.unsafe_access(66) = static_cast<float>(0.9999999999999964);
  result.unsafe_access(67) = static_cast<float>(2843.5828395226254);
  result.unsafe_access(68) = static_cast<float>(295.4402220388795);
  result.unsafe_access(69) = static_cast<float>(-128.53960442822535);
  result.unsafe_access(70) = static_cast<float>(-9.592770337562914);
  result.unsafe_access(71) = static_cast<float>(17.789622289611955);
  result.unsafe_access(72) = static_cast<float>(0.9881426379330857);
  result.unsafe_access(73) = static_cast<float>(0.28770212249736427);
  result.unsafe_access(74) = static_cast<float>(0.16222898844576655);
  result.unsafe_access(75) = static_cast<float>(-7.535228648206532);
  result.unsafe_access(76) = static_cast<float>(0.246894282020161);
  result.unsafe_access(77) = static_cast<float>(0.34988975333883876);
  result.unsafe_access(78) = static_cast<float>(4.007204833349746);
  result.unsafe_access(79) = static_cast<float>(1.079259436140682);
  result.unsafe_access(80) = static_cast<float>(0.6091399767013648);
  result.unsafe_access(81) = static_cast<float>(-0.24543312185016478);
  result.unsafe_access(82) = static_cast<float>(10.338159824911989);
  result.unsafe_access(83) = static_cast<float>(3.4181695622356743);
  result.unsafe_access(84) = static_cast<float>(-0.8712702611711131);
  result.unsafe_access(85) = static_cast<float>(-0.06341806543299872);
  result.unsafe_access(86) = static_cast<float>(-0.4478532204967295);
  result.unsafe_access(87) = static_cast<float>(2.7820259775691283);
  result.unsafe_access(88) = static_cast<float>(0.4459530730752431);
  result.unsafe_access(89) = static_cast<float>(0.06049143763641253);
  result.unsafe_access(90) = static_cast<float>(0.9994591027770551);
  result.unsafe_access(91) = static_cast<float>(2955.397828572711);
  result.unsafe_access(92) = static_cast<float>(434.0422689599408);
  result.unsafe_access(93) = static_cast<float>(-122.45785063212085);
  result.unsafe_access(94) = static_cast<float>(-11.127077496184782);
  result.unsafe_access(95) = static_cast<float>(32.80732813802509);
  result.unsafe_access(96) = static_cast<float>(0.9791732691020943);
  result.unsafe_access(97) = static_cast<float>(-0.0890593099129663);
  result.unsafe_access(98) = static_cast<float>(0.17457370254677493);
  result.unsafe_access(99) = static_cast<float>(-7.382097949182996);
  result.unsafe_access(100) = static_cast<float>(0.3202319043611101);
  result.unsafe_access(101) = static_cast<float>(-0.06332153800538592);
  result.unsafe_access(102) = static_cast<float>(4.5241128245953455);
  result.unsafe_access(103) = static_cast<float>(1.2501679142524658);
  result.unsafe_access(104) = static_cast<float>(0.5655764636428091);
  result.unsafe_access(105) = static_cast<float>(-0.26782578287500125);
  result.unsafe_access(106) = static_cast<float>(5.6247732594573066);
  result.unsafe_access(107) = static_cast<float>(2.3679133201007874);
  result.unsafe_access(108) = static_cast<float>(-0.396513658993323);
  result.unsafe_access(109) = static_cast<float>(-0.14435954880133184);
  result.unsafe_access(110) = static_cast<float>(2.692966667656168);
  result.unsafe_access(111) = static_cast<float>(0.6205267756220171);
  result.unsafe_access(112) = static_cast<float>(0.11319355569335834);
  result.unsafe_access(113) = static_cast<float>(0.9971662968272463);
  result.unsafe_access(114) = static_cast<float>(2551.795951386641);
  result.unsafe_access(115) = static_cast<float>(577.5081634139666);
  result.unsafe_access(116) = static_cast<float>(-91.19117280190025);
  result.unsafe_access(117) = static_cast<float>(-11.293815021807063);
  result.unsafe_access(118) = static_cast<float>(47.0155952947623);
  result.unsafe_access(119) = static_cast<float>(0.9709715033057426);
  result.unsafe_access(120) = static_cast<float>(-0.4581642073721164);
  result.unsafe_access(121) = static_cast<float>(0.17244104765736026);
  result.unsafe_access(122) = static_cast<float>(0.06871371327499917);
  result.unsafe_access(123) = static_cast<float>(-6.205795099230789);
  result.unsafe_access(124) = static_cast<float>(-0.3274508270989582);
  result.unsafe_access(125) = static_cast<float>(0.24168734088800323);
  result.unsafe_access(126) = static_cast<float>(-0.08857187087635905);
  result.unsafe_access(127) = static_cast<float>(4.805351487568211);
  result.unsafe_access(128) = static_cast<float>(1.3685635802575051);
  result.unsafe_access(129) = static_cast<float>(0.545750780693143);
  result.unsafe_access(130) = static_cast<float>(-0.27504376031506783);
  result.unsafe_access(131) = static_cast<float>(0.7381295112037018);
  result.unsafe_access(132) = static_cast<float>(1.8776069991195168);
  result.unsafe_access(133) = static_cast<float>(2.234802460284056);
  result.unsafe_access(134) = static_cast<float>(0.7929678232793831);
  result.unsafe_access(135) = static_cast<float>(0.18190726896835946);
  result.unsafe_access(136) = static_cast<float>(0.9917074139771686);
  result.unsafe_access(137) = static_cast<float>(1657.6642783948487);
  result.unsafe_access(138) = static_cast<float>(710.6653344989846);
  result.unsafe_access(139) = static_cast<float>(-38.15742570303609);
  result.unsafe_access(140) = static_cast<float>(-10.033753402447037);
  result.unsafe_access(141) = static_cast<float>(57.63267632711098);
  result.unsafe_access(142) = static_cast<float>(0.9651238564443269);
  result.unsafe_access(143) = static_cast<float>(-0.7684539623336554);
  result.unsafe_access(144) = static_cast<float>(0.15606850630241262);
  result.unsafe_access(145) = static_cast<float>(0.08079808031939935);
  result.unsafe_access(146) = static_cast<float>(-4.107252348929051);
  result.unsafe_access(147) = static_cast<float>(-0.5773913951320663);
  result.unsafe_access(148) = static_cast<float>(0.12353606240348076);
  result.unsafe_access(149) = static_cast<float>(-0.1053773565075339);
  result.unsafe_access(150) = static_cast<float>(4.842257963128396);
  result.unsafe_access(151) = static_cast<float>(1.462443930213481);
  result.unsafe_access(152) = static_cast<float>(0.5438528655053614);
  result.unsafe_access(153) = static_cast<float>(-0.27502850247151656);
  result.unsafe_access(154) = static_cast<float>(-4.108974904469146);
  result.unsafe_access(155) = static_cast<float>(1.5858132849936537);
  result.unsafe_access(156) = static_cast<float>(0.24656916665406076);
  result.unsafe_access(157) = static_cast<float>(1.4663484979503971);
  result.unsafe_access(158) = static_cast<float>(0.9490363295817943);
  result.unsafe_access(159) = static_cast<float>(0.2627053492877591);
  result.unsafe_access(160) = static_cast<float>(0.981819937583272);
  result.unsafe_access(161) = static_cast<float>(363.9375877575027);
  result.unsafe_access(162) = static_cast<float>(816.8531178737587);
  result.unsafe_access(163) = static_cast<float>(30.468201535166443);
  result.unsafe_access(164) = static_cast<float>(-7.451261993021666);
  result.unsafe_access(165) = static_cast<float>(62.579299651382115);
  result.unsafe_access(166) = static_cast<float>(0.9627910557040167);
  result.unsafe_access(167) = static_cast<float>(-0.9738165797801087);
  result.unsafe_access(168) = static_cast<float>(0.12719893654580935);
  result.unsafe_access(169) = static_cast<float>(0.08697488343957338);
  result.unsafe_access(170) = static_cast<float>(-1.3144501251476173);
  result.unsafe_access(171) = static_cast<float>(-0.7608964870227617);
  result.unsafe_access(172) = static_cast<float>(-0.11000171252198616);
  result.unsafe_access(173) = static_cast<float>(4.636809217904938);
  result.unsafe_access(174) = static_cast<float>(1.5417345944631637);
  result.unsafe_access(175) = static_cast<float>(0.5561813238380644);
  result.unsafe_access(176) = static_cast<float>(-0.27236675587008924);
  result.unsafe_access(177) = static_cast<float>(-8.482031156376609);
  result.unsafe_access(178) = static_cast<float>(1.3079874265207623);
  result.unsafe_access(179) = static_cast<float>(0.4639470278515025);
  result.unsafe_access(180) = static_cast<float>(0.49253191817029124);
  result.unsafe_access(181) = static_cast<float>(1.0762352661275958);
  result.unsafe_access(182) = static_cast<float>(0.3496802327273314);
  result.unsafe_access(183) = static_cast<float>(0.9666635933639979);
  result.unsafe_access(184) = static_cast<float>(-1179.5433576993082);
  result.unsafe_access(185) = static_cast<float>(881.006715253513);
  result.unsafe_access(186) = static_cast<float>(106.21222198480007);
  result.unsafe_access(187) = static_cast<float>(-3.8231169792627027);
  result.unsafe_access(188) = static_cast<float>(60.61034535336707);
  result.unsafe_access(189) = static_cast<float>(0.964626801524282);
  result.unsafe_access(190) = static_cast<float>(-1.0395390860374902);
  result.unsafe_access(191) = static_cast<float>(0.08915411219467127);
  result.unsafe_access(192) = static_cast<float>(0.08598031905125778);
  result.unsafe_access(193) = static_cast<float>(1.8382187221133615);
  result.unsafe_access(194) = static_cast<float>(-0.8511584122321969);
  result.unsafe_access(195) = static_cast<float>(-0.17038402200188063);
  result.unsafe_access(196) = static_cast<float>(-0.10056376485065043);
  result.unsafe_access(197) = static_cast<float>(4.2127076600861075);
  result.unsafe_access(198) = static_cast<float>(1.6071339657892019);
  result.unsafe_access(199) = static_cast<float>(0.5793786752306396);
  result.unsafe_access(200) = static_cast<float>(-0.2698258442043221);
  result.unsafe_access(201) = static_cast<float>(-11.873599993908599);
  result.unsafe_access(202) = static_cast<float>(0.9616818258939462);
  result.unsafe_access(203) = static_cast<float>(0.6060541458954043);
  result.unsafe_access(204) = static_cast<float>(-0.5470071678671986);
  result.unsafe_access(205) = static_cast<float>(1.165389378322266);
  result.unsafe_access(206) = static_cast<float>(0.43566055177858853);
  result.unsafe_access(207) = static_cast<float>(0.9460071635186216);
  result.unsafe_access(208) = static_cast<float>(-2780.0154707155516);
  result.unsafe_access(209) = static_cast<float>(892.269155793307);
  result.unsafe_access(210) = static_cast<float>(179.1981407826508);
  result.unsafe_access(211) = static_cast<float>(0.42144218738641037);
  result.unsafe_access(212) = static_cast<float>(51.43754327256127);
  result.unsafe_access(213) = static_cast<float>(0.9707093504878956);
  result.unsafe_access(214) = static_cast<float>(-0.9476281499318175);
  result.unsafe_access(215) = static_cast<float>(0.0774611179511637);
  result.unsafe_access(216) = static_cast<float>(4.947236677429471);
  result.unsafe_access(217) = static_cast<float>(-0.831125471508658);
  result.unsafe_access(218) = static_cast<float>(-0.30783598054312894);
  result.unsafe_access(219) = static_cast<float>(-0.07719008773250487);
  result.unsafe_access(220) = static_cast<float>(3.6190276603906772);
  result.unsafe_access(221) = static_cast<float>(1.6552180570838992);
  result.unsafe_access(222) = static_cast<float>(0.6096813825254098);
  result.unsafe_access(223) = static_cast<float>(-0.26907303606259997);
  result.unsafe_access(224) = static_cast<float>(-13.820424562096434);
  result.unsafe_access(225) = static_cast<float>(0.5275362973870044);
  result.unsafe_access(226) = static_cast<float>(0.6618961254780018);
  result.unsafe_access(227) = static_cast<float>(-1.4946353177990153);
  result.unsafe_access(228) = static_cast<float>(1.2119855699053237);
  result.unsafe_access(229) = static_cast<float>(0.5131216697297548);
  result.unsafe_access(230) = static_cast<float>(0.9203225454307126);
  result.unsafe_access(231) = static_cast<float>(-4224.821867373102);
  result.unsafe_access(232) = static_cast<float>(845.971128359507);
  result.unsafe_access(233) = static_cast<float>(239.33811200896335);
  result.unsafe_access(234) = static_cast<float>(4.745612415701979);
  result.unsafe_access(235) = static_cast<float>(35.779472388235945);
  result.unsafe_access(236) = static_cast<float>(0.980520651377669);
  result.unsafe_access(237) = static_cast<float>(-0.700266316060347);
  result.unsafe_access(238) = static_cast<float>(0.062069318924007126);
  result.unsafe_access(239) = static_cast<float>(7.589474604561726);
  result.unsafe_access(240) = static_cast<float>(-0.6969738047419519);
  result.unsafe_access(241) = static_cast<float>(-0.4127236066313105);
  result.unsafe_access(242) = static_cast<float>(2.9280064322858554);
  result.unsafe_access(243) = static_cast<float>(1.6815948719532494);
  result.unsafe_access(244) = static_cast<float>(0.6427761887993099);
  result.unsafe_access(245) = static_cast<float>(-0.27099736059335755);
  result.unsafe_access(246) = static_cast<float>(-13.996040018714975);
  result.unsafe_access(247) = static_cast<float>(0.6250289576157835);
  result.unsafe_access(248) = static_cast<float>(-0.09714873482305131);
  result.unsafe_access(249) = static_cast<float>(-2.1949016338593594);
  result.unsafe_access(250) = static_cast<float>(1.2170254879129478);
  result.unsafe_access(251) = static_cast<float>(0.575190988653761);
  result.unsafe_access(252) = static_cast<float>(0.8907784229561806);
  result.unsafe_access(253) = static_cast<float>(-5308.723655623572);
  result.unsafe_access(254) = static_cast<float>(744.7844221780285);
  result.unsafe_access(255) = static_cast<float>(277.65512050961894);
  result.unsafe_access(256) = static_cast<float>(8.568920766282385);
  result.unsafe_access(257) = static_cast<float>(15.30364374080378);
  result.unsafe_access(258) = static_cast<float>(0.9929915121895392);
  result.unsafe_access(259) = static_cast<float>(-0.3207925858322599);
  result.unsafe_access(260) = static_cast<float>(9.377795630297962);
  result.unsafe_access(261) = static_cast<float>(-0.4597528721868144);
  result.unsafe_access(262) = static_cast<float>(-0.4687211671042623);
  result.unsafe_access(263) = static_cast<float>(2.2282044313501066);
  result.unsafe_access(264) = static_cast<float>(1.6829450043807808);
  result.unsafe_access(265) = static_cast<float>(0.674027636680099);
  result.unsafe_access(266) = static_cast<float>(-0.2758547973345101);
  result.unsafe_access(267) = static_cast<float>(-12.277568550422714);
  result.unsafe_access(268) = static_cast<float>(-0.4927022331175832);
  result.unsafe_access(269) = static_cast<float>(0.497849231735011);
  result.unsafe_access(270) = static_cast<float>(-0.14998202330045593);
  result.unsafe_access(271) = static_cast<float>(-2.5156942196916177);
  result.unsafe_access(272) = static_cast<float>(1.1872167156834703);
  result.unsafe_access(273) = static_cast<float>(0.6166241272462021);
  result.unsafe_access(274) = static_cast<float>(0.8591335697279839);
  result.unsafe_access(275) = static_cast<float>(-5862.029260193178);
  result.unsafe_access(276) = static_cast<float>(598.8980354732353);
  result.unsafe_access(277) = static_cast<float>(287.5533078290154);
  result.unsafe_access(278) = static_cast<float>(11.34194051217135);
  result.unsafe_access(279) = static_cast<float>(-7.55061404558049);
  result.unsafe_access(280) = static_cast<float>(1.0066173887268342);
  result.unsafe_access(281) = static_cast<float>(0.14809719568263985);
  result.unsafe_access(282) = static_cast<float>(10.01551391250409);
  result.unsafe_access(283) = static_cast<float>(-0.14499400765318207);
  result.unsafe_access(284) = static_cast<float>(-0.4650815013397671);
  result.unsafe_access(285) = static_cast<float>(1.6143260038289706);
  result.unsafe_access(286) = static_cast<float>(1.6583098927249016);
  result.unsafe_access(287) = static_cast<float>(0.6989200982668495);
  result.unsafe_access(288) = static_cast<float>(-0.2833538984995329);
  result.unsafe_access(289) = static_cast<float>(-8.782536612555154);
  result.unsafe_access(290) = static_cast<float>(-0.9708560277515803);
  result.unsafe_access(291) = static_cast<float>(0.29343706014713766);
  result.unsafe_access(292) = static_cast<float>(-0.1877624803491448);
  result.unsafe_access(293) = static_cast<float>(-2.3675970240089783);
  result.unsafe_access(294) = static_cast<float>(1.1344202998446629);
  result.unsafe_access(295) = static_cast<float>(0.6346212074834336);
  result.unsafe_access(296) = static_cast<float>(0.8275405945312431);
  result.unsafe_access(297) = static_cast<float>(-5775.768304739203);
  result.unsafe_access(298) = static_cast<float>(425.15978253485986);
  result.unsafe_access(299) = static_cast<float>(265.8553113001593);
  result.unsafe_access(300) = static_cast<float>(12.622046496085732);
  result.unsafe_access(301) = static_cast<float>(-29.85844053465531);
  result.unsafe_access(302) = static_cast<float>(1.0196369998732764);
  result.unsafe_access(303) = static_cast<float>(0.6488728913078434);
  result.unsafe_access(304) = static_cast<float>(-0.06004611622146924);
  result.unsafe_access(305) = static_cast<float>(9.34221962686065);
  result.unsafe_access(306) = static_cast<float>(0.20973044899862536);
  result.unsafe_access(307) = static_cast<float>(-0.39843176195160734);
  result.unsafe_access(308) = static_cast<float>(0.08897471876705079);
  result.unsafe_access(309) = static_cast<float>(1.175199173201213);
  result.unsafe_access(310) = static_cast<float>(1.6097670913373225);
  result.unsafe_access(311) = static_cast<float>(0.7135919512742064);
  result.unsafe_access(312) = static_cast<float>(-0.29274202251699016);
  result.unsafe_access(313) = static_cast<float>(-3.872138162882139);
  result.unsafe_access(314) = static_cast<float>(-1.3444107182901524);
  result.unsafe_access(315) = static_cast<float>(-0.20354274781645504);
  result.unsafe_access(316) = static_cast<float>(-1.7187241327011342);
  result.unsafe_access(317) = static_cast<float>(1.0743741836231921);
  result.unsafe_access(318) = static_cast<float>(0.6293642126536748);
  result.unsafe_access(319) = static_cast<float>(0.7982823179834625);
  result.unsafe_access(320) = static_cast<float>(-5020.306314736401);
  result.unsafe_access(321) = static_cast<float>(245.24769267541868);
  result.unsafe_access(322) = static_cast<float>(213.44832639518154);
  result.unsafe_access(323) = static_cast<float>(12.139847927929521);
  result.unsafe_access(324) = static_cast<float>(-48.576454781306744);
  result.unsafe_access(325) = static_cast<float>(1.0302553828145682);
  result.unsafe_access(326) = static_cast<float>(1.1159838726508766);
  result.unsafe_access(327) = static_cast<float>(7.364194421726658);
  result.unsafe_access(328) = static_cast<float>(0.5590663832377852);
  result.unsafe_access(329) = static_cast<float>(-0.27369681927428735);
  result.unsafe_access(330) = static_cast<float>(0.12196184861956517);
  result.unsafe_access(331) = static_cast<float>(0.9815922650571062);
  result.unsafe_access(332) = static_cast<float>(1.5425465554228148);
  result.unsafe_access(333) = static_cast<float>(0.7153566601155973);
  result.unsafe_access(334) = static_cast<float>(-0.30291915990781293);
  result.unsafe_access(335) = static_cast<float>(1.8799622502017523);
  result.unsafe_access(336) = static_cast<float>(-1.558321701259381);
  result.unsafe_access(337) = static_cast<float>(-0.2447407778792584);
  result.unsafe_access(338) = static_cast<float>(-0.1933260176633202);
  result.unsafe_access(339) = static_cast<float>(-0.6027402600502589);
  result.unsafe_access(340) = static_cast<float>(1.024814589851655);
  result.unsafe_access(341) = static_cast<float>(0.6041856297263342);
  result.unsafe_access(342) = static_cast<float>(0.7734727773740315);
  result.unsafe_access(343) = static_cast<float>(-3654.456541997445);
  result.unsafe_access(344) = static_cast<float>(83.06322160048138);
  result.unsafe_access(345) = static_cast<float>(135.42478527774838);
  result.unsafe_access(346) = static_cast<float>(9.846625239461936);
  result.unsafe_access(347) = static_cast<float>(-60.947763641465066);
  result.unsafe_access(348) = static_cast<float>(1.0368849950995573);
  result.unsafe_access(349) = static_cast<float>(1.4841935937372097);
  result.unsafe_access(350) = static_cast<float>(4.264300444770919);
  result.unsafe_access(351) = static_cast<float>(0.855480544222066);
  result.unsafe_access(352) = static_cast<float>(-0.10396573324042145);
  result.unsafe_access(353) = static_cast<float>(0.14060628194895636);
  result.unsafe_access(354) = static_cast<float>(1.0755903775671938);
  result.unsafe_access(355) = static_cast<float>(1.4646304703598458);
  result.unsafe_access(356) = static_cast<float>(0.7031196212216344);
  result.unsafe_access(357) = static_cast<float>(-0.31258546079097893);
  result.unsafe_access(358) = static_cast<float>(7.750430366293641);
  result.unsafe_access(359) = static_cast<float>(-1.5744999889917828);
  result.unsafe_access(360) = static_cast<float>(-0.5100406867480024);
  result.unsafe_access(361) = static_cast<float>(-0.15658629295581855);
  result.unsafe_access(362) = static_cast<float>(0.8814533336869498);
  result.unsafe_access(363) = static_cast<float>(1.0032083152420064);
  result.unsafe_access(364) = static_cast<float>(0.5653222058352797);
  result.unsafe_access(365) = static_cast<float>(0.7547613291955785);
  result.unsafe_access(366) = static_cast<float>(-1823.2418647114841);
  result.unsafe_access(367) = static_cast<float>(-38.3480333257933);
  result.unsafe_access(368) = static_cast<float>(40.665216608305144);
  result.unsafe_access(369) = static_cast<float>(5.935088413084751);
  result.unsafe_access(370) = static_cast<float>(-64.89356119164913);
  result.unsafe_access(371) = static_cast<float>(1.038373912359902);
  result.unsafe_access(372) = static_cast<float>(1.6974086159757555);
  result.unsafe_access(373) = static_cast<float>(0.3886751837087335);
  result.unsafe_access(374) = static_cast<float>(1.0555280401691547);
  result.unsafe_access(375) = static_cast<float>(0.09075073036038193);
  result.unsafe_access(376) = static_cast<float>(0.14146755870380087);
  result.unsafe_access(377) = static_cast<float>(1.4631118958818758);
  result.unsafe_access(378) = static_cast<float>(1.3859054709102567);
  result.unsafe_access(379) = static_cast<float>(0.6776175868842342);
  result.unsafe_access(380) = static_cast<float>(-0.32041477543876984);
  result.unsafe_access(381) = static_cast<float>(12.95378426591369);
  result.unsafe_access(382) = static_cast<float>(-1.3785612692373475);
  result.unsafe_access(383) = static_cast<float>(-0.7237177798537375);
  result.unsafe_access(384) = static_cast<float>(-0.09645627129927127);
  result.unsafe_access(385) = static_cast<float>(2.578861949662693);
  result.unsafe_access(386) = static_cast<float>(1.0243760678434572);
  result.unsafe_access(387) = static_cast<float>(0.521260495282202);
  result.unsafe_access(388) = static_cast<float>(0.743080195114573);
  result.unsafe_access(389) = static_cast<float>(256.1252390465148);
  result.unsafe_access(390) = static_cast<float>(-99.9608378158803);
  result.unsafe_access(391) = static_cast<float>(-59.1168095296001);
  result.unsafe_access(392) = static_cast<float>(0.828906129417494);
  result.unsafe_access(393) = static_cast<float>(-59.33800357841713);
  result.unsafe_access(394) = static_cast<float>(1.0341897021233017);
  result.unsafe_access(395) = static_cast<float>(1.7168423751611923);
  result.unsafe_access(396) = static_cast<float>(0.07394415460991234);
  result.unsafe_access(397) = static_cast<float>(-3.789472175114536);
  result.unsafe_access(398) = static_cast<float>(1.1259768996659283);
  result.unsafe_access(399) = static_cast<float>(0.28577628419354323);
  result.unsafe_access(400) = static_cast<float>(0.1232831552330886);
  result.unsafe_access(401) = static_cast<float>(2.1108011091775603);
  result.unsafe_access(402) = static_cast<float>(1.3169774074483893);
  result.unsafe_access(403) = static_cast<float>(0.6414316978915473);
  result.unsafe_access(404) = static_cast<float>(-0.3252375890037334);
  result.unsafe_access(405) = static_cast<float>(16.744189809087704);
  result.unsafe_access(406) = static_cast<float>(-0.9835590443767348);
  result.unsafe_access(407) = static_cast<float>(-0.8535318428178577);
  result.unsafe_access(408) = static_cast<float>(4.295704324823869);
  result.unsafe_access(409) = static_cast<float>(1.0983202224533668);
  result.unsafe_access(410) = static_cast<float>(0.4817363212471441);
  result.unsafe_access(411) = static_cast<float>(0.7384724389687566);
  result.unsafe_access(412) = static_cast<float>(2318.912867639275);
  result.unsafe_access(413) = static_cast<float>(-89.59726297901946);
  result.unsafe_access(414) = static_cast<float>(-150.77362896932118);
  result.unsafe_access(415) = static_cast<float>(-4.8596090678324355);
  result.unsafe_access(416) = static_cast<float>(-44.41840650442189);
  result.unsafe_access(417) = static_cast<float>(1.0245314022463823);
  result.unsafe_access(418) = static_cast<float>(1.5273687664054654);
  result.unsafe_access(419) = static_cast<float>(0.13024299959320884);
  result.unsafe_access(420) = static_cast<float>(-7.725554409604089);
  result.unsafe_access(421) = static_cast<float>(1.0489512849223857);
  result.unsafe_access(422) = static_cast<float>(0.4548002751861987);
  result.unsafe_access(423) = static_cast<float>(0.08727998090191721);
  result.unsafe_access(424) = static_cast<float>(2.9480105996319454);
  result.unsafe_access(425) = static_cast<float>(1.2677994552295526);
  result.unsafe_access(426) = static_cast<float>(0.5987551057506544);
  result.unsafe_access(427) = static_cast<float>(-0.32621187824725756);
  result.unsafe_access(428) = static_cast<float>(18.518591149416775);
  result.unsafe_access(429) = static_cast<float>(-0.43012605756029615);
  result.unsafe_access(430) = static_cast<float>(-0.8765147001793561);
  result.unsafe_access(431) = static_cast<float>(0.0650499507834395);
  result.unsafe_access(432) = static_cast<float>(5.823073091229321);
  result.unsafe_access(433) = static_cast<float>(1.2285632220465739);
  result.unsafe_access(434) = static_cast<float>(0.4565009614217641);
  result.unsafe_access(435) = static_cast<float>(0.740028840584596);
  result.unsafe_access(436) = static_cast<float>(4085.5468326868936);
  result.unsafe_access(437) = static_cast<float>(-4.012084458360231);
  result.unsafe_access(438) = static_cast<float>(-221.40619611343476);
  result.unsafe_access(439) = static_cast<float>(-10.400007997643623);
  result.unsafe_access(440) = static_cast<float>(-21.545096582974693);
  result.unsafe_access(441) = static_cast<float>(1.0103499047974507);
  result.unsafe_access(442) = static_cast<float>(1.1410910459252608);
  result.unsafe_access(443) = static_cast<float>(0.18269056383932814);
  result.unsafe_access(444) = static_cast<float>(-10.872529097394402);
  result.unsafe_access(445) = static_cast<float>(0.8253344665802852);
  result.unsafe_access(446) = static_cast<float>(0.5733248111164921);
  result.unsafe_access(447) = static_cast<float>(3.8739401571027843);
  result.unsafe_access(448) = static_cast<float>(1.2462931523515377);
  result.unsafe_access(449) = static_cast<float>(0.5549293707416866);
  result.unsafe_access(450) = static_cast<float>(-0.3229593807080856);
  result.unsafe_access(451) = static_cast<float>(17.907279488452314);
  result.unsafe_access(452) = static_cast<float>(0.2171641243987513);
  result.unsafe_access(453) = static_cast<float>(-0.7826550388084523);
  result.unsafe_access(454) = static_cast<float>(0.14624102549112125);
  result.unsafe_access(455) = static_cast<float>(6.964164137154581);
  result.unsafe_access(456) = static_cast<float>(1.411253785885902);
  result.unsafe_access(457) = static_cast<float>(0.45400561535569456);
  result.unsafe_access(458) = static_cast<float>(0.7459492412455312);
  result.unsafe_access(459) = static_cast<float>(5298.399738834658);
  result.unsafe_access(460) = static_cast<float>(150.0930310737428);
  result.unsafe_access(461) = static_cast<float>(-260.1217371182456);
  result.unsafe_access(462) = static_cast<float>(-15.034707494715557);
  result.unsafe_access(463) = static_cast<float>(6.706808373775288);
  result.unsafe_access(464) = static_cast<float>(0.9932681133610038);

  return result;
}

} // namespace servo_motor_constraints_instant_MPC_impc_w_unc_map

#endif // SERVO_MOTOR_CONSTRAINTS_INSTANT_MPC_IMPC_W_UNC_MAP_HPP_
