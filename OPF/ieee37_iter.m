function [Voltage_output, phasors] = ieee37_iter(loads, Zbus, Cbus)

tapA799R = 1.0 + 0.00625 * 0;
tapB799R = 1.0 + 0.00625 * 0;
tapC799R = 1.0 + 0.00625 * 0;
alpha799R = [tapA799R; tapB799R; tapC799R];
alphaM799R = alpha799R * alpha799R';

Z701702 = Zbus([13, 14, 15],[16, 17, 18]);
Z702705 = Zbus([16, 17, 18],[19, 20, 21]);
Z702713 = Zbus([16, 17, 18],[22, 23, 24]);
Z702703 = Zbus([16, 17, 18],[25, 26, 27]);
Z703727 = Zbus([25, 26, 27],[28, 29, 30]);
Z703730 = Zbus([25, 26, 27],[31, 32, 33]);
Z704714 = Zbus([34, 35, 36],[37, 38, 39]);
Z704720 = Zbus([34, 35, 36],[40, 41, 42]);
Z705742 = Zbus([19, 20, 21],[43, 44, 45]);
Z705712 = Zbus([19, 20, 21],[46, 47, 48]);
Z706725 = Zbus([49, 50, 51],[52, 53, 54]);
Z707724 = Zbus([55, 56, 57],[58, 59, 60]);
Z707722 = Zbus([55, 56, 57],[61, 62, 63]);
Z708733 = Zbus([64, 65, 66],[67, 68, 69]);
Z708732 = Zbus([64, 65, 66],[70, 71, 72]);
Z709731 = Zbus([7, 8, 9],[73, 74, 75]);
Z709708 = Zbus([7, 8, 9],[64, 65, 66]);
Z710735 = Zbus([76, 77, 78],[79, 80, 81]);
Z710736 = Zbus([76, 77, 78],[82, 83, 84]);
Z711741 = Zbus([85, 86, 87],[88, 89, 90]);
Z711740 = Zbus([85, 86, 87],[91, 92, 93]);
Z713704 = Zbus([22, 23, 24],[34, 35, 36]);
Z714718 = Zbus([37, 38, 39],[94, 95, 96]);
Z720707 = Zbus([40, 41, 42],[55, 56, 57]);
Z720706 = Zbus([40, 41, 42],[49, 50, 51]);
Z727744 = Zbus([28, 29, 30],[97, 98, 99]);
Z730709 = Zbus([31, 32, 33],[7, 8, 9]);
Z733734 = Zbus([67, 68, 69],[100, 101, 102]);
Z734737 = Zbus([100, 101, 102],[103, 104, 105]);
Z734710 = Zbus([100, 101, 102],[76, 77, 78]);
Z737738 = Zbus([103, 104, 105],[106, 107, 108]);
Z738711 = Zbus([106, 107, 108],[85, 86, 87]);
Z744728 = Zbus([97, 98, 99],[109, 110, 111]);
Z744729 = Zbus([97, 98, 99],[112, 113, 114]);
Z709775 = Zbus([7, 8, 9],[10, 11, 12]);
Z799R701 = Zbus([115, 116, 117],[13, 14, 15]);
ZSOURCEBUS799 = Zbus([1, 2, 3],[4, 5, 6]);
Z799799R = Zbus([4, 5, 6],[115, 116, 117]);


% three phase voltage at slack bus
Vbase = 4800 / sqrt(3);
v0=1.05 * Vbase * [0,sqrt(3),0]';
% voltage upper and lower bounds
V_lb = 0.80 * Vbase;
V_ub = 1.08 * Vbase;
v_lb = V_lb * V_lb;
v_ub = V_ub * V_ub;

% sequential component parameters
a = -0.5 + 0.5 * i * sqrt(3);
A = 1/sqrt(3) * [1,1,1; 1, a*a, a; 1, a, a*a];
AH = 1/sqrt(3) * [1,1,1; 1, a, a*a; 1, a*a, a];



cvx_begin sdp quiet
% the solver: 
cvx_solver Mosek;
% cvx_solver Mosek;

% voltage square variables
variable v702(3,3) hermitian
variable v705(3,3) hermitian
variable v713(3,3) hermitian
variable v703(3,3) hermitian
variable v727(3,3) hermitian
variable v730(3,3) hermitian
variable v714(3,3) hermitian
variable v720(3,3) hermitian
variable v742(3,3) hermitian
variable v712(3,3) hermitian
variable v725(3,3) hermitian
variable v724(3,3) hermitian
variable v722(3,3) hermitian
variable v733(3,3) hermitian
variable v732(3,3) hermitian
variable v731(3,3) hermitian
variable v708(3,3) hermitian
variable v735(3,3) hermitian
variable v736(3,3) hermitian
variable v741(3,3) hermitian
variable v740(3,3) hermitian
variable v704(3,3) hermitian
variable v718(3,3) hermitian
variable v707(3,3) hermitian
variable v706(3,3) hermitian
variable v744(3,3) hermitian
variable v709(3,3) hermitian
variable v734(3,3) hermitian
variable v737(3,3) hermitian
variable v710(3,3) hermitian
variable v738(3,3) hermitian
variable v711(3,3) hermitian
variable v728(3,3) hermitian
variable v729(3,3) hermitian
variable v775(3,3) hermitian
variable v701(3,3) hermitian
variable v799(3,3) hermitian
variable v799R(3,3) hermitian
variable vSOURCEBUS(3,3) hermitian

% complex power variables
variable S701702(3,3) complex
variable S702705(3,3) complex
variable S702713(3,3) complex
variable S702703(3,3) complex
variable S703727(3,3) complex
variable S703730(3,3) complex
variable S704714(3,3) complex
variable S704720(3,3) complex
variable S705742(3,3) complex
variable S705712(3,3) complex
variable S706725(3,3) complex
variable S707724(3,3) complex
variable S707722(3,3) complex
variable S708733(3,3) complex
variable S708732(3,3) complex
variable S709731(3,3) complex
variable S709708(3,3) complex
variable S710735(3,3) complex
variable S710736(3,3) complex
variable S711741(3,3) complex
variable S711740(3,3) complex
variable S713704(3,3) complex
variable S714718(3,3) complex
variable S720707(3,3) complex
variable S720706(3,3) complex
variable S727744(3,3) complex
variable S730709(3,3) complex
variable S733734(3,3) complex
variable S734737(3,3) complex
variable S734710(3,3) complex
variable S737738(3,3) complex
variable S738711(3,3) complex
variable S744728(3,3) complex
variable S744729(3,3) complex
variable S709775(3,3) complex
variable S799R701(3,3) complex
variable SSOURCEBUS799(3,3) complex
variable S799799R(3,3) complex

% current square variables
variable l701702(3,3) hermitian
variable l702705(3,3) hermitian
variable l702713(3,3) hermitian
variable l702703(3,3) hermitian
variable l703727(3,3) hermitian
variable l703730(3,3) hermitian
variable l704714(3,3) hermitian
variable l704720(3,3) hermitian
variable l705742(3,3) hermitian
variable l705712(3,3) hermitian
variable l706725(3,3) hermitian
variable l707724(3,3) hermitian
variable l707722(3,3) hermitian
variable l708733(3,3) hermitian
variable l708732(3,3) hermitian
variable l709731(3,3) hermitian
variable l709708(3,3) hermitian
variable l710735(3,3) hermitian
variable l710736(3,3) hermitian
variable l711741(3,3) hermitian
variable l711740(3,3) hermitian
variable l713704(3,3) hermitian
variable l714718(3,3) hermitian
variable l720707(3,3) hermitian
variable l720706(3,3) hermitian
variable l727744(3,3) hermitian
variable l730709(3,3) hermitian
variable l733734(3,3) hermitian
variable l734737(3,3) hermitian
variable l734710(3,3) hermitian
variable l737738(3,3) hermitian
variable l738711(3,3) hermitian
variable l744728(3,3) hermitian
variable l744729(3,3) hermitian
variable l709775(3,3) hermitian
variable l799R701(3,3) hermitian
variable lSOURCEBUS799(3,3) hermitian
variable l799799R(3,3) hermitian


minimize(trace(real(A * Z701702*l701702 * AH)) + trace(real(A * Z702705*l702705 * AH)) + trace(real(A * Z702713*l702713 * AH)) + trace(real(A * Z702703*l702703 * AH)) + trace(real(A * Z703727*l703727 * AH)) + trace(real(A * Z703730*l703730 * AH)) + trace(real(A * Z704714*l704714 * AH)) + trace(real(A * Z704720*l704720 * AH)) + trace(real(A * Z705742*l705742 * AH)) + trace(real(A * Z705712*l705712 * AH)) + trace(real(A * Z706725*l706725 * AH)) + trace(real(A * Z707724*l707724 * AH)) + trace(real(A * Z707722*l707722 * AH)) + trace(real(A * Z708733*l708733 * AH)) + trace(real(A * Z708732*l708732 * AH)) + trace(real(A * Z709731*l709731 * AH)) + trace(real(A * Z709708*l709708 * AH)) + trace(real(A * Z710735*l710735 * AH)) + trace(real(A * Z710736*l710736 * AH)) + trace(real(A * Z711741*l711741 * AH)) + trace(real(A * Z711740*l711740 * AH)) + trace(real(A * Z713704*l713704 * AH)) + trace(real(A * Z714718*l714718 * AH)) + trace(real(A * Z720707*l720707 * AH)) + trace(real(A * Z720706*l720706 * AH)) + trace(real(A * Z727744*l727744 * AH)) + trace(real(A * Z730709*l730709 * AH)) + trace(real(A * Z733734*l733734 * AH)) + trace(real(A * Z734737*l734737 * AH)) + trace(real(A * Z734710*l734710 * AH)) + trace(real(A * Z737738*l737738 * AH)) + trace(real(A * Z738711*l738711 * AH)) + trace(real(A * Z744728*l744728 * AH)) + trace(real(A * Z744729*l744729 * AH)) + trace(real(A * Z709775*l709775 * AH)) + trace(real(A * Z799R701*l799R701 * AH)) + trace(real(A * ZSOURCEBUS799*lSOURCEBUS799 * AH)) + trace(real(A * Z799799R*l799799R * AH)) + 0)
subject to


% constraints: 
% (1): voltage lower and upper bounds 
v_lb <= diag(A * v702 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v705 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v713 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v703 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v727 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v730 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v714 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v720 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v742 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v712 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v725 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v724 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v722 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v733 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v732 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v731 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v708 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v735 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v736 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v741 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v740 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v704 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v718 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v707 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v706 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v744 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v709 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v734 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v737 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v710 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v738 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v711 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v728 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v729 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v775 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v701 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v799 * ctranspose(A)) <= v_ub;
v_lb <= diag(A * v799R * ctranspose(A)) <= v_ub;
vSOURCEBUS == v0 * ctranspose(v0);

% (1): voltage across a line 
v702 == v701 - S701702*ctranspose(Z701702) - Z701702*ctranspose(S701702) + Z701702*l701702*ctranspose(Z701702);
[v701, S701702; ctranspose(S701702), l701702] >= 0;
diag(A *(S701702-Z701702*l701702) * AH)- loads([16, 17, 18]) + diag(A * v702 * Cbus([16, 17, 18],[16, 17, 18]) * AH) == diag(A * S702705 * AH) + diag(A * S702713 * AH) + diag(A * S702703 * AH) + 0;

v705 == v702 - S702705*ctranspose(Z702705) - Z702705*ctranspose(S702705) + Z702705*l702705*ctranspose(Z702705);
[v702, S702705; ctranspose(S702705), l702705] >= 0;
diag(A *(S702705-Z702705*l702705) * AH)- loads([19, 20, 21]) + diag(A * v705 * Cbus([19, 20, 21],[19, 20, 21]) * AH) == diag(A * S705742 * AH) + diag(A * S705712 * AH) + 0;

v713 == v702 - S702713*ctranspose(Z702713) - Z702713*ctranspose(S702713) + Z702713*l702713*ctranspose(Z702713);
[v702, S702713; ctranspose(S702713), l702713] >= 0;
diag(A *(S702713-Z702713*l702713) * AH)- loads([22, 23, 24]) + diag(A * v713 * Cbus([22, 23, 24],[22, 23, 24]) * AH) == diag(A * S713704 * AH) + 0;

v703 == v702 - S702703*ctranspose(Z702703) - Z702703*ctranspose(S702703) + Z702703*l702703*ctranspose(Z702703);
[v702, S702703; ctranspose(S702703), l702703] >= 0;
diag(A *(S702703-Z702703*l702703) * AH)- loads([25, 26, 27]) + diag(A * v703 * Cbus([25, 26, 27],[25, 26, 27]) * AH) == diag(A * S703727 * AH) + diag(A * S703730 * AH) + 0;

v727 == v703 - S703727*ctranspose(Z703727) - Z703727*ctranspose(S703727) + Z703727*l703727*ctranspose(Z703727);
[v703, S703727; ctranspose(S703727), l703727] >= 0;
diag(A *(S703727-Z703727*l703727) * AH)- loads([28, 29, 30]) + diag(A * v727 * Cbus([28, 29, 30],[28, 29, 30]) * AH) == diag(A * S727744 * AH) + 0;

v730 == v703 - S703730*ctranspose(Z703730) - Z703730*ctranspose(S703730) + Z703730*l703730*ctranspose(Z703730);
[v703, S703730; ctranspose(S703730), l703730] >= 0;
diag(A *(S703730-Z703730*l703730) * AH)- loads([31, 32, 33]) + diag(A * v730 * Cbus([31, 32, 33],[31, 32, 33]) * AH) == diag(A * S730709 * AH) + 0;

v714 == v704 - S704714*ctranspose(Z704714) - Z704714*ctranspose(S704714) + Z704714*l704714*ctranspose(Z704714);
[v704, S704714; ctranspose(S704714), l704714] >= 0;
diag(A *(S704714-Z704714*l704714) * AH)- loads([37, 38, 39]) + diag(A * v714 * Cbus([37, 38, 39],[37, 38, 39]) * AH) == diag(A * S714718 * AH) + 0;

v720 == v704 - S704720*ctranspose(Z704720) - Z704720*ctranspose(S704720) + Z704720*l704720*ctranspose(Z704720);
[v704, S704720; ctranspose(S704720), l704720] >= 0;
diag(A *(S704720-Z704720*l704720) * AH)- loads([40, 41, 42]) + diag(A * v720 * Cbus([40, 41, 42],[40, 41, 42]) * AH) == diag(A * S720707 * AH) + diag(A * S720706 * AH) + 0;

v742 == v705 - S705742*ctranspose(Z705742) - Z705742*ctranspose(S705742) + Z705742*l705742*ctranspose(Z705742);
[v705, S705742; ctranspose(S705742), l705742] >= 0;
diag(A *(S705742-Z705742*l705742) * AH)- loads([43, 44, 45]) + diag(A * v742 * Cbus([43, 44, 45],[43, 44, 45]) * AH) == 0;

v712 == v705 - S705712*ctranspose(Z705712) - Z705712*ctranspose(S705712) + Z705712*l705712*ctranspose(Z705712);
[v705, S705712; ctranspose(S705712), l705712] >= 0;
diag(A *(S705712-Z705712*l705712) * AH)- loads([46, 47, 48]) + diag(A * v712 * Cbus([46, 47, 48],[46, 47, 48]) * AH) == 0;

v725 == v706 - S706725*ctranspose(Z706725) - Z706725*ctranspose(S706725) + Z706725*l706725*ctranspose(Z706725);
[v706, S706725; ctranspose(S706725), l706725] >= 0;
diag(A *(S706725-Z706725*l706725) * AH)- loads([52, 53, 54]) + diag(A * v725 * Cbus([52, 53, 54],[52, 53, 54]) * AH) == 0;

v724 == v707 - S707724*ctranspose(Z707724) - Z707724*ctranspose(S707724) + Z707724*l707724*ctranspose(Z707724);
[v707, S707724; ctranspose(S707724), l707724] >= 0;
diag(A *(S707724-Z707724*l707724) * AH)- loads([58, 59, 60]) + diag(A * v724 * Cbus([58, 59, 60],[58, 59, 60]) * AH) == 0;

v722 == v707 - S707722*ctranspose(Z707722) - Z707722*ctranspose(S707722) + Z707722*l707722*ctranspose(Z707722);
[v707, S707722; ctranspose(S707722), l707722] >= 0;
diag(A *(S707722-Z707722*l707722) * AH)- loads([61, 62, 63]) + diag(A * v722 * Cbus([61, 62, 63],[61, 62, 63]) * AH) == 0;

v733 == v708 - S708733*ctranspose(Z708733) - Z708733*ctranspose(S708733) + Z708733*l708733*ctranspose(Z708733);
[v708, S708733; ctranspose(S708733), l708733] >= 0;
diag(A *(S708733-Z708733*l708733) * AH)- loads([67, 68, 69]) + diag(A * v733 * Cbus([67, 68, 69],[67, 68, 69]) * AH) == diag(A * S733734 * AH) + 0;

v732 == v708 - S708732*ctranspose(Z708732) - Z708732*ctranspose(S708732) + Z708732*l708732*ctranspose(Z708732);
[v708, S708732; ctranspose(S708732), l708732] >= 0;
diag(A *(S708732-Z708732*l708732) * AH)- loads([70, 71, 72]) + diag(A * v732 * Cbus([70, 71, 72],[70, 71, 72]) * AH) == 0;

v731 == v709 - S709731*ctranspose(Z709731) - Z709731*ctranspose(S709731) + Z709731*l709731*ctranspose(Z709731);
[v709, S709731; ctranspose(S709731), l709731] >= 0;
diag(A *(S709731-Z709731*l709731) * AH)- loads([73, 74, 75]) + diag(A * v731 * Cbus([73, 74, 75],[73, 74, 75]) * AH) == 0;

v708 == v709 - S709708*ctranspose(Z709708) - Z709708*ctranspose(S709708) + Z709708*l709708*ctranspose(Z709708);
[v709, S709708; ctranspose(S709708), l709708] >= 0;
diag(A *(S709708-Z709708*l709708) * AH)- loads([64, 65, 66]) + diag(A * v708 * Cbus([64, 65, 66],[64, 65, 66]) * AH) == diag(A * S708733 * AH) + diag(A * S708732 * AH) + 0;

v735 == v710 - S710735*ctranspose(Z710735) - Z710735*ctranspose(S710735) + Z710735*l710735*ctranspose(Z710735);
[v710, S710735; ctranspose(S710735), l710735] >= 0;
diag(A *(S710735-Z710735*l710735) * AH)- loads([79, 80, 81]) + diag(A * v735 * Cbus([79, 80, 81],[79, 80, 81]) * AH) == 0;

v736 == v710 - S710736*ctranspose(Z710736) - Z710736*ctranspose(S710736) + Z710736*l710736*ctranspose(Z710736);
[v710, S710736; ctranspose(S710736), l710736] >= 0;
diag(A *(S710736-Z710736*l710736) * AH)- loads([82, 83, 84]) + diag(A * v736 * Cbus([82, 83, 84],[82, 83, 84]) * AH) == 0;

v741 == v711 - S711741*ctranspose(Z711741) - Z711741*ctranspose(S711741) + Z711741*l711741*ctranspose(Z711741);
[v711, S711741; ctranspose(S711741), l711741] >= 0;
diag(A *(S711741-Z711741*l711741) * AH)- loads([88, 89, 90]) + diag(A * v741 * Cbus([88, 89, 90],[88, 89, 90]) * AH) == 0;

v740 == v711 - S711740*ctranspose(Z711740) - Z711740*ctranspose(S711740) + Z711740*l711740*ctranspose(Z711740);
[v711, S711740; ctranspose(S711740), l711740] >= 0;
diag(A *(S711740-Z711740*l711740) * AH)- loads([91, 92, 93]) + diag(A * v740 * Cbus([91, 92, 93],[91, 92, 93]) * AH) == 0;

v704 == v713 - S713704*ctranspose(Z713704) - Z713704*ctranspose(S713704) + Z713704*l713704*ctranspose(Z713704);
[v713, S713704; ctranspose(S713704), l713704] >= 0;
diag(A *(S713704-Z713704*l713704) * AH)- loads([34, 35, 36]) + diag(A * v704 * Cbus([34, 35, 36],[34, 35, 36]) * AH) == diag(A * S704714 * AH) + diag(A * S704720 * AH) + 0;

v718 == v714 - S714718*ctranspose(Z714718) - Z714718*ctranspose(S714718) + Z714718*l714718*ctranspose(Z714718);
[v714, S714718; ctranspose(S714718), l714718] >= 0;
diag(A *(S714718-Z714718*l714718) * AH)- loads([94, 95, 96]) + diag(A * v718 * Cbus([94, 95, 96],[94, 95, 96]) * AH) == 0;

v707 == v720 - S720707*ctranspose(Z720707) - Z720707*ctranspose(S720707) + Z720707*l720707*ctranspose(Z720707);
[v720, S720707; ctranspose(S720707), l720707] >= 0;
diag(A *(S720707-Z720707*l720707) * AH)- loads([55, 56, 57]) + diag(A * v707 * Cbus([55, 56, 57],[55, 56, 57]) * AH) == diag(A * S707724 * AH) + diag(A * S707722 * AH) + 0;

v706 == v720 - S720706*ctranspose(Z720706) - Z720706*ctranspose(S720706) + Z720706*l720706*ctranspose(Z720706);
[v720, S720706; ctranspose(S720706), l720706] >= 0;
diag(A *(S720706-Z720706*l720706) * AH)- loads([49, 50, 51]) + diag(A * v706 * Cbus([49, 50, 51],[49, 50, 51]) * AH) == diag(A * S706725 * AH) + 0;

v744 == v727 - S727744*ctranspose(Z727744) - Z727744*ctranspose(S727744) + Z727744*l727744*ctranspose(Z727744);
[v727, S727744; ctranspose(S727744), l727744] >= 0;
diag(A *(S727744-Z727744*l727744) * AH)- loads([97, 98, 99]) + diag(A * v744 * Cbus([97, 98, 99],[97, 98, 99]) * AH) == diag(A * S744728 * AH) + diag(A * S744729 * AH) + 0;

v709 == v730 - S730709*ctranspose(Z730709) - Z730709*ctranspose(S730709) + Z730709*l730709*ctranspose(Z730709);
[v730, S730709; ctranspose(S730709), l730709] >= 0;
diag(A *(S730709-Z730709*l730709) * AH)- loads([7, 8, 9]) + diag(A * v709 * Cbus([7, 8, 9],[7, 8, 9]) * AH) == diag(A * S709731 * AH) + diag(A * S709708 * AH) + diag(A * S709775 * AH) + 0;

v734 == v733 - S733734*ctranspose(Z733734) - Z733734*ctranspose(S733734) + Z733734*l733734*ctranspose(Z733734);
[v733, S733734; ctranspose(S733734), l733734] >= 0;
diag(A *(S733734-Z733734*l733734) * AH)- loads([100, 101, 102]) + diag(A * v734 * Cbus([100, 101, 102],[100, 101, 102]) * AH) == diag(A * S734737 * AH) + diag(A * S734710 * AH) + 0;

v737 == v734 - S734737*ctranspose(Z734737) - Z734737*ctranspose(S734737) + Z734737*l734737*ctranspose(Z734737);
[v734, S734737; ctranspose(S734737), l734737] >= 0;
diag(A *(S734737-Z734737*l734737) * AH)- loads([103, 104, 105]) + diag(A * v737 * Cbus([103, 104, 105],[103, 104, 105]) * AH) == diag(A * S737738 * AH) + 0;

v710 == v734 - S734710*ctranspose(Z734710) - Z734710*ctranspose(S734710) + Z734710*l734710*ctranspose(Z734710);
[v734, S734710; ctranspose(S734710), l734710] >= 0;
diag(A *(S734710-Z734710*l734710) * AH)- loads([76, 77, 78]) + diag(A * v710 * Cbus([76, 77, 78],[76, 77, 78]) * AH) == diag(A * S710735 * AH) + diag(A * S710736 * AH) + 0;

v738 == v737 - S737738*ctranspose(Z737738) - Z737738*ctranspose(S737738) + Z737738*l737738*ctranspose(Z737738);
[v737, S737738; ctranspose(S737738), l737738] >= 0;
diag(A *(S737738-Z737738*l737738) * AH)- loads([106, 107, 108]) + diag(A * v738 * Cbus([106, 107, 108],[106, 107, 108]) * AH) == diag(A * S738711 * AH) + 0;

v711 == v738 - S738711*ctranspose(Z738711) - Z738711*ctranspose(S738711) + Z738711*l738711*ctranspose(Z738711);
[v738, S738711; ctranspose(S738711), l738711] >= 0;
diag(A *(S738711-Z738711*l738711) * AH)- loads([85, 86, 87]) + diag(A * v711 * Cbus([85, 86, 87],[85, 86, 87]) * AH) == diag(A * S711741 * AH) + diag(A * S711740 * AH) + 0;

v728 == v744 - S744728*ctranspose(Z744728) - Z744728*ctranspose(S744728) + Z744728*l744728*ctranspose(Z744728);
[v744, S744728; ctranspose(S744728), l744728] >= 0;
diag(A *(S744728-Z744728*l744728) * AH)- loads([109, 110, 111]) + diag(A * v728 * Cbus([109, 110, 111],[109, 110, 111]) * AH) == 0;

v729 == v744 - S744729*ctranspose(Z744729) - Z744729*ctranspose(S744729) + Z744729*l744729*ctranspose(Z744729);
[v744, S744729; ctranspose(S744729), l744729] >= 0;
diag(A *(S744729-Z744729*l744729) * AH)- loads([112, 113, 114]) + diag(A * v729 * Cbus([112, 113, 114],[112, 113, 114]) * AH) == 0;

v775 == v709 - S709775*ctranspose(Z709775) - Z709775*ctranspose(S709775) + Z709775*l709775*ctranspose(Z709775);
[v709, S709775; ctranspose(S709775), l709775] >= 0;
diag(A *(S709775-Z709775*l709775) * AH)- loads([10, 11, 12]) + diag(A * v775 * Cbus([10, 11, 12],[10, 11, 12]) * AH) == 0;

v701 == v799R - S799R701*ctranspose(Z799R701) - Z799R701*ctranspose(S799R701) + Z799R701*l799R701*ctranspose(Z799R701);
[v799R, S799R701; ctranspose(S799R701), l799R701] >= 0;
diag(A *(S799R701-Z799R701*l799R701) * AH)- loads([13, 14, 15]) + diag(A * v701 * Cbus([13, 14, 15],[13, 14, 15]) * AH) == diag(A * S701702 * AH) + 0;

v799 == vSOURCEBUS - SSOURCEBUS799*ctranspose(ZSOURCEBUS799) - ZSOURCEBUS799*ctranspose(SSOURCEBUS799) + ZSOURCEBUS799*lSOURCEBUS799*ctranspose(ZSOURCEBUS799);
[vSOURCEBUS, SSOURCEBUS799; ctranspose(SSOURCEBUS799), lSOURCEBUS799] >= 0;
diag(A *(SSOURCEBUS799-ZSOURCEBUS799*lSOURCEBUS799) * AH)- loads([4, 5, 6]) + diag(A * v799 * Cbus([4, 5, 6],[4, 5, 6]) * AH) == diag(A * S799799R * AH) + 0;

A * v799R * AH == (A * v799([1, 2, 3],[1, 2, 3]) * AH) .* alphaM799R;
[v799([1, 2, 3],[1, 2, 3]), S799799R; ctranspose(S799799R), l799799R] >= 0;
diag(A *(S799799R-Z799799R*l799799R) * AH)- loads([115, 116, 117]) + diag(A * v799R * Cbus([115, 116, 117],[115, 116, 117]) * AH) == diag(A * S799R701 * AH) + 0;



cvx_end


VSOURCEBUS = A * v0;
ISOURCEBUS799 = 1/trace(A * vSOURCEBUS([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * SSOURCEBUS799 * AH)*VSOURCEBUS([1, 2, 3]);
V799 = VSOURCEBUS([1, 2, 3]) - A * ZSOURCEBUS799* AH *ISOURCEBUS799;
I799799R = 1/trace(A * v799([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S799799R * AH)*V799([1, 2, 3]);
V799R = V799([1, 2, 3]) .* alpha799R;
I799R701 = 1/trace(A * v799R([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S799R701 * AH)*V799R([1, 2, 3]);
V701 = V799R([1, 2, 3]) - A * Z799R701* AH *I799R701;
I701702 = 1/trace(A * v701([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S701702 * AH)*V701([1, 2, 3]);
V702 = V701([1, 2, 3]) - A * Z701702* AH *I701702;
I702705 = 1/trace(A * v702([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S702705 * AH)*V702([1, 2, 3]);
V705 = V702([1, 2, 3]) - A * Z702705* AH *I702705;
I702713 = 1/trace(A * v702([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S702713 * AH)*V702([1, 2, 3]);
V713 = V702([1, 2, 3]) - A * Z702713* AH *I702713;
I702703 = 1/trace(A * v702([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S702703 * AH)*V702([1, 2, 3]);
V703 = V702([1, 2, 3]) - A * Z702703* AH *I702703;
I705742 = 1/trace(A * v705([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S705742 * AH)*V705([1, 2, 3]);
V742 = V705([1, 2, 3]) - A * Z705742* AH *I705742;
I705712 = 1/trace(A * v705([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S705712 * AH)*V705([1, 2, 3]);
V712 = V705([1, 2, 3]) - A * Z705712* AH *I705712;
I713704 = 1/trace(A * v713([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S713704 * AH)*V713([1, 2, 3]);
V704 = V713([1, 2, 3]) - A * Z713704* AH *I713704;
I703727 = 1/trace(A * v703([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S703727 * AH)*V703([1, 2, 3]);
V727 = V703([1, 2, 3]) - A * Z703727* AH *I703727;
I703730 = 1/trace(A * v703([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S703730 * AH)*V703([1, 2, 3]);
V730 = V703([1, 2, 3]) - A * Z703730* AH *I703730;
I704714 = 1/trace(A * v704([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S704714 * AH)*V704([1, 2, 3]);
V714 = V704([1, 2, 3]) - A * Z704714* AH *I704714;
I704720 = 1/trace(A * v704([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S704720 * AH)*V704([1, 2, 3]);
V720 = V704([1, 2, 3]) - A * Z704720* AH *I704720;
I727744 = 1/trace(A * v727([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S727744 * AH)*V727([1, 2, 3]);
V744 = V727([1, 2, 3]) - A * Z727744* AH *I727744;
I730709 = 1/trace(A * v730([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S730709 * AH)*V730([1, 2, 3]);
V709 = V730([1, 2, 3]) - A * Z730709* AH *I730709;
I714718 = 1/trace(A * v714([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S714718 * AH)*V714([1, 2, 3]);
V718 = V714([1, 2, 3]) - A * Z714718* AH *I714718;
I720707 = 1/trace(A * v720([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S720707 * AH)*V720([1, 2, 3]);
V707 = V720([1, 2, 3]) - A * Z720707* AH *I720707;
I720706 = 1/trace(A * v720([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S720706 * AH)*V720([1, 2, 3]);
V706 = V720([1, 2, 3]) - A * Z720706* AH *I720706;
I744728 = 1/trace(A * v744([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S744728 * AH)*V744([1, 2, 3]);
V728 = V744([1, 2, 3]) - A * Z744728* AH *I744728;
I744729 = 1/trace(A * v744([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S744729 * AH)*V744([1, 2, 3]);
V729 = V744([1, 2, 3]) - A * Z744729* AH *I744729;
I709731 = 1/trace(A * v709([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S709731 * AH)*V709([1, 2, 3]);
V731 = V709([1, 2, 3]) - A * Z709731* AH *I709731;
I709708 = 1/trace(A * v709([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S709708 * AH)*V709([1, 2, 3]);
V708 = V709([1, 2, 3]) - A * Z709708* AH *I709708;
I709775 = 1/trace(A * v709([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S709775 * AH)*V709([1, 2, 3]);
V775 = V709([1, 2, 3]) - A * Z709775* AH *I709775;
I707724 = 1/trace(A * v707([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S707724 * AH)*V707([1, 2, 3]);
V724 = V707([1, 2, 3]) - A * Z707724* AH *I707724;
I707722 = 1/trace(A * v707([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S707722 * AH)*V707([1, 2, 3]);
V722 = V707([1, 2, 3]) - A * Z707722* AH *I707722;
I706725 = 1/trace(A * v706([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S706725 * AH)*V706([1, 2, 3]);
V725 = V706([1, 2, 3]) - A * Z706725* AH *I706725;
I708733 = 1/trace(A * v708([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S708733 * AH)*V708([1, 2, 3]);
V733 = V708([1, 2, 3]) - A * Z708733* AH *I708733;
I708732 = 1/trace(A * v708([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S708732 * AH)*V708([1, 2, 3]);
V732 = V708([1, 2, 3]) - A * Z708732* AH *I708732;
I733734 = 1/trace(A * v733([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S733734 * AH)*V733([1, 2, 3]);
V734 = V733([1, 2, 3]) - A * Z733734* AH *I733734;
I734737 = 1/trace(A * v734([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S734737 * AH)*V734([1, 2, 3]);
V737 = V734([1, 2, 3]) - A * Z734737* AH *I734737;
I734710 = 1/trace(A * v734([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S734710 * AH)*V734([1, 2, 3]);
V710 = V734([1, 2, 3]) - A * Z734710* AH *I734710;
I737738 = 1/trace(A * v737([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S737738 * AH)*V737([1, 2, 3]);
V738 = V737([1, 2, 3]) - A * Z737738* AH *I737738;
I710735 = 1/trace(A * v710([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S710735 * AH)*V710([1, 2, 3]);
V735 = V710([1, 2, 3]) - A * Z710735* AH *I710735;
I710736 = 1/trace(A * v710([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S710736 * AH)*V710([1, 2, 3]);
V736 = V710([1, 2, 3]) - A * Z710736* AH *I710736;
I738711 = 1/trace(A * v738([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S738711 * AH)*V738([1, 2, 3]);
V711 = V738([1, 2, 3]) - A * Z738711* AH *I738711;
I711741 = 1/trace(A * v711([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S711741 * AH)*V711([1, 2, 3]);
V741 = V711([1, 2, 3]) - A * Z711741* AH *I711741;
I711740 = 1/trace(A * v711([1, 2, 3],[1, 2, 3]) * AH) * ctranspose(A * S711740 * AH)*V711([1, 2, 3]);
V740 = V711([1, 2, 3]) - A * Z711740* AH *I711740;


phasors=[];
phasors=[phasors;recover_voltage(VSOURCEBUS, 123)];
phasors=[phasors;recover_voltage(V799, 123)];
phasors=[phasors;recover_voltage(V799R, 123)];
phasors=[phasors;recover_voltage(V701, 123)];
phasors=[phasors;recover_voltage(V702, 123)];
phasors=[phasors;recover_voltage(V705, 123)];
phasors=[phasors;recover_voltage(V713, 123)];
phasors=[phasors;recover_voltage(V703, 123)];
phasors=[phasors;recover_voltage(V742, 123)];
phasors=[phasors;recover_voltage(V712, 123)];
phasors=[phasors;recover_voltage(V704, 123)];
phasors=[phasors;recover_voltage(V727, 123)];
phasors=[phasors;recover_voltage(V730, 123)];
phasors=[phasors;recover_voltage(V714, 123)];
phasors=[phasors;recover_voltage(V720, 123)];
phasors=[phasors;recover_voltage(V744, 123)];
phasors=[phasors;recover_voltage(V709, 123)];
phasors=[phasors;recover_voltage(V718, 123)];
phasors=[phasors;recover_voltage(V707, 123)];
phasors=[phasors;recover_voltage(V706, 123)];
phasors=[phasors;recover_voltage(V728, 123)];
phasors=[phasors;recover_voltage(V729, 123)];
phasors=[phasors;recover_voltage(V731, 123)];
phasors=[phasors;recover_voltage(V708, 123)];
phasors=[phasors;recover_voltage(V775, 123)];
phasors=[phasors;recover_voltage(V724, 123)];
phasors=[phasors;recover_voltage(V722, 123)];
phasors=[phasors;recover_voltage(V725, 123)];
phasors=[phasors;recover_voltage(V733, 123)];
phasors=[phasors;recover_voltage(V732, 123)];
phasors=[phasors;recover_voltage(V734, 123)];
phasors=[phasors;recover_voltage(V737, 123)];
phasors=[phasors;recover_voltage(V710, 123)];
phasors=[phasors;recover_voltage(V738, 123)];
phasors=[phasors;recover_voltage(V735, 123)];
phasors=[phasors;recover_voltage(V736, 123)];
phasors=[phasors;recover_voltage(V711, 123)];
phasors=[phasors;recover_voltage(V741, 123)];
phasors=[phasors;recover_voltage(V740, 123)];

% change to per unit
phasors(:, 1) = phasors(:, 1) / Vbase;
phasors(:, 3) = phasors(:, 3) / Vbase;
phasors(:, 5) = phasors(:, 5) / Vbase;

% for k = 1:size(phasors, 1)
%     if(phasors(k, 3) > 0)
%         phasors(k, 4) = phasors(k, 4) - 180;
%     end
%     if(phasors(k, 5) > 0)
%         phasors(k, 6) = phasors(k, 6) + 180;
%     end
% end



Voltage_output=[];
Voltage_output = [Voltage_output; recover_voltage(V701, 123)];
Voltage_output = [Voltage_output; recover_voltage(V712, 123)];
Voltage_output = [Voltage_output; recover_voltage(V713, 123)];
Voltage_output = [Voltage_output; recover_voltage(V714, 123)];
Voltage_output = [Voltage_output; recover_voltage(V718, 123)];
Voltage_output = [Voltage_output; recover_voltage(V720, 123)];
Voltage_output = [Voltage_output; recover_voltage(V722, 123)];
Voltage_output = [Voltage_output; recover_voltage(V724, 123)];
Voltage_output = [Voltage_output; recover_voltage(V725, 123)];
Voltage_output = [Voltage_output; recover_voltage(V727, 123)];
Voltage_output = [Voltage_output; recover_voltage(V728, 123)];
Voltage_output = [Voltage_output; recover_voltage(V729, 123)];
Voltage_output = [Voltage_output; recover_voltage(V730, 123)];
Voltage_output = [Voltage_output; recover_voltage(V731, 123)];
Voltage_output = [Voltage_output; recover_voltage(V732, 123)];
Voltage_output = [Voltage_output; recover_voltage(V733, 123)];
Voltage_output = [Voltage_output; recover_voltage(V734, 123)];
Voltage_output = [Voltage_output; recover_voltage(V735, 123)];
Voltage_output = [Voltage_output; recover_voltage(V736, 123)];
Voltage_output = [Voltage_output; recover_voltage(V737, 123)];
Voltage_output = [Voltage_output; recover_voltage(V738, 123)];
Voltage_output = [Voltage_output; recover_voltage(V740, 123)];
Voltage_output = [Voltage_output; recover_voltage(V741, 123)];
Voltage_output = [Voltage_output; recover_voltage(V742, 123)];
Voltage_output = [Voltage_output; recover_voltage(V744, 123)];

% change to per unit
Voltage_output(:, 1) = Voltage_output(:, 1) / Vbase;
Voltage_output(:, 3) = Voltage_output(:, 3) / Vbase;
Voltage_output(:, 5) = Voltage_output(:, 5) / Vbase;

disp(diag(A * S799799R * AH) / 1000);

% disp(S701702 / 1000)
% disp(l701702 / 1000)
% disp(v702)