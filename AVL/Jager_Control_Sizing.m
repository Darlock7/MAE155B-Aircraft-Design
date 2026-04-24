%% ============================================================
%  JAGER_AVL_AUTORUN_CONTROL_SIZING_V6.m
%
%  PURPOSE:
%   1) Write jager geometry file
%   2) Write jager mass file
%   3) Visualize control-surface geometry in MATLAB
%   4) Run AVL automatically from MATLAB
%   5) Export FT and HM files automatically
%   6) Run baseline / positive / negative control cases
%   7) Compute finite-difference control derivatives
%   8) Do this for:
%         - D1 = aileron
%         - D2 = V-tail pitch mix
%         - D3 = V-tail yaw mix
%
%  NOTES:
%   - This script does NOT use a .run file
%   - It uses direct OPER commands
%   - HM save follows your manual workflow:
%         X
%         HM
%         filename
%         O
%   - AVL may still return a nonzero exit status on scripted EOF even when
%     FT/HM files are created correctly. This script continues if FT exists.
%
%  COORDINATES:
%   - AVL geometry file uses X forward, Y right, Z up
%   - MATLAB plots use same convention
% ============================================================

clear; clc; close all;

%% =========================
% 0) FILENAMES / PATHS
% =========================
scriptDir = fileparts(mfilename('fullpath'));

avlExe   = fullfile(scriptDir, "avl352");

geomName = "jager.geo.avl";
massName = "jager.mass";
cmdName  = "jager_control_commands.txt";
ftName   = "jager_ft.txt";
hmName   = "jager_hm.txt";
logName  = "jager_avl_log.txt";

geomFile = fullfile(scriptDir, geomName);
massFile = fullfile(scriptDir, massName);
cmdFile  = fullfile(scriptDir, cmdName);
ftFile   = fullfile(scriptDir, ftName);
hmFile   = fullfile(scriptDir, hmName);
logFile  = fullfile(scriptDir, logName);
%% =========================
% 1) GLOBAL CONSTANTS / UNITS
% =========================
g_ft   = 32.174;          % [ft/s^2]
rho    = 0.0023769;       % [slug/ft^3]
Mach   = 0.00;
CDp    = 0.02813;

%% =========================
% 2) AIRCRAFT WEIGHT / CG / INERTIA
% =========================
W_lbf  = 55;
m_slug = W_lbf / g_ft;

Xcg_in = 39.1;            % [in]
Ycg_ft = 0.0;             % [ft]
Zcg_ft = -1.74/12;        % [ft]
Xcg_ft = Xcg_in/12;

% SolidWorks principal inertias [lbm*ft^2]
Ixx_sw =  75.7671;
Iyy_sw = 119.6108;
Izz_sw = 191.5794;

convI = 1/32.174;
Ixx = Ixx_sw * convI;
Iyy = Iyy_sw * convI;
Izz = Izz_sw * convI;

Ixy = 0;
Ixz = 0;
Iyz = 0;

%% =========================
% 3) AERODYNAMIC REFERENCE GEOMETRY
% =========================
Sref = 6.64;      % [ft^2]
Cref = 1.052;     % [ft]
Bref = 6.31;      % [ft]
b2   = Bref/2;

Xref = Xcg_ft;
Yref = 0.0;
Zref = Zcg_ft;

%% =========================
% 4) CAD DATUM LOCATIONS
% =========================
XwingLE_in = 35.0;
XwingLE    = XwingLE_in/12;   % [ft]

%% =========================
% 5) FLIGHT CONDITION
% =========================
V_ftps    = 75.0;
alpha_deg = 8.25;
beta_deg  = 0.0;

qbar = 0.5 * rho * V_ftps^2;

%% =========================
% 6) WING GEOMETRY + AILERON
% =========================
c_w = Cref;

eta_a0 = 0.50;
y_a0   = eta_a0*b2;
y_tip  = 1.00*b2;

wing_airfoil_type = "NACA";
wing_naca         = "4412";

Xhinge_ail     = 0.80;
aileron_gain   = 1.0;
aileron_SgnDup = -1.0;

%% =========================
% 7) V-TAIL GEOMETRY + CONTROL MIXES
% =========================
S_panel      = 0.809;         % [ft^2] each panel area
AR_tail      = 2.8;           % [-]
taper_vt     = 0.4;           % c_tip / c_root
gamma_deg    = 36.38;         % [deg]
LambdaLE_deg = 45.0;          % [deg]
XtailQC      = 80.3636/12;    % [ft]

b_panel   = sqrt(AR_tail*S_panel);
c_root_vt = 2*S_panel/(b_panel*(1+taper_vt));
c_tip_vt  = taper_vt*c_root_vt;

XtailLE_root = XtailQC - 0.25*c_root_vt;

yT = b_panel*cosd(gamma_deg);
zT = b_panel*sind(gamma_deg);

eta_rv0 = 0.30;

vtail_airfoil_type = "NACA";
vtail_naca         = "0010";

Xhinge_rv = 0.70;

pitchv_gain = 1.0;
yawv_gain   = 1.0;

%% =========================
% 8) CONTROL TEST SETUP
% =========================
% Order must match CONTROL declaration order in the geometry file.
% Wing aileron is declared first -> D1
% V-tail pitch mix is declared second -> D2
% V-tail yaw mix is declared third -> D3

controlMenu_ail = "D1";
controlName_ail = "aileron";

controlMenu_pitchv = "D2";
controlName_pitchv = "pitchv";

controlMenu_yawv = "D3";
controlName_yawv = "yawv";

delta0 =  0.0;
deltaP = +4.0;
deltaM = -4.0;

%% =========================
% 9) DERIVED CONTROL GEOMETRY FOR PLOTTING / PRINTING
% =========================
x_hinge_ail = XwingLE + Xhinge_ail*c_w;

% V-tail shared stations
eta0 = 0.0;
eta1 = eta_rv0;
eta2 = 1.0;

y0p = eta0*abs(yT); z0 = eta0*zT;
y1p = eta1*abs(yT); z1 = eta1*zT;
y2p = eta2*abs(yT); z2 = eta2*zT;

c0 = c_root_vt + (c_tip_vt - c_root_vt)*eta0;
c1 = c_root_vt + (c_tip_vt - c_root_vt)*eta1;
c2 = c_root_vt + (c_tip_vt - c_root_vt)*eta2;

x0 = XtailLE_root + y0p*tand(LambdaLE_deg);
x1 = XtailLE_root + y1p*tand(LambdaLE_deg);
x2 = XtailLE_root + y2p*tand(LambdaLE_deg);

x_hinge0 = x1 + Xhinge_rv*c1;
x_hinge2 = x2 + Xhinge_rv*c2;

ail_span      = y_tip - y_a0;
ail_chord     = (1 - Xhinge_ail)*c_w;
rv_span_top   = sqrt((x2-x1)^2 + (y2p-y1p)^2);
rv_span_front = sqrt((y2p-y1p)^2 + (z2-z1)^2);
rv_chord_root = (1 - Xhinge_rv)*c1;
rv_chord_tip  = (1 - Xhinge_rv)*c2;

% Inch conversions for display
ft2in = 12.0;
ail_span_in      = ail_span * ft2in;
ail_chord_in     = ail_chord * ft2in;
rv_span_top_in   = rv_span_top * ft2in;
rv_span_front_in = rv_span_front * ft2in;
rv_chord_root_in = rv_chord_root * ft2in;
rv_chord_tip_in  = rv_chord_tip * ft2in;

%% =========================
% 10) QUICK CONTROL GEOMETRY VISUALIZATION
% =========================
figure('Name','JAGER Control Geometry','Color','w');

% ------------------------------------------------------------
% 10A) TOP VIEW
% ------------------------------------------------------------
subplot(1,2,1); hold on; axis equal; grid on;
title('Top View');
xlabel('X [ft]');
ylabel('Y [ft]');

% Right wing
xw = [XwingLE, XwingLE + c_w, XwingLE + c_w, XwingLE];
yw = [0,       0,            y_tip,         y_tip];
patch(xw, yw, [0.85 0.85 0.85], 'EdgeColor','k', 'LineWidth',1.2);

% Left wing
patch(xw, -yw, [0.85 0.85 0.85], 'EdgeColor','k', 'LineWidth',1.2);

% Right aileron
xa = [x_hinge_ail, XwingLE + c_w, XwingLE + c_w, x_hinge_ail];
ya = [y_a0,        y_a0,          y_tip,         y_tip];
patch(xa, ya, [0.2 0.6 1.0], 'FaceAlpha',0.45, 'EdgeColor','b', 'LineWidth',1.2);

% Left aileron
patch(xa, -ya, [0.2 0.6 1.0], 'FaceAlpha',0.45, 'EdgeColor','b', 'LineWidth',1.2);

% Aileron hinge lines
plot([x_hinge_ail x_hinge_ail], [ y_a0  y_tip], 'b--', 'LineWidth',1.2);
plot([x_hinge_ail x_hinge_ail], [-y_a0 -y_tip], 'b--', 'LineWidth',1.2);

% V-tail planforms
x_vr = [x0, x0 + c0, x2 + c2, x2];
y_vr = [y0p, y0p,    y2p,     y2p];
patch(x_vr, y_vr, [0.80 0.90 0.80], 'EdgeColor','k', 'LineWidth',1.2);

x_vl = [x0, x0 + c0, x2 + c2, x2];
y_vl = [-y0p, -y0p,  -y2p,    -y2p];
patch(x_vl, y_vl, [0.80 0.90 0.80], 'EdgeColor','k', 'LineWidth',1.2);

% Ruddervator physical region
xcr = [x_hinge0, x1 + c1, x2 + c2, x_hinge2];
ycr = [y1p,      y1p,     y2p,     y2p];
patch(xcr, ycr, [1.0 0.5 0.2], 'FaceAlpha',0.45, ...
    'EdgeColor',[0.85 0.33 0.1], 'LineWidth',1.2);

xcl = [x_hinge0, x1 + c1, x2 + c2, x_hinge2];
ycl = [-y1p,     -y1p,    -y2p,    -y2p];
patch(xcl, ycl, [1.0 0.5 0.2], 'FaceAlpha',0.45, ...
    'EdgeColor',[0.85 0.33 0.1], 'LineWidth',1.2);

% Ruddervator hinge lines
plot([x_hinge0 x_hinge2], [ y1p  y2p], '--', 'Color',[0.85 0.33 0.1], 'LineWidth',1.2);
plot([x_hinge0 x_hinge2], [-y1p -y2p], '--', 'Color',[0.85 0.33 0.1], 'LineWidth',1.2);

% Labels: aileron
text(XwingLE + 0.5*c_w, 0.5*(y_a0+y_tip), ...
    sprintf('Aileron span = %.2f in', ail_span_in), ...
    'Color','b', 'FontWeight','bold', ...
    'HorizontalAlignment','center', 'BackgroundColor','w');

text(0.5*(x_hinge_ail + XwingLE + c_w), y_tip + 0.08, ...
    sprintf('Aileron chord = %.2f in', ail_chord_in), ...
    'Color','b', 'FontWeight','bold', ...
    'HorizontalAlignment','center', 'BackgroundColor','w');

% Labels: ruddervator
text(0.5*(x1+x2), 0.5*(y1p+y2p)+0.08, ...
    sprintf('Ruddervator span = %.2f in', rv_span_top_in), ...
    'Color',[0.85 0.33 0.1], 'FontWeight','bold', ...
    'HorizontalAlignment','center', 'BackgroundColor','w');

text(0.5*(x_hinge0 + x1 + c1), y1p - 0.10, ...
    sprintf('Root ctrl chord = %.2f in', rv_chord_root_in), ...
    'Color',[0.85 0.33 0.1], 'FontWeight','bold', ...
    'HorizontalAlignment','center', 'BackgroundColor','w');

text(0.5*(x_hinge2 + x2 + c2), y2p + 0.10, ...
    sprintf('Tip ctrl chord = %.2f in', rv_chord_tip_in), ...
    'Color',[0.85 0.33 0.1], 'FontWeight','bold', ...
    'HorizontalAlignment','center', 'BackgroundColor','w');

legend({'Wing','Wing','Aileron','Aileron','Aileron hinge','Aileron hinge', ...
        'V-tail','V-tail','Ruddervator','Ruddervator'}, ...
        'Location','bestoutside');

% ------------------------------------------------------------
% 10B) FRONT VIEW
% ------------------------------------------------------------
subplot(1,2,2); hold on; axis equal; grid on;
title('Front View');
xlabel('Y [ft]');
ylabel('Z [ft]');

% Wing span reference
plot([-y_tip y_tip],[0 0],'k-','LineWidth',2);

% Aileron active span
plot([ y_a0  y_tip],[0 0],'b-','LineWidth',6);
plot([-y_a0 -y_tip],[0 0],'b-','LineWidth',6);

% V-tail panel lines
plot([ y0p  y2p],[z0 z2],'k-','LineWidth',2);
plot([-y0p -y2p],[z0 z2],'k-','LineWidth',2);

% Ruddervator active span
plot([ y1p  y2p],[z1 z2],'-','Color',[0.85 0.33 0.1],'LineWidth',6);
plot([-y1p -y2p],[z1 z2],'-','Color',[0.85 0.33 0.1],'LineWidth',6);

% Front-view labels
text(0.5*(y_a0+y_tip), 0.08, sprintf('%.2f in', ail_span_in), ...
    'Color','b', 'FontWeight','bold', ...
    'HorizontalAlignment','center', 'BackgroundColor','w');

text(0.5*(y1p+y2p), 0.5*(z1+z2)+0.08, sprintf('%.2f in', rv_span_front_in), ...
    'Color',[0.85 0.33 0.1], 'FontWeight','bold', ...
    'HorizontalAlignment','center', 'BackgroundColor','w');

legend({'Wing span','Aileron span','Aileron span','V-tail R','V-tail L', ...
        'Ruddervator R','Ruddervator L'}, ...
        'Location','bestoutside');

%% =========================
% 11) CONTROL GEOMETRY SUMMARY
% =========================
fprintf('\n================ CONTROL GEOMETRY SUMMARY ================\n');

fprintf('Aileron:\n');
fprintf('  Inboard start y = %.3f in\n', 12*y_a0);
fprintf('  Outboard end y  = %.3f in\n', 12*y_tip);
fprintf('  Span            = %.3f in\n', ail_span_in);
fprintf('  Hinge x         = %.3f in\n', 12*x_hinge_ail);
fprintf('  Control chord   = %.3f in\n', ail_chord_in);

fprintf('\nRuddervator:\n');
fprintf('  Inboard start eta = %.4f\n', eta_rv0);
fprintf('  Start point (x,y,z) = (%.3f, %.3f, %.3f) in\n', 12*x1, 12*y1p, 12*z1);
fprintf('  Tip point   (x,y,z) = (%.3f, %.3f, %.3f) in\n', 12*x2, 12*y2p, 12*z2);
fprintf('  Span along panel    = %.3f in\n', rv_span_top_in);
fprintf('  Front-view span     = %.3f in\n', rv_span_front_in);
fprintf('  Root hinge x        = %.3f in\n', 12*x_hinge0);
fprintf('  Tip hinge x         = %.3f in\n', 12*x_hinge2);
fprintf('  Root ctrl chord     = %.3f in\n', rv_chord_root_in);
fprintf('  Tip ctrl chord      = %.3f in\n', rv_chord_tip_in);

%% ============================================================
% WRITE 1) GEOMETRY FILE
% ============================================================
fid = fopen(geomFile,'w');
assert(fid>0, "Could not write geometry file.");

fprintf(fid,"JAGER Geometry V_control (generated by MATLAB)\n");
fprintf(fid,"%.4f\n", Mach);
fprintf(fid,"0 0 0.0\n");
fprintf(fid,"%.4f %.4f %.4f\n", Sref, Cref, Bref);
fprintf(fid,"%.4f %.4f %.4f\n", Xref, Yref, Zref);
fprintf(fid,"%.6f\n\n", CDp);

% ---------------- Wing ----------------
fprintf(fid,"SURFACE\nWing\n");
fprintf(fid,"10 1.0 24 1.0\n");
fprintf(fid,"YDUPLICATE\n0.0\n\n");

fprintf(fid,"SECTION\n");
fprintf(fid,"%.4f 0.0000 0.0000 %.4f 0.0000\n", XwingLE, c_w);
write_airfoil(fid, wing_airfoil_type, wing_naca, "");

fprintf(fid,"SECTION\n");
fprintf(fid,"%.4f %.4f 0.0000 %.4f 0.0000\n", XwingLE, y_a0, c_w);
write_airfoil(fid, wing_airfoil_type, wing_naca, "");
fprintf(fid,"CONTROL\n");
fprintf(fid,"aileron %.3f %.3f 0 0 0 %.1f\n\n", ...
    aileron_gain, Xhinge_ail, aileron_SgnDup);

fprintf(fid,"SECTION\n");
fprintf(fid,"%.4f %.4f 0.0000 %.4f 0.0000\n", XwingLE, y_tip, c_w);
write_airfoil(fid, wing_airfoil_type, wing_naca, "");
fprintf(fid,"CONTROL\n");
fprintf(fid,"aileron %.3f %.3f 0 0 0 %.1f\n\n", ...
    aileron_gain, Xhinge_ail, aileron_SgnDup);

% ---------------- Vtail Right ----------------
fprintf(fid,"SURFACE\nVtail_R\n");
fprintf(fid,"8 1.0 16 1.0\n\n");

fprintf(fid,"SECTION\n");
fprintf(fid,"%.4f %.4f %.4f %.4f 0.0000\n", x0, +y0p, z0, c0);
write_airfoil(fid, vtail_airfoil_type, vtail_naca, "");

fprintf(fid,"SECTION\n");
fprintf(fid,"%.4f %.4f %.4f %.4f 0.0000\n", x1, +y1p, z1, c1);
write_airfoil(fid, vtail_airfoil_type, vtail_naca, "");

fprintf(fid,"CONTROL\n");
fprintf(fid,"pitchv %.3f %.3f 0 0 0 1.0\n", +pitchv_gain, Xhinge_rv);
fprintf(fid,"CONTROL\n");
fprintf(fid,"yawv %.3f %.3f 0 0 0 1.0\n\n", +yawv_gain, Xhinge_rv);

fprintf(fid,"SECTION\n");
fprintf(fid,"%.4f %.4f %.4f %.4f 0.0000\n", x2, +y2p, z2, c2);
write_airfoil(fid, vtail_airfoil_type, vtail_naca, "");

fprintf(fid,"CONTROL\n");
fprintf(fid,"pitchv %.3f %.3f 0 0 0 1.0\n", +pitchv_gain, Xhinge_rv);
fprintf(fid,"CONTROL\n");
fprintf(fid,"yawv %.3f %.3f 0 0 0 1.0\n\n", +yawv_gain, Xhinge_rv);

% ---------------- Vtail Left ----------------
fprintf(fid,"SURFACE\nVtail_L\n");
fprintf(fid,"8 1.0 16 1.0\n\n");

fprintf(fid,"SECTION\n");
fprintf(fid,"%.4f %.4f %.4f %.4f 0.0000\n", x0, -y0p, z0, c0);
write_airfoil(fid, vtail_airfoil_type, vtail_naca, "");

fprintf(fid,"SECTION\n");
fprintf(fid,"%.4f %.4f %.4f %.4f 0.0000\n", x1, -y1p, z1, c1);
write_airfoil(fid, vtail_airfoil_type, vtail_naca, "");

fprintf(fid,"CONTROL\n");
fprintf(fid,"pitchv %.3f %.3f 0 0 0 1.0\n", -pitchv_gain, Xhinge_rv);
fprintf(fid,"CONTROL\n");
fprintf(fid,"yawv %.3f %.3f 0 0 0 1.0\n\n", +yawv_gain, Xhinge_rv);

fprintf(fid,"SECTION\n");
fprintf(fid,"%.4f %.4f %.4f %.4f 0.0000\n", x2, -y2p, z2, c2);
write_airfoil(fid, vtail_airfoil_type, vtail_naca, "");

fprintf(fid,"CONTROL\n");
fprintf(fid,"pitchv %.3f %.3f 0 0 0 1.0\n", -pitchv_gain, Xhinge_rv);
fprintf(fid,"CONTROL\n");
fprintf(fid,"yawv %.3f %.3f 0 0 0 1.0\n\n", +yawv_gain, Xhinge_rv);

fclose(fid);

%% ============================================================
% WRITE 2) MASS FILE
% ============================================================
fid = fopen(massFile,'w');
assert(fid>0, "Could not write mass file.");

fprintf(fid,"# Jager mass file (generated by MATLAB)\n");
fprintf(fid,"# mass  x  y  z  Ixx  Iyy  Izz  Ixz  Ixy  Iyz\n");
fprintf(fid,"# mass units: slug\n");
fprintf(fid,"# length units: ft\n");
fprintf(fid,"# inertia units: slug*ft^2\n");
fprintf(fid,"%.6f  %.6f  %.6f  %.6f  %.6f  %.6f  %.6f  %.6f  %.6f  %.6f\n", ...
    m_slug, Xcg_ft, Ycg_ft, Zcg_ft, Ixx, Iyy, Izz, Ixz, Ixy, Iyz);

fclose(fid);

%% =========================
% 12) PRINT MODEL SUMMARY
% =========================
fprintf("\n================ MODEL SUMMARY ================\n");
fprintf("CG     = (%.4f, %.4f, %.4f) ft\n", Xcg_ft, Ycg_ft, Zcg_ft);
fprintf("Sref   = %.4f ft^2\n", Sref);
fprintf("Cref   = %.4f ft\n", Cref);
fprintf("Bref   = %.4f ft\n", Bref);
fprintf("Alpha  = %.2f deg\n", alpha_deg);
fprintf("Beta   = %.2f deg\n", beta_deg);
fprintf("V      = %.2f ft/s\n", V_ftps);
fprintf("qbar   = %.4f psf\n", qbar);

%% =========================
% 13) RUN AILERON CASES
% =========================
fprintf("\n================ AILERON TEST ================\n");
fprintf("Control tested = %s\n", controlMenu_ail);

out0_ail = run_one_case(avlExe, scriptDir, geomName, massName, cmdFile, ftFile, hmFile, logFile, ...
    alpha_deg, beta_deg, controlMenu_ail, delta0);

outP_ail = run_one_case(avlExe, geomFile, massFile, cmdFile, ftFile, hmFile, logFile, ...
    alpha_deg, beta_deg, controlMenu_ail, deltaP);

outM_ail = run_one_case(avlExe, geomFile, massFile, cmdFile, ftFile, hmFile, logFile, ...
    alpha_deg, beta_deg, controlMenu_ail, deltaM);

report_control_results(out0_ail, outP_ail, outM_ail, controlName_ail, qbar, Sref, Bref, Ixx, "AILERON");

%% =========================
% 14) RUN V-TAIL PITCH MIX CASES
% =========================
fprintf("\n================ V-TAIL PITCH MIX TEST ================\n");
fprintf("Control tested = %s\n", controlMenu_pitchv);

out0_pitchv = run_one_case(avlExe, geomFile, massFile, cmdFile, ftFile, hmFile, logFile, ...
    alpha_deg, beta_deg, controlMenu_pitchv, delta0);

outP_pitchv = run_one_case(avlExe, geomFile, massFile, cmdFile, ftFile, hmFile, logFile, ...
    alpha_deg, beta_deg, controlMenu_pitchv, deltaP);

outM_pitchv = run_one_case(avlExe, geomFile, massFile, cmdFile, ftFile, hmFile, logFile, ...
    alpha_deg, beta_deg, controlMenu_pitchv, deltaM);

report_control_results(out0_pitchv, outP_pitchv, outM_pitchv, controlName_pitchv, qbar, Sref, Bref, Ixx, "V-TAIL PITCH MIX");

%% =========================
% 15) RUN V-TAIL YAW MIX CASES
% =========================
fprintf("\n================ V-TAIL YAW MIX TEST ================\n");
fprintf("Control tested = %s\n", controlMenu_yawv);

out0_yawv = run_one_case(avlExe, geomFile, massFile, cmdFile, ftFile, hmFile, logFile, ...
    alpha_deg, beta_deg, controlMenu_yawv, delta0);

outP_yawv = run_one_case(avlExe, geomFile, massFile, cmdFile, ftFile, hmFile, logFile, ...
    alpha_deg, beta_deg, controlMenu_yawv, deltaP);

outM_yawv = run_one_case(avlExe, geomFile, massFile, cmdFile, ftFile, hmFile, logFile, ...
    alpha_deg, beta_deg, controlMenu_yawv, deltaM);

report_control_results(out0_yawv, outP_yawv, outM_yawv, controlName_yawv, qbar, Sref, Bref, Ixx, "V-TAIL YAW MIX");

fprintf('\nDone.\n');

%% ============================================================
% LOCAL FUNCTIONS
% ============================================================

function out = run_one_case(avlExe, scriptDir, geomName, massName, cmdFile, ftFile, hmFile, logFile, ...
    alpha_deg, beta_deg, controlMenu, controlValue)

    safeDelete(cmdFile);
    safeDelete(logFile);
    safeDelete(ftFile);
    safeDelete(hmFile);

    [~, cmdName, cmdExt] = fileparts(cmdFile);
    [~, ftName,  ftExt ] = fileparts(ftFile);
    [~, hmName,  hmExt ] = fileparts(hmFile);
    [~, logName, logExt] = fileparts(logFile);

    cmdBase = cmdName + cmdExt;
    ftBase  = ftName  + ftExt;
    hmBase  = hmName  + hmExt;
    logBase = logName + logExt;

    fid = fopen(cmdFile,'w');
    assert(fid>0, "Could not write AVL command file.");

    fprintf(fid,"LOAD %s\n", geomName);
    fprintf(fid,"MASS %s\n", massName);
    fprintf(fid,"OPER\n");

    fprintf(fid,"A\n");
    fprintf(fid,"%.6f\n", alpha_deg);

    fprintf(fid,"B\n");
    fprintf(fid,"%.6f\n", beta_deg);

    fprintf(fid,"%s\n", controlMenu);
    fprintf(fid,"%.6f\n", controlValue);

    fprintf(fid,"X\n");

    fprintf(fid,"FT\n");
    fprintf(fid,"%s\n", ftBase);

    fprintf(fid,"HM\n");
    fprintf(fid,"%s\n", hmBase);

    fprintf(fid,"QUIT\n");
    fclose(fid);

    % Run AVL from the script folder so LOAD/MASS/FT/HM use local filenames
    syscmd = sprintf('cd "%s" && "%s" < "%s" > "%s" 2>&1', ...
        scriptDir, avlExe, cmdBase, logBase);

    [status, cmdout] = system(syscmd);

    fprintf("\n------------------------------------------------------------\n");
    fprintf("AVL run for %s = %.3f deg | exit status = %d\n", controlMenu, controlValue, status);
    fprintf("------------------------------------------------------------\n");
    fprintf("%s\n", cmdout);

    if ~isfile(logFile)
        warning("AVL log file was not created.");
    end

    if ~isfile(ftFile)
        fprintf('\n================ AVL COMMAND FILE ================\n');
        type(cmdFile);

        fprintf('\n================ AVL LOG FILE ================\n');
        if isfile(logFile)
            type(logFile);
        else
            fprintf('Log file was not created.\n');
        end

        error("FT output file was not created. See command file and AVL log above.");
    end

    if status ~= 0
        warning("AVL returned nonzero exit status, but FT file was created. Continuing.");
    end

    out = struct();
    out.FT = parseFTfile(ftFile);

    if isfile(hmFile)
        out.HM = parseHMfile(hmFile);
    else
        out.HM = struct('name',{},'Chinge',{},'deflection_deg',{},'moment_Nm',{});
    end
end

function report_control_results(out0, outP, outM, controlName, qbar, Sref, Bref, Ixx, label)

    fprintf('\n================ %s BASELINE FT ================\n', label);
    printFT(out0.FT);

    fprintf('\n================ %s POSITIVE FT ================\n', label);
    printFT(outP.FT);

    fprintf('\n================ %s NEGATIVE FT ================\n', label);
    printFT(outM.FT);

    fprintf('\n================ %s BASELINE HM ================\n', label);
    printHM(out0.HM);

    fprintf('\n================ %s POSITIVE HM ================\n', label);
    printHM(outP.HM);

    fprintf('\n================ %s NEGATIVE HM ================\n', label);
    printHM(outM.HM);

    deltaPlusEcho  = get_control_value(outP.FT.controls, controlName);
    deltaMinusEcho = get_control_value(outM.FT.controls, controlName);

    fprintf('\n========== %s ACTUAL ECHOED DEFLECTIONS ==========\n', label);
    fprintf('delta+ = %+8.4f deg\n', deltaPlusEcho);
    fprintf('delta- = %+8.4f deg\n', deltaMinusEcho);

    if isnan(deltaPlusEcho) || isnan(deltaMinusEcho)
        error("Could not find echoed control '%s' in FT output.", controlName);
    end

    den = deltaPlusEcho - deltaMinusEcho;
    if abs(den) < 1e-12
        error('%s deflection denominator is zero.', label);
    end

    dCl_dDelta = (outP.FT.Cl - outM.FT.Cl) / den;
    dCm_dDelta = (outP.FT.Cm - outM.FT.Cm) / den;
    dCn_dDelta = (outP.FT.Cn - outM.FT.Cn) / den;
    dCY_dDelta = (outP.FT.CY - outM.FT.CY) / den;
    dCL_dDelta = (outP.FT.CL - outM.FT.CL) / den;
    dCD_dDelta = (outP.FT.CD - outM.FT.CD) / den;

    fprintf('\n========== %s FINITE-DIFFERENCE DERIVATIVES ==========\n', label);
    fprintf('dCl/dDelta = %+10.6f per deg\n', dCl_dDelta);
    fprintf('dCm/dDelta = %+10.6f per deg\n', dCm_dDelta);
    fprintf('dCn/dDelta = %+10.6f per deg\n', dCn_dDelta);
    fprintf('dCY/dDelta = %+10.6f per deg\n', dCY_dDelta);
    fprintf('dCL/dDelta = %+10.6f per deg\n', dCL_dDelta);
    fprintf('dCD/dDelta = %+10.6f per deg\n', dCD_dDelta);

    % Keep roll-authority estimate for consistency across controls
    delta_cmd_deg = 10.0;
    Cl_avail      = dCl_dDelta * delta_cmd_deg;
    L_avail       = Cl_avail * qbar * Sref * Bref;
    pdot_est      = L_avail / Ixx;

    fprintf('\n========== %s ESTIMATED ROLL AUTHORITY ==========\n', label);
    fprintf('Using delta_cmd = %.2f deg\n', delta_cmd_deg);
    fprintf('Estimated Cl_avail = %.6f\n', Cl_avail);
    fprintf('Estimated roll moment L = %.6f lbf*ft\n', L_avail);
    fprintf('Estimated roll acceleration p_dot = %.6f rad/s^2\n', pdot_est);

    fprintf('\n========== %s QUICK INTERPRETATION ==========\n', label);
    fprintf('Roll effectiveness magnitude : |dCl/dDelta| = %.6f per deg\n', abs(dCl_dDelta));
    fprintf('Pitch effectiveness magnitude: |dCm/dDelta| = %.6f per deg\n', abs(dCm_dDelta));
    fprintf('Yaw effectiveness magnitude  : |dCn/dDelta| = %.6f per deg\n', abs(dCn_dDelta));
    fprintf('Side-force coupling magnitude: |dCY/dDelta| = %.6f per deg\n', abs(dCY_dDelta));
end

function FT = parseFTfile(ftFile)
    txt = fileread(ftFile);

    FT = struct();
    FT.CL = grabScalar(txt, 'CLtot');
    FT.CD = grabScalar(txt, 'CDtot');
    FT.CY = grabScalar(txt, 'CYtot');
    FT.Cl = grabScalar(txt, 'Cltot');
    FT.Cm = grabScalar(txt, 'Cmtot');
    FT.Cn = grabScalar(txt, 'Cntot');
    FT.controls = parseControlEchoes(txt);
end

function x = grabScalar(txt, label)
    pat = [label '\s*=\s*([-+]?\d*\.?\d+(?:[EeDd][-+]?\d+)?)'];
    tok = regexp(txt, pat, 'tokens', 'once');

    if isempty(tok)
        error('Could not find %s in FT file. File contents were:\n\n%s', label, txt);
    end

    x = str2double(strrep(tok{1}, 'D', 'E'));
end

function controls = parseControlEchoes(txt)
    controls = struct('name',{},'value_deg',{});

    lines = regexp(txt, '\r\n|\n|\r', 'split');
    skipNames = ["CLtot","CDtot","CYtot","Cltot","Cmtot","Cntot", ...
                 "CXtot","CZtot","CDvis","CDind","CLff","CDff","CYff", ...
                 "Alpha","Beta","Mach"];

    for i = 1:numel(lines)
        line = strtrim(lines{i});

        tok = regexp(line, ...
            '^([A-Za-z0-9_\-]+)\s*=\s*([-+]?\d*\.?\d+(?:[EeDd][-+]?\d+)?)$', ...
            'tokens', 'once');

        if ~isempty(tok)
            nm = string(tok{1});
            if ~ismember(nm, skipNames)
                s = struct();
                s.name = nm;
                s.value_deg = str2double(strrep(tok{2}, 'D', 'E'));
                controls(end+1) = s; %#ok<AGROW>
            end
        end
    end
end

function val = get_control_value(ctrls, targetName)
    val = NaN;
    for k = 1:numel(ctrls)
        if string(ctrls(k).name) == string(targetName)
            val = ctrls(k).value_deg;
            return;
        end
    end
end

function printFT(FT)
    fprintf('CL = %+10.6f\n', FT.CL);
    fprintf('CD = %+10.6f\n', FT.CD);
    fprintf('CY = %+10.6f\n', FT.CY);
    fprintf('Cl = %+10.6f\n', FT.Cl);
    fprintf('Cm = %+10.6f\n', FT.Cm);
    fprintf('Cn = %+10.6f\n', FT.Cn);

    if ~isempty(FT.controls)
        fprintf('Echoed controls:\n');
        for k = 1:numel(FT.controls)
            fprintf('   %-12s = %+8.4f deg\n', FT.controls(k).name, FT.controls(k).value_deg);
        end
    end
end

function printHM(HM)
    if isempty(HM)
        fprintf('No HM data parsed.\n');
        return;
    end

    for k = 1:numel(HM)
        fprintf('   %-12s | Chinge = %+10.6e | defl = %+8.4f deg | M = %+10.6f N-m\n', ...
            HM(k).name, HM(k).Chinge, HM(k).deflection_deg, HM(k).moment_Nm);
    end
end

function safeDelete(fname)
    if isfile(fname)
        delete(fname);
    end
end

function touch_empty(fname)
    fid = fopen(fname,'w');
    if fid >= 0
        fclose(fid);
    else
        error("Could not create placeholder file: %s", fname);
    end
end

function write_airfoil(fid, airfoil_type, naca_code, afile)
    switch upper(string(airfoil_type))
        case "NACA"
            fprintf(fid,"NACA\n");
            fprintf(fid,"%s\n\n", string(naca_code));
        case "AFILE"
            fprintf(fid,"AFILE\n");
            fprintf(fid,"%s\n\n", string(afile));
        otherwise
            error("Unsupported airfoil type: %s", string(airfoil_type));
    end
end