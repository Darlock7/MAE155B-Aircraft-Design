%% ============================================================
%  main_aileronSweep_JAGER.m
%
%  PURPOSE:
%   Sweep aileron geometry using AVL and rank cases by roll authority
%   while penalizing yaw/pitch coupling and hinge loading.
%
%  SWEEP VARIABLES:
%   - eta_a0      = aileron inboard start fraction of semispan
%   - Xhinge_ail  = hinge x/c
%
%  OUTPUTS:
%   - command-window progress and ranking
%   - best case summary
%   - best/final geometry plot with labels in inches
% ============================================================

clear; clc; close all;

%% =========================
% 1) FIXED AIRCRAFT / AVL SETTINGS
% =========================
p = struct();

% files / executable
p.avlExe   = "avl352.exe";
p.geomFile = "jager.geo.avl";
p.massFile = "jager.mass";
p.cmdFile  = "jager_control_commands.txt";
p.ftFile   = "jager_ft.txt";
p.hmFile   = "jager_hm.txt";
p.logFile  = "jager_avl_log.txt";

% constants
p.g_ft = 32.174;
p.rho  = 0.0023769;
p.Mach = 0.00;
p.CDp  = 0.02813;

% aircraft weight / cg / inertia
p.W_lbf  = 55;
p.m_slug = p.W_lbf / p.g_ft;

p.Xcg_in = 39.1;
p.Ycg_ft = 0.0;
p.Zcg_ft = -1.74/12;
p.Xcg_ft = p.Xcg_in/12;

p.Ixx_sw =  75.7671;
p.Iyy_sw = 119.6108;
p.Izz_sw = 191.5794;

convI = 1/32.174;
p.Ixx = p.Ixx_sw * convI;
p.Iyy = p.Iyy_sw * convI;
p.Izz = p.Izz_sw * convI;

p.Ixy = 0;
p.Ixz = 0;
p.Iyz = 0;

% reference geometry
p.Sref = 6.64;
p.Cref = 1.052;
p.Bref = 6.31;
p.b2   = p.Bref/2;

p.Xref = p.Xcg_ft;
p.Yref = 0.0;
p.Zref = p.Zcg_ft;

% wing datum
p.XwingLE_in = 35.0;
p.XwingLE    = p.XwingLE_in/12;

% flight condition
p.V_ftps    = 75.0;
p.alpha_deg = 8.25;
p.beta_deg  = 0.0;
p.qbar      = 0.5 * p.rho * p.V_ftps^2;

% wing / aileron baseline
p.c_w = p.Cref;
p.y_tip = p.b2;

p.wing_airfoil_type = "NACA";
p.wing_naca         = "4412";

p.aileron_gain   = 1.0;
p.aileron_SgnDup = -1.0;

% V-tail fixed geometry
p.S_panel      = 0.809;
p.AR_tail      = 2.8;
p.taper_vt     = 0.4;
p.gamma_deg    = 36.38;
p.LambdaLE_deg = 45.0;
p.XtailQC      = 80.3636/12;

p.b_panel   = sqrt(p.AR_tail*p.S_panel);
p.c_root_vt = 2*p.S_panel/(p.b_panel*(1+p.taper_vt));
p.c_tip_vt  = p.taper_vt*p.c_root_vt;
p.XtailLE_root = p.XtailQC - 0.25*p.c_root_vt;

p.yT = p.b_panel*cosd(p.gamma_deg);
p.zT = p.b_panel*sind(p.gamma_deg);

p.eta_rv0 = 0.30;

p.vtail_airfoil_type = "NACA";
p.vtail_naca         = "0010";

p.Xhinge_rv  = 0.70;
p.pitchv_gain = 1.0;
p.yawv_gain   = 1.0;

% control test setup
p.controlMenu = "D1";
p.controlName = "aileron";
p.delta0 = 0.0;
p.deltaP = 4.0;
p.deltaM = -4.0;
p.delta_cmd_deg = 10.0;

%% =========================
% 2) SWEEP RANGES
% =========================
eta_a0_vec      = [0.50 0.55 0.60 0.65];
Xhinge_ail_vec  = [0.70 0.75 0.80];

n_eta   = numel(eta_a0_vec);
n_hinge = numel(Xhinge_ail_vec);
nCases  = n_eta * n_hinge;

fprintf('\nStarting aileron sweep...\n');
fprintf('eta_a0 points    : %d\n', n_eta);
fprintf('Xhinge_ail points: %d\n', n_hinge);
fprintf('Total cases      : %d\n\n', nCases);

%% =========================
% 3) SCORE WEIGHTS
% =========================
w_roll  = 1.0;
w_yaw   = 2.0;
w_pitch = 1.0;
w_hinge = 1.0;

%% =========================
% 4) PREALLOCATE RESULTS
% =========================
results = repmat(struct( ...
    'success', false, ...
    'errMsg', "", ...
    'eta_a0', NaN, ...
    'Xhinge_ail', NaN, ...
    'score', NaN, ...
    'out', struct()), nCases, 1);

caseIdx = 0;
bestScore = -Inf;
bestIdx   = NaN;

%% =========================
% 5) SWEEP LOOP
% =========================
for i = 1:n_eta
    for j = 1:n_hinge
        caseIdx = caseIdx + 1;

        p.eta_a0     = eta_a0_vec(i);
        p.Xhinge_ail = Xhinge_ail_vec(j);

        out = runAVL_aileronCase_JAGER(p);

        results(caseIdx).success     = out.success;
        results(caseIdx).errMsg      = out.errMsg;
        results(caseIdx).eta_a0      = p.eta_a0;
        results(caseIdx).Xhinge_ail  = p.Xhinge_ail;
        results(caseIdx).out         = out;

        if out.success
            score = w_roll  * abs(out.derivs.dCl_dDelta) ...
                  - w_yaw   * abs(out.derivs.dCn_dDelta) ...
                  - w_pitch * abs(out.derivs.dCm_dDelta) ...
                  - w_hinge * abs(out.hinge.Chinge_abs_max);

            results(caseIdx).score = score;

            fprintf(['Case %3d / %3d | eta_a0=%.3f | Xhinge=%.3f | ' ...
                     'dCl/dD=%+.6f | dCn/dD=%+.6f | Ch=%+.6e | Score=%+.6f\n'], ...
                     caseIdx, nCases, p.eta_a0, p.Xhinge_ail, ...
                     out.derivs.dCl_dDelta, out.derivs.dCn_dDelta, ...
                     out.hinge.Chinge_abs_max, score);

            if score > bestScore
                bestScore = score;
                bestIdx   = caseIdx;
                fprintf('   NEW BEST\n');
            end
        else
            fprintf(['Case %3d / %3d | eta_a0=%.3f | Xhinge=%.3f | FAILED\n' ...
                     '   Reason: %s\n'], ...
                     caseIdx, nCases, p.eta_a0, p.Xhinge_ail, out.errMsg);
        end
    end
end

%% =========================
% 6) CHECK BEST CASE
% =========================
if isnan(bestIdx)
    error('No successful cases found in sweep.');
end

best = results(bestIdx);
bestOut = best.out;

fprintf('\n============================================================\n');
fprintf('BEST AILERON CASE\n');
fprintf('============================================================\n');
fprintf('eta_a0          = %.4f\n', best.eta_a0);
fprintf('Xhinge_ail      = %.4f\n', best.Xhinge_ail);
fprintf('Score           =  %+10.6f\n', best.score);
fprintf('dCl/dDelta      =  %+10.6f per deg\n', bestOut.derivs.dCl_dDelta);
fprintf('dCn/dDelta      =  %+10.6f per deg\n', bestOut.derivs.dCn_dDelta);
fprintf('dCm/dDelta      =  %+10.6f per deg\n', bestOut.derivs.dCm_dDelta);
fprintf('Chinge max      =  %+10.6e\n', bestOut.hinge.Chinge_abs_max);
fprintf('p_dot @ 10 deg  =  %+10.6f rad/s^2\n', bestOut.roll.pdot_10deg);
fprintf('p_dot @ 10 deg  =  %+10.6f deg/s^2\n', bestOut.roll.pdot_10deg_deg);

%% =========================
% 7) OPTIONAL: SHOW SORTED TABLE IN COMMAND WINDOW
% =========================
validMask = [results.success];
validResults = results(validMask);

scores = [validResults.score]';
[~, order] = sort(scores, 'descend');
validResults = validResults(order);

fprintf('\nTop cases:\n');
nShow = min(5, numel(validResults));
for k = 1:nShow
    r = validResults(k);
    fprintf(['%d) eta_a0=%.3f | Xhinge=%.3f | Score=%+.6f | ' ...
             'dCl/dD=%+.6f | dCn/dD=%+.6f | Ch=%+.6e\n'], ...
             k, r.eta_a0, r.Xhinge_ail, r.score, ...
             r.out.derivs.dCl_dDelta, r.out.derivs.dCn_dDelta, ...
             r.out.hinge.Chinge_abs_max);
end

%% =========================
% 8) PLOT BEST / FINAL GEOMETRY
% =========================
eta_a0_best     = best.eta_a0;
Xhinge_ail_best = best.Xhinge_ail;

% derived best geometry
x_hinge_ail = p.XwingLE + Xhinge_ail_best*p.c_w;
y_a0        = eta_a0_best*p.b2;
y_tip       = p.y_tip;

% V-tail shared stations
eta0 = 0.0;
eta1 = p.eta_rv0;
eta2 = 1.0;

y0p = eta0*abs(p.yT); z0 = eta0*p.zT;
y1p = eta1*abs(p.yT); z1 = eta1*p.zT;
y2p = eta2*abs(p.yT); z2 = eta2*p.zT;

c0 = p.c_root_vt + (p.c_tip_vt - p.c_root_vt)*eta0;
c1 = p.c_root_vt + (p.c_tip_vt - p.c_root_vt)*eta1;
c2 = p.c_root_vt + (p.c_tip_vt - p.c_root_vt)*eta2;

x0 = p.XtailLE_root + y0p*tand(p.LambdaLE_deg);
x1 = p.XtailLE_root + y1p*tand(p.LambdaLE_deg);
x2 = p.XtailLE_root + y2p*tand(p.LambdaLE_deg);

x_hinge0 = x1 + p.Xhinge_rv*c1;
x_hinge2 = x2 + p.Xhinge_rv*c2;

% dimensions
ail_span      = y_tip - y_a0;
ail_chord     = (1 - Xhinge_ail_best)*p.c_w;
rv_span_top   = sqrt((x2-x1)^2 + (y2p-y1p)^2);
rv_span_front = sqrt((y2p-y1p)^2 + (z2-z1)^2);
rv_chord_root = (1 - p.Xhinge_rv)*c1;
rv_chord_tip  = (1 - p.Xhinge_rv)*c2;

ft2in = 12.0;
ail_span_in      = ail_span * ft2in;
ail_chord_in     = ail_chord * ft2in;
rv_span_top_in   = rv_span_top * ft2in;
rv_span_front_in = rv_span_front * ft2in;
rv_chord_root_in = rv_chord_root * ft2in;
rv_chord_tip_in  = rv_chord_tip * ft2in;

figure('Name','BEST Aileron Geometry','Color','w');

% ------------------------------------------------------------
% Top View
% ------------------------------------------------------------
subplot(1,2,1); hold on; axis equal; grid on;
title('Best / Final Geometry - Top View');
xlabel('X [ft]');
ylabel('Y [ft]');

xw = [p.XwingLE, p.XwingLE + p.c_w, p.XwingLE + p.c_w, p.XwingLE];
yw = [0,         0,                 y_tip,             y_tip];
patch(xw, yw, [0.85 0.85 0.85], 'EdgeColor','k', 'LineWidth',1.2);
patch(xw, -yw, [0.85 0.85 0.85], 'EdgeColor','k', 'LineWidth',1.2);

xa = [x_hinge_ail, p.XwingLE + p.c_w, p.XwingLE + p.c_w, x_hinge_ail];
ya = [y_a0,        y_a0,              y_tip,             y_tip];
patch(xa, ya, [0.2 0.6 1.0], 'FaceAlpha',0.45, 'EdgeColor','b', 'LineWidth',1.2);
patch(xa, -ya, [0.2 0.6 1.0], 'FaceAlpha',0.45, 'EdgeColor','b', 'LineWidth',1.2);

plot([x_hinge_ail x_hinge_ail], [ y_a0  y_tip], 'b--', 'LineWidth',1.2);
plot([x_hinge_ail x_hinge_ail], [-y_a0 -y_tip], 'b--', 'LineWidth',1.2);

x_vr = [x0, x0 + c0, x2 + c2, x2];
y_vr = [y0p, y0p,    y2p,     y2p];
patch(x_vr, y_vr, [0.80 0.90 0.80], 'EdgeColor','k', 'LineWidth',1.2);

x_vl = [x0, x0 + c0, x2 + c2, x2];
y_vl = [-y0p, -y0p,  -y2p,    -y2p];
patch(x_vl, y_vl, [0.80 0.90 0.80], 'EdgeColor','k', 'LineWidth',1.2);

xcr = [x_hinge0, x1 + c1, x2 + c2, x_hinge2];
ycr = [y1p,      y1p,     y2p,     y2p];
patch(xcr, ycr, [1.0 0.5 0.2], 'FaceAlpha',0.45, ...
    'EdgeColor',[0.85 0.33 0.1], 'LineWidth',1.2);

xcl = [x_hinge0, x1 + c1, x2 + c2, x_hinge2];
ycl = [-y1p,     -y1p,    -y2p,    -y2p];
patch(xcl, ycl, [1.0 0.5 0.2], 'FaceAlpha',0.45, ...
    'EdgeColor',[0.85 0.33 0.1], 'LineWidth',1.2);

plot([x_hinge0 x_hinge2], [ y1p  y2p], '--', 'Color',[0.85 0.33 0.1], 'LineWidth',1.2);
plot([x_hinge0 x_hinge2], [-y1p -y2p], '--', 'Color',[0.85 0.33 0.1], 'LineWidth',1.2);

text(p.XwingLE + 0.5*p.c_w, 0.5*(y_a0+y_tip), ...
    sprintf('Aileron span = %.2f in', ail_span_in), ...
    'Color','b', 'FontWeight','bold', ...
    'HorizontalAlignment','center', 'BackgroundColor','w');

text(0.5*(x_hinge_ail + p.XwingLE + p.c_w), y_tip + 0.08, ...
    sprintf('Aileron chord = %.2f in', ail_chord_in), ...
    'Color','b', 'FontWeight','bold', ...
    'HorizontalAlignment','center', 'BackgroundColor','w');

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
% Front View
% ------------------------------------------------------------
subplot(1,2,2); hold on; axis equal; grid on;
title('Best / Final Geometry - Front View');
xlabel('Y [ft]');
ylabel('Z [ft]');

plot([-y_tip y_tip],[0 0],'k-','LineWidth',2);
plot([ y_a0  y_tip],[0 0],'b-','LineWidth',6);
plot([-y_a0 -y_tip],[0 0],'b-','LineWidth',6);

plot([ y0p  y2p],[z0 z2],'k-','LineWidth',2);
plot([-y0p -y2p],[z0 z2],'k-','LineWidth',2);

plot([ y1p  y2p],[z1 z2],'-','Color',[0.85 0.33 0.1],'LineWidth',6);
plot([-y1p -y2p],[z1 z2],'-','Color',[0.85 0.33 0.1],'LineWidth',6);

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
% 9) BEST GEOMETRY SUMMARY
% =========================
fprintf('\n============================================================\n');
fprintf('BEST GEOMETRY SUMMARY\n');
fprintf('============================================================\n');

fprintf('Aileron:\n');
fprintf('  eta_a0          = %.4f\n', eta_a0_best);
fprintf('  Xhinge_ail      = %.4f\n', Xhinge_ail_best);
fprintf('  Inboard start y = %.3f in\n', 12*y_a0);
fprintf('  Outboard end y  = %.3f in\n', 12*y_tip);
fprintf('  Span            = %.3f in\n', ail_span_in);
fprintf('  Hinge x         = %.3f in\n', 12*x_hinge_ail);
fprintf('  Control chord   = %.3f in\n', ail_chord_in);

fprintf('\nRuddervator:\n');
fprintf('  Inboard start eta = %.4f\n', p.eta_rv0);
fprintf('  Start point (x,y,z) = (%.3f, %.3f, %.3f) in\n', 12*x1, 12*y1p, 12*z1);
fprintf('  Tip point   (x,y,z) = (%.3f, %.3f, %.3f) in\n', 12*x2, 12*y2p, 12*z2);
fprintf('  Span along panel    = %.3f in\n', rv_span_top_in);
fprintf('  Front-view span     = %.3f in\n', rv_span_front_in);
fprintf('  Root hinge x        = %.3f in\n', 12*x_hinge0);
fprintf('  Tip hinge x         = %.3f in\n', 12*x_hinge2);
fprintf('  Root ctrl chord     = %.3f in\n', rv_chord_root_in);
fprintf('  Tip ctrl chord      = %.3f in\n', rv_chord_tip_in);