% 155B Group 2 Main Sizing Script

% Intention:
% Shared main sizing script for the group that can evolve with the
% project as the design matures. Keep the main script organized and modular
% by calling external functions whenever practical.
% Allocate a section to each topic. When getting code from chat try to only
% paste sections to avoid mistakes

% Version Evolution:
% Version 1.0: calls J(x) function
% Version 1.1: uses energyCalc(...) as a separate function
% Version 1.2: couples VPS to payload volume, effective L/D, and effective
%              empty-weight fraction
% Version 2.0: includes mission profile via missionProfileCigar(mission)
% Version 3.0: introduces aero assumptions (CL, L/D, etc.)
% Version 4.0: includes propulsion sizing via propulsionAnalysis(...)
% Version 5.0: includes CTOL sizing equations via preliminarySizingCTOL(...)
% Version 6.0: includes wing geometry design variables via wingGeometryDesign(...)
% Version 7.0: integrates XFOIL airfoil analysis via airfoilAnalysisXFOIL(...)
% Version 8.0: extracts detailed airfoil parameters (Clalpha, Cm0, alphaL0,
%              Clmax, and best L/D)
% Version 9.0: introduces spanwise twist modeling via twistFunctionPanknin(...)
% Version 10.0: adds vertical stabilizer / winglet sizing via
%               verticalSurfaceDesign(...)
% Version 11.0: implements spanwise aerodynamic estimation via
%               spanwiseAeroEstimate(...)
% Version 12.0: introduces 3D aircraft geometry visualization via
%               plotAircraftGeometry3D(...)

clc; clearvars; close all;

timestamp = datetime('now','Format','yyyy-MM-dd HH:mm:ss');
fprintf('========= Main Sizing Code executed at: %s =======\n\n', string(timestamp));

%% ================ Given ===============
g = 9.81;                  % [m/s^2]
Wprop = 2.43341;           % [N] Total propulsion system weight
ft3_to_m3 = 0.02831685;    % [m^3/ft^3]

%% ============== Engineering Assumptions ========
eta_p = 0.75;              % [-] Propulsion efficiency
LD = 20;                   % [-] Fisrt pass L/D (UPDATE) 
reserve_factor = 1.15;     % [-] Energy margin multiplier

% Baseline empty-weight fraction
fe = 0.50;                 % [-] baseline empty weight fraction = We / Wg

% Aerodynamic penalty model for payload volume
VPS_ref = 0.4;             % [-] scalar below which no aero penalty is applied
kd = 0.5;                  % [-] L/D penalty strength for VPS > VPS_ref

% Empty-weight penalty model for payload volume
ke = 0.08;                 % [-] empty-weight-fraction penalty slope
fe_max = 0.85;             % [-] hard upper cap for sanity

%% ============== Mission Parameters ===========
delta_h = 120;             % [m]   Climb altitude change
R_cruise = 18000;          % [m]   Cruise range
Tf_measured = 61;          % [s]   Measured flight time

Wp_g = 700;                % [g]   Payload weight
Wp = (Wp_g/1000)*g;        % [N]   Payload weight

VPS = .90;                 % [-]   volumetric package scalar
Vp = ft3_to_m3 * VPS;      % [m^3] payload volume

%% ============== Effective Aircraft Penalties ===========
% Effective aerodynamic penalty from package size
if VPS <= VPS_ref
    LD_eff = LD;
else
    LD_eff = LD / (1 + kd*(VPS - VPS_ref));
end

% Effective empty-weight fraction penalty from package size
if VPS <= VPS_ref
    fe_eff = fe;
else
    fe_eff = fe + ke*(VPS - VPS_ref);
end

fe_eff = min(fe_eff, fe_max);

%% ============== Derived Weights ===========
Wg = (Wp + Wprop) / (1 - fe_eff);   % [N] Gross weight
We = fe_eff * Wg;                   % [N] Empty weight
Wg_grams = Wg * g^(-1) * 1000;      % [g] Gross weight

%% ============== Energy Calculation ===========
fprintf('================ Energy Calculation ======================\n');

[E_climb, E_cruise, E_f, E_design, E_f_Wh, E_design_Wh] = ...
    energyCalc(Wg, eta_p, LD_eff, delta_h, R_cruise, reserve_factor);

% Use mission energy for scoring
Ef_measured = E_f;   % [J]

fprintf('Climb Energy            (E_climb)   = %.2f J\n', E_climb);
fprintf('Cruise Energy           (E_cruise)  = %.2f J\n', E_cruise);
fprintf('Total Mission Energy    (E_f)       = %.2f J\n', E_f);
fprintf('Total Mission Energy    (E_f)       = %.2f Wh\n', E_f_Wh);
fprintf('Design Energy w/Reserve (E_design)  = %.2f J\n', E_design);
fprintf('Design Energy w/Reserve (E_design)  = %.2f Wh\n', E_design_Wh);
fprintf('----------------------------------------------------------\n\n');

%% ================ Profit Per Unit Time J(x) ===============
fprintf('================ Profit Per Unit Time J(x) ===============\n');

% Apply competition scaling (x20)
Ef = 20 * Ef_measured;
Tf = 20 * Tf_measured;

% Print Inputs
fprintf('---------------- INPUT VARIABLES ----------------\n');
fprintf('Payload Weight             (Wp)            = %.4f N\n', Wp);
fprintf('Payload Weight             (Wp)            = %.1f g\n', Wp_g);
fprintf('Volumetric Package Scalar  (VPS)           = %.4f\n', VPS);
fprintf('Payload Volume             (Vp)            = %.6f m^3\n', Vp);
fprintf('Measured Energy            (Ef_measured)   = %.2f J\n', Ef_measured);
fprintf('Scaled Energy              (Ef = x20)      = %.2f J\n', Ef);
fprintf('Measured Flight Time       (Tf_measured)   = %.2f s\n', Tf_measured);
fprintf('Scaled Flight Time         (Tf = x20)      = %.2f s\n', Tf);
fprintf('Propulsion System Weight   (Wprop)         = %.4f N\n', Wprop);
fprintf('Baseline Empty Wt Fraction (fe)            = %.4f\n', fe);
fprintf('Effective Empty Wt Frac.   (fe_eff)        = %.4f\n', fe_eff);
fprintf('Baseline L/D               (LD)            = %.4f\n', LD);
fprintf('Effective L/D              (LD_eff)        = %.4f\n', LD_eff);
fprintf('Reference VPS              (VPS_ref)       = %.4f\n', VPS_ref);
fprintf('Aero penalty factor        (kd)            = %.4f\n', kd);
fprintf('Empty-weight penalty       (ke)            = %.4f\n', ke);
fprintf('Derived Empty Weight       (We)            = %.4f N\n', We);
fprintf('Derived Gross Weight       (Wg)            = %.4f N\n', Wg);
fprintf('Derived Gross Weight [g]   (Wg_grams)      = %.4f g\n', Wg_grams);
fprintf('-------------------------------------------------\n\n');

% Call J(x) function
J = profitPerUnitTime(Wp, Vp, Ef, Wg, Tf);

% Output
fprintf('---------------- SCORE OUTPUT -------------------\n');
fprintf('Profit per Unit Time (J) = %.8f $/s\n', J);
fprintf('Profit per Unit Time (J) = %.4f $/hr\n', J*3600);
fprintf('-------------------------------------------------\n');

%% ============== Mission Profile ===========
mission.nLaps             = 3;
mission.lapLengthTarget_m = 407.103;   % [m]
mission.V_pattern         = 20;        % [m/s]
mission.h_ground          = 0;         % [m]
mission.h_cruise          = 30;        % [m]

mission.runwayLength_m    = 138.35;    % [m]
mission.straightLength_m  = 140.0;     % [m] turn begins after this x-location

mission.liftoffFrac       = 0.75;      % [-]
mission.touchdownFrac     = 1/3;       % [-]
mission.n_turn            = 1.3;       % [-]

mission.climbRate_mps     = 3.0;       % [m/s] % can we double check this?
mission.descentRate_mps   = 2.5;       % [m/s]

missionOut = missionProfileCigar(mission);

%% ============== Propulsion Analysis ============
propIn = struct();

% Atmosphere
propIn.rho = 1.225;                    % [kg/m^3]

% Motor / battery
propIn.KV   = 1100;                    % [RPM/V]
propIn.Rm   = 0.073;                   % [ohm]
propIn.I0   = 0.9;                     % [A]
propIn.Vbat = 11.1;                    % [V]
propIn.I_max = 35;                     % [A] safer design target

% Candidate propeller
propIn.propName = 'APC 10x4.7SF';
propIn.D_in = 10;
propIn.pitch_in = 4.7;
propIn.V_vec_mps = linspace(0,25,150);

% Preliminary thrust model inputs
% T/T0 = 1 - V/V0   for V <= V0, else 0
propIn.usePrelimModel = false;
propIn.T_static_N = 9.0;               % [N] first-pass static thrust estimate
propIn.V0_mps     = 24.0;              % [m/s] speed where thrust falls to ~0

fprintf('\n================ Propulsion Analysis Inputs ===================\n');
fprintf('Propeller           = %s\n', propIn.propName);
fprintf('KV                  = %.1f RPM/V\n', propIn.KV);
fprintf('Rm                  = %.4f ohm\n', propIn.Rm);
fprintf('I0                  = %.3f A\n', propIn.I0);
fprintf('Vbat                = %.3f V\n', propIn.Vbat);
fprintf('I_max               = %.3f A\n', propIn.I_max);
fprintf('D                   = %.2f in\n', propIn.D_in);
fprintf('Pitch               = %.2f in\n', propIn.pitch_in);
fprintf('T_static            = %.3f N\n', propIn.T_static_N);
fprintf('V0                  = %.3f m/s\n', propIn.V0_mps);
fprintf('================================================================\n\n');

propOut = propulsionAnalysis(propIn);

%% =================== CTOL Sizing =================
% Preliminary sizing inputs pulled from current main-script variables
% and propulsion-analysis outputs

sIn = struct();

% Basic constants / current weight estimate
sIn.rho_sl = 1.225;      % [kg/m^3]
sIn.g      = g;          % [m/s^2]
sIn.W0_N   = Wg;         % [N] current gross-weight estimate

% Aero assumptions
sIn.AR    = 7;           % [-] first-pass flying-wing assumption
sIn.e     = 0.80;        % [-] Oswald efficiency factor
sIn.CD0   = 0.030;       % [-] first-pass parasite drag estimate
sIn.CLmax = 1.8;         % [-] first-pass max lift coefficient

% Stall requirement
sIn.V_stall_mps = 10.0;  % [m/s]

% Climb sizing
sIn.V_climb_mps = mission.V_pattern;                 % [m/s]
sIn.G_climb     = mission.climbRate_mps / mission.V_pattern;

% Maneuver sizing
sIn.V_turn_mps    = mission.V_pattern;               % [m/s]
sIn.n_maneuver    = mission.n_turn;

% Optional ceiling sizing
sIn.use_ceiling      = false;
sIn.V_ceiling_mps    = mission.V_pattern;

% Plot / search domain
sIn.WS_min = 20;
sIn.WS_max = 400;
sIn.Npts   = 500;

% Design buffers
sIn.buf_WS = 0.05;   % 5% left of stall limit
sIn.buf_TW = 0.05;   % 5% above required T/W envelope

% Propulsion data passed into sizing for comparison
sIn.propV_vec_mps = propOut.V_vec_mps;
sIn.propT_vec_N   = propOut.T_vec_N;

fprintf('\n================ CTOL Preliminary Sizing Inputs ================\n');
fprintf('W0                 = %.4f N\n', sIn.W0_N);
fprintf('AR                 = %.3f\n', sIn.AR);
fprintf('e                  = %.3f\n', sIn.e);
fprintf('CD0                = %.4f\n', sIn.CD0);
fprintf('CLmax              = %.3f\n', sIn.CLmax);
fprintf('V_stall            = %.3f m/s\n', sIn.V_stall_mps);
fprintf('V_climb            = %.3f m/s\n', sIn.V_climb_mps);
fprintf('G_climb            = %.4f\n', sIn.G_climb);
fprintf('V_turn             = %.3f m/s\n', sIn.V_turn_mps);
fprintf('n_maneuver         = %.3f\n', sIn.n_maneuver);
fprintf('================================================================\n\n');

% Call function
sizingOut = preliminarySizingCTOL(sIn);

% Useful outputs
WS_design       = sizingOut.WS_design;
TW_design       = sizingOut.TW_design;
T_req_N         = sizingOut.T_design_N;

TW_avail_climb  = sizingOut.TW_avail_climb;
TW_avail_turn   = sizingOut.TW_avail_turn;

T_avail_climb_N = sizingOut.T_avail_climb_N;
T_avail_turn_N  = sizingOut.T_avail_turn_N;

% Wing area from selected wing loading
S_ref = Wg / WS_design;    % [m^2]

fprintf('Selected wing area S_ref      = %.4f m^2\n', S_ref);
fprintf('Selected wing loading         = %.2f N/m^2\n', WS_design);
fprintf('Required thrust loading T/W   = %.4f\n', TW_design);
fprintf('Required thrust               = %.3f N\n', T_req_N);
fprintf('Avail thrust @ climb speed    = %.3f N\n', T_avail_climb_N);
fprintf('Avail thrust @ turn speed     = %.3f N\n', T_avail_turn_N);
fprintf('Avail T/W @ climb speed       = %.4f\n', TW_avail_climb);
fprintf('Avail T/W @ turn speed        = %.4f\n', TW_avail_turn);

%% ============== Wing Geometry & Design Variables =============

wingIn = struct();

% Required from CTOL sizing
wingIn.S_ref_m2 = S_ref;

% User-selected planform variables
wingIn.AR           = 7.0;      % [-]
wingIn.taper        = 0.45;     % [-]
wingIn.sweep_c4_deg = 20.0;      % [deg] quarter-chord sweep

% Span control option
wingIn.symmetric        = true;
wingIn.useSpecifiedSpan = false;
% wingIn.b_m            = 1.80;  % only if useSpecifiedSpan = true

% Reference placement
wingIn.xLE_root_m = 0.0;
wingIn.y_root_m   = 0.0;
wingIn.z_root_m   = 0.0;

% First-pass elevon / control-surface assumptions
wingIn.eta_cs_start = 0.60;     % starts at 60% semispan
wingIn.eta_cs_end   = 0.90;     % ends at 90% semispan
wingIn.cs_chord_frac = 0.25;    % 25% of local chord

wingOut = wingGeometryDesign(wingIn);

% Useful outputs
b            = wingOut.b_m;
b_half       = wingOut.semiSpan_m;
c_root       = wingOut.c_root_m;
c_tip        = wingOut.c_tip_m;
MAC          = wingOut.MAC_m;

sweep_c4_deg = wingOut.sweep_c4_deg;
sweep_LE_deg = wingOut.sweep_LE_deg;
sweep_TE_deg = wingOut.sweep_TE_deg;

y_MAC        = wingOut.y_MAC_m;
xLE_tip      = wingOut.xLE_tip_m;
xLE_MAC      = wingOut.xLE_MAC_m;
x_c4_MAC     = wingOut.x_c4_MAC_m;

fprintf('Selected full span b         = %.4f m\n', b);
fprintf('Selected half-span b/2       = %.4f m\n', b_half);
fprintf('Root chord c_root           = %.4f m\n', c_root);
fprintf('Tip chord c_tip             = %.4f m\n', c_tip);
fprintf('Mean aerodynamic chord MAC  = %.4f m\n', MAC);
fprintf('Quarter-chord sweep         = %.3f deg\n', sweep_c4_deg);
fprintf('Leading-edge sweep          = %.3f deg\n', sweep_LE_deg);
fprintf('Trailing-edge sweep         = %.3f deg\n', sweep_TE_deg);
fprintf('MAC span station y_MAC      = %.4f m\n', y_MAC);
fprintf('MAC LE x-location xLE_MAC   = %.4f m\n', xLE_MAC);
fprintf('MAC c/4 x-location          = %.4f m\n', x_c4_MAC);
fprintf('Tip LE x-location xLE_tip   = %.4f m\n', xLE_tip);

%% ============== Airfoil Selection & Analysis(xfoil) ===============

airfoilIn = struct();

% -------- User-selected airfoils --------
airfoilIn.rootFoil = 'e387.dat';   % example: NACA airfoil
airfoilIn.tipFoil  = 'mh60.dat';   % example: coordinate file in working folder

% -------- XFOIL executable location --------
% Leave as '.' if xfoilWindows / xfoilMAC are in the current folder.
% Otherwise set this to the folder containing those executables.
airfoilIn.xfoilFolder = '.';

% -------- Flight condition --------
airfoilIn.V_ref_mps = mission.V_pattern;   % use existing mission speed
airfoilIn.rho       = 1.225;               % [kg/m^3]
airfoilIn.mu        = 1.789e-5;            % [kg/(m*s)] NEEDS TO BE CHECKED
airfoilIn.Mach      = 0.03;                % [-] NEEDS TO BE CHECKED

% -------- Reynolds numbers from current wing geometry --------
airfoilIn.Re_root = airfoilIn.rho * airfoilIn.V_ref_mps * c_root / airfoilIn.mu;
airfoilIn.Re_tip  = airfoilIn.rho * airfoilIn.V_ref_mps * c_tip  / airfoilIn.mu;

% -------- XFOIL settings --------
airfoilIn.alpha_deg = -6:0.5:12;   % analysis sweep
airfoilIn.Ncrit     = 9;           % e^n transition parameter
airfoilIn.xtr_top   = 1.0;         % forced transition location, top
airfoilIn.xtr_bot   = 1.0;         % forced transition location, bottom
airfoilIn.maxIter   = 150;         % viscous solver iterations

% -------- Optional behavior --------
airfoilIn.cleanupFiles = true;     % delete temp files after run
airfoilIn.printSummary = true;     % print key values to command window

% Run XFOIL-based section analysis for the selected root and tip airfoils
airfoilOut = airfoilAnalysisXFOIL(airfoilIn);

% Useful outputs
rootPolar = airfoilOut.root;
tipPolar  = airfoilOut.tip;

fprintf('\n================ Airfoil Analysis =================\n');
fprintf('Root airfoil              = %s\n', airfoilOut.root.name);
fprintf('Tip airfoil               = %s\n', airfoilOut.tip.name);
fprintf('Root Reynolds number      = %.3e\n', airfoilOut.root.Re);
fprintf('Tip Reynolds number       = %.3e\n', airfoilOut.tip.Re);
fprintf('Root Cla                  = %.4f per deg\n', airfoilOut.root.Cla_per_deg);
fprintf('Tip Cla                   = %.4f per deg\n', airfoilOut.tip.Cla_per_deg);
fprintf('Root alphaL0              = %.3f deg\n', airfoilOut.root.alphaL0_deg);
fprintf('Tip alphaL0               = %.3f deg\n', airfoilOut.tip.alphaL0_deg);
fprintf('Root Cm0                  = %.4f\n', airfoilOut.root.Cm0);
fprintf('Tip Cm0                   = %.4f\n', airfoilOut.tip.Cm0);
fprintf('Root Cl_max               = %.4f\n', airfoilOut.root.Cl_max);
fprintf('Tip Cl_max                = %.4f\n', airfoilOut.tip.Cl_max);
fprintf('Root best L/D             = %.4f\n', airfoilOut.root.bestLD);
fprintf('Tip best L/D              = %.4f\n', airfoilOut.tip.bestLD);
fprintf('===================================================\n\n');

% Detailed aero plots
plotAirfoilResults(airfoilOut);
%% ================= Twist Function ===============

twistIn = struct();

% Span geometry from wing planform
twistIn.b_half_m = b_half;

% User-selected twist model
twistIn.model = 'linear';      % first-pass option
twistIn.twist_root_deg = 0.0;  % root reference incidence
twistIn.twist_tip_deg  = -4.0; % negative = washout

% Span discretization for plotting / later analysis
twistIn.Nspan = 200;

% Run twist function
twistOut = twistFunctionPanknin(twistIn);

% Useful outputs
eta_twist = twistOut.eta;
y_twist_m = twistOut.y_m;
twist_deg = twistOut.twist_deg;

fprintf('\n================ Twist Function =================\n');
fprintf('Twist model               = %s\n', twistOut.model);
fprintf('Root twist                = %.3f deg\n', twistOut.twist_root_deg);
fprintf('Tip twist                 = %.3f deg\n', twistOut.twist_tip_deg);
fprintf('Semi-span                 = %.4f m\n', twistOut.b_half_m);
fprintf('Number of span stations   = %d\n', numel(twistOut.y_m));
fprintf('=================================================\n\n');

% Plot
plotTwistFunction(twistOut);

%% ================ Vertical Stabilizer / Winglet Sizing ========

vertIn = struct();

% ---------- Reference geometry ----------
vertIn.S_ref_m2 = S_ref;
vertIn.b_m      = b;

% ---------- User-selected sizing ----------
vertIn.S_v_m2 = 0.08 * S_ref;

vertIn.AR_v           = 2.0;
vertIn.taper_v        = 0.50;
vertIn.sweep_c4_v_deg = 20.0;

vertIn.cant_deg = 0.0;
vertIn.toe_deg  = 0.0;

vertIn.topFrac = 0.75;

% ---------- Mounting at wing tip ----------
vertIn.xLE_root_v_m = xLE_tip;
vertIn.y_root_v_m   = b_half;
vertIn.z_root_v_m   = 0.0;

% ---------- Airfoil ----------
vertIn.airfoilName = 'NACA0010';

% Run function
vertOut = verticalSurfaceDesign(vertIn);

% -------- Extract outputs (UPDATED) --------
b_v        = vertOut.b_v_m;
c_root_v   = vertOut.c_root_v_m;
c_tip_v    = vertOut.c_tip_v_m;
MAC_v      = vertOut.MAC_v_m;

% Top / Bottom geometry (NEW)
xLE_top_v    = vertOut.xLE_top_v_m;
y_top_v      = vertOut.y_top_v_m;
z_top_v      = vertOut.z_top_v_m;

xLE_bottom_v = vertOut.xLE_bottom_v_m;
y_bottom_v   = vertOut.y_bottom_v_m;
z_bottom_v   = vertOut.z_bottom_v_m;

z_top_m      = vertOut.z_top_m;
z_bottom_m   = vertOut.z_bottom_m;

fprintf('\n================ Vertical Surface Sizing =================\n');
fprintf('Airfoil                  = %s\n', vertOut.airfoilName);
fprintf('Vertical surface area    = %.4f m^2\n', vertOut.S_v_m2);
fprintf('Aspect ratio             = %.3f\n', vertOut.AR_v);
fprintf('Taper ratio              = %.3f\n', vertOut.taper_v);
fprintf('Quarter-chord sweep      = %.3f deg\n', vertOut.sweep_c4_v_deg);
fprintf('Cant angle               = %.3f deg\n', vertOut.cant_deg);
fprintf('Toe angle                = %.3f deg\n', vertOut.toe_deg);
fprintf('Top area fraction        = %.3f\n', vertOut.topFrac);
fprintf('Span/height              = %.4f m\n', vertOut.b_v_m);
fprintf('Root chord               = %.4f m\n', vertOut.c_root_v_m);
fprintf('Tip chord                = %.4f m\n', vertOut.c_tip_v_m);
fprintf('MAC                      = %.4f m\n', vertOut.MAC_v_m);
fprintf('Top extent above mount   = %.4f m\n', vertOut.z_top_m);
fprintf('Bottom extent below mount= %.4f m\n', vertOut.z_bottom_m);
fprintf('Top tip LE location      = (%.4f, %.4f, %.4f) m\n', ...
    vertOut.xLE_top_v_m, vertOut.y_top_v_m, vertOut.z_top_v_m);
fprintf('Bottom tip LE location   = (%.4f, %.4f, %.4f) m\n', ...
    vertOut.xLE_bottom_v_m, vertOut.y_bottom_v_m, vertOut.z_bottom_v_m);
fprintf('==========================================================\n\n');

%% ================= Spanwise Aero Estimate ===============

spanIn = struct();

% -------- Reference flight condition --------
spanIn.rho        = airfoilIn.rho;
spanIn.V_ref_mps  = airfoilIn.V_ref_mps;
spanIn.alpha_ref_deg = 4.0;     % first-pass aircraft reference AoA

% -------- Wing geometry --------
spanIn.b_half_m   = b_half;
spanIn.c_root_m   = c_root;
spanIn.c_tip_m    = c_tip;
spanIn.taper      = wingIn.taper;

% -------- Section airfoil data --------
spanIn.rootCla_per_deg = airfoilOut.root.Cla_per_deg;
spanIn.tipCla_per_deg  = airfoilOut.tip.Cla_per_deg;

spanIn.rootAlphaL0_deg = airfoilOut.root.alphaL0_deg;
spanIn.tipAlphaL0_deg  = airfoilOut.tip.alphaL0_deg;

spanIn.rootClmax = airfoilOut.root.Cl_max;
spanIn.tipClmax  = airfoilOut.tip.Cl_max;

spanIn.rootCm0 = airfoilOut.root.Cm0;
spanIn.tipCm0  = airfoilOut.tip.Cm0;

% -------- Twist data --------
spanIn.eta_twist   = twistOut.eta;
spanIn.twist_deg   = twistOut.twist_deg;

% -------- Span discretization --------
spanIn.Nspan = 200;

% -------- Run spanwise estimate --------
spanOut = spanwiseAeroEstimate(spanIn);

% -------- Useful outputs --------
eta_span      = spanOut.eta;
y_span_m      = spanOut.y_m;
c_span_m      = spanOut.c_m;
alpha_eff_deg = spanOut.alpha_eff_deg;
cl_span       = spanOut.cl_local;
Lprime_Npm    = spanOut.Lprime_N_per_m;

fprintf('\n================ Spanwise Aero Estimate =================\n');
fprintf('Reference alpha            = %.3f deg\n', spanOut.alpha_ref_deg);
fprintf('Dynamic pressure q         = %.3f Pa\n', spanOut.q_Pa);
fprintf('Estimated semispan lift    = %.3f N\n', spanOut.L_half_N);
fprintf('Estimated total wing lift  = %.3f N\n', spanOut.L_total_N);
fprintf('Root local cl              = %.4f\n', spanOut.cl_local(1));
fprintf('Tip  local cl              = %.4f\n', spanOut.cl_local(end));
fprintf('Root effective alpha       = %.3f deg\n', spanOut.alpha_eff_deg(1));
fprintf('Tip  effective alpha       = %.3f deg\n', spanOut.alpha_eff_deg(end));
fprintf('=========================================================\n\n');

plotSpanwiseAeroEstimate(spanOut);
%% =============== 3D Geometry Plot & Sanity Check =========

geom3DIn = struct();

% -------- Main wing geometry --------
geom3DIn.b_m        = b;
geom3DIn.b_half_m   = b_half;
geom3DIn.c_root_m   = c_root;
geom3DIn.c_tip_m    = c_tip;

geom3DIn.xLE_root_m = wingIn.xLE_root_m;
geom3DIn.y_root_m   = wingIn.y_root_m;
geom3DIn.z_root_m   = wingIn.z_root_m;

geom3DIn.xLE_tip_m  = xLE_tip;

geom3DIn.xLE_MAC_m  = xLE_MAC;
geom3DIn.y_MAC_m    = y_MAC;
geom3DIn.MAC_m      = MAC;

% -------- Twist --------
geom3DIn.twist_root_deg = twistOut.twist_root_deg;
geom3DIn.twist_tip_deg  = twistOut.twist_tip_deg;

% -------- Vertical surfaces --------
geom3DIn.plotVertical = true;
geom3DIn.vertOut = vertOut;

% -------- Simple body reference --------
geom3DIn.plotBody = false;
geom3DIn.bodyLength_m = 0.45;
geom3DIn.bodyWidth_m  = 0.12;
geom3DIn.bodyHeight_m = 0.10;

% -------- CG marker --------
geom3DIn.plotCG = true;
geom3DIn.xCG_m = x_c4_MAC;
geom3DIn.yCG_m = 0.0;
geom3DIn.zCG_m = 0.0;

% -------- Plot --------
plotAircraftGeometry3D(geom3DIn);

%% ============== Static Stability Analysis ===========

%% =========== V-n Diagram ===================

%% =============== Dynamic Stability Analysis (AVL) ==============

%% ============== Advanced Aerodynamics (CFD) ============

%% =========== Control Surface (AVL) ================

%% ============= Structure Sizing ==============