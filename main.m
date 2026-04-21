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
% Version 13.0: Organizes user input into blocks USER Input, and 
%               CAD Design Variables
% Version 14.0: replaces runtime XFOIL calls with prebuilt Reynolds-based
%               airfoil surrogate database generated offline from XFOIL
clc; clearvars; close all;

timestamp = datetime('now','Format','yyyy-MM-dd HH:mm:ss');
fprintf('========= Main Sizing Code executed at: %s =======\n\n', string(timestamp));

% –– top of main.m ––
repoRoot = fileparts(mfilename('fullpath'));

%%            ================ User Input ==================
% (i) Given:
g = 9.81;                  % [m/s^2]
Wprop = 2.43341;           % [N] total propulsion system weight
ft3_to_m3 = 0.02831685;    % [m^3/ft^3]
roh = 1.19;                % [kg/m^3] first-pass June density near SeaWorld

% (i) Engineering assumptions:
eta_p = 0.75;              % [-] propulsion efficiency
LD = 20;                   % [-] baseline first-pass L/D
reserve_factor = 1.15;     % [-] energy margin multiplier

% -------- Baseline empty-weight fraction --------
fe = 0.450;                 % [-] baseline empty-weight fraction = We / Wg

% -------- Payload / cargo volume definition --------
% Vp is the ACTUAL required package / bay volume in physical units.
% VPS is a NONDIMENSIONAL scaling ratio relative to a chosen reference volume.
%
% Example:
%   Vp = 0.001 m^3  --> VPS = 1
%   Vp = 0.005 m^3  --> VPS = 5
%   Vp = 0.010 m^3  --> VPS = 10

Vp_ref = 0.001;            % [m^3] reference package volume for penalty scaling
Vp     = 0.002;            % [m^3] actual payload 
VPS    = Vp / Vp_ref;      % [-] nondimensional package-volume scalar

% -------- Aerodynamic penalty model for package volume --------
% No penalty below Vp_ref (i.e. VPS <= 1)
kd = 0.02;                 % [-] L/D penalty strength per unit VPS beyond reference

% -------- Empty-weight penalty model for package volume --------
% First-pass empirical penalty on empty-weight fraction
ke =  fe / 12;               % [-] empty-weight-fraction penalty slope per unit VPS beyond reference
fe_max = 0.60;             % [-] hard upper cap for sanity

% (i) Aero:
e     = 0.80;              % [-] Oswald efficiency factor
CD0   = 0.030;             % [-] first-pass parasite drag estimate
CLmax = 1.8;               % [-] first-pass max lift coefficient

% (i) Mission Parameters:
delta_h = 120;             % [m] climb altitude change
R_cruise = 18000;          % [m] cruise range
Tf_measured = 61;          % [s] measured flight time
V_cruise = 20;             % [m/s] chosen cruise speed
V_stall_mps = 0.50 * V_cruise;   % [m/s]
Wp_g = 800;                % [g] payload weight
Wp = (Wp_g/1000)*g;        % [N] payload weight

%% =================== CAD Design Variables ==================
% (i) Wing Geometry Sliders:
AR          = 8.5;           % [-] first-pass flying-wing assumption
wingTapper  = 0.85;        % [-]
wingSweep   = 30;          % [deg] quarter-chord sweep

%% ============== Effective Aircraft Penalties ===========
% Penalties applied relative to Vp_ref
if VPS <= 1
    LD_eff = LD;
else
    LD_eff = LD / (1 + kd*(VPS - 1));
end

if VPS <= 1
    fe_eff = fe;
else
    fe_eff = fe + ke*(VPS - 1);
end

fe_eff = min(fe_eff, fe_max);

fprintf('\n================ Package Volume Scaling =================\n');
fprintf('Reference package volume Vp_ref = %.6f m^3\n', Vp_ref);
fprintf('Actual package volume    Vp     = %.6f m^3\n', Vp);
fprintf('Volume scaling ratio     VPS    = %.4f\n', VPS);
fprintf('Baseline L/D             LD     = %.4f\n', LD);
fprintf('Effective L/D            LD_eff = %.4f\n', LD_eff);
fprintf('Baseline empty wt frac   fe     = %.4f\n', fe);
fprintf('Effective empty wt frac  fe_eff = %.4f\n', fe_eff);
fprintf('=========================================================\n\n');

%% ============== Derived Weights ===========
Wg = (Wp + Wprop) / (1 - fe_eff);   % [N] gross weight
We = fe_eff * Wg;                   % [N] empty weight
Wg_grams = Wg / g * 1000;           % [g] gross weight

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
fprintf('Reference package volume   (Vp_ref)        = %.6f m^3\n', Vp_ref);
fprintf('Payload volume             (Vp)            = %.6f m^3\n', Vp);
fprintf('Volume scaling ratio       (VPS)           = %.4f\n', VPS);
fprintf('Payload Weight             (Wp)            = %.4f N\n', Wp);
fprintf('Payload Weight             (Wp)            = %.1f g\n', Wp_g);
fprintf('Measured Energy            (Ef_measured)   = %.2f J\n', Ef_measured);
fprintf('Scaled Energy              (Ef = x20)      = %.2f J\n', Ef);
fprintf('Measured Flight Time       (Tf_measured)   = %.2f s\n', Tf_measured);
fprintf('Scaled Flight Time         (Tf = x20)      = %.2f s\n', Tf);
fprintf('Propulsion System Weight   (Wprop)         = %.4f N\n', Wprop);
fprintf('Baseline Empty Wt Fraction (fe)            = %.4f\n', fe);
fprintf('Effective Empty Wt Frac.   (fe_eff)        = %.4f\n', fe_eff);
fprintf('Baseline L/D               (LD)            = %.4f\n', LD);
fprintf('Effective L/D              (LD_eff)        = %.4f\n', LD_eff);
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
mission.V_pattern         = V_cruise;  % [m/s]
mission.h_ground          = 0;         % [m]
mission.h_cruise          = 30;        % [m]

mission.runwayLength_m    = 138.35;    % [m]
mission.straightLength_m  = 140.0;     % [m]
mission.liftoffFrac       = 0.85;      % [-]
mission.touchdownFrac     = 1/3;       % [-]
mission.n_turn            = 1.6;       % [-]

% -------- Climb / descent design choices --------
mission.delta_h           = delta_h;   % [m] altitude gain

mission.V_climb_mps       = 14.0;      % [m/s] forward climb speed
mission.gamma_climb_deg   = 11.0;       % [deg]

mission.V_descent_mps     = 25.0;      % [m/s] forward descent speed
mission.gamma_descent_deg = 12.0;       % [deg]

% -------- Derived climb / descent quantities --------
mission.G_climb   = sind(mission.gamma_climb_deg);      % [-]
mission.G_descent = sind(mission.gamma_descent_deg);    % [-]

mission.climbRate_mps   = mission.V_climb_mps   * mission.G_climb;      % [m/s]
mission.descentRate_mps = mission.V_descent_mps * mission.G_descent;    % [m/s]

mission.climbDistance_m   = mission.delta_h / tand(mission.gamma_climb_deg);   % [m]
mission.descentDistance_m = mission.h_cruise / tand(mission.gamma_descent_deg); % [m]

mission.climbTime_s   = mission.delta_h / mission.climbRate_mps;    % [s]
mission.descentTime_s = mission.h_cruise / mission.descentRate_mps; % [s]

fprintf('\n================ Derived Climb / Descent Quantities ================\n');
fprintf('Climb speed V_climb        = %.3f m/s\n', mission.V_climb_mps);
fprintf('Climb angle gamma_climb    = %.3f deg\n', mission.gamma_climb_deg);
fprintf('Climb gradient G_climb     = %.4f\n', mission.G_climb);
fprintf('Required climb rate        = %.3f m/s\n', mission.climbRate_mps);
fprintf('Climb distance             = %.3f m\n', mission.climbDistance_m);
fprintf('Climb time                 = %.3f s\n', mission.climbTime_s);
fprintf('Descent speed V_descent    = %.3f m/s\n', mission.V_descent_mps);
fprintf('Descent angle gamma_desc   = %.3f deg\n', mission.gamma_descent_deg);
fprintf('Descent rate               = %.3f m/s\n', mission.descentRate_mps);
fprintf('====================================================================\n\n');

missionOut = missionProfileCigar(mission);

%% ================= Prop =================== 
propIn = struct();

% Atmosphere
propIn.rho = roh;                 % [kg/m^3]

% Motor / battery
propIn.KV    = 1100;              % [RPM/V]
propIn.Rm    = 0.073;             % [ohm]
propIn.I0    = 0.9;               % [A]
propIn.Vbat  = 11.1;              % [V]
propIn.I_max = 35;                % [A]

% Propeller
propIn.propName  = '10x4.5MR';
propIn.D_in      = 10;            % [in]
propIn.pitch_in  = 4.5;           % [in]

% Speed grid
propIn.V_vec_mps = linspace(0,25,150);

% Mode switch
propIn.usePrelimModel = false;

% APC model source
propIn.apcModelFile = 'all_prop_surrogates.mat';

propOut = propulsionAnalysis(propIn);

%% =================== CTOL Sizing =================
sIn = struct();

% Basic constants / current weight estimate
sIn.rho_sl = roh;        % [kg/m^3]
sIn.g      = g;          % [m/s^2]
sIn.W0_N   = Wg;         % [N]

% Aero assumptions
sIn.AR    = AR;          % [-]
sIn.e     = e;           % [-]
sIn.CD0   = CD0;         % [-]
sIn.CLmax = CLmax;       % [-]

% Stall requirement
sIn.V_stall_mps = V_stall_mps;  % [m/s]

% Climb sizing
sIn.V_climb_mps = mission.V_climb_mps;   % [m/s]
sIn.G_climb     = mission.G_climb;       % [-]

% Maneuver sizing
sIn.V_turn_mps  = mission.V_pattern;     % [m/s]
sIn.n_maneuver  = mission.n_turn;        % [-]

% Takeoff sizing from mission geometry
sIn.use_takeoff = true;
sIn.rho_takeoff = roh;
sIn.TOP_m       = mission.liftoffFrac * mission.runwayLength_m;

% Optional ceiling sizing
sIn.use_ceiling   = false;
sIn.V_ceiling_mps = mission.V_pattern;

% Plot / search domain
sIn.WS_min = 5;
sIn.WS_max = 150;
sIn.Npts   = 500;

% Design buffers
sIn.buf_WS = 0.10;
sIn.buf_TW = 0.10;

% Propulsion data passed into sizing
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

%% ============== Design Lift Coefficient =================
% Use the flight condition that should drive the twist requirement.
% For now, use cruise / pattern speed as the design speed.

V_design_mps = mission.V_pattern;   % [m/s] replace later if you choose a different cruise speed

q_design_Pa = 0.5 * roh * V_design_mps^2;   % [Pa] = [N/m^2]

CLdesign = Wg / (q_design_Pa * S_ref);      % [-]
% Equivalent form:
% CLdesign = 2 * Wg / (roh * V_design_mps^2 * S_ref);

fprintf('\n================ Design Lift Coefficient =================\n');
fprintf('Design speed V           = %.3f m/s\n', V_design_mps);
fprintf('Dynamic pressure q       = %.3f Pa\n', q_design_Pa);
fprintf('Design lift coefficient  = %.4f\n', CLdesign);
fprintf('==========================================================\n\n');

%% ============== Wing Geometry & Design Variables =============

wingIn = struct();

% Required from CTOL sizing
wingIn.S_ref_m2 = S_ref;

% User-selected planform variables
wingIn.AR           = AR;      % [-]
wingIn.taper        = wingTapper;     % [-]
wingIn.sweep_c4_deg = wingSweep;      % [deg] quarter-chord sweep

% Span control option
wingIn.symmetric        = true;
wingIn.useSpecifiedSpan = false;
% wingIn.b_m            = 1.80;  % only if useSpecifiedSpan = true

% Reference placement
wingIn.xLE_root_m = 0.085; % Imported from OnShape 4/20/2026
wingIn.y_root_m   = 0.145; % Imported from OnShape 4/20/2026
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

%% ============== Airfoil Selection & Analysis (Surrogate) ===============
fprintf('================ Airfoil Analysis (Surrogate) ================\n');

% -------- User-selected root and tip airfoils --------
airfoilRootName = 'e222.dat';
airfoilTipName  = 'e230.dat';

% -------- Flow properties --------
rho = roh;               % [kg/m^3]
mu  = 1.789e-5;          % [kg/(m*s)]
Vref_mps = mission.V_pattern;   % [m/s]

% -------- Reynolds numbers from current geometry --------
Re_root = rho * Vref_mps * c_root / mu;   % [-]
Re_tip  = rho * Vref_mps * c_tip  / mu;   % [-]

fprintf('Reference speed           = %.3f m/s\n', Vref_mps);
fprintf('Root chord                = %.5f m\n', c_root);
fprintf('Tip chord                 = %.5f m\n', c_tip);
fprintf('Root Reynolds number      = %.3e\n', Re_root);
fprintf('Tip Reynolds number       = %.3e\n', Re_tip);

% -------- Load surrogate database --------
airfoilDB_cached = loadAirfoilSurrogateDB(repoRoot);

% -------- Evaluate root and tip airfoils --------
rootAirfoil = evaluateAirfoilSurrogate(airfoilDB_cached, airfoilRootName, Re_root);
tipAirfoil  = evaluateAirfoilSurrogate(airfoilDB_cached, airfoilTipName,  Re_tip);
% -------- Package to match old workflow --------
airfoilOut = struct();
airfoilOut.root = rootAirfoil;
airfoilOut.tip  = tipAirfoil;
airfoilOut.xfoilExe = 'SURROGATE_DB';
airfoilOut.aeroTwist_deg = airfoilOut.root.alphaL0_deg - airfoilOut.tip.alphaL0_deg;

fprintf('Root airfoil              = %s\n', airfoilOut.root.name);
fprintf('Tip airfoil               = %s\n', airfoilOut.tip.name);

fprintf('\n---- Root airfoil metrics ----\n');
fprintf('Cla                       = %.5f per deg\n', airfoilOut.root.Cla_per_deg);
fprintf('alphaL0                   = %.5f deg\n', airfoilOut.root.alphaL0_deg);
fprintf('Cm0                       = %.5f\n', airfoilOut.root.Cm0);
fprintf('Cl_max                    = %.5f\n', airfoilOut.root.Cl_max);
fprintf('Best L/D                  = %.5f\n', airfoilOut.root.bestLD);

fprintf('\n---- Tip airfoil metrics ----\n');
fprintf('Cla                       = %.5f per deg\n', airfoilOut.tip.Cla_per_deg);
fprintf('alphaL0                   = %.5f deg\n', airfoilOut.tip.alphaL0_deg);
fprintf('Cm0                       = %.5f\n', airfoilOut.tip.Cm0);
fprintf('Cl_max                    = %.5f\n', airfoilOut.tip.Cl_max);
fprintf('Best L/D                  = %.5f\n', airfoilOut.tip.bestLD);

fprintf('\nAerodynamic twist         = %.5f deg\n', airfoilOut.aeroTwist_deg);
fprintf('===============================================================\n\n');
%% ================= Twist Function (Panknin) ===============

twistIn = struct();

% Geometry inputs from wing planform
twistIn.b_m            = b;              % full span [m]
twistIn.AR             = AR;             % aspect ratio [-]
twistIn.c_root_m       = c_root;         % [m]
twistIn.c_tip_m        = c_tip;          % [m]
twistIn.sweep_c4_deg   = sweep_c4_deg;   % quarter-chord sweep [deg]

% Airfoil inputs from XFOIL outputs
twistIn.alphaL0_root_deg = airfoilOut.root.alphaL0_deg;
twistIn.alphaL0_tip_deg  = airfoilOut.tip.alphaL0_deg;
twistIn.Cm_root          = airfoilOut.root.Cm0;
twistIn.Cm_tip           = airfoilOut.tip.Cm0;

% Design condition inputs
twistIn.CL_design      = CLdesign;
twistIn.static_margin  = 0.10;

% Distribution settings
twistIn.model          = 'linear';
twistIn.twist_root_deg = 0.0;
twistIn.Nspan          = 200;

% Run twist function
twistOut = twistFunctionPanknin(twistIn);

% Useful outputs
eta_twist = twistOut.eta;
y_twist_m = twistOut.y_m;
twist_deg = twistOut.twist_deg;

fprintf('\n================ Twist Function (Panknin) =================\n');
fprintf('Twist model               = %s\n', twistOut.model);
fprintf('Root geometric twist      = %.3f deg\n', twistOut.twist_root_deg);
fprintf('Tip geometric twist       = %.3f deg\n', twistOut.twist_tip_deg);
fprintf('Total twist required      = %.3f deg\n', twistOut.alphaTotal_deg);
fprintf('Aerodynamic twist term    = %.3f deg\n', twistOut.aeroTwist_deg);
fprintf('Geometric twist required  = %.3f deg\n', twistOut.alphaGeo_deg);
fprintf('Panknin lambda = AR       = %.4f\n', twistOut.lambda_panknin);
fprintf('Taper ratio               = %.4f\n', twistOut.taperRatio);
fprintf('K1                        = %.4f\n', twistOut.K1);
fprintf('K2                        = %.4f\n', twistOut.K2);
fprintf('Numerator                 = %.6f\n', twistOut.numerator);
fprintf('Denominator               = %.6f\n', twistOut.denominator);
fprintf('Semi-span                 = %.4f m\n', twistOut.b_half_m);
fprintf('Number of span stations   = %d\n', numel(twistOut.y_m));
fprintf('===========================================================\n\n');

% Plot
plotTwistFunction(twistOut);

%% ================ Vertical Stabilizer / Winglet Sizing ========

vertIn = struct();

% ---------- Reference geometry ----------
vertIn.S_ref_m2 = S_ref;
vertIn.b_w_m    = b;

% ---------- Twin-fin flag ----------
% true  = total area is split equally into two fins
% false = single center fin
vertIn.isTwin = true;

% ---------- Sizing mode ----------
% 'manualArea' or 'tailVolumeCoeff'
vertIn.sizeMode = 'tailVolumeCoeff';

% ---------- Tail-volume method inputs ----------
% c_v here is interpreted using TOTAL vertical area:
%     c_v = (L_v * S_v_total) / (b_w * S_w)
%
% Therefore, leave c_v as the desired TOTAL-system coefficient.
vertIn.c_v = 0.04;

% Wing reference quarter-chord x-location
vertIn.x_c4_wing_ref_m = x_c4_MAC;

% ---------- If using manual area mode instead ----------
% IMPORTANT: if manualArea is used, S_v_m2 below is TOTAL area
% vertIn.S_v_m2 = 0.08 * S_ref;

% ---------- User-selected shape ----------
vertIn.AR_v           = 2.0;
vertIn.taper_v        = 0.60;
vertIn.sweep_c4_v_deg = 30.0;

vertIn.cant_deg = 0.0;
vertIn.toe_deg  = 0.0;

vertIn.topFrac = 0.66;

% ---------- Mounting at wing tip ----------
vertIn.xLE_root_v_m = xLE_tip;
vertIn.y_root_v_m   = b_half;
vertIn.z_root_v_m   = 0.0;

% ---------- Airfoil ----------
vertIn.airfoilName = 'NACA0010';

% ---------- Rudder sizing ----------
vertIn.rudder.enable      = true;
vertIn.rudder.useTopOnly  = true;   % best match for winglet-like fin
vertIn.rudder.eta_start   = 0.15;   % start at 15% of top exposed height
vertIn.rudder.eta_end     = 0.95;   % end near tip
vertIn.rudder.cf_root     = 0.30;   % rudder chord = 30% of local chord
vertIn.rudder.cf_tip      = 0.30;

% Run function
vertOut = verticalSurfaceDesign(vertIn);

% -------- Extract outputs --------
b_v        = vertOut.b_v_m;
c_root_v   = vertOut.c_root_v_m;
c_tip_v    = vertOut.c_tip_v_m;
MAC_v      = vertOut.MAC_v_m;

fprintf('\n================ Vertical Surface Sizing =================\n');
fprintf('Sizing mode                    = %s\n', vertOut.sizeMode);
fprintf('Twin-fin configuration        = %d\n', vertOut.isTwin);
fprintf('Airfoil                       = %s\n', vertOut.airfoilName);
fprintf('Single-fin area               = %.4f m^2\n', vertOut.S_v_m2);
fprintf('Total vertical area           = %.4f m^2\n', vertOut.S_v_total_m2);
fprintf('Aspect ratio                  = %.3f\n', vertOut.AR_v);
fprintf('Taper ratio                   = %.3f\n', vertOut.taper_v);
fprintf('Quarter-chord sweep           = %.3f deg\n', vertOut.sweep_c4_v_deg);
fprintf('Cant angle                    = %.3f deg\n', vertOut.cant_deg);
fprintf('Toe angle                     = %.3f deg\n', vertOut.toe_deg);
fprintf('Top area fraction             = %.3f\n', vertOut.topFrac);
fprintf('Single-fin span/height        = %.4f m\n', vertOut.b_v_m);
fprintf('Single-fin root chord         = %.4f m\n', vertOut.c_root_v_m);
fprintf('Single-fin tip chord          = %.4f m\n', vertOut.c_tip_v_m);
fprintf('Single-fin MAC                = %.4f m\n', vertOut.MAC_v_m);

if strcmpi(vertOut.sizeMode,'tailVolumeCoeff')
    fprintf('Vertical tail coeff c_v       = %.4f\n', vertOut.c_v);
    fprintf('Moment arm L_v                = %.4f m\n', vertOut.L_v_m);
    fprintf('Wing c/4 ref x                = %.4f m\n', vertOut.x_c4_wing_ref_m);
    fprintf('Vert c/4 ref x (single fin)   = %.4f m\n', vertOut.x_c4_vert_ref_m);
end

fprintf('Top extent above mount        = %.4f m\n', vertOut.z_top_m);
fprintf('Bottom extent below mount     = %.4f m\n', vertOut.z_bottom_m);
fprintf('Top tip LE location           = (%.4f, %.4f, %.4f) m\n', ...
    vertOut.xLE_top_v_m, vertOut.y_top_v_m, vertOut.z_top_v_m);
fprintf('Bottom tip LE location        = (%.4f, %.4f, %.4f) m\n', ...
    vertOut.xLE_bottom_v_m, vertOut.y_bottom_v_m, vertOut.z_bottom_v_m);

if vertOut.rudder.enable
    fprintf('---------------- Rudder (single fin) ----------------\n');
    fprintf('Rudder area                   = %.4f m^2\n', vertOut.rudder.S_rudder_m2);
    fprintf('Rudder / single-fin area      = %.4f\n', vertOut.rudder.S_over_Sv);
    fprintf('Rudder height                 = %.4f m\n', vertOut.rudder.height_m);
    fprintf('Rudder root chord             = %.4f m\n', vertOut.rudder.c_root_m);
    fprintf('Rudder tip chord              = %.4f m\n', vertOut.rudder.c_tip_m);
    fprintf('Rudder eta start              = %.3f\n', vertOut.rudder.eta_start);
    fprintf('Rudder eta end                = %.3f\n', vertOut.rudder.eta_end);
end

fprintf('==========================================================\n\n');

%% ================= Spanwise Aero Estimate ===============

spanIn = struct();

% -------- Reference flight condition --------
spanIn.rho        = rho;
spanIn.V_ref_mps  = Vref_mps;
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
%Vp ratios:
% Vp_Width = b * .3;
% Vp_Hight = b * .1;
% Vp_fwdArea = Vp_Width * Vp_Hight;
% Vp_Length = Vp_fwdArea / Vp;


geom3DIn = struct();

geom3DIn.b_m        = wingOut.b_m;
geom3DIn.b_half_m   = wingOut.b_m / 2;

geom3DIn.c_root_m   = wingOut.c_root_m;
geom3DIn.c_tip_m    = wingOut.c_tip_m;

geom3DIn.xLE_root_m = wingIn.xLE_root_m;
geom3DIn.y_root_m   = wingIn.y_root_m;
geom3DIn.z_root_m   = wingIn.z_root_m;

geom3DIn.xLE_tip_m  = wingOut.xLE_tip_m;
geom3DIn.yLE_tip_m  = wingIn.y_root_m + wingOut.semiSpan_m;
geom3DIn.zLE_tip_m  = wingIn.z_root_m;

geom3DIn.xLE_MAC_m  = wingOut.xLE_MAC_m;
geom3DIn.y_MAC_m    = wingOut.y_MAC_m;
geom3DIn.z_MAC_m    = wingIn.z_root_m;
geom3DIn.MAC_m      = wingOut.MAC_m;

geom3DIn.twist_root_deg = twistOut.twist_root_deg;
geom3DIn.twist_tip_deg  = twistOut.twist_tip_deg;

geom3DIn.plotVertical = true;
geom3DIn.plotBody     = true;
geom3DIn.plotCG       = true;
geom3DIn.vertOut      = vertOut;

% -------- Simple Volume Package reference --------
geom3DIn.plotBody = true;

% Simple cube from required package volume only
geom3DIn.bodyLength_m = Vp^(1/3);
geom3DIn.bodyWidth_m  = Vp^(1/3);
geom3DIn.bodyHeight_m = Vp^(1/3);

% Temporary: place simple package box at current assumed CG marker
geom3DIn.xCG_m = x_c4_MAC;
geom3DIn.yCG_m = 0.0;
geom3DIn.zCG_m = 0.0;

% -------- CG marker --------
geom3DIn.plotCG = true;
geom3DIn.xCG_m = x_c4_MAC;
geom3DIn.yCG_m = 0.0;
geom3DIn.zCG_m = 0.0;
% -------- Plot --------
plotAircraftGeometry3D(geom3DIn);

geom3DIn.x_cg = x_c4_MAC;   % from your mass / geometry calc
geom3DIn.y_cg = 0;
geom3DIn.z_cg = 0;

%% ================ Drag build up ==================

%% =============== CL, CD, plots ==================

% %% ============== Static Stability Analysis ===========
% 
% % Purpose:
% %   Build total aircraft mass properties using CAD-based empty-airframe
% %   values plus discrete components, then compute CG location, CG as %MAC,
% %   and static margin using a supplied or approximate neutral point.
% %
% % Notes:
% %   - SI units only
% %   - x positive aft
% %   - y positive right
% %   - z positive up
% %   - Neutral point should ultimately come from AVL / aero model
% %
% % Required geometry inputs from wingGeometryDesign (or equivalent):
% %   wingOut.S_ref_m2
% %   wingOut.b_m
% %   wingOut.cMAC_m
% %   wingOut.xLEMAC_m
% %
% % Optional:
% %   wingOut.xAC_wing_m   % if you already compute this
% %
% % Assumption for temporary use only:
% %   If no neutral point is available, use wing AC ~= xLEMAC + 0.25*cMAC
% %   This is only a placeholder for early iteration.
% 
% % ---------- CAD-derived empty airframe ----------
% cadMass.emptyAirframe.mass_kg = 0.99727226;
% cadMass.emptyAirframe.cg_m    = [0.26519, -0.00009, 0.01560];
% 
% % Inertia tensor from CAD about empty-airframe CG [kg*m^2]
% cadMass.emptyAirframe.Icg_kgm2 = 1e-4 * [ ...
%     1307.911,   0.153,  -0.748;
%        0.153, 272.945,   0.113;
%       -0.748,   0.113, 1565.041 ];
% 
% % ---------- Optional: fuselage-only CAD data ----------
% cadMass.fuselage.mass_kg = 0.605;
% cadMass.fuselage.cg_m    = [0.19237805, -0.00013, 0.02017];
% 
% cadMass.fuselage.Icg_kgm2 = 1e-4 * [ ...
%      24.292,   0.177,  -0.679;
%       0.177, 112.664,  -0.024;
%      -0.679, -0.024, 129.372 ];
% 
% % ---------- Component mass build-up ----------
% % Replace these with your current values / variables as you refine.
% comp = [];
% 
% % Example helper pattern:
% % comp(end+1) = makePointMass('battery',   m_batt_kg, [x_batt_m, y_batt_m, z_batt_m]);
% % comp(end+1) = makePointMass('motor',     m_motor_kg, [x_motor_m, y_motor_m, z_motor_m]);
% % comp(end+1) = makePointMass('esc',       m_esc_kg,   [x_esc_m,   y_esc_m,   z_esc_m]);
% % comp(end+1) = makePointMass('rx',        m_rx_kg,    [x_rx_m,    y_rx_m,    z_rx_m]);
% % comp(end+1) = makePointMass('payload',   m_pay_kg,   [x_pay_m,   y_pay_m,   z_pay_m]);
% % comp(end+1) = makePointMass('ballast',   m_bal_kg,   [x_bal_m,   y_bal_m,   z_bal_m]);
% 
% % ---------- Static stability inputs ----------
% stabIn = struct();
% 
% % Geometry references
% stabIn.cMAC_m   = wingOut.cMAC_m;
% stabIn.xLEMAC_m = wingOut.xLEMAC_m;
% stabIn.Sref_m2  = wingOut.S_ref_m2;
% stabIn.b_m      = wingOut.b_m;
% 
% % Empty airframe from CAD
% stabIn.emptyMass = cadMass.emptyAirframe;
% 
% % Additional discrete components
% stabIn.components = comp;
% 
% % ---- Neutral point choice ----
% % Preferred: from AVL / aero function
% % stabIn.xNP_m = aeroOut.xNP_m;
% 
% % Temporary fallback if AVL not ready:
% stabIn.useApproxNP = true;
% stabIn.xACwingApprox_m = wingOut.xLEMAC_m + 0.25 * wingOut.cMAC_m;
% 
% % Optional target range
% stabIn.SM_target_min = 0.05;
% stabIn.SM_target_max = 0.20;
% 
% stabOut = staticStabilityAnalysis(stabIn);
% 
% fprintf('\n===== Static Stability Summary =====\n');
% fprintf('Total mass                : %.4f kg\n', stabOut.mass_kg);
% fprintf('CG location               : [%.4f, %.4f, %.4f] m\n', stabOut.cg_m(1), stabOut.cg_m(2), stabOut.cg_m(3));
% fprintf('CG as %%MAC                : %.2f %%\n', 100*stabOut.xcg_over_MAC);
% fprintf('Neutral point as %%MAC     : %.2f %%\n', 100*stabOut.xnp_over_MAC);
% fprintf('Static margin             : %.2f %%\n', 100*stabOut.SM);
% fprintf('Longitudinal static stable: %s\n', string(stabOut.isStaticallyStable));
% fprintf('In target SM band         : %s\n', string(stabOut.inTargetBand));
% 
% if stabOut.usedApproxNP
%     fprintf('WARNING: Neutral point currently uses approximate wing AC only.\n');
%     fprintf('         Replace with AVL / aero-based xNP before trusting final results.\n');
% end
%% =========== V-n Diagram ===================

%% =============== Dynamic Stability Analysis (AVL) ==============

%% ============== Advanced Aerodynamics (CFD) ============

%% =========== Control Surface (AVL) ================

%% ============= Structure Sizing ==============
%% ============= STRUCTURE SIZING (FINAL) ==============

fprintf('\n================ STRUCTURE SIZING =================\n');

%Inputs
sigma_allow = 200e6;   % [Pa] allowable stress (carbon fiber)
SF = 2.0;              % Safety factor
E = 70e9;              % [Pa] Young's modulus (carbon fiber)
rho_cf = 1600;         % [kg/m^3] density

b = wingOut.b_m;       % [m] full span
W = Wg;                % [N] total weight
g = 9.81;

%Wing Root Bending Moment
M_max = W * b / 8;     % [Nm]

fprintf('Max bending moment at root = %.3f Nm\n', M_max);

%Required Spar Diameter 
d_req = ((32 * M_max * SF) / (pi * sigma_allow))^(1/3);

fprintf('Required spar diameter = %.4f m (%.2f mm)\n', d_req, d_req*1000);

%Select Practical Spar Diameter 
d_selected = 0.010;   % [m] (10 mm carbon spar)

I = (pi/64) * d_selected^4;
y = d_selected / 2;

sigma_actual = M_max * y / I;

fprintf('Selected spar diameter = %.2f mm\n', d_selected*1000);
fprintf('Actual bending stress = %.2f MPa\n', sigma_actual/1e6);

%Factor of Safety 
FoS = sigma_allow / sigma_actual;

fprintf('Factor of Safety = %.2f\n', FoS);

if FoS > SF
    fprintf('✅ BENDING SAFE\n');
else
    fprintf('❌ BENDING NOT SAFE\n');
end

%Wing Deflection 
delta_max = (W * b^3) / (48 * E * I);

fprintf('Max wing deflection = %.4f m\n', delta_max);

if delta_max < 0.05 * b
    fprintf('✅ DEFLECTION OK (<5%% span)\n');
else
    fprintf('❌ DEFLECTION TOO HIGH\n');
end

%Shear Stress 
V_max = W / 2;   % [N] shear at root

A_shear = pi*(d_selected/2)^2;

tau = V_max / A_shear;

fprintf('Shear stress = %.2f MPa\n', tau/1e6);

%Landing Impact Load 
h_drop = 0.3;   % [m] assumed drop height

V_impact = sqrt(2*g*h_drop);

F_impact = (W/g) * V_impact / 0.1; % deceleration time ~0.1 s

fprintf('Landing impact force = %.2f N\n', F_impact);

%Spar Weight
spar_volume = pi*(d_selected/2)^2 * b;
spar_mass = spar_volume * rho_cf;

fprintf('Estimated spar mass = %.3f kg\n', spar_mass);

% FINAL STATUS 
if FoS > SF && delta_max < 0.05*b
    fprintf('\n✅ FINAL STRUCTURE DESIGN SAFE\n');
else
    fprintf('\n❌ STRUCTURE NEEDS IMPROVEMENT\n');
end

fprintf('=====================================================\n\n');
