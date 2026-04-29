% 155B Group 2 Main Sizing Script

% Intention:
% Shared main sizing script for the group that can evolve with the
% project as the design matures. Keep the main script organized and modular
% by calling external functions whenever practical.
% Allocate a section to each topic. When getting code from chat try to only
% paste sections to avoid mistakes

% Version Evolution:

% Version 1.0:   calls J(x) function

% Version 1.1:   uses energyCalc(...) as a separate function
% Version 1.2:   couples VPS to payload volume, effective L/D, and effective
%               empty-weight fraction

% Version 2.0:   includes mission profile via missionProfileCigar(mission)

% Version 3.0:   introduces aero assumptions (CL, L/D, etc.)

% Version 4.0:   includes propulsion sizing via propulsionAnalysis(...)

% Version 5.0:   includes CTOL sizing equations via preliminarySizingCTOL(...)

% Version 6.0:   includes wing geometry design variables via wingGeometryDesign(...)

% Version 7.0:   integrates XFOIL airfoil analysis via airfoilAnalysisXFOIL(...)

% Version 8.0:   extracts detailed airfoil parameters (Clalpha, Cm0, alphaL0,
%               Clmax, and best L/D)

% Version 9.0:   introduces spanwise twist modeling via twistFunctionPanknin(...)

% Version 10.0:  adds vertical stabilizer / winglet sizing via
%               verticalSurfaceDesign(...)
% Version 11.0:  implements spanwise aerodynamic estimation via
%               spanwiseAeroEstimate(...)
% Version 12.0:  introduces 3D aircraft geometry visualization via
%               plotAircraftGeometry3D(...)
% Version 13.0:  Organizes user input into blocks USER Input, and 
%               CAD Design Variables
% Version 14.0:  replaces runtime XFOIL calls with prebuilt Reynolds-based
%               airfoil surrogate database generated offline from XFOIL
% Version 15.0:  Includes Mass properties, parametric Static Stability that
%               updates with change in wing geo and new 3d plot w/ CG
% Version 16.0:  Adds Drag Build-up, and proper L/D ratio, and respective
%               Plots
% Version 17.0:  Recalculates Engineering design choices like Vstall given
%               aircraft calcs to showcase how feasible design is.
% Version 18.0:  Runs AVL for dynamic stability analysis.

% Version 19.0: Runs AVL Control Surface Sizing Optimization

% Version 20.0: Full On Profit Optimization

clc; clear; close all;

timestamp = datetime('now','Format','yyyy-MM-dd HH:mm:ss');
fprintf('========= Main Sizing Code executed at: %s =======\n\n', string(timestamp));

% –– top of main.m ––
repoRoot = fileparts(mfilename('fullpath'));

%% =================== Run Flags =========================
% Figures
showPlots       = true;  % true = show all figures throughout the script

% AVL geometry viewer (opens interactive Terminal window — requires manual close)
viewGeometry    = false;   % true = open AVL 3D viewer before stability run
modelCenterbody = true;   % true = include fuselage as AVL lifting surface (MH95)
                       %        (flat-plate model overestimates lift — keep false)

% Long-running analyses (keep false for normal design runs)
useDragBuildUp  = true;   % true = compute CD0 from geometry build-up
runCSopt        = false;  % true = CMA-ES elevon optimizer          
runSweep        = false;  % true = dynamic stability parameter sweep 
runOptimization = false;  % true = CMA-ES dynamic stability optimizer (~30 min)
runMonteCarlo   = false;   % true = Monte Carlo profit sensitivity analysis (~30 s)
runProfitOpt    = false;   % true = CMA-ES full aircraft profit optimizer (~6-10 hr)
%% ==================================================================

if ~showPlots; set(0,'DefaultFigureVisible','off'); else; set(0,'DefaultFigureVisible','on'); end

%%            ================ User Input ==================
% (i) Given:
g = 9.81;                  % [m/s^2]
Wprop = 2.43341;           % [N] total propulsion system weight
ft3_to_m3 = 0.02831685;    % [m^3/ft^3]
roh = 1.19;                % [kg/m^3] first-pass June density near SeaWorld

% (i) Engineering assumptions:
eta_p = 0.75;              % [-] propulsion efficiency
LD = 11.15666;                   % [-] From AC polar
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
Vp     = 0.01500000000000;            % [m^3] CMA-ES optimal (was 0.0039)
VPS    = Vp / Vp_ref;      % [-] nondimensional package-volume scalar

% -------- Empty-weight penalty model for package volume --------
ke =  fe / 12;               % [-] empty-weight-fraction penalty slope per unit VPS beyond reference
fe_max = 0.60;             % [-] hard upper cap for sanity

% (i) Aero:
e     = 0.80;              % [-] Oswald efficiency factor
CD0   = 0.01224;             % [-] first-pass parasite drag estimate
CLmax = 0.92863;               % [-] first-pass max lift coefficient

% (i) Mission Parameters:
delta_h = 120;             % [m] climb altitude change
R_cruise = 18000;          % [m] cruise range
Tf_measured = 61;          % [s] measured flight time
V_cruise = 20.0;           % [m/s] CMA-ES optimal (was 24)
V_stall_mps = 12;   % [m/s] chosen Stall speed
Wp_g = 800;               % [g] payload weight
Wp = (Wp_g/1000)*g;        % [N] payload weight

%% =================== CAD Design Variables ==================
% (i) Wing Geometry Sliders:
AR          = 8;         % [-] profit optimizer
wingTapper  = 0.661;         % [-] profit optimizer
wingSweep   = 28.3;          % [deg] profit optimizer

% (i) Fuselage / Centerbody Airfoil:
fuselageAirfoil = 'mh95';    % ['mh95' | 'n0012' | other airfoil name in data/airfoils/]
                              % Used for: AVL centerbody model, cargo bay geometry

%% ============== Drag Build-Up User Inputs ==============
% These are user-entered first-pass values and should be updated from CAD.

% ---- fallback parasite drag if not using build-up ----
CD0_user = CD0;            % [-]

% ---- body dimensions for centerbody / fuselage drag model ----
Lf = 0.9500;                   % [m] body length — capped at 0.95 m (optimizer wanted 0.9926)
Wf = 0.1491;                   % [m] max body width — profit optimizer (2 × cb_halfwidth = 2 × 0.0746)

% ---- wetted areas (scaled from geometry; wing/fin overwritten after wingOut/vertOut) ----
Swet_wing = 0.64897702;        % [m^2] placeholder — overwritten after wingGeometryDesign
Swet_fuse = 0.21672003 * (Lf * Wf) / (0.620 * 0.290);  % [m^2] scales with fuselage box area
Swet_fin  = 0.14780168;        % [m^2] placeholder — overwritten after verticalSurfaceDesign
Hf = 0.10145110;               % [m] max body height

% ---- wing / fin form-factor settings ----
tc = 0.12;                 % [-] representative thickness-to-chord ratio
xc = 0.30;                 % [-] x/c location of max thickness

% ---- interference factors ----
Q_wing = 1.10;             % [-]
Q_fuse = 1.10;             % [-]
Q_fin  = 1.10;             % [-]

% ---- plot settings ----
alphaPolar_deg = -12:0.25:16;   % [deg]

%% ============== Effective Aircraft Penalties ===========
LD_eff = LD;
fe_eff = min(fe + ke * max(0, VPS - 1), fe_max);

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

%% ============== Cargo Bay Geometry (Fuselage Airfoil) ===========
fprintf('================ Cargo Bay Geometry ======================\n');

% Build airfoil filename from selection
airfoilFileMap = struct();
airfoilFileMap.mh95 = 'data/airfoils/mh95.dat';
airfoilFileMap.n0012 = 'data/airfoils/n0012.dat';
airfoilFileMap.mh64 = 'data/airfoils/mh64.dat';
airfoilFileMap.mh62 = 'data/airfoils/mh62.dat';
airfoilFileMap.mh61 = 'data/airfoils/mh61.dat';
airfoilFileMap.mh60 = 'data/airfoils/mh60.dat';
airfoilFileMap.eh3012 = 'data/airfoils/eh3012.dat';
airfoilFileMap.eh2012 = 'data/airfoils/eh2012.dat';
airfoilFileMap.e387 = 'data/airfoils/e387.dat';

if isfield(airfoilFileMap, fuselageAirfoil)
    cargoAirfoilFile = airfoilFileMap.(fuselageAirfoil);
else
    % Default fallback
    cargoAirfoilFile = 'data/airfoils/mh95.dat';
    warning('Unknown fuselage airfoil "%s", using mh95', fuselageAirfoil);
end

cargoIn = struct();
cargoIn.L_fuse_m = Lf;              % [m] fuselage length from design vars
cargoIn.airfoilFile = cargoAirfoilFile;
cargoIn.showPlot = showPlots;       % show the cargo bay cross-section plot

cargoOut = cargoBayGeometry(cargoIn);

fprintf('Fuselage Length         (Lf)            = %.4f m\n', Lf);
fprintf('Airfoil Type            = %s\n', upper(fuselageAirfoil));
fprintf('Max Rectangle Width     (cargo width)   = %.4f m\n', cargoOut.width_m);
fprintf('Max Rectangle Height    (cargo height)  = %.4f m\n', cargoOut.height_m);
fprintf('Cargo Cross-Section Area                 = %.6f m^2\n', cargoOut.area_m2);
fprintf('Cargo Bay Volume                          = %.6f m^3\n', cargoOut.volume_m3);
fprintf('----------------------------------------------------------\n\n');

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
fprintf('Empty-weight penalty       (ke)            = %.4f\n', ke);
fprintf('Derived Empty Weight       (We)            = %.4f N\n', We);
fprintf('Derived Empty Weight       (We)            = %.4f kg\n', We/g);
fprintf('Derived Gross Weight       (Wg)            = %.4f N\n', Wg);
fprintf('Derived Gross Weight       (Wg)            = %.4f kg\n', Wg/g);
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
mission.liftoffFrac       = 0.89;      % [-]
mission.touchdownFrac     = 1/3;       % [-]
mission.n_turn            = 1.5;       % [-] working @ 1.2

% -------- Climb / descent design choices --------
mission.delta_h           = delta_h;   % [m] altitude gain

mission.V_climb_mps       = 11.0;      % [m/s] forward climb speed
mission.gamma_climb_deg   = 6.0;       % [deg] UPDATED (was too aggressive)

mission.V_descent_mps     = 30.0;      % [m/s]
mission.gamma_descent_deg = 12.0;      % [deg]

% -------- Derived climb / descent quantities --------
mission.G_climb   = sind(mission.gamma_climb_deg);      % [-]
mission.G_descent = sind(mission.gamma_descent_deg);    % [-]

mission.climbRate_mps   = mission.V_climb_mps   * mission.G_climb;      % [m/s]
mission.descentRate_mps = mission.V_descent_mps * mission.G_descent;    % [m/s]

mission.climbDistance_m   = mission.delta_h / tand(mission.gamma_climb_deg);   % [m]
mission.descentDistance_m = mission.h_cruise / tand(mission.gamma_descent_deg); % [m]

mission.climbTime_s   = mission.delta_h / mission.climbRate_mps;    % [s]
mission.descentTime_s = mission.h_cruise / mission.descentRate_mps; % [s] FIXED
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
propIn.V_vec_mps = linspace(0,40,250);   % expand search range

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
sIn.W0_N   = 2.251 * g;  % [N] mass-model result (fe-fraction overestimates at Vp=15L)

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
sIn.V_turn_mps  = 11;     % [m/s]
sIn.n_maneuver  = mission.n_turn;        % [-]

% Takeoff sizing from mission geometry
sIn.use_takeoff = true;
sIn.rho_takeoff = roh;
sIn.TOP_m       = mission.runwayLength_m;

% Optional ceiling sizing
sIn.use_ceiling   = false;
sIn.V_ceiling_mps = mission.V_pattern;

% Plot / search domain
sIn.WS_min = 1;
sIn.WS_max = 90;
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
S_ref = 2.251 * g / WS_design;    % [m^2] — uses mass-model weight (fe-fraction overestimates)

fprintf('Selected wing area S_ref      = %.4f m^2\n', S_ref);
fprintf('Selected wing loading         = %.2f N/m^2\n', WS_design);
fprintf('Required thrust loading T/W   = %.4f\n', TW_design);
fprintf('Required thrust               = %.3f N\n', T_req_N);
fprintf('Avail thrust @ climb speed    = %.3f N\n', T_avail_climb_N);
fprintf('Avail thrust @ turn speed     = %.3f N\n', T_avail_turn_N);
fprintf('Avail T/W @ climb speed       = %.4f\n', TW_avail_climb);
fprintf('Avail T/W @ turn speed        = %.4f\n', TW_avail_turn);

%% =============== Monte Carlo Profit Sensitivity ==============
% Runs early — only needs mission params and first-pass sizing estimates.
% x0 baseline uses early-script values (Wg, LD_eff, S_ref, CLmax).
% Re-run with runMonteCarlo=true after updating any mission parameter.
if runMonteCarlo
    mcIn = struct();

    % ---- actual mission parameters from this run ----
    mcIn.R_cruise_m     = R_cruise;
    mcIn.eta_p          = eta_p;
    mcIn.reserve_factor = reserve_factor;
    mcIn.rho            = roh;
    mcIn.g              = g;
    mcIn.Vs_max_mps     = V_stall_mps;
    mcIn.stall_margin   = 1.30;
    mcIn.SM_min_pct     = 5.0;
    mcIn.SM_max_pct     = 13.0;
    mcIn.N              = 200000;
    mcIn.showPlots      = showPlots;

    % ---- exploration bounds ----
    mcIn.bounds.W_empty_N = [8,  30];
    mcIn.bounds.Wp_N      = [2,  15];
    mcIn.bounds.Vp_m3     = [0.001, 0.010];
    mcIn.bounds.V_mps     = [18,  28];
    mcIn.bounds.LD        = [5.5, 15];
    mcIn.bounds.Sref_m2   = [0.15, 0.70];
    mcIn.bounds.CLmax     = [0.75, 1.20];
    mcIn.bounds.SM_pct    = [0,   20];

    % ---- local sensitivity baseline: early-script estimates ----
    % W_empty uses parametric fe estimate; LD and CLmax are first-pass values.
    % These update automatically when you change mission params at the top.
    mcIn.x0.W_empty_N = We;           % [N] parametric empty weight
    mcIn.x0.Wp_N      = Wp;           % [N] payload
    mcIn.x0.Vp_m3     = Vp;           % [m³] cargo volume
    mcIn.x0.V_mps     = V_cruise;     % [m/s]
    mcIn.x0.LD        = LD_eff;       % [-] first-pass L/D estimate
    mcIn.x0.Sref_m2   = S_ref;        % [m²] from CTOL sizing
    mcIn.x0.CLmax     = CLmax;        % [-] first-pass CLmax
    mcIn.x0.SM_pct    = 7.5;          % [%] target SM (AVL not run yet)

    mcOut = monteCarloProfitSensitivity(mcIn);
end

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
wingIn.xLE_root_m = 0.0908; % profit optimizer
wingIn.y_root_m   = 0.0746; % profit optimizer (= cb_halfwidth)
wingIn.z_root_m   = 0.0;

% Elevon geometry — CMA-ES optimized (runCSopt)
wingIn.eta_cs_start  = 0.300;   % starts at 30% semispan
wingIn.eta_cs_end    = 0.749;   % ends at 74.9% semispan
wingIn.cs_chord_frac = 0.450;   % 45% of local chord

wingOut = wingGeometryDesign(wingIn);
Swet_wing = 2.04 * wingOut.S_ref_m2;  % overwrite placeholder above

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
fprintf('Tip LE x-location xLE_tip   = %.4f m\n\n', xLE_tip);

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

%% ============== Airfoil Polar Plots (Surrogate) ===============
fprintf('================ Airfoil Polar Plots (Surrogate) ================\n');

% -------- Alpha grid for plotting --------
alpha_plot_deg = (-4:0.25:10).';

% -------- Evaluate surrogate polars at current Reynolds numbers --------
rootPolarPlot = evaluateAirfoilSurrogate(airfoilDB_cached, airfoilRootName, Re_root, alpha_plot_deg);
tipPolarPlot  = evaluateAirfoilSurrogate(airfoilDB_cached, airfoilTipName,  Re_tip,  alpha_plot_deg);

% -------- 1) CL vs alpha --------
figure;
plot(rootPolarPlot.alpha_deg, rootPolarPlot.CL, 'LineWidth', 2); hold on;
plot(tipPolarPlot.alpha_deg,  tipPolarPlot.CL,  'LineWidth', 2);
grid on;
xlabel('\alpha [deg]');
ylabel('C_L [-]');
title(sprintf('Section Lift Curve at Re_{root}=%.2e, Re_{tip}=%.2e', Re_root, Re_tip));
legend(sprintf('Root: %s', rootPolarPlot.name), ...
       sprintf('Tip: %s',  tipPolarPlot.name), ...
       'Location','best');

% -------- 2) CD vs alpha --------
figure;
plot(rootPolarPlot.alpha_deg, rootPolarPlot.CD, 'LineWidth', 2); hold on;
plot(tipPolarPlot.alpha_deg,  tipPolarPlot.CD,  'LineWidth', 2);
grid on;
xlabel('\alpha [deg]');
ylabel('C_D [-]');
title(sprintf('Section Drag Curve at Re_{root}=%.2e, Re_{tip}=%.2e', Re_root, Re_tip));
legend(sprintf('Root: %s', rootPolarPlot.name), ...
       sprintf('Tip: %s',  tipPolarPlot.name), ...
       'Location','best');

% -------- 3) CM vs alpha --------
figure;
plot(rootPolarPlot.alpha_deg, rootPolarPlot.CM, 'LineWidth', 2); hold on;
plot(tipPolarPlot.alpha_deg,  tipPolarPlot.CM,  'LineWidth', 2);
grid on;
xlabel('\alpha [deg]');
ylabel('C_M [-]');
title(sprintf('Section Pitching Moment Curve at Re_{root}=%.2e, Re_{tip}=%.2e', Re_root, Re_tip));
legend(sprintf('Root: %s', rootPolarPlot.name), ...
       sprintf('Tip: %s',  tipPolarPlot.name), ...
       'Location','best');

% -------- 4) Drag polar: CL vs CD --------
figure;
plot(rootPolarPlot.CD, rootPolarPlot.CL, 'LineWidth', 2); hold on;
plot(tipPolarPlot.CD,  tipPolarPlot.CL,  'LineWidth', 2);
grid on;
xlabel('C_D [-]');
ylabel('C_L [-]');
title(sprintf('Section Drag Polar at Re_{root}=%.2e, Re_{tip}=%.2e', Re_root, Re_tip));
legend(sprintf('Root: %s', rootPolarPlot.name), ...
       sprintf('Tip: %s',  tipPolarPlot.name), ...
       'Location','best');

% -------- 5) L/D vs alpha --------
LD_root = rootPolarPlot.CL ./ rootPolarPlot.CD;
LD_tip  = tipPolarPlot.CL  ./ tipPolarPlot.CD;

LD_root(~isfinite(LD_root)) = nan;
LD_tip(~isfinite(LD_tip))   = nan;

figure;
plot(rootPolarPlot.alpha_deg, LD_root, 'LineWidth', 2); hold on;
plot(tipPolarPlot.alpha_deg,  LD_tip,  'LineWidth', 2);
grid on;
xlabel('\alpha [deg]');
ylabel('L/D [-]');
title(sprintf('Section L/D vs \\alpha at Re_{root}=%.2e, Re_{tip}=%.2e', Re_root, Re_tip));
legend(sprintf('Root: %s', rootPolarPlot.name), ...
       sprintf('Tip: %s',  tipPolarPlot.name), ...
       'Location','best');

fprintf('Generated surrogate polar plots for root and tip airfoils.\n');
fprintf('===================================================================\n\n');

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
twistIn.static_margin  = 0.06;

% Distribution settings
twistIn.model          = 'linear';
twistIn.twist_root_deg = 0; %2.40;  % profit optimizer
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
vertIn.c_v = 0.020;  % reduced for delta winglet — c_root ≈ c_tip_wing at AR=1.5, taper=0.10

% Wing reference quarter-chord x-location
vertIn.x_c4_wing_ref_m = x_c4_MAC;

% ---------- If using manual area mode instead ----------
% IMPORTANT: if manualArea is used, S_v_m2 below is TOTAL area
% vertIn.S_v_m2 = 0.08 * S_ref;

% ---------- User-selected shape ----------
vertIn.AR_v           = 1.500; % delta winglet
vertIn.taper_v        = 0.100; % delta winglet
vertIn.sweep_c4_v_deg = 65.0;  % delta winglet

vertIn.cant_deg = 0.0;
vertIn.toe_deg  = 0.0;

vertIn.topFrac = 0.66;

% ---------- Mounting at wing tip ----------
vertIn.xLE_root_v_m = wingOut.xLE_tip_m;
vertIn.y_root_v_m   = wingIn.y_root_m + wingOut.semiSpan_m;
vertIn.z_root_v_m   = wingIn.z_root_m;

% ---------- Airfoil ----------
vertIn.airfoilName = 'NACA0010';

% ---------- Rudder sizing ----------
vertIn.rudder.enable      = true;
vertIn.rudder.useTopOnly  = true;   % best match for winglet-like fin
vertIn.rudder.eta_start   = 0.15;   % start at 15% of top exposed height
vertIn.rudder.eta_end     = 0.95;   % end near tip
vertIn.rudder.cf_root     = 0.500;  % rudder chord — CMA-ES optimized (runCSopt)
vertIn.rudder.cf_tip      = 0.500;

% Run function
vertOut = verticalSurfaceDesign(vertIn);
Swet_fin = 2.04 * vertOut.S_v_total_m2;  % overwrite placeholder above

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
spanIn.alpha_ref_deg = 6.5;     % first-pass aircraft reference AoA

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


%% ================ Drag Build-Up + Aircraft Polar ==================
fprintf('\n================ Drag Build-Up + Aircraft Polar =================\n');

aeroIn = struct();

% -------- Atmosphere / flight condition --------
aeroIn.rho_kgm3      = roh;          % [kg/m^3]
aeroIn.mu_Pas        = 1.789e-5;     % [Pa*s]
aeroIn.V_cruise_mps  = V_cruise;     % [m/s]
aeroIn.W_N           = Wg;           % [N]

% -------- Aircraft geometry --------
aeroIn.Sref_m2       = S_ref;        % [m^2]
aeroIn.AR            = AR;           % [-]
aeroIn.e             = e;            % [-]
aeroIn.MAC_m         = MAC;          % [m]
aeroIn.sweepC4_deg   = wingSweep;    % [deg]

% -------- Lift-curve inputs from surrogate airfoils --------
aeroIn.Cla_root_per_deg = airfoilOut.root.Cla_per_deg;      % [1/deg]
aeroIn.Cla_tip_per_deg  = airfoilOut.tip.Cla_per_deg;       % [1/deg]

aeroIn.alphaL0_root_deg = airfoilOut.root.alphaL0_deg;      % [deg]
aeroIn.alphaL0_tip_deg  = airfoilOut.tip.alphaL0_deg;       % [deg]

aeroIn.Clmax_root = airfoilOut.root.Cl_max;                 % [-]
aeroIn.Clmax_tip  = airfoilOut.tip.Cl_max;                  % [-]

% -------- Drag build-up inputs --------
aeroIn.useDragBuildUp = useDragBuildUp;
aeroIn.CD0_user       = CD0_user;

aeroIn.Swet_wing_m2 = Swet_wing;
aeroIn.Swet_fuse_m2 = Swet_fuse;
aeroIn.Swet_fin_m2  = Swet_fin;

aeroIn.Lf_m = Lf;
aeroIn.Wf_m = Wf;
aeroIn.Hf_m = Hf;

aeroIn.tc = tc;
aeroIn.xc = xc;

aeroIn.Q_wing = Q_wing;
aeroIn.Q_fuse = Q_fuse;
aeroIn.Q_fin  = Q_fin;

% -------- Plot settings --------
aeroIn.alpha_vec_deg = alphaPolar_deg;
aeroIn.plotFigures   = true;

%-------- Run aircraft aero polar --------
aeroOut = aeroPolarAircraft(aeroIn);

%-------- Feed back useful outputs into main --------
CD0   = aeroOut.CD0;         % update main CD0 with drag build-up result
CLmax = aeroOut.CLmax_3D;    % update main CLmax with first-pass 3D estimate

fprintf('Reynolds number              = %.4e\n', aeroOut.Re);
fprintf('Skin-friction coeff Cf       = %.6f\n', aeroOut.Cf);
fprintf('CD0_wing                     = %.5f\n', aeroOut.CD0_wing);
fprintf('CD0_fuse                     = %.5f\n', aeroOut.CD0_fuse);
fprintf('CD0_fin                      = %.5f\n', aeroOut.CD0_fin);
fprintf('Total CD0                    = %.5f\n', aeroOut.CD0);
fprintf('CLalpha_2D_avg               = %.5f per deg\n', aeroOut.CLalpha_2D_avg_perDeg);
fprintf('CLalpha_3D                   = %.5f per deg\n', aeroOut.CLalpha_3D_perDeg);
fprintf('alphaL0_avg                  = %.5f deg\n', aeroOut.alphaL0_avg_deg);
fprintf('CLmax_2D_avg                 = %.5f\n', aeroOut.CLmax_2D_avg);
fprintf('CLmax_3D                     = %.5f\n', aeroOut.CLmax_3D);
fprintf('alpha_stall estimate         = %.5f deg\n', aeroOut.alpha_stall_deg);
fprintf('CL_cruise                    = %.5f\n', aeroOut.CL_cruise);
fprintf('alpha_cruise                 = %.5f deg\n', aeroOut.alpha_cruise_deg);
fprintf('CD_cruise                    = %.5f\n', aeroOut.CD_cruise);
fprintf('L/D_cruise                   = %.5f\n', aeroOut.LD_cruise);
fprintf('(L/D)_max                    = %.5f\n', aeroOut.LD_max);
fprintf('=================================================================\n\n');
%% ============== Aircraft Mass Properties ==================
fprintf('\n================ Aircraft Mass Properties =================\n');

% -------------------------------------------------------------------------
% Fuselage-only CAD mass properties
% Source: Onshape fuselage-only mass properties
%
% IMPORTANT:
%   These values should EXCLUDE wings/tails if wing/tail geometry is
%   recomputed in MATLAB during optimization.
%
% Coordinate convention assumed:
%   x aft, y right, z up, origin at aircraft nose/reference point.
% -------------------------------------------------------------------------

cadMass = struct();

cadMass.fuselageOnly.name    = 'Fuselage CAD';

cadMass.fuselageOnly.mass_kg = 1.135;                 % [kg]
cadMass.fuselageOnly.volume_m3 = 0.013;               % [m^3]
cadMass.fuselageOnly.area_m2   = 0.517;               % [m^2]

cadMass.fuselageOnly.cg_m = [ ...
    0.389, ...       % x_CG [m]
    -1.552e-6, ...   % y_CG [m]
    0.032 ];         % z_CG [m]

% Inertia tensor about fuselage CAD CG, expressed in CAD/body axes.
% Units: [kg*m^2]
cadMass.fuselageOnly.Icg_kgm2 = [ ...
     0.004,      8.011e-7,  -0.002; ...
     8.011e-7,  0.050,       1.597e-7; ...
    -0.002,      1.597e-7,   0.052 ];

% Optional cleanup: force exact symmetry to remove tiny CAD/export mismatch
cadMass.fuselageOnly.Icg_kgm2 = 0.5 * ...
    (cadMass.fuselageOnly.Icg_kgm2 + cadMass.fuselageOnly.Icg_kgm2.');

fprintf('Fuselage CAD mass     = %.3f kg\n', cadMass.fuselageOnly.mass_kg);
fprintf('Fuselage CAD CG       = [%.4f %.4g %.4f] m\n', cadMass.fuselageOnly.cg_m);
fprintf('Fuselage CAD I_CG     =\n');
disp(cadMass.fuselageOnly.Icg_kgm2);
% -------------------------------------------------------------------------
% Discrete point masses
% All coordinates are ABSOLUTE aircraft coordinates [m]
% x positive aft, y positive right, z positive up
% -------------------------------------------------------------------------
comp = repmat(makePointMass('template', 0, [0 0 0]), 0, 1);


% NOTE: makePointMass(name, mass_kg, [x, y, z]) add accurate mass values 

% ---- Main propulsion ----
comp(end+1) = makePointMass('M1 Main Motor', 0.084, [0.000,  0.000,  0.000]);
comp(end+1) = makePointMass('P1 Main Prop',  0.012, [0.000,  0.000,  0.000]);
comp(end+1) = makePointMass('ESC1 Main ESC', 0.051, [0.06,  0.000,  0.000]);

% ---- Battery / avionics ---- % MOVE THE BATTERY FOR BEST RESULTS!
comp(end+1) = makePointMass('B1 Main Battery', 0.3, [0.15, 0.000, -0.01750000/2]);
comp(end+1) = makePointMass('R1 Receiver',     0.015, [0.1, 0.000, 0.000]);

% ---- Payload ----
comp(end+1) = makePointMass('Payload', Wp/g, [0.3306, 0.000, -0.01750000/2]);

% *** SWEEP WARNING — DO NOT reorder or insert entries above this line ***
% dynamicStabilitySweep.m passes sweepIn.compFixed = comp(1:6), which
% assumes indices 1-6 are exactly: Motor, Prop, ESC, Battery, Receiver, Payload.
% If you add a mass above, increment the slice in the sweep section below.
% ************************************************************************

% ---- Wing servos: geometry-aware placement ----
eta_servo = 0.65;   % span fraction on semispan

y_servo_abs = wingIn.y_root_m + eta_servo * wingOut.semiSpan_m;
x_hinge_abs = wingOut.xLE_root_m + ...
    (wingOut.xLE_tip_m - wingOut.xLE_root_m) * eta_servo + ...
    0.75 * (wingOut.c_root_m + (wingOut.c_tip_m - wingOut.c_root_m) * eta_servo);

z_servo_abs = wingIn.z_root_m;

comp(end+1) = makePointMass('S2 Servo LHS wing', 0.0125, [x_hinge_abs, -y_servo_abs, z_servo_abs]);
comp(end+1) = makePointMass('S3 Servo RHS wing', 0.0125, [x_hinge_abs,  y_servo_abs, z_servo_abs]);

% ---- Center/back wing servo ----
comp(end+1) = makePointMass('S1 Servo back wing', 0.0125, [x_c4_MAC + 0.020, 0.000, wingIn.z_root_m]);

% ---- Vertical stabilizer servo ----
comp(end+1) = makePointMass('S4 Servo vertical stabilizer', 0.0125, ...
    [vertOut.xLE_root_v_m + 0.70*vertOut.c_root_v_m, ...
     vertOut.y_root_v_m, ...
     vertOut.z_root_v_m + 0.20*vertOut.b_v_m]);

% ---- Cargo bay servo ----
comp(end+1) = makePointMass('S5 Servo cargo bay', 0.0125, [0.61980000, 0.000, 0.000]);

% -------------------------------------------------------------------------
% Wing / vertical structure lumped masses
% First-pass placeholders. Replace with better models or CAD subtraction later.
% -------------------------------------------------------------------------

% First-pass wing structure estimate
m_wing_struct_kg = 0.392;   % [kg] <-- replace later with model/CAD-informed value

% Place wing structural mass near quarter-chord MAC of each half wing
x_wing_struct = x_c4_MAC;
y_wing_struct = wingIn.y_root_m + 0.42 * wingOut.semiSpan_m;
z_wing_struct = wingIn.z_root_m;

comp(end+1) = makePointMass('Wing structure L', 0.5*m_wing_struct_kg, [x_wing_struct, -y_wing_struct, z_wing_struct]);
comp(end+1) = makePointMass('Wing structure R', 0.5*m_wing_struct_kg, [x_wing_struct,  y_wing_struct, z_wing_struct]);

% First-pass vertical structure estimate
m_vert_struct_kg = 0.048;   % [kg] total both fins — scaled from AR_v=2 baseline (mass ∝ AR_v, optimized AR_v=2.41)

if vertOut.isTwin
    m_fin_each = 0.5 * m_vert_struct_kg;
else
    m_fin_each = m_vert_struct_kg;
end

x_fin_struct = vertOut.xLE_root_v_m + 0.40*vertOut.c_root_v_m;
y_fin_struct = vertOut.y_root_v_m;
z_fin_struct = vertOut.z_root_v_m + 0.30*vertOut.b_v_m;

comp(end+1) = makePointMass('Vertical structure R', m_fin_each, [x_fin_struct,  y_fin_struct, z_fin_struct]);

if vertOut.isTwin
    comp(end+1) = makePointMass('Vertical structure L', m_fin_each, [x_fin_struct, -y_fin_struct, z_fin_struct]);
end

% -------------------------------------------------------------------------
% Mass properties input
% -------------------------------------------------------------------------
massIn = struct();
massIn.cadBodies   = cadMass;
massIn.pointMasses = comp;

massOut = aircraftMassProperties(massIn);

fprintf('Total aircraft mass           = %.4f kg\n', massOut.mass_kg);
fprintf('Total aircraft weight         = %.4f N\n', massOut.weight_N);
fprintf('Aircraft CG                   = [%.4f, %.4f, %.4f] m\n', ...
    massOut.cg_m(1), massOut.cg_m(2), massOut.cg_m(3));

fprintf('\nInertia tensor about aircraft CG [kg*m^2]:\n');
disp(massOut.Icg_kgm2);

fprintf('Ixx = %.6f kg*m^2\n', massOut.Icg_kgm2(1,1));
fprintf('Iyy = %.6f kg*m^2\n', massOut.Icg_kgm2(2,2));
fprintf('Izz = %.6f kg*m^2\n', massOut.Icg_kgm2(3,3));
fprintf('Ixy = %.6f kg*m^2\n', massOut.Icg_kgm2(1,2));
fprintf('Ixz = %.6f kg*m^2\n', massOut.Icg_kgm2(1,3));
fprintf('Iyz = %.6f kg*m^2\n', massOut.Icg_kgm2(2,3));
fprintf('=============================================================\n\n');


%% ============== Unloaded Mass Case ==================
% Remove payload only; payload location remains fixed by design

payloadIdx = find(strcmp({comp.name}, 'Payload'), 1);

if isempty(payloadIdx)
    error('Could not find Payload component in comp array.');
end

comp_unloaded = comp;
comp_unloaded(payloadIdx) = [];

massIn_unloaded = struct();
massIn_unloaded.cadBodies   = cadMass;
massIn_unloaded.pointMasses = comp_unloaded;

massOut_unloaded = aircraftMassProperties(massIn_unloaded);

fprintf('---------------- Unloaded Mass Case ----------------\n');
fprintf('Total unloaded mass          = %.4f kg\n', massOut_unloaded.mass_kg);
fprintf('Total unloaded weight        = %.4f N\n', massOut_unloaded.weight_N);
fprintf('Unloaded aircraft CG         = [%.4f, %.4f, %.4f] m\n', ...
    massOut_unloaded.cg_m(1), massOut_unloaded.cg_m(2), massOut_unloaded.cg_m(3));
fprintf('----------------------------------------------------\n\n');

%% ============== CG as % MAC ===============
fprintf('================ CG Location (% MAC) =================\n');

x_cg   = massOut.cg_m(1);          % [m]
xLEMAC = wingOut.xLE_MAC_m;        % [m]
MAC    = wingOut.MAC_m;            % [m]

cg_percent_MAC = (x_cg - xLEMAC) / MAC * 100;

fprintf('x_cg           = %.4f m\n', x_cg);
fprintf('x_LE_MAC       = %.4f m\n', xLEMAC);
fprintf('MAC            = %.4f m\n', MAC);
fprintf('CG location    = %.2f %% MAC\n', cg_percent_MAC);

% Optional flagging
if cg_percent_MAC < 0
    fprintf('⚠️ CG is ahead of MAC leading edge (very nose heavy)\n');
elseif cg_percent_MAC < 10
    fprintf('⚠️ CG is very forward (high stability, high trim drag)\n');
elseif cg_percent_MAC > 40
    fprintf('⚠️ CG is aft (potential instability)\n');
else
    fprintf('✅ CG in reasonable range\n');
end

fprintf('=====================================================\n\n');

%% ============== Updated Performance State =================
fprintf('\n================ Updated Performance State =================\n');

perfIn = struct();

% Use loaded aircraft weight from mass model
perfIn.rho_kgm3 = roh;                 % [kg/m^3]
perfIn.W_N      = massOut.weight_N;    % [N]
perfIn.Sref_m2  = S_ref;               % [m^2]

% Refined aero
perfIn.CD0   = aeroOut.CD0;            % [-]
perfIn.CLmax = aeroOut.CLmax_3D;       % [-]
perfIn.e     = e;                      % [-]
perfIn.AR    = AR;                     % [-]

% Propulsion
perfIn.V_vec_mps = propOut.V_vec_mps(:);
perfIn.T_vec_N   = propOut.T_vec_N(:);

% Selected climb-evaluation speed
perfIn.V_climb_eval_mps = mission.V_climb_mps;   % [m/s]
perfIn.CLTO_frac        = 0.8;                   % [-]

perfOut = updatePerformanceState(perfIn);

fprintf('Loaded weight W               = %.4f N\n', massOut.weight_N);
fprintf('Updated stall speed Vs        = %.4f m/s\n', perfOut.Vs_mps);
fprintf('Estimated takeoff speed VTO   = %.4f m/s\n', perfOut.VTO_mps);
fprintf('Takeoff CL used               = %.4f\n', perfOut.CLTO);

if perfOut.validCruise
    fprintf('Solved cruise speed           = %.4f m/s\n', perfOut.Vcruise_mps);
    fprintf('Solved cruise speed           = %.2f mph\n', perfOut.Vcruise_mps * 2.23694);
    fprintf('Cruise CL                     = %.5f\n', perfOut.CLcruise);
    fprintf('Cruise CD                     = %.5f\n', perfOut.CDcruise);
    fprintf('Cruise L/D                    = %.5f\n', perfOut.LDcruise);
else
    fprintf('No thrust-drag cruise intersection found in current speed range.\n');
end

fprintf('Eval climb speed              = %.4f m/s\n', perfOut.Vclimb_eval_mps);
fprintf('Thrust at climb speed         = %.4f N\n', perfOut.Tclimb_N);
fprintf('Drag at climb speed           = %.4f N\n', perfOut.Dclimb_N);
fprintf('Max climb gradient at Vclimb  = %.5f\n', perfOut.G_climb_max);
fprintf('Max climb rate at Vclimb      = %.5f m/s\n', perfOut.ROC_climb_max_mps);
fprintf('============================================================\n\n');

% Extract actual performance (DO NOT FEED BACK)

V_stall_actual = perfOut.Vs_mps;

if perfOut.validCruise
    V_cruise_actual = perfOut.Vcruise_mps;
else
    V_cruise_actual = NaN;
end

fprintf('\n===== PERFORMANCE CONSISTENCY CHECK =====\n');
fprintf('Design stall speed  = %.3f m/s\n', V_stall_mps);
fprintf('Actual stall speed  = %.3f m/s\n', V_stall_actual);

if perfOut.validCruise
    fprintf('Design cruise speed = %.3f m/s\n', V_cruise);
    fprintf('Actual cruise speed = %.3f m/s\n', V_cruise_actual);
end
fprintf('========================================\n\n');

if perfOut.G_climb_max < mission.G_climb
    warning('Required climb gradient exceeds available climb gradient at selected climb speed.');
end
%% ============== Static Stability Analysis ==================
fprintf('\n================ Static Stability Analysis =================\n');

stabIn = struct();

% -------- Geometry references --------
stabIn.cMAC_m   = wingOut.MAC_m;
stabIn.xLEMAC_m = wingOut.xLE_MAC_m;

% -------- Loaded / unloaded CGs from mass model --------
stabIn.cg_loaded_m   = massOut.cg_m;
stabIn.cg_unloaded_m = massOut_unloaded.cg_m;

% -------- Neutral point choice --------
% Preferred final path:
% stabIn.xNP_m = aeroOut.xNP_m;   % from AVL / aerodynamic model

% Temporary fallback:
stabIn.useApproxNP     = true;
stabIn.xACwingApprox_m = wingOut.xLE_MAC_m + 0.25 * wingOut.MAC_m;

% -------- Optional target band --------
stabIn.SM_target_min = 0.10;   % 10%
stabIn.SM_target_max = 0.20;   % 20%

stabOut = staticStabilityAnalysisNP_SM(stabIn);

fprintf('Neutral point x_NP             = %.4f m\n', stabOut.xNP_m);
fprintf('Neutral point                  = %.2f %% MAC\n', 100*stabOut.loaded.xnp_over_MAC);

fprintf('\n---- Loaded case ----\n');
fprintf('CG                            = %.4f m\n', stabOut.loaded.xcg_m);
fprintf('CG                            = %.2f %% MAC\n', 100*stabOut.loaded.xcg_over_MAC);
fprintf('Static margin                 = %.2f %%\n', 100*stabOut.loaded.SM);
fprintf('Statically stable             = %s\n', string(stabOut.loaded.isStaticallyStable));
fprintf('In target band (10-20%%)       = %s\n', string(stabOut.loaded.inTargetBand));

fprintf('\n---- Unloaded case ----\n');
fprintf('CG                            = %.4f m\n', stabOut.unloaded.xcg_m);
fprintf('CG                            = %.2f %% MAC\n', 100*stabOut.unloaded.xcg_over_MAC);
fprintf('Static margin                 = %.2f %%\n', 100*stabOut.unloaded.SM);
fprintf('Statically stable             = %s\n', string(stabOut.unloaded.isStaticallyStable));
fprintf('In target band (10-20%%)       = %s\n', string(stabOut.unloaded.inTargetBand));

fprintf('\nCG shift due to payload removal = %.2f %% MAC\n', ...
    100*(stabOut.unloaded.xcg_over_MAC - stabOut.loaded.xcg_over_MAC));

if stabOut.usedApproxNP
    fprintf('\nWARNING: Neutral point currently uses approximate wing AC only.\n');
    fprintf('         Replace with AVL / aero-based xNP before trusting final stability margins.\n');
end

fprintf('==============================================================\n\n');


%% =============== 3D Geometry Plot (Loaded / Unloaded CG) =========

geom3DIn = struct();

% -------- Wing geometry --------
geom3DIn.b_m      = wingOut.b_m;
geom3DIn.b_half_m = wingOut.b_m / 2;

geom3DIn.c_root_m = wingOut.c_root_m;
geom3DIn.c_tip_m  = wingOut.c_tip_m;

% -------- Absolute wing root --------
geom3DIn.xLE_root_m = wingIn.xLE_root_m;
geom3DIn.y_root_m   = wingIn.y_root_m;
geom3DIn.z_root_m   = wingIn.z_root_m;

% -------- Absolute wing tip --------
geom3DIn.xLE_tip_m  = wingOut.xLE_tip_m;
geom3DIn.yLE_tip_m  = wingIn.y_root_m + wingOut.semiSpan_m;
geom3DIn.zLE_tip_m  = wingIn.z_root_m;

% -------- MAC --------
geom3DIn.xLE_MAC_m = wingOut.xLE_MAC_m;
geom3DIn.y_MAC_m   = wingIn.y_root_m + wingOut.y_MAC_m;
geom3DIn.z_MAC_m   = wingIn.z_root_m;
geom3DIn.MAC_m     = wingOut.MAC_m;

% -------- Twist --------
geom3DIn.twist_root_deg = twistOut.twist_root_deg;
geom3DIn.twist_tip_deg  = twistOut.twist_tip_deg;

% -------- Plot options --------
geom3DIn.plotVertical        = true;
geom3DIn.plotBody            = false;
geom3DIn.plotCG              = true;
geom3DIn.plotComponents      = true;
geom3DIn.plotComponentLabels = false;   % turn true later if you want labels
geom3DIn.plotControlSurfaces = true;

% -------- Control surface geometry for visualization --------
geom3DIn.eta_cs_start    = wingIn.eta_cs_start;
geom3DIn.eta_cs_end      = wingIn.eta_cs_end;
geom3DIn.cs_chord_frac   = wingIn.cs_chord_frac;
geom3DIn.rudder_cf       = vertIn.rudder.cf_root;
geom3DIn.rudder_eta_start = vertIn.rudder.eta_start;
geom3DIn.rudder_eta_end   = vertIn.rudder.eta_end;

% -------- Vertical surfaces --------
geom3DIn.vertOut = vertOut;

% -------- Loaded CG --------
geom3DIn.xCG_loaded_m = massOut.cg_m(1);
geom3DIn.yCG_loaded_m = massOut.cg_m(2);
geom3DIn.zCG_loaded_m = massOut.cg_m(3);

% -------- Unloaded CG --------
geom3DIn.xCG_unloaded_m = massOut_unloaded.cg_m(1);
geom3DIn.yCG_unloaded_m = massOut_unloaded.cg_m(2);
geom3DIn.zCG_unloaded_m = massOut_unloaded.cg_m(3);

% -------- Components to display --------
geom3DIn.components = comp;

plotAircraftGeometry3D(geom3DIn);
%% =========== V-n Diagram ===================

vnIn = struct();

% atmosphere / aircraft
vnIn.rho       = roh;      % [kg/m^3]
vnIn.W_N       = Wg;       % [N]
vnIn.S_ref_m2  = S_ref;    % [m^2]

% maneuver / aero assumptions
vnIn.CLmax_pos   = CLmax;          % [-]
vnIn.CLmax_neg   = -0.8 * CLmax;   % [-] first-pass assumption
vnIn.n_pos_limit = 3.8;            % [-]
vnIn.n_neg_limit = -1.5;           % [-]

% speeds
vnIn.Vc_mps = mission.V_pattern;   % [m/s]
vnIn.Vd_mps = 1.25 * vnIn.Vc_mps;  % [m/s] first-pass assumption

% plotting options
vnIn.plotUnits  = 'mps';
vnIn.Npts       = 500;
vnIn.makeFigure = true;

%% -------- Gust overlay inputs --------
% Use class / project-required gust velocities if provided.
% First-pass example values shown here:
Ude_Vc_fps = 30;     % [ft/s] example at Vc
Ude_Vd_fps = 15;     % [ft/s] example at Vd

ft_to_m = 0.3048;
Ude_Vc = Ude_Vc_fps * ft_to_m;   % [m/s]
Ude_Vd = Ude_Vd_fps * ft_to_m;   % [m/s]

% Mean 2D section lift-curve slope from surrogate airfoils
a0_root_per_rad = airfoilOut.root.Cla_per_deg * (180/pi);
a0_tip_per_rad  = airfoilOut.tip.Cla_per_deg  * (180/pi);
a0_avg_per_rad  = 0.5 * (a0_root_per_rad + a0_tip_per_rad);

% First-pass finite-wing lift-curve slope
a_per_rad = a0_avg_per_rad / (1 + a0_avg_per_rad/(pi*e*AR));

% Use current design values
WS = WS_design;    % [N/m^2]
cbar = MAC;        % [m]
rho_g = roh;       % [kg/m^3]
g0 = g;            % [m/s^2]

% Gust alleviation factor
mu_g = 2*WS / (rho_g * cbar * a_per_rad * g0);
K_g  = 0.88*mu_g / (5.3 + mu_g);

% Speeds used for gust overlay
Vc = vnIn.Vc_mps;
Vd = vnIn.Vd_mps;

% Load increments at Vc and Vd
delta_n_Vc = (K_g * rho_g * Vc * a_per_rad * Ude_Vc) / (2*WS);
delta_n_Vd = (K_g * rho_g * Vd * a_per_rad * Ude_Vd) / (2*WS);

% Store for plotting
vnIn.gust.enable = true;
vnIn.gust.V_pts_mps = [0, Vc, Vd];
vnIn.gust.n_pos = [1, 1 + delta_n_Vc, 1 + delta_n_Vd];
vnIn.gust.n_neg = [1, 1 - delta_n_Vc, 1 - delta_n_Vd];

fprintf('\n================ Gust Overlay =================\n');
fprintf('Mean 2D lift-curve slope a0   = %.4f per rad\n', a0_avg_per_rad);
fprintf('Finite-wing lift slope a      = %.4f per rad\n', a_per_rad);
fprintf('Wing loading W/S              = %.4f N/m^2\n', WS);
fprintf('Mean aerodynamic chord cbar   = %.4f m\n', cbar);
fprintf('Gust alleviation factor K_g   = %.4f\n', K_g);
fprintf('Ude at Vc                     = %.4f m/s\n', Ude_Vc);
fprintf('Ude at Vd                     = %.4f m/s\n', Ude_Vd);
fprintf('Delta n at Vc                 = %.4f\n', delta_n_Vc);
fprintf('Delta n at Vd                 = %.4f\n', delta_n_Vd);
fprintf('Positive gust load at Vc      = %.4f\n', 1 + delta_n_Vc);
fprintf('Negative gust load at Vc      = %.4f\n', 1 - delta_n_Vc);
fprintf('Positive gust load at Vd      = %.4f\n', 1 + delta_n_Vd);
fprintf('Negative gust load at Vd      = %.4f\n', 1 - delta_n_Vd);
fprintf('================================================\n\n');

% run function
vnOut = plotVNDiagram(vnIn);

fprintf('\n================ V-n Diagram =================\n');
fprintf('Positive CLmax              = %.4f\n', vnOut.CLmax_pos);
fprintf('Negative CLmax              = %.4f\n', vnOut.CLmax_neg);
fprintf('Positive stall speed Vs+    = %.3f m/s\n', vnOut.Vs_pos_mps);
fprintf('Negative stall speed Vs-    = %.3f m/s\n', vnOut.Vs_neg_mps);
fprintf('Maneuver speed Va           = %.3f m/s\n', vnOut.Va_mps);
fprintf('Negative corner speed       = %.3f m/s\n', vnOut.Vneg_mps);
fprintf('Cruise speed Vc             = %.3f m/s\n', vnOut.Vc_mps);
fprintf('Dive speed Vd               = %.3f m/s\n', vnOut.Vd_mps);
fprintf('Positive limit load factor  = %.3f\n', vnOut.n_pos_limit);
fprintf('Negative limit load factor  = %.3f\n', vnOut.n_neg_limit);
fprintf('================================================\n\n');

%% =============== Dynamic Stability Analysis (AVL) ==============

dynIn = struct();

% Mass and inertia (body axes, at CG)
dynIn.mass_kg     = massOut.mass_kg;
dynIn.Icg_kgm2    = massOut.Icg_kgm2;
dynIn.cg_m        = massOut.cg_m;

% Aerodynamic reference
dynIn.S_ref_m2    = S_ref;
dynIn.MAC_m       = MAC;
dynIn.b_m         = b;

% Wing geometry
dynIn.xLE_root_m  = wingIn.xLE_root_m;
dynIn.xLE_tip_m   = wingOut.xLE_tip_m;
dynIn.y_root_m    = wingIn.y_root_m;
dynIn.semiSpan_m  = wingOut.semiSpan_m;
dynIn.c_root_m    = c_root;
dynIn.c_tip_m     = c_tip;

% Control surface (elevon)
dynIn.eta_cs_start  = wingIn.eta_cs_start;
dynIn.eta_cs_end    = wingIn.eta_cs_end;
dynIn.cs_chord_frac = wingIn.cs_chord_frac;

% Airfoil zero-lift angle root/tip (spanwise-interpolated in AInc formula)
dynIn.alphaL0_root_deg = airfoilOut.root.alphaL0_deg;
dynIn.alphaL0_tip_deg  = airfoilOut.tip.alphaL0_deg;

% Airfoil lift curve slope root/tip (spanwise-interpolated CLAF in AVL)
dynIn.Cla_root_per_deg = airfoilOut.root.Cla_per_deg;
dynIn.Cla_tip_per_deg  = airfoilOut.tip.Cla_per_deg;

% Actual airfoil dat files (AVL reads camber directly; AInc = geometric twist only)
dynIn.airfoilRootFile     = fullfile(repoRoot, 'data', 'airfoils', airfoilRootName);
dynIn.airfoilTipFile      = fullfile(repoRoot, 'data', 'airfoils', airfoilTipName);
dynIn.airfoilFuselageFile = fullfile(repoRoot, 'data', 'airfoils', 'naca0012.dat');  % symmetric airfoil for pitch damping

% Centerbody geometry: fixed fuselage, wing slides fwd/aft via xLE_root
% Wing-fuselage join distance scales as 6% of fuselage length
% CONSTRAINT: Fuselage LE must be ≥ 0.08154122 m to preserve EH0.0/9.0 wing-join sections
cb_join_distance = max(0.06 * Lf, 0.08154122);  % [m] gap from motor (x=0) to fuselage LE

dynIn.cb_chord_m = Lf;          % [m] centerbody chord at centerline (= fuselage length)
dynIn.cb_z_m     = 0.03;        % [m] centerbody LE height above wing plane
dynIn.cb_xLE_m   = cb_join_distance;  % [m] fuselage LE x-position (motor at origin)

% Flight condition
dynIn.V_mps         = V_cruise;
dynIn.rho_kgm3      = roh;
dynIn.CD0           = aeroOut.CD0;
dynIn.CL_trim       = aeroOut.CL_cruise;
dynIn.alpha_trim_deg = aeroOut.alpha_cruise_deg;

% Wing twist (linear from root to tip; AInc varies spanwise in AVL)
dynIn.twist_root_deg = twistOut.twist_root_deg;
dynIn.twist_tip_deg  = twistOut.twist_tip_deg;

% Vertical fin geometry (root = wing tip, top and bottom tips)
dynIn.xLE_root_v_m    = vertOut.xLE_root_v_m;
dynIn.y_root_v_m      = vertOut.y_root_v_m;
dynIn.z_root_v_m      = vertOut.z_root_v_m;
dynIn.xLE_top_v_m     = vertOut.xLE_top_v_m;
dynIn.y_top_v_m       = vertOut.y_top_v_m;
dynIn.z_top_v_m       = vertOut.z_top_v_m;
dynIn.xLE_bottom_v_m  = vertOut.xLE_bottom_v_m;
dynIn.y_bottom_v_m    = vertOut.y_bottom_v_m;
dynIn.z_bottom_v_m    = vertOut.z_bottom_v_m;
dynIn.c_root_v_m      = c_root_v;
dynIn.c_tip_v_m       = c_tip_v;

% Rudder
dynIn.rudder_eta_start = vertIn.rudder.eta_start;
dynIn.rudder_eta_end   = vertIn.rudder.eta_end;
dynIn.rudder_cf        = vertIn.rudder.cf_root;

% AVL executable and working directory
%
% ---- WINDOWS SETUP (one-time, teammates on PC) ----
% 1. Go to: https://web.mit.edu/drela/Public/web/avl/
% 2. Download the Windows binary (e.g. "AVL 3.36 Win")
% 3. Extract the zip and find avl.exe inside
% 4. Copy/rename it to:  <project root>/AVL/avl.exe
% 5. If Windows flags it as unrecognized: right-click avl.exe
%    -> Properties -> check "Unblock" -> OK
% 6. Run main.m normally — no other changes needed
% ---------------------------------------------------
%
% Mac/Linux: avl352 is already in AVL/ and runs as-is
%
avlDir     = fullfile(fileparts(mfilename('fullpath')), 'AVL');
avlExeDir  = fullfile(avlDir, 'Nimbus');
if ispc
    dynIn.avlExe = fullfile(avlExeDir, 'avl.exe');
else
    dynIn.avlExe = fullfile(avlExeDir, 'avl352');
end
dynIn.workDir     = avlDir;
dynIn.plotModes        = showPlots;
dynIn.viewGeometry     = viewGeometry;
dynIn.modelCenterbody  = modelCenterbody;

dynOut = dynamicStabilityAVL(dynIn);

fprintf('\n================ DYNAMIC STABILITY SUMMARY =================\n');
fprintf('Short period: wn=%.3f rad/s, zeta=%.3f\n', ...
    dynOut.longModes.shortPeriod.metrics.wn, ...
    dynOut.longModes.shortPeriod.metrics.zeta);
fprintf('Phugoid:      wn=%.3f rad/s, zeta=%.3f\n', ...
    dynOut.longModes.phugoid.metrics.wn, ...
    dynOut.longModes.phugoid.metrics.zeta);
fprintf('Dutch roll:   wn=%.3f rad/s, zeta=%.3f\n', ...
    dynOut.latModes.dutchRoll.metrics.wn, ...
    dynOut.latModes.dutchRoll.metrics.zeta);
fprintf('Roll subside: tau=%.3f s\n', dynOut.latModes.rollSubsidence.metrics.tau);
if real(dynOut.latModes.spiral.lambda) > 0
    fprintf('Spiral:       t_double=%.1f s\n', dynOut.latModes.spiral.metrics.tDouble);
else
    fprintf('Spiral:       stable (t_half=%.1f s)\n', dynOut.latModes.spiral.metrics.tHalf);
end
fprintf('=============================================================\n\n');

%% =============== SM Correction Advisor ==============
fprintf('\n================ SM CORRECTION ADVISOR =================\n');

SM_target    = 7.5;   % [%] midpoint of 5-10% target band
xNP_curr     = massOut.cg_m(1) + dynOut.SM_pct/100 * wingOut.MAC_m;
m_batt_kg    = 0.161;
x_batt_curr  = 0.553;
m_no_batt_kg = massOut.mass_kg - m_batt_kg;
x_cg_no_batt = (massOut.mass_kg*massOut.cg_m(1) - m_batt_kg*x_batt_curr) / m_no_batt_kg;
x_cg_target  = xNP_curr - SM_target/100 * wingOut.MAC_m;
x_batt_req   = (massOut.mass_kg*x_cg_target - m_no_batt_kg*x_cg_no_batt) / m_batt_kg;

fprintf('  Current SM (AVL)             = %.2f%%\n', dynOut.SM_pct);
fprintf('  Target SM                    = %.1f%%  (5-10%% band midpoint)\n', SM_target);
fprintf('  Required battery x-position  = %.3f m  (fuselage = 0 to %.2f m)\n', x_batt_req, Lf);

if x_batt_req >= 0 && x_batt_req <= Lf
    fprintf('  --> Move battery to x = %.3f m  (currently %.3f m)\n', x_batt_req, x_batt_curr);
else
    fprintf('  *** Battery fix infeasible — 161g has too little CG authority ***\n');
    fprintf('  --> Adjust wing x-position instead:\n');
    % masses that translate with xLE_root: wing structure + 2 wing servos + fin structure + fin servo
    m_wing_move_kg = m_wing_struct_kg + 0.018 + m_vert_struct_kg + 0.009;
    f_move   = m_wing_move_kg / massOut.mass_kg;
    % ΔSM = δ*(1 - f_move)/MAC  →  δ = ΔSM*MAC/(1-f_move)  [negative = move forward]
    delta_xLE  = (SM_target - dynOut.SM_pct)/100 * wingOut.MAC_m / (1 - f_move);
    xLE_target = wingIn.xLE_root_m + delta_xLE;
    fprintf('     Set  wingIn.xLE_root_m = %.4f m  (currently %.4f m)\n', xLE_target, wingIn.xLE_root_m);
    fprintf('     Re-run main.m to verify SM with AVL.\n');
end
fprintf('=========================================================\n\n');

%% =============== Control Surface Sizing ==============
csIn.CLde      = dynOut.controlDerivs.CLde;
csIn.Cmde      = dynOut.controlDerivs.Cmde;
csIn.Clda      = dynOut.controlDerivs.Clda;
csIn.Cnda      = dynOut.controlDerivs.Cnda;
csIn.Cndr      = dynOut.controlDerivs.Cndr;
csIn.Cm0_trim  = dynOut.controlDerivs.Cm0_trim;
csIn.CL_trim   = aeroOut.CL_cruise;
csIn.CLmax     = CLmax;
csIn.V_mps     = V_cruise;
csIn.rho_kgm3  = roh;
csIn.S_ref_m2  = wingOut.S_ref_m2;
csIn.b_m       = wingOut.b_m;
csIn.mass_kg   = massOut.mass_kg;
csIn.Clp       = dynOut.derivatives.Clp;
csIn.showPlots = showPlots;

% elevon + rudder geometry for hinge moment calculation
csIn.cs_chord_frac   = wingIn.cs_chord_frac;
csIn.eta_cs_start    = wingIn.eta_cs_start;
csIn.eta_cs_end      = wingIn.eta_cs_end;
csIn.c_root_m        = wingOut.c_root_m;
csIn.c_tip_m         = wingOut.c_tip_m;
csIn.semiSpan_m      = wingOut.semiSpan_m;
csIn.rudder_c_avg_m  = 0.5 * (vertOut.rudder.c_root_m + vertOut.rudder.c_tip_m);
csIn.rudder_height_m = vertOut.rudder.height_m;

csOut = controlSurfaceSizing(csIn);

%% =============== Control Surface Optimization ==============
if runCSopt
    csOptIn.dynIn           = dynIn;
    csOptIn.CL_trim         = aeroOut.CL_cruise;
    csOptIn.CLmax           = CLmax;
    csOptIn.V_mps           = V_cruise;
    csOptIn.rho_kgm3        = roh;
    csOptIn.S_ref_m2        = wingOut.S_ref_m2;
    csOptIn.b_m             = wingOut.b_m;
    csOptIn.mass_kg         = massOut.mass_kg;
    csOptIn.delta_e_max     = 20;
    csOptIn.delta_r_max     = 25;
    csOptIn.p_ss_min_dps    = 100;
    csOptIn.de_trim_max_deg = 15;
    csOptIn.eta_end_max     = 0.95;

    csOptOut = optimizeControlSurfaces(csOptIn);
end

%% =============== Dynamic Stability Parameter Sweep ==============
sweepIn.wingIn  = wingIn;
sweepIn.twistIn = twistIn;
sweepIn.vertIn  = vertIn;
sweepIn.dynIn   = dynIn;
sweepIn.maxIter = 50;

% Wing: [lo, hi]
sweepIn.wingSweep_range = [0,   40 ];   % [deg]
sweepIn.wingTaper_range = [0.60, 1.00];  % [-]
sweepIn.twistTip_range  = [-5.0, 0.0];  % [deg] tip washout (root fixed at 0)

% Vertical fins: [lo, hi]
sweepIn.AR_v_range    = [1.0,  2.0 ];  % [-]  delta winglet space
sweepIn.taperV_range  = [0.05, 0.25];  % [-]  delta winglet space
sweepIn.sweepV_range  = [55,   75  ];  % [deg] delta winglet space

% Wing attachment fore/aft position: slides NP aft when wing moves aft
sweepIn.xLE_root_range = [0.05, 0.30];  % [m]  baseline is 0.0822 m

% Mass inputs — fixed components + scalars for geometry-dependent rebuild
sweepIn.cadMass          = cadMass;
sweepIn.compFixed        = comp(1:6);    % motor, prop, ESC, battery, receiver, payload
sweepIn.eta_servo        = eta_servo;
sweepIn.m_wing_struct_kg = m_wing_struct_kg;
sweepIn.m_vert_struct_kg = m_vert_struct_kg;

if runSweep
    sweepOut = dynamicStabilitySweep(sweepIn);
end

%% =============== CMA-ES Dynamic Stability Optimization ==============
if runOptimization
    optIn.ctx    = sweepIn;   % reuse context built above (has cadMass, compFixed, etc.)

    % initial point: delta winglet baseline
    % x(3) = tip twist (root fixed at 0; negative = washout toward tip)
    % x(4) = AR_v, x(5) = taper_v, x(6) = sweep_c4_v_deg
    optIn.x0     = [21.0; 0.849; -2.0; 1.5; 0.10; 65.0; 0.1498];

    % search bounds — delta winglet space
    optIn.lb     = [20;  0.60; -5.0; 1.0; 0.05; 55; 0.05];
    optIn.ub     = [40;  1.00;  0.0; 2.0; 0.25; 75; 0.20];

    optIn.sigma0 = 1.0;

    optIn.lambda   = 20;
    optIn.maxGen   = 500;
    optIn.tolSigma = 1e-7;
    optIn.tolFun   = 1e-6;
    optIn.verbose  = 10;

    optOut = optimizeDynamicStability(optIn);
end

%% ============== Advanced Aerodynamics (CFD) ============

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

%% =============== Profit Re-evaluation with Actual Physics ==============
fprintf('\n================ PROFIT RE-EVALUATION (Actual Physics) =================\n');

% Use drag-polar L/D (consistent with optimizer) rather than perfOut.LDcruise
LD_physics = aeroOut.LD_cruise;

Wg_physics    = massOut.weight_N;             % [N] actual gross weight (loaded)
Wg_no_payload = Wg_physics - Wp;             % [N] aircraft without current payload

% Re-compute mission energy with actual Wg and LD
[~, ~, Ef_phys_raw, ~, ~, ~] = energyCalc(Wg_physics, eta_p, LD_physics, delta_h, R_cruise, reserve_factor);
Ef_physics = 20 * Ef_phys_raw;
J_physics  = profitPerUnitTime(Wp, Vp, Ef_physics, Wg_physics, Tf);

fprintf('  Parametric J (early script)  = %+.4f $/hr   [LD=%.2f, Wg=%.0fg]\n', J*3600, LD, Wg/g*1000);
fprintf('  Physics-based J              = %+.4f $/hr   [LD=%.2f, Wg=%.0fg]\n', J_physics*3600, LD_physics, Wg_physics/g*1000);
fprintf('  Actual cruise L/D            = %.3f\n', LD_physics);
fprintf('  Actual gross weight          = %.1f g\n', Wg_physics/g*1000);

% ---- CG / stability setup for sweep ----
% xNP is fixed (aero surfaces don't change); derive it from current AVL SM.
x_payload_m      = 0.2894;                              % [m] payload CG x-location (matches loaded CG)
m_total_kg       = massOut.mass_kg;                     % [kg]
m_no_payload_kg  = m_total_kg - Wp/g;                   % [kg] aircraft without payload
x_cg_total       = massOut.cg_m(1);                     % [m] current loaded CG
x_cg_no_payload  = (m_total_kg*x_cg_total - (Wp/g)*x_payload_m) / m_no_payload_kg;  % [m]
xNP_m            = x_cg_total + dynOut.SM_pct/100 * wingOut.MAC_m;  % [m] neutral point (fixed)
SM_min_pct       = 5.0;   % [%] minimum acceptable static margin

% ---- payload weight sweep (Vp fixed by cargo bay geometry) ----
Wp_g_sweep = linspace(200, 1200, 120);
J_sweep    = nan(size(Wp_g_sweep));
Vs_sweep   = nan(size(Wp_g_sweep));
SM_sweep   = nan(size(Wp_g_sweep));
xCG_sweep  = nan(size(Wp_g_sweep));

CLmax_3D  = aeroOut.CLmax_3D;
S_ref_m2  = wingOut.S_ref_m2;
MAC_m     = wingOut.MAC_m;

for k = 1:length(Wp_g_sweep)
    Wp_k   = (Wp_g_sweep(k)/1000) * g;
    Wg_k   = Wg_no_payload + Wp_k;
    m_k    = m_no_payload_kg + Wp_k/g;

    % CG shift: payload at fixed x, rest of aircraft CG unchanged
    x_cg_k  = (m_no_payload_kg*x_cg_no_payload + (Wp_k/g)*x_payload_m) / m_k;
    SM_k    = (xNP_m - x_cg_k) / MAC_m * 100;

    [~, ~, Ef_k, ~, ~, ~] = energyCalc(Wg_k, eta_p, LD_physics, delta_h, R_cruise, reserve_factor);
    J_sweep(k)   = profitPerUnitTime(Wp_k, Vp, 20*Ef_k, Wg_k, Tf);
    Vs_sweep(k)  = sqrt(2*Wg_k / (roh * S_ref_m2 * CLmax_3D));
    SM_sweep(k)  = SM_k;
    xCG_sweep(k) = x_cg_k;
end

% constrained optimum: best J where SM >= SM_min
feasible = SM_sweep >= SM_min_pct;
if any(feasible)
    J_feasible       = J_sweep;
    J_feasible(~feasible) = NaN;
    [J_con, idx_con] = max(J_feasible);
    Wp_g_con         = Wp_g_sweep(idx_con);
    SM_con           = SM_sweep(idx_con);
    Vs_con           = Vs_sweep(idx_con);
else
    J_con    = NaN;  Wp_g_con = NaN;
    SM_con   = NaN;  Vs_con   = NaN;
    fprintf('  *** No feasible payload weight found with SM >= %.0f%% ***\n', SM_min_pct);
end

fprintf('\n  Constrained optimum (SM>=%.0f%%) = %.0f g   (J = %.4f $/hr,  SM = %.1f%%)\n', SM_min_pct, Wp_g_con, J_con*3600, SM_con);
fprintf('  Current payload weight        = %.0f g   (J = %.4f $/hr,  SM = %.1f%%)\n', Wp_g, J_physics*3600, dynOut.SM_pct);
fprintf('  Stall speed at optimum        = %.2f m/s  (design limit = %.2f m/s)\n', Vs_con, V_stall_mps);
if ~isnan(Vs_con) && Vs_con > V_stall_mps * 1.10
    fprintf('  *** Optimal Wp raises stall speed >10%% above design — check CTOL runway length ***\n');
end
fprintf('=======================================================================\n\n');

% ---- figure ----
figure('Name','Profit vs Payload Weight (Actual Physics)');
subplot(3,1,1);
plot(Wp_g_sweep, J_sweep*3600, 'b-', 'LineWidth', 2); hold on;
plot(Wp_g_con, J_con*3600, 'gs', 'MarkerSize', 9, 'MarkerFaceColor', 'g');
xline(Wp_g, 'k--', sprintf(' Current %.0fg', Wp_g), 'LabelVerticalAlignment', 'bottom');
ylabel('J  [$ / hr]'); grid on; box on;
title(sprintf('Profit vs Payload  (LD=%.1f, aircraft base=%.0fg)', LD_physics, m_no_payload_kg*1000));
legend('J sweep', sprintf('Optimum %.0fg (SM\\geq%.0f%%)', Wp_g_con, SM_min_pct), 'Current', 'Location', 'best');

subplot(3,1,2);
plot(Wp_g_sweep, SM_sweep, 'b-', 'LineWidth', 2); hold on;
patch([Wp_g_sweep(1) Wp_g_sweep(end) Wp_g_sweep(end) Wp_g_sweep(1)], ...
      [0 0 SM_min_pct SM_min_pct], 'r', 'FaceAlpha', 0.10, 'EdgeColor', 'none');
yline(SM_min_pct, 'r--', sprintf(' SM_{min}=%.0f%%', SM_min_pct));
xline(Wp_g, 'k--');
plot(Wp_g_con, SM_con, 'gs', 'MarkerSize', 9, 'MarkerFaceColor', 'g');
ylabel('Static margin [%]'); grid on; box on;

subplot(3,1,3);
plot(Wp_g_sweep, Vs_sweep, 'b-', 'LineWidth', 2); hold on;
yline(V_stall_mps, 'r--', sprintf(' Design V_{stall}=%.1f m/s', V_stall_mps));
xline(Wp_g, 'k--');
plot(Wp_g_con, Vs_con, 'gs', 'MarkerSize', 9, 'MarkerFaceColor', 'g');
xlabel('Payload weight  W_p  [g]');
ylabel('V_{stall}  [m/s]'); grid on; box on;

%% =============== Manufacturing Dimension Sheet ==============

outDir = fullfile(repoRoot, 'outputs');
if ~exist(outDir, 'dir'), mkdir(outDir); end
mfgFile = fullfile(outDir, 'manufacturing_dimensions.txt');

fid = fopen(mfgFile, 'w');
fprintf(fid, '=================================================================\n');
fprintf(fid, '  MAE 155B Group 2 — Manufacturing Dimension Sheet\n');
fprintf(fid, '  Generated: %s\n', string(timestamp));
fprintf(fid, '=================================================================\n\n');

% ---- Wing planform ----
fprintf(fid, '--- WING PLANFORM ---\n');
fprintf(fid, '  Full span          b        = %.4f m  (%.2f in)\n', wingOut.b_m, wingOut.b_m/0.0254);
fprintf(fid, '  Semispan           b/2      = %.4f m  (%.2f in)\n', wingOut.semiSpan_m, wingOut.semiSpan_m/0.0254);
fprintf(fid, '  Root chord         c_root   = %.4f m  (%.2f in)\n', wingOut.c_root_m, wingOut.c_root_m/0.0254);
fprintf(fid, '  Tip chord          c_tip    = %.4f m  (%.2f in)\n', wingOut.c_tip_m, wingOut.c_tip_m/0.0254);
fprintf(fid, '  MAC                MAC      = %.4f m  (%.2f in)\n', wingOut.MAC_m, wingOut.MAC_m/0.0254);
fprintf(fid, '  MAC span station   y_MAC    = %.4f m  (%.2f in) from root\n', wingOut.y_MAC_m, wingOut.y_MAC_m/0.0254);
fprintf(fid, '  Aspect ratio       AR       = %.3f\n', wingIn.AR);
fprintf(fid, '  Taper ratio        lambda   = %.3f\n', wingIn.taper);
fprintf(fid, '  Quarter-chord sweep         = %.2f deg\n', wingOut.sweep_c4_deg);
fprintf(fid, '  Leading-edge sweep          = %.2f deg\n', wingOut.sweep_LE_deg);
fprintf(fid, '  Wing root LE x (from nose)  = %.4f m  (%.2f in)\n', wingIn.xLE_root_m, wingIn.xLE_root_m/0.0254);
fprintf(fid, '  Wing root y (from CL)       = %.4f m  (%.2f in)\n', wingIn.y_root_m, wingIn.y_root_m/0.0254);
fprintf(fid, '\n');

% ---- Elevon ----
eta_elev_mid  = 0.5 * (wingIn.eta_cs_start + wingIn.eta_cs_end);
c_at_mid      = wingOut.c_root_m + (wingOut.c_tip_m - wingOut.c_root_m) * eta_elev_mid;
c_cs_mid      = wingIn.cs_chord_frac * c_at_mid;
y_cs_start_m  = wingIn.y_root_m + wingIn.eta_cs_start * wingOut.semiSpan_m;
y_cs_end_m    = wingIn.y_root_m + wingIn.eta_cs_end   * wingOut.semiSpan_m;
b_cs_m        = y_cs_end_m - y_cs_start_m;

fprintf(fid, '--- ELEVON (each side) ---\n');
fprintf(fid, '  Chord fraction              = %.3f  (%.1f%% of local chord)\n', wingIn.cs_chord_frac, wingIn.cs_chord_frac*100);
fprintf(fid, '  Inboard  eta / y            = %.3f  /  %.4f m  (%.2f in)\n', wingIn.eta_cs_start, y_cs_start_m, y_cs_start_m/0.0254);
fprintf(fid, '  Outboard eta / y            = %.3f  /  %.4f m  (%.2f in)\n', wingIn.eta_cs_end,   y_cs_end_m,   y_cs_end_m/0.0254);
fprintf(fid, '  Elevon span (each side)     = %.4f m  (%.2f in)\n', b_cs_m, b_cs_m/0.0254);
fprintf(fid, '  Chord at midspan            = %.4f m  (%.2f in)\n', c_cs_mid, c_cs_mid/0.0254);
fprintf(fid, '\n');

% ---- Vertical fin ----
fprintf(fid, '--- VERTICAL FIN (each fin) ---\n');
fprintf(fid, '  Span / height      b_v      = %.4f m  (%.2f in)\n', vertOut.b_v_m, vertOut.b_v_m/0.0254);
fprintf(fid, '  Root chord         c_root   = %.4f m  (%.2f in)\n', vertOut.c_root_v_m, vertOut.c_root_v_m/0.0254);
fprintf(fid, '  Tip chord          c_tip    = %.4f m  (%.2f in)\n', vertOut.c_tip_v_m, vertOut.c_tip_v_m/0.0254);
fprintf(fid, '  MAC                MAC_v    = %.4f m  (%.2f in)\n', vertOut.MAC_v_m, vertOut.MAC_v_m/0.0254);
fprintf(fid, '  Aspect ratio                = %.3f\n', vertOut.AR_v);
fprintf(fid, '  Taper ratio                 = %.3f\n', vertOut.taper_v);
fprintf(fid, '  Quarter-chord sweep         = %.2f deg\n', vertOut.sweep_c4_v_deg);
fprintf(fid, '  Mount x (at wing-tip LE)    = %.4f m  (%.2f in) from nose\n', vertOut.xLE_root_v_m, vertOut.xLE_root_v_m/0.0254);
fprintf(fid, '  Mount y (wing tip)          = %.4f m  (%.2f in) from CL\n', vertOut.y_root_v_m, vertOut.y_root_v_m/0.0254);
fprintf(fid, '  Twin fins                   = %d\n', vertOut.isTwin);
fprintf(fid, '\n');

% ---- Rudder ----
fprintf(fid, '--- RUDDER (each fin) ---\n');
fprintf(fid, '  Chord fraction              = %.3f  (%.1f%%)\n', vertIn.rudder.cf_root, vertIn.rudder.cf_root*100);
fprintf(fid, '  Height span                 = %.4f m  (%.2f in)\n', vertOut.rudder.height_m, vertOut.rudder.height_m/0.0254);
fprintf(fid, '  Root chord                  = %.4f m  (%.2f in)\n', vertOut.rudder.c_root_m, vertOut.rudder.c_root_m/0.0254);
fprintf(fid, '  Tip chord                   = %.4f m  (%.2f in)\n', vertOut.rudder.c_tip_m, vertOut.rudder.c_tip_m/0.0254);
fprintf(fid, '  Area (single fin)           = %.4f m^2  (%.2f in^2)\n', vertOut.rudder.S_rudder_m2, vertOut.rudder.S_rudder_m2/0.0254^2);
fprintf(fid, '\n');

% ---- Mass and CG ----
fprintf(fid, '--- MASS AND CG ---\n');
fprintf(fid, '  Total mass (loaded)         = %.4f kg  (%.3f lb)\n', massOut.mass_kg, massOut.mass_kg*2.20462);
fprintf(fid, '  Total mass (unloaded)       = %.4f kg  (%.3f lb)\n', massOut_unloaded.mass_kg, massOut_unloaded.mass_kg*2.20462);
fprintf(fid, '  CG loaded   x (from nose)   = %.4f m  (%.2f in)\n', massOut.cg_m(1), massOut.cg_m(1)/0.0254);
fprintf(fid, '  CG loaded   y (from CL)     = %.4f m  (%.2f in)\n', massOut.cg_m(2), massOut.cg_m(2)/0.0254);
fprintf(fid, '  CG loaded   %% MAC           = %.2f %%\n', cg_percent_MAC);
fprintf(fid, '  CG unloaded x (from nose)   = %.4f m  (%.2f in)\n', massOut_unloaded.cg_m(1), massOut_unloaded.cg_m(1)/0.0254);
fprintf(fid, '\n');

% ---- Performance ----
fprintf(fid, '--- PERFORMANCE ---\n');
fprintf(fid, '  Design cruise speed         = %.3f m/s  (%.1f mph)\n', V_cruise, V_cruise*2.23694);
fprintf(fid, '  Stall speed (loaded)        = %.3f m/s  (%.1f mph)\n', V_stall_actual, V_stall_actual*2.23694);
if perfOut.validCruise
    fprintf(fid, '  Solved cruise speed         = %.3f m/s  (%.1f mph)\n', perfOut.Vcruise_mps, perfOut.Vcruise_mps*2.23694);
end
fprintf(fid, '  Wing loading  W/S           = %.2f N/m^2\n', massOut.weight_N / wingOut.S_ref_m2);
fprintf(fid, '  Min turn radius (AVL)       = %.2f m\n', csOut.R_min_m);
fprintf(fid, '  Max bank angle              = %.1f deg\n', csOut.phi_max_deg);
fprintf(fid, '  Steady-state roll rate      = %.1f deg/s\n', csOut.p_ss_dps);
fprintf(fid, '\n');

% ---- Stability ----
fprintf(fid, '--- STABILITY ---\n');
fprintf(fid, '  Static margin (AVL)         = %.2f %%\n', dynOut.SM_pct);
fprintf(fid, '  Short period: wn = %.3f rad/s,  zeta = %.3f\n', ...
    dynOut.longModes.shortPeriod.metrics.wn, dynOut.longModes.shortPeriod.metrics.zeta);
fprintf(fid, '  Phugoid:      wn = %.3f rad/s,  zeta = %.3f\n', ...
    dynOut.longModes.phugoid.metrics.wn, dynOut.longModes.phugoid.metrics.zeta);
fprintf(fid, '  Dutch roll:   wn = %.3f rad/s,  zeta = %.3f\n', ...
    dynOut.latModes.dutchRoll.metrics.wn, dynOut.latModes.dutchRoll.metrics.zeta);
fprintf(fid, '  Trim elevon deflection      = %.2f deg\n', csOut.delta_e_trim_deg);
fprintf(fid, '\n');

% ---- Servo check ----
fprintf(fid, '--- SERVO CHECK  (SG90: 1.8 kg*cm = 0.177 N*m at 4.8V) ---\n');
if ~isnan(csOut.HM_elevon_Nm)
    if csOut.elevon_servo_ok
        fprintf(fid, '  Elevon HM (one side) = %.4f N*m  OK  (%.0f%% of capacity)\n', ...
            csOut.HM_elevon_Nm, 100*csOut.HM_elevon_Nm/csOut.T_sg90_Nm);
    else
        fprintf(fid, '  Elevon HM (one side) = %.4f N*m  *** UNDERSIZED (%.0f%% over limit) ***\n', ...
            csOut.HM_elevon_Nm, 100*(csOut.HM_elevon_Nm/csOut.T_sg90_Nm - 1));
    end
    if csOut.rudder_servo_ok
        fprintf(fid, '  Rudder HM (one fin)  = %.4f N*m  OK  (%.0f%% of capacity)\n', ...
            csOut.HM_rudder_Nm, 100*csOut.HM_rudder_Nm/csOut.T_sg90_Nm);
    else
        fprintf(fid, '  Rudder HM (one fin)  = %.4f N*m  *** UNDERSIZED (%.0f%% over limit) ***\n', ...
            csOut.HM_rudder_Nm, 100*(csOut.HM_rudder_Nm/csOut.T_sg90_Nm - 1));
    end
else
    fprintf(fid, '  Hinge moment geometry not available (run full pipeline).\n');
end
fprintf(fid, '\n');

% ---- Structure ----
fprintf(fid, '--- STRUCTURE ---\n');
fprintf(fid, '  Spar diameter (selected)    = %.0f mm\n', d_selected*1000);
fprintf(fid, '  Required spar diameter      = %.2f mm\n', d_req*1000);
fprintf(fid, '  Bending factor of safety    = %.2f\n', FoS);
fprintf(fid, '  Max wing deflection         = %.1f mm  (%.1f%% span)\n', delta_max*1000, 100*delta_max/wingOut.b_m);
fprintf(fid, '\n');

fprintf(fid, '=================================================================\n');
fclose(fid);

fprintf('\n Manufacturing dimension sheet written to:\n   %s\n\n', mfgFile);

%% =============== Full Aircraft Profit Optimization (CMA-ES) ===============
% Toggle: set runProfitOpt = true in the Run Flags block to execute.
% Expected runtime: 6-10 hr with default settings (parfor recommended).
% After completion, update main.m with the printed parameter values and
% verify SM, stall speed, and mode quality with a normal run.
% =========================================================================
if runProfitOpt

    fprintf('\n===== FULL AIRCRAFT PROFIT OPTIMIZATION =====\n');
    fprintf('  Building optimizer context from current pipeline outputs...\n\n');

    optIn = struct();

    % ---- base geometry structs (fixed fields carried into each eval) ----
    % dynIn already has AVL paths, airfoil files, control surface fractions,
    % rudder geometry, modelCenterbody flag — the objective function overwrites
    % only the geometry-dependent fields per sample.
    optIn.dynIn_base  = dynIn;
    optIn.wingIn_base = wingIn;   % carries y_root_m, z_root_m, eta_cs_*, symmetric
    optIn.vertIn_base = vertIn;   % carries isTwin, sizeMode, c_v, cant, toe, rudder

    % ---- airfoil surrogates (re-evaluated per sample at new Re) ----
    optIn.airfoilDB       = airfoilDB_cached;
    optIn.airfoilRootName = airfoilRootName;
    optIn.airfoilTipName  = airfoilTipName;

    % ---- fixed mass components (indices 1-6: motor/prop/ESC/batt/rx/payload) ----
    % WARNING: do not reorder comp(1:6) — the optimizer assumes this slice.
    optIn.cadMass        = cadMass;
    optIn.m_fuse_ref_kg  = cadMass.fuselageOnly.mass_kg;
    optIn.compFixed      = comp(1:6);
    optIn.eta_servo      = eta_servo;

    % ---- mission and sizing parameters ----
    optIn.Wp_N           = Wp;
    optIn.Wprop_N        = Wprop;
    optIn.Vp_ref_m3      = Vp_ref;
    optIn.fe_base        = fe;
    optIn.ke             = ke;
    optIn.fe_max         = fe_max;
    optIn.eta_p          = eta_p;
    optIn.R_cruise_m     = R_cruise;
    optIn.delta_h_m      = delta_h;
    optIn.reserve_factor = reserve_factor;
    optIn.Tf_s           = Tf_measured;   % un-scaled; x20 applied inside optimizer
    optIn.roh            = roh;
    optIn.mu_Pas         = 1.789e-5;
    optIn.g              = g;

    % ---- drag build-up: fuselage Swet scales with Vp^(2/3) ----
    optIn.Swet_fuse_m2 = Swet_fuse;   % [m²] reference at current Vp
    optIn.Vp_m3_base   = Vp;          % [m³] reference Vp for Swet_fuse scaling
    optIn.Lf_m = Lf;   optIn.Wf_m = Wf;   optIn.Hf_m = Hf;   % form factor dims stay fixed
    optIn.tc = tc;     optIn.xc = xc;
    optIn.Q_wing = Q_wing;   optIn.Q_fuse = Q_fuse;   optIn.Q_fin = Q_fin;

    % ---- fuselage airfoil for cargo bay geometry ----
    optIn.fuselageAirfoil = fuselageAirfoil;
    optIn.cargoAirfoilFile = cargoAirfoilFile;
    optIn.cargoBayVolume_m3 = cargoOut.volume_m3;   % [m³] max available from airfoil
    optIn.cargoWidth_m = cargoOut.width_m;
    optIn.cargoHeight_m = cargoOut.height_m;

    % ---- structural mass references for scaling ----
    % Wing mass scales as S_ref * sqrt(AR) relative to these baseline values.
    % Fin mass scales linearly with total fin wetted area.
    optIn.m_wing_struct_ref_kg = m_wing_struct_kg;
    optIn.m_vert_struct_ref_kg = m_vert_struct_kg;
    optIn.S_ref_base_m2        = S_ref;
    optIn.AR_base              = AR;
    optIn.S_fin_base_m2        = vertOut.S_v_total_m2;

    % ---- physical constraints ----
    optIn.SM_min_pct      = 5.0;    % [%]   static margin lower bound
    optIn.SM_max_pct      = 13.0;   % [%]   static margin upper bound
    optIn.Vs_max_mps      = 12.0;   % [m/s] stall speed limit
    optIn.b_max_m         = 2.5;    % [m]   wingspan limit (current design ~2.0-2.3 m)
    optIn.c_tip_min_m     = 0.05;   % [m]   minimum buildable tip chord
    optIn.b_v_max_frac    = 0.50;   % [-]   fin height / semispan limit
    optIn.de_trim_max_deg = 15.0;   % [deg] max trim elevon deflection
    optIn.Vs_margin_fac   = 1.30;   % [-]   V_cruise / Vs minimum ratio
    optIn.Wg_max_N        = 60.0;   % [N]   gross weight hard cap

    % ---- initial guess: current design values ----
    optIn.x0 = [AR; wingTapper; wingSweep; twistOut.twist_root_deg; WS_design; ...
                wingIn.xLE_root_m; vertIn.AR_v; vertIn.taper_v; vertIn.sweep_c4_v_deg; ...
                V_cruise; Vp; wingIn.y_root_m; Lf];

    % ---- CMA-ES settings ----
    % lambda=0 uses 2x Hansen default (≈26 for n=13).
    % For shorter test runs, reduce maxGen (e.g. 50 for a ~20-min smoke test).
    optIn.sigma0   = 0.15;   % initial step in normalized [0,1] space
    optIn.maxGen   = 200;    % increase to 500 for thorough run
    optIn.lambda   = 0;      % 2× Hansen default ≈ 22 for n=11
    optIn.verbose  = 10;
    optIn.debugObj = false;

    optProfOut = profitOptimization(optIn);

    % Save result to outputs/ for later reference
    optSaveFile = fullfile(repoRoot, 'outputs', 'profit_opt_result.mat');
    save(optSaveFile, 'optProfOut', 'optIn');
    fprintf('  Optimization result saved to:\n   %s\n\n', optSaveFile);

end
