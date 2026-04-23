function sizing = preliminarySizingCTOL(inp)
% preliminarySizingCTOL
%
% Purpose:
%   Preliminary wing-loading / thrust-loading sizing for a CTOL prop aircraft.
%
% Active constraints in this version:
%   1) Stall
%   2) Climb
%   3) Maneuver
%   4) Takeoff (optional)
%   5) Ceiling (optional)
%
% Notes:
%   - Sizing is carried out in T/W form.
%   - Propulsion data are used only to report available thrust at key speeds.
%   - Takeoff constraint uses the lecture-summary form:
%         T/W >= (W/S) / (TOP * sigma * CL_takeoff)
%     where sigma = rho_takeoff / rho_sl and CL_takeoff = CLmax / 1.21

%% -------------------------
% Unpack inputs
% -------------------------
rho_sl = inp.rho_sl;
W0_N   = inp.W0_N;

AR    = inp.AR;
e     = inp.e;
CD0   = inp.CD0;
CLmax = inp.CLmax;

V_stall_mps = inp.V_stall_mps;

V_climb_mps = inp.V_climb_mps;
G_climb     = inp.G_climb;

V_turn_mps  = inp.V_turn_mps;
n_maneuver  = inp.n_maneuver;

use_takeoff = inp.use_takeoff;
if use_takeoff
    rho_takeoff = inp.rho_takeoff;
    TOP_m       = inp.TOP_m;
end

use_ceiling = inp.use_ceiling;
if use_ceiling
    V_ceiling_mps = inp.V_ceiling_mps; %#ok<NASGU>
end

WS_min = inp.WS_min;
WS_max = inp.WS_max;
Npts   = inp.Npts;

buf_WS = inp.buf_WS;
buf_TW = inp.buf_TW;

propV_vec_mps = inp.propV_vec_mps;
propT_vec_N   = inp.propT_vec_N;

%% -------------------------
% Basic input checks
% -------------------------
if WS_max <= WS_min
    error('WS_max must be greater than WS_min.');
end

if Npts < 2
    error('Npts must be at least 2.');
end

if V_stall_mps <= 0 || V_climb_mps <= 0 || V_turn_mps <= 0
    error('All sizing speeds must be positive.');
end

if AR <= 0 || e <= 0 || CLmax <= 0
    error('AR, e, and CLmax must be positive.');
end

if use_takeoff
    if rho_takeoff <= 0
        error('rho_takeoff must be positive when use_takeoff = true.');
    end
    if TOP_m <= 0
        error('TOP_m must be positive when use_takeoff = true.');
    end
end

%% -------------------------
% Wing-loading grid
% -------------------------
WS_vec = linspace(WS_min, WS_max, Npts);

%% -------------------------
% Common terms
% -------------------------
k_ind = 1 / (pi * e * AR);

q_climb = 0.5 * rho_sl * V_climb_mps^2;
q_turn  = 0.5 * rho_sl * V_turn_mps^2;

%% -------------------------
% 1) Stall constraint
% -------------------------
WS_stall_max = 0.5 * rho_sl * V_stall_mps^2 * CLmax;

%% -------------------------
% 2) Climb constraint
% -------------------------
TW_climb = (CD0 .* q_climb ./ WS_vec) + ...
           (k_ind .* WS_vec ./ q_climb) + ...
           G_climb;

%% -------------------------
% 3) Maneuver constraint
% -------------------------
TW_maneuver = (CD0 .* q_turn ./ WS_vec) + ...
              ((n_maneuver^2) .* k_ind .* WS_vec ./ q_turn);

%% -------------------------
% 4) Takeoff constraint (optional)
% -------------------------
if use_takeoff
    sigma_takeoff = rho_takeoff / rho_sl;    % [-]
    CL_takeoff    = CLmax / 1.21;            % [-]
    TW_takeoff    = WS_vec ./ (TOP_m .* sigma_takeoff .* CL_takeoff);
else
    sigma_takeoff = NaN;
    CL_takeoff    = NaN;
    TW_takeoff    = zeros(size(WS_vec));
end

%% -------------------------
% 5) Ceiling constraint (optional)
% -------------------------
if use_ceiling
    LD_max = 0.5 * sqrt((pi * e * AR) / CD0);
    TW_ceiling = 1 / LD_max;
else
    LD_max     = NaN;
    TW_ceiling = zeros(size(WS_vec));
end

%% -------------------------
% Required envelope for design-point selection
% -------------------------
TW_stack = [TW_climb;
            TW_maneuver];

if use_takeoff
    TW_stack = [TW_stack; TW_takeoff];
end

if use_ceiling
    TW_stack = [TW_stack; TW_ceiling * ones(size(WS_vec))];
end

TW_req = max(TW_stack, [], 1);

%% -------------------------
% Feasible region from stall
% -------------------------
feasible = WS_vec <= WS_stall_max;

WS_feas     = WS_vec(feasible);
TW_req_feas = TW_req(feasible);

if isempty(WS_feas)
    error('No feasible region found: stall constraint lies below WS_min.');
end

%% -------------------------
% Best point and buffered design point
% -------------------------
WS_best = WS_feas(end);
TW_best = TW_req_feas(end);

WS_design = WS_best * (1 - buf_WS);
WS_design = max(WS_design, WS_min);
WS_design = min(WS_design, WS_best);

TW_req_design = interp1(WS_vec, TW_req, WS_design, 'linear', 'extrap');
TW_design     = TW_req_design * (1 + buf_TW);

T_design_N = TW_design * W0_N;

%% -------------------------
% Available propulsion at key speeds
% -------------------------
T_avail_climb_N = interp1(propV_vec_mps, propT_vec_N, V_climb_mps, 'linear', 'extrap');
T_avail_turn_N  = interp1(propV_vec_mps, propT_vec_N, V_turn_mps,  'linear', 'extrap');

TW_avail_climb = T_avail_climb_N / W0_N;
TW_avail_turn  = T_avail_turn_N  / W0_N;

%% -------------------------
% Unit conversions
% -------------------------
Nm2_per_lbf_ft2 = 47.88025898;

WS_stall_imp  = WS_stall_max / Nm2_per_lbf_ft2;
WS_best_imp   = WS_best      / Nm2_per_lbf_ft2;
WS_design_imp = WS_design    / Nm2_per_lbf_ft2;

%% -------------------------
% Store outputs
% -------------------------
sizing = struct();

sizing.WS_vec       = WS_vec;
sizing.TW_climb     = TW_climb;
sizing.TW_maneuver  = TW_maneuver;
sizing.TW_takeoff   = TW_takeoff;
sizing.TW_ceiling   = TW_ceiling;
sizing.TW_req       = TW_req;

sizing.WS_stall_max = WS_stall_max;
sizing.WS_best      = WS_best;
sizing.TW_best      = TW_best;

sizing.WS_design    = WS_design;
sizing.TW_design    = TW_design;
sizing.T_design_N   = T_design_N;

sizing.T_avail_climb_N = T_avail_climb_N;
sizing.T_avail_turn_N  = T_avail_turn_N;
sizing.TW_avail_climb  = TW_avail_climb;
sizing.TW_avail_turn   = TW_avail_turn;

sizing.use_takeoff   = use_takeoff;
sizing.sigma_takeoff = sigma_takeoff;
sizing.CL_takeoff    = CL_takeoff;

sizing.use_ceiling = use_ceiling;
sizing.LD_max      = LD_max;

%% -------------------------
% Plot
% -------------------------
fig = figure('Name','CTOL Preliminary Sizing Constraint Plot','Color','w');
ax = axes(fig); hold(ax,'on'); box(ax,'on'); grid(ax,'on');

h = gobjects(0);
leg = {};

h(end+1) = plot(ax, WS_vec, TW_climb, 'LineWidth', 2);
leg{end+1} = 'Climb';

h(end+1) = plot(ax, WS_vec, TW_maneuver, 'LineWidth', 2);
leg{end+1} = sprintf('Maneuver (n = %.2f)', n_maneuver);

if use_takeoff
    h(end+1) = plot(ax, WS_vec, TW_takeoff, 'LineWidth', 2);
    leg{end+1} = 'Takeoff';
end

h(end+1) = xline(ax, WS_stall_max, '--', 'LineWidth', 2);
leg{end+1} = 'Stall limit';

if use_ceiling
    h(end+1) = yline(ax, TW_ceiling, '-.', 'LineWidth', 2);
    leg{end+1} = 'Ceiling';
end

h(end+1) = plot(ax, WS_best, TW_best, 'o', 'MarkerSize', 8, 'LineWidth', 2);
leg{end+1} = 'Best point';

h(end+1) = plot(ax, WS_design, TW_design, 's', 'MarkerSize', 8, 'LineWidth', 2);
leg{end+1} = 'Buffered design point';

xlabel(ax, 'Wing Loading W/S [N/m^2]');
ylabel(ax, 'Thrust Loading T/W [-]');
title(ax, 'CTOL Prop Aircraft Preliminary Sizing');

legend(ax, h, leg, 'Location', 'best');

txt = sprintf([ ...
    'Design point:\n' ...
    'W/S = %.1f N/m^2 | %.3f lbf/ft^2\n' ...
    'T/W = %.4f\n' ...
    'Treq = %.3f N\n' ...
    'Tavail@climb = %.3f N\n' ...
    'Tavail@turn  = %.3f N'], ...
    WS_design, WS_design_imp, TW_design, T_design_N, ...
    T_avail_climb_N, T_avail_turn_N);

annotation(fig, 'textbox', [0.54 0.12 0.32 0.20], ...
    'String', txt, ...
    'FitBoxToText', 'on', ...
    'BackgroundColor', 'w', ...
    'EdgeColor', 'k', ...
    'FontName', 'Consolas', ...
    'FontSize', 10);

sizing.fig = fig;

%% -------------------------
% Command window summary
% -------------------------
fprintf('\n============================================================\n');
fprintf('CTOL PRELIMINARY SIZING SUMMARY (T/W FORM)\n');
fprintf('============================================================\n');
fprintf('WS_stall,max        = %8.2f N/m^2 | %8.3f lbf/ft^2\n', ...
    WS_stall_max, WS_stall_imp);
fprintf('WS_best             = %8.2f N/m^2 | %8.3f lbf/ft^2\n', ...
    WS_best, WS_best_imp);
fprintf('TW_best             = %8.5f\n', TW_best);
fprintf('WS_design           = %8.2f N/m^2 | %8.3f lbf/ft^2\n', ...
    WS_design, WS_design_imp);
fprintf('TW_design           = %8.5f\n', TW_design);
fprintf('T_design            = %8.3f N\n', T_design_N);

if use_takeoff
    fprintf('Takeoff sigma       = %8.4f\n', sigma_takeoff);
    fprintf('CL_takeoff          = %8.4f\n', CL_takeoff);
end

if use_ceiling
    fprintf('LD_max              = %8.4f\n', LD_max);
    fprintf('TW_ceiling          = %8.5f\n', TW_ceiling);
end

fprintf('T_avail @ climb     = %8.3f N  | T/W = %8.5f\n', ...
    T_avail_climb_N, TW_avail_climb);
fprintf('T_avail @ turn      = %8.3f N  | T/W = %8.5f\n', ...
    T_avail_turn_N, TW_avail_turn);
fprintf('============================================================\n\n');

end