% 155B Group 2 Optimization Script
% 2D sweep of payload weight and volumetric package scalar
% Includes:
%   1) aerodynamic penalty through effective L/D
%   2) empty-weight penalty through effective empty-weight fraction

clc; clearvars; close all;

%% Printing set-up
timestamp = datetime('now','Format','yyyy-MM-dd HH:mm:ss');
fprintf('========= Optimization Script executed at: %s =======\n\n', string(timestamp));

%% ================ Given ===============
g = 9.81;             % [m/s^2]
Wprop = 2.43341;      % [N] Total propulsion system weight
ft3_to_m3 = 0.02831685;

%% ============== Engineering Assumptions ========
eta_p = 0.75;          % [-] Propulsion efficiency
LD = 20;               % flying wings avg 15-30
reserve_factor = 1.15; % [-] Energy margin multiplier

% Baseline empty-weight fraction
fe = 0.5;             % [-] baseline empty weight fraction = We / Wg

% Aerodynamic penalty model for payload volume
VPS_ref = 0.4;         % [-] scalar below which no penalty is applied
kd = 0.5;              % [-] L/D penalty strength for VPS > VPS_ref

% Empty-weight penalty model for payload volume
ke = 0.08;             % [-] empty-weight-fraction penalty slope
fe_max = 0.85;         % [-] hard upper cap for sanity

%% ============== Mission Parameters ===========
delta_h = 120;         % [m] Climb altitude change
R_cruise = 18000;      % [m] Cruise range
Tf_measured = 61;      % [s] Measured flight time

%% ============== Sweep Variables ===========
Wp_g_vec = linspace(227, 1000, 60);   % [g]
VPS_vec  = linspace(0.1, 1.1, 60);    % [-]

%% ============== Preallocate ===========
J_map            = zeros(length(Wp_g_vec), length(VPS_vec));
Wg_map_g         = zeros(length(Wp_g_vec), length(VPS_vec));
We_map_g         = zeros(length(Wp_g_vec), length(VPS_vec));
Vp_map           = zeros(length(Wp_g_vec), length(VPS_vec));
LD_eff_map       = zeros(length(Wp_g_vec), length(VPS_vec));
fe_eff_map       = zeros(length(Wp_g_vec), length(VPS_vec));
Ef_measured_map  = zeros(length(Wp_g_vec), length(VPS_vec));
Ef_map           = zeros(length(Wp_g_vec), length(VPS_vec));

%% ============== Sweep ===========
for i = 1:length(Wp_g_vec)
    for j = 1:length(VPS_vec)

        % Payload inputs
        Wp_g = Wp_g_vec(i);
        Wp   = (Wp_g/1000)*g;           % [N]

        VPS  = VPS_vec(j);
        Vp   = ft3_to_m3 * VPS;         % [m^3]

        % Aerodynamic penalty:
        % no penalty up to VPS_ref, then L/D decreases for larger packages
        if VPS <= VPS_ref
            LD_eff = LD;
        else
            LD_eff = LD / (1 + kd*(VPS - VPS_ref));
        end

        % Empty-weight penalty:
        % no penalty up to VPS_ref, then empty-weight fraction increases
        if VPS <= VPS_ref
            fe_eff = fe;
        else
            fe_eff = fe + ke*(VPS - VPS_ref);
        end

        % Cap empty-weight fraction for sanity
        fe_eff = min(fe_eff, fe_max);

        % Derived weights using penalized empty-weight fraction
        Wg = (Wp + Wprop) / (1 - fe_eff);   % [N]
        We = fe_eff * Wg;                   % [N]

        % Energy model
        [~, ~, E_f, ~, ~, ~] = energyCalc(Wg, eta_p, LD_eff, delta_h, R_cruise, reserve_factor);
        Ef_measured = E_f;                  % [J]

        % Competition scaling
        Ef = 20 * Ef_measured;
        Tf = 20 * Tf_measured;

        % Score
        J = profitPerUnitTime(Wp, Vp, Ef, Wg, Tf);

        % Store results
        J_map(i,j)           = J;
        Wg_map_g(i,j)        = Wg/g*1000;   % [g]
        We_map_g(i,j)        = We/g*1000;   % [g]
        Vp_map(i,j)          = Vp;
        LD_eff_map(i,j)      = LD_eff;
        fe_eff_map(i,j)      = fe_eff;
        Ef_measured_map(i,j) = Ef_measured;
        Ef_map(i,j)          = Ef;
    end
end

%% ============== Find Best Point ===========
[J_best, idx_best] = max(J_map(:));
[row_best, col_best] = ind2sub(size(J_map), idx_best);

Wp_g_best = Wp_g_vec(row_best);
VPS_best  = VPS_vec(col_best);
Vp_best   = Vp_map(row_best, col_best);
Wg_best_g = Wg_map_g(row_best, col_best);
We_best_g = We_map_g(row_best, col_best);
LD_eff_best = LD_eff_map(row_best, col_best);
fe_eff_best = fe_eff_map(row_best, col_best);
Ef_best   = Ef_map(row_best, col_best);
Ef_measured_best = Ef_measured_map(row_best, col_best);
Tf_best   = 20 * Tf_measured;

%% ============== Plots ===========

% 3D surface of score
figure;
surf(VPS_vec, Wp_g_vec, J_map*3600);
xlabel('Volumetric Package Scalar VPS [-]');
ylabel('Payload Weight W_p [g]');
zlabel('J [$ / hr]');
title('Score Surface: J vs Payload Weight and Package Scalar');
grid on;
shading interp;
colorbar;

% Score contour
figure;
contourf(VPS_vec, Wp_g_vec, J_map*3600, 25, 'LineColor', 'none');
xlabel('Volumetric Package Scalar VPS [-]');
ylabel('Payload Weight W_p [g]');
title('Score Contours: J [$ / hr]');
grid on;
colorbar;

% Gross weight contour
figure;
contourf(VPS_vec, Wp_g_vec, Wg_map_g, 25, 'LineColor', 'none');
xlabel('Volumetric Package Scalar VPS [-]');
ylabel('Payload Weight W_p [g]');
title('Derived Gross Weight W_g [g]');
grid on;
colorbar;

% Empty-weight fraction contour
figure;
contourf(VPS_vec, Wp_g_vec, fe_eff_map, 25, 'LineColor', 'none');
xlabel('Volumetric Package Scalar VPS [-]');
ylabel('Payload Weight W_p [g]');
title('Effective Empty Weight Fraction f_{e,eff}');
grid on;
colorbar;

% Effective L/D contour
figure;
contourf(VPS_vec, Wp_g_vec, LD_eff_map, 25, 'LineColor', 'none');
xlabel('Volumetric Package Scalar VPS [-]');
ylabel('Payload Weight W_p [g]');
title('Effective Lift-to-Drag Ratio (L/D)_{eff}');
grid on;
colorbar;

% Energy contour
figure;
contourf(VPS_vec, Wp_g_vec, Ef_measured_map/3600, 25, 'LineColor', 'none');
xlabel('Volumetric Package Scalar VPS [-]');
ylabel('Payload Weight W_p [g]');
title('Measured Mission Energy E_f [Wh]');
grid on;
colorbar;

%% ============== Print Summary ===========
fprintf('================ OPTIMIZATION SUMMARY ====================\n');
fprintf('Sweep Variable 1: Payload Weight      = 227 g to 1000 g\n');
fprintf('Sweep Variable 2: Volume Scalar VPS   = 0.1 to 1.1\n');
fprintf('Fixed Flight Time (measured)          = %.2f s\n', Tf_measured);
fprintf('Baseline Empty Weight Fraction        = %.4f\n', fe);
fprintf('Fixed Propulsion Weight               = %.4f N\n', Wprop);
fprintf('Baseline L/D                          = %.4f\n', LD);
fprintf('Reference VPS for penalties           = %.4f\n', VPS_ref);
fprintf('Aerodynamic penalty factor kd         = %.4f\n', kd);
fprintf('Empty-weight penalty factor ke        = %.4f\n', ke);
fprintf('Maximum empty-weight fraction         = %.4f\n', fe_max);
fprintf('----------------------------------------------------------\n');
fprintf('Best Payload Weight                   = %.2f g\n', Wp_g_best);
fprintf('Best Volume Scalar VPS                = %.4f\n', VPS_best);
fprintf('Best Payload Volume                   = %.6f m^3\n', Vp_best);
fprintf('Best Effective L/D                    = %.4f\n', LD_eff_best);
fprintf('Best Effective Empty Weight Fraction  = %.4f\n', fe_eff_best);
fprintf('Best Derived Gross Weight             = %.2f g\n', Wg_best_g);
fprintf('Best Derived Empty Weight             = %.2f g\n', We_best_g);
fprintf('Best Measured Mission Energy          = %.2f J\n', Ef_measured_best);
fprintf('Best Scaled Energy                    = %.2f J\n', Ef_best);
fprintf('Best Scaled Flight Time               = %.2f s\n', Tf_best);
fprintf('----------------------------------------------------------\n');
fprintf('Best J                                = %.8f $/s\n', J_best);
fprintf('Best J                                = %.4f $/hr\n', J_best*3600);
fprintf('==========================================================\n');
%% ============== Local Sensitivity Analysis ===========
fprintf('\n================ LOCAL SENSITIVITY ANALYSIS ================\n');

% Baseline design point
Wp_g0 = 680;      % [g]
VPS0  = 2/3;      % [-]

% Small perturbations (1%)
dWp_g = 0.01 * Wp_g0;
dVPS  = 0.01 * VPS0;

% ---- Helper calculation for baseline ----
Wp0 = (Wp_g0/1000)*g;
Vp0 = ft3_to_m3 * VPS0;

if VPS0 <= VPS_ref
    LD_eff0 = LD;
else
    LD_eff0 = LD / (1 + kd*(VPS0 - VPS_ref));
end

if VPS0 <= VPS_ref
    fe_eff0 = fe;
else
    fe_eff0 = fe + ke*(VPS0 - VPS_ref);
end

fe_eff0 = min(fe_eff0, fe_max);

Wg0 = (Wp0 + Wprop) / (1 - fe_eff0);
We0 = fe_eff0 * Wg0;

[~, ~, E_f0, ~, ~, ~] = energyCalc(Wg0, eta_p, LD_eff0, delta_h, R_cruise, reserve_factor);
Ef_measured0 = E_f0;
Ef0 = 20 * Ef_measured0;
Tf0 = 20 * Tf_measured;

J0 = profitPerUnitTime(Wp0, Vp0, Ef0, Wg0, Tf0);

% ---- Perturb Wp_g upward/downward ----
Wp_g_p = Wp_g0 + dWp_g;
Wp_p   = (Wp_g_p/1000)*g;
Vp_p   = ft3_to_m3 * VPS0;

% same VPS, so same LD_eff and fe_eff logic
LD_eff_p = LD_eff0;
fe_eff_p = fe_eff0;

Wg_p = (Wp_p + Wprop) / (1 - fe_eff_p);
[~, ~, E_f_p, ~, ~, ~] = energyCalc(Wg_p, eta_p, LD_eff_p, delta_h, R_cruise, reserve_factor);
Ef_p = 20 * E_f_p;
J_p_Wp = profitPerUnitTime(Wp_p, Vp_p, Ef_p, Wg_p, Tf0);

Wp_g_m = Wp_g0 - dWp_g;
Wp_m   = (Wp_g_m/1000)*g;
Vp_m   = ft3_to_m3 * VPS0;

Wg_m = (Wp_m + Wprop) / (1 - fe_eff_p);
[~, ~, E_f_m, ~, ~, ~] = energyCalc(Wg_m, eta_p, LD_eff_p, delta_h, R_cruise, reserve_factor);
Ef_m = 20 * E_f_m;
J_m_Wp = profitPerUnitTime(Wp_m, Vp_m, Ef_m, Wg_m, Tf0);

dJdWp_g = (J_p_Wp - J_m_Wp) / (2*dWp_g);

% ---- Perturb VPS upward/downward ----
VPS_p = VPS0 + dVPS;
Wp_p2 = Wp0;
Vp_p2 = ft3_to_m3 * VPS_p;

if VPS_p <= VPS_ref
    LD_eff_p2 = LD;
else
    LD_eff_p2 = LD / (1 + kd*(VPS_p - VPS_ref));
end

if VPS_p <= VPS_ref
    fe_eff_p2 = fe;
else
    fe_eff_p2 = fe + ke*(VPS_p - VPS_ref);
end

fe_eff_p2 = min(fe_eff_p2, fe_max);

Wg_p2 = (Wp_p2 + Wprop) / (1 - fe_eff_p2);
[~, ~, E_f_p2, ~, ~, ~] = energyCalc(Wg_p2, eta_p, LD_eff_p2, delta_h, R_cruise, reserve_factor);
Ef_p2 = 20 * E_f_p2;
J_p_VPS = profitPerUnitTime(Wp_p2, Vp_p2, Ef_p2, Wg_p2, Tf0);

VPS_m = VPS0 - dVPS;
Wp_m2 = Wp0;
Vp_m2 = ft3_to_m3 * VPS_m;

if VPS_m <= VPS_ref
    LD_eff_m2 = LD;
else
    LD_eff_m2 = LD / (1 + kd*(VPS_m - VPS_ref));
end

if VPS_m <= VPS_ref
    fe_eff_m2 = fe;
else
    fe_eff_m2 = fe + ke*(VPS_m - VPS_ref);
end

fe_eff_m2 = min(fe_eff_m2, fe_max);

Wg_m2 = (Wp_m2 + Wprop) / (1 - fe_eff_m2);
[~, ~, E_f_m2, ~, ~, ~] = energyCalc(Wg_m2, eta_p, LD_eff_m2, delta_h, R_cruise, reserve_factor);
Ef_m2 = 20 * E_f_m2;
J_m_VPS = profitPerUnitTime(Wp_m2, Vp_m2, Ef_m2, Wg_m2, Tf0);

dJdVPS = (J_p_VPS - J_m_VPS) / (2*dVPS);

% ---- Normalized sensitivities ----
S_Wp_g = dJdWp_g * (Wp_g0 / J0);
S_VPS  = dJdVPS  * (VPS0  / J0);

sensNames = {'Payload Weight W_p [g]', 'Volume Scalar VPS [-]'};
sensVals  = [S_Wp_g, S_VPS];

[~, idx] = sort(abs(sensVals), 'descend');

% ---- Print results ----
fprintf('Baseline J = %.8f $/s\n', J0);
fprintf('Baseline J = %.4f $/hr\n\n', J0*3600);

fprintf('Normalized sensitivities at baseline:\n');
fprintf('Payload Weight Wp_g : %.6f\n', S_Wp_g);
fprintf('Volume Scalar VPS   : %.6f\n', S_VPS);

fprintf('\nRanking by absolute importance:\n');
for k = 1:length(idx)
    fprintf('%d) %s  (%.6f)\n', k, sensNames{idx(k)}, sensVals(idx(k)));
end

% ---- Bar chart ----
figure;
bar(sensVals(idx));
grid on;
set(gca, 'XTickLabel', sensNames(idx), 'FontSize', 11);
ylabel('Normalized Sensitivity');
title('Design Variable Importance on J(x)');