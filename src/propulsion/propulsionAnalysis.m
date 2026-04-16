function propOut = propulsionAnalysis(propIn)
% propulsionAnalysis
%
% Purpose:
% Preliminary propulsion analysis for a single motor/prop combination.
%
% Modes:
% 1) Preliminary thrust model:
%    T/T0 = 1 - V/V0, for V <= V0
%
% 2) APC-table mode:
%    Uses PER3_10x47SF.txt only, builds interpolants for torque and thrust,
%    and solves motor-prop torque balance across V_vec_mps.
%
% Preserves the same input/output interface as the original version.

%% -------------------------
% Unpack inputs
% -------------------------
rho = propIn.rho; %#ok<NASGU>
KV = propIn.KV;
Rm = propIn.Rm;
I0 = propIn.I0;
Vbat = propIn.Vbat;

if isfield(propIn, 'I_max')
    I_max = propIn.I_max;
else
    I_max = inf;
end

propName = propIn.propName;
D_in = propIn.D_in;
pitch_in = propIn.pitch_in;
V_vec = propIn.V_vec_mps(:);

if isfield(propIn, 'usePrelimModel')
    usePrelimModel = propIn.usePrelimModel;
else
    usePrelimModel = true;
end

%% -------------------------
% Motor constants
% -------------------------
Kt = 60 / (2*pi*KV);   % [N*m/A]
Ke = Kt;               % [V*s/rad]

%% -------------------------
% Preliminary mode
% -------------------------
if usePrelimModel
    T_static_N = propIn.T_static_N;
    V0_mps = propIn.V0_mps;

    % T/T0 = 1 - V/V0 for V <= V0, else 0
    T_vec_N = T_static_N * max(0, 1 - V_vec./V0_mps);

    % Very rough rpm estimate from pitch speed idea
    pitch_m = pitch_in * 0.0254;
    n_rps_est = V0_mps / max(pitch_m, 1e-6);
    rpm_est = 60 * n_rps_est;
    omega_est = 2*pi*n_rps_est;

    % Very rough current estimate
    I_est = (Vbat - Ke*omega_est) / Rm;
    I_est = max(I_est, I0);

    I_vec_A = I_est * ones(size(V_vec));
    I_vec_A(T_vec_N <= 0) = I0;

    P_elec_vec_W = Vbat .* I_vec_A;
    P_prop_vec_W = T_vec_N .* V_vec;

    eta_vec = nan(size(V_vec));
    valid_eta = P_elec_vec_W > 0;
    eta_vec(valid_eta) = P_prop_vec_W(valid_eta) ./ P_elec_vec_W(valid_eta);

    propOut = struct();
    propOut.mode = 'preliminary';
    propOut.propName = propName;

    propOut.KV = KV;
    propOut.Rm = Rm;
    propOut.I0 = I0;
    propOut.Vbat = Vbat;
    propOut.I_max = I_max;

    propOut.D_in = D_in;
    propOut.pitch_in = pitch_in;

    propOut.V_vec_mps = V_vec;
    propOut.T_vec_N = T_vec_N;
    propOut.I_vec_A = I_vec_A;
    propOut.P_elec_vec_W = P_elec_vec_W;
    propOut.P_prop_vec_W = P_prop_vec_W;
    propOut.eta_vec = eta_vec;

    propOut.T_static_N = T_vec_N(1);
    propOut.I_static_A = I_vec_A(1);

    [propOut.T_max_N, idx_max] = max(T_vec_N);
    propOut.V_at_Tmax = V_vec(idx_max);

    fprintf('\n============================================================\n');
    fprintf('PROPULSION ANALYSIS SUMMARY (PRELIMINARY MODEL)\n');
    fprintf('============================================================\n');
    fprintf('Propeller = %s\n', string(propName));
    fprintf('Battery voltage = %.3f V\n', Vbat);
    fprintf('Motor KV = %.1f RPM/V\n', KV);
    fprintf('Motor Rm = %.4f ohm\n', Rm);
    fprintf('Motor I0 = %.3f A\n', I0);
    fprintf('Estimated static thrust = %.3f N\n', propOut.T_static_N);
    fprintf('Estimated static current = %.3f A\n', propOut.I_static_A);
    fprintf('Zero-thrust speed = %.3f m/s\n', V0_mps);
    fprintf('Current limit = %.3f A\n', I_max);
    fprintf('============================================================\n\n');

    figure('Name', sprintf('Propulsion Analysis - %s', propName), 'Color', 'w');
    plot(V_vec, T_vec_N, 'LineWidth', 2);
    grid on; box on;
    xlabel('Flight Speed V_{\infty} [m/s]');
    ylabel('Thrust T [N]');
    title(sprintf('Thrust vs Flight Speed - %s', propName), 'Interpreter', 'none');

    figure('Name', sprintf('Current Draw - %s', propName), 'Color', 'w');
    plot(V_vec, I_vec_A, 'LineWidth', 2); hold on;
    yline(I_max, '--', 'LineWidth', 1.5);
    grid on; box on;
    xlabel('Flight Speed V_{\infty} [m/s]');
    ylabel('Current I [A]');
    title(sprintf('Current vs Flight Speed - %s', propName), 'Interpreter', 'none');
    legend('Current draw', 'Current limit', 'Location', 'best');

    return;
end

%% -------------------------
% APC-table mode: PER3_10x45MR only
% -------------------------
filename = 'PER3_10x47SF.txt';
D_m = D_in * 0.0254;

txt = fileread(filename);
lines = splitlines(string(txt));

RPM_all = [];
n_all   = [];
J_all   = [];
Q_all   = [];
T_all   = [];

currentRPM = NaN;

for k = 1:length(lines)
    line = strtrim(lines(k));

    tokRPM = regexp(line, 'PROP RPM =\s*([0-9]+)', 'tokens');
    if ~isempty(tokRPM)
        currentRPM = str2double(tokRPM{1}{1});
        continue;
    end

    if isnan(currentRPM)
        continue;
    end

    nums = sscanf(line, '%f');

    % APC columns: 2 = J, 10 = Torque [N*m], 11 = Thrust [N]
    if numel(nums) >= 11
        J = nums(2);
        Q = nums(10);
        T = nums(11);

        if isfinite(J) && isfinite(Q) && isfinite(T)
            RPM_all(end+1,1) = currentRPM; %#ok<AGROW>
            n_all(end+1,1)   = currentRPM / 60; %#ok<AGROW>
            J_all(end+1,1)   = J; %#ok<AGROW>
            Q_all(end+1,1)   = Q; %#ok<AGROW>
            T_all(end+1,1)   = T; %#ok<AGROW>
        end
    end
end

F_Q = scatteredInterpolant(n_all, J_all, Q_all, 'natural', 'nearest');
F_T = scatteredInterpolant(n_all, J_all, T_all, 'natural', 'nearest');

Qm = @(omega) Kt * (((Vbat - Ke*omega)/Rm) - I0);
Im = @(omega) max((Vbat - Ke*omega)/Rm, I0);

Nv = numel(V_vec);
T_vec_N = nan(Nv,1);
I_vec_A = nan(Nv,1);
P_elec_vec_W = nan(Nv,1);
P_prop_vec_W = nan(Nv,1);
eta_vec = nan(Nv,1);
rpm_vec = nan(Nv,1);
J_vec = nan(Nv,1);
Qp_vec_Nm = nan(Nv,1);

omega_guess = 9000 * 2*pi/60;

for i = 1:Nv
    Vinf = V_vec(i);

    balanceFun = @(omega) localBalance(omega, Vinf, D_m, Qm, F_Q);

    try
        omega_sol = fzero(balanceFun, omega_guess);
        n_sol = omega_sol / (2*pi);

        if ~isfinite(n_sol) || n_sol <= 0
            continue;
        end

        J_sol = Vinf / (n_sol * D_m);
        Qp_sol = F_Q(n_sol, J_sol);
        T_sol = F_T(n_sol, J_sol);
        I_sol = Im(omega_sol);

        if ~isfinite(J_sol) || ~isfinite(Qp_sol) || ~isfinite(T_sol) || ~isfinite(I_sol)
            continue;
        end

        rpm_vec(i) = 60 * n_sol;
        J_vec(i) = J_sol;
        Qp_vec_Nm(i) = Qp_sol;
        T_vec_N(i) = T_sol;
        I_vec_A(i) = I_sol;
        P_elec_vec_W(i) = Vbat * I_sol;
        P_prop_vec_W(i) = T_sol * Vinf;

        if P_elec_vec_W(i) > 0
            eta_vec(i) = P_prop_vec_W(i) / P_elec_vec_W(i);
        end

        omega_guess = omega_sol;
    catch
        % leave NaN
    end
end

propOut = struct();
propOut.mode = 'APC_10x47SF';
propOut.propName = propName;

propOut.KV = KV;
propOut.Rm = Rm;
propOut.I0 = I0;
propOut.Vbat = Vbat;
propOut.I_max = I_max;

propOut.D_in = D_in;
propOut.pitch_in = pitch_in;

propOut.V_vec_mps = V_vec;
propOut.T_vec_N = T_vec_N;
propOut.I_vec_A = I_vec_A;
propOut.P_elec_vec_W = P_elec_vec_W;
propOut.P_prop_vec_W = P_prop_vec_W;
propOut.eta_vec = eta_vec;

propOut.rpm_vec = rpm_vec;
propOut.J_vec = J_vec;
propOut.Qp_vec_Nm = Qp_vec_Nm;

propOut.T_static_N = T_vec_N(1);
propOut.I_static_A = I_vec_A(1);

[propOut.T_max_N, idx_max] = max(T_vec_N);
propOut.V_at_Tmax = V_vec(idx_max);

fprintf('\n============================================================\n');
fprintf('PROPULSION ANALYSIS SUMMARY (APC MODE)\n');
fprintf('============================================================\n');
fprintf('Propeller = %s\n', string(propName));
fprintf('APC file = %s\n', filename);
fprintf('Battery voltage = %.3f V\n', Vbat);
fprintf('Motor KV = %.1f RPM/V\n', KV);
fprintf('Motor Rm = %.4f ohm\n', Rm);
fprintf('Motor I0 = %.3f A\n', I0);
fprintf('Computed static thrust = %.3f N\n', propOut.T_static_N);
fprintf('Computed static current = %.3f A\n', propOut.I_static_A);
fprintf('Current limit = %.3f A\n', I_max);
fprintf('============================================================\n\n');

figure('Name', sprintf('Propulsion Analysis - %s', propName), 'Color', 'w');
plot(V_vec, T_vec_N, 'LineWidth', 2);
grid on; box on;
xlabel('Flight Speed V_{\infty} [m/s]');
ylabel('Thrust T [N]');
title(sprintf('Thrust vs Flight Speed - %s', propName), 'Interpreter', 'none');

figure('Name', sprintf('Current Draw - %s', propName), 'Color', 'w');
plot(V_vec, I_vec_A, 'LineWidth', 2); hold on;
yline(I_max, '--', 'LineWidth', 1.5);
grid on; box on;
xlabel('Flight Speed V_{\infty} [m/s]');
ylabel('Current I [A]');
title(sprintf('Current vs Flight Speed - %s', propName), 'Interpreter', 'none');
legend('Current draw', 'Current limit', 'Location', 'best');

end

function val = localBalance(omega, Vinf, D_m, Qm, F_Q)
    n = omega / (2*pi);

    if ~isfinite(n) || n <= 0
        val = NaN;
        return;
    end

    J = Vinf / (n * D_m);

    if ~isfinite(J)
        val = NaN;
        return;
    end

    val = Qm(omega) - F_Q(n, J);
end