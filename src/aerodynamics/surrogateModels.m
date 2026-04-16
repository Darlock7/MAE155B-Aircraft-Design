%% Build surrogate models for all candidate APC propellers
clear; clc;

% ------------------------------------------------------------
% Candidate propeller files
% filename, label, diameter_in
% ------------------------------------------------------------
props = {
    'PER3_14x10.txt',     '14x10',      8;
    'PER3_9x6.txt',     '9x6',      9;
    'PER3_9x45MR.txt',  '9x4.5MR',  9;
    'PER3_9x47SF.txt',  '9x4.7SF',  9;
    'PER3_10x45MR.txt', '10x4.5MR', 10;
};

models = struct();

for p = 1:size(props,1)
    filename   = props{p,1};
    propLabel  = props{p,2};
    D_in       = props{p,3};
    D_m        = D_in * 0.0254;

    fprintf('\nProcessing %s ...\n', propLabel);

    % Read file
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

        % Detect RPM header
        tokRPM = regexp(line, 'PROP RPM =\s*([0-9]+)', 'tokens');
        if ~isempty(tokRPM)
            currentRPM = str2double(tokRPM{1}{1});
            continue;
        end

        if isnan(currentRPM)
            continue;
        end

        nums = sscanf(line, '%f');

        % APC data row structure:
        % 1:V, 2:J, ... 10:Torque(N-m), 11:Thrust(N)
        if numel(nums) >= 11
            J = nums(2);
            Q = nums(10);
            T = nums(11);

            if isfinite(J) && isfinite(Q) && isfinite(T)
                RPM_all(end+1,1) = currentRPM;
                n_all(end+1,1)   = currentRPM / 60; % rev/s
                J_all(end+1,1)   = J;
                Q_all(end+1,1)   = Q;
                T_all(end+1,1)   = T;
            end
        end
    end

    % Build surrogate models
    F_Q = scatteredInterpolant(n_all, J_all, Q_all, 'natural', 'nearest');
    F_T = scatteredInterpolant(n_all, J_all, T_all, 'natural', 'nearest');

    % Store
    models(p).name     = propLabel;
    models(p).filename = filename;
    models(p).D_in     = D_in;
    models(p).D_m      = D_m;
    models(p).RPM      = RPM_all;
    models(p).n        = n_all;
    models(p).J        = J_all;
    models(p).Q        = Q_all;
    models(p).T        = T_all;
    models(p).F_Q      = F_Q;
    models(p).F_T      = F_T;

    % Plot torque surrogate
    n_plot = linspace(min(n_all), max(n_all), 80);
    J_plot = linspace(min(J_all), max(J_all), 80);
    [NG, JG] = meshgrid(n_plot, J_plot);

    QG = F_Q(NG, JG);
    TG = F_T(NG, JG);

    figure('Name',[propLabel ' Torque']);
    surf(NG*60, JG, QG, 'EdgeColor', 'none');
    xlabel('RPM');
    ylabel('Advance Ratio J');
    zlabel('Torque Q_p (N*m)');
    title([propLabel ' Torque Surrogate']);
    colorbar; view(135,30);

    figure('Name',[propLabel ' Thrust']);
    surf(NG*60, JG, TG, 'EdgeColor', 'none');
    xlabel('RPM');
    ylabel('Advance Ratio J');
    zlabel('Thrust T (N)');
    title([propLabel ' Thrust Surrogate']);
    colorbar; view(135,30);
end

save('all_prop_surrogates.mat', 'models');
fprintf('\nSaved surrogate models to all_prop_surrogates.mat\n');

%% Motor parameters
Kv = 1100;      % RPM/V
Rm = 0.073;     % ohm
I0 = 0.9;       % A
Vbatt = 11.1;   % V

Kt = 60/(2*pi*Kv);   % N*m/A
Ke = Kt;             % V*s/rad

Qm = @(omega) Kt * (((Vbatt - Ke*omega)/Rm) - I0);
Im = @(omega) (Vbatt - Ke*omega)/Rm;

Vinf = 20;  % UPDATED ONCE WE DO AERODYNAMIC ANALYSIS 

results = struct();

for p = 1:length(models)
    D = models(p).D_m;
    FQ = models(p).F_Q;
    FT = models(p).F_T;

    balanceFun = @(omega) Qm(omega) - FQ(omega/(2*pi), Vinf / ((omega/(2*pi))*D));

    omega_guess = 9000 * 2*pi/60;

    try
        omega_sol = fzero(balanceFun, omega_guess);

        n_sol = omega_sol/(2*pi);
        rpm_sol = n_sol * 60;
        J_sol = Vinf/(n_sol*D);
        I_sol = Im(omega_sol);
        Qp_sol = FQ(n_sol, J_sol);
        T_sol = FT(n_sol, J_sol);

        results(p).name = models(p).name;
        results(p).rpm  = rpm_sol;
        results(p).J    = J_sol;
        results(p).I    = I_sol;
        results(p).Qp   = Qp_sol;
        results(p).T    = T_sol;
    catch
        results(p).name = models(p).name;
        results(p).rpm  = NaN;
        results(p).J    = NaN;
        results(p).I    = NaN;
        results(p).Qp   = NaN;
        results(p).T    = NaN;
    end
end

%% Display results
fprintf('\nOperating points at Vinf = %.2f m/s\n', Vinf);
fprintf('---------------------------------------------\n');
for p = 1:length(results)
    fprintf('%-8s  RPM=%8.1f   I=%6.2f A   T=%7.3f N   J=%6.3f\n', ...
        results(p).name, results(p).rpm, results(p).I, results(p).T, results(p).J);
end
%% Rank candidate propellers using motor-prop torque balance
clear; clc;

% ------------------------------------------------------------
% Candidate propeller files
% filename, label, diameter_in
% ------------------------------------------------------------
props = {
    'PER3_8x6.txt',     '8x6',      8;
    'PER3_9x6.txt',     '9x6',      9;
    'PER3_9x45MR.txt',  '9x4.5MR',  9;
    'PER3_9x47SF.txt',  '9x4.7SF',  9;
    'PER3_10x45MR.txt', '10x4.5MR', 10;
};

% ------------------------------------------------------------
% Motor parameters
% ------------------------------------------------------------
Kv   = 1100;     % RPM/V
Rm   = 0.073;    % ohm
I0   = 0.9;      % A
Vbatt = 11.1;    % V
Imax  = 40;      % absolute max current
Isoft = 35;      % preferred upper limit

Kt = 60/(2*pi*Kv);   % N*m/A
Ke = Kt;             % V*s/rad

Qm = @(omega) Kt * (((Vbatt - Ke*omega)/Rm) - I0);
Im = @(omega) (Vbatt - Ke*omega)/Rm;

% ------------------------------------------------------------
% Flight conditions [m/s]
% edit these to match your assignment
% ------------------------------------------------------------
flightNames = {'Takeoff', 'Climb', 'Cruise'};
Vinf_list   = [0, 8, 16];

% ------------------------------------------------------------
% Build surrogate models
% ------------------------------------------------------------
models = struct();

for p = 1:size(props,1)
    filename   = props{p,1};
    propLabel  = props{p,2};
    D_in       = props{p,3};
    D_m        = D_in * 0.0254;

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

        % APC columns:
        % 2 = J, 10 = Torque (N-m), 11 = Thrust (N)
        if numel(nums) >= 11
            J = nums(2);
            Q = nums(10);
            T = nums(11);

            if isfinite(J) && isfinite(Q) && isfinite(T)
                RPM_all(end+1,1) = currentRPM;
                n_all(end+1,1)   = currentRPM/60;
                J_all(end+1,1)   = J;
                Q_all(end+1,1)   = Q;
                T_all(end+1,1)   = T;
            end
        end
    end

    models(p).name = propLabel;
    models(p).D_m  = D_m;
    models(p).F_Q  = scatteredInterpolant(n_all, J_all, Q_all, 'natural', 'nearest');
    models(p).F_T  = scatteredInterpolant(n_all, J_all, T_all, 'natural', 'nearest');
    models(p).nmin = min(n_all);
    models(p).nmax = max(n_all);
    models(p).Jmin = min(J_all);
    models(p).Jmax = max(J_all);
end

% ------------------------------------------------------------
% Solve operating point for each prop at each flight condition
% ------------------------------------------------------------
results = struct();

for p = 1:length(models)
    results(p).name = models(p).name;
    results(p).valid_all = true;
    results(p).soft_ok_all = true;
    results(p).T_values = nan(size(Vinf_list));
    results(p).I_values = nan(size(Vinf_list));
    results(p).rpm_values = nan(size(Vinf_list));
    results(p).J_values = nan(size(Vinf_list));

    D  = models(p).D_m;
    FQ = models(p).F_Q;
    FT = models(p).F_T;

    for i = 1:length(Vinf_list)
        Vinf = Vinf_list(i);

        % residual for torque balance
        balanceFun = @(omega) motorPropResidual(omega, Vinf, D, Qm, FQ, models(p));

        % try several guesses for robustness
        guesses_rpm = [5000 7000 9000 11000 13000];
        solved = false;

        for g = 1:length(guesses_rpm)
            omega_guess = guesses_rpm(g) * 2*pi/60;
            try
                omega_sol = fzero(balanceFun, omega_guess);

                n_sol   = omega_sol/(2*pi);
                rpm_sol = n_sol*60;
                J_sol   = Vinf/(n_sol*D);
                I_sol   = Im(omega_sol);
                T_sol   = FT(n_sol, J_sol);

                if isfinite(I_sol) && isfinite(T_sol) && rpm_sol > 0
                    solved = true;
                    break;
                end
            catch
            end
        end

        if ~solved
            results(p).valid_all = false;
            continue;
        end

        results(p).rpm_values(i) = rpm_sol;
        results(p).J_values(i)   = J_sol;
        results(p).I_values(i)   = I_sol;
        results(p).T_values(i)   = T_sol;

        if I_sol > Imax
            results(p).valid_all = false;
        end
        if I_sol > Isoft
            results(p).soft_ok_all = false;
        end
    end

    % scoring
    if results(p).valid_all
        results(p).T_takeoff = results(p).T_values(1);
        results(p).T_climb   = results(p).T_values(2);
        results(p).T_cruise  = results(p).T_values(3);
        results(p).I_peak    = max(results(p).I_values);
        results(p).score     = 100*results(p).valid_all ...
                             + 20*results(p).soft_ok_all ...
                             + 10*results(p).T_takeoff ...
                             + 5*results(p).T_climb ...
                             + 2*results(p).T_cruise ...
                             - 0.5*results(p).I_peak;
    else
        results(p).T_takeoff = NaN;
        results(p).T_climb   = NaN;
        results(p).T_cruise  = NaN;
        results(p).I_peak    = NaN;
        results(p).score     = -Inf;
    end
end

% ------------------------------------------------------------
% Sort and print ranking
% ------------------------------------------------------------
scores = [results.score];
[~, idx] = sort(scores, 'descend');
ranked = results(idx);

fprintf('\n=== PROPELLER RANKING ===\n');
fprintf('Criteria: must stay below %.1f A, prefer below %.1f A\n\n', Imax, Isoft);

for k = 1:length(ranked)
    r = ranked(k);
    fprintf('%d) %-8s | score = %8.3f | valid = %d | soft_ok = %d\n', ...
        k, r.name, r.score, r.valid_all, r.soft_ok_all);

    if r.valid_all
        fprintf('   Takeoff: RPM=%8.1f  I=%6.2f A  T=%7.3f N  J=%6.3f\n', ...
            r.rpm_values(1), r.I_values(1), r.T_values(1), r.J_values(1));
        fprintf('   Climb  : RPM=%8.1f  I=%6.2f A  T=%7.3f N  J=%6.3f\n', ...
            r.rpm_values(2), r.I_values(2), r.T_values(2), r.J_values(2));
        fprintf('   Cruise : RPM=%8.1f  I=%6.2f A  T=%7.3f N  J=%6.3f\n', ...
            r.rpm_values(3), r.I_values(3), r.T_values(3), r.J_values(3));
    else
        fprintf('   No valid operating point under current limit.\n');
    end
    fprintf('\n');
end

fprintf('Best propeller: %s\n', ranked(1).name);

% ------------------------------------------------------------
% Optional: summary table
% ------------------------------------------------------------
fprintf('\n%-8s %-8s %-8s %-8s %-8s %-8s\n', ...
    'Prop', 'Valid', 'SoftOK', 'T_TO(N)', 'T_CL(N)', 'Ipk(A)');
for k = 1:length(ranked)
    r = ranked(k);
    fprintf('%-8s %-8d %-8d %-8.3f %-8.3f %-8.2f\n', ...
        r.name, r.valid_all, r.soft_ok_all, r.T_takeoff, r.T_climb, r.I_peak);
end

% ------------------------------------------------------------
% Helper function
% ------------------------------------------------------------
function res = motorPropResidual(omega, Vinf, D, Qm, FQ, model)
    n = omega/(2*pi);

    if n <= 0
        res = 1e6;
        return;
    end

    J = Vinf/(n*D);

    % keep query in interpolant range
    nq = min(max(n, model.nmin), model.nmax);
    Jq = min(max(J, model.Jmin), model.Jmax);

    Qp = FQ(nq, Jq);
    res = Qm(omega) - Qp;
end