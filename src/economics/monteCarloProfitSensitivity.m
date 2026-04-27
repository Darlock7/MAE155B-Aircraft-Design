function mcOut = monteCarloProfitSensitivity(mcIn)
% monteCarloProfitSensitivity
%
% Purpose:
%   Monte Carlo sensitivity analysis for profit per unit time J.
%   Samples the flying-wing design space, filters infeasible designs,
%   then reports global (Pearson correlation) and local (finite-difference)
%   sensitivities. Generates trade-off surface plots.
%
% Inputs (mcIn struct):
%   .N              number of Monte Carlo samples   (default 200000)
%   .rng_seed       random seed for reproducibility (default 1)
%   .R_cruise_m     mission range [m]
%   .eta_p          propulsive efficiency [-]
%   .reserve_factor energy reserve multiplier [-]
%   .rho            air density [kg/m³]
%   .g              gravity [m/s²]
%   .Vs_max_mps     stall speed constraint [m/s]
%   .stall_margin   V_cruise / Vs minimum ratio [-]  (default 1.30)
%   .SM_min_pct     min static margin [%]            (default 5)
%   .SM_max_pct     max static margin [%]            (default 20)
%   .bounds         struct with [lo, hi] per variable (see defaults below)
%   .x0             struct with baseline values for local sensitivity
%                   fields: W_empty_N, Wp_N, Vp_m3, V_mps, LD,
%                           Sref_m2, CLmax, SM_pct
%   .showPlots      generate figures (default true)
%
% Outputs (mcOut struct):
%   .validData          table of feasible samples
%   .feasibleFraction   fraction of samples that passed all constraints
%   .sensitivityTable   Pearson correlation of each variable with J
%   .derivativeTable    local dJ/dx and normalised sensitivity at x0
%   .bestDesign         table row of highest-J feasible design

    % ---- defaults ----
    if ~isfield(mcIn,'N'),            mcIn.N            = 200000; end
    if ~isfield(mcIn,'rng_seed'),     mcIn.rng_seed     = 1;      end
    if ~isfield(mcIn,'stall_margin'), mcIn.stall_margin = 1.30;   end
    if ~isfield(mcIn,'SM_min_pct'),   mcIn.SM_min_pct   = 5.0;    end
    if ~isfield(mcIn,'SM_max_pct'),   mcIn.SM_max_pct   = 20.0;   end
    if ~isfield(mcIn,'showPlots'),    mcIn.showPlots    = true;   end

    % ---- default exploration bounds ----
    if ~isfield(mcIn,'bounds')
        mcIn.bounds = struct();
    end
    b = mcIn.bounds;
    if ~isfield(b,'W_empty_N'), b.W_empty_N = [8,  25];          end  % [N]  ~0.8-2.5 kg
    if ~isfield(b,'Wp_N'),      b.Wp_N      = [2,  15];          end  % [N]  payload
    if ~isfield(b,'Vp_m3'),     b.Vp_m3     = [0.001, 0.010];   end  % [m³] 1-10 L
    if ~isfield(b,'V_mps'),     b.V_mps     = [18,  28];         end  % [m/s]
    if ~isfield(b,'LD'),        b.LD        = [5.5, 15];         end
    if ~isfield(b,'Sref_m2'),   b.Sref_m2   = [0.15, 0.70];     end  % [m²]
    if ~isfield(b,'CLmax'),     b.CLmax     = [0.75, 1.20];      end
    if ~isfield(b,'SM_pct'),    b.SM_pct    = [0,   20];         end  % [%]

    N     = mcIn.N;
    rng(mcIn.rng_seed);

    R_cruise_m     = mcIn.R_cruise_m;
    eta_p          = mcIn.eta_p;
    reserve_factor = mcIn.reserve_factor;
    rho            = mcIn.rho;
    Vs_max         = mcIn.Vs_max_mps;
    sm_fac         = mcIn.stall_margin;
    SM_lo          = mcIn.SM_min_pct;
    SM_hi          = mcIn.SM_max_pct;

    % ---- Monte Carlo sampling ----
    W_empty_N = randUniform(b.W_empty_N, N);
    Wp_N      = randUniform(b.Wp_N,      N);
    Vp_m3     = randUniform(b.Vp_m3,     N);
    V_mps     = randUniform(b.V_mps,     N);
    LD        = randUniform(b.LD,        N);
    Sref_m2   = randUniform(b.Sref_m2,   N);
    CLmax     = randUniform(b.CLmax,     N);
    SM_pct    = randUniform(b.SM_pct,    N);

    Wg_N       = W_empty_N + Wp_N;
    J          = nan(N,1);
    Vs_payload = nan(N,1);
    isFeasible = false(N,1);

    for i = 1:N
        Vs_payload(i) = sqrt(2*Wg_N(i) / (rho*Sref_m2(i)*CLmax(i)));
        if Vs_payload(i) > Vs_max;                    continue; end
        if V_mps(i) < sm_fac * Vs_payload(i);        continue; end
        if SM_pct(i) < SM_lo || SM_pct(i) > SM_hi;  continue; end

        Tf_raw = R_cruise_m / V_mps(i);
        Tf_sc  = 20 * Tf_raw;
        Ef_sc  = 20 * reserve_factor * (Wg_N(i)/LD(i)) * R_cruise_m / eta_p;

        J(i) = profitPerUnitTime(Wp_N(i), Vp_m3(i), Ef_sc, Wg_N(i), Tf_sc);
        isFeasible(i) = true;
    end

    J_hr = 3600 * J;

    data = table(W_empty_N, Wp_N, Vp_m3, V_mps, LD, Sref_m2, CLmax, ...
                 SM_pct, Wg_N, Vs_payload, J, J_hr, isFeasible);
    validData = data(isFeasible, :);

    % ---- print summary ----
    fprintf('\n===== MONTE CARLO PROFIT SENSITIVITY =====\n');
    fprintf('  Mission:  R=%.0f m,  eta_p=%.2f,  reserve=%.2f,  rho=%.3f kg/m³\n', ...
        R_cruise_m, eta_p, reserve_factor, rho);
    fprintf('  Total samples:      %d\n', N);
    fprintf('  Feasible samples:   %d  (%.1f %%)\n', height(validData), 100*height(validData)/N);
    fprintf('  Best profit:        %.4f $/hr\n',   max(validData.J_hr));
    fprintf('  Mean profit:        %.4f $/hr\n',  mean(validData.J_hr));
    fprintf('  Median profit:      %.4f $/hr\n', median(validData.J_hr));

    [~, idxBest] = max(validData.J_hr);
    bestDesign = validData(idxBest, :);
    fprintf('\n  Best design:\n');
    fprintf('    W_empty=%.1f N  Wp=%.1f N  Vp=%.2f L  V=%.1f m/s  LD=%.1f  Sref=%.3f m²\n', ...
        bestDesign.W_empty_N, bestDesign.Wp_N, bestDesign.Vp_m3*1000, ...
        bestDesign.V_mps, bestDesign.LD, bestDesign.Sref_m2);

    % ---- global Pearson correlation sensitivity ----
    varNames = {'W_empty_N','Wp_N','Vp_m3','V_mps','LD','Sref_m2','CLmax','SM_pct','Wg_N'};
    X = validData{:, varNames};
    Y = validData.J_hr;
    corrVals = zeros(numel(varNames), 1);
    for k = 1:numel(varNames)
        corrVals(k) = corr(X(:,k), Y, 'Rows', 'complete');
    end
    sensitivityTable = table(varNames', corrVals, 'VariableNames', {'Variable','CorrWithProfit'});
    sensitivityTable = sortrows(sensitivityTable, 'CorrWithProfit', 'descend');
    fprintf('\n  Global correlation sensitivity (Pearson):\n');
    disp(sensitivityTable);

    % ---- local finite-difference sensitivity ----
    if isfield(mcIn, 'x0') && ~isempty(mcIn.x0)
        x0 = mcIn.x0;
        [J0, feas0] = profitModelSimple(x0, R_cruise_m, eta_p, reserve_factor, ...
                                        rho, Vs_max, sm_fac, SM_lo, SM_hi);
        derivNames = {'W_empty_N','Wp_N','Vp_m3','V_mps','LD','Sref_m2','CLmax','SM_pct'};
        dJdx   = nan(numel(derivNames),1);
        S_norm = nan(numel(derivNames),1);

        for k = 1:numel(derivNames)
            nm = derivNames{k};
            dx = 0.01 * x0.(nm);
            xp = x0;  xp.(nm) = x0.(nm) + dx;
            xm = x0;  xm.(nm) = x0.(nm) - dx;
            [Jp, fp] = profitModelSimple(xp, R_cruise_m, eta_p, reserve_factor, rho, Vs_max, sm_fac, SM_lo, SM_hi);
            [Jm, fm] = profitModelSimple(xm, R_cruise_m, eta_p, reserve_factor, rho, Vs_max, sm_fac, SM_lo, SM_hi);
            if fp && fm && feas0
                dJdx(k)   = (Jp - Jm) / (2*dx);
                S_norm(k) = dJdx(k) * x0.(nm) / J0;
            end
        end
        derivativeTable = table(derivNames', dJdx*3600, S_norm, ...
            'VariableNames', {'Variable','dJ_dhr_perUnit','NormSensitivity'});
        derivativeTable = sortrows(derivativeTable, 'NormSensitivity', 'descend');
        fprintf('  Local sensitivity at current design (J=%.4f $/hr, feasible=%d):\n', J0*3600, feas0);
        disp(derivativeTable);
        mcOut.derivativeTable = derivativeTable;
    else
        mcOut.derivativeTable = table();
    end

    mcOut.validData         = validData;
    mcOut.feasibleFraction  = height(validData) / N;
    mcOut.sensitivityTable  = sensitivityTable;
    mcOut.bestDesign        = bestDesign;

    if ~mcIn.showPlots; return; end

    % ---- Figure 1: Profit surface — Payload Volume vs Weight ----
    figure('Name','MC: Profit Surface — Payload Volume vs Weight');
    plotTradeoffSurface(validData.Vp_m3*1000, validData.Wp_N, validData.J_hr, 25, ...
        'Payload Volume V_p [L]', 'Payload Weight W_p [N]', 'Profit J [$/hr]', ...
        'MC Profit Surface: Volume vs Weight');

    % ---- Figure 2: Profit surface — L/D vs Cruise Speed ----
    figure('Name','MC: Profit Surface — L/D vs Speed');
    plotTradeoffSurface(validData.LD, validData.V_mps, validData.J_hr, 25, ...
        'Lift-to-Drag Ratio L/D [-]', 'Cruise Speed V [m/s]', 'Profit J [$/hr]', ...
        'MC Profit Surface: L/D vs Speed');

    % ---- Figure 3: 3D surface — Volume vs Speed ----
    figure('Name','MC: 3D Surface — Payload Volume vs Speed');
    plotSmoothedSurface(validData.Vp_m3*1000, validData.V_mps, validData.J_hr, 25, ...
        'Payload Volume V_p [L]', 'Cruise Speed V [m/s]', 'Profit J [$/hr]', ...
        'MC Smoothed Surface: Volume vs Speed');

    % ---- Figure 4: Tornado chart (global sensitivity) ----
    figure('Name','MC: Global Sensitivity (Pearson Correlation)');
    barh(sensitivityTable.CorrWithProfit, 'FaceColor', [0.2 0.5 0.8]);
    set(gca, 'YTick', 1:height(sensitivityTable), 'YTickLabel', sensitivityTable.Variable);
    xlabel('Pearson Correlation with Profit J'); ylabel('Design Variable');
    title('Global Monte Carlo Sensitivity');
    xline(0, 'k-');
    grid on; box on;

end

%% ========================================================================
function x = randUniform(bounds, N)
    x = bounds(1) + (bounds(2) - bounds(1)) * rand(N, 1);
end

function [J, feasible] = profitModelSimple(x, R_cruise_m, eta_p, reserve_factor, ...
                                            rho, Vs_max_mps, stall_margin, ...
                                            SM_min_pct, SM_max_pct)
    feasible = false;
    J = NaN;

    Wg_N = x.W_empty_N + x.Wp_N;
    Vs   = sqrt(2*Wg_N / (rho * x.Sref_m2 * x.CLmax));

    if Vs > Vs_max_mps;                               return; end
    if x.V_mps < stall_margin * Vs;                  return; end
    if x.SM_pct < SM_min_pct || x.SM_pct > SM_max_pct; return; end

    Tf = 20 * R_cruise_m / x.V_mps;
    Ef = 20 * reserve_factor * (Wg_N / x.LD) * R_cruise_m / eta_p;

    J = profitPerUnitTime(x.Wp_N, x.Vp_m3, Ef, Wg_N, Tf);
    feasible = true;
end

function plotTradeoffSurface(x, y, z, nbins, xLbl, yLbl, zLbl, ttl)
    xedges = linspace(min(x), max(x), nbins+1);
    yedges = linspace(min(y), max(y), nbins+1);
    [~,~,xb] = histcounts(x, xedges);
    [~,~,yb] = histcounts(y, yedges);
    Z = nan(nbins, nbins);
    for i = 1:numel(z)
        if xb(i)>0 && yb(i)>0 && isfinite(z(i))
            if isnan(Z(xb(i),yb(i)))
                Z(xb(i),yb(i)) = z(i);
            else
                Z(xb(i),yb(i)) = max(Z(xb(i),yb(i)), z(i));
            end
        end
    end
    xc = (xedges(1:end-1)+xedges(2:end))/2;
    yc = (yedges(1:end-1)+yedges(2:end))/2;
    [X,Y] = meshgrid(xc, yc);
    surf(X, Y, Z'); shading interp; colorbar;
    xlabel(xLbl); ylabel(yLbl); zlabel(zLbl); title(ttl);
    grid on; view(135,30); drawnow;
end

function plotSmoothedSurface(x, y, z, nbins, xLbl, yLbl, zLbl, ttl)
    xedges = linspace(min(x), max(x), nbins+1);
    yedges = linspace(min(y), max(y), nbins+1);
    [~,~,xb] = histcounts(x, xedges);
    [~,~,yb] = histcounts(y, yedges);
    Z     = nan(nbins, nbins);
    count = zeros(nbins, nbins);
    for i = 1:numel(z)
        if xb(i)>0 && yb(i)>0 && isfinite(z(i))
            if isnan(Z(xb(i),yb(i))), Z(xb(i),yb(i)) = 0; end
            Z(xb(i),yb(i)) = Z(xb(i),yb(i)) + z(i);
            count(xb(i),yb(i)) = count(xb(i),yb(i)) + 1;
        end
    end
    Z = Z ./ count;
    Z(count==0) = NaN;
    xc = (xedges(1:end-1)+xedges(2:end))/2;
    yc = (yedges(1:end-1)+yedges(2:end))/2;
    [X,Y] = meshgrid(xc, yc);
    surf(X, Y, Z'); shading interp; colorbar;
    xlabel(xLbl); ylabel(yLbl); zlabel(zLbl); title(ttl);
    grid on; view(135,30); drawnow;
end
