function optOut = profitOptimization(optIn)
% profitOptimization
%
% Purpose:
%   CMA-ES global optimizer for full aircraft design.
%   Maximizes profit per unit time J subject to stability, stall,
%   aerodynamic mode, and physical build constraints.
%
% Design variables (11):
%   x(1)  = Wing AR                    [-]     bounds [4, 10]
%   x(2)  = Wing taper ratio           [-]     bounds [0.30, 0.80]
%   x(3)  = Wing quarter-chord sweep   [deg]   bounds [0, 30]
%   x(4)  = Root geometric twist       [deg]   bounds [-5, 2]
%   x(5)  = Wing loading WS_design     [N/m²]  bounds [20, 60]
%   x(6)  = Wing LE root x-position    [m]     bounds [0.03, 0.15]
%   x(7)  = Fin AR                     [-]     bounds [0.5, 2.5]
%   x(8)  = Fin taper                  [-]     bounds [0.3, 0.8]
%   x(9)  = Fin quarter-chord sweep    [deg]   bounds [20, 55]
%   x(10) = Cruise speed               [m/s]   bounds [18, 28]
%   x(11) = Cargo volume Vp            [m³]    bounds [0.001, 0.010]
%
% CMA-ES operates in normalized [0,1]^11 space for numerical conditioning.
%
% Physical constraints (hard — return 1e6 if violated):
%   - Wingspan ≤ b_max_m
%   - Tip chord ≥ c_tip_min_m
%   - Stall speed ≤ Vs_max_mps
%   - Cruise speed ≥ Vs_margin_fac × Vs  (stall margin)
%   - Fin height ≤ b_v_max_frac × semispan
%   - Gross weight ≤ Wg_max_N
%
% Stability constraints (soft quadratic penalty on objective):
%   - SM ∈ [SM_min_pct, SM_max_pct]
%   - Short-period oscillatory and stable
%   - Phugoid non-divergent
%   - Dutch roll non-divergent
%   - Trim deflection |de_trim| ≤ de_trim_max_deg
%
% Required inputs (optIn struct):
%   .dynIn_base           base dynIn with AVL paths, CS fractions, airfoil files
%   .wingIn_base          base wingIn with y/z_root, eta_cs_*, symmetric flag
%   .vertIn_base          base vertIn with isTwin, sizeMode, c_v, rudder, airfoilName
%   .airfoilDB            preloaded surrogate database (from loadAirfoilSurrogateDB)
%   .airfoilRootName      root airfoil filename string
%   .airfoilTipName       tip airfoil filename string
%   .cadMass              fuselage CAD mass struct
%   .compFixed            point mass array, indices 1-6: motor/prop/ESC/batt/rx/payload
%   .eta_servo            servo span fraction for wing servo placement
%   .Wp_N                 payload weight [N]  (fixed)
%   .Wprop_N              propulsion system weight [N]
%   .Vp_ref_m3            reference volume for VPS penalty [m³]
%   .fe_base              baseline empty weight fraction
%   .ke                   VPS penalty slope [per VPS unit]
%   .fe_max               hard cap on fe
%   .eta_p                propulsive efficiency
%   .R_cruise_m           mission range [m]
%   .delta_h_m            climb altitude [m]
%   .reserve_factor       energy reserve multiplier
%   .Tf_s                 un-scaled flight time [s]  (×20 applied for scoring)
%   .roh                  air density [kg/m³]
%   .mu_Pas               dynamic viscosity [Pa·s]
%   .g                    gravitational acceleration [m/s²]
%   .Swet_fuse_m2         fixed fuselage wetted area [m²]
%   .Lf_m / .Wf_m / .Hf_m fuselage dimensions [m]
%   .tc / .xc             airfoil t/c and x/c of max thickness
%   .Q_wing / .Q_fuse / .Q_fin  interference factors
%   .m_wing_struct_ref_kg reference wing structure mass [kg]
%   .m_vert_struct_ref_kg reference total fin structure mass [kg]
%   .S_ref_base_m2        reference wing area for structural scaling [m²]
%   .AR_base              reference AR for structural scaling
%   .S_fin_base_m2        reference total fin area for structural scaling [m²]
%
% Optional / defaulted inputs:
%   .SM_min_pct           min static margin [%]        (default 5)
%   .SM_max_pct           max static margin [%]        (default 20)
%   .Vs_max_mps           max stall speed [m/s]        (default 12)
%   .b_max_m              max wingspan [m]             (default 1.8)
%   .c_tip_min_m          min tip chord [m]            (default 0.05)
%   .b_v_max_frac         max fin height / semispan    (default 0.50)
%   .de_trim_max_deg      max trim deflection [deg]    (default 15)
%   .Vs_margin_fac        V_cruise / Vs floor          (default 1.30)
%   .Wg_max_N             max gross weight [N]         (default 60)
%   .x0                   [11×1] initial guess, physical units (default: current design)
%   .sigma0               initial step size in [0,1] space (default 0.15)
%   .maxGen               max CMA-ES generations       (default 500)
%   .lambda               population size; 0 = 2×Hansen ≈ 22 (default 0)
%   .tolSigma             step-size convergence tolerance  (default 1e-5)
%   .tolFun               J-range tolerance over last 10 gen (default 1e-4)
%   .verbose              print every N generations    (default 10)
%
% Outputs (optOut struct):
%   .xBest        [11×1] best design vector in physical units
%   .JBest        best J [$/s]
%   .JBest_hr     best J [$/hr]
%   .best         struct: SM_pct, Vs_mps, LD, Wg_g, b_m, AR, S_ref, Vp_L, V_cruise
%   .history      struct array per generation

    % ---- physical bounds ----
    %           AR    tap   swp   twst  WS    xLE   ARv  tapv  swpv  Vc    Vp    cb_hw  cb_len
    lb = [4.0; 0.30;  0.0; -5.0; 20.0; 0.030; 2.00; 0.30; 20.0; 20.0; 0.001; 0.050; 0.600];
    ub = [10.0; 0.80; 30.0; 3.0; 90.0; 0.300; 3.50; 0.70; 50.0; 28.0; 0.015; 0.300; 1.000];
    x0_def = [8.5; 0.702; 22.7; 0.0; 40.0; 0.100; 2.41; 0.400; 40.7; 24.0; 0.0060; 0.145; 0.620];

    if ~isfield(optIn,'x0'),              optIn.x0              = x0_def;  end
    if ~isfield(optIn,'sigma0'),          optIn.sigma0          = 0.15;    end
    if ~isfield(optIn,'maxGen'),          optIn.maxGen          = 500;     end
    if ~isfield(optIn,'lambda'),          optIn.lambda          = 0;       end
    if ~isfield(optIn,'tolSigma'),        optIn.tolSigma        = 1e-5;    end
    if ~isfield(optIn,'tolFun'),          optIn.tolFun          = 1e-6;    end
    if ~isfield(optIn,'verbose'),         optIn.verbose         = 10;      end
    if ~isfield(optIn,'SM_min_pct'),      optIn.SM_min_pct      = 5.0;     end
    if ~isfield(optIn,'SM_max_pct'),      optIn.SM_max_pct      = 13.0;    end
    if ~isfield(optIn,'Vs_max_mps'),      optIn.Vs_max_mps      = 12.0;    end
    if ~isfield(optIn,'b_max_m'),         optIn.b_max_m         = 1.8;     end
    if ~isfield(optIn,'c_tip_min_m'),     optIn.c_tip_min_m     = 0.05;    end
    if ~isfield(optIn,'b_v_max_frac'),    optIn.b_v_max_frac    = 0.50;    end
    if ~isfield(optIn,'de_trim_max_deg'), optIn.de_trim_max_deg = 15.0;    end
    if ~isfield(optIn,'Vs_margin_fac'),   optIn.Vs_margin_fac   = 1.30;    end
    if ~isfield(optIn,'Wg_max_N'),        optIn.Wg_max_N        = 60.0;    end
    if ~isfield(optIn,'debugObj'),        optIn.debugObj        = false;   end

    n = numel(lb);

    % normalize initial guess to [0,1]
    x0_norm = (optIn.x0(:) - lb) ./ (ub - lb);
    x0_norm = max(0, min(1, x0_norm));

    % ---- CMA-ES hyperparameters (Hansen 2016) ----
    if optIn.lambda > 0
        lam = optIn.lambda;
    else
        lam = 2 * (4 + floor(3*log(n)));   % 2× Hansen ≈ 22
    end
    mu    = floor(lam/2);
    w_raw = log(mu + 0.5) - log(1:mu)';
    w     = w_raw / sum(w_raw);
    mueff = 1 / sum(w.^2);

    cs   = (mueff + 2) / (n + mueff + 5);
    ds   = 1 + 2*max(0, sqrt((mueff-1)/(n+1)) - 1) + cs;
    chiN = sqrt(n) * (1 - 1/(4*n) + 1/(21*n^2));
    cc   = (4 + mueff/n) / (n + 4 + 2*mueff/n);
    c1   = 2 / ((n+1.3)^2 + mueff);
    cmu  = min(1-c1, 2*(mueff - 2 + 1/mueff) / ((n+2)^2 + mueff));

    % ---- state initialization ----
    m      = x0_norm;
    sigma  = optIn.sigma0;
    C      = eye(n);
    ps     = zeros(n,1);
    pc     = zeros(n,1);
    eigenC = eye(n);
    diagD  = ones(n,1);
    eigAge = 0;

    JBest      = Inf;
    xBest_norm = m;
    infoBest   = struct('J_hr',NaN,'SM_pct',NaN,'Vs_mps',NaN,'LD',NaN, ...
                        'Wg_g',NaN,'b_m',NaN,'AR',NaN,'S_ref',NaN, ...
                        'Vp_L',NaN,'V_cruise',NaN, ...
                        'cb_halfwidth_m',NaN,'cb_length_m',NaN,'failed',true);
    history    = struct('gen',{},'JBest',{},'sigma',{},'J_hr',{},'SM_pct',{},'Vs_mps',{});

    ctx    = optIn;
    ctx.lb = lb;
    ctx.ub = ub;

    fprintf('\n===== FULL AIRCRAFT PROFIT OPTIMIZATION (CMA-ES) =====\n');
    fprintf('  n=%d variables,  lambda=%d,  mu=%d,  maxGen=%d\n', n, lam, mu, optIn.maxGen);
    fprintf('  AR[%.0f,%.0f]  taper[%.1f,%.1f]  sweep[%.0f,%.0f]deg  ', lb(1),ub(1),lb(2),ub(2),lb(3),ub(3));
    fprintf('V_cruise[%.0f,%.0f]m/s  Vp[%.0f,%.0f]L\n', lb(10),ub(10),lb(11)*1000,ub(11)*1000);
    fprintf('  Rough estimate: %.0f–%.0f min total (AVL ~2 s/call, parfor)\n\n', ...
        lam*2*optIn.maxGen/60*0.4, lam*2*optIn.maxGen/60);
    fprintf('%-6s %-10s %-7s %-7s %-7s %-6s %-5s %-5s %-8s\n', ...
        'Gen','J[$/hr]','SM[%]','Vs[m/s]','L/D','b[m]','AR','Vp[L]','sigma');

    for gen = 1:optIn.maxGen

        if eigAge >= n/10
            [eigenC, D] = eig(C);
            diagD  = sqrt(max(diag(D), 0));
            eigAge = 0;
        end
        eigAge = eigAge + 1;

        Z      = randn(n, lam);
        Y      = eigenC * (diagD .* Z);
        X_norm = m + sigma * Y;
        X_norm = max(0, min(1, X_norm));

        Jvals = nan(lam,1);
        infos = cell(lam,1);
        Xeval = X_norm;

        parfor k = 1:lam
            [Jvals(k), infos{k}] = profit_obj(Xeval(:,k), ctx);
        end

        [Jsorted, idx] = sort(Jvals, 'ascend');
        Xsorted = Xeval(:, idx);
        Ysorted = Y(:, idx);

        if Jsorted(1) < JBest
            JBest      = Jsorted(1);
            xBest_norm = Xsorted(:,1);
            infoBest   = infos{idx(1)};
        end

        m_old = m;
        m     = Xsorted(:, 1:mu) * w;

        y_m         = (m - m_old) / sigma;
        invsqrtC_ym = eigenC * ((1./diagD) .* (eigenC' * y_m));
        ps          = (1-cs)*ps + sqrt(cs*(2-cs)*mueff) * invsqrtC_ym;
        hsig        = norm(ps)/sqrt(1-(1-cs)^(2*(gen+1)))/chiN < 1.4 + 2/(n+1);
        pc          = (1-cc)*pc + hsig * sqrt(cc*(2-cc)*mueff) * y_m;

        rank1  = c1  * (pc*pc' + (1-hsig)*cc*(2-cc)*C);
        rankmu = cmu * (Ysorted(:,1:mu) * diag(w) * Ysorted(:,1:mu)');
        C      = (1 - c1 - cmu) * C + rank1 + rankmu;
        C      = (C + C') / 2;
        sigma  = sigma * exp((cs/ds) * (norm(ps)/chiN - 1));

        if optIn.verbose > 0 && mod(gen, optIn.verbose) == 0
            fprintf('%-6d %-10.4f %-7.2f %-7.2f %-7.3f %-6.3f %-5.2f %-5.2f %-8.2e\n', ...
                gen, infoBest.J_hr, infoBest.SM_pct, infoBest.Vs_mps, infoBest.LD, ...
                infoBest.b_m, infoBest.AR, infoBest.Vp_L, sigma);
        end
        history(end+1) = struct('gen',gen,'JBest',JBest,'sigma',sigma, ...
            'J_hr',infoBest.J_hr,'SM_pct',infoBest.SM_pct,'Vs_mps',infoBest.Vs_mps);

        if sigma < optIn.tolSigma
            fprintf('Converged: sigma = %.2e < tolSigma\n', sigma); break;
        end
        if gen > 50 && range([history(end-9:end).JBest]) < optIn.tolFun
            fprintf('Converged: objective range < tolFun over last 10 gen\n'); break;
        end
    end

    % decode best to physical units
    xBest = lb + xBest_norm .* (ub - lb);

    varNames = {'AR','Taper','Sweep c/4 [deg]','Root twist [deg]','WS [N/m²]', ...
                'xLE_root [m]','AR_v','Taper_v','Sweep_v [deg]','V_cruise [m/s]','Vp [m³]', ...
                'cb_halfwidth [m]','cb_length [m]'};
    fprintf('\n========== OPTIMAL AIRCRAFT DESIGN ==========\n');
    for i = 1:n
        fprintf('  %-24s = %.4f\n', varNames{i}, xBest(i));
    end
    fprintf('\n  Profit J               = %.4f $/hr\n', infoBest.J_hr);
    fprintf('  Static margin          = %.2f %%\n',    infoBest.SM_pct);
    fprintf('  Stall speed            = %.2f m/s\n',   infoBest.Vs_mps);
    fprintf('  Cruise L/D             = %.3f\n',       infoBest.LD);
    fprintf('  Gross weight           = %.0f g\n',     infoBest.Wg_g);
    fprintf('  Wingspan               = %.3f m\n',     infoBest.b_m);
    fprintf('  Wing area              = %.4f m²\n',    infoBest.S_ref);
    fprintf('  Cargo volume           = %.2f L\n',     infoBest.Vp_L);
    fprintf('  Cruise speed           = %.1f m/s\n',   infoBest.V_cruise);
    fprintf('\n--- Update main.m with these values ---\n');
    fprintf('  AR                        = %.3f\n',    xBest(1));
    fprintf('  wingTapper                = %.3f\n',    xBest(2));
    fprintf('  wingSweep                 = %.1f\n',    xBest(3));
    fprintf('  twistIn.twist_root_deg    = %.2f\n',    xBest(4));
    fprintf('  WS → S_ref = Wg / %.1f  N/m²\n',       xBest(5));
    fprintf('  wingIn.xLE_root_m         = %.4f\n',    xBest(6));
    fprintf('  vertIn.AR_v               = %.3f\n',    xBest(7));
    fprintf('  vertIn.taper_v            = %.3f\n',    xBest(8));
    fprintf('  vertIn.sweep_c4_v_deg     = %.1f\n',    xBest(9));
    fprintf('  V_cruise                  = %.1f\n',    xBest(10));
    fprintf('  Vp (CAD target)           = %.4f m³  (%.2f L)\n', xBest(11), xBest(11)*1000);
    fprintf('  cb_halfwidth [m]          = %.4f m  (total width = %.4f m)\n', xBest(12), 2*xBest(12));
    fprintf('  cb_length [m]             = %.4f m\n', xBest(13));
    fprintf('==============================================\n\n');

    % ---- convergence plots ----
    gens = [history.gen];
    figure('Name','Profit Optimization Convergence');
    subplot(3,1,1);
    plot(gens, [history.J_hr], 'b-', 'LineWidth', 2);
    ylabel('Best J [$/hr]'); xlabel('Generation');
    title('Full Aircraft CMA-ES Convergence'); grid on;
    subplot(3,1,2);
    plot(gens, [history.SM_pct], 'g-', 'LineWidth', 2); hold on;
    yline(optIn.SM_min_pct, 'r--', 'SM_{min}');
    yline(optIn.SM_max_pct, 'r--', 'SM_{max}');
    ylabel('SM [%]'); xlabel('Generation'); grid on;
    subplot(3,1,3);
    plot(gens, [history.sigma], 'k-', 'LineWidth', 2);
    ylabel('\sigma (step size)'); xlabel('Generation'); grid on;

    optOut.xBest    = xBest;
    optOut.JBest    = -JBest;
    optOut.JBest_hr = infoBest.J_hr;
    optOut.best     = infoBest;
    optOut.history  = history;
end

%% ========================================================================
function [Jobj, info] = profit_obj(x_norm, ctx)

    Jobj = 1e6;
    info = struct('J_hr',NaN,'SM_pct',NaN,'Vs_mps',NaN,'LD',NaN, ...
                  'Wg_g',NaN,'b_m',NaN,'AR',NaN,'S_ref',NaN, ...
                  'Vp_L',NaN,'V_cruise',NaN,'failed',true);
    try
        lb = ctx.lb;   ub = ctx.ub;
        xp = lb + x_norm .* (ub - lb);

        AR_x           = xp(1);
        taper_x        = xp(2);
        sweep_x        = xp(3);
        twist_root_x   = xp(4);
        WS_x           = xp(5);
        xLE_root_x     = xp(6);
        AR_v_x         = xp(7);
        taper_v_x      = xp(8);
        sweep_v_x      = xp(9);
        V_cruise_x     = xp(10);
        Vp_x           = xp(11);
        cb_halfwidth_m = xp(12);
        cb_length_m    = xp(13);

        g_c  = ctx.g;
        roh  = ctx.roh;
        mu_x = ctx.mu_Pas;

        % ---- gross weight estimate for S_ref sizing ----
        VPS_x    = Vp_x / ctx.Vp_ref_m3;
        fe_eff_x = min(ctx.fe_base + ctx.ke * max(0, VPS_x - 1), ctx.fe_max);
        Wg_est   = (ctx.Wp_N + ctx.Wprop_N) / (1 - fe_eff_x);
        if Wg_est > ctx.Wg_max_N; return; end

        % ---- wing geometry ----
        wingIn_x              = ctx.wingIn_base;
        wingIn_x.AR           = AR_x;
        wingIn_x.taper        = taper_x;
        wingIn_x.sweep_c4_deg = sweep_x;
        wingIn_x.xLE_root_m   = xLE_root_x;
        wingIn_x.S_ref_m2     = Wg_est / WS_x;
        wingIn_x.y_root_m     = cb_halfwidth_m;
        wingOut_x             = wingGeometryDesign(wingIn_x);

        if wingOut_x.c_tip_m < ctx.c_tip_min_m; return; end
        if wingOut_x.b_m > ctx.b_max_m;         return; end
        if cb_halfwidth_m >= wingOut_x.semiSpan_m; return; end
        if Vp_x > cb_length_m * 2 * cb_halfwidth_m * ctx.Hf_m; return; end

        % early stall feasibility (conservative CLmax=0.80 placeholder)
        if sqrt(2*Wg_est / (roh * wingOut_x.S_ref_m2 * 0.80)) > ctx.Vs_max_mps * 1.2
            return;
        end

        % ---- airfoil surrogates at cruise Re for new geometry ----
        Re_root = roh * V_cruise_x * wingOut_x.c_root_m / mu_x;
        Re_tip  = roh * V_cruise_x * wingOut_x.c_tip_m  / mu_x;
        rootAF  = evaluateAirfoilSurrogate(ctx.airfoilDB, ctx.airfoilRootName, Re_root);
        tipAF   = evaluateAirfoilSurrogate(ctx.airfoilDB, ctx.airfoilTipName,  Re_tip);

        % ---- twist ----
        CL_des_x           = Wg_est / (0.5*roh*V_cruise_x^2 * wingOut_x.S_ref_m2);
        twistIn_x          = struct();
        twistIn_x.b_m            = wingOut_x.b_m;
        twistIn_x.AR             = wingOut_x.AR;
        twistIn_x.c_root_m       = wingOut_x.c_root_m;
        twistIn_x.c_tip_m        = wingOut_x.c_tip_m;
        twistIn_x.sweep_c4_deg   = wingOut_x.sweep_c4_deg;
        twistIn_x.alphaL0_root_deg = rootAF.alphaL0_deg;
        twistIn_x.alphaL0_tip_deg  = tipAF.alphaL0_deg;
        twistIn_x.Cm_root        = rootAF.Cm0;
        twistIn_x.Cm_tip         = tipAF.Cm0;
        twistIn_x.CL_design      = CL_des_x;
        twistIn_x.static_margin  = 0.10;
        twistIn_x.model          = 'linear';
        twistIn_x.twist_root_deg = twist_root_x;
        twistIn_x.Nspan          = 80;
        twistOut_x               = twistFunctionPanknin(twistIn_x);

        % ---- fin geometry ----
        vertIn_x                 = ctx.vertIn_base;
        vertIn_x.S_ref_m2        = wingOut_x.S_ref_m2;
        vertIn_x.b_w_m           = wingOut_x.b_m;
        vertIn_x.AR_v            = AR_v_x;
        vertIn_x.taper_v         = taper_v_x;
        vertIn_x.sweep_c4_v_deg  = sweep_v_x;
        vertIn_x.x_c4_wing_ref_m = wingOut_x.x_c4_MAC_m;
        vertIn_x.xLE_root_v_m    = wingOut_x.xLE_tip_m;
        vertIn_x.y_root_v_m      = cb_halfwidth_m + wingOut_x.semiSpan_m;
        vertIn_x.z_root_v_m      = ctx.wingIn_base.z_root_m;
        vertOut_x                = verticalSurfaceDesign(vertIn_x);

        if vertOut_x.b_v_m > ctx.b_v_max_frac * wingOut_x.semiSpan_m; return; end

        % ---- structural mass scaling ----
        m_wing_x   = ctx.m_wing_struct_ref_kg ...
                     * (wingOut_x.S_ref_m2 / ctx.S_ref_base_m2) ...
                     * sqrt(AR_x / ctx.AR_base);
        m_vert_x   = ctx.m_vert_struct_ref_kg * (vertOut_x.S_v_total_m2 / ctx.S_fin_base_m2);
        if vertOut_x.isTwin
            m_fin_ea = 0.5 * m_vert_x;
        else
            m_fin_ea = m_vert_x;
        end

        % ---- fuselage mass and geometry scaling ----
        cb_area_x    = cb_length_m * 2 * cb_halfwidth_m;
        cb_area_base = ctx.Lf_m * 2 * ctx.wingIn_base.y_root_m;
        m_fuse_x     = ctx.m_fuse_ref_kg * cb_area_x / cb_area_base;
        fuse_cg_frac = ctx.cadMass.fuselageOnly.cg_m(1) / ctx.Lf_m;
        cadMass_x                      = ctx.cadMass;
        cadMass_x.fuselageOnly.mass_kg = m_fuse_x;
        cadMass_x.fuselageOnly.cg_m    = [fuse_cg_frac * cb_length_m, 0, 0];

        % ---- mass model ----
        comp_x = ctx.compFixed;
        y_root = cb_halfwidth_m;
        z_root = ctx.wingIn_base.z_root_m;
        eta_s  = ctx.eta_servo;

        y_sv = y_root + eta_s * wingOut_x.semiSpan_m;
        x_hg = wingOut_x.xLE_root_m + ...
               (wingOut_x.xLE_tip_m - wingOut_x.xLE_root_m)*eta_s + ...
               0.75*(wingOut_x.c_root_m + (wingOut_x.c_tip_m - wingOut_x.c_root_m)*eta_s);
        comp_x(end+1) = makePointMass('S2 LHS',  0.009, [x_hg, -y_sv, z_root]);
        comp_x(end+1) = makePointMass('S3 RHS',  0.009, [x_hg,  y_sv, z_root]);
        comp_x(end+1) = makePointMass('S1 back', 0.009, [wingOut_x.x_c4_MAC_m+0.02, 0, z_root]);
        comp_x(end+1) = makePointMass('S4 fin',  0.009, ...
            [vertOut_x.xLE_root_v_m + 0.70*vertOut_x.c_root_v_m, ...
             vertOut_x.y_root_v_m, vertOut_x.z_root_v_m + 0.20*vertOut_x.b_v_m]);
        comp_x(end+1) = makePointMass('S5 cargo', 0.009, [0.61980, 0, 0]);

        y_ws = y_root + 0.42*wingOut_x.semiSpan_m;
        comp_x(end+1) = makePointMass('Wstr L', 0.5*m_wing_x, [wingOut_x.x_c4_MAC_m, -y_ws, z_root]);
        comp_x(end+1) = makePointMass('Wstr R', 0.5*m_wing_x, [wingOut_x.x_c4_MAC_m,  y_ws, z_root]);

        x_fs = vertOut_x.xLE_root_v_m + 0.40*vertOut_x.c_root_v_m;
        comp_x(end+1) = makePointMass('Vstr R', m_fin_ea, ...
            [x_fs,  vertOut_x.y_root_v_m, vertOut_x.z_root_v_m + 0.30*vertOut_x.b_v_m]);
        if vertOut_x.isTwin
            comp_x(end+1) = makePointMass('Vstr L', m_fin_ea, ...
                [x_fs, -vertOut_x.y_root_v_m, vertOut_x.z_root_v_m + 0.30*vertOut_x.b_v_m]);
        end

        massIn_x.cadBodies   = cadMass_x;
        massIn_x.pointMasses = comp_x;
        massOut_x            = aircraftMassProperties(massIn_x);
        Wg_actual            = massOut_x.weight_N;

        % ---- drag build-up / aero polar ----
        aeroIn_x                  = struct();
        aeroIn_x.rho_kgm3         = roh;
        aeroIn_x.mu_Pas           = mu_x;
        aeroIn_x.V_cruise_mps     = V_cruise_x;
        aeroIn_x.W_N              = Wg_actual;
        aeroIn_x.Sref_m2          = wingOut_x.S_ref_m2;
        aeroIn_x.AR               = AR_x;
        aeroIn_x.e                = 0.80;
        aeroIn_x.MAC_m            = wingOut_x.MAC_m;
        aeroIn_x.sweepC4_deg      = sweep_x;
        aeroIn_x.Cla_root_per_deg = rootAF.Cla_per_deg;
        aeroIn_x.Cla_tip_per_deg  = tipAF.Cla_per_deg;
        aeroIn_x.alphaL0_root_deg = rootAF.alphaL0_deg;
        aeroIn_x.alphaL0_tip_deg  = tipAF.alphaL0_deg;
        aeroIn_x.Clmax_root       = rootAF.Cl_max;
        aeroIn_x.Clmax_tip        = tipAF.Cl_max;
        aeroIn_x.useDragBuildUp   = true;
        aeroIn_x.CD0_user         = 0.01224;
        aeroIn_x.Swet_wing_m2     = 2.04 * wingOut_x.S_ref_m2;
        % Fuselage wetted area scales with box planform area (L × 2W)
        aeroIn_x.Swet_fuse_m2     = ctx.Swet_fuse_m2 * cb_area_x / cb_area_base;
        aeroIn_x.Swet_fin_m2      = 2.04 * vertOut_x.S_v_total_m2;
        aeroIn_x.Lf_m             = cb_length_m;
        aeroIn_x.Wf_m             = 2 * cb_halfwidth_m;
        aeroIn_x.Hf_m             = ctx.Hf_m;
        aeroIn_x.tc               = ctx.tc;
        aeroIn_x.xc               = ctx.xc;
        aeroIn_x.Q_wing           = ctx.Q_wing;
        aeroIn_x.Q_fuse           = ctx.Q_fuse;
        aeroIn_x.Q_fin            = ctx.Q_fin;
        aeroIn_x.alpha_vec_deg    = -4:2:12;   % coarser grid for speed
        aeroIn_x.plotFigures      = false;
        aeroOut_x                 = aeroPolarAircraft(aeroIn_x);

        CLmax_x = aeroOut_x.CLmax_3D;
        Vs_x    = sqrt(2*Wg_actual / (roh * wingOut_x.S_ref_m2 * CLmax_x));
        if Vs_x > ctx.Vs_max_mps
            Jobj = 1e6 + 100*(Vs_x - ctx.Vs_max_mps)^2; return;
        end
        if V_cruise_x < ctx.Vs_margin_fac * Vs_x; return; end

        % ---- AVL dynamic stability ----
        dynIn_x                  = ctx.dynIn_base;
        dynIn_x.verbose          = false;
        dynIn_x.plotModes        = false;
        dynIn_x.viewGeometry     = false;
        dynIn_x.mass_kg          = massOut_x.mass_kg;
        dynIn_x.Icg_kgm2         = massOut_x.Icg_kgm2;
        dynIn_x.cg_m             = massOut_x.cg_m;
        dynIn_x.S_ref_m2         = wingOut_x.S_ref_m2;
        dynIn_x.MAC_m            = wingOut_x.MAC_m;
        dynIn_x.b_m              = wingOut_x.b_m;
        dynIn_x.xLE_root_m       = xLE_root_x;
        dynIn_x.xLE_tip_m        = wingOut_x.xLE_tip_m;
        dynIn_x.y_root_m         = cb_halfwidth_m;
        dynIn_x.cb_chord_m       = cb_length_m;
        dynIn_x.semiSpan_m       = wingOut_x.semiSpan_m;
        dynIn_x.c_root_m         = wingOut_x.c_root_m;
        dynIn_x.c_tip_m          = wingOut_x.c_tip_m;
        dynIn_x.twist_root_deg   = twistOut_x.twist_root_deg;
        dynIn_x.twist_tip_deg    = twistOut_x.twist_tip_deg;
        dynIn_x.alphaL0_root_deg = rootAF.alphaL0_deg;
        dynIn_x.alphaL0_tip_deg  = tipAF.alphaL0_deg;
        dynIn_x.Cla_root_per_deg = rootAF.Cla_per_deg;
        dynIn_x.Cla_tip_per_deg  = tipAF.Cla_per_deg;
        dynIn_x.V_mps            = V_cruise_x;
        dynIn_x.rho_kgm3         = roh;
        dynIn_x.CD0              = aeroOut_x.CD0;
        dynIn_x.CL_trim          = aeroOut_x.CL_cruise;
        dynIn_x.alpha_trim_deg   = aeroOut_x.alpha_cruise_deg;
        dynIn_x.xLE_root_v_m     = vertOut_x.xLE_root_v_m;
        dynIn_x.y_root_v_m       = vertOut_x.y_root_v_m;
        dynIn_x.z_root_v_m       = vertOut_x.z_root_v_m;
        dynIn_x.xLE_top_v_m      = vertOut_x.xLE_top_v_m;
        dynIn_x.y_top_v_m        = vertOut_x.y_top_v_m;
        dynIn_x.z_top_v_m        = vertOut_x.z_top_v_m;
        dynIn_x.xLE_bottom_v_m   = vertOut_x.xLE_bottom_v_m;
        dynIn_x.y_bottom_v_m     = vertOut_x.y_bottom_v_m;
        dynIn_x.z_bottom_v_m     = vertOut_x.z_bottom_v_m;
        dynIn_x.c_root_v_m       = vertOut_x.c_root_v_m;
        dynIn_x.c_tip_v_m        = vertOut_x.c_tip_v_m;

        dynOut_x = dynamicStabilityAVL(dynIn_x);

        SM_x = dynOut_x.SM_pct;
        sp   = dynOut_x.longModes.shortPeriod.metrics;
        ph   = dynOut_x.longModes.phugoid.metrics;
        dr   = dynOut_x.latModes.dutchRoll.metrics;

        % ---- profit (physics-based, consistent with re-eval block in main.m) ----
        LD_x = aeroOut_x.LD_cruise;
        [~, ~, Ef_raw, ~, ~, ~] = energyCalc(Wg_actual, ctx.eta_p, LD_x, ...
                                              ctx.delta_h_m, ctx.R_cruise_m, ctx.reserve_factor);
        Ef_x = 20 * Ef_raw;
        Tf_x = 20 * ctx.Tf_s;
        J_x  = profitPerUnitTime(ctx.Wp_N, Vp_x, Ef_x, Wg_actual, Tf_x);

        % ---- objective: minimize -J + soft penalties ----
        Jobj = -J_x;

        % SM band (quadratic outside target band)
        if SM_x <= ctx.SM_min_pct
            Jobj = Jobj + 5.0 * (ctx.SM_min_pct - SM_x)^2;
        elseif SM_x >= ctx.SM_max_pct
            Jobj = Jobj + 5.0 * (SM_x - ctx.SM_max_pct)^2;
        end

        % Short-period
        if sp.isComplex
            if sp.zeta < 0,    Jobj = Jobj + 0.50; end   % divergent
            if sp.zeta < 0.25, Jobj = Jobj + 0.15; end   % underdamped
        else
            Jobj = Jobj + 0.25;   % aperiodic SP
        end

        % Phugoid
        if ph.isComplex && ph.zeta < 0
            Jobj = Jobj + 0.10;
        end

        % Dutch roll
        if dr.isComplex && dr.zeta < 0
            Jobj = Jobj + 0.15;
        end

        % Trim authority
        if isfield(dynOut_x, 'controlDerivs') && abs(dynOut_x.controlDerivs.Cmde) > 1e-9
            de_trim_x = -dynOut_x.controlDerivs.Cm0_trim / dynOut_x.controlDerivs.Cmde;
            if abs(de_trim_x) > ctx.de_trim_max_deg
                Jobj = Jobj + 0.05 * (abs(de_trim_x) - ctx.de_trim_max_deg);
            end
        end

        info.J_hr     = J_x * 3600;
        info.SM_pct   = SM_x;
        info.Vs_mps   = Vs_x;
        info.LD       = LD_x;
        info.Wg_g     = Wg_actual / g_c * 1000;
        info.b_m      = wingOut_x.b_m;
        info.AR       = AR_x;
        info.S_ref    = wingOut_x.S_ref_m2;
        info.Vp_L           = Vp_x * 1000;
        info.V_cruise       = V_cruise_x;
        info.cb_halfwidth_m = cb_halfwidth_m;
        info.cb_length_m    = cb_length_m;
        info.failed         = false;

    catch ME
        if ctx.debugObj
            fprintf('[profit_obj] CAUGHT: %s — %s\n', ME.identifier, ME.message);
        end
    end
end
