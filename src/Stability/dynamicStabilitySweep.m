function sweepOut = dynamicStabilitySweep(sweepIn)
% dynamicStabilitySweep
%
% Purpose:
%   Random-sample parameter sweep over wing and fin geometry to explore
%   dynamic stability space. Mass properties (CG, inertia) are recomputed
%   for every sample so that CG shift from geometry changes is captured.
%   Calls dynamicStabilityAVL for each sample.
%
% Inputs (sweepIn struct):
%   .wingIn            baseline wingIn struct
%   .twistIn           baseline twistIn struct
%   .vertIn            baseline vertIn struct
%   .dynIn             baseline dynIn struct (flags, airfoil paths, flight cond.)
%   .maxIter           number of samples [-]
%   .wingSweep_range   [lo hi] wing quarter-chord sweep [deg]
%   .wingTaper_range   [lo hi] wing taper ratio [-]
%   .twistRoot_range   [lo hi] root geometric twist [deg]
%   .AR_v_range        [lo hi] fin aspect ratio [-]
%   .taperV_range      [lo hi] fin taper ratio [-]
%   .sweepV_range      [lo hi] fin quarter-chord sweep [deg]
%   .xLE_root_range    [lo hi] wing LE root x-position on fuselage [m]
%   .cadMass           fixed fuselage CAD body struct (passed to aircraftMassProperties)
%   .compFixed         point-mass array for non-geometry components
%                      (motor, prop, ESC, battery, receiver, payload)
%   .eta_servo         span fraction for wing servo placement [-]
%   .m_wing_struct_kg  lumped wing structural mass [kg]
%   .m_vert_struct_kg  lumped vertical fin structural mass [kg]
%
% Outputs (sweepOut struct):
%   .results     struct array (maxIter x 1)
%   .bestIdx     index of iteration with highest SM
%   .bestSM_pct  best static margin [%MAC]

    arguments
        sweepIn struct
    end

    wingIn  = sweepIn.wingIn;
    twistIn = sweepIn.twistIn;
    vertIn  = sweepIn.vertIn;
    dynIn   = sweepIn.dynIn;
    maxIter = sweepIn.maxIter;

    % sweep bounds
    lo = [sweepIn.wingSweep_range(1), sweepIn.wingTaper_range(1), sweepIn.twistRoot_range(1), ...
          sweepIn.AR_v_range(1),      sweepIn.taperV_range(1),    sweepIn.sweepV_range(1), ...
          sweepIn.xLE_root_range(1)];
    hi = [sweepIn.wingSweep_range(2), sweepIn.wingTaper_range(2), sweepIn.twistRoot_range(2), ...
          sweepIn.AR_v_range(2),      sweepIn.taperV_range(2),    sweepIn.sweepV_range(2), ...
          sweepIn.xLE_root_range(2)];

    rng(42);
    params = lo + rand(maxIter, 7) .* (hi - lo);

    emptyRes = struct( ...
        'wingSweep', NaN, 'wingTaper', NaN, 'twistRoot', NaN, ...
        'AR_v',      NaN, 'taperV',    NaN, 'sweepV',    NaN, 'xLE_root', NaN, ...
        'SM_pct',    NaN, 'Xcg_m',     NaN, ...
        'sp_wn',     NaN, 'sp_zeta',   NaN, ...
        'ph_zeta',   NaN, 'dr_wn',     NaN, 'dr_zeta',   NaN, ...
        'failed',    true);
    results = repmat(emptyRes, maxIter, 1);

    fprintf('\n===== DYNAMIC STABILITY SWEEP (%d iterations) =====\n', maxIter);
    fprintf('%-4s %-7s %-6s %-7s %-5s %-6s %-6s %-6s | %-6s %-7s %-7s %-7s %-7s\n', ...
        'Iter','Sweep','Taper','TwstR','AR_v','Lam_v','Sw_v','xLErt', ...
        'Xcg','SM%','SP_wn','SP_z','PH_z');

    for k = 1:maxIter
        try
            wingSweep_k  = params(k,1);
            taper_k      = params(k,2);
            twistRoot_k  = params(k,3);
            AR_v_k       = params(k,4);
            taperV_k     = params(k,5);
            sweepV_k     = params(k,6);
            xLE_root_k   = params(k,7);

            % --- wing ---
            wingIn_k              = wingIn;
            wingIn_k.sweep_c4_deg = wingSweep_k;
            wingIn_k.taper        = taper_k;
            wingIn_k.xLE_root_m   = xLE_root_k;
            wingOut_k             = wingGeometryDesign(wingIn_k);

            % --- twist ---
            twistIn_k                = twistIn;
            twistIn_k.b_m            = wingOut_k.b_m;
            twistIn_k.AR             = wingOut_k.AR;
            twistIn_k.c_root_m       = wingOut_k.c_root_m;
            twistIn_k.c_tip_m        = wingOut_k.c_tip_m;
            twistIn_k.sweep_c4_deg   = wingOut_k.sweep_c4_deg;
            twistIn_k.twist_root_deg = twistRoot_k;
            twistOut_k               = twistFunctionPanknin(twistIn_k);

            % --- fins ---
            vertIn_k                = vertIn;
            vertIn_k.AR_v           = AR_v_k;
            vertIn_k.taper_v        = taperV_k;
            vertIn_k.sweep_c4_v_deg = sweepV_k;
            vertIn_k.xLE_root_v_m   = wingOut_k.xLE_tip_m;
            vertIn_k.y_root_v_m     = wingIn_k.y_root_m + wingOut_k.semiSpan_m;
            vertOut_k               = verticalSurfaceDesign(vertIn_k);

            % --- rebuild mass properties with updated geometry ---
            x_c4_MAC_k = wingOut_k.x_c4_MAC_m;
            eta_s      = sweepIn.eta_servo;

            comp_k = sweepIn.compFixed;   % motor, prop, ESC, battery, receiver, payload

            % cargo bay servo (fixed position)
            comp_k(end+1) = makePointMass('S5 Servo cargo bay', 0.009, [0.61980000, 0.000, 0.000]);

            % wing servos
            y_servo_k = wingIn_k.y_root_m + eta_s * wingOut_k.semiSpan_m;
            x_hinge_k = wingOut_k.xLE_root_m + ...
                (wingOut_k.xLE_tip_m - wingOut_k.xLE_root_m)*eta_s + ...
                0.75*(wingOut_k.c_root_m + (wingOut_k.c_tip_m - wingOut_k.c_root_m)*eta_s);
            comp_k(end+1) = makePointMass('S2 Servo LHS wing', 0.009, [x_hinge_k, -y_servo_k, wingIn_k.z_root_m]);
            comp_k(end+1) = makePointMass('S3 Servo RHS wing', 0.009, [x_hinge_k,  y_servo_k, wingIn_k.z_root_m]);
            comp_k(end+1) = makePointMass('S1 Servo back wing', 0.009, [x_c4_MAC_k + 0.020, 0.000, wingIn_k.z_root_m]);

            % vertical fin servo
            comp_k(end+1) = makePointMass('S4 Servo vert stab', 0.009, ...
                [vertOut_k.xLE_root_v_m + 0.70*vertOut_k.c_root_v_m, ...
                 vertOut_k.y_root_v_m, ...
                 vertOut_k.z_root_v_m + 0.20*vertOut_k.b_v_m]);

            % wing structural mass
            y_ws_k = wingIn_k.y_root_m + 0.42*wingOut_k.semiSpan_m;
            comp_k(end+1) = makePointMass('Wing structure L', 0.5*sweepIn.m_wing_struct_kg, ...
                [x_c4_MAC_k, -y_ws_k, wingIn_k.z_root_m]);
            comp_k(end+1) = makePointMass('Wing structure R', 0.5*sweepIn.m_wing_struct_kg, ...
                [x_c4_MAC_k,  y_ws_k, wingIn_k.z_root_m]);

            % fin structural mass
            if vertOut_k.isTwin
                m_fin_k = 0.5 * sweepIn.m_vert_struct_kg;
            else
                m_fin_k = sweepIn.m_vert_struct_kg;
            end
            x_fin_k = vertOut_k.xLE_root_v_m + 0.40*vertOut_k.c_root_v_m;
            y_fin_k = vertOut_k.y_root_v_m;
            z_fin_k = vertOut_k.z_root_v_m + 0.30*vertOut_k.b_v_m;
            comp_k(end+1) = makePointMass('Vertical structure R', m_fin_k, [ x_fin_k,  y_fin_k, z_fin_k]);
            if vertOut_k.isTwin
                comp_k(end+1) = makePointMass('Vertical structure L', m_fin_k, [x_fin_k, -y_fin_k, z_fin_k]);
            end

            massIn_k.cadBodies   = sweepIn.cadMass;
            massIn_k.pointMasses = comp_k;
            massOut_k            = aircraftMassProperties(massIn_k);

            % --- update dynIn ---
            dynIn_k             = dynIn;
            dynIn_k.mass_kg     = massOut_k.mass_kg;
            dynIn_k.Icg_kgm2    = massOut_k.Icg_kgm2;
            dynIn_k.cg_m        = massOut_k.cg_m;
            dynIn_k.S_ref_m2    = wingOut_k.S_ref_m2;
            dynIn_k.MAC_m       = wingOut_k.MAC_m;
            dynIn_k.b_m         = wingOut_k.b_m;
            dynIn_k.xLE_root_m  = wingOut_k.xLE_root_m;
            dynIn_k.xLE_tip_m   = wingOut_k.xLE_tip_m;
            dynIn_k.semiSpan_m  = wingOut_k.semiSpan_m;
            dynIn_k.c_root_m    = wingOut_k.c_root_m;
            dynIn_k.c_tip_m     = wingOut_k.c_tip_m;
            dynIn_k.twist_root_deg = twistOut_k.twist_root_deg;
            dynIn_k.twist_tip_deg  = twistOut_k.twist_tip_deg;
            dynIn_k.xLE_root_v_m   = vertOut_k.xLE_root_v_m;
            dynIn_k.y_root_v_m     = vertOut_k.y_root_v_m;
            dynIn_k.z_root_v_m     = vertOut_k.z_root_v_m;
            dynIn_k.xLE_top_v_m    = vertOut_k.xLE_top_v_m;
            dynIn_k.y_top_v_m      = vertOut_k.y_top_v_m;
            dynIn_k.z_top_v_m      = vertOut_k.z_top_v_m;
            dynIn_k.xLE_bottom_v_m = vertOut_k.xLE_bottom_v_m;
            dynIn_k.y_bottom_v_m   = vertOut_k.y_bottom_v_m;
            dynIn_k.z_bottom_v_m   = vertOut_k.z_bottom_v_m;
            dynIn_k.c_root_v_m     = vertOut_k.c_root_v_m;
            dynIn_k.c_tip_v_m      = vertOut_k.c_tip_v_m;

            dynOut_k = dynamicStabilityAVL(dynIn_k);

            sp = dynOut_k.longModes.shortPeriod.metrics;
            ph = dynOut_k.longModes.phugoid.metrics;
            dr = dynOut_k.latModes.dutchRoll.metrics;

            results(k).wingSweep = wingSweep_k;
            results(k).wingTaper = taper_k;
            results(k).twistRoot = twistRoot_k;
            results(k).AR_v      = AR_v_k;
            results(k).taperV    = taperV_k;
            results(k).sweepV    = sweepV_k;
            results(k).xLE_root  = xLE_root_k;
            results(k).SM_pct    = dynOut_k.SM_pct;
            results(k).Xcg_m     = massOut_k.cg_m(1);
            results(k).sp_wn     = sp.wn;
            results(k).sp_zeta   = sp.zeta;
            results(k).ph_zeta   = ph.zeta;
            results(k).dr_wn     = dr.wn;
            results(k).dr_zeta   = dr.zeta;
            results(k).failed    = false;

            fprintf('%-4d %-7.1f %-6.3f %-7.2f %-5.2f %-6.3f %-6.1f %-6.4f | %-6.4f %-7.2f %-7.3f %-7.3f %-7.3f\n', ...
                k, wingSweep_k, taper_k, twistRoot_k, AR_v_k, taperV_k, sweepV_k, xLE_root_k, ...
                massOut_k.cg_m(1), dynOut_k.SM_pct, sp.wn, sp.zeta, ph.zeta);

        catch ME
            fprintf('Iter %-3d FAILED: %s\n', k, ME.message);
        end
    end

    SM_vals = [results.SM_pct];
    SM_vals(isnan(SM_vals)) = -Inf;
    [bestSM, iBest] = max(SM_vals);

    fprintf('\nBest SM = %.2f %%MAC  (iter %d)  Xcg = %.4f m\n', ...
        bestSM, iBest, results(iBest).Xcg_m);
    fprintf('  wingSweep=%.1f deg  taper=%.3f  twistRoot=%.2f deg\n', ...
        results(iBest).wingSweep, results(iBest).wingTaper, results(iBest).twistRoot);
    fprintf('  AR_v=%.2f  taper_v=%.3f  sweep_v=%.1f deg\n', ...
        results(iBest).AR_v, results(iBest).taperV, results(iBest).sweepV);
    fprintf('  xLE_root=%.4f m\n', results(iBest).xLE_root);
    fprintf('====================================================\n\n');

    sweepOut.results    = results;
    sweepOut.bestIdx    = iBest;
    sweepOut.bestSM_pct = bestSM;
end
