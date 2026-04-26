function csOut = controlSurfaceSizing(csIn)
% controlSurfaceSizing
%
% Purpose:
%   Elevon and rudder maneuvering authority check from AVL control derivatives.
%   Computes trim deflection, max load factor, min turn radius, and roll rate.
%
% Inputs (csIn struct):
%   .CLde        [/deg]  elevator CL effectiveness  (CLd01 from AVL)
%   .Cmde        [/deg]  elevator Cm effectiveness  (Cmd01 from AVL)
%   .Clda        [/deg]  aileron  Cl effectiveness  (Cld02 from AVL)
%   .Cnda        [/deg]  aileron adverse yaw        (Cnd02 from AVL)
%   .Cndr        [/deg]  rudder yaw effectiveness   (Cnd03 from AVL)
%   .Cm0_trim    [-]     pitching moment at δe=0    (Cmtot from AVL)
%   .CL_trim     [-]     cruise lift coefficient
%   .CLmax       [-]     airfoil stall CL (upper bound on CL_turn)
%   .V_mps       [m/s]   cruise speed
%   .rho_kgm3    [kg/m³] air density
%   .S_ref_m2    [m²]    wing reference area
%   .b_m         [m]     wingspan
%   .mass_kg     [kg]    aircraft mass
%   .Clp         [/rad]  roll damping (from AVL, negative)
%   .delta_e_max [deg]   max elevon deflection (default 20)
%   .delta_a_max [deg]   max aileron deflection (default 20)
%   .delta_r_max [deg]   max rudder deflection (default 25)
%   .showPlots   bool    generate turn-radius plot
%
% Outputs (csOut struct):
%   .delta_e_trim_deg  [deg]   trim elevon deflection
%   .n_max             [-]     max load factor (pitch authority)
%   .phi_max_deg       [deg]   max bank angle
%   .R_min_m           [m]     minimum turn radius
%   .phi_deg           [deg]   bank angle array (0 → phi_max)
%   .R_m               [m]     turn radius array
%   .delta_e_deg       [deg]   required elevon at each bank angle
%   .p_ss_dps          [°/s]   steady-state roll rate at delta_a_max

    if ~isfield(csIn,'delta_e_max'), csIn.delta_e_max = 20;    end
    if ~isfield(csIn,'delta_a_max'), csIn.delta_a_max = 20;    end
    if ~isfield(csIn,'delta_r_max'), csIn.delta_r_max = 25;    end
    if ~isfield(csIn,'showPlots'),   csIn.showPlots   = false; end

    CLde    = csIn.CLde;
    Cmde    = csIn.Cmde;
    Clda    = abs(csIn.Clda);   % magnitude — sign depends on which wing AVL picked
    Cnda    = csIn.Cnda;
    Cndr    = csIn.Cndr;
    Cm0     = csIn.Cm0_trim;
    CL_c    = csIn.CL_trim;
    CLmax   = csIn.CLmax;
    V       = csIn.V_mps;
    rho     = csIn.rho_kgm3;
    S       = csIn.S_ref_m2;
    b       = csIn.b_m;
    Clp     = csIn.Clp;         % /rad, negative (roll damping)
    de_max  = csIn.delta_e_max;
    da_max  = csIn.delta_a_max;
    dr_max  = csIn.delta_r_max;
    g       = 9.81;

    % ---- trim elevon deflection ----
    % Cm0 + Cmde * de_trim = 0
    de_trim = -Cm0 / Cmde;

    % ---- pitch authority → max load factor ----
    % CL limited by either elevon saturation or airfoil stall
    dCL_elevon = CLde * (de_max - de_trim);   % CL available from full up-pull
    CL_max_elev  = CL_c + dCL_elevon;
    CL_max_stall = CLmax;
    CL_max       = min(CL_max_elev, CL_max_stall);
    n_max        = CL_max / CL_c;

    limited_by = 'elevon';
    if CL_max_stall < CL_max_elev
        limited_by = 'stall';
    end

    % ---- turn performance curve ----
    phi_max_deg = acosd(1 / n_max);
    phi_arr     = linspace(0, phi_max_deg * 0.999, 200);
    n_arr       = 1 ./ cosd(phi_arr);
    CL_arr      = CL_c * n_arr;
    de_arr      = de_trim + (CL_arr - CL_c) / CLde;
    R_arr       = V^2 ./ (g * tand(phi_arr));
    R_arr(phi_arr < 0.5) = Inf;

    R_min_m = V^2 / (g * tand(phi_max_deg));

    % ---- roll authority ----
    % Steady-state roll rate: Clda*da + Clp*(p*b/2V) = 0
    % => p_ss = -2V * Clda * da / (Clp * b)   [rad/s]
    p_ss_rads = 2 * V * Clda * da_max / (abs(Clp) * b);
    p_ss_dps  = p_ss_rads * 180 / pi;

    % ---- rudder yaw authority ----
    % Yaw moment coefficient at max rudder:  Cn_rud = Cndr * dr_max
    Cn_rud = abs(Cndr) * dr_max;
    qbar   = 0.5 * rho * V^2;
    N_rud  = Cn_rud * qbar * S * b;   % [N·m] yaw moment at max rudder

    % ---- print ----
    fprintf('\n============= CONTROL SURFACE SIZING ================\n');
    fprintf('--- Elevon (pitch / roll) ---\n');
    fprintf('  Trim deflection          = %+.2f deg  (+ = TE down)\n', de_trim);
    fprintf('  Max deflection           = +/-%.0f deg\n', de_max);
    fprintf('  CL increment at de_max   = %.4f\n', dCL_elevon);
    fprintf('  Max load factor          = %.2f g  (limited by %s)\n', n_max, limited_by);
    fprintf('  Max bank angle           = %.1f deg\n', phi_max_deg);
    fprintf('  Minimum turn radius      = %.1f m  (at V=%.0f m/s)\n', R_min_m, V);
    fprintf('--- Aileron ---\n');
    fprintf('  Steady-state roll rate   = %.1f deg/s  (at da=%.0f deg)\n', p_ss_dps, da_max);
    fprintf('  Adverse yaw (Cnda*da)    = %.5f  (small = good)\n', Cnda * da_max);
    fprintf('--- Rudder ---\n');
    fprintf('  Max yaw moment           = %.4f N*m  (at dr=%.0f deg)\n', N_rud, dr_max);
    fprintf('  Cndr (per deg)           = %.6f\n', Cndr);
    fprintf('======================================================\n\n');

    % ---- turn radius plot ----
    if csIn.showPlots
        figure('Name','Turn Performance');
        idx = isfinite(R_arr);
        plot(phi_arr(idx), R_arr(idx), 'b-', 'LineWidth', 2); hold on;
        plot(phi_max_deg, R_min_m, 'ro', 'MarkerSize', 10, 'MarkerFaceColor','r');
        xline(phi_max_deg, 'r--', sprintf('  Max bank = %.1f°  (R_{min}=%.0f m)', ...
            phi_max_deg, R_min_m), 'LabelVerticalAlignment','bottom');
        xlabel('Bank angle [deg]');
        ylabel('Turn radius [m]');
        title('Turn Radius vs Bank Angle');
        ylim([0, min(400, max(R_arr(idx)))]);
        grid on; box on;
    end

    csOut.delta_e_trim_deg = de_trim;
    csOut.n_max            = n_max;
    csOut.phi_max_deg      = phi_max_deg;
    csOut.R_min_m          = R_min_m;
    csOut.phi_deg          = phi_arr;
    csOut.R_m              = R_arr;
    csOut.delta_e_deg      = de_arr;
    csOut.p_ss_dps         = p_ss_dps;
    csOut.N_rud_Nm         = N_rud;
    csOut.limited_by       = limited_by;
end
