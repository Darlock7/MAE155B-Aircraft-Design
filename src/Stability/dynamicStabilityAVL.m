function dynOut = dynamicStabilityAVL(dynIn)
% dynamicStabilityAVL
%
% Purpose:
%   Full dynamic stability pipeline for the flying-wing aircraft using AVL.
%   Steps: write AVL files → run AVL → parse ST → dimensionalize (SI) →
%   transform inertia to stability axes → build Along/Alat → eigenanalysis →
%   MIL-STD handling quality assessment.
%
% Inputs (dynIn struct):
%   .mass_kg          scalar [kg]
%   .Icg_kgm2         [3x3] inertia tensor in body axes at CG [kg*m^2]
%   .cg_m             [1x3] CG position [m] (x forward, y right, z up)
%   .S_ref_m2         wing reference area [m^2]
%   .MAC_m            mean aerodynamic chord [m]
%   .b_m              full span [m]
%   .xLE_root_m       wing root LE x [m]
%   .xLE_tip_m        wing tip LE x [m]
%   .y_root_m         wing root y offset from symmetry plane [m]
%   .semiSpan_m       wing semispan (from root to tip) [m]
%   .c_root_m         root chord [m]
%   .c_tip_m          tip chord [m]
%   .eta_cs_start     elevon inboard eta [-]
%   .eta_cs_end       elevon outboard eta [-]
%   .cs_chord_frac    elevon chord fraction [-]
%   .alphaL0_avg_deg  mean zero-lift angle of attack [deg]
%   .V_mps            trim speed [m/s]
%   .rho_kgm3         air density [kg/m^3]
%   .CD0              zero-lift drag coefficient [-]
%   .CL_trim          trim lift coefficient [-]
%   .alpha_trim_deg   trim angle of attack [deg]
%   .xLE_root_v_m     fin root LE x [m]
%   .y_root_v_m       fin root y [m]  (at wing tip)
%   .z_root_v_m       fin root z [m]
%   .xLE_top_v_m      fin top tip LE x [m]
%   .y_top_v_m        fin top tip y [m]
%   .z_top_v_m        fin top tip z [m]
%   .c_root_v_m       fin root chord [m]
%   .c_tip_v_m        fin tip chord [m]
%   .rudder_eta_start rudder inboard eta (fraction of top height) [-]
%   .rudder_eta_end   rudder outboard eta [-]
%   .rudder_cf        rudder chord fraction [-]
%   .avlExe           path to AVL executable (string)
%   .workDir          working directory for temp files (string)
%   .plotModes        logical, generate mode response plots
%
% Outputs (dynOut struct):
%   .Along            [4x4] longitudinal state matrix
%   .Alat             [4x4] lateral state matrix
%   .longModes        struct with shortPeriod, phugoid fields
%   .latModes         struct with dutchRoll, rollSubsidence, spiral fields
%   .derivatives      struct of raw dimensionless AVL derivatives
%   .dimDerivs        struct of dimensionalized SI derivatives
%   .stableFlags      struct of handling quality pass/fail

    arguments
        dynIn struct
    end

    g = 9.81;  % [m/s^2]

    %% ---- unpack ----
    m      = dynIn.mass_kg;
    Icg    = dynIn.Icg_kgm2;
    cg     = dynIn.cg_m(:)';

    Sref   = dynIn.S_ref_m2;
    Cref   = dynIn.MAC_m;
    Bref   = dynIn.b_m;

    xLE_r  = dynIn.xLE_root_m;
    xLE_t  = dynIn.xLE_tip_m;
    y_root = dynIn.y_root_m;
    bHalf  = dynIn.semiSpan_m;
    c_r    = dynIn.c_root_m;
    c_t    = dynIn.c_tip_m;

    eta0   = dynIn.eta_cs_start;
    eta1   = dynIn.eta_cs_end;
    cf     = dynIn.cs_chord_frac;

    alphaL0_deg = dynIn.alphaL0_avg_deg;
    u0     = dynIn.V_mps;
    rho    = dynIn.rho_kgm3;
    CD0    = dynIn.CD0;
    CL0    = dynIn.CL_trim;
    alpha_trim_deg = dynIn.alpha_trim_deg;

    xLE_rv = dynIn.xLE_root_v_m;
    y_rv   = dynIn.y_root_v_m;
    z_rv   = dynIn.z_root_v_m;
    xLE_tv = dynIn.xLE_top_v_m;
    y_tv   = dynIn.y_top_v_m;
    z_tv   = dynIn.z_top_v_m;
    c_rv   = dynIn.c_root_v_m;
    c_tv   = dynIn.c_tip_v_m;
    rud_e0 = dynIn.rudder_eta_start;
    rud_e1 = dynIn.rudder_eta_end;
    rud_cf = dynIn.rudder_cf;

    avlExe  = string(dynIn.avlExe);
    workDir = string(dynIn.workDir);
    doPlot  = dynIn.plotModes;

    %% ---- file paths ----
    geomFile = fullfile(workDir, "fw_dyn.avl");
    massFile = fullfile(workDir, "fw_dyn.mass");
    runFile  = fullfile(workDir, "fw_dyn.run");
    cmdFile  = fullfile(workDir, "fw_dyn_cmd.txt");
    stFile   = fullfile(workDir, "fw_dyn_st.txt");

    %% ===== STEP 1: WRITE AVL GEOMETRY FILE =====
    write_geom(geomFile, cg, Sref, Cref, Bref, ...
        xLE_r, xLE_t, y_root, bHalf, c_r, c_t, alphaL0_deg, ...
        eta0, eta1, cf, ...
        xLE_rv, y_rv, z_rv, xLE_tv, y_tv, z_tv, c_rv, c_tv, ...
        rud_e0, rud_e1, rud_cf);

    %% ===== STEP 2: WRITE MASS FILE (SI) =====
    Ixx = Icg(1,1);  Iyy = Icg(2,2);  Izz = Icg(3,3);
    Ixy = Icg(1,2);  Ixz = Icg(1,3);  Iyz = Icg(2,3);

    fid = fopen(massFile,'w');
    assert(fid > 0, 'dynamicStabilityAVL: cannot write mass file');
    fprintf(fid,"# Flying-wing mass file (SI)\n");
    fprintf(fid,"# Lunit = 1.0 m\n# Munit = 1.0 kg\n# Tunit = 1.0 s\n");
    fprintf(fid,"Lunit = 1.0 m\nMunit = 1.0 kg\nTunit = 1.0 s\n");
    fprintf(fid,"# mass  xcg  ycg  zcg  Ixx  Iyy  Izz  Ixy  Ixz  Iyz\n");
    fprintf(fid,"%.6f  %.6f  %.6f  %.6f  %.6f  %.6f  %.6f  %.6f  %.6f  %.6f\n", ...
        m, cg(1), cg(2), cg(3), Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
    fclose(fid);

    %% ===== STEP 3: WRITE RUN CASE FILE =====
    fid = fopen(runFile,'w');
    assert(fid > 0, 'dynamicStabilityAVL: cannot write run file');
    fprintf(fid," ---------------------------------------------\n");
    fprintf(fid," Run case  1:  cruise trim\n");
    fprintf(fid,"\n");
    fprintf(fid," alpha        ->  alpha        =  %.6f\n", alpha_trim_deg);
    fprintf(fid," beta         ->  beta         =  0.000000\n");
    fprintf(fid," pb/2V        ->  pb/2V        =  0.000000\n");
    fprintf(fid," qc/2V        ->  qc/2V        =  0.000000\n");
    fprintf(fid," rb/2V        ->  rb/2V        =  0.000000\n");
    fprintf(fid," aileron      ->  aileron      =  0.000000\n");
    fprintf(fid," elevator     ->  elevator     =  0.000000\n");
    fprintf(fid," rudder       ->  rudder       =  0.000000\n");
    fprintf(fid,"\n");
    fprintf(fid," alpha        =  %.6f  deg\n", alpha_trim_deg);
    fprintf(fid," velocity     =  %.6f  m/s\n", u0);
    fprintf(fid," density      =  %.6f  kg/m^3\n", rho);
    fprintf(fid," grav.acc.    =  %.6f  m/s^2\n", g);
    fprintf(fid," turn_rad.    =  0.000000  m\n");
    fprintf(fid," load_fac.    =  1.000000\n");
    fprintf(fid," X_cg         =  %.6f  m\n", cg(1));
    fprintf(fid," Y_cg         =  %.6f  m\n", cg(2));
    fprintf(fid," Z_cg         =  %.6f  m\n", cg(3));
    fprintf(fid," mass         =  %.6f  kg\n", m);
    fprintf(fid," Ixx          =  %.6f  kg-m^2\n", Ixx);
    fprintf(fid," Iyy          =  %.6f  kg-m^2\n", Iyy);
    fprintf(fid," Izz          =  %.6f  kg-m^2\n", Izz);
    fprintf(fid," Ixy          =  %.6f  kg-m^2\n", Ixy);
    fprintf(fid," Ixz          =  %.6f  kg-m^2\n", Ixz);
    fprintf(fid," Iyz          =  %.6f  kg-m^2\n", Iyz);
    fclose(fid);

   %% ===== STEP 4: WRITE AVL COMMAND FILE =====
if isfile(stFile), delete(stFile); end

fid = fopen(cmdFile,'w');
assert(fid > 0, 'dynamicStabilityAVL: cannot write command file');

fprintf(fid,"LOAD %s\n", geomFile);
fprintf(fid,"MASS %s\n", massFile);
fprintf(fid,"OPER\n");
fprintf(fid,"A A %.6f\n", alpha_trim_deg);
fprintf(fid,"X\n");
fprintf(fid,"ST\n");
fprintf(fid,"%s\n", stFile);
fprintf(fid,"\n");
fprintf(fid,"QUIT\n");

fclose(fid);

    %% ===== STEP 5: RUN AVL =====
    % If caller passed a path without extension, auto-append .exe on Windows
    if ispc && ~endsWith(avlExe, '.exe')
        avlExe = avlExe + ".exe";
    end
    assert(isfile(avlExe), ...
        ['dynamicStabilityAVL: AVL executable not found: %s\n' ...
         'Windows users: download avl.exe from https://web.mit.edu/drela/Public/web/avl/ ' ...
         'and place it in the AVL/ folder.'], avlExe);

    syscmd = sprintf('"%s" < "%s"', avlExe, cmdFile);
    [status, cmdout] = system(syscmd);

    fprintf('\n--- AVL exit status = %d ---\n', status);
    fprintf('\n--- AVL command output ---\n');
    fprintf('%s\n', cmdout);

    if ~isfile(stFile)
        fprintf('\n--- AVL command file used ---\n');
        type(cmdFile)

        fprintf('\n--- AVL geometry file used ---\n');
        type(geomFile)

        error('dynamicStabilityAVL: ST file not created. AVL may have failed.');
    end

    if status ~= 0
        warning('dynamicStabilityAVL: AVL returned nonzero status (%d), ST file exists, continuing.', status);
    end

    %% ===== STEP 6: PARSE ST FILE =====
    txt = fileread(stFile);
    get = @(pat) avl_getnum(txt, pat);

    CL_avl = get("CLtot =");
    CD_avl = get("CDtot =");
    CLa = get("CLa =");
    CDa = get("CDa =");
    Cma = get("Cma =");
    CLq = get("CLq =");
    CDq = get("CDq =");
    Cmq = get("Cmq =");
    CYb = get("CYb =");
    Clb = get("Clb =");
    Cnb = get("Cnb =");
    CYp = get("CYp =");
    Clp = get("Clp =");
    Cnp = get("Cnp =");
    CYr = get("CYr =");
    Clr = get("Clr =");
    Cnr = get("Cnr =");

    if isnan(CDa)
      CDa = 0.0;
    end

    if isnan(CDq)
        CDq = 0.0;
    end

    % use AVL totals if they look reasonable, else use input values
    if ~isnan(CL_avl) && abs(CL_avl) > 1e-4,  CL0 = CL_avl; end
    if ~isnan(CD_avl) && abs(CD_avl) > 1e-5,  CD0 = CD_avl; end

    derivs.CLa=CLa; derivs.CDa=CDa; derivs.Cma=Cma;
    derivs.CLq=CLq; derivs.CDq=CDq; derivs.Cmq=Cmq;
    derivs.CYb=CYb; derivs.Clb=Clb; derivs.Cnb=Cnb;
    derivs.CYp=CYp; derivs.Clp=Clp; derivs.Cnp=Cnp;
    derivs.CYr=CYr; derivs.Clr=Clr; derivs.Cnr=Cnr;

    fprintf('\n--- Parsed derivatives ---\n');
    fprintf('CL0=%.5f  CD0=%.5f\n', CL0, CD0);
    fprintf('CLa=%.5f  Cma=%.5f  Cmq=%.5f\n', CLa, Cma, Cmq);
    fprintf('CYb=%.5f  Clb=%.5f  Cnb=%.5f\n', CYb, Clb, Cnb);

    %% ===== STEP 7: INERTIA → STABILITY AXES =====
    % Rotate body inertia by alpha_trim (epsilon) about y-axis
    eps = deg2rad(alpha_trim_deg);
    ce = cos(eps);  se = sin(eps);

    Jx  = Ixx*ce^2 + Izz*se^2 - 2*Ixz*se*ce;
    Jy  = Iyy;
    Jz  = Ixx*se^2 + Izz*ce^2 + 2*Ixz*se*ce;
    Jxz = (Ixx - Izz)*se*ce + Ixz*(ce^2 - se^2);

    %% ===== STEP 8: DIMENSIONALIZE (SI) =====
    qbar = 0.5*rho*u0^2;
    theta0 = 0.0;   % level flight trim

    % Longitudinal
    Xu = -rho*u0*Sref*CD0;
    Zu = -rho*u0*Sref*CL0;
    Xw = -(qbar*Sref*CDa) / u0;
    Zw = -(qbar*Sref*CLa) / u0;
    Xq = -(qbar*Sref*CDq*(Cref/(2*u0)));
    Zq = -(qbar*Sref*CLq*(Cref/(2*u0)));
    Mw = (qbar*Sref*Cref*Cma) / u0;
    Mq = qbar*Sref*Cref * (Cmq*(Cref/(2*u0)));
    Mu = 0.0;
    Zw_dot = 0.0;
    Mw_dot = 0.0;

    % Lateral
    Yv = (qbar*Sref*CYb) / u0;
    Lv = (qbar*Sref*Bref*Clb) / u0;
    Nv = (qbar*Sref*Bref*Cnb) / u0;
    Yp = qbar*Sref*(CYp*(Bref/(2*u0)));
    Lp = qbar*Sref*Bref*(Clp*(Bref/(2*u0)));
    Np = qbar*Sref*Bref*(Cnp*(Bref/(2*u0)));
    Yr = qbar*Sref*(CYr*(Bref/(2*u0)));
    Lr = qbar*Sref*Bref*(Clr*(Bref/(2*u0)));
    Nr = qbar*Sref*Bref*(Cnr*(Bref/(2*u0)));

    dimD.Xu=Xu; dimD.Zu=Zu; dimD.Xw=Xw; dimD.Zw=Zw;
    dimD.Xq=Xq; dimD.Zq=Zq; dimD.Mw=Mw; dimD.Mq=Mq;
    dimD.Yv=Yv; dimD.Lv=Lv; dimD.Nv=Nv;
    dimD.Yp=Yp; dimD.Lp=Lp; dimD.Np=Np;
    dimD.Yr=Yr; dimD.Lr=Lr; dimD.Nr=Nr;

    %% ===== STEP 9: BUILD STATE MATRICES =====
    % Longitudinal: states [du, w, q, dtheta]
    den = m - Zw_dot;

    Along = zeros(4,4);
    Along(1,1) = Xu/m;
    Along(1,2) = Xw/m;
    Along(1,3) = Xq/m;
    Along(1,4) = -g*cos(theta0);
    Along(2,1) = Zu/den;
    Along(2,2) = Zw/den;
    Along(2,3) = (Zq + m*u0)/den;
    Along(2,4) = -(m*g*sin(theta0))/den;
    Along(3,1) = (1/Jy)*(Mu + Mw_dot*Zu/den);
    Along(3,2) = (1/Jy)*(Mw + Mw_dot*Zw/den);
    Along(3,3) = (1/Jy)*(Mq + Mw_dot*(Zq + m*u0)/den);
    Along(4,3) = 1;

    % Lateral: states [v, p, r, phi]
    Dval  = Jx*Jz - Jxz^2;
    Jx_p  = Dval/Jz;
    Jz_p  = Dval/Jx;
    Jzx_p = Jxz/Dval;

    Alat = zeros(4,4);
    Alat(1,1) = Yv/m;
    Alat(1,2) = Yp/m;
    Alat(1,3) = (Yr/m) - u0;
    Alat(1,4) = g*cos(theta0);
    Alat(2,1) = Lv/Jx_p + Jzx_p*Nv;
    Alat(2,2) = Lp/Jx_p + Jzx_p*Np;
    Alat(2,3) = Lr/Jx_p + Jzx_p*Nr;
    Alat(3,1) = Jzx_p*Lv + Nv/Jz_p;
    Alat(3,2) = Jzx_p*Lp + Np/Jz_p;
    Alat(3,3) = Jzx_p*Lr + Nr/Jz_p;
    Alat(4,2) = 1;
    Alat(4,3) = tan(theta0);

    %% ===== STEP 10: EIGENANALYSIS + MODE CLASSIFICATION =====
    [VL, DL] = eig(Along);
    lamLong  = diag(DL);
    [VT, DT] = eig(Alat);
    lamLat   = diag(DT);
     
    fprintf('\n--- Longitudinal eigenvalues ---\n');
    disp(lamLong)

    fprintf('\n--- Lateral eigenvalues ---\n');
    disp(lamLat)

    longModes = classify_long_modes(lamLong, VL);
    latModes  = classify_lat_modes(lamLat, VT);

    %% ===== STEP 11: PRINT HANDLING QUALITY REPORT =====
    fprintf('\n================ LONGITUDINAL HANDLING QUALITY =================\n');
    print_long_hq(longModes);
    fprintf('\n================ LATERAL HANDLING QUALITY ======================\n');
    print_lat_hq(latModes);

    %% ===== STEP 12: OPTIONAL MODE PLOTS =====
    %% ===== STEP 12: OPTIONAL MODE PLOTS =====
    fprintf('\n--- Plot flag check ---\n');
    fprintf('doPlot = %d\n', doPlot);
    fprintf('Reached plotting section.\n');
    
    if doPlot
        fprintf('Generating dynamic stability plots...\n');
        set(0,'DefaultFigureVisible','on');
    
        sysLong = ss(Along, zeros(4,1), eye(4), zeros(4,1));
        sysLat  = ss(Alat,  zeros(4,1), eye(4), zeros(4,1));
    
        plot_mode(sysLong, linspace(0,10,2001), ...
            longModes.shortPeriod.vec, longModes.shortPeriod.lambda, ...
            ["du","w","q","dtheta"], "Short Period", true);
    
        plot_mode(sysLong, linspace(0,120,6001), ...
            longModes.phugoid.vec, longModes.phugoid.lambda, ...
            ["du","w","q","dtheta"], "Phugoid", false);
    
        plot_mode(sysLat, linspace(0,30,3001), ...
            latModes.dutchRoll.vec, latModes.dutchRoll.lambda, ...
            ["v","p","r","phi"], "Dutch Roll", true);
    
        plot_mode(sysLat, linspace(0,8,1601), ...
            latModes.rollSubsidence.vec, latModes.rollSubsidence.lambda, ...
            ["v","p","r","phi"], "Roll Subsidence", false);
    
        plot_mode(sysLat, linspace(0,60,3001), ...
            latModes.spiral.vec, latModes.spiral.lambda, ...
            ["v","p","r","phi"], "Spiral", false);
    
        drawnow;
    end

    %% ===== PACK OUTPUT =====
    dynOut.Along       = Along;
    dynOut.Alat        = Alat;
    dynOut.longModes   = longModes;
    dynOut.latModes    = latModes;
    dynOut.derivatives = derivs;
    dynOut.dimDerivs   = dimD;
    dynOut.Jx          = Jx;
    dynOut.Jy          = Jy;
    dynOut.Jz          = Jz;
    dynOut.Jxz         = Jxz;
    dynOut.qbar_Pa     = qbar;
    dynOut.stFile      = stFile;
end

%% ========================================================================
%  GEOMETRY WRITER
%% ========================================================================
function write_geom(fname, cg, Sref, Cref, Bref, ...
        xLE_r, xLE_t, y_root, bHalf, c_r, c_t, alphaL0_deg, ...
        eta0, eta1, cf, ...
        xLE_rv, y_rv, z_rv, xLE_tv, y_tv, z_tv, c_rv, c_tv, ...
        rud_e0, rud_e1, rud_cf)

    Xcg = cg(1);  Ycg = cg(2);  Zcg = cg(3);

    % Wing tip coordinates (assuming linear LE sweep, no dihedral)
    %xLE_tip_wing = xLE_r + (xLE_tv - xLE_rv);   % rough proxy; use actual tip
    % Recompute using direct geometry: tip LE comes from wingOut
    % We use the passed-in xLE_tv only for the fin. For wing we derive from
    % the span stations below.

    % Wing chord at each spanwise station (linear taper)
    y0 = y_root;                      % root station (inner edge of panel)
    y1 = y_root + eta0*bHalf;         % elevon inner edge
    y2 = y_root + eta1*bHalf;         % elevon outer edge
    y3 = y_root + bHalf;              % tip

    % LE x at each station (linear sweep from root LE to tip LE)
    lerp = @(eta) xLE_r + eta*(xLE_t - xLE_r);  % xLE_rv is tip LE (fin root = wing tip)
    xw0 = lerp(0);
    xw1 = lerp(eta0);
    xw2 = lerp(eta1);
    xw3 = lerp(1.0);

    % chord at each station
    cw0 = c_r + (c_t - c_r)*0;
    cw1 = c_r + (c_t - c_r)*eta0;
    cw2 = c_r + (c_t - c_r)*eta1;
    cw3 = c_t;

    % hinge x (as fraction of chord → local chord fraction of LE position)
    xh_elev  = 1.0 - cf;   % hinge at (1-cf) fraction for elevator (symmetric)
    xh_ail   = 1.0 - cf;   % same fraction for aileron

    % Fin geometry via eta (0=root, 1=top tip)
    % Fin root = wing tip, fin top = vertOut top
    % For right fin: y increases toward fin tip, z increases (vertical up)
    % We model root → top only (topFrac = 0.66 dominates stabilizing area)

    %fin_b = sqrt((y_tv - y_rv)^2 + (z_tv - z_rv)^2);  % fin span length

    % rudder hinge stations
    y_rud0_R = y_rv + rud_e0*(y_tv - y_rv);
    z_rud0_R = z_rv + rud_e0*(z_tv - z_rv);
    x_rud0_R = xLE_rv + rud_e0*(xLE_tv - xLE_rv);
    c_rud0   = c_rv + rud_e0*(c_tv - c_rv);

    y_rud1_R = y_rv + rud_e1*(y_tv - y_rv);
    z_rud1_R = z_rv + rud_e1*(z_tv - z_rv);
    x_rud1_R = xLE_rv + rud_e1*(xLE_tv - xLE_rv);
    c_rud1   = c_rv + rud_e1*(c_tv - c_rv);

    xh_rud = 1.0 - rud_cf;

    fid = fopen(fname,'w');
    assert(fid > 0, 'dynamicStabilityAVL: cannot write geometry file');

    fprintf(fid,"Flying-Wing Dynamic Stability (generated by MATLAB)\n");
    fprintf(fid,"0.0\n");
    fprintf(fid,"0 0 0.0\n");
    fprintf(fid,"%.6f  %.6f  %.6f\n", Sref, Cref, Bref);
    fprintf(fid,"%.6f  %.6f  %.6f\n", Xcg, Ycg, Zcg);
    fprintf(fid,"0.0\n\n");

    % ---- Wing (YDUPLICATE mirrors about y=0) ----
    fprintf(fid,"SURFACE\nWing\n");
    fprintf(fid,"12 1.0 20 1.0\n");
    fprintf(fid,"YDUPLICATE\n0.0\n\n");

    % Section 0: root (inner edge, no control)
    fprintf(fid,"SECTION\n");
    fprintf(fid,"%.6f  %.6f  0.000000  %.6f  %.6f\n", xw0, y0, cw0, alphaL0_deg);
    fprintf(fid,"NACA\n0012\n\n");

    % Section 1: elevon inboard edge (elevator + aileron split here)
    fprintf(fid,"SECTION\n");
    fprintf(fid,"%.6f  %.6f  0.000000  %.6f  %.6f\n", xw1, y1, cw1, alphaL0_deg);
    fprintf(fid,"NACA\n0012\n\n");

    % Section 2: elevon outboard edge
    fprintf(fid,"SECTION\n");
    fprintf(fid,"%.6f  %.6f  0.000000  %.6f  %.6f\n", xw2, y2, cw2, alphaL0_deg);
    fprintf(fid,"NACA\n0012\n\n");
    fprintf(fid,"CONTROL\n");
    fprintf(fid,"elevator  1.000  %.3f  0 0 0  1.0\n\n", xh_elev);
    fprintf(fid,"CONTROL\n");
    fprintf(fid,"aileron   1.000  %.3f  0 0 0  -1.0\n\n", xh_ail);

    % Section 3: tip
    fprintf(fid,"SECTION\n");
    fprintf(fid,"%.6f  %.6f  0.000000  %.6f  %.6f\n", xw3, y3, cw3, alphaL0_deg);
    fprintf(fid,"NACA\n0012\n\n");
    fprintf(fid,"CONTROL\n");
    fprintf(fid,"elevator  1.000  %.3f  0 0 0  1.0\n\n", xh_elev);
    fprintf(fid,"CONTROL\n");
    fprintf(fid,"aileron   1.000  %.3f  0 0 0  -1.0\n\n", xh_ail);

    % ---- Right fin ----
    fprintf(fid,"SURFACE\nVfin_R\n");
    fprintf(fid,"8 1.0 12 1.0\n\n");

    fprintf(fid,"SECTION\n");
    fprintf(fid,"%.6f  %.6f  %.6f  %.6f  0.000000\n", xLE_rv, y_rv, z_rv, c_rv);
    fprintf(fid,"NACA\n0010\n\n");

    % rudder start
    fprintf(fid,"SECTION\n");
    fprintf(fid,"%.6f  %.6f  %.6f  %.6f  0.000000\n", x_rud0_R, y_rud0_R, z_rud0_R, c_rud0);
    fprintf(fid,"NACA\n0010\n\n");

    % rudder end / tip
    fprintf(fid,"SECTION\n");
    fprintf(fid,"%.6f  %.6f  %.6f  %.6f  0.000000\n", x_rud1_R, y_rud1_R, z_rud1_R, c_rud1);
    fprintf(fid,"NACA\n0010\n\n");
    fprintf(fid,"CONTROL\n");
    fprintf(fid,"rudder  1.000  %.3f  0 0 0  1.0\n\n", xh_rud);

    % fin tip
    fprintf(fid,"SECTION\n");
    fprintf(fid,"%.6f  %.6f  %.6f  %.6f  0.000000\n", xLE_tv, y_tv, z_tv, c_tv);
    fprintf(fid,"NACA\n0010\n\n");
    fprintf(fid,"CONTROL\n");
    fprintf(fid,"rudder  1.000  %.3f  0 0 0  1.0\n\n", xh_rud);

    % ---- Left fin (mirror in y) ----
    fprintf(fid,"SURFACE\nVfin_L\n");
    fprintf(fid,"8 1.0 12 1.0\n\n");

    fprintf(fid,"SECTION\n");
    fprintf(fid,"%.6f  %.6f  %.6f  %.6f  0.000000\n", xLE_rv, -y_rv, z_rv, c_rv);
    fprintf(fid,"NACA\n0010\n\n");

    fprintf(fid,"SECTION\n");
    fprintf(fid,"%.6f  %.6f  %.6f  %.6f  0.000000\n", x_rud0_R, -y_rud0_R, z_rud0_R, c_rud0);
    fprintf(fid,"NACA\n0010\n\n");

    fprintf(fid,"SECTION\n");
    fprintf(fid,"%.6f  %.6f  %.6f  %.6f  0.000000\n", x_rud1_R, -y_rud1_R, z_rud1_R, c_rud1);
    fprintf(fid,"NACA\n0010\n\n");
    fprintf(fid,"CONTROL\n");
    fprintf(fid,"rudder  -1.000  %.3f  0 0 0  1.0\n\n", xh_rud);

    fprintf(fid,"SECTION\n");
    fprintf(fid,"%.6f  %.6f  %.6f  %.6f  0.000000\n", xLE_tv, -y_tv, z_tv, c_tv);
    fprintf(fid,"NACA\n0010\n\n");
    fprintf(fid,"CONTROL\n");
    fprintf(fid,"rudder  -1.000  %.3f  0 0 0  1.0\n\n", xh_rud);

    fclose(fid);
end

%% ========================================================================
%  ST FILE NUMBER PARSER
%% ========================================================================
function val = avl_getnum(txt, pattern)
    expr = pattern + "\s*([-+]?(\d+(\.\d*)?|\.\d+)([eEdD][-+]?\d+)?)";
    tok  = regexp(txt, expr, 'tokens', 'once');
    if isempty(tok)
        warning('dynamicStabilityAVL: pattern not found: %s', pattern);
        val = NaN;
        return;
    end
    val = str2double(strrep(tok{1},'D','E'));
end

%% ========================================================================
%  MODE METRICS
%% ========================================================================
function m = mode_metrics(lam)
    m.lambda = lam;
    m.sigma  = real(lam);
    m.omega  = imag(lam);
    if abs(imag(lam)) > 1e-8
        m.isComplex = true;
        m.wn   = hypot(real(lam), imag(lam));
        m.zeta = -real(lam)/m.wn;
        m.T    = 2*pi/abs(imag(lam));
        m.tau  = 1/abs(real(lam));
        if real(lam) < 0
            m.ts2    = 4/abs(real(lam));
            m.tDouble = NaN;
            m.tHalf  = NaN;
        else
            m.ts2    = NaN;
            m.tDouble = log(2)/real(lam);
            m.tHalf  = NaN;
        end
    else
        m.isComplex = false;
        m.wn   = NaN;
        m.zeta = NaN;
        m.T    = NaN;
        m.tau  = 1/max(abs(real(lam)), 1e-12);
        if real(lam) < 0
            m.tHalf  = log(2)/abs(real(lam));
            m.tDouble = NaN;
        elseif real(lam) > 1e-10
            m.tDouble = log(2)/real(lam);
            m.tHalf  = NaN;
        else
            m.tDouble = Inf;
            m.tHalf  = Inf;
        end
        m.ts2 = NaN;
    end
end

%% =======================================================================
%  MODE CLASSIFIERS
%% ========================================================================

function lm = classify_long_modes(lam, evecs)
    idxC = find(abs(imag(lam)) > 1e-8);
    idxR = find(abs(imag(lam)) <= 1e-8);

    lm.allEigenvalues = lam;

    if numel(idxC) >= 2
        lamC = lam(idxC);
        vecC = evecs(:, idxC);
        [~, ord] = sort(abs(imag(lamC)), 'descend');

        lm.shortPeriod.lambda  = lamC(ord(1));
        lm.shortPeriod.vec     = vecC(:, ord(1));
        lm.shortPeriod.metrics = mode_metrics(lamC(ord(1)));

        lm.phugoid.lambda  = lamC(ord(end));
        lm.phugoid.vec     = vecC(:, ord(end));
        lm.phugoid.metrics = mode_metrics(lamC(ord(end)));
    else
        warning('dynamicStabilityAVL: longitudinal modes are not two clean complex pairs. Classifying by eigenvalue magnitude.');

        [~, ord] = sort(abs(lam), 'descend');

        lm.shortPeriod.lambda  = lam(ord(1));
        lm.shortPeriod.vec     = evecs(:, ord(1));
        lm.shortPeriod.metrics = mode_metrics(lam(ord(1)));

        lm.phugoid.lambda  = lam(ord(end));
        lm.phugoid.vec     = evecs(:, ord(end));
        lm.phugoid.metrics = mode_metrics(lam(ord(end)));
    end
end

function lm = classify_lat_modes(lam, evecs)
    idxC = find(abs(imag(lam)) > 1e-8);
    idxR = find(abs(imag(lam)) <= 1e-8);
    assert(~isempty(idxC), 'dynamicStabilityAVL: no complex lateral eigenvalue (Dutch roll missing)');
    assert(numel(idxR) >= 2, 'dynamicStabilityAVL: expected 2 real lateral eigenvalues');

    lamC = lam(idxC);
    vecC = evecs(:, idxC);
    [~, iDR] = max(abs(imag(lamC)));
    lm.dutchRoll.lambda  = lamC(iDR);
    lm.dutchRoll.vec     = vecC(:, iDR);
    lm.dutchRoll.metrics = mode_metrics(lamC(iDR));

    lamR = real(lam(idxR));
    vecR = evecs(:, idxR);
    [~, iRoll] = min(lamR);
    lm.rollSubsidence.lambda  = lamR(iRoll);
    lm.rollSubsidence.vec     = vecR(:, iRoll);
    lm.rollSubsidence.metrics = mode_metrics(lamR(iRoll));

    tmp = lamR;  tmp(iRoll) = Inf;
    [~, iSp] = min(abs(tmp));
    lm.spiral.lambda  = lamR(iSp);
    lm.spiral.vec     = evecs(:, idxR(iSp));
    lm.spiral.metrics = mode_metrics(lamR(iSp));
end

%% ========================================================================
%  HANDLING QUALITY PRINTERS
%% ========================================================================
function print_long_hq(lm)
    sp = lm.shortPeriod.metrics;
    ph = lm.phugoid.metrics;

    fprintf('Short Period:\n');
    fprintf('  lambda = %+.4f %+.4fi\n', real(sp.lambda), imag(sp.lambda));
    fprintf('  wn     = %.4f rad/s\n', sp.wn);
    fprintf('  zeta   = %.4f\n', sp.zeta);
    fprintf('  period = %.4f s\n', sp.T);
    fprintf('  ts2%%   = %.4f s\n', sp.ts2);
    if sp.zeta >= 0.35 && sp.zeta <= 1.30
        lvl = 'Level 1 (zeta 0.35-1.30, Phase A/C)';
    elseif sp.zeta > 0
        lvl = 'Stable but outside Level 1 zeta band';
    else
        lvl = 'UNSTABLE';
    end
    fprintf('  HQ     = %s\n\n', lvl);

    fprintf('Phugoid:\n');
    fprintf('  lambda = %+.4f %+.4fi\n', real(ph.lambda), imag(ph.lambda));
    fprintf('  wn     = %.4f rad/s\n', ph.wn);
    fprintf('  zeta   = %.4f\n', ph.zeta);
    fprintf('  period = %.4f s\n', ph.T);
    if ph.zeta >= 0.04
        lvl = 'Level 1 (zeta >= 0.04)';
    elseif ph.zeta > 0
        lvl = 'Stable but below Level 1 zeta=0.04';
    else
        lvl = 'UNSTABLE';
    end
    fprintf('  HQ     = %s\n', lvl);
end

function print_lat_hq(lm)
    dr = lm.dutchRoll.metrics;
    rs = lm.rollSubsidence.metrics;
    sp = lm.spiral.metrics;

    fprintf('Dutch Roll:\n');
    fprintf('  lambda   = %+.4f %+.4fi\n', real(dr.lambda), imag(dr.lambda));
    fprintf('  wn       = %.4f rad/s\n', dr.wn);
    fprintf('  zeta     = %.4f\n', dr.zeta);
    fprintf('  zeta*wn  = %.4f rad/s\n', dr.zeta*dr.wn);
    % MIL-F-8785C Class I, Level 1: zeta>=0.19, zeta*wn>=0.35, wn>=1.0
    if dr.zeta >= 0.19 && (dr.zeta*dr.wn) >= 0.35 && dr.wn >= 1.0
        lvl = 'Level 1 (Class I: zeta>=0.19, z*wn>=0.35, wn>=1.0)';
    elseif dr.zeta >= 0.08 && (dr.zeta*dr.wn) >= 0.15 && dr.wn >= 0.4
        lvl = 'Level 2 first-cut';
    elseif dr.zeta > 0
        lvl = 'Stable but below Level 2 Dutch-roll criterion';
    else
        lvl = 'UNSTABLE';
    end
    fprintf('  HQ       = %s\n\n', lvl);

    fprintf('Roll Subsidence:\n');
    fprintf('  lambda   = %+.6f\n', rs.lambda);
    fprintf('  tau      = %.4f s\n', rs.tau);
    fprintf('  t_half   = %.4f s\n', rs.tHalf);
    % MIL-F-8785C Class I, Level 1: tau_roll <= 1.0 s
    if rs.tau <= 1.0
        lvl = 'Level 1 (tau <= 1.0 s)';
    elseif rs.tau <= 1.4
        lvl = 'Level 2 (tau <= 1.4 s)';
    else
        lvl = 'Below Level 2 roll-mode criterion';
    end
    fprintf('  HQ       = %s\n\n', lvl);

    fprintf('Spiral Mode:\n');
    fprintf('  lambda   = %+.6f\n', sp.lambda);
    fprintf('  tau      = %.4f s\n', sp.tau);
    if real(sp.lambda) > 1e-10
        fprintf('  t_double = %.4f s\n', sp.tDouble);
        % MIL-F-8785C Level 1: t_double >= 20 s
        if sp.tDouble >= 20
            lvl = 'Level 1 (t_double >= 20 s)';
        elseif sp.tDouble >= 8
            lvl = 'Level 2 (t_double >= 8 s)';
        else
            lvl = 'Below Level 2 spiral criterion';
        end
    else
        fprintf('  t_half   = %.4f s\n', sp.tHalf);
        lvl = 'Stable spiral (always acceptable)';
    end
    fprintf('  HQ       = %s\n', lvl);
end

%% ========================================================================
%  MODE RESPONSE PLOTTER
%% ========================================================================
function plot_mode(sys, t, vec, lam, stateNames, modeName, isComplex)
    x0 = real(vec);
    if norm(x0) < 1e-10, x0 = imag(vec); end
    if norm(x0) < 1e-10, x0 = ones(length(vec),1); end
    x0 = x0 / norm(x0);

    [y, tt] = initial(sys, x0, t);

    figure('Color','w','Name', modeName);
    plot(tt, y, 'LineWidth', 1.25);
    grid on;
    xlabel('t (s)');
    ylabel('State response');
    title(modeName);
    legend(stateNames, 'Location','best');

    m = mode_metrics(lam);
    if isComplex && ~isnan(m.wn)
        ann = sprintf('%s\n\\lambda=%+.3f%+.3fi\nwn=%.3f rad/s\nzeta=%.3f\nT=%.3f s', ...
            modeName, real(lam), imag(lam), m.wn, m.zeta, m.T);
    else
        ann = sprintf('%s\n\\lambda=%+.4f\ntau=%.3f s', modeName, real(lam), m.tau);
    end
    annotation('textbox',[0.14 0.68 0.28 0.24], 'String', ann, ...
        'FitBoxToText','on','BackgroundColor','w','EdgeColor',[0.3 0.3 0.3]);
end
