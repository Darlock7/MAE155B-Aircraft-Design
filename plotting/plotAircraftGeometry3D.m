function plotAircraftGeometry3D(geom3DIn)
% plotAircraftGeometry3D
% Clean 3D aircraft geometry visualization
%
% Geometry convention:
%   - Supply RIGHT wing geometry only; function mirrors about y = 0.
%   - ALL coordinates passed in must be ABSOLUTE aircraft coordinates.
%   - The vertical surface data in geom3DIn.vertOut must also already be
%     in ABSOLUTE aircraft coordinates.
%
% Optional features:
%   - Loaded / unloaded CG markers
%   - Point-mass component markers and labels

    arguments
        geom3DIn struct
    end

    %% ---------------- Required checks ----------------
    req = { ...
        'c_root_m','c_tip_m', ...
        'xLE_root_m','y_root_m','z_root_m', ...
        'xLE_tip_m','yLE_tip_m','zLE_tip_m', ...
        'xLE_MAC_m','y_MAC_m','MAC_m', ...
        'twist_root_deg','twist_tip_deg'};

    for k = 1:numel(req)
        if ~isfield(geom3DIn, req{k})
            error('plotAircraftGeometry3D:MissingField', ...
                'Missing required input field: %s', req{k});
        end
    end

    %% ---------------- Defaults ----------------
    if ~isfield(geom3DIn,'plotVertical');        geom3DIn.plotVertical = false; end
    if ~isfield(geom3DIn,'plotBody');            geom3DIn.plotBody = false; end
    if ~isfield(geom3DIn,'plotCG');              geom3DIn.plotCG = false; end
    if ~isfield(geom3DIn,'plotComponents');      geom3DIn.plotComponents = false; end
    if ~isfield(geom3DIn,'plotComponentLabels'); geom3DIn.plotComponentLabels = false; end

    if ~isfield(geom3DIn,'z_MAC_m')
        geom3DIn.z_MAC_m = geom3DIn.z_root_m;
    end

    %% ---------------- Pull variables ----------------
    c_root = geom3DIn.c_root_m;
    c_tip  = geom3DIn.c_tip_m;

    xLE_root = geom3DIn.xLE_root_m;
    y_root   = geom3DIn.y_root_m;
    z_root   = geom3DIn.z_root_m;

    xLE_tip = geom3DIn.xLE_tip_m;
    y_tip   = geom3DIn.yLE_tip_m;
    z_tip   = geom3DIn.zLE_tip_m;

    xLE_MAC = geom3DIn.xLE_MAC_m;
    y_MAC   = geom3DIn.y_MAC_m;
    z_MAC   = geom3DIn.z_MAC_m;
    MAC     = geom3DIn.MAC_m;

    %% ---------------- Figure ----------------
    figure; hold on; grid on; axis equal;
    view(35,24);
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    title('3D Aircraft Geometry with Loaded / Unloaded CG');

    %% ---------------- Wing planform outlines ----------------
    % Right wing
    P1 = [xLE_root,          y_root, z_root];
    P2 = [xLE_root + c_root, y_root, z_root];
    P3 = [xLE_tip  + c_tip,  y_tip,  z_tip];
    P4 = [xLE_tip,           y_tip,  z_tip];

    % Left wing mirror
    P1L = [xLE_root,          -y_root, z_root];
    P2L = [xLE_root + c_root, -y_root, z_root];
    P3L = [xLE_tip  + c_tip,  -y_tip,  z_tip];
    P4L = [xLE_tip,           -y_tip,  z_tip];

    plot3([P1(1) P2(1) P3(1) P4(1) P1(1)], ...
          [P1(2) P2(2) P3(2) P4(2) P1(2)], ...
          [P1(3) P2(3) P3(3) P4(3) P1(3)], ...
          'k','LineWidth',1.8);

    plot3([P1L(1) P2L(1) P3L(1) P4L(1) P1L(1)], ...
          [P1L(2) P2L(2) P3L(2) P4L(2) P1L(2)], ...
          [P1L(3) P2L(3) P3L(3) P4L(3) P1L(3)], ...
          'k','LineWidth',1.8);

    %% ---------------- Twisted chord lines ----------------
    plotTwistedChord([xLE_root,  y_root,  z_root], c_root, geom3DIn.twist_root_deg,'b-');
    plotTwistedChord([xLE_root, -y_root,  z_root], c_root, geom3DIn.twist_root_deg,'b-');

    plotTwistedChord([xLE_tip,   y_tip,   z_tip],  c_tip,  geom3DIn.twist_tip_deg,'c-');
    plotTwistedChord([xLE_tip,  -y_tip,   z_tip],  c_tip,  geom3DIn.twist_tip_deg,'c-');

    %% ---------------- MAC ----------------
    plot3([xLE_MAC xLE_MAC+MAC],[ y_MAC  y_MAC],[z_MAC z_MAC], ...
        'r','LineWidth',3);
    plot3([xLE_MAC xLE_MAC+MAC],[-y_MAC -y_MAC],[z_MAC z_MAC], ...
        'r','LineWidth',3);

    %% ---------------- Body / package ----------------
    if geom3DIn.plotBody
        reqBody = {'bodyLength_m','bodyWidth_m','bodyHeight_m','xBody_m','yBody_m','zBody_m'};
        for k = 1:numel(reqBody)
            if ~isfield(geom3DIn, reqBody{k})
                error('plotAircraftGeometry3D:MissingBodyField', ...
                    'Missing required body field: %s', reqBody{k});
            end
        end

        L = geom3DIn.bodyLength_m;
        W = geom3DIn.bodyWidth_m;
        H = geom3DIn.bodyHeight_m;

        x_c = geom3DIn.xBody_m;
        y_c = geom3DIn.yBody_m;
        z_c = geom3DIn.zBody_m;

        x0 = x_c - L/2;
        y0 = y_c - W/2;
        z0 = z_c - H/2;

        X = [0 1 1 0 0 1 1 0]*L + x0;
        Y = [0 0 1 1 0 0 1 1]*W + y0;
        Z = [0 0 0 0 1 1 1 1]*H + z0;

        faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];

        patch('Vertices',[X' Y' Z'], ...
              'Faces',faces, ...
              'FaceColor',[0.3 0.3 0.8], ...
              'FaceAlpha',0.15, ...
              'EdgeColor','none');
    end

    %% ---------------- Vertical surfaces ----------------
    if geom3DIn.plotVertical
        if ~isfield(geom3DIn,'vertOut')
            error('plotAircraftGeometry3D:MissingVertOut', ...
                'plotVertical=true but geom3DIn.vertOut was not provided.');
        end
        drawVerticalSurfacePairAbsolute(geom3DIn.vertOut);
    end

    %% ---------------- Component points ----------------
    if geom3DIn.plotComponents
        if ~isfield(geom3DIn,'components')
            error('plotAircraftGeometry3D:MissingComponents', ...
                'plotComponents=true but geom3DIn.components was not provided.');
        end

        comps = geom3DIn.components;
        for i = 1:numel(comps)
            r = comps(i).r_m;
            plot3(r(1), r(2), r(3), 'ko', ...
                'MarkerFaceColor', [0.2 0.2 0.2], ...
                'MarkerSize', 5);

            if geom3DIn.plotComponentLabels
                text(r(1), r(2), r(3), ['  ' comps(i).name], ...
                    'FontSize', 8, 'Interpreter','none');
            end
        end
    end

    %% ---------------- CG markers ----------------
    if geom3DIn.plotCG

        if isfield(geom3DIn,'xCG_loaded_m')
            plot3(geom3DIn.xCG_loaded_m, geom3DIn.yCG_loaded_m, geom3DIn.zCG_loaded_m, ...
                'ro','MarkerFaceColor','r','MarkerSize',8);
            text(geom3DIn.xCG_loaded_m, geom3DIn.yCG_loaded_m, geom3DIn.zCG_loaded_m, ...
                '  CG loaded','Color','r','FontSize',9);
        elseif isfield(geom3DIn,'xCG_m')
            plot3(geom3DIn.xCG_m, geom3DIn.yCG_m, geom3DIn.zCG_m, ...
                'ro','MarkerFaceColor','r','MarkerSize',8);
            text(geom3DIn.xCG_m, geom3DIn.yCG_m, geom3DIn.zCG_m, ...
                '  CG','Color','r','FontSize',9);
        end

        if isfield(geom3DIn,'xCG_unloaded_m')
            plot3(geom3DIn.xCG_unloaded_m, geom3DIn.yCG_unloaded_m, geom3DIn.zCG_unloaded_m, ...
                'ms','MarkerFaceColor','m','MarkerSize',8);
            text(geom3DIn.xCG_unloaded_m, geom3DIn.yCG_unloaded_m, geom3DIn.zCG_unloaded_m, ...
                '  CG unloaded','Color','m','FontSize',9);
        end
    end

    %% ---------------- Legend proxies ----------------
    legendHandles = [];
    legendEntries = {};

    hWing = plot3(nan,nan,nan,'k','LineWidth',1.8);
    legendHandles(end+1) = hWing; %#ok<AGROW>
    legendEntries{end+1} = 'Wing planform'; %#ok<AGROW>

    hMAC = plot3(nan,nan,nan,'r','LineWidth',3);
    legendHandles(end+1) = hMAC; %#ok<AGROW>
    legendEntries{end+1} = 'MAC'; %#ok<AGROW>

    if geom3DIn.plotVertical
        hVert = plot3(nan,nan,nan,'g','LineWidth',2);
        legendHandles(end+1) = hVert; %#ok<AGROW>
        legendEntries{end+1} = 'Vertical surface'; %#ok<AGROW>
    end

    if geom3DIn.plotComponents
        hComp = plot3(nan,nan,nan,'ko','MarkerFaceColor',[0.2 0.2 0.2],'MarkerSize',5);
        legendHandles(end+1) = hComp; %#ok<AGROW>
        legendEntries{end+1} = 'Components'; %#ok<AGROW>
    end

    if geom3DIn.plotCG
        if isfield(geom3DIn,'xCG_loaded_m')
            hLoaded = plot3(nan,nan,nan,'ro','MarkerFaceColor','r','MarkerSize',8);
            legendHandles(end+1) = hLoaded; %#ok<AGROW>
            legendEntries{end+1} = 'CG loaded'; %#ok<AGROW>
        end
        if isfield(geom3DIn,'xCG_unloaded_m')
            hUnloaded = plot3(nan,nan,nan,'ms','MarkerFaceColor','m','MarkerSize',8);
            legendHandles(end+1) = hUnloaded; %#ok<AGROW>
            legendEntries{end+1} = 'CG unloaded'; %#ok<AGROW>
        end
    end

    legend(legendHandles, legendEntries, 'Location','best');
    hold off;
end

%% ================= Helper Functions =================

function plotTwistedChord(LE, chord, twist_deg, style)
    theta = deg2rad(twist_deg);
    x1 = LE(1) + chord*cos(theta);
    z1 = LE(3) - chord*sin(theta);
    plot3([LE(1) x1], [LE(2) LE(2)], [LE(3) z1], style, 'LineWidth', 2);
end

function drawVerticalSurfacePairAbsolute(vertOut)

    req = { ...
        'xLE_root_v_m','y_root_v_m','z_root_v_m', ...
        'xLE_top_v_m','y_top_v_m','z_top_v_m', ...
        'xLE_bottom_v_m','y_bottom_v_m','z_bottom_v_m', ...
        'c_root_v_m','c_tip_v_m'};

    for k = 1:numel(req)
        if ~isfield(vertOut, req{k})
            error('drawVerticalSurfacePairAbsolute:MissingField', ...
                'Missing vertOut field: %s', req{k});
        end
    end

    xr = vertOut.xLE_root_v_m;
    yr = vertOut.y_root_v_m;
    zr = vertOut.z_root_v_m;

    xt = vertOut.xLE_top_v_m;
    yt = vertOut.y_top_v_m;
    zt = vertOut.z_top_v_m;

    xb = vertOut.xLE_bottom_v_m;
    yb = vertOut.y_bottom_v_m;
    zb = vertOut.z_bottom_v_m;

    cr = vertOut.c_root_v_m;
    ct = vertOut.c_tip_v_m;

    % Right
    R_LE = [xr,     yr, zr];
    R_TE = [xr+cr,  yr, zr];
    T_LE = [xt,     yt, zt];
    T_TE = [xt+ct,  yt, zt];
    B_LE = [xb,     yb, zb];
    B_TE = [xb+ct,  yb, zb];

    plotPanel(R_LE, R_TE, T_TE, T_LE);
    plotPanel(B_LE, B_TE, R_TE, R_LE);

    % Left
    R_LE_L = [xr,    -yr, zr];
    R_TE_L = [xr+cr, -yr, zr];
    T_LE_L = [xt,    -yt, zt];
    T_TE_L = [xt+ct, -yt, zt];
    B_LE_L = [xb,    -yb, zb];
    B_TE_L = [xb+ct, -yb, zb];

    plotPanel(R_LE_L, R_TE_L, T_TE_L, T_LE_L);
    plotPanel(B_LE_L, B_TE_L, R_TE_L, R_LE_L);
end

function plotPanel(A,B,C,D)
    plot3([A(1) B(1) C(1) D(1) A(1)], ...
          [A(2) B(2) C(2) D(2) A(2)], ...
          [A(3) B(3) C(3) D(3) A(3)], ...
          'g','LineWidth',2);
end