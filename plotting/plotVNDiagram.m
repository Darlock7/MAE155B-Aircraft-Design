function vnOut = plotVNDiagram(vnIn)
% plotVNdiagram
% First-pass maneuver V-n diagram for CTOL aircraft

    %% ---------------------------
    % defaults / checks
    %% ---------------------------
    if ~isfield(vnIn,'CLmax_neg')
        vnIn.CLmax_neg = -0.8 * vnIn.CLmax_pos;
    end
    if ~isfield(vnIn,'plotUnits')
        vnIn.plotUnits = 'mph';
    end
    if ~isfield(vnIn,'Npts')
        vnIn.Npts = 500;
    end
    if ~isfield(vnIn,'makeFigure')
        vnIn.makeFigure = true;
    end

    requiredFields = {'rho','W_N','S_ref_m2','CLmax_pos','n_pos_limit','n_neg_limit','Vc_mps','Vd_mps'};
    for k = 1:numel(requiredFields)
        if ~isfield(vnIn, requiredFields{k})
            error('plotVNdiagram:MissingField', 'Missing required field: %s', requiredFields{k});
        end
    end

    %% ---------------------------
    % unpack inputs
    %% ---------------------------
    rho         = vnIn.rho;
    W           = vnIn.W_N;
    S           = vnIn.S_ref_m2;
    CLmax_pos   = vnIn.CLmax_pos;
    CLmax_neg   = vnIn.CLmax_neg;
    n_pos_limit = vnIn.n_pos_limit;
    n_neg_limit = vnIn.n_neg_limit;
    Vc          = vnIn.Vc_mps;
    Vd          = vnIn.Vd_mps;
    Npts        = vnIn.Npts;

    %% ---------------------------
    % characteristic speeds
    %% ---------------------------
    Vs_pos = sqrt(2*W / (rho*S*CLmax_pos));
    Vs_neg = sqrt(2*W / (rho*S*abs(CLmax_neg)));

    Va   = Vs_pos * sqrt(n_pos_limit);
    Vneg = Vs_neg * sqrt(abs(n_neg_limit));

    %% ---------------------------
    % speed grid + stall curves
    %% ---------------------------
    V = linspace(0, Vd, Npts);

    n_stall_pos = 0.5 * rho .* V.^2 * S * CLmax_pos / W;
    n_stall_neg = -0.5 * rho .* V.^2 * S * abs(CLmax_neg) / W;

    %% ---------------------------
    % output struct
    %% ---------------------------
    vnOut = struct();
    vnOut.Vs_pos_mps   = Vs_pos;
    vnOut.Vs_neg_mps   = Vs_neg;
    vnOut.Va_mps       = Va;
    vnOut.Vneg_mps     = Vneg;
    vnOut.Vc_mps       = Vc;
    vnOut.Vd_mps       = Vd;
    vnOut.V_vec_mps    = V;
    vnOut.n_stall_pos  = n_stall_pos;
    vnOut.n_stall_neg  = n_stall_neg;
    vnOut.n_pos_limit  = n_pos_limit;
    vnOut.n_neg_limit  = n_neg_limit;
    vnOut.CLmax_pos    = CLmax_pos;
    vnOut.CLmax_neg    = CLmax_neg;

    %% ---------------------------
    % plotting unit conversion
    %% ---------------------------
    switch lower(vnIn.plotUnits)
        case 'mph'
            speedScale = 2.2369362920544;
            speedLabel = 'Velocity [mph]';
        case 'mps'
            speedScale = 1.0;
            speedLabel = 'Velocity [m/s]';
        otherwise
            error('plotVNdiagram:BadUnits', 'plotUnits must be ''mph'' or ''mps''.')
    end

    vnOut.speedScale = speedScale;

    %% ---------------------------
    % make plot
    %% ---------------------------
if vnIn.makeFigure
    figure('Name','V-n Diagram (Maneuver Envelope)','NumberTitle','off');
    hold on;

    helperColor = [0.7 0.7 0.7];
    ptColor     = [0.2 0.6 0.9];

    % curves
    V_pos_curve = linspace(0, Va, 250);
    n_pos_curve = 0.5 * rho .* V_pos_curve.^2 * S * CLmax_pos / W;

    V_neg_curve = linspace(0, Vneg, 250);
    n_neg_curve = -0.5 * rho .* V_neg_curve.^2 * S * abs(CLmax_neg) / W;

    % main envelope
    plot(V_pos_curve*speedScale, n_pos_curve, 'k', 'LineWidth', 1.8);
    plot([Va Vd]*speedScale, [n_pos_limit n_pos_limit], 'k', 'LineWidth', 1.8);
    plot([Vd Vd]*speedScale, [n_neg_limit n_pos_limit], 'k', 'LineWidth', 1.8);
    plot([Vneg Vd]*speedScale, [n_neg_limit n_neg_limit], 'k', 'LineWidth', 1.8);
    plot(V_neg_curve*speedScale, n_neg_curve, 'k', 'LineWidth', 1.8);

    % helper curves
    plot(V*speedScale, n_stall_pos, '--', 'Color', helperColor, 'LineWidth', 0.8);
    plot(V*speedScale, n_stall_neg, '--', 'Color', helperColor, 'LineWidth', 0.8);

    xline(Vc*speedScale, '--', 'Color', [0.75 0.75 0.75], 'LineWidth', 0.8);

    % optional structural helper lines
    plot([0 Vd]*speedScale, [n_pos_limit n_pos_limit], '--', 'Color', helperColor, 'LineWidth', 0.8);
    plot([0 Vd]*speedScale, [n_neg_limit n_neg_limit], '--', 'Color', helperColor, 'LineWidth', 0.8);

% Gust overlay
if isfield(vnIn,'gust') && isfield(vnIn.gust,'enable') && vnIn.gust.enable
    Vg_plot = vnIn.gust.V_pts_mps * speedScale;

    plot(Vg_plot, vnIn.gust.n_pos, '--', ...
        'Color', [0.7 0.7 0.7], 'LineWidth', 1.0);

    plot(Vg_plot, vnIn.gust.n_neg, '--', ...
        'Color', [0.7 0.7 0.7], 'LineWidth', 1.0);
end

    % key points
    plot(Va*speedScale, n_pos_limit, 'o', 'MarkerSize', 4, 'MarkerFaceColor', ptColor, 'MarkerEdgeColor', 'k');
    plot(Vd*speedScale, n_pos_limit, 'o', 'MarkerSize', 4, 'MarkerFaceColor', ptColor, 'MarkerEdgeColor', 'k');
    plot(Vneg*speedScale, n_neg_limit, 'o', 'MarkerSize', 4, 'MarkerFaceColor', ptColor, 'MarkerEdgeColor', 'k');
    plot(Vd*speedScale, n_neg_limit, 'o', 'MarkerSize', 4, 'MarkerFaceColor', ptColor, 'MarkerEdgeColor', 'k');

    xlabel(speedLabel);
    ylabel('Load Factor n');
    xlim([0, 1.08*Vd*speedScale]);
    ylim([n_neg_limit - 0.3, n_pos_limit + 0.3]);

    grid off;
    box off;
    set(gca,'FontSize',11,'LineWidth',0.8);

    % manual labels AFTER axes limits set
    xl = xlim;
    yl = ylim;
    dx = xl(2) - xl(1);
    dy = yl(2) - yl(1);

    ySpeed = yl(1) + 0.06*dy;
    xStruct = xl(1) + 0.02*dx;

    text(Vc*speedScale, ySpeed, 'V_C', ...
    'HorizontalAlignment','center', 'VerticalAlignment','bottom');

    % critical speed labels (horizontal)
    
    text(Va*speedScale + 0.6, n_pos_limit - 0.08, 'V_A', ...
        'HorizontalAlignment','left', 'VerticalAlignment','top');
    text(Vd*speedScale + 0.6, 0.5*(n_pos_limit+n_neg_limit), 'V_D', ...
        'HorizontalAlignment','left', 'VerticalAlignment','middle');

    % structural limit labels
    text(xStruct, n_pos_limit + 0.03, 'n_{max}', ...
        'HorizontalAlignment','left', 'VerticalAlignment','bottom');
    text(xStruct, n_neg_limit - 0.03, 'n_{min}', ...
        'HorizontalAlignment','left', 'VerticalAlignment','top');

    % point labels
    text(Va*speedScale - 1.2, n_pos_limit + 0.03, 'PHAA', ...
        'HorizontalAlignment','right', 'VerticalAlignment','bottom');
    text(Vd*speedScale - 1.2, n_pos_limit + 0.03, 'PLAA', ...
        'HorizontalAlignment','right', 'VerticalAlignment','bottom');
    text(Vneg*speedScale - 1.2, n_neg_limit - 0.03, 'NHAA', ...
        'HorizontalAlignment','right', 'VerticalAlignment','top');
    text(Vd*speedScale - 1.2, n_neg_limit - 0.03, 'NLAA', ...
        'HorizontalAlignment','right', 'VerticalAlignment','top');

    hold off;
end