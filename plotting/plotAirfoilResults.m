function plotAirfoilResults(airfoilOut)
% plotAirfoilResults
%
% Plots key airfoil characteristics for root and tip

    root = airfoilOut.root;
    tip  = airfoilOut.tip;

    %% -------- CL vs alpha --------
    figure;
    plot(root.alpha_deg, root.CL, 'LineWidth',2); hold on;
    plot(tip.alpha_deg,  tip.CL,  'LineWidth',2);
    grid on;
    xlabel('\alpha (deg)');
    ylabel('C_L');
    title('Lift Curve');
    legend(['Root: ' root.name], ['Tip: ' tip.name], 'Location','Best');

    %% -------- CD vs alpha --------
    figure;
    plot(root.alpha_deg, root.CD, 'LineWidth',2); hold on;
    plot(tip.alpha_deg,  tip.CD,  'LineWidth',2);
    grid on;
    xlabel('\alpha (deg)');
    ylabel('C_D');
    title('Drag Curve');
    legend(['Root: ' root.name], ['Tip: ' tip.name], 'Location','Best');

    %% -------- CM vs alpha (CRITICAL) --------
    figure;
    plot(root.alpha_deg, root.CM, 'LineWidth',2); hold on;
    plot(tip.alpha_deg,  tip.CM,  'LineWidth',2);
    grid on;
    xlabel('\alpha (deg)');
    ylabel('C_m');
    title('Moment Curve (Pitching Moment)');
    legend(['Root: ' root.name], ['Tip: ' tip.name], 'Location','Best');

    %% -------- L/D vs alpha --------
    figure;
    LD_root = root.CL ./ root.CD;
    LD_tip  = tip.CL  ./ tip.CD;

    plot(root.alpha_deg, LD_root, 'LineWidth',2); hold on;
    plot(tip.alpha_deg,  LD_tip,  'LineWidth',2);
    grid on;
    xlabel('\alpha (deg)');
    ylabel('L/D');
    title('Lift-to-Drag Ratio');
    legend(['Root: ' root.name], ['Tip: ' tip.name], 'Location','Best');

    %% -------- Drag Polar (CD vs CL) --------
    figure;
    plot(root.CD, root.CL, 'LineWidth',2); hold on;
    plot(tip.CD,  tip.CL,  'LineWidth',2);
    grid on;
    xlabel('C_D');
    ylabel('C_L');
    title('Drag Polar');
    legend(['Root: ' root.name], ['Tip: ' tip.name], 'Location','Best');

end