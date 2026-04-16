function plotTwistFunction(twistOut)
% plotTwistFunction
%
% Plots spanwise geometric twist distribution.

    figure;
    plot(twistOut.eta, twistOut.twist_deg, 'LineWidth', 2);
    grid on;
    xlabel('\eta = y / (b/2)');
    ylabel('Twist (deg)');
    title('Spanwise Twist Distribution');
end