clc; clear; close all;

% figure export config
set(0, "DefaultFigureRenderer", "painters");

% load CSV data (first row is header)
data_sim = csvread("pd-test-gazebo-1.csv", 1, 0);
data_irl = csvread("pd-test-panda-1.csv",  1, 0);

% crop matrices (to fit length)
max_len = min(length(data_sim), length(data_irl));
data_sim(max_len + 1:end, :) = [];
data_irl(max_len + 1:end, :) = [];

% plot joint information for q5

% q5: position
figure()
hold on, box on
plot(data_sim(:, 1), data_sim(:, 7))
plot(data_irl(:, 1), data_irl(:, 7))
title("Joint 6 (q5): Position")
legend("Simulation", "Real")
xlabel("Time [s]")
ylabel("Position [rad]")

diff = abs(data_sim(:,7) - data_irl(:,7));
max_diff = max(diff)
t = data_sim(find( diff == max_diff ), 1)
e_ss = diff(end)

save_figure("img/sim-vs-real-q5-pos");

% q5: desired torque
figure()
hold on, box on
plot(data_sim(:, 1), data_sim(:, 21))
plot(data_irl(:, 1), data_irl(:, 21))
title("Joint 6 (q5): Desired Torque")
legend("Simulation", "Real")
xlabel("Time [s]")
ylabel("Torque [Nm]")

save_figure("img/sim-vs-real-q5-tau-des");

% q0-q6: all positions
figure('Renderer', 'painters', 'Position', [10 10 1000 800])
box on
tiled_plot = tiledlayout(4,2)
title(tiled_plot, "Joint Positions: Simulation vs. Real")
xlabel(tiled_plot,"Time [s]")
ylabel(tiled_plot,"Position [rad]")
tiled_plot.TileSpacing = "compact";
tiled_plot.Padding = "compact";

nexttile
title("Joint 1 (q0)")
hold on, box on
plot(data_sim(:, 1), data_sim(:, 2))
plot(data_irl(:, 1), data_irl(:, 2))

nexttile
title("Joint 2 (q1)")
hold on, box on
plot(data_sim(:, 1), data_sim(:, 3))
plot(data_irl(:, 1), data_irl(:, 3))
lgnd = legend("Simulation", "Real")

nexttile
title("Joint 3 (q2)")
hold on, box on
plot(data_sim(:, 1), data_sim(:, 4))
plot(data_irl(:, 1), data_irl(:, 4))

nexttile
title("Joint 4 (q3)")
hold on, box on
plot(data_sim(:, 1), data_sim(:, 5))
plot(data_irl(:, 1), data_irl(:, 5))

nexttile
title("Joint 5 (q4)")
hold on, box on
plot(data_sim(:, 1), data_sim(:, 6))
plot(data_irl(:, 1), data_irl(:, 6))

nexttile
title("Joint 6 (q5)")
hold on, box on
plot(data_sim(:, 1), data_sim(:, 7))
plot(data_irl(:, 1), data_irl(:, 7))

nexttile
title("Joint 7 (q6)")
hold on, box on
plot(data_sim(:, 1), data_sim(:, 8))
plot(data_irl(:, 1), data_irl(:, 8))

set(gcf, 'PaperPosition', [0 0 10 8]);
set(gcf, 'PaperSize', [10 8]);
saveas(gcf, "img/sim-vs-real-pos", "pdf");

function save_figure(filename)
    
    set(gcf, 'PaperPosition', [0 0 5 5]);
    set(gcf, 'PaperSize', [5 5]);
    saveas(gcf, filename, "pdf");

end