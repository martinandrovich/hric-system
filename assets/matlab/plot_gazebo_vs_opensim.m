clc; clear; close all;

% figure export config
set(0, "DefaultFigureRenderer", "painters");

% load CSV data (first row is header)
data_gazebo  = csvread("pd-test-gazebo-nolim-nokdl.csv", 1, 0);
data_opensim = csvread("pd-test-opensim.csv", 1, 0);

% time example
figure()
plot(data_opensim(:, 1));
xlim([3800 4000])
title("OpenSim time steps")
xlabel("Time step")
ylabel("Time [s]")
save_figure("img/opensim-timestep");

% fix OpenSim data (time nonlinearity)
data_opensim = fix_data(data_opensim(12:13000, :));

% limit all data to 3 seconds
data_gazebo = data_gazebo(1:find(data_gazebo(:, 1) > 3, 1, "first") - 1, :);
data_opensim = data_opensim(1:find(data_opensim(:, 1) > 3, 1, "first") - 1, :);

% figure()
% hold on
% plot(data_opensim(:, 1), data_opensim(:, 3))
% plot(data_gazebo(:, 1), data_gazebo(:, 3))

%% Plot
close all;

% q0-q6: all positions
figure('Renderer', 'painters', 'Position', [10 10 1000 800])
box on
tiled_plot = tiledlayout(4,2)
title(tiled_plot, "Joint Positions: Gazebo vs. OpenSim")
xlabel(tiled_plot,"Time [s]")
ylabel(tiled_plot,"Position [rad]")
tiled_plot.TileSpacing = "compact";
tiled_plot.Padding = "compact";

nexttile
title("Joint 1 (q0)")
hold on, box on
plot(data_gazebo(:, 1), data_gazebo(:, 2))
plot(data_opensim(:, 1), data_opensim(:, 2))

nexttile
title("Joint 2 (q1)")
hold on, box on
plot(data_gazebo(:, 1), data_gazebo(:, 3))
plot(data_opensim(:, 1), data_opensim(:, 3))
lgnd = legend("Gazebo", "OpenSim");

nexttile
title("Joint 3 (q2)")
hold on, box on
plot(data_gazebo(:, 1), data_gazebo(:, 4))
plot(data_opensim(:, 1), data_opensim(:, 4))

nexttile
title("Joint 4 (q3)")
hold on, box on
plot(data_gazebo(:, 1), data_gazebo(:, 5))
plot(data_opensim(:, 1), data_opensim(:, 5))

nexttile
title("Joint 5 (q4)")
hold on, box on
plot(data_gazebo(:, 1), data_gazebo(:, 6))
plot(data_opensim(:, 1), data_opensim(:, 6))

nexttile
title("Joint 6 (q5)")
hold on, box on
plot(data_gazebo(:, 1), data_gazebo(:, 7))
plot(data_opensim(:, 1), data_opensim(:, 7))

nexttile
title("Joint 7 (q6)")
hold on, box on
plot(data_gazebo(:, 1), data_gazebo(:, 8))
plot(data_opensim(:, 1), data_opensim(:, 8))

set(gcf, 'PaperPosition', [0 0 10 8]);
set(gcf, 'PaperSize', [10 8]);
saveas(gcf, "img/gazebo-vs-opensim-pos", "pdf");

%%

function data = fix_data(data)

%     data = data(10:end, :);

    prev = 0;
    i = 1;
    while (i < length(data))
        
        if (prev > data(i, 1))
            
            % backtrack
            j = i - 1;
            s = j;
            while (data(i, 1) <= data(j, 1))
                j = j - 1;
            end
            
            data(j + 1: s, :) = [];
            i = j;
%             plot(data);
            
        end
        
        prev = data(i, 1);
        i = i + 1;

    end

end


function save_figure(filename)
    
    set(gcf, 'PaperPosition', [0 0 5 5]);
    set(gcf, 'PaperSize', [5 5]);
    saveas(gcf, filename, "pdf");

end
