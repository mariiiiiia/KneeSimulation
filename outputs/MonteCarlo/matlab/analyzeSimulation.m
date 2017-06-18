%%
% This script analyzes the simulation results of
% the Monte Carlo analysis. It assumes that there
% exists a set of *custom_reporter_flexion.sto files 
% somewhere in the root directory.

clear all;
clc;
close all;

numRaws = 2;
index1 = 95;
index2 = 98;

%% find all files
cwd = pwd;
cd(strcat(cwd, '/../'));
[status, out] = dos(['dir /s/b *custom_reporter_flexion.mot']);
files = strread(out, '%s', 'delimiter', sprintf('\n'));
cd(cwd);

%% construct fan char
labels = [];
totalCollect = [];
for i=1:75
    id = readMotionFile(files{i});
    data = id.data;
    labels = id.labels;
    
    timePlot = [];
    for mt=1:100
        numTime = 0.25/100*mt;
        timePlot = [timePlot; numTime];  
    end
    
    collect = [];
    for j=index1:index2
        collect = [collect, data(:, j)];
    end
    
    timeSim = [];
    timeSim = data(:, 1); 
    dataPlot = [];
    dataPlot = [dataPlot, collect(1,:)];
    for nt=1:length(timePlot)
        for t=1:length(timeSim)
            if (timeSim(t) > timePlot(nt)) 
                dataPlot = [dataPlot; (collect(t-1,:)+collect(t,:))/2];
                break;
            end
        end
    end
    
%     plot(timePlot, dataPlot(:,1));
%     hold;
%     plot(timePlot, dataPlot(:,2));
%     hold;
    totalCollect = cat(3, totalCollect, dataPlot);
end

%% plot
figure;
[t, n, f] = size(totalCollect);
c = ceil(n / numRaws);
for i = 1:n
    subplot(numRaws, c, i);
    %size(totalCollect(:, i, :))
    fanChart(timePlot, squeeze(totalCollect(:, i, :)), ...
        'mean', 5:5:95, 'alpha', .3, 'colormap', ...
        {'shadesOfColor', [0 0 .9]});
    title(labels{i-1+index1}, 'Interpreter', 'none');
    xlabel('time (s)');
    ylabel('force (N)');
end