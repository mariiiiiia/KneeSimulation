%%
% This script analyzes the simulation results of
% the Monte Carlo analysis. It assumes that there
% exists a set of *custom_reporter_flexion.sto files 
% somewhere in the root directory.

clear all;
clc;
close all;

numRaws = 2;
index1 = 22;
index2 = 24;

%% find all files
cwd = pwd;
cd(strcat(cwd, '/../'));
[status, out] = dos(['dir /s/b *custom_reporter_flex.mot']);
files = strread(out, '%s', 'delimiter', sprintf('\n'));
cd(cwd);

%% construct fan char
labels = [];
totalCollect = [];

id = readMotionFile(files{1});
data = id.data;
labels = id.labels;
    
kneeAnglePlot = [0, 30, 60, 90];  

collect = [];
for j=index1:index2
    collect = [collect, data(:, j)];
end
    
kneeAngleSim = [];
kneeAngleSim = data(:, 4); 
dataPlot = [];
dataPlot = [dataPlot, collect(1,:)];
for nt=2:length(kneeAnglePlot)
    for t=2:length(kneeAngleSim)
        if (kneeAngleSim(t) > kneeAnglePlot(nt)) 
            dataPlot = [dataPlot; (collect(t-1,:)+collect(t,:))/2];
            break;
        end
    end
end
    
totalCollect = cat(3, totalCollect, dataPlot);

%% plot
figure;
bar(kneeAnglePlot, dataPlot);
xlabel('knee angle (degrees)');
ylabel('force (N)');
