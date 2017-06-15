%%
% This script analyzes the simulation results of
% the Monte Carlo analysis. It assumes that there
% exists a set of *inverse_dynamics.sto files 
% somewhere in the root directory.

clear all;
clc;
close all;

numRaws = 2;
index1 = 5;
index2 = 8;

%% find all files
cwd = pwd;
cd(strcat(cwd, '/../'));
[status, out] = dos(['dir /s/b *custom_reporter_flexion.mot']);
files = strread(out, '%s', 'delimiter', sprintf('\n'));
cd(cwd);

%% construct fan char
time = [];
labels = [];
totalCollect = [];
for i=6:length(files)
    
    id = readMotionFile(files{i});
    data = id.data;
    if i == 6
        labels = id.labels;
        time = data(:, 1);
%         time = linspace(0, 0.2, 10000);
    end

    collect = [];
    for j=index1:index2
        collect = [collect, data(:, j)];
    end
    
%     plot(time, collect);
%     hold;
    totalCollect = cat(3, totalCollect, collect);
end

%% plot
figure;
[t, n, f] = size(totalCollect);
c = ceil(n / numRaws);
for i = 1:n
    subplot(numRaws, c, i);
    %size(totalCollect(:, i, :))
    fanChart(time, squeeze(totalCollect(:, i, :)), ...
        'mean', 5:5:95, 'alpha', .3, 'colormap', ...
        {'shadesOfColor', [0 0 .9]});
    title(labels{i+1+index1}, 'Interpreter', 'none');
    xlabel('time (s)');
    ylabel('moment (Nm)');
end