clear all;
clc;
close all;

%% find all files
cwd = pwd;
cd(strcat(cwd, '/../'));
[status, out] = dos(['dir /s/b custom_reporter_ant_load*.mot']);
files = strread(out, '%s', 'delimiter', sprintf('\n'));
cd(cwd);

%% construct fan char
labels = [];
APT = [];
kneeAngle = [];
aACL = [];
pACL = [];
ACL = [];
aACL_strain = [];
pACL_strain = [];
bonyContact = [];
MCL = [];
menisc = [];
for i=1:5
    id = readMotionFile(files{i});
    data = id.data;
    labels = id.labels;
    
    raw = size(data, 1);
    APT = [APT; data( raw, 2)];   
    kneeAngle = [kneeAngle, data( 1, 4)];
    aACL = [aACL, data( raw, 5)];
    pACL = [pACL, data( raw, 6)];
%     aACL_strain = [aACL_strain, data( raw, 33)];
%     pACL_strain = [pACL_strain, data( raw, 34)];
%     MCL = [MCL, data(raw, 9)+data(raw, 10)+data(raw, 11)];
%     bonyContact = [bonyContact, data(raw, 53) + data(raw, 51)];
%     menisc = [menisc, data(raw, 49) + data(raw, 47)];
end
    
for j=1:length(kneeAngle)
    ACL = [ACL, (aACL(1,j)+pACL(1,j))];
end;

totalACL = [aACL; pACL; ACL];

% plot att
figure;
aptPaper = [0.003; 0.0045; 0.006; 0.0075; 0.0065];
APT = cat(2, APT, aptPaper);
bar(kneeAngle, APT);
title('Anterior Tibial Loads (110N)');
legend('this thesis','M. Sakane Et Al', 'Location', 'NorthWest');
xlabel('knee angle (degrees)');
ylabel('anterior tibial translation (m)');
hold;

% plot my ACL forces
figure;
%subplot(1,2,1);
plot(kneeAngle, totalACL, '- o');
%title('Anterior Tibial Loads (110N) [My Thesis]');
legend('AM bundle', 'PM bundle', 'ACL');
xlabel('knee angle (degrees)');
ylabel('force (N)');
ylim([0 110]);
hold;

% plot paper acl forces
aACL_paper = [30, 35, 40, 45, 44];
pACL_paper = [62, 75, 60, 35, 25];
ACL_paper = [];
for i=1:5
    ACL_paper = [ACL_paper, aACL_paper(i) + pACL_paper(i)];
end
total_ACL_paper = [aACL_paper; pACL_paper; ACL_paper];
% figure;
%subplot(1,2,2);
h = plot(kneeAngle, total_ACL_paper, '--.');
title('Anterior Tibial Loads (110N)');
legend('aACL (this Thesis)', 'pACL (this Thesis)', 'ACL (this Thesis)', 'aACL (Sakane et al)', 'pACL (Sakane et al)', 'ACL (Sakane et al)');
xlabel('knee angle (degrees)');
ylabel('force (N)');
ylim([0 110]);
hold;
