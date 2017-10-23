clear all;
clc;
close all;

% plot scheburn flexion - ACL,PCL
figure;
subplot(2,1,2);
Array=dlmread('scheburn_aACL_pACL.csv', '\t');
scheburn_ACL_x = Array(:, 1);
scheburn_ACL_y = Array(:, 2);

Array=dlmread('scheburn_aPCL.csv', '\t');
scheburn_aPCL_x = Array(:, 1);
scheburn_aPCL_y = Array(:, 2);

Array=dlmread('scheburn_pPCL.csv', '\t');
scheburn_pPCL_x = Array(:, 1);
scheburn_pPCL_y = Array(:, 2);

hold on;
h1 = plot(scheburn_ACL_x, scheburn_ACL_y, 'color', 'blue');
% hold;
h2 = plot(scheburn_aPCL_x, scheburn_aPCL_y, 'color', 'green')
% hold;
h3 = plot(scheburn_pPCL_x, scheburn_pPCL_y, 'color', 'cyan')
hold off;
title('Maximum isometric flexion [Scheburn et al]');
legend([h1, h2, h3], 'ACL', '(PCL) AL bundle', '(PCL) PM bundle', 'Location', 'NorthWest');
xlabel('knee angle (degrees)');
ylabel('force (N)');

% plot my results
subplot(2,1,1);
Array = dlmread('custom_reporter_flex_v6.mot', '\t');
my_flex_x = Array(:, 1);
my_aACL = Array(:, 2);
my_pACL = Array(:, 3);
my_aPCL = Array(:, 4);
my_pPCL = Array(:, 5);
my_flex_y = [my_aPCL, my_pPCL];
hold on;
h1 = plot(my_flex_x, my_aACL, 'color', 'b');
h2 = plot(my_flex_x, my_pACL, 'color', 'b');
h3 = plot(my_flex_x, my_aPCL, 'color', 'green');
h4 = plot(my_flex_x, my_pPCL, 'color', 'cyan');
plot(my_flex_x, my_flex_y );
hold off;

title('Active flexion [this thesis]');
legend([h1, h3, h4], 'aACL, pACL', '(PCL) AL bundle', '(PCL) PM bundle', 'Location', 'NorthWest');
xlabel('knee angle (degrees)');
ylabel('force (N)');
