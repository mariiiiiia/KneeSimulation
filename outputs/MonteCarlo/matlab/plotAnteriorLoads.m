clear all;
clc;
close all;

knee_angle = [0,20,40,60,80,90];
att = [1.281,3.007,4.9811,6.417,5.178,5.778];

figure;
plot(knee_angle, att);
hold;

aACL_forces = [28,51,55,82,80,47];
pACL_forces = [55,37,14,20,29,20];

ACL_forces = [];
ACL_forces = cat(3, aACL_forces, pACL_forces);

figure;
plot(knee_angle, aACL_forces);
hold;
plot(knee_angle, pACL_forces);
hold;