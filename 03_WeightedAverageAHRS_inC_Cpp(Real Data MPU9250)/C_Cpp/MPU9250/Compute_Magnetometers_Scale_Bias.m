clc; close all; clear all; %#ok<*CLALL>

% testdata = load('imu_ascii.dat'); save testdata testdata;
load testdata;
%%

mgx_biase = (max(testdata(:,8)) + min(testdata(:,8))  )/2;
mgy_biase = (max(testdata(:,9)) + min(testdata(:,9))  )/2;
mgz_biase = (max(testdata(:,10)) + min(testdata(:,10))  )/2;

mgx_scale = (max(testdata(:,8)) - min(testdata(:,8))  )/2;
mgy_scale = (max(testdata(:,9)) - min(testdata(:,9))  )/2;
mgz_scale = (max(testdata(:,10)) - min(testdata(:,10))  )/2;

disp([mgx_biase, mgy_biase, mgz_biase]);
disp([mgx_scale, mgy_scale, mgz_scale]);

