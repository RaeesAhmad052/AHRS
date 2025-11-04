clc; close all; clear all; %#ok<*CLALL>

cd 'X up';    
xup = load('imu_raw.dat'); 
xup([1:40*200, 120*200:end ] ,:) = [];
cd ..;

cd 'X down';      
xdown = load('imu_raw.dat'); 
xdown(91*200:end,:) = [];
cd ..;

cd 'Y up';    
yup = load('imu_raw.dat'); 
yup(91*200:end,:) = [];
cd ..;

cd 'Y down';   
ydown = load('imu_raw.dat'); save ydown ydown;
ydown([1:85*200, 120*200:end],:) = [];
cd ..;

cd 'Z up';    
zup = load('imu_raw.dat'); 
zup( 1:40*200,:) = [];
cd ..;

cd 'Z down'; 
zdown = load('imu_raw.dat'); 
cd ..;
%% Time

offset_xup   = mean(sqrt(xup(:,2).^2 + xup(:,3).^2 + xup(:,4).^2)) - 9.8; 
offset_xdown = mean(sqrt(xdown(:,2).^2 + xdown(:,3).^2 + xdown(:,4).^2)) - 9.8; 
offset_yup   = mean(sqrt(yup(:,2).^2 + yup(:,3).^2 + yup(:,4).^2)) - 9.8; 
offset_ydown = mean(sqrt(ydown(:,2).^2 + ydown(:,3).^2 + ydown(:,4).^2)) - 9.8; 
offset_zup   = mean(sqrt(zup(:,2).^2 + zup(:,3).^2 + zup(:,4).^2)) - 9.8; 
offset_zdown = mean(sqrt(zdown(:,2).^2 + zdown(:,3).^2 + zdown(:,4).^2)) - 9.8; 

disp([offset_xup, offset_xdown, offset_yup, offset_ydown, offset_zup, offset_zdown])