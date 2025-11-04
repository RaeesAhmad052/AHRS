clc; close all; clear all; %#ok<*CLALL>
r2d = 180/pi;
testdata = load('imu_raw.dat'); save testdata testdata;
% load testdata1;testdata([2352, 46915],:) = [];

load testdata;

testdata(:,1) = testdata(:,1)*1000.0;

%% Time
h = figure(1); set(h,'name','Time');
ax(1)=subplot(211);plot(testdata(:,1)/1000.0,testdata(:,1)/1000,'r.-'); ylabel('Time');grid on;shg;
ax(2)=subplot(212);plot(testdata(2:end,1)/1000.0,diff(testdata(:,1)),'r.-'); ylabel('Timediff [ms]');
xlabel('Time [sec]');linkaxes(ax,'x');shg;grid on;shg;
%%
h = figure(2); set(h,'name','Acceleration');
ax(1)=subplot(311);plot(testdata(:,1)/1000.0,testdata(:,2),'r.-'); ylabel('abx [m/s/s]');grid on;shg;
ax(2)=subplot(312);plot(testdata(:,1)/1000.0,testdata(:,3),'r.-'); ylabel('aby [m/s/s]');grid on;shg;
ax(3)=subplot(313);plot(testdata(:,1)/1000.0,testdata(:,4),'r.-'); ylabel('abz [m/s/s]');grid on;shg;
xlabel('Time [sec]');linkaxes(ax,'x');shg;grid on;shg;

h = figure(3); set(h,'name','Total Acceleration');
plot(testdata(:,1)/1000.0,sqrt(testdata(:,2).^2 + testdata(:,3).^2 + testdata(:,4).^2),'r.-'); ylabel('abxyz [m/s/s]');grid on;shg;
xlabel('Time [sec]');linkaxes(ax,'x');shg;grid on;shg;
return;
%%
% testdata = testdata(200*200:450*200,:);

h = figure(4); set(h,'name','Angular rates');
ax(1)=subplot(311);plot(testdata(:,1)/1000.0,testdata(:,5),'r.-'); ylabel('wbx [deg/s]');grid on;shg;
ax(2)=subplot(312);plot(testdata(:,1)/1000.0,testdata(:,6),'r.-'); ylabel('wby [deg/s]');grid on;shg;
ax(3)=subplot(313);plot(testdata(:,1)/1000.0,testdata(:,7),'r.-'); ylabel('wbz [deg/s]');grid on;shg;
xlabel('Time [sec]');linkaxes(ax,'x');shg;grid on;shg;
%%
h = figure(5); set(h,'name','Magnetic Fields');
ax(1)=subplot(311);plot(testdata(:,1)/1000.0,testdata(:,8),'r.-');  ylabel('bx [mT]');grid on;shg;
ax(2)=subplot(312);plot(testdata(:,1)/1000.0,testdata(:,9),'r.-');  ylabel('by [mT]');grid on;shg;
ax(3)=subplot(313);plot(testdata(:,1)/1000.0,testdata(:,10),'r.-'); ylabel('bz [mT]');grid on;shg;
xlabel('Time [sec]');linkaxes(ax,'x');shg;grid on;shg;
vv
%%

mgx_biase = (max(testdata(:,8)) + min(testdata(:,8))  )/2;
mgy_biase = (max(testdata(:,9)) + min(testdata(:,9))  )/2;
mgz_biase = (max(testdata(:,10)) + min(testdata(:,10))  )/2;

mgx_scale = (max(testdata(:,8)) - min(testdata(:,8))  )/2;
mgy_scale = (max(testdata(:,9)) - min(testdata(:,9))  )/2;
mgz_scale = (max(testdata(:,10)) - min(testdata(:,10))  )/2;


avg = (mgx_scale + mgy_scale)/2.0;
% avg = (mgx_scale + mgy_scale + mgz_scale)/3.0;

mgx_scale_n = avg/mgx_scale;
mgy_scale_n = avg/mgy_scale;
mgz_scale_n = avg/mgz_scale;
% disp([mgx_scale_n, mgy_scale_n, mgz_scale_n;  mgx_biase, mgy_biase, mgz_biase ]);

disp([mgx_scale_n, mgy_scale_n;  mgx_biase, mgy_biase ]);
bb
%%
h = figure(6); set(h,'name','Magnetic Fields');
plot(testdata(:,9),testdata(:,8),'b.-',testdata(:,9)*mgy_scale_n-mgy_biase,testdata(:,8)*mgx_scale_n-mgx_biase,'r.-');  
ylabel('bx [mT]');grid on;shg;xlabel('by [mT]');grid on;axis equal;shg;
%
% h = figure(601); set(h,'name','Magnetic Fields');
% plot3(testdata(:,8),testdata(:,9),testdata(:,10),'b.-'     ,testdata(:,8)*mgx_scale_n-mgx_biase, testdata(:,9)*mgy_scale_n-mgy_biase, testdata(:,10)*mgz_scale_n-mgz_biase,'r.-');  
% ylabel('bx [mT]');grid on;shg;xlabel('by [mT]');grid on;axis equal;shg;

%%

h = figure(); set(h,'name',' Yaw Angle');
plot(testdata(:,1), testdata(:,13)*r2d, 'b.-' ); shg;grid on;
xlabel('Time '); ylabel('Yaw [deg]'); legend('Measurement','Filter Aided Output');

%%
h = figure(4); set(h,'name','Angular rates');
plot(testdata(:,1)/1000.0,testdata(:,5)*r2d,'r.-',testdata(:,1)/1000.0, testdata(:,12)*r2d, 'b.-'); ylabel('wbx [deg/s]');grid on;shg;
h = figure(401); set(h,'name','Angular rates');
plot(testdata(:,1)/1000.0,testdata(:,6)*r2d,'r.-',testdata(:,1)/1000.0, testdata(:,11)*r2d, 'b.-'); ylabel('wby [deg/s]');grid on;shg;
h = figure(402); set(h,'name','Angular rates');
plot(testdata(:,1)/1000.0,testdata(:,7)*r2d,'r.-',testdata(:,1)/1000.0, testdata(:,13)*r2d, 'b.-'); ylabel('wbz [deg/s]');grid on;shg;
xlabel('Time [sec]');shg;grid on;shg;