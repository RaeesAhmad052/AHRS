clc; close all; clear all; %#ok<*CLALL>

testdata = load('imu_data9dof.dat'); 

%%
h = figure(5); set(h,'name','Magnetic Fields');
ax(1)=subplot(311);plot(testdata(:,1)/1000.0,testdata(:,8),'r.-');  ylabel('bx [mT]');grid on;shg;
ax(2)=subplot(312);plot(testdata(:,1)/1000.0,testdata(:,9),'r.-');  ylabel('by [mT]');grid on;shg;
ax(3)=subplot(313);plot(testdata(:,1)/1000.0,testdata(:,10),'r.-'); ylabel('bz [mT]');grid on;shg;
xlabel('Time [sec]');linkaxes(ax,'x');shg;grid on;shg;

%%

mgx_biase = (max(testdata(:,8)) + min(testdata(:,8))  )/2;
mgy_biase = (max(testdata(:,9)) + min(testdata(:,9))  )/2;
mgz_biase = (max(testdata(:,10)) + min(testdata(:,10))  )/2;

mgx_scale = (max(testdata(:,8)) - min(testdata(:,8))  )/2;
mgy_scale = (max(testdata(:,9)) - min(testdata(:,9))  )/2;
mgz_scale = (max(testdata(:,10)) - min(testdata(:,10))  )/2;


avg = (mgx_scale + mgy_scale)/2.0;
% avg = (mgx_scale + mgy_scale + mgz_scale)/3.0;
%% normalize scale factors
mgx_scale_n = avg/mgx_scale;
mgy_scale_n = avg/mgy_scale;
mgz_scale_n = avg/mgz_scale;
% disp([mgx_scale_n, mgy_scale_n, mgz_scale_n;  mgx_biase, mgy_biase, mgz_biase ]);


%%
h = figure(6); set(h,'name','Magnetic Fields');
plot(testdata(:,9),testdata(:,8),'b.-',...
     testdata(:,9)*mgy_scale_n-mgy_biase,testdata(:,8)*mgx_scale_n-mgx_biase,'r.-',...
     testdata(:,9)*mgy_scale_n-mgy_biase,testdata(:,8)*mgx_scale_n-(mgx_biase-5),'k.-');  
ylabel('bx [mT]');grid on;shg;xlabel('by [mT]');grid on;axis equal;shg;
legend('uncalibrated','Calibrated','Mannually Ajdusted Calibration');


disp([mgx_scale_n, mgy_scale_n;  mgx_biase-5, mgy_biase ]);
