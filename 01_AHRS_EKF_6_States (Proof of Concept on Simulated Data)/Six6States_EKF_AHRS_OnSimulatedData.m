clc; 
clearvars;
close all;
warning off;
format long g;
%% File Opening for Output

fid = fopen('output1.dat','w');

%% Initialization
SimTime = 300; % simulation time in seconds
lenth = round(300/0.020); 
Mb = zeros(3,1);
C_b2h = zeros(3,3);
ac_ave = [0;0;0];
mg_ave = [0;0;0];
psi_c = 0;
theta_c = 0;
phi_c = 0;
psi_m = 0;
theta_m = 0;
phi_m = 0;

%% Constants
g = 9.8;
tau = 360;     %   Correlation Time of gyros depends upon gyros specification
dt = 0.02;
d2r = pi/180;
dph2rps = pi/180/3600;
millig = 9.8e-3;
W = 0.00007292115; %earth rate 15deg/h= 0.00007292115rad/s
time = 0.0;


%% Initialization of kalman paramaters ........
I = eye(6);
bias = [0;0;0]*dph2rps;

xk_up      = zeros(6,1);  % State Vector
xk_m      = zeros(6,1);  % State Vector previous
trans_mat = zeros(6,6);


P_up = [  (5*d2r)^2    0           0           0                       0               0
            0           (1*d2r)^2   0           0                       0               0
            0           0           (1*d2r)^2   0                       0               0
            0           0           0           (100*dph2rps)^2         0               0
            0           0           0           0                      (100*dph2rps)^2  0
            0           0           0           0                       0               (100*dph2rps)^2];

%% Guass-Markov Modeling 
var_gyro = (100.0*dph2rps)^2;
beta = (1/tau);
dtkf = 0.2;
twobdt   = 2.0*beta*dtkf;
xponent2 = exp(-twobdt);
qdt_gyro = var_gyro *(1.0-xponent2);

%%% Process Noise Matrix
Q = [   0 0 0 0                 0                   0
        0 0 0 0                 0                   0
        0 0 0 0                 0                   0
        0 0 0 qdt_gyro          0                   0
        0 0 0 0                 qdt_gyro            0
        0 0 0 0                 0                   qdt_gyro];
%%% H Matrix
H = [ 1 0 0 0 0 0
      0 1 0 0 0 0
      0 0 1 0 0 0];
%%% Measurement Matrix
R = [(1*d2r)^2   0           0
      0           (0.5*d2r)^2   0                
      0           0           (0.5*d2r)^2];
%% Input Reference Attitude Angles
thead = 15.0;
pitch = 10.0;
roll  =  5.0;
%% Quaternions initialization ....................
q_g2b= angle2quat(0.0,0.0,0.0, 'ZYX');
q_p = q_g2b;
% C_g2b = quat2dcm(q_p);
Lat_i = 33.0;
%% Computational Loop

for i = 1:lenth
    time = time + 0.02;
    
    %   Sensor Data Simulation
    we_g = [W*cos(Lat_i); 0; -W*sin(Lat_i)];
    cg2b_true = angle2dcm(thead*d2r,pitch*d2r,roll*d2r,'ZYX');
       
    g_n = [0; 0; -9.8] ;
    
    ww  = cg2b_true*we_g + [100; 200; 300]*dph2rps - bias  + [10*randn; 10*randn; 10*randn]*dph2rps; 
    f_b = cg2b_true*g_n + [1; 1; 1]*millig + [5*randn; 5*randn; 5*randn]*millig; % noise 5mg 1mg = 1e-3 *9.8 m/s/s
   
      %% Simulate Magnetic field
    % Example: Magnetic field at Islamabad (33.0°N, 73.0°E) at sea level
    lat = 33.0;     % degrees north
    lon = 73.0;     % degrees east
    alt = 100;        % altitude in meters
    decimalYear = decyear(2020,7,4); % decimal year
    model = 2020;       % model wmm

    % Compute magnetic field using WMM
    [BXYZ, Btotal, decl, incl] = wrldmagm(alt, lat, lon, model);
    cmg2b_true = angle2dcm((thead+decl)*d2r,pitch*d2r,roll*d2r,'ZYX');
    Mag_B_Ref = cmg2b_true*[BXYZ(1), BXYZ(2), BXYZ(3)]' + [500*randn; 500*randn; 500*randn]; % nanoTesla Noise
  
    %   End Sensor Data Simulation
    
    %% Quaternion updating
    q(1) = q_p(1) - 0.5*(q_p(2)*ww(1) + q_p(3)*ww(2) + q_p(4)*ww(3))*dt;
    q(2) = q_p(2) + 0.5*(q_p(1)*ww(1) + q_p(3)*ww(3) - q_p(4)*ww(2))*dt;
    q(3) = q_p(3) + 0.5*(q_p(1)*ww(2) - q_p(2)*ww(3) + q_p(4)*ww(1))*dt;
    q(4) = q_p(4) + 0.5*(q_p(1)*ww(3) + q_p(2)*ww(2) - q_p(3)*ww(1))*dt;
    
    [psi,theta,phi] = quat2angle(q,'ZYX');
     q_p = q;
  
     ac_ave = ac_ave + f_b;
     mg_ave = mg_ave + [Mag_B_Ref(1), Mag_B_Ref(2),   Mag_B_Ref(3)]'; %RefData(1,17:19)';
   
    %% Kalman Filter Loop
    if (rem(i,10)==0)  
        %Averaging Accelerometer and Magnetometer outputs upto 10 samples
        
        ac_ave = -ac_ave/10;
        mg_ave = mg_ave/10;
        
        %%% Calculating the pitch and roll from the Accelerometer accelerations
        phi_m = atan2(ac_ave(2), ac_ave(3));
        theta_m = atan2(-ac_ave(1), sqrt(ac_ave(2)^2 +  ac_ave(3)^2));
        
        %%% Magnetic heading computation as psi_m
        C_b2h = angle2dcm(-phi,-theta,0,'XYZ');
        Mw = C_b2h*mg_ave;
        psi_m = atan2 (-Mw(2), Mw(1));
        
        %%% Measurements
        psi_del    = psi - psi_m ;
        if(psi_del > pi)
            psi_del = psi_del - 2*pi;
        elseif(psi_del <= -pi)
            psi_del = psi_del + 2*pi;
        end
        th_del = theta - theta_m ;
        phi_del = phi  - phi_m ;

        ac_ave = [0;0;0];
        mg_ave = [0;0;0];
        
        %% KALMAN FILTERING...............
        sp = sin(theta);
        cp = cos(theta);
        sr = sin(phi);
        cr = cos(phi);
        
        z = [psi_del; th_del; phi_del];
        
        trans_mat = eye(6)+ [0   sp*(ww(2)*sr+ww(3)*cr)/cp^2      (ww(2)*cr-ww(3)*sr)/cp    0       sr/cp       cr/cp
                             0   0                                (-ww(2)*sr-ww(3)*cr)      0       cr          -sr
                             0   (ww(2)*sr+ww(3)*cr)/cp^2         (ww(2)*cr-ww(3)*sr)       1       sr*sp/cp    cr*sp/cp
                             0   0                                 0                       -1/tau    0          0
                             0   0                                 0                        0       -1/tau      0
                             0   0                                 0                        0        0         -1/tau   ]*dtkf;        
       
                           
        %%% State & Error Covariance Prediction Ahead
        xk_m = trans_mat*xk_m;
        P_minus = trans_mat*P_up*transpose(trans_mat) + Q*dtkf;      % phi*P-*phi' + Q*dt
        
        %%% Computing Kalman Gain
        K  = P_minus*transpose(H)/(H*P_minus*transpose(H) + R);
        
        %%% State & Error Covariance Update
        xk_up = xk_m + K*z;
        P_up = (I-K*H)*P_minus;

        %%% Applying Correction .........
        psi    = psi    - xk_up(1);
        theta  = theta  - xk_up(2);
        phi    = phi    - xk_up(3);
        bias   = bias   + xk_up(4:6);
        
        
        %% reinitializing quaternions ...........................
        q_p = angle2quat(psi, theta, phi, 'ZYX');

        
    end
    %%              1      2     3     4      5       6       7     8       9     10    11      12      13    14      15    16     %% 1    2       3          4       567     8~13          14         15         16                
    fprintf(fid,'%15.3f %15.6f %15.6f %15.6f %15.6f %15.6f %15.6f %15.6f %15.6f %15.6f %15.6f %15.6f %15.6f %15.6f %15.6f %15.6f \n',time ,psi/d2r,theta/d2r, phi/d2r,bias,sqrt(diag(P_up)),psi_m/d2r,theta_m/d2r,phi_m/d2r);
        
end
fclose(fid);


%% Plotting algorithm output
disp('Plotting EKF algorithm output');
load output1.dat;
%%
h=figure(1);set(h,'name','heading');
plot([output1(1,1),output1(end,1)],[thead,thead],'ko-',output1(:,1),output1(:,14),'b.-',output1(:,1),output1(:,2),'r.-');
legend('True Value','Magnetometers Measurements','EKF Estimation','location','best');
xlabel('time(s)'); ylabel('Heading Angle [deg]');
title('Heading Angle');grid on;shg;
saveas(h,'Heading','fig');
saveas(h,'Heading','png');
%% Pitch Angle
h=figure(3);set(h,'name','Pitch');
plot([output1(1,1),output1(end,1)],[pitch,pitch],'ko-',output1(:,1),output1(:,15),'b.-',output1(:,1),output1(:,3),'r.-');
legend('True Value','Accelerometers Measurements','EKF Estimated','location','best');
xlabel('time(s)'); ylabel('Pitch angle [deg]');
title('Pitch Angle');grid on;shg;
saveas(h,'Pitch_Angle','fig');
saveas(h,'Pitch_Angle','png');

%% Roll Angle
h=figure(2);set(h,'name','Roll');
hhl=plot([output1(1,1),output1(end,1)],[roll,roll],'ko-',output1(:,1),output1(:,16),'b.-',output1(:,1),output1(:,4),'r.-');
legend('True Value','Accelerometers Measurements','EKF Estimated','location','best');
xlabel('time(s)'); ylabel('Roll angle [deg]');grid on;
title('Roll Angle');grid on;shg;
saveas(h,'Roll_Angle','fig');
saveas(h,'Roll_Angle','png');

%% Gyro Biases
h=figure(4);  set(h,'name','baises');
hL=plot(output1(:,1),output1(:,5)/dph2rps,'k.-',output1(:,1),(output1(:,5)-output1(:,11))/dph2rps,'k--',output1(:,1),(output1(:,5)+output1(:,11))/dph2rps,'k--',...
        output1(:,1),output1(:,6)/dph2rps,'g.-',output1(:,1),(output1(:,6)-output1(:,12))/dph2rps,'g--',output1(:,1),(output1(:,6)+output1(:,12))/dph2rps,'g--',...
        output1(:,1),output1(:,7)/dph2rps,'r.-',output1(:,1),(output1(:,7)-output1(:,13))/dph2rps,'r--',output1(:,1),(output1(:,7)+output1(:,13))/dph2rps,'r--');
legend(hL([1,4,7]),'\delta wx_p','\delta wy_q','\delta wz_r');
xlabel('time(s)'); ylabel('degree/hour');
title('Gyro biases estimation');grid on;shg;
saveas(h,'Gyro_Biases','fig');
saveas(h,'Gyro_Biases','png');
%% Covariances
h = figure(5); set(h,'name','Attitude Angles Errors State Covariance');
plot(output1(:,1),output1(:,8:10)*180/pi,'.-'); 
legend('\Delta \psi','\Delta \theta','\Delta \phi');
xlabel('time(s)'); ylabel('degrees');
title('Attitude Angles Errors State Covaiance Analysis');grid on;shg;

h = figure(6); set(h,'name','Gyro Biases Covariance');
plot(output1(:,1),output1(:,11:13)/dph2rps,'.-'); 
legend('\delta wx_p','\delta wy_q','\delta wz_r');
xlabel('time(s)'); ylabel('degree/hour');
title('Gyro biases Covaiance Analysis');grid on;shg;
