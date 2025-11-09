%% Import and plot sensor data
clc;clear all;close all;
d2r = pi/180;
load TestData;%load TestData.dat; save TestData TestData;
%%
Q = zeros(4,1); 
Mb = zeros(3,1); 
g=9.8;   
A = [0;0;g]; 
TMM = zeros(3,3); 

TM = zeros(3,3);
tau = 360;     %   Correlation Time of gyros depends upon gyros specification
dph2rps = pi/180/3600;
millig = 9.8e-3;
W = 0.00007292115; %earth rate 15deg/h= 0.00007292115rad/s
%% Averaging Accelerometer and Magnetometer outputs --------------------
gb_ave1=0; gb_ave2=0; gb_ave3=0; mb_ave1=0; mb_ave2=0; mb_ave3=0;
for j = 1: 10
    gb_ave1 = gb_ave1 + TestData(j,5);    mb_ave1 = mb_ave1 + TestData(j,8);
    gb_ave2 = gb_ave2 + TestData(j,6);    mb_ave2 = mb_ave2 + TestData(j,9);
    gb_ave3 = gb_ave3 + TestData(j,7);    mb_ave3 = mb_ave3 + TestData(j,10);
end
gb_1 = -gb_ave1/10;      mb_1 = mb_ave1/10;
gb_2 = -gb_ave2/10;      mb_2 = mb_ave2/10;
gb_3 = -gb_ave3/10;      mb_3 = mb_ave3/10;
%%%% Calculating the pitch and roll from the Accelerometer accelerations
phi_g1 = atan2(gb_2, gb_3);
theta_g1 = atan2(-gb_1, sqrt(gb_2^2 +  gb_3^2));
%% Magnetometer output average vector ----------------------------
Mb1 = [mb_1;mb_2;mb_3];
%%% Decomposing the b-fram to n-frame by teh reverse rotation of the pitch
%%% and roll to compute the heading (yaw) following is the decomposition matrix
TMM(1,1)= cos(theta_g1);     TMM(1,2)= sin(phi_g1)*sin(theta_g1);    TMM(1,3)= sin(theta_g1)*cos(phi_g1);
TMM(2,1)= 0;                 TMM(2,2)= cos(phi_g1);                  TMM(2,3)= -sin(phi_g1);
TMM(3,1)= -sin(theta_g1);    TMM(3,2)= cos(theta_g1)*sin(phi_g1);    TMM(3,3)= cos(phi_g1)*cos(theta_g1);
% Magnetic heading computation as xi_g
Mww = TMM*Mb1;     xi_g1 = atan2 (-Mww(2), Mww(1));
%% Quaternion initialization from accelerometer and magnetometer
Qt= angle2quat(xi_g1, theta_g1, phi_g1, 'ZYX');
q_p=Qt;
time = zeros(length(TestData(:,1)),1);    time(:,1)= TestData(:,1);
kprev = 1;  
knxt = 10;
xi_c = 0; 
theta_c =0; 
phi_c=0;

lenth = length(time);
wx=zeros(lenth,1); wx=TestData(:,2);
wy=zeros(lenth,1); wy=TestData(:,3);
wz=zeros(lenth,1); wz=TestData(:,4);
%% kalman initialization paramaters ........
xi=zeros(6,1); 
xp=zeros(6,1); 
xc=zeros(6,1);
Fi=zeros(6,6); 
ddt=0.1; 
I= eye(6); 
d2r = pi/180;

Pi =[(20*d2r)^2 0 0 0 0 0; 
    0 (1*d2r)^2 0 0 0 0; 
    0 0 (1*d2r)^2 0 0 0;
    0 0 0 (100*dph2rps)^2 0 0; 
    0 0 0 0 (100*dph2rps)^2 0; 
    0 0 0 0 0 (100*dph2rps)^2];

%%
var_gyro = (20.0*dph2rps)^2;
beta = (1/tau);
dtkf = 0.1;
twobdt   = 2.0*beta*dtkf;
xponent2 = exp(-twobdt);
qdt_gyro = var_gyro *(1.0-xponent2);

Q = [0 0 0 0     0      0; 
     0 0 0 0     0      0; 
     0 0 0 0     0      0; 
     0 0 0 0.1^2 0      0; 
     0 0 0 0     0.1^2  0; 
     0 0 0 0     0      0.1^2];

H=[1 0 0 0 0 0; 
   0 1 0 0 0 0; 
   0 0 1 0 0 0]; 

R= [(10*d2r)^2      0           0; 
    0           (2*d2r)^2       0; 
    0               0       (2*d2r)^2];
%%
bias = [0;0;0];
fid = fopen('output1.dat','w');
for i = 1:lenth
    dt = 0.01;
    ww = [wx(i);wy(i);wz(i)] - bias;
    q(1) = q_p(1) - 0.5*(q_p(2)*ww(1) + q_p(3)*ww(2) + q_p(4)*ww(3))*dt;
    q(2) = q_p(2) + 0.5*(q_p(1)*ww(1) + q_p(3)*ww(3) - q_p(4)*ww(2))*dt;
    q(3) = q_p(3) + 0.5*(q_p(1)*ww(2) - q_p(2)*ww(3) + q_p(4)*ww(1))*dt;
    q(4) = q_p(4) + 0.5*(q_p(1)*ww(3) + q_p(2)*ww(2) - q_p(3)*ww(1))*dt;

    [xi_q,theta_q,phi_q] = quat2angle(q,'ZYX');

    %     xi_q = xi(i); theta_q = theta(i);  phi_q = phi(i);
    %%
    if rem(i,10) == 0;
        %%%Averaging Accelerometer and Magnetometer outputs upto 10 samples
        ac_ave1=0; ac_ave2=0; ac_ave3=0; mg_ave1=0; mg_ave2=0; mg_ave3=0;
        for k = kprev: knxt
            ac_ave1 = ac_ave1 + TestData(k,5);    mg_ave1 = mg_ave1 + TestData(k,8);
            ac_ave2 = ac_ave2 + TestData(k,6);    mg_ave2 = mg_ave2 + TestData(k,9);
            ac_ave3 = ac_ave3 + TestData(k,7);    mg_ave3 = mg_ave3 + TestData(k,10);
        end
        ac_1 = -ac_ave1/10;      mg_1 = mg_ave1/10;
        ac_2 = -ac_ave2/10;      mg_2 = mg_ave2/10;
        ac_3 = -ac_ave3/10;      mg_3 = mg_ave3/10;
        %%%% Calculating the pitch and roll from the Accelerometer accelerations
        phi_g = atan2(ac_2, ac_3);
        theta_g = atan2(-ac_1, sqrt(ac_2^2 +  ac_3^2));
        %%% Magnetometer output average vector ----------------------------
        Mb = [mg_1;mg_2;mg_3];
        %%% Decomposing the b-fram to n-frame by teh reverse rotation of the pitch and roll
        TM(1,1)= cos(theta_g);     TM(1,2)= sin(phi_g)*sin(theta_g);    TM(1,3)= sin(theta_g)*cos(phi_g);
        TM(2,1)= 0;                TM(2,2)= cos(phi_g);                 TM(2,3)= -sin(phi_g);
        TM(3,1)= -sin(theta_g);    TM(3,2)= cos(theta_g)*sin(phi_g);    TM(3,3)= cos(phi_g)*cos(theta_g);
        %%% Magnetic heading computation as xi_g
        Mw = TM*Mb;     xi_g = atan2 (-Mw(2), Mw(1));
        if((xi_g > pi/2) && (xi_g < -pi/2))
            xi_g = xi_g + 2*pi;
        elseif((xi_q < -pi/2) && (xi_g > pi/2))
            xi_g = xi_g - 2*pi;
        end
        %%% Weighted average --------
        xi_del    = xi_q - xi_g ;
        th_del = theta_q - theta_g ;
        phi_del = phi_q  - phi_g ;
        %% KALMAN FILTERING...............
        th= theta_q; 
        ph= phi_q;
        z = [xi_del; th_del; phi_del];

        Fi =I+ [0  sec(th)*tan(th)*(ww(2)*sin(ph)+ww(3)*cos(ph))        sec(th)*(ww(2)*cos(ph)-ww(3)*sin(ph))      0       sec(th)*sin(ph)         sec(th)*cos(ph)
                0   0                                                   (-ww(2)*sin(ph)-ww(3)*cos(ph))             0       cos(ph)                 -sin(ph)
                0  sec(th)^2*(ww(2)*sin(ph)+ww(3)*cos(ph))              (ww(2)*cos(ph)-ww(3)*sin(ph))              1       sin(ph)*tan(th)          cos(ph)*tan(th)
                0   0                                                       0                                   -1/tau          0                     0
                0   0                                                       0                                      0            -1/tau                 0
                0   0                                                       0                                      0             0                     -1/tau]*ddt;

        xp =Fi*xi;
        Pn = Fi*Pi*Fi' + Q*ddt;
        K = Pn*H'/(H*Pn*H' + R);
        xc = xp + K*(z-H*xp);
        Pnx = (I-K*H)*Pn;
        
        % correction .........
        xi_kc= xi_q-xc(1);  
        theta_kc= theta_q-xc(2);  
        phi_kc= phi_q-xc(3);
        bias = bias + xc(4:6);
        fprintf(fid,'%15.3f %15.6f %15.6f %15.6f %15.6f %15.6f %15.6f %15.6f %15.6f %15.6f %15.6f %15.6f %15.6f  %15.6f %15.6f %15.6f\n',TestData(i,1),xi_kc/d2r,theta_kc/d2r, phi_kc/d2r,xi_g/d2r,theta_g/d2r,phi_g/d2r,bias, sqrt(diag(Pnx)));

        % % disp(bias/dph2rps);
        % % disp(z*180/pi)
        %%
        Q_new = angle2quat(xi_kc, theta_kc, phi_kc, 'ZYX');
        q_p = Q_new;

        kprev = knxt +1;
        knxt = kprev + 9;
        Pi = Pnx;   
        xi = 0*xc;
    end
end
fclose(fid);
load output1.dat;
%%
h=figure(1);set(h,'name','heading');
plot(output1(:,1),output1(:,5),'k.-',output1(:,1),output1(:,2),'r.-',TestData(:,1),TestData(:,11),'b.-');legend('Magnetic Heading','estimated','Reference');
xlabel('time(s)'); ylabel('heading angle in degree');title('Heading angle comparison');grid on;shg;
saveas(h,'Heading','fig');
saveas(h,'Heading','png');


h=figure(2);set(h,'name','pitch');
plot(output1(:,1),output1(:,6),'k.-',output1(:,1),output1(:,3),'r.-',TestData(:,1),TestData(:,12),'b.-');legend('AccelMeas','estimated','Reference');
xlabel('time(s)'); ylabel('pitch angle in degree');title('pitch angle comparison');grid on;shg;
saveas(h,'Pitch_Angle','fig');
saveas(h,'Pitch_Angle','png');


h=figure(3);set(h,'name','roll');
plot(output1(:,1),output1(:,7),'k.-',output1(:,1),output1(:,4),'r.-',TestData(:,1),TestData(:,13),'b.-'); legend('AccelMeas','estimated','Reference');
xlabel('time(s)'); ylabel('roll angle in degree');title('roll angle comparison');grid on;shg;
saveas(h,'Roll_Angle','fig');
saveas(h,'Roll_Angle','png');
