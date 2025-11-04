clc; clear all; close all; %#ok<*CLALL>

load out.dat;

r2d = 180/pi;

h = figure(1); set(h,'name',' Yaw Angle');
plot(out(:,1), out(:,5)*r2d, 'b.-', out(:,1), out(:,2)*r2d, 'r.-' ); shg;grid on;
xlabel('Time '); ylabel('Yaw [deg]'); legend('Measurement','Filter Aided Output');

h = figure(2);  set(h,'name',' Pitch Angle');
plot(out(:,1), out(:,6)*r2d, 'b.-',out(:,1), out(:,3)*r2d, 'r.-' ); shg;grid on;
xlabel('Time '); ylabel('pitch [deg]'); legend('Measurement','Filter Aided Output');

h = figure(3);  set(h,'name',' Roll Angle');
plot(out(:,1), out(:,7)*r2d, 'b.-',out(:,1), out(:,4)*r2d, 'r.-' ); shg;grid on;
xlabel('Time '); ylabel('roll [deg]'); legend('Measurement','Filter Aided Output');


