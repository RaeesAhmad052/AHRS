clc; clear all; close all; %#ok<*CLALL>

load out.dat;
r2d = 180/pi;

% h = figure(100); set(h,'name',' Times');
% ax(1)=subplot(211);plot(out(:,1), out(:,1), 'b.-', out(:,1), out(:,8), 'r.-' ); shg;grid on;
% ax(2)=subplot(212);plot(out(2:end,1), diff(out(:,1)), 'b.-', out(2:end,1), diff(out(:,8)), 'r.-' ); shg;grid on;
% xlabel('Time '); ylabel('Yaw [deg]'); legend('pp','RT');linkaxes(ax,'x');
%%
idx = find(out(:,5) < 0.0);
out(idx,5) = out(idx,5) +2*pi;

h = figure(1); set(h,'name',' Yaw Angle');
plot(out(:,1), out(:,5)*r2d, 'go-',out(:,1), out(:,11)*r2d, 'bO-', out(:,1), out(:,2)*r2d, 'r.-'  ); shg;grid on;
xlabel('Time '); ylabel('Yaw [deg]'); legend('measure','RT','PP');
%%
h = figure(2);  set(h,'name',' Roll Angle');
plot(out(:,1), out(:,7)*r2d,'go-', out(:,1), out(:,10)*r2d, 'b.-',out(:,1), out(:,4)*r2d, 'r.-' ); shg;grid on;
xlabel('Time '); ylabel('pitch [deg]'); legend('measure','RT','PP');
%%
h = figure(3);  set(h,'name',' Pitch Angle');
plot(out(:,1), out(:,6)*r2d,'go-', out(:,1), out(:,09)*r2d, 'b.-',out(:,8), out(:,3)*r2d, 'r.-' ); shg;grid on;
xlabel('Time '); ylabel('roll [deg]'); legend('measure','RT','PP');


