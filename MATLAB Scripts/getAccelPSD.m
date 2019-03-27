close all
clear all
% load the files
loadFiles %t1x_t etc
dt = 0.001

t = 0:dt:max(res_table.Time); % set times based on start and end times of test_table

% plots
figure
hold on
plot3(t,feval(t1x_t,t),feval(t1y_t,t),'g','LineWidth',2)
plot3(t,feval(t2x_t,t),feval(t2y_t,t),'b','LineWidth',2)
plot3(t,feval(t3x_t,t),feval(t3y_t,t),'r','LineWidth',2)
% labels
xlabel('Time (s)')
ylabel('X distance from "odom" frame (m)')
zlabel('Y distance from "odom" frame (m)')
axis([min(t) max(t) 0 22 0 22])


%% calculate velocity
[Va1, Vv1] = getVar(t1x_t,t1y_t,t)
[Va2, Vv2] = getVar(t2x_t,t2y_t,t)
[Va3, Vv3] = getVar(t3x_t,t3y_t,t)

Va = (Va1 + Va2 + Va3)/3
Vv = (Vv1 + Vv2 + Vv3)/3

dt_array = diff(res_table.Time);
tau = mean(dt_array ~= 0);
tau = 0.1
Sv = Vv/tau
Sa = Va/tau

%% functions
function plotPSD(time_domain_signal)
Fs=0.1
N= length(time_domain_signal);
xdft = fft(time_domain_signal);
xdft = xdft(1:N/2+1);
psdx = (1/(Fs*N)) * abs(xdft).^2;
psdx(2:end-1) = 2*psdx(2:end-1);
freq = 0:Fs/length(time_domain_signal):Fs/2;
figure
plot(freq,10*log10(psdx))
grid on
title('Periodogram Using FFT')
xlabel('Frequency (Hz)')
ylabel('Power/Frequency (dB/Hz)')
end

function v = getVelocity(x,t)
% returns acceleration of time series displacement a evaluated at times t
dx = diff(x)';
dt = diff(t);
% d2x = diff(x,2)';
% dt2 = diff(t,2);

v = dx./dt;
end

function a = getAcceleration(v,t)
% returns acceleration of time series displacement a evaluated at times t
dt = diff(t);
dv = diff(v);
a = dv./dt(2:end);



end

function plotXVA(x,v,a,t)
% plots x,v and a
figure
plot (t, x);
hold on
plot(t(2:end),v);

t2 = t(3:end);
plot(t2,a)
legend('displacement','velocity','acceleration')
xlabel('Time (s)')
ylabel('magnitude')
end

function [Va,Vv] = getVar(fitx, fity, t)
% returns the scalar velocity and acceleration variance of the fitx,fity
% evaluated across t
xx1 = feval(fitx,t);
xy1 = feval(fity,t);
x1 = sqrt(xx1.^2 + xy1.^2);

vx1 = getVelocity(xx1,t);
vy1 = getVelocity(xy1,t);
v1 = sqrt(vx1.^2 + vy1.^2);
ax1 = getAcceleration(vx1,t);
ay1 = getAcceleration(vy1,t);
a1 = sqrt(ax1.^2 + ay1.^2);

% plotXVA(x1,v1,a1,t)
Vv = cov(v1);
Va = cov(a1);

end
