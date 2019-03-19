function doEDAPlots(res_table, gnd_table)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
%%
figure
subplot(3,1,1)
plot(gnd_table.Time, gnd_table.Detection_X, 'rx')
title('Detection distances from odom over time')

xlabel('Time (s)')
ylabel('Detection X (m)')
subplot(3,1,2)
plot(gnd_table.Time, gnd_table.Detection_Y, 'rx')
xlabel('Time (s)')
ylabel('Detection Y (m)')
subplot(3,1,3)
plot(gnd_table.Time, gnd_table.Detection_Z, 'rx')
xlabel('Time (s)')
ylabel('Detection Z (m)')

%%
figure
subplot(3,1,1)
plot(res_table.Time, res_table.KF_X, 'gx')
title('KF Position state over time')

xlabel('Time (s)')
ylabel('KF_X (m)')
subplot(3,1,2)
plot(res_table.Time, res_table.KF_Y, 'gx')
xlabel('Time (s)')
ylabel('KF_Y (m)')
subplot(3,1,3)
plot(res_table.Time, res_table.KF_Z, 'gx')
xlabel('Time (s)')
ylabel('KF_Z (m)')

fprintf('EDA Plots Generated.')
end

