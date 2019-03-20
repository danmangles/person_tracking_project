%% plot ground truth
t = 0:0.1:75
figure
plot3(t,feval(t1x_t,t),feval(t1y_t,t),'r','LineWidth',2)
hold on
plot3(t,feval(t2x_t,t),feval(t2y_t,t),'g','LineWidth',2)
plot3(t,feval(t3x_t,t),feval(t3y_t,t),'b','LineWidth',2)
hold on
for i = 1:size(true_tracklet_cell,2)
    table_i = true_tracklet_cell{i};
    fprintf('Displaying table %d\n',i)
    plot3(table_i.Time,table_i.X,table_i.Y,'k.','LineWidth',1)
    xlabel('Time (s)')
    ylabel('Detection X (m)')
    zlabel('Detection Y (m)')
end
title('Smoothed ground truth against detection points')
legend('Tracklet 0','Tracklet 1','Tracklet 2')