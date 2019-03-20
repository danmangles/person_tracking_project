%% test ground truth
close all
% gnd_truth_file = "gnd_truth_cells_03_19_10.mat"
gnd_truth_file = "dataset_2_tracklets.mat"
load(gnd_truth_file);
%%
% for i = 1:size(var_to_save,2)
%     table_i = var_to_save{i};
%     fprintf('Displaying table %d\n',i)
%     figure    
%     plot3(table_i.Time,table_i.X,table_i.Y,'k.','LineWidth',1)
%     xlabel('Time (s)')
%     ylabel('Detection X (m)')
%     zlabel('Detection Y (m)')
%     
%     tx = table_i.X
%     ty = table_i.Y
%     tt = table_i.Time
% end
% true_tracklets = var_to_save
% t1x = true_tracklets{1}.X
% t1y = true_tracklets{1}.Y
% t1t = true_tracklets{1}.Time
% t2x = true_tracklets{2}.X
% t2y = true_tracklets{2}.Y
% t2t = true_tracklets{2}.Time
% t3x = true_tracklets{3}.X
% t3y = true_tracklets{3}.Y
% t3t = true_tracklets{3}.Time
%%
figure
t1x = tracklet_0(:,2)
t1y = tracklet_0(:,3)
t1t = tracklet_0(:,1)

plot3(t1t,t1x,t1y,'r.')
hold on

t2x = tracklet_1(:,2)
t2y = tracklet_1(:,3)
t2t = tracklet_1(:,1)
plot3(t2t,t2x,t2y,'b.')

t3x = tracklet_2(:,2)
t3y = tracklet_2(:,3)
t3t = tracklet_2(:,1)
plot3(t3t,t3x,t3y,'g.')


% % t1x = t1.X
% t1y = t1.Y
% t2 = true_tracklets{2}
% t3 = true_tracklets{3}