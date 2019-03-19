%% test ground truth
close all
gnd_truth_file = "gnd_truth_cells_03_19_10.mat"
load(gnd_truth_file);

for i = 1:size(var_to_save,2)
    table_i = var_to_save{i};
    fprintf('Displaying table %d\n',i)
    figure    
    plot3(table_i.Time,table_i.X,table_i.Y,'k.','LineWidth',1)
    xlabel('Time (s)')
    ylabel('Detection X (m)')
    zlabel('Detection Y (m)')
    
    tx = table_i.X
    ty = table_i.Y
    tt = table_i.Time
end
true_tracklets = var_to_save
t1x = true_tracklets{1}.X
t1y = true_tracklets{1}.Y
t1t = true_tracklets{1}.Time
t2x = true_tracklets{2}.X
t2y = true_tracklets{2}.Y
t2t = true_tracklets{2}.Time
t3x = true_tracklets{3}.X
t3y = true_tracklets{3}.Y
t3t = true_tracklets{3}.Time
% % t1x = t1.X
% t1y = t1.Y
% t2 = true_tracklets{2}
% t3 = true_tracklets{3}