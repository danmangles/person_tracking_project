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
end
true_tracklets = var_to_save

%% get MOTA
%% add spreadsheet folder to path
filepath = '/home/ori/catkin_ws/src/multi_sensor_tracker/results_CSVs';
addpath(filepath)
% results file
res_filename = "res_0319_959.csv";
res_table = readtable(res_filename);
fprintf('res_table has size (%d,%d)',size(res_table,1),size(res_table,2))
res_table(1:3,:) %print the first few rows
%%
test_table = res_table(:,{'Time','KF_X','KF_Y','Tracklet_ID'});
test_table(1:3,:) %print the first few rows
test_table = sortrows(test_table,'Time')
test_table(1:20,:) %print the first few rows


%%
% for each time window
dt = 0.4; % window size in seconds
GT = 0
for i = 1:size(true_tracklets,2)
    true_tracklet = true_tracklets{i};
    %     true_tracklet. = [true_tracklet.Properties.VariableNames, 'temp_ID']
    true_tracklet.temp_ID = ones(size(true_tracklet,1),1)*-1
    
    true_tracklets{i} = true_tracklet;
end
%% Conduct initial tracklet association to setup

% setup temp_IDs
% associate

%%
% for t = 0:dt:max(smaller_table.Time) % loop thru timestamps in table
for t = .2:dt:0.4 % loop thru timestamps in table
    
    for i = 1:size(true_tracklets,2) %loop thru 3 tracklets
        true_tracklet = true_tracklets{i};
        true_window = true_tracklet(true_tracklet.Time < t +dt & true_tracklet.Time > t, :)
        
        % get median X and Y for cases where more than 1 detection falls
        % into this
        med_X = median(true_window.X);
        med_Y = median(true_window.Y);
        
        % populate dist_matrix
        for j = 1:size(test_window) %loop through test points in window
            dist_matrix(j,i) = sqrt((test_window.KF_X(j)-med_X).^2 + (test_window.KF_Y(j)-med_Y).^2); % get distance from this point to med value
        end
    end
    
    dist_matrix
    [M, I] = min(dist_matrix,[],2) % index of true tracklet for each test point
    
    % if this tracklet is not yet initialised
    if true_tracklets{I}.temp_ID(1) == -1
        % set the initial temp_ID of each tracklet to the ID of this test point
        true_tracklets{I}.temp_ID = ones(size(true_tracklets{I},1),1)*test_window.Tracklet_ID;
        true_tracklets{I}
        
        
        % if the tracklet has got a different ID, switch it
    else
        if true_tracklets{I}.temp_ID(1) != test_window.Tracklet_ID(j)
            true_tracklets{I}.temp_ID = ones(size(true_tracklets{I},1),1)*test_window.Tracklet_ID;
            
        end
    end
    
    
    for j = 1:size(test_window) %loop through test points in window
        true_ID = true_tracklets{I(j)}.temp_ID
        test_ID = test_window.tracklet_ID{j}
    end
    % do we register an IDSW?
    %     for
    true_ID = true_tracklets{I}.temp_ID
    % if that test point has a different ID
    % update the test ID
    % increment IDSW
    
    %2. Evaluate FNs and FPs
    %     for i =
    %for each unassociated true tracklet
    % register a false negative
    
    % for each unassociated test point
    %  register a FP
end
%% if time, modify the IDSW algo to adjust test point ID not GND truth



%     %for each tracklet point
%     for i = 1:size(test_table,1)
%         for j = 1:size(true_tracklets) %loop thru 3 tracklets


