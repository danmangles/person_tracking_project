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
FP = 0
FN = 0
IDSW = 0
GT = 0
DISTANCE_THRESHOLD = 0.4 % m 
dt = 0.3; % window size in seconds
mota = []
% for t = 0:dt:max(smaller_table.Time) % loop thru timestamps in table
times = min(test_table.Time):dt:max(test_table.Time)
mota = zeros(1,size(times,2))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
subplot(2,1,1)
xlabel('Time (s)')
ylabel('MOTA (%)')
axis([min(test_table.Time) max(test_table.Time) 0 100])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(2,1,2)
hold on

xlabel('Time (s)')
ylabel('X')
zlabel('Y')
for i = 1:size(true_tracklets,2)
    hold on
    table_i = true_tracklets{i};
    plot3(table_i.Time,table_i.X,table_i.Y,'k.','LineWidth',1)
    hold on
end

hold on

plot3(test_table.Time, test_table.KF_X, test_table.KF_Y,'ro') 

axis([min(test_table.Time) max(test_table.Time) 0 14 0 14])
hold on
for k = 1:size(times,2) % loop thru timestamps in table
    t = times(k)
    % extract the window
    test_window = test_table(test_table.Time < t +dt & test_table.Time > t, :);
    % generate a matrix of distances: columns denote true tracklets, rows
    % denote test_window entries
    fprintf('\n\n***************TIMESTEP %.1f\n',t)
    dist_matrix = ones(size(test_window,1),size(true_tracklets,2))*100;
    
    % loop through the test window, populating the distance matrix
    for j = 1:size(test_window) % loop thru each point in the window
        %%%%%%%%%%%%%%% 1. Associate tracklets with this point
        for i = 1:size(true_tracklets,2) %loop thru 3 tracklets
            GT = GT + 1;
            true_tracklet = true_tracklets{i};
            % get the window for this particular true tracklet
            true_window = true_tracklet(true_tracklet.Time < t +dt & true_tracklet.Time > t, :);
            
            % get median X and Y for cases where more than 1 detection falls
            % into this
            med_X = median(true_window.X);
            med_Y = median(true_window.Y);
            
            % populate dist_matrix
            dist_matrix(j,i) = sqrt((test_window.KF_X(j)-med_X).^2 + (test_window.KF_Y(j)-med_Y).^2); % get distance from this point to med value
            
        end
    end
    [M, associated_tracklet_index] = min(dist_matrix,[],2); % index of true tracklet for each test point
    
    fprintf('we are associating the test points with the following indices:\n')
    associated_tracklet_index
    matched_tracklets = [];
  
    %%%%%%%%%%%%%%%%%% for each test point, check IDs
    for j = 1:size(test_window) % loop thru each point in the window
        if M(j) > DISTANCE_THRESHOLD % register a false positive if min distance is too great
            fprintf('distance is %.3fm, registering false positive\n',M(j))
            FP = FP + 1;
        else
            matched_tracklets = [matched_tracklets, associated_tracklet_index(j)];
            ass_tracklet = true_tracklets{associated_tracklet_index(j)}; % get the tracklet at min distance to this test point

            %%%%%%%%%%%%%% 2. Check IDs of true and test tracklets, act accordingly
            % if this tracklet is not yet initialised
            if ass_tracklet.temp_ID(1) == -1 % if the 1st ID of this tracklet is -1, it hasn't been initialised
                % set the initial temp_ID of each tracklet to the ID of this test point
                ass_tracklet.temp_ID(1) = test_window.Tracklet_ID(j);
                fprintf('initialising tracklet %d to ID %d\n',associated_tracklet_index(j),test_window.Tracklet_ID(j))
                
            % if the ass tracklet has got a different ID to the test tracklet, switch it
            else
                if ass_tracklet.temp_ID(1)  ~= test_window.Tracklet_ID(j)
                    % we have an IDSW
                    IDSW = IDSW + 1; % increment
                    ass_tracklet.temp_ID(1) = test_window.Tracklet_ID(j);
                    fprintf('switched tracklet %d to ID %d\n',associated_tracklet_index(j),test_window.Tracklet_ID(j))
                else
                %else we have a correct association
                    fprintf('tracklet %d has the correct ID: %d\n',associated_tracklet_index(j),ass_tracklet.temp_ID(1))
                end
            end   
            true_tracklets{associated_tracklet_index(j)} = ass_tracklet; % modify the true_tracklets
        end
    end
    
    for i = 1:size(true_tracklets,2) % loop through thru associated tracklet indices. If there are any unmatched tracklets, increment FN
        if ~ismember(i, matched_tracklets)
            FN = FN + 1;
            fprintf('couldnt find a match for tracklet %d. Incrementing FN to %d',i,FN)
        end
    end
        
    mota(k) =  100.*(1 - (FN + FP + IDSW)/GT)
    %2. Evaluate FNs and FPs
    %     for i =
    %for each unassociated true tracklet
    % register a false negative
    
    % for each unassociated test point
    %  register a FP
    hold on
    subplot(2,1,1)
    hold on
    plot(t,mota(k),'rx')
    hold on
    pause(0.03)
end
%% if time, modify the IDSW algo to adjust test point ID not GND truth
FN
FP
IDSW
GT



%     %for each tracklet point
%     for i = 1:size(test_table,1)
%         for j = 1:size(true_tracklets) %loop thru 3 tracklets


