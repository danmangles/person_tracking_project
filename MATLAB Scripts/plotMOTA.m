%% load gnd truth and results
% clc;clear all; close all;
loadFiles % loads results and ground truth
% plot_ground_truth % loads and plots the ground truth

%% Extract the useful bit of the results table
test_table = res_table(:,{'Time','KF_X','KF_Y','Tracklet_ID'});
test_table = sortrows(test_table,'Time')
test_table(1:20,:) %print the first few rows

%% Set a temporary ID for each tracklet
tracklet_temp_IDs = ones(1,size(true_tracklet_cell,2))*-1 % set all to -1 initially

%%

DISTANCE_THRESHOLD = 0.4; % m
%% times
dt = 0.3; % window size in seconds
time_array = min(test_table.Time):dt:max(test_table.Time); % set times based on start and end times of test_table
%% Setup MOTA params
FP = 0;
FP_array = NaN(1,size(time_array,2)); % set MOTA as an array of zeros
FN = 0;
FN_array = NaN(1,size(time_array,2)); % set MOTA as an array of zeros
IDSW = 0;
IDSW_array = NaN(1,size(time_array,2)); % set MOTA as an array of zeros
GT = 0;
MOTA = zeros(1,size(time_array,2)); % set MOTA as an array of zeros
global n_gnd_truth_objects;
n_gnd_truth_objects = 3; % hardcoded: there are 3 people
%% INITIALISE THE PLOTS
figure
% first pane has the MOTA score
subplot(2,1,1)
title('Plot of MOTA against time. Symbols at bottom indicate FN (cyan), FN (red), IDSW (blue cross)')
hold on
xlabel('Time (s)')
ylabel('MOTA (%)')
% legend('2','3','4')
axis([min(test_table.Time) max(test_table.Time) 0 100]) % AXES FOR THE MOTA %
hold on
% second pane has the gnd truth estimates
subplot(2,1,2)
hold on
% plots
            hold on
plot3(time_array,feval(t1x_t,time_array),feval(t1y_t,time_array),'r','LineWidth',2)
            hold on
plot3(time_array,feval(t2x_t,time_array),feval(t2y_t,time_array),'g','LineWidth',2)
plot3(time_array,feval(t3x_t,time_array),feval(t3y_t,time_array),'b','LineWidth',2)
            hold on
% labels
xlabel('Time (s)')
ylabel('X distance from "odom" frame (m)')
zlabel('Y distance from "odom" frame (m)')
axis([min(test_table.Time) max(test_table.Time) 0 16 0 16])
% legend
hold on

%% MAIN LOOP: Calculate MOTA at each time window
for k = 1:size(time_array,2) % loop thru timestamps in table
    t = time_array(k); % get current time
    fprintf('\n\n***************TIME %.2fs\n',t)
    
    % extract the window of results from the test_table
    test_window = test_table(test_table.Time < t +dt & test_table.Time > t, :) 
    
    %% index of true tracklet for each test point by evaluating the distance
    % matrix at time t + dt/2 (halway thru window
    dist_matrix = getDistanceMatrix(test_window,gnd_truth_fits, t+dt/2)
    [M, associated_tracklet_index] = min(dist_matrix,[],2);
    fprintf('we are associating the test points with the following indice:\n')
    associated_tracklet_index;
    matched_tracklets = [];
    
    %%%%%%%%%%%%%%%%%% for each test point, check IDs
    for j = 1:size(test_window) % loop thru each point in the window
        if M(j) > DISTANCE_THRESHOLD % register a false positive if min distance is too great
            fprintf('distance is %.3fm, registering false positive\n',M(j))
            FP = FP + 1;
            hold on
                    subplot(2,1,1)
                    hold on
            plot(t, 0,'ro','LineWidth',2)
        else
            matched_tracklets = [matched_tracklets, associated_tracklet_index(j)];
            ass_tracklet = true_tracklet_cell{associated_tracklet_index(j)}; % get the tracklet at min distance to this test point
            
            %%%%%%%%%%%%%% 2. Check IDs of true and test tracklets, act accordingly
            % if this tracklet is not yet initialised
            true_id_of_this_tracklet = tracklet_temp_IDs(associated_tracklet_index(j));
            if true_id_of_this_tracklet == -1 % if the 1st ID of this tracklet is -1, it hasn't been initialised
                % set the initial temp_ID of each tracklet to the ID of this test point
                true_id_of_this_tracklet = test_window.Tracklet_ID(j);
                fprintf('initialising tracklet %d to ID %d\n',associated_tracklet_index(j),test_window.Tracklet_ID(j))
                
            % if the ass tracklet has got a different ID to the test tracklet, switch it
            else
                if true_id_of_this_tracklet  ~= test_window.Tracklet_ID(j)
                    % we have an IDSW
                    IDSW = IDSW + 1; % increment
                    subplot(2,1,1)
                    hold on
                    plot(t, 0,'bx','MarkerSize',20,'LineWidth',2)
                    true_id_of_this_tracklet = test_window.Tracklet_ID(j);
                    fprintf('switched tracklet %d to ID %d\n',associated_tracklet_index(j),test_window.Tracklet_ID(j))
                else
                    %else we have a correct association
                    fprintf('tracklet %d has the correct ID: %d\n',associated_tracklet_index(j),true_id_of_this_tracklet)
                end
            end
            % set the tracklet_id to whatever we decided
            tracklet_temp_IDs(associated_tracklet_index(j)) = true_id_of_this_tracklet; % modify the true_tracklet_cell
        end
    end
    
    for i = 1:size(true_tracklet_cell,2) % loop through thru associated tracklet indices. If there are any unmatched tracklets, increment FN
        if ~ismember(i, matched_tracklets)
            FN = FN + 1;
            hold on
                    subplot(2,1,1)
                    hold on
            plot(t, 0,'co','LineWidth',2)
            fprintf('couldnt find a match for tracklet %d. Incrementing FN to %d',i,FN)
        end
    end
    
    GT = GT + n_gnd_truth_objects; % increment GT by number of gnd truth objects
    
    MOTA(k) =  100.*(1 - (FN + FP + IDSW)/GT);
    %2. Evaluate FNs and FPs
    %     for i =
    %for each unassociated true tracklet
    % register a false negative
    
    % for each unassociated test point
    %  register a FP
    subplot(2,1,1)
    plot(t,MOTA(k),'rx')
    hold on
    %% plot the progress of the tracks
%     subplot(2,1,2)
%     plot3(
    
    
    pause(0.005)
end


function dist_matrix = getDistanceMatrix(test_points, gnd_truth_fits, t)
global n_gnd_truth_objects
%% returns the hungarian algo- style distance matrix 
% between n test points and all gnd_truth_fits objects evaluated at time t
% param: test_points: a table of points in the window with fields
%        KF_X: X coord of a test point
%        KF_Y: Y coord
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% preallocate: 3 is the number of people in the dataset (should make more
% robust later)
dist_matrix = ones(size(test_points,1),n_gnd_truth_objects)*100;

% loop through the test window, populating the distance matrix
for j = 1:size(test_points) % loop thru each point in the window
    % 1. Associate tracklets with this point
    for i = 1:n_gnd_truth_objects %loop thru 6 cells, skip every 2nd bc we process 2 at a time      
        gnd_X = feval(gnd_truth_fits{2*i-1},t); % gnd_X fit object evaluated halway across time windoww
        gnd_Y = feval(gnd_truth_fits{2*i},t); % gnd_Y fit object evaluated halway across time window
        
        % populate dist_matrix
        dist_matrix(j,i) = sqrt((test_points.KF_X(j)-gnd_X).^2 + (test_points.KF_Y(j)-gnd_Y).^2); % get distance from this point to med value
    end
    colors_o = {'go','ro','bo','co','mo','yo','ro','go','bo','co','mo','yo','ro','go','bo','co','mo','yo'};
    color = colors_o{mod(test_points.Tracklet_ID(j)+1,7)}; % wierd color formula
    
    subplot(2,1,2)
    plot3(t, test_points.KF_X(j), test_points.KF_Y(j),color,'LineWidth',1)

end

end
