%% load gnd truth and results
% clc;clear all; close all;
loadFiles2 % loads results and ground truth

%% Extract the useful bit of the results table
test_table = res_table(:,{'Time','KF_X','KF_Y','Tracklet_ID'});
test_table = sortrows(test_table,'Time');
test_table(1:20,:) %print the first few rows

%% Set a temporary ID for each tracklet
tracklet_temp_IDs = ones(1,size(gnd_tracklet_array,2))*-1; % set all to -1 initially

%%

DISTANCE_THRESHOLD = 1; % m
%% times
dt = 0.7; % window size in seconds
START_TIME = 7; %s start this one late because the cfit objects are poorly behaved before this time
time_array = START_TIME:dt:max(test_table.Time); % set times based on start and end times of test_table

%% Setup MOTA params
global FP_array; global FN_array; global IDSW_array; global GT;
FP = 0;
FP_array = zeros(1,size(time_array,2)); % set MOTA as an array of zeros
FN = 0;
FN_array = zeros(1,size(time_array,2)); % set MOTA as an array of zeros
IDSW = 0;
IDSW_array = zeros(1,size(time_array,2)); % set MOTA as an array of zeros
GT = 0;
MOTA = zeros(1,size(time_array,2)); % set MOTA as an array of zeros

global n_gnd_truth_objects; global gnd_fit_array;
n_gnd_truth_objects = 3; % hardcoded: there are 3 people

%% INITIALISE THE PLOTS
figure
% first pane has the MOTA score
subplot(2,1,1)
hold on
title('Plot of MOTA against time. Symbols at bottom indicate FP (magenta triangle), FN (cyan cross), IDSW (blue cross)')
xlabel('Time (s)')
ylabel('MOTA (%)')
% legend('2','3','4')
axis([min(time_array) max(time_array) 0 100]) % AXES FOR THE MOTA %
% second pane has the gnd truth estimates
subplot(2,1,2)
% plots
hold on
plot3(time_array,feval(fit1x,time_array),feval(fit1y,time_array),'g','LineWidth',2)
plot3(time_array,feval(fit2x,time_array),feval(fit2y,time_array),'b','LineWidth',2)
plot3(time_array,feval(fit3x,time_array),feval(fit3y,time_array),'r','LineWidth',2)
% labels
xlabel('Time (s)')
ylabel('X distance from "odom" frame (m)')
zlabel('Y distance from "odom" frame (m)')
axis([min(time_array) max(time_array) 0 22 0 22])
% legend
hold on

%% %%%%%%%%%%%%%%%%%%%%%%% MAIN LOOP: Calculate MOTA at each time window
for k = 1:size(time_array,2) % loop thru timestamps in table
    
    t = time_array(k); % get current time
    fprintf('\n\n***************TIME %.2fs\n',t)
    
    % extract the window of results from the test_table
    test_window = test_table(test_table.Time < t +dt & test_table.Time > t, :);
    fprintf('There are %d datapoints in this test window\n',size(test_window,1))
    
    %% index of true tracklet for each test point by evaluating the distance
    % matrix at time t + dt/2 (halway thru window
    dist_matrix = getDistanceMatrix(test_window,gnd_fit_array, t+dt/2)
    [M, associated_tracklet_index] = min(dist_matrix,[],2);
    %     M
    %     associated_tracklet_index
    %     fprintf('we are associating the test points with the following indice:\n')
    matched_tracklets = []; % this array carries the IDs of all tracklets that have been matched, enabling us to calculate false negatives.
    false_positive_registered = boolean([0 0 0]);  % this array tells us if we've already registered an FP for this tracklet
    %%%%%%%%%%%%%%%%%% for each test point, check IDs
    for j = 1:size(test_window) % loop thru each point in the window
        if M(j) > DISTANCE_THRESHOLD & ~false_positive_registered(associated_tracklet_index(j)) % register a false positive if min distance is too great
            registerFP(t,test_window, j, M, associated_tracklet_index, k)
            false_positive_registered(associated_tracklet_index(j)) = 1; % prevent us registering another false positive from the same test point.
        else % e.g. we have a detection that's within range of this tracklet
            
            % add the ID of this tracklet to the list of matched tracklets so we know that it's been detected
            matched_tracklets = [matched_tracklets, associated_tracklet_index(j)];
            
            % get the tracklet at min distance to this test point
            ass_tracklet = gnd_tracklet_array{associated_tracklet_index(j)};
            
            % get the ID of the tracklet associated with this estimate
            true_id_of_this_tracklet = tracklet_temp_IDs(associated_tracklet_index(j));
            
            % if the 1st ID of this tracklet is -1, it hasn't been initialised
            if true_id_of_this_tracklet == -1
                % set the initial temp_ID of each tracklet to the ID of this test point
                true_id_of_this_tracklet = test_window.Tracklet_ID(j);
                fprintf('matching gnd_tracklet_%d to test_tracklet_%d\n',associated_tracklet_index(j),j)
                
            else
                % if the ass tracklet has got a different ID to the test tracklet, switch it
                if true_id_of_this_tracklet  ~= test_window.Tracklet_ID(j)
                    % register an IDSW, and plot it
                    true_id_of_this_tracklet = registerIDSW(t,test_window, associated_tracklet_index,true_id_of_this_tracklet, j, k)
                    
                else
                    %else we have a correct association
                    %                     fprintf('tracklet %d has the correct ID: %d\n',associated_tracklet_index(j),true_id_of_this_tracklet)
                end
            end
            % set the tracklet_id to whatever we decided
            tracklet_temp_IDs(associated_tracklet_index(j)) = true_id_of_this_tracklet; % modify the gnd_tracklet_array
        end
    end
    
    for i = 1:size(gnd_tracklet_array,2) % loop through thru associated tracklet indices. If there are any unmatched tracklets, increment FN
        if ~ismember(i, matched_tracklets)
            registerFN(t,i, k)
        end
    end
    
    GT = GT + n_gnd_truth_objects; %*size(test_window,1); % increment GT by number of gnd truth objects
    
%     MOTA(k) =  100.*(1 - (FN + FP + IDSW)/GT);
    MOTA(k) = getMOTAtoPlot(k);
    fprintf('\nGT = %d\nFN = %d\nFP = %d\nIDSW = %d\n',GT,FN,FP,IDSW)
    fprintf('\n\n**** MOTA = %.2fpc*****\n',MOTA(k))
    
    subplot(2,1,1)
    plot(t,MOTA(k),'rx')
    hold on
    pause(0.0001)
    %         pause
    
end


function dist_matrix = getDistanceMatrix(test_points, gnd_fit_array, t)
global n_gnd_truth_objects
%% returns the hungarian algo- style distance matrix
% between n test points and all gnd_fit_array objects evaluated at time t
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
        gnd_X = feval(gnd_fit_array{2*i-1},t); % gnd_X fit object evaluated halway across time windoww
        gnd_Y = feval(gnd_fit_array{2*i},t); % gnd_Y fit object evaluated halway across time window
        
        % populate dist_matrix
        dist_matrix(j,i) = sqrt((test_points.KF_X(j)-gnd_X).^2 + (test_points.KF_Y(j)-gnd_Y).^2); % get distance from this point to med value
    end
    colors_o = {'go','ro','bo','co','mo','yo','ro','go','bo','co','mo','yo','ro','go','bo','co','mo','yo'};
    color = colors_o{mod(test_points.Tracklet_ID(j),7)+1}; % wierd color formula
    
    subplot(2,1,2)
    plot3(t, test_points.KF_X(j), test_points.KF_Y(j),color,'LineWidth',1)
    
end

end

function true_id_of_this_tracklet = registerIDSW(t,test_window, associated_tracklet_index, true_id_of_this_tracklet, j, k)
% VOID: increments IDSW and plots a point on the X 3d plot, using j the
% index of this test point
global IDSW_array
% IDSW = IDSW + 1; % increment IDSW

IDSW_array(k) = IDSW_array(k)+1; % set a IDSW at this location in the array
% reset the ID of this tracklet
fprintf('gnd_tracklet_%d ID was %d but test_tracklet_%d has ID %d. Registering IDSW.\n',associated_tracklet_index(j),test_window.Tracklet_ID(j),j,true_id_of_this_tracklet)
true_id_of_this_tracklet = test_window.Tracklet_ID(j);

%plot
subplot(2,1,2)
hold on
plot3(t, test_window.KF_X(j), test_window.KF_Y(j),'bx','Markersize',30,'LineWidth',3)

end

function registerFN(t,i, k)
global FN_array; global gnd_fit_array
FN_array(k) = FN_array(k) + 1; % set a false negative at this location in the array
% FN = FN + 1;
fprintf('couldnt find a match for gnd_tracklet_%d. registering FN\n',i)

hold on
subplot(2,1,2)
hold on
% plot a cyan cross on the gnd truth where it was missed.
plot3(t, feval(gnd_fit_array{2*i-1},t), feval(gnd_fit_array{2*i},t),'cx','Markersize',20,'LineWidth',1)

end

function registerFP(t, test_window, j, M, associated_tracklet_index, k)

fprintf('distance from test_tracklet_%d to gnd_tracklet_%d is %.3fm, registering FP\n',j,associated_tracklet_index(j), M(j))

global FP_array
FP_array(k) = FP_array(k) + 1; % set a false positive at this location in the array
% FP = FP + 1;
hold on
subplot(2,1,2)
hold on
plot3(t, test_window.KF_X(j), test_window.KF_Y(j),'m^','Markersize',20,'LineWidth',1)
end

function MOTA = getMOTAtoPlot(k)

global FP_array; global FN_array; global IDSW_array; global n_gnd_truth_objects;
window_length = 10;
if k <= window_length
    k
    window_length = k-1
end

FP = sum(FP_array(k-window_length:k))
FN = sum(FN_array(k-window_length:k))
IDSW = sum(IDSW_array(k-window_length:k))
GT = n_gnd_truth_objects*(window_length+1)

MOTA =  100.*(1 - (FN + FP + IDSW)/GT)

end
