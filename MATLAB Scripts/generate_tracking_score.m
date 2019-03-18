%% add spreadsheet folder to path
filepath = '/home/ori/catkin_ws/src/multi_sensor_tracker/results_CSVs';
addpath(filepath)

%% results file
res_filename = "res_0318_151.csv";
res_table = readtable(res_filename);
fprintf('res_table has size (%d,%d)',size(res_table,1),size(res_table,2))
res_table(1:3,:) %print the first few rows

%% ground truth file
gnd_filename = "gnd_0318_151.csv";
gnd_table = readtable(gnd_filename);
fprintf('gnd_table has size (%d,%d)',size(gnd_table,1),size(gnd_table,2))
gnd_table(1:3,:) %print the first few rows

%% Split into tracklet tables
max_tracklet_id = max(res_table.Tracklet_ID);
tracklet_table_array = cell(1,2);
cell_length = 1;

for i = 1:max_tracklet_id
    tracklet_table = res_table(res_table.Tracklet_ID == i,:);
    if size(tracklet_table,1 > 0) % if this table contains data, add it to the cell array
        tracklet_table_array(cell_length,:) = {i,tracklet_table};
        cell_length = cell_length + 1; %counter for cell array
        fprintf('adding table with id %d to trackle_table_array\n',i)
    end
end

%% 
% every 10 results, plot 1) the estimate and 2 a covariance ellipse
figure
subplot(2,1,1)
plot(gnd_table.Time,gnd_table.Detection_X,'k.','LineWidth',1)

subplot(2,1,2)
plot(gnd_table.Time,gnd_table.Detection_Y,'k.','LineWidth',1)

%%
colors_o = {'ro','go','bo','co','mo','yo','ro','go','bo','co','mo','yo','ro','go','bo','co','mo','yo'};
colors = {'r','g','b','c','m','y','r','g','b','c','m','y','r','g','b','c','m','y'};


%% setup the figure

hold on
filter_freq = 3; % remove 3 out of 4 results
nSigma = 0.05;
fprintf('we are looping through %d tracklets',size(tracklet_table_array,1))

for i=1:size(tracklet_table_array)
% for i = 1:1
    table = tracklet_table_array{i,2}(:,:); % get the current table
    hold on
%     PlotCovarianceEllipseFromTable(table, colors{i})
    subplot(2,1,1)
    hold on
    plot(table.Time, table.KF_X,colors_o{i},'LineWidth',.2)
    
%     y plot
    subplot(2,1,2)
    hold on
    plot(table.Time, table.KF_Y,colors_o{i},'LineWidth',.2)
end

% figure stuff
subplot(2,1,1)
title('Plot of tracklet X and Y values against time')
xlabel('Time (s)')
ylabel('Detection and Tracklet X (m)')
% axis([0 50 0 15])
% 
subplot(2,1,2)
xlabel('Time (s)')
ylabel('Detection and Tracklet Y (m)')
% axis([0 50 0 15])
saveas(gcf,'detections_and_tracklets_0.eps')
saveas(gcf,'detections_and_tracklets_0.png')

%% get current score

