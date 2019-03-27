close all
% load the files
loadFiles2


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
figure
for i = 1:size(tracklet_table_array)
% for i = 1
figure
   % plot the innovation of this tracklet
    this_table = tracklet_table_array{i,2}(:,:); % get the current table]
    subplot(2,1,1)
    plot(this_table.Time, this_table.v_X,'r.')
   hold on
    plot(this_table.Time, zeros(1,size(this_table.Time,1)),'k')
    title('Plot of Innovation in X and Y against Time')
    grid on
    xlabel('Time (s)')
    ylabel('X Innovation (m)')
    axis([min(this_table.Time) max(this_table.Time) -2.5 2.5])
    subplot(2,1,2)
    plot(this_table.Time, this_table.v_Y,'g.')
    meanX = nanmean(this_table.v_X) 
    meanY = nanmean(this_table.v_Y)
    hold on
    plot(this_table.Time, zeros(1,size(this_table.Time,1)),'k')
    xlabel('Time (s)')
    ylabel('Y Innovation (m)')
    grid on
    axis([min(this_table.Time) max(this_table.Time) -2.5 2.5])
    pause
end


%% plot

% 
% hold on
% plot(res_table.Time, zeros(1,size(res_table.Time,1)),'k')
% title('Plot of Innovation in X and Y against Time')
% xlabel('Time (s)')
% ylabel('X Innovation (m)')
% subplot(2,1,2)
% plot(res_table.Time, res_table.v_Y,'g.')
% meanY = nanmean(res_table.v_Y)
% hold on
% plot(res_table.Time, zeros(1,size(res_table.Time,1)),'k')
% xlabel('Time (s)')
% ylabel('Y Innovation (m)')