%% loads a bunch of stuff into work place
load("dataset_2_gnd_truth_mod.mat");
gnd_fit_array = {fit1x, fit1y, fit2x, fit2y, fit3x, fit3y}; % put stuff in a cell array
gnd_tracklet_array = {tracklet_0, tracklet_1, tracklet_2};
%% hack: true_tracklet_array

%% Results
% add spreadsheet folder to path
filepath = '/home/ori/catkin_ws/src/multi_sensor_tracker/results_CSVs';
addpath(filepath)
% results file
% res_filename = "res_0320_1247.csv"; % ds2,0.1
% res_filename = "res_0321_1529.csv"; % ds2,0.5
res_filename = "res_0327_1754.csv"; % ds2,0.5
res_table = readtable(res_filename);
fprintf('res_table has size (%d,%d)',size(res_table,1),size(res_table,2))
res_table(1:3,:) %print the first few rows

