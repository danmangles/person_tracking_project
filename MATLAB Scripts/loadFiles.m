%% loads a bunch of stuff into work place
load("gnd_truth_cells_03_19_10.mat");
load('gnd_truth_fits.mat');
gnd_truth_fits = {t1x_t, t1y_t, t2x_t, t2y_t, t3x_t, t3y_t}; % put stuff in a cell array
% clear t1x_t; clear t1y_t; clear  t2x_t; clear  t2y_t; clear t3x_t; clear t3y_t;

%% hack: true_tracklet_array
true_tracklet_cell = var_to_save; %better name
clear var_to_save % delete the copy

%% Results
% add spreadsheet folder to path
filepath = '/home/ori/catkin_ws/src/multi_sensor_tracker/results_CSVs';
addpath(filepath)
% results file
res_filename = "res_0319_959.csv";
res_table = readtable(res_filename);
fprintf('res_table has size (%d,%d)',size(res_table,1),size(res_table,2))
res_table(1:3,:) %print the first few rows

