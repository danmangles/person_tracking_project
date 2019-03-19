function generateMatFileFromTrackletArrays(tracklet_A,tracklet_B,tracklet_C)

table_A = table(tracklet_A(:,1),tracklet_A(:,2),tracklet_A(:,3));
table_A.Properties.VariableNames = {'Time','X','Y'};

table_B = table(tracklet_B(:,1),tracklet_B(:,2),tracklet_B(:,3));
table_B.Properties.VariableNames = {'Time','X','Y'};

table_C = table(tracklet_C(:,1),tracklet_C(:,2),tracklet_C(:,3));
table_C.Properties.VariableNames = {'Time','X','Y'};

gnd_truth_cell_array = {table_A, table_B, table_C}

savefile('gnd_truth_cells',gnd_truth_cell_array)

end