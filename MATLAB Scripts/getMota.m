function mota = getMota(res_table)
% INPUTS res_table: a table of all groundtruth points with NaNs where there
% are no detections
% calculates FP, FN, IDSW, GT

FN = 0 % number of false negatives
FP = 0 % number of false positives
IDSW = 0 % number of ID switches
GT = 0 % number of ground truth objects
DIST_THRESHOLD = 0.5 %m threshold beyond which we count a tracklet as a false positive

% loop through results table
for i = 1:size(res_table)
    % get the numbers out
   x_hat = res_table.KF_X(i)
   y_hat = res_table.KF_Y(i)
   x = res_table.Detection_X(i)
   y = res_table.Detection_Y(i)
    
    % if KF_X or KF_Y are undefined then we have a missed detection
    % increment the false negatives
    if (isnan(x_hat) || isnan(y_hat))
        FN = FN + 1;
        
    % if KF state is more than 0.5m from a detection, we have a false
    % positive
    else if sqrt((x-x_hat)^2 + (y-y_hat)^2) > DIST_THRESHOLD
       FP = FP + 1;
    end
    
    % if Tracklet_ID from 1 detection to the next is different AND gnd
    % truth did not switch, increment IDSW
    if res_table.Tracklet_ID(i) != res_table.Tracklet_ID(i-1) && res_table.true_tracklet_id(i) != res_table.true_tracklet_id(i-1)
        IDSW = IDSW + 1;
    end
    
    % increment number of GT objects regardless
    GT = GT + 1;
end

mota = 1 - (FN + FP + IDSW)/GT

end

