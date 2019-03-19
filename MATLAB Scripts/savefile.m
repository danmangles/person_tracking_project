function savefile(rootname, var_to_save)

%% save to mat
% save to excel
formatOut = 'mm_dd_hh';
date = datestr(now,formatOut);
fileName = sprintf('%s_%s',rootname,date)
matName = strcat(fileName,'.mat');

save(matName,'true_tracklets')
fprintf('\n\nvariables written to mat file %s',matName);

end
