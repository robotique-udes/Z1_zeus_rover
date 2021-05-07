%%

% dinfo = dir(pwd + "/raw_data");
% dinfo(ismember( {dinfo.name}, {'.', '..'})) = [];
% 
% 
% 
% for k = 1 : length(dinfo)
%     
%     s = dir(fullfile('raw_data/' + convertCharsToStrings(dinfo(k).name) + "/", "*.csv"));
%     prevDir = '';
%     for kk = 1:numel(s)
%       if ~s(kk).isdir
%         oldName = s(kk).name;
%         [~,~,ext] = fileparts(oldName);
%         [~,currDir] = fileparts(s(kk).folder);
%         if ~strcmp(s(kk).folder,prevDir)
%           pref = 1;
%           prevDir = s(kk).folder;
%         else
%           pref = pref + 1;
%         end
%         newStr = erase(oldName,"_slash_ros_talon2_slash");
%         newName = sprintf('%s%s', currDir,newStr);
%         movefile(fullfile(s(kk).folder,oldName),fullfile(pwd,newName));
%         
%       end
%     end
%   
% end