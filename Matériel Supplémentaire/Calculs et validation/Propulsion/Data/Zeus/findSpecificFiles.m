function [specificFilesToAnalyzeInFolder] = findSpecificFiles(folderPath, fileExtensionToAnalyze)
%findSpecificFiles Output a list of all desired file within a certain
%folder
%     fullSubDirectoryPath = [mainDirectoryToAnalyze + "\" + subDirectoryToAnalyze + "\"];
    specificFilesToAnalyzeInFolder = dir(fullfile(folderPath, fileExtensionToAnalyze));
end

