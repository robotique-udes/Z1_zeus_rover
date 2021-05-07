clear all
clc
close all

%% GET DATA

dinfo = dir(pwd + "/raw_data");
dinfo(ismember( {dinfo.name}, {'.', '..', '.DS_Store'})) = [];

energy.printed = [];
energy.big = [];
energy.vex = [];
energy.fresh = [];
energy.compacted = [];

energy.twenty = [];
energy.forthy = [];
energy.seventy = [];


for k = 1 : length(dinfo)
    if dinfo(k).isdir
       DATA.("data_" + dinfo(k).name) = get_results(dinfo(k).name);
       data(k) = get_results(dinfo(k).name);
             
       if (contains(dinfo(k).name, "3d"))
            summary_data(k, :) = [1, data(k).energy_mAh, data(k).average_value, data(k).maximum_value, data(k).total_time];
            energy.printed = [energy.printed; data(k).averaged_current];
       end
       
       if (contains(dinfo(k).name, "big"))
            summary_data(k, :) = [2, data(k).energy_mAh, data(k).average_value, data(k).maximum_value, data(k).total_time];
            energy.big = [energy.big; data(k).averaged_current];
       end
       
       if (contains(dinfo(k).name, "vex"))
            summary_data(k, :) = [3, data(k).energy_mAh, data(k).average_value, data(k).maximum_value, data(k).total_time];
            energy.vex = [energy.vex; data(k).averaged_current];
       end
       
       if (contains(dinfo(k).name, "fresh"))
            summary_data2(k, :) = [1, data(k).energy_mAh];
            energy.fresh = [energy.fresh; data(k).averaged_current];
       end
       
       if (contains(dinfo(k).name, "compacted"))
            summary_data2(k, :) = [2, data(k).energy_mAh];
            energy.compacted = [energy.compacted; data(k).averaged_current];
       end
       
       
       
       if (contains(dinfo(k).name, "20") && ~contains(dinfo(k).name, "fresh"))
            summary_data3(k, :) = [1, data(k).energy_mAh];
            energy.twenty = [energy.twenty; data(k).averaged_current];
       end
       
       if (contains(dinfo(k).name, "40") && ~contains(dinfo(k).name, "fresh"))
            summary_data3(k, :) = [2, data(k).energy_mAh];
            energy.forthy = [energy.forthy; data(k).averaged_current];
          
       end
       
       if (contains(dinfo(k).name, "70") && ~contains(dinfo(k).name, "fresh"))
            summary_data3(k, :) = [3, data(k).energy_mAh];
            energy.seventy = [energy.seventy; data(k).averaged_current];
       end
       
    end
end


%%
summary_data3( all(~summary_data3,2), : ) = [];

%%

total_printed3 = mean(energy.printed);
total_big3 = mean(energy.big);
total_vex3 = mean(energy.vex);
total_fresh = mean(energy.fresh);
total_compacted = mean(energy.compacted);

total_20 = mean(energy.twenty);
total_40 = mean(energy.forthy);
total_70 = mean(energy.seventy);


%% GENERAL DATA
figure()
boxplot(summary_data(:,2), summary_data(:,1), 'Labels',{'3D','Big','Vex'})
ylabel("Énergie consommée (mAh)")
xlabel("Roue testée")
title("Distribution de l'énergie totale consommée par test")

figure()
boxplot(summary_data(:,3), summary_data(:,1), 'Labels',{'3D','Big','Vex'})
ylabel("Courant moyen (A)")
xlabel("Roue testée")
title("Distribution du courant moyen par test")

figure()
boxplot(summary_data(:,4), summary_data(:,1), 'Labels',{'3D','Big','Vex'})
ylabel("Courant maximum (A)")
xlabel("Roue testée")
title("Distribution du courant maximum atteinte par test")

figure()
boxplot(summary_data(:,5), summary_data(:,1), 'Labels',{'3D','Big','Vex'})
ylabel("Temps (s)")
xlabel("Roue testée")
title("Distribution du temps total par test")

%% ENERGY VS SOIL STATE
figure()
boxplot(summary_data2(:,2), summary_data2(:,1), 'Labels',{'Frais','Compacté'})
ylabel("Énergie consommée (mAh)")
xlabel("État du sol")
title("Distribution de l'énergie consommée par test")


%% ENERGY VS SPEED
figure()
boxplot(summary_data3(:,2), summary_data3(:,1), 'Labels',{'20%','40%', '70%'})
ylabel("Énergie consommée (mAh)")
xlabel("Vitesse de rotation de la roue")
title("Distribution de l'énergie consommée par test")

%%
% plot3D
% plotVex
% plotBig
% plotComparison