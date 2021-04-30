figure()
subplot(2, 3, 1)
plot(DATA.data_vex_20_fresh.averaged_current)
title("Big 20% - Compaction sans charge")
hold on
plot(DATA.data_vex_20_compacted.averaged_current)


subplot(2, 3, 4)
plot(DATA.data_big_20_fresh_charge.averaged_current)
title("Big 20% - Compaction avec charge")
hold on
plot(DATA.data_big_20_compacted_charge.averaged_current)



subplot(2, 3, 2)
plot(DATA.data_big_40_fresh.averaged_current)
title("Big 40% - Compaction sans charge")
hold on
plot(DATA.data_big_40_compacted.averaged_current)


subplot(2, 3, 5)
title("Big 40% - Compaction avec charge")
hold on
plot(DATA.data_big_40_compacted_charge.averaged_current)



subplot(2, 3, 3)
plot(DATA.data_big_70_fresh.averaged_current)
title("Vec 70% - Compaction sans charge")
hold on
plot(DATA.data_big_70_compacted.averaged_current)


subplot(2, 3, 6)
plot(DATA.data_big_70_fresh_charge.averaged_current)
title("Big 70% - Compaction avec charge")
hold on
plot(DATA.data_big_70_compacted_charge.averaged_current)