figure()
subplot(2, 3, 1)
plot(DATA.data_3d_40_1.averaged_current)
title("3D 40% - Compaction sans charge")
hold on
plot(DATA.data_3d_40_3_compacted.averaged_current)


subplot(2, 3, 4)
plot(DATA.data_3d_40_fresh_charge.averaged_current)
title("3D 40% - Compaction avec charge")
hold on
plot(DATA.data_3d_40_compacted_charge.averaged_current)



subplot(2, 3, 2)
plot(DATA.data_3d_70_1.averaged_current)
title("3D 40% - Compaction sans charge")
hold on
plot(DATA.data_3d_70_compacted.averaged_current)


subplot(2, 3, 5)
plot(DATA.data_3d_70_fresh_charge.averaged_current)
title("3D 40% - Compaction avec charge")
hold on
plot(DATA.data_3d_70_compacted_charge.averaged_current)



subplot(2, 3, 3)
plot(DATA.data_3d_40_1.averaged_current)
title("3D 40% - Compaction sans charge")
hold on
plot(DATA.data_3d_40_3_compacted.averaged_current)


subplot(2, 3, 6)
plot(DATA.data_3d_40_fresh_charge.averaged_current)
title("3D 40% - Compaction avec charge")
hold on
plot(DATA.data_3d_40_compacted_charge.averaged_current)