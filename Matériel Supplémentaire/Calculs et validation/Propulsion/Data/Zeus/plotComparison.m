figure()
subplot(2, 1, 1)
plot(DATA.data_3d_20_fresh.averaged_current)
title("20% Fresh No Load")
hold on
plot(DATA.data_big_20_fresh.averaged_current)
plot(DATA.data_vex_20_fresh.averaged_current)
legend('3D','Big', 'Vex')

subplot(2, 1, 2)
plot(DATA.data_3d_40_1.averaged_current)
title("40% Fresh No Load")
hold on
plot(DATA.data_big_40_fresh.averaged_current)
plot(DATA.data_vex_40_fresh.averaged_current)
legend('3D','Big', 'Vex')

