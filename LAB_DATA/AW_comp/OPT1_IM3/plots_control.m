clc
clear
close all


control_15 = load("./DC_1.5/control_DAR.txt");
control_15_2Kg = load("./DC_1.5_2Kg/control_DAR.txt");

figure()
hold on; grid on;
plot(control_15(:,1),'r')
plot(control_15_2Kg(:,1),'k')
legend("no mass","2 Kg")
title('u1')

figure()
hold on; grid on;
plot(control_15(:,2),'r')
plot(control_15_2Kg(:,2),'k')
legend("no mass","2 Kg")
title('u2')



