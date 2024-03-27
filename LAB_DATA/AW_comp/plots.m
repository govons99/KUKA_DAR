clc
clear
close all


error_IM2_15 = load("./OPT1_IM2/DC_1.5/error.txt");
error_IM3_15 = load("./OPT1_IM3/DC_1.5/error.txt");

control_IM2_15 = load("./OPT1_IM2/DC_1.5/control_DAR.txt");
control_IM3_15 = load("./OPT1_IM3/DC_1.5/control_DAR.txt");


figure()
hold on; grid on;
plot(error_IM2_15(:,1),'b')
plot(error_IM3_15(:,1),'r')
legend("IM2","IM3")
title('e1')


figure()
hold on; grid on;
plot(error_IM2_15(:,2),'b')
plot(error_IM3_15(:,2),'r')
legend("IM2","IM3")
title('e2')

figure()
hold on; grid on;
plot(control_IM2_15(:,1),'b')
plot(control_IM3_15(:,1),'r')
legend("IM2","IM3")
title('u1')


figure()
hold on; grid on;
plot(control_IM2_15(:,2),'b')
plot(control_IM3_15(:,2),'r')
legend("IM2","IM3")
title('u2')



