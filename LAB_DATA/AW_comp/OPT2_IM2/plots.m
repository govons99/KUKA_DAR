clc
clear
close all


error_05 = load("./DC_0.5/error.txt");
error_1 = load("./DC_1/error.txt");
error_15 = load("./DC_1.5/error.txt");

control_05 = load("./DC_0.5/control_DAR.txt");
control_1 = load("./DC_1/control_DAR.txt");
control_15 = load("./DC_1.5/control_DAR.txt");


figure()
hold on; grid on;
plot(error_05(:,1),'b')
plot(error_1(:,1),'r')
plot(error_15(:,1),'k')
legend("tau2 0.5","tau2 1","tau2 1.5")
title('e1')

figure()
hold on; grid on;
plot(error_05(:,2),'b')
plot(error_1(:,2),'r')
plot(error_15(:,2),'k')
legend("tau2 0.5","tau2 1","tau2 1.5")
title('e2')

figure()
hold on; grid on;
plot(control_05(:,1),'b')
plot(control_1(:,1),'r')
plot(control_15(:,1),'k')
legend("tau2 0.5","tau2 1","tau2 1.5")
title('u1')

figure()
hold on; grid on;
plot(control_05(:,2),'b')
plot(control_1(:,2),'r')
plot(control_15(:,2),'k')
legend("tau2 0.5","tau2 1","tau2 1.5")
title('u2')



