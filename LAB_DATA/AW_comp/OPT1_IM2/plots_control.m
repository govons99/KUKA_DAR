clc
clear
close all


control_15 = load("./DC_1.5/control_DAR.txt");
control_15_2Kg = load("./DC_1.5_2Kg/control_DAR.txt");
control_15_2Kg_w = load("./DC_1.5_2Kg_wrong/control_DAR.txt");

figure()
hold on; grid on;
plot(control_15(:,1),'r')
plot(control_15_2Kg(:,1),'k')
plot(control_15_2Kg_w(:,1),'b')
legend("no mass","2 Kg","wrong")
title('u1')

figure()
hold on; grid on;
plot(control_15(:,2),'r')
plot(control_15_2Kg(:,2),'k')
plot(control_15_2Kg_w(:,2),'b')
legend("no mass","2 Kg","wrong")
title('u2')



