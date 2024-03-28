clc
clear
close all


error_no = load("./no_mass/error.txt");
error = load("./with_mass/error.txt");

figure()
hold on; grid on;
plot(error_no(:,1),'r')
plot(error(:,1),'k')
legend("no mass","2 Kg")
title('e1')

figure()
hold on; grid on;
plot(error_no(:,2),'r')
plot(error(:,2),'k')
legend("no mass","2 Kg")
title('e2')



