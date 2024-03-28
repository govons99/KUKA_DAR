clc
clear
close all

q = load("Q.txt");

error = load("error.txt");

eta = load("eta.txt");
control_DAR = load("control_DAR.txt");

%% reference

t = 0:0.0005:300;

%ref = [q(1,2) + 0.1*(1-cos(t));q(1,4) + 0.1*(1-cos(t))];
ref = [0.4*cos(t);-0.4*cos(t)];
figure()
hold on; grid on;
plot(eta)
title('eta')

figure()
hold on; grid on;
plot(error(:,1),'b')
plot(error(:,2),'r')
title('error')

%% Joint position

figure()
hold on; grid on;
plot(q(:,2),'b','linewidth',2.5)
plot(q(:,4),'r','linewidth',2.5)
plot(ref(1,1:max(size(q))),'--b','linewidth',2.5)
plot(ref(2,1:max(size(q))),'--r','linewidth',2.5)
title('joint position');
legend('q1','q2','qref1','qref2');

%% control

figure()
hold on; grid on;
plot(control_DAR(:,1),'b');
plot(control_DAR(:,2),'r');
title("control DAR");


