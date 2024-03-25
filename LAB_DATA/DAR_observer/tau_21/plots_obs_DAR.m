clc
clear
close all

q = load("Q.txt");

dq_hat = load("dQ_hat.txt");
q_hat = load("Q_hat.txt");

dq = load("dQ.txt");

error = load("error.txt");

state_hat = load("qhat_DAR.txt");

control_DAR = load("control_DAR.txt");

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
plot(state_hat(:,1),'--b','linewidth',2.5)
plot(state_hat(:,2),'--r','linewidth',2.5)
plot(q_hat(:,2),'b-.','linewidth',2.5)
plot(q_hat(:,4),'r-.','linewidth',2.5)
title('joint position');
legend('q1','q2','q1 DAR','q2 DAR','q1 full','q2 full');

%% joint speed

figure()
hold on; grid on;
plot(dq_hat(:,2),'b-.','linewidth',2.5);
plot(dq_hat(:,4),'r-.','linewidth',2.5);
plot(state_hat(:,3),'b--','linewidth',2.5);
plot(state_hat(:,4),'r--','linewidth',2.5);
plot(dq(:,2),'b','linewidth',2.5);
plot(dq(:,4),'r','linewidth',2.5);
title('joint speed (estimated)');
legend('dq1 their','dq2 their','dq1 ours','dq2 ours','dq1 actual','dq2 actual');

figure()
hold on; grid on;
plot(state_hat(:,3),'--');
plot(state_hat(:,4),'--');
title("DAR velocity")

figure()
hold on; grid on;
plot(control_DAR(:,1),'b');
plot(control_DAR(:,2),'r');
title("control DAR");




