clc
clear
close all

q = load("Q.txt");

dq_hat = load("dQ_hat.txt");
dq = load("dQ.txt");

eta = load("eta.txt");

q_ref = load("Qref.txt");

error = load("error.txt");

dim = max(size(q));
time = zeros(1,dim);
index = 0.0;

for i=1:dim
    time(i) = index;
    index = index + 0.005;
end

%% reference

t = 0:0.0001:500;

%ref = [q(1,2) + 0.1*(1-cos(t));q(1,4) + 0.1*(1-cos(t))];
ref = [0.4*cos(t);-0.4*cos(t)];

a0 = 0.2359;
a1 = -0.4484;
b1 = -0.1237;
a2 = 0.0815;
b2 = -0.0143; 
f = 5; 

w1 = a0 + a1*cos(t*f) + b1*sin(t*f) + a2*cos(2*t*f) + b2*sin(2*t*f);

a0 = 1.0989;
a1 = 0.3253;
b1 = -0.3256;
a2 = -0.0000;
b2 = 0.0291;
f = 5;
                   
w2 = a0 + a1*cos(t*f) + b1*sin(t*f) + a2*cos(2*t*f) + b2*sin(2*t*f);

%ref = [w1;w2];

x_circ = 0.5*cos(q(:,2))+0.5*cos(q(:,2)+q(:,4));
y_circ = 0.5*sin(q(:,2))+0.5*sin(q(:,2)+q(:,4));

%% Joint position

figure()
hold on; grid on;
plot(time,q(:,2),'b','linewidth',2)
plot(time,q(:,4),'r','linewidth',2)
plot(time,ref(1,1:dim),'b--','linewidth',2)
plot(time,ref(2,1:dim),'r--','linewidth',2)
title('joint position');
legend('q1','q2','qref1','qref2');

%% EE position

figure()
hold on; grid on;
plot(x_circ,y_circ,'linewidth',2)
axis equal
title('end effector trajectory')

%% error

figure()
hold on; grid on;
plot(time(1,1:end-1),error,'linewidth',2);

%% joint speed

figure()
hold on; grid on;
plot(time,dq_hat(:,2),'linewidth',2);
plot(time,dq_hat(:,4),'linewidth',2);
plot(time,dq(:,2),'--','linewidth',2);
plot(time,dq(:,4),'--','linewidth',2);
title('joint speed (estimated)');
legend('dq1','dq2');

%% control

 K = [-9277.13944860092 3710.17708672289 -349.234832428913 266.395117894730 -194809.308523000 29590.8387072452 -497382.571831891 83024.4056398725 -512598.396322723 96517.0532040094 -268819.441373604 59336.0150773882 -73589.1199037631 20303.8593336261;
 -2029.21453824429 -7028.36935443040 -80.0713649245077 -523.663369755656 -42740.8155134585 -58507.6257749021 -108901.648552769 -163109.287869734 -111981.948671448 -188381.316026516 -58595.8106911847 -115090.430611396 -16021.4507995792 -39186.9823159717];

%K = [-51479.1506564270 3568.82244932025 -11854.7020446130 1079.87266377154 -633338.094409722 4333.26737636959 508505.390501465 -47862.5287158016 -98358.8518500649 6135.56008409700 ;
%1263.68859842101 -44137.3005591795 480.698288000380 -10896.5985110252 -9654.94397920896 -440771.759801160 -20520.6564484375 470082.073219219 1813.31207683121 -82035.0166304382 ];

for i = 1:max(size(eta))
    control(:,i) = K * [q(i,2);q(i,4);dq(i,2);dq(i,4);eta(i,:)'];

    %if ( abs(control(1,i)) >= 15000 )
	%control(1,i) = sign(control(1,i))*15000;
    %end

    %if ( abs(control(2,i)) >= 15000 )
	%control(2,i) = sign(control(2,i))*15000;
    %end
end

figure()
hold on; grid on;
plot(time(1,1:end-1),control','linewidth',2)
title('control')

%% internal model

figure()
hold on; grid on;
plot(time(1,1:end-1),eta,'linewidth',2)
title('internal model')





