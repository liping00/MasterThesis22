% Input experiment design of the supply boat model 
addpath 'C:\Users\wangl\Desktop\MASTER THESIS\MATLAB EXAMPLE\matlab_files_4pidtun' % add the path of the prbs1.m function 
clear all;
randn('seed',1); 
rand('seed',1);
dt = 0.01; %sampling time [s]
t0 = 0; %starting time
t1 = 20; %ending time [s]
T = (0:dt:t1); %time vector
N = length(T); % no. samples

Uexp=10^8*prbs1(N,1,60);  % PRBS signal generation 

dv=0; dw=0; %dv=0.001; dw=0.001;

X=zeros(N,6); Y=zeros(N); U=zeros(N);  % storing matrices
R=zeros(N,1);
v=randn(N,6)*dv; w=randn(N,1)*dw;  %No noise, gives perfekt ID model
randn('seed',1); 
rand('seed',1);v=randn(N,5)*dv; w=randn(N,1)*dw;  %No noise, gives perfekt ID model
vu=0;
x = [ 10 10 30 10 10 10]'; %x = [ x y psi u v r]
z=0;
for k = 1:N
    u=Uexp(k);
    tk = T(k);
    y=x(2)+w(k);
    Y(k)=y; % store data 
    U(k)=u; 
    tau = [0,u,0]';
    [Time,sol] = ode45(@supply1,[tk tk+dt],x',[],tau); % discretise the model
    x = sol(end,:)'+v(k)';  % update model with last element
end 

figure(1)
plot(1:N,Y,1:N,10^-8*U,'--'),title('Input experiment design'),ylabel('$position in y-direction$','Interpreter','latex'),legend('position in y-direction','control signal'),ylim([-40,40]),xlim([1,N])