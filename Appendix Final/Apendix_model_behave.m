% To test the supply boat model input & output behavior 
% test model behaviour 
clear all;
dt = 0.01; %sampling time [s]
t0 = 0; %starting time
t1 = 100; %ending time [s]
T = (0:dt:t1); %time vector
N = length(T); % no. samples

X=zeros(N,6); Y=zeros(N,3); U=zeros(N,3);  % storing matrices
U1=zeros(3);
Y1=zeros(N);
u=0; 

for i=1:3
    x = [ 10 10 30 10 10 10]';
    if (i==1)
        tau= [0,0,0]';    %input matrix
    elseif (i==2)
        u=10^7;
        tau = [u,0,0]';
    else 
        u=10^7;
        tau = [0,u,0]';
    end

for k = 1:N
    tk = T(k);
    y=x(1);
    y1=(2);
    Y(k,i)=x(1);% store data 
    Y1(k,i)=x(2);% store data
    U1(i)=u;
    U(k,i)=u; 
    [Time,sol] = ode45(@supply1,[tk tk+dt],x',[],tau); % discretise the model
    x = sol(end,:)';  % update model with last element
end 
end
 
figure(1)
subplot(311),plot(1:N,Y(:,1),'r-',1:N,Y1(:,1),'b--',1:N,U(:,1),'k--'),title('Plot1: Position in x&y - direction [m]- no control force','Interpreter','latex'),ylabel('$x/y$','Interpreter','latex'),grid,t=legend('position in x-direction','position in y-direction','control force');set(t,'Interpreter','latex'),ylim([-100,200]),xlim([1,N])
subplot(312),plot(1:N,Y(:,2),'r-',1:N,Y1(:,2),'b--',1:N,U(:,2),'k--'),title('Plot2: Position in x&y - direction [m]- control force in x-direction','Interpreter','latex'),ylabel('$x/y$','Interpreter','latex'),grid,t=legend('position in x-direction','position in y-direction','control force');set(t,'Interpreter','latex'),ylim([-800,3000]),xlim([1,N])
subplot(313),plot(1:N,Y(:,3),'r-',1:N,Y1(:,3),'b--',1:N,U(:,3),'k--'),title('Plot3: Position in x&y - direction [m]- control force in y-direction','Interpreter','latex'),ylabel('$x/y$','Interpreter','latex'),grid,t=legend('position in x-direction','position in y-direction','control force');set(t,'Interpreter','latex'),ylim([-500,600]),xlim([1,N])
