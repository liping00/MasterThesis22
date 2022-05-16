% Design the supply boat DP control system based on the SSM model
clear all;
addpath 'C:\Users\wangl\Desktop\MASTER THESIS\MATLAB EXAMPLE\matlab_files_4pidtun\d-sr';  % add path to DSR toolbox & prbs1.m function

randn('seed',1); 
rand('seed',1);
dt = 0.01; %sampling time [s]
t0 = 0; %starting time
t1 = 25; %ending time [s]
T = (0:dt:t1); %time vector
N = length(T); % no. samples

Uexp=10^8*prbs1(N,1,60);  % Input experiment PRBS signal

dv=0; dw=0;   %dv=0.001; dw=0.001
X=zeros(N,6); Y1=zeros(N,1); U1=zeros(N,1);  % storing matrices
Y2=zeros(N,1); U2=zeros(N,1);
R=zeros(N,2);
v=randn(N,6)*dv; w=randn(N,1)*dw;  %simulated noise
 
L=5;m=2;  % settings for pidtun tuning
randn('seed',1); 
rand('seed',1);
%v=randn(N,5)*dv; w=randn(N,1)*dw;  %No noise, gives perfekt ID model
vu=0;
bprbs=1;y1_old=0; y2_old=0;
r1=0;r2=0; u1=0; u2=0;
x1 = [ 0 0 30 0 0 0]';
x2 = [ 0 0 30 0 0 0]';
z1=0;z2=0;
for k = 1:N
    tk = T(k);
    y1=x1(1)+w(k); 
    y2=x2(2)+w(k); 
	if(bprbs==0)
        if k<1000
            u1=0;
            u2=0;
        elseif(k==1000 || k==1700)       % Bump less transfer 
            e1=r1-y1;
            e2=r2-y2;
            z1=u1-Kp1*e1+KpTdh1*(y1-y1_old);
            u1=Kp1*e1+z1-KpTdh1*(y1-y1_old);
            z2=u2-Kp2*e2+KpTdh2*(y2-y2_old);
            u2=Kp2*e2+z2-KpTdh2*(y2-y2_old);
        else
            e1=r1-y1;
            e2=r2-y2;
            u1=Kp1*e1+z1-KpTdh1*(y1-y1_old);
            u2=Kp2*e2+z2-KpTdh2*(y2-y2_old);
            z1=z1+KpTih1*e1; 
            z2=z2+KpTih2*e2; 
        end
        
	elseif(bprbs==1)
        if k<400
            u1=0;
            u2=0;
        elseif k<900
            u1=Uexp(k); 
            u2=Uexp(k); 
        else 
           Yid1=Y1(400:end,1); Uid1=U1(400:end,1);
           [A1,B1,C1,D1,CF,F,x0]=dsr(Yid1,Uid1,L);  % system identification 
           [Kp1,Ti1,Td1]=pidtun(Yid1,Uid1,dt,m);    % tune controller parameters 
           KpTdh1=Kp1*Td1/dt;
           KpTih1=Kp1*dt/Ti1; 
           Yid2=Y2(400:end,1); Uid2=U2(400:end,1);
           [A2,B2,C2,D2,CF,F,x0]=dsr(Yid2,Uid2,L);
           [Kp2,Ti2,Td2]=pidtun(Yid2,Uid2,dt,m);
           KpTdh2=Kp2*Td2/dt;
           KpTih2=Kp2*dt/Ti2; 
           bprbs=0; 
        end
    end
    y1_old=y1;  % store last value
    u1_old=u1;
    y2_old=y2; 
    u2_old=u2;
    
   if k>1700
        r1=1;  
        r2=0.5;
   end
    Y1(k,1)=y1; U1(k,1)=u1; R(k,1)=r1; R(k,2)=r2; % store data 
    Y2(k,1)=y2; U2(k,1)=u2; 
    tau1 = [u1,0,0]';
    tau2 = [0,u2,0]';
    [Time,sol] = ode45(@supply1,[tk tk+dt],x1',[],tau1); % discretise the model
    x1 = sol(end,:)'+v(k)';  % update model with last element
    [Time,sol] = ode45(@supply1,[tk tk+dt],x2',[],tau2);
    x2 = sol(end,:)'+v(k)';    

end
fprintf('The value of Kp1 is %10.3f.\n',Kp1)
fprintf('The value of Ti1 is %10.3f.\n',Ti1)
fprintf('The value of Td1 is %10.3f.\n',Td1)
fprintf('The value of Kp2 is %10.3f.\n',Kp2)
fprintf('The value of Ti2 is %10.3f.\n',Ti2)
fprintf('The value of Td2 is %10.3f.\n',Td2)
figure(2)
subplot(211),plot(1:N,Y1,1:N,Y2,'r-',1:N,R(:,1),1:N,R(:,2),'k--'),title('$y$: position [m]','Interpreter','latex'),ylabel('$y$','Interpreter','latex'),grid,t=legend('position in x direction','position in y direction','reference in x direction','reference in y direction');set(t,'Interpreter','latex'),xlim([1,N])
%subplot(212),plot(1:N,U1,1:N,U2,'b--'),title('$u$: control force','Interpreter','latex'),grid,ylabel('$u$','Interpreter','latex'),xlabel('Time, [samples]','Interpreter','latex'),xlim([1,N])
subplot(212),plot(1:N,U1,'b--',1:N,U2,'r--'),title('$u$: control force [N]','Interpreter','latex'),grid,ylabel('$u$','Interpreter','latex'),xlabel('Time, [samples]','Interpreter','latex'),t=legend('x direction','y direction');set(t,'Interpreter','latex'),xlim([1,N])
