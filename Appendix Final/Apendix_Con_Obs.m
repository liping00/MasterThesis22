% To check the controllability and observability of the supply boat model 
psi= pi/6; % a random selected yaw angle to calculate a matrix A  value 
A= [0 0 0 cos(psi) -sin(psi) 0 % matrix A in SSM model 
    0 0 0 sin(psi) cos(psi) 0
    0 0 0 0 0 1.0000
    0 0 0 -0.0114 0 0
    0 0 0 0 -0.0225 -0.0818
    0 0 0 0 -0.0000 -0.0871];
B=10^6*[0 0 0    % matrix B in SSM model 
    0 0 0
    0 0 0
    0.1478 0 0
    0 0.0902 0.0007
    0 0.0007 0.0002];
D=[1 0 0 0 0 0   % matrix D in SSM model
    0 1 0 0 0 0
    0 0 1 0 0 0];    
Co = ctrb(A,B);   % compute the controllability matrix 
Ob = obsv(A,D);   % compute the observabilty matrix
unco = length(A)-rank(Co); % compute the number of uncontrollable states
unob=length(A)-rank(Ob);  % compute the number of unobservable states
fprintf('Number of uncontrollable state is %d.\n',unco)
fprintf('Number of unobservable state is %d.\n',unob)