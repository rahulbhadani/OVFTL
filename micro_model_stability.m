function[n, L, t, P, P_c] = micro_model_stability
%MICRO_MODEL_STABILITY
%   Considers the Aw-Klar-Materne-Rascle model, which is a
%   combination of the follow-the-leader model and the
%   optimal-velocity model:
%      x_j'' = b*(x_{j+1}'-x_j')/(x_{j+1}-x_j)
%            + a*(V(x_{j+1}-x_j)-x_j')
%   The model is initialized with a uniform initial state, plus
%   a small perturbation. Its linearization around this uniform
%   state is found, and the eigenvalues computed. Then, a
%   linear feedback control is applied to the velocity of the
%   first vehicle. The shifted poles of the linearized system
%   are shown, as well as the trajectories of all vehicles for
%   the original system and the feedback-controlled system.
%
%   (C) 2015/04/10 by Benjamin Seibold

%------------------------------------------------------------------------
% Model parameters
%------------------------------------------------------------------------
L = 50; % length of circular road
n = 22; % number of vehicles
V = @(d) tanh(d-2)+tanh(2); % optimal velocity function
b = 0.7; % follow-the-leader strength
a = .3; % optimal velocity strength

tf = 400; % final time of trajectory computation
B = (1:2*n)'==2; % control matrix (can affect only velocity of vehicle 1)
F = B'*0; F(2) = F(2)-5; % feedback matrix

%------------------------------------------------------------------------
% Derived parameters and functions

%------------------------------------------------------------------------
d = L/n; % initial distance of adjacent vehicles
dV = @(d) (V(d+1e-7)-V(d-1e-7))/2e-7;
fprintf('Microscopic model (AKMR) on circular road of length %g.\n',L)
fprintf('%d vehicles; vehicle spacing: %0.3g\n',n,d)
fprintf('opt. vel.: V(d) = %0.3g; V''(d) = %0.3g\n',V(d),dV(d))
dist = @(x) [x(3:2:end);x(1)+L]-x(1:2:end); % distance between vehicles
% ODE system right hand side function
R = @(~,x) reshape([x(2:2:end),b*(x([4:2:end,2])-x(2:2:end))./dist(x)...
    + a*(V(dist(x))-x(2:2:end))]',[],1);
%R_c = @(~,x) R(0,x)+B*(F*x); % feedback-controlled right hand side
R_c = @(~,x) R(0,x)+B*(F*(x-V(d))); % feedback-controlled right hand side

%------------------------------------------------------------------------
% Set up linearized system matrices and calculate eigenmodes
%------------------------------------------------------------------------
cb = b/d; cc = a*dV(d); ca = a+cb; % model parameters
av = (1+(-1).^(1:2*n))/2; % alternating vector
A = diag(-ca*av)+diag(-cc*av(2:end),-1)+...      % linearlized
    diag(cc+(1-cc)*av(2:end),1)+diag(cb*av(3:end),2); % system
A(end,1:2) = [cc cb];                      % matrix (circular)
%[EV,D] = eig(A); lambda = diag(D);
lambda = eig(A); % eiegenvalues of linearized system
ind_unstable = find(real(lambda)>1e-14); % indices of unstable modes
fprintf('number of unstable modes: %d\n',numel(ind_unstable))
A_c = A+B*F; % feedback-controlled matrix of linearized system
lambda_c = eig(A_c); % eigenvalues of feedback-controlled matrix
fprintf('feedback-controlled system: unstable modes: %d\n',...
    sum(real(lambda_c)>1e-14))

%------------------------------------------------------------------------
% Run computation of microscopic model (original; not linearized)
%------------------------------------------------------------------------
p = 0:d:d*(n-1); % initial positions of vehicles
v = p*0+V(d); % initial velocities of vehicles
v = v+(rand(size(v))*2-1)*d*.1; % add noise to initial velocities
x0 = reshape([p;v],[],1); % initial state vector, containing inital velocities and positions of the vehicles
t = linspace(0,tf,500); % time vector
[~,Y] = ode45(R,t,x0); % compute vehicle trajectories
[~,Y_c] = ode45(R_c,t,x0); % compute vehicle trajectories for feedback control
P = mod([Y(:,1:2:end),Y(:,1:2:end)+L,Y(:,1:2:end)+2*L],3*L)-L;
P(P<-.5*L|P>1.5*L) = nan;
P_c = mod([Y_c(:,1:2:end),Y_c(:,1:2:end)+L,Y_c(:,1:2:end)+2*L],3*L)-L;
P_c(P_c<-.5*L|P_c>1.5*L) = nan;

%------------------------------------------------------------------------
% Plot results
%------------------------------------------------------------------------
clf
subplot(2,2,1)
plot(t,P,'b-',t,P(:,1:n:end),'r-')
% axis([0 tf 0 L])
xlabel('time t'), ylabel('position on road x')
title('vehicle trajectories of original system')

subplot(2,2,2)
plot(t,P_c,'b-',t,P_c(:,1:n:end),'r-')
axis([0 tf 0 L])
xlabel('time t'), ylabel('position on road x')
title('vehicle trajectories of feedback-controlled system')

subplot(2,2,3)
p = sort(P(end,:)); p = p(~isnan(p)); % positions at final time
p_mid = (p(1:end-1)+p(2:end))/2; % mid points between vehicles
rho = 1./diff(p); % vehicle density
p_c = sort(P_c(end,:)); % controlled system
veh1 = p_c(1:n:end); veh1 = veh1(0<=veh1&veh1<L); % pos. of vehicle 1
p_c = p_c(~isnan(p_c)); % controlled system: positions at final time
p_mid_c = (p_c(1:end-1)+p_c(2:end))/2; % mid points between vehicles
rho_c = 1./diff(p_c); % vehicle density
ax = [0 L 0 max([rho rho_c])*1.05];
plot(p_mid,rho,'k-',p_mid_c,rho_c,'r-',veh1*[1 1],ax(3:4),'r:')
axis(ax)
legend('original system','controlled system')
xlabel('position on road x'), ylabel('vehicle density \rho')
title(sprintf('density at final time (t=%g)',tf))

subplot(2,2,4)
lar = real(lambda); lai = imag(lambda);
ax = [min(lar),max(lar),min(lai),max(lai)];
ax = ax+[[-1 1]*(ax(2)-ax(1)),[-1 1]*(ax(4)-ax(3))]*.05;
plot([0 0],ax(3:4),'k-',...
    lar,lai,'b.',lar(ind_unstable),lai(ind_unstable),'bo',...
    real(lambda_c),imag(lambda_c),'r.')
axis(ax)
xlabel('real part'), ylabel('imaginary part')
title('eigenvalues of linearized system in complex plane')