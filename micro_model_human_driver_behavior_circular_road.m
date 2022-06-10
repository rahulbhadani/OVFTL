function micro_model_human_driver_behavior_circular_road
%   Considers the Aw-Klar-Materne-Rascle model, which is a
%   combination of the follow-the-leader model and the
%   optimal-velocity model:
%      x_j'' = b*(x_{j+1}'-x_j')/(x_{j+1}-x_j)^2
%            + a*(V(x_{j+1}-x_j)-x_j')
%   The model is initialized with a uniform flow initial state.
%   Over the course of the evolution, random perturbations are added
%   to the vehicle velocities in periodic intervals. The trajectories
%   of all vehicles are plotted. The parameters of the model (and the
%   noise) are found so that the results reproduce the characteristic
%   behavior of a true traffic experiment (2008).
%   All units are standard SI units.
%
%   (C) 2015/06/24 by Benjamin Seibold and Hannah Pohlmann

%------------------------------------------------------------------------
% Parameters defining geometry (in space and time)
%------------------------------------------------------------------------
L = 230; % length of circular road [m]
l = 4.5; % length of each vehicle [m]
n = 22; % number of vehicles
tf = 200; % final time of trajectory computation [s]
dt_plot = .5; % size of plotting steps [s]
%------------------------------------------------------------------------
% Parameters defining the model
%------------------------------------------------------------------------
Vm = 35/3.6; % maximum velocity for optimal velocity function [m/s]
d0 = 2.23; % reference vehicle distance [m]
V = @(d) (tanh(d/d0-2)+tanh(2))/(1+tanh(2))*Vm; % optimal vel. fct. [m/s]
% Note: Vm, d0 chosen s.t. initial velocity at equilibirum point.
a = .5; % optimal velocity strength [1/s]
b = 20; % follow-the-leader strength [m^nu/s]
nu = 2; % power of distance in denominator of follow-the-leader term
dt_noise = 2; % time between two applications of noise [s]
sigma = .25; % total amount of noise per second [m/s^2]

%------------------------------------------------------------------------
% Derived parameters and functions
%------------------------------------------------------------------------
d = (L/n)-l; % initial distance between adjacent vehicles
p = 0:(d+l):(d+l)*(n-1); % initial positions of vehicles
v = p*0+V(d); % initial velocities of vehicles
dV = @(d) (V(d+1e-7)-V(d-1e-7))/2e-7; % slope of opt. vel. fct. at equil.
fprintf('Microscopic model (AKMR) on circular road of length %gm.\n',L)
fprintf('%d vehicles; space between vehicles: %0.3gm\n',n,d)
fprintf(['max. vel.: %0.3gm/s; equil. vel.: V(d) = %0.3gm/s; ',...
    'V''(d) = %0.3g/s\n'],Vm,V(d),dV(d))
dist = @(x) [x(3:2:end);x(1)+L]-x(1:2:end)-l; % distance between vehicles
R = @(~,x) reshape([x(2:2:end),...                     % ODE system
    b*(x([4:2:end,2])-x(2:2:end))./((dist(x)).^nu)+... % right hand
    a*(V(dist(x))-x(2:2:end))]',[],1);                 % side function

%------------------------------------------------------------------------
% Computation of model
%------------------------------------------------------------------------
ns = ceil(tf/dt_noise); % number of intervals (between them apply noise)
sigma_dt = sqrt(dt_noise)*sigma; % amount of noise per interval
x0 = reshape([p;v],[],1)'; % initial state vector
t = 0; % initialize time vector
Y = x0; % initialize solution matrix
for j = 1:ns % loop over intervals
    t_start = (j-1)*dt_noise; % start time of this interval
    t_end = min(j*dt_noise,tf); % end time of this interval
    dt_j = t_end-t_start; % length of this interval
    t_j = linspace(t_start,t_end,ceil(dt_j/dt_plot)+1);% int. time vector
    x0 = Y(end,:);% new initial condition is final state from sol. matrix
    x0(2:2:end) = x0(2:2:end)+randntrunc(1,n,3)*sigma_dt; % add noise
    x0(2:2:end) = max(x0(2:2:end),0); % cap new velocity from below by 0
    [~,Yj] = ode45(R,t_j,x0); % compute vehicle trajectories in interval
    if length(t_j)==2 % if time interval t_j has only 2 entries,
       Yj = Yj([1 end],:); % use only first and last solution values
    end
    t = [t,t_j(2:end)]; % append time vector
    Y = [Y; Yj(2:end,:)]; % append solution matrix with new interval data
end
P = mod([Y(:,1:2:end),Y(:,1:2:end)+L,Y(:,1:2:end)+2*L],3*L)-L;   % create
P(P<-.5*L|P>1.5*L) = nan;        % copies of trajectories above and below

%------------------------------------------------------------------------
% Output important values
%------------------------------------------------------------------------
veh_pos = Y(:,1:2:end); % positions of vehicles
veh_dist = [veh_pos(:,2:end),veh_pos(:,1)+L]-veh_pos-l;
min_dist = min(veh_dist,[],2); % minimum distance over time
fprintf('Minimum spacing between vehicles: %0.2fm\n',min(min_dist))
vel = Y(:,2:2:end); % velocities of vehicles
min_vel = min(vel,[],2); % minimum velocity over time
max_vel = max(vel,[],2);% maximum velocity over time
fprintf('Minimum|maximum vehicle velocities: ')
fprintf('%0.2fm/s | %0.2fm/s\n', min(min_vel),max(max_vel));

%------------------------------------------------------------------------
% Plot vehicle trajectories
%------------------------------------------------------------------------
clf
plot(P,t,'b-',P(:,1:n:end),t,'r-') % plot first vehicle in red
axis([0 L 0 tf])
ylabel('time t [s]'), xlabel('position on road x [m]')
title('Vehicle trajectories')

%========================================================================
% Functions
%========================================================================

function R = randntrunc(m,n,c)
%RANDNTRUNC Truncated normally distributed pseudorandom numbers.
%   R = RANDNTRUNC(M,N,C) returns an M-by-N matrix containing
%   pseudorandom values drawn from the standard normal
%   distribution, with samples larger (in magnitude) than C
%   discarded.
if nargin<3, c = inf; end
R = randn(m,n); % array of random samples
while 1 % loop until stopping criterion
    ind = abs(R)>c; % indices of unacceptable samples
    if ~nnz(ind), break, end % if everything acceptable, break
    R(ind) = randn(nnz(ind),1); % resample unacceptable entries
end
