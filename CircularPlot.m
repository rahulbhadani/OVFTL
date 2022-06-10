%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++%
%                   Author: Rahul Kumar Bhadani                           %
%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++%
[n, L, t, P, P_c] = micro_model_stability;
j = 1;
k = 1;
[M,N] = size(P);
Q = zeros(length(t),n);
Qtemp = zeros(length(t),3);

Q_c = zeros(length(t),n);
Qtemp_c = zeros(length(t),3);

%------- Polar Coordinate Calculation for Original Trajectory-----------
for i=1:n
    Qtemp = P(:,i:n:end);
    for j = 1:M
        for k = 1:3
            if(Qtemp(j,k) >= 0 && Qtemp(j,k) <=L)
                Q(j,i) = Qtemp(j,k);
            end
        end
    end
end

C = zeros(length(t),n,2);
for ts = 1:length(t)
    for i = 1:n
        C(ts,i,1)  = (L/(2*pi))*sin(((2*pi)/L)*Q(ts,i));
        C(ts,i,2)  = (L/(2*pi))*cos(((2*pi)/L)*Q(ts,i));
    end
end

%------- Polar Coordinate Calculation for Controlled Trajectory-----------
for i=1:n
    Qtemp_c = P_c(:,i:n:end);
    for j = 1:M
        for k = 1:3
            if(Qtemp_c(j,k) >= 0 && Qtemp_c(j,k) <=L)
                Q_c(j,i) = Qtemp_c(j,k);
            end
        end
    end
end

C_c = zeros(length(t),n,2);
for ts = 1:length(t)
    for i = 1:n
        C_c(ts,i,1)  = (L/(2*pi))*sin(((2*pi)/L)*Q_c(ts,i));
        C_c(ts,i,2)  = (L/(2*pi))*cos(((2*pi)/L)*Q_c(ts,i));
    end
end
%---------------------- Patch Data ---------------------------
vertX = [-0.5 -0.5 +0.5 +0.5 +0.0 -0.5 +0.5];
vertY = [-1.0 +1.0 +1.0 -1.0 -0.25 -1.0 -1.0];

vertAX = [+0.50 +0.65 +0.65 +0.50];
vertAY = [+0.95 +0.95 +0.40 +0.40];

vertBX = [+0.50 +0.65 +0.65 +0.50];
vertBY = [-0.95 -0.95 -0.40 -0.40];

vertCX = [-0.50 -0.65 -0.65 -0.50];
vertCY = [-0.95 -0.95 -0.40 -0.40];

vertDX = [-0.50 -0.65 -0.65 -0.50];
vertDY = [+0.95 +0.95 +0.40 +0.40];

%------- Plot for Original Trajectory Movement-----------

figure(2);

for ts=1:1:length(t)
    X = C(ts,:,1);
    Y = C(ts,:,2);
    plotCustom(X, Y, vertX, vertY, vertAX, vertAY, vertBX, vertBY, vertCX, vertCY, vertDX, vertDY, 0.25);
    str = sprintf('Vehicle trajectories of original system | n = %d, L = %d',n,L);
    title(str);
     
    drawnow
    pause(0.1);
end

%------- Plot for Controlled Trajectory Movement-----------
figure(3);

for ts=1:1:length(t)
    X = C_c(ts,:,1);
    Y = C_c(ts,:,2);
    plotCustom(X, Y, vertX, vertY, vertAX, vertAY, vertBX, vertBY, vertCX, vertCY, vertDX, vertDY, 0.25);
    str = sprintf('Vehicle trajectories of feedback-controlled system | n = %d, L = %d',n,L);
    title(str);
     
    drawnow
    pause(0.1);
end