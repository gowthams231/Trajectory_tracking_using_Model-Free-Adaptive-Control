clear; clc;
% step factors initialization
rho = 0.6;
eta = 1;

% input difference
epsilon = 10^(-5);

% weighing factors initialization
lambda = 2;
mu = 1;

% input initialization
u = zeros(1000,1);

% parameter initialization
phi = zeros(1000,1);
phi(1,1) = 2;
phi(2,1) = 2;

% System Dynamics initialization
y = zeros(1000,1);
y(1,1) = -1;
y(2,1) = 1;

% Trajectory initilization
%yt = 30*ones(1001,1);

for k=1:1000
    
    % Trajectory
    if (k<=300)
        yt(k+1,1) = 0.5 * (-1)^(round(k/500));
    elseif (k>300 && k<=700)
        yt(k+1,1) = 0.5*sin(k*pi/100) + 0.3*cos(k*pi/50);
    else
        yt(k+1,1) = 0.5 * (-1)^(round(k/500));
    end
      
    
    %Parameter Estimate
    if (k>=3)
        phi(k,1) = phi(k-1,1) + ((eta*(u(k-1,1)-u(k-2,1)))/(mu + (u(k-1,1)-u(k-2,1))^2))* (y(k,1)-y(k-1,1) - phi(k-1,1)*(u(k-1,1)-u(k-2,1)));
        
        if(abs(phi(k,1))<=epsilon || abs(u(k-1,1)-u(k-2,1))<=epsilon || sign(phi(k,1))~=sign(phi(1,1)))
            phi(k,1) = phi(1,1);
        end
        
    end
   
    % Input Calculation
    if (k>=3)
        u(k,1) = u(k-1,1) + (rho*phi(k,1)/(lambda+norm(phi(k,1))^2))*(yt(k+1,1)-y(k,1));
    end
    
    % System Dynamics
    if (k>=2 && k<=500)
        y(k+1,1) = (y(k,1)/(1+(y(k,1))^2))+(u(k,1)^3);
    elseif (k>500)
        y(k+1,1) = (y(k,1)*y(k-1,1)*y(k-2,1)*u(k-1,1)*(y(k-2,1)-1)+round(k/500)*u(k,1))/(1+(y(k-1,1)^2)+(y(k-2,1)^2));
    end
    
end
plot(yt)
hold on
plot(y)
title('SISO CFDL MFAC Trajectory Tracking')
legend('trajectory','tracking')