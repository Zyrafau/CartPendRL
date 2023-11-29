% Script for deployment of RL-based control on cart-pendulum hardware
%
% MATLAB R2015a

% Author:  Rafal Gembalczyk
% Company: AGH UST
% Date:    2023-08-15

%% Setup
clear, clc
cd E:\MATLAB_WORK\CartPend
if (~strcmp(version('-release'), '2015a'))
    error('Error: Incompatible MATLAB version')
end
load CartPendParams
load policyLearnables\robust
open CartPendReal

% LQR
div = Jp - m*l^2;
A = [0, 0, 1, 0;
    0, 0, 0, 1;
    0, m*g*l^2/div, Jp*(p2 - fc)/(m*div), -fp*l/div;
    0, m*g*l/div, l*(p2 - fc)/div, -fp/div];
B = [0; 0; Jp*p1/(m*div); p1*l/div];
C = eye(4);
D = zeros(4, 1);

stable = eig(A) < 0
controllable = rank(ctrb(A, B)) == length(A)
observable = rank(obsv(A, C)) == length(A)

Q = diag([500 1 10 1]); % state cost
R = 50; % control cost

[K, ~, e] = lqr(A, B, Q, R)

stable_control = e < 0
e % eig(A - B*K)

%% Save
save(['ScopeData_', datestr(now, 'yyyy-mm-dd_HH-MM-SS'), '.mat'], 'ScopeData')

%% Cleanup
delete CartPendReal.rxw64 CartPendReal.slx.original
rmdir CartPendReal_sldrt_win64 s
rmdir slprj s
