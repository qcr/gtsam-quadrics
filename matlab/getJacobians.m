%% use calculateError to get and compare error
clear all; clc

K = [525,0,320; 0,525,240; 0,0,1];
quadricVec = [1.1,2.2,3.3, 4.4,5.5,6.6, 7.7,8.8,9.9];
poseVec = [1.1,2.2,3.3, 3.3,-5.5,-10];
measurement = [0,0,0,0];

% prediction = calculateError(poseVec, quadricVec, K, measurement)

% calculate the derivative wrt q,x
% x = sym('x', [1,6]);
% q = sym('q', [1,9]);
% [b_, t1, t2] = calculateError(x,q,K,measurement,false);
% db_dx = double(subs(subs(jacobian(b_, x), x, poseVec), q, quadricVec));
% db_dq = double(subs(subs(jacobian(b_, q), x, poseVec), q, quadricVec));
% [C_, t1, t2] = calculateError(x, q, K, measurement, false);
% dC_dx = double(subs(subs(jacobian(C_(:), x(:)), x, poseVec), q, quadricVec));
% dC_dq = double(subs(subs(jacobian(C_(:), q(:)), x, poseVec), q, quadricVec));

% test analytical jacobians
disp('testing analytical')
[t3, db_dx_a, db_dq_a] = calculateError(poseVec, quadricVec, K, measurement, true);
% [t3, dC_dx_a, dC_dq_a] = calculateError(poseVec, quadricVec, K, measurement, true);

% disp('db_dx magic subbed')
% disp(db_dx)
% disp('db_dq magic subbed')
% disp(db_dq)
disp('db_dx analytical')
disp(db_dx_a)
disp('db_dq analytical')
disp(db_dq_a)

% disp('dC_dx == dC_dx_a'); disp(dC_dx==dC_dx_a)
% disp('dC_dq == dC_dq_a'); disp(dC_dq==dC_dq_a)









