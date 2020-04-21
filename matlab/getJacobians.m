%% use calculateError to get and compare error
K = [525,0,320; 0,525,240; 0,0,1];
quadricVec = [0,0,0, 0,0,0, 1.0,1.0,1.0];
poseVec = [0,0,0, 0,0,-3];
measurement = [0,0,0,0];

% prediction = calculateError(poseVec, quadricVec, K, measurement)

% calculate the derivative wrt q,x
x = sym('x', [1,6]);
q = sym('q', [1,9]);
[C_, t1, t2] = calculateError(x, q, K, measurement, false);
dC_dx = double(subs(subs(jacobian(C_(:), x(:)), x, poseVec), q, quadricVec));
dC_dq = double(subs(subs(jacobian(C_(:), q(:)), x, poseVec), q, quadricVec));

% test analytical jacobians
disp('testing analytical')
[t3, dC_dx_a, dC_dq_a] = calculateError(poseVec, quadricVec, K, measurement, true);

disp('dC_dx == dC_dx_a'); disp(dC_dx==dC_dx_a)
disp('dC_dq == dC_dq_a'); disp(dC_dq==dC_dq_a)
% disp('dC_dx_a'); disp(dC_dx_a)
% disp('dC_dq_a'); disp(dC_dq_a)








