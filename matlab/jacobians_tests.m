%% Calculate jacobian of P = K*I34*X^-1

clc; clear;

% A = sym('A', [3,4]);
% I = eye(3,4);
% B = sym('B', [4,4]);
% 
% C = A*B
% dC_dA = jacobian(C(:),A(:))
% dC_dB = jacobian(C(:),B(:))
% 
% A1 = [1,2,3,4; 5,6,7,8; 9,10,11,12];
% B1 = ones(4,4)*2;
% C1 = subs(subs(C, A,A1), B,B1)
% dC_dA1 = subs(dC_dA, B, B1)
% dC_dB1 = subs(dC_dB, A, A1)

% P = sym('P', [3,4]);
% Q = sym('Q', [4,4]);
% C = P*Q*P.'

%% full test

% constants
% K = [320,0,320; 0,320,240; 0,0,1];
% I3x4 = [1,0,0,0; 0,1,0,0; 0,0,1,0];

% build Q from qvec
% Qv = [sym('Q1'), sym('Q2'), sym('Q3'), sym('Q4'), sym('Q5'), sym('Q6'), sym('Q7'), sym('Q8'), sym('Q9')];
% Q_rotation = [(4 + Qv(1)^2 - Qv(2)^2 - Qv(3)^2), (2 * Qv(1) * Qv(2) - 4 * Qv(3)), (4 * Qv(2) + 2 * Qv(1) * Qv(3)); 
%                     (2 * Qv(1) * Qv(2) + 4 * Qv(3)), (4 - Qv(1)^2 + Qv(2)^2 - Qv(3)^2), (2 * (-2 * Qv(1) + Qv(2) * Qv(3))); 
%                     (2 * (-2 * Qv(2) + Qv(1) * Qv(3))), (4 * Qv(1) + 2 * Qv(2) * Qv(3)), (4 - Qv(1)^2 - Qv(2)^2 + Qv(3)^2)];
% Q_rotation = Q_rotation / (4 + Qv(1)^2 + Qv(2)^2 + Qv(3)^2);
% Q_translation = [Qv(4); Qv(5); Qv(6)];
% Q_pose = [Q_rotation, Q_translation; 0,0,0,1];
% 
% Q_shape = sym(zeros(4,4));
% Q_shape(1,1) = Qv(7)^2;
% Q_shape(2,2) = Qv(8)^2;
% Q_shape(3,3) = Qv(9)^2;
% Q_shape(4,4) = -1;
% Q = Q_pose * Q_shape * Q_pose.';

% build pose from pvec
% p = sym('p', [1,6]);
% CwrR = [(4 + p(1)^2 - p(2)^2 - p(3)^2), (2 * p(1) * p(2) - 4 * p(3)), (4 * p(2) + 2 * p(1) * p(3)); 
%         (2 * p(1) * p(2) + 4 * p(3)), (4 - p(1)^2 + p(2)^2 - p(3)^2), (2 * (-2 * p(1) + p(2) * p(3))); 
%         (2 * (-2 * p(2) + p(1) * p(3))), (4 * p(1) + 2 * p(2) * p(3)), (4 - p(1)^2 - p(2)^2 + p(3)^2)];
% CwrR = CwrR / (4 + p(1)^2 + p(2)^2 + p(3)^2);
% Cwrt = [p(4); p(5); p(6)];
% Pose = [CwrR, Cwrt; 0,0,0,1];

% define functions
% P = K * I3x4 * inv(Pose)


%% Check kron
A = [1,2;3,4;5,6]; % 3,2
B = [1,2,3,4;5,6,7,8]; % 2,4
C = [2,2;2,2;2,2;2,2]; % 4,2
Y = A*B*C
yvec = Y(:)
v1 = kron(C.', A) * B(:)





%% Check dP_dPose^-1 = kron(I4x4, K*I3x4)
K = [320,0,320; 0,320,240; 0,0,1];
I3x4 = [1,0,0,0; 0,1,0,0; 0,0,1,0];
poseinv = sym('pi', [4,4]);
P = K*I3x4*poseinv;

% calculate jacobian
dP_dposeinv = jacobian(P(:), poseinv(:));

% confirm correct 
poseinv1 = [1,2,3,4;5,6,7,8;0,1,2,3;2,4,6,8];
dP_dposeinv_test = double(subs(dP_dposeinv, poseinv, poseinv1))

dP_dposeinv_analytical = kron(eye(4), K*I3x4)



%% Check dC_dQ = kron(P,P)
P = sym('P', [3,4]);
Q = sym('Q', [4,4]);
C = P*Q*P.';

% calculate jacobian 
dC_dQ = jacobian(C(:),Q(:))

% confirm correct
P1 = [1,2,3,4;5,6,7,8;9,10,11,12];
Q1 = [2,4,6,8;1,2,3,4;3,6,9,12;1,0,1,0];
dC_dQ_ = subs(dC_dQ, P, P1);
dC_dQ_test = double(subs(dC_dQ_, Q, Q1))

dC_dQ_analytical = kron(P1,P1)


%% Check dC_dP = kron(I3, PQ) *tvec(3,4) + kron(PQ.', I3)
% declare variables
P = sym('P', [3,4]);
Q = sym('Q', [4,4]);
C = P*Q*P.';

% calculate jacobian
dC_dP = jacobian(C(:), P(:))
P1 = [1,2,3,4;5,6,7,8;9,10,11,12];
Q1 = [2,4,6,8;1,2,3,4;3,6,9,12;1,0,1,0];
dC_dP_test = double(subs(subs(dC_dP, P, P1), Q, Q1))

% check correct
dC_dP_analytical = kron(eye(3), P1*Q1) * TvecMat(3,4) + kron(P1*Q1.', eye(3))


