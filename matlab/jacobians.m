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
K = [320,0,320; 0,320,240; 0,0,1];
I3x4 = [1,0,0,0; 0,1,0,0; 0,0,1,0];

% build Q from qvec
Qv = [sym('Q1'), sym('Q2'), sym('Q3'), sym('Q4'), sym('Q5'), sym('Q6'), sym('Q7'), sym('Q8'), sym('Q9')];
Q_rotation = [(4 + Qv(1)^2 - Qv(2)^2 - Qv(3)^2), (2 * Qv(1) * Qv(2) - 4 * Qv(3)), (4 * Qv(2) + 2 * Qv(1) * Qv(3)); 
                    (2 * Qv(1) * Qv(2) + 4 * Qv(3)), (4 - Qv(1)^2 + Qv(2)^2 - Qv(3)^2), (2 * (-2 * Qv(1) + Qv(2) * Qv(3))); 
                    (2 * (-2 * Qv(2) + Qv(1) * Qv(3))), (4 * Qv(1) + 2 * Qv(2) * Qv(3)), (4 - Qv(1)^2 - Qv(2)^2 + Qv(3)^2)];
Q_rotation = Q_rotation / (4 + Qv(1)^2 + Qv(2)^2 + Qv(3)^2);
Q_translation = [Qv(4); Qv(5); Qv(6)];
Q_pose = [Q_rotation, Q_translation; 0,0,0,1];

Q_shape = sym(zeros(4,4));
Q_shape(1,1) = Qv(7)^2;
Q_shape(2,2) = Qv(8)^2;
Q_shape(3,3) = Qv(9)^2;
Q_shape(4,4) = -1;
Q = Q_pose * Q_shape * Q_pose.';





