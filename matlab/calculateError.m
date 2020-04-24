function calculateError(x, q, K, measurement)
% derive = 'none', 'analytical', 'symbolic'
format shortG

% TEST JACOBIANS (analytical)
disp('ANALYTICAL JACOBIANS')
derive = 'analytical';
[X, dX_dx__a] = buildPose(x, derive)
[Q, dQ_dq__a] = buildQuadric(q, derive)
[P, dP_dX__a] = calculateProjection(X, K, derive)
[C, dC_dQ__a, dC_dP__a] = calculateConic(Q, P, derive)
[b, db_dC__a] = getBounds(C, derive)
dC_dx__a = dC_dP__a * dP_dX__a * dX_dx__a
dC_dq__a = dC_dQ__a * dQ_dq__a
db_dx__a = db_dC__a * dC_dx__a
db_dq__a = db_dC__a * dC_dq__a

% TEST JACOBIANS (symbolic)
disp('');
disp('SYMBOLIC JACOBIANS')
derive = 'symbolic';
[X, dX_dx__s] = buildPose(x, derive)
[Q, dQ_dq__s] = buildQuadric(q, derive)
[P, dP_dX__s] = calculateProjection(X, K, derive)
[C, dC_dQ__s, dC_dP__s] = calculateConic(Q, P, derive)
[b, db_dC__s] = getBounds(C, derive)

x_ = sym('x_', [1,6]);
q_ = sym('q_', [1,9]);
[X_,~] = buildPose(x_, 'none');
[Q_,~] = buildQuadric(q_, 'none');
[P_,~] = calculateProjection(X_,K, 'none');
[C_,~,~] = calculateConic(Q_,P_, 'none');
% C_ = C_/C_(3,3);
% c_ = [C_(1,1),C_(1,2),C_(2,2),C_(1,3),C_(2,3)];
% C_ = [c_(1),c_(2),c_(4);c_(2),c_(3),c_(5);c_(4),c_(5),1.0];
[b_,~] = getBounds(C_, 'none');
db_dx__s = double(subs(subs(jacobian(b_,x_), x_, x), q_, q))
db_dq__s = double(subs(subs(jacobian(b_,q_), x_, x), q_, q))


% COMPARE
disp('COMPARING: ')

fprintf('\ndX_dx')
isClose(dX_dx__s, dX_dx__a)
fprintf('\ndQ_dq')
isClose(dQ_dq__s, dQ_dq__a)
fprintf('\ndP_dX')
isClose(dP_dX__s, dP_dX__a)
fprintf('\ndC_dQ')
isClose(dC_dQ__s, dC_dQ__a)
fprintf('\ndC_dP')
isClose(dC_dP__s, dC_dP__a)
fprintf('\ndb_dC')
isClose(db_dC__s, db_dC__a)
fprintf('\ndb_dx')
isClose(db_dx__s, db_dx__a)
fprintf('\ndb_dq')
isClose(db_dq__s, db_dq__a)

disp('debug')
end

function r = isClose(A, B)
tol = 1e-08;
r = abs(A-B)<tol;
end

function [b, db_dC] = getBounds(C, derive)
% C = C/C(3,3);
xmin = (C(1,3) + sqrt(C(1,3).^2 - (C(1,1) * C(3,3)))) / C(3,3);
xmax = (C(1,3) - sqrt(C(1,3).^2 - (C(1,1) * C(3,3)))) / C(3,3);
ymin = (C(2,3) + sqrt(C(2,3).^2 - (C(2,2) * C(3,3)))) / C(3,3);
ymax = (C(2,3) - sqrt(C(2,3).^2 - (C(2,2) * C(3,3)))) / C(3,3);
b = [xmin, ymin, xmax, ymax];


if strcmp(derive, 'analytical') || strcmp(derive, 'symbolic')
    C_ = sym('C_',[3,3]);
    [b_, ~] = getBounds(C_, 'none');
    db_dC_ = jacobian(b_, C_(:));
    db_dC = double(subs(db_dC_, C_, C));
else
    db_dC = zeros(4,9);
end
end

% normalize Q?
function [C, dC_dQ, dC_dP] = calculateConic(Q, P, derive)
C = P * Q * P.';

if strcmp(derive, 'analytical')
    dC_dP = kron(eye(3), P*Q) * TvecMat(3,4) + kron(P*Q, eye(3));
    dC_dQ = kron(P,P);
elseif strcmp(derive, 'symbolic')
    P_ = sym('P_', [3,4]);
    Q_ = sym('Q_', [4,4]);
    [C_,~,~] = calculateConic(Q_,P_, 'none');
    dC_dP = double(subs(subs(jacobian(C_(:),P_(:)), P_, P), Q_, Q));
    dC_dQ = double(subs(subs(jacobian(C_(:),Q_(:)), P_, P), Q_, Q));
else
    dC_dP = zeros(9,12);
    dC_dQ = zeros(9,16);
end
end

function [P, dP_dX] = calculateProjection(X, K, derive)
P = K * eye(3,4) * inv(X);

if strcmp(derive, 'analytical')
    dXi_dX = -1.0 * kron(inv(X).', inv(X))
    dP_dXi = kron(eye(4), K*eye(3,4));
    dP_dX = dP_dXi * dXi_dX;
elseif strcmp(derive, 'symbolic')
    X_ = sym('X_', [4,4]);
    [P_,~] = calculateProjection(X_, K, 'none');
    dP_dX = double(subs(jacobian(P_(:), X_(:)), X_, X));
else
    dP_dX = zeros(12,16);
end
end

function [Q, dQ_dq] = buildQuadric(q, derive)
[Z, dZ_dx] = buildPose(q(1:6), derive);
Qc = diag([q(7:9).^2, -1.0]);
Q = Z * Qc * Z.';

if strcmp(derive, 'analytical')
    dZ_dq = zeros(16,9);
    dZ_dq(1:16,1:6) = dZ_dx;
    dQc_dq = zeros(16,9); 
    dQc_dq(1,7) = 2.0*q(7);
    dQc_dq(6,8) = 2.0*q(8);
    dQc_dq(11,9) = 2.0*q(9);
    
    dQ_dq = kron(eye(4), Z*Qc) * TvecMat(4,4) * dZ_dq + kron(Z, eye(4)) * (kron(eye(4), Z)*dQc_dq + kron(Qc, eye(4)) * dZ_dq);
elseif strcmp(derive, 'symbolic')
    q_ = sym('q_', [1,9]);
    [Q_,~] = buildQuadric(q_, 'none');
    dQ_dq = double(subs(jacobian(Q_(:),q_), q_, q));
else
    dQ_dq = zeros(16,9);
end
end

function [X, dX_dx] = buildPose(x, derive)
[R, dR_dr] = buildRotation(x(1:3), derive);
t = x(4:6);
X = [R,t.'; 0,0,0,1];

if strcmp(derive, 'analytical')
    dX_dx = zeros(16,6);
    dX_dx(1:3,1:3) = dR_dr(1:3,1:3);
    dX_dx(5:7,1:3) = dR_dr(4:6,1:3);
    dX_dx(9:11,1:3) = dR_dr(7:9,1:3);
    dX_dx(13,4) = 1.0;
    dX_dx(14,5) = 1.0;
    dX_dx(15,6) = 1.0;
elseif strcmp(derive, 'symbolic')
    x_ = sym('x_', [1,6]);
    [X_,~] = buildPose(x_, 'none');
    dX_dx = double(subs(jacobian(X_(:), x_), x_, x));
else 
    dX_dx = zeros(16,6);
end
end

function [R, dR_dr] = buildRotation(r, derive)
x = r(1); y = r(2); z = r(3);
x2 = x.^2; y2 = y.^2; z2 = z.^2;
xy = x*y; xz = x*z; yz = y*z;
f = 1.0 / (4.0+x2+y2+z2); f2 = 2.0*f;
R = [(4 + x2 - y2 - z2) * f, (xy - 2 * z) * f2, (xz + 2 * y) * f2;
     (xy + 2 * z) * f2, (4 - x2 + y2 - z2) * f, (yz - 2 * x) * f2;
     (xz - 2 * y) * f2, (yz + 2 * x) * f2, (4 - x2 - y2 + z2) * f];
 
% recursively calculate jacobian
if strcmp(derive, 'analytical') || strcmp(derive, 'symbolic')
    r_ = sym('r_',[1,3]);
    [R_, ~] = buildRotation(r_, false);
    dR_dr_ = jacobian(R_(:), r_);
    dR_dr = double(subs(dR_dr_, r_, r));
%     d = (x2 + y2 + z2 + 4).^2;
%     dR_dr = [(4*x*(y2+z2)), (2 * (y * (-x2 + z2 + 4) - 4*x*z + y.^3)), (-2*x2*z + 8*x*y + 2*z*(y2 + z2 + 4)),
%             (2 * (y * (-x2 + z2 + 4) + 4*x*z + y.^3)), -(4*x*(y2 + 4)),-(4* (x2 + x*y*z - y2 - z2 - 4)),
%             (-2*x2*z -8*x*y + 2*z*(y2 + z2 + 4)), (4*(x2 - x*y*z - y2 - z2 - 4)), -(4*x*(z2+4))] / d;
else
    dR_dr = zeros(9,3);
end
end










