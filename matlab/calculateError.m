function [C, dC_dx, dC_dq] = calculateError(x, q, K, measurement, calculateDerivatives)
[X, dX_dx] = buildPose(x, calculateDerivatives);
[Q, dQ_dq] = buildQuadric(q, calculateDerivatives);
[P, dP_dX] = calculateProjection(X, K, calculateDerivatives);
[C, dC_dQ, dC_dP] = calculateConic(Q, P, calculateDerivatives);
[b, db_dC] = getBounds(C, calculateDerivatives);
e = b-measurement; 

if calculateDerivatives
    dC_dx = dC_dP * dP_dX * dX_dx;
    dC_dq = dC_dQ * dQ_dq;
else
    dC_dx = zeros(9,6);
    dC_dq = zeros(9,9);
end
end

function [b, db_dC] = getBounds(C, calculateDerivatives)
xmin = (C(1,3) + sqrt(C(1,3).^2 - (C(1,1) * C(3,3)))) / C(3,3);
xmax = (C(1,3) - sqrt(C(1,3).^2 - (C(1,1) * C(3,3)))) / C(3,3);
ymin = (C(2,3) + sqrt(C(2,3).^2 - (C(2,2) * C(3,3)))) / C(3,3);
ymax = (C(2,3) - sqrt(C(2,3).^2 - (C(2,2) * C(3,3)))) / C(3,3);
b = [xmin, ymin, xmax, ymax];

% recursively calculate jacobian
if calculateDerivatives
    C_ = sym('C_',[3,3]);
    [b_, ~] = getBounds(C_, false);
    db_dC_ = jacobian(b_, C_(:));
    db_dC = double(subs(db_dC_, C_, C));
else
    db_dC = zeros(4,9);
end
end

% normalize Q?
function [C, dC_dQ, dC_dP] = calculateConic(Q, P, calculateDerivatives)
C = P * Q * P.';

if calculateDerivatives
    dC_dP = kron(eye(3), P*Q) * TvecMat(3,4) + kron(P*Q, eye(3));
    dC_dQ = kron(P,P);
else
    dC_dP = zeros(9,12);
    dC_dQ = zeros(9,16);
end
end

function [P, dP_dX] = calculateProjection(X, K, calculateDerivatives)
P = K * eye(3,4) * inv(X);

if calculateDerivatives
    dXi_dX = -1.0 * kron(inv(X).', inv(X));
    dP_dXi = kron(eye(4), K*eye(3,4));
    dP_dX = dP_dXi * dXi_dX;
else
    dP_dX = zeros(12,16);
end
end

function [Q, dQ_dq] = buildQuadric(q, calculateDerivatives)
[Z, dZ_dx] = buildPose(q(1:6), calculateDerivatives);
Qc = diag([q(7:9).^2, -1.0]);
Q = Z * Qc * Z.';

if calculateDerivatives
    dZ_dq = zeros(16,9);
    dZ_dq(1:16,1:6) = dZ_dx;
    dQc_dq = zeros(16,9); 
    dQc_dq(1,7) = 2.0*q(7);
    dQc_dq(6,8) = 2.0*q(8);
    dQc_dq(11,9) = 2.0*q(9);

    
    dQ_dq = kron(eye(4), Z*Qc) * TvecMat(4,4) * dZ_dq + kron(Z, eye(4)) * (kron(eye(4), Z)*dQc_dq + kron(Qc, eye(4)) * dZ_dq);
else
    dQ_dq = zeros(16,9);
end
end

function [X, dX_dx] = buildPose(x, calculateDerivatives)
[R, dR_dr] = buildRotation(x(1:3), calculateDerivatives);
t = x(4:6);
X = [R,t.'; 0,0,0,1];

if calculateDerivatives
    dX_dx = zeros(16,6);
    dX_dx(1:3,1:3) = dR_dr(1:3,1:3);
    dX_dx(5:7,1:3) = dR_dr(4:6,1:3);
    dX_dx(9:11,1:3) = dR_dr(7:9,1:3);
    dX_dx(13,4) = 1.0;
    dX_dx(14,5) = 1.0;
    dX_dx(15,6) = 1.0;
else 
    dX_dx = zeros(16,6);
end
end

function [R, dR_dr] = buildRotation(r, calculateDerivatives)
x = r(1); y = r(2); z = r(3);
x2 = x.^2; y2 = y.^2; z2 = z.^2;
xy = x*y; xz = x*z; yz = y*z;
f = 1.0 / (4.0+x2+y2+z2); f2 = 2.0*f;
R = [(4 + x2 - y2 - z2) * f, (xy - 2 * z) * f2, (xz + 2 * y) * f2;
     (xy + 2 * z) * f2, (4 - x2 + y2 - z2) * f, (yz - 2 * x) * f2;
     (xz - 2 * y) * f2, (yz + 2 * x) * f2, (4 - x2 - y2 + z2) * f];
 
% recursively calculate jacobian
if calculateDerivatives
    r_ = sym('r_',[1,3]);
    [R_, ~] = buildRotation(r_, false);
    dR_dr = double(subs(jacobian(R_(:), r_), r_, r));
%     d = (x2 + y2 + z2 + 4).^2;
%     dR_dr = [(4*x*(y2+z2)), (2 * (y * (-x2 + z2 + 4) - 4*x*z + y.^3)), (-2*x2*z + 8*x*y + 2*z*(y2 + z2 + 4)),
%             (2 * (y * (-x2 + z2 + 4) + 4*x*z + y.^3)), -(4*x*(y2 + 4)),-(4* (x2 + x*y*z - y2 - z2 - 4)),
%             (-2*x2*z -8*x*y + 2*z*(y2 + z2 + 4)), (4*(x2 - x*y*z - y2 - z2 - 4)), -(4*x*(z2+4))] / d;
else
    dR_dr = zeros(9,3);
end
end










