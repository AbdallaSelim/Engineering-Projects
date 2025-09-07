clc
clear
close all
syms q1 q2 d3 q4 real
syms dq1 dq2 dd3 dq4 real
syms ddq1 ddq2 ddd3 ddq4 real
syms L1 L2 L3 L5 real
syms I1 I2 I4 real
syms m1 m2 m3 m4 real 
syms Ixx1 Iyy1 Izz1 Ixx2 Iyy2 Izz2 Ixx4 Iyy4 Izz4 real

q = [q1 q2 d3 q4]';
dq = [dq1 dq2 dd3 dq4]';
ddq = [ddq1 ddq2 ddd3 ddq4]';
g = 9.81;
%%%%%%%% Rotation Matrices %%%%%%%%%%
R10 = rotz(q1);
R20 = R10 * rotz(q2);
R40 = R20 * rotz(q4);

%%%%%%%%% Forward kinemtaics %%%%%%%%%
P0 = [0 0 L1]';
P1 = P0 + R10 * [L2 0 0]';
C1 = P0 + R10 * 0.5 * [L2 0 0]';
P2 = P1 + R20 * [L3 0 0]';
C2 = P1 + R20 * 0.5 * [L3 0 0]';
P3 = P2 + [0 0 d3]'; % Doesn't rotate. Be careful with d3 because it'll mostly move down
C3 = P2 + 0.5 * [0 0 d3]';
P4 = P3 + R40 * [0 0 L5]'
C4 = P3 + R40 * 0.5 * [0 0 L5]';

%%%%%%%%%% Jacobians %%%%%%%%%%
%%%%%%% Linear %%%%%%%
Jac_End = jacobian(P4, q);
Jac_COM_1 = jacobian(C1, q);
Jac_COM_2 = jacobian(C2, q);
Jac_COM_3 = jacobian(C3, q);
Jac_COM_4 = jacobian(C4, q);

%%%%%%%% Rotational %%%%%%%%%
dR10 = diff(R10,q1)*dq1 + diff(R10,q2)*dq2 + diff(R10,q4)*dq4;
dR20 = diff(R20,q1)*dq1 + diff(R20,q2)*dq2 + diff(R20,q4)*dq4;
dR40 = diff(R40,q1)*dq1 + diff(R40,q2)*dq2 + diff(R40,q4)*dq4;

% Skew symmetric matrix. Angular velocities will be extracted from here
Wsk1 = simplify(dR10*R10');
Wsk2 = simplify(dR20*R20');
Wsk4 = simplify(dR40*R40');

W1(1,1) = Wsk1(3,2);
W1(2,1) = Wsk1(1,3);
W1(3,1) = Wsk1(2,1);

W2(1,1) = Wsk2(3,2);
W2(2,1) = Wsk2(1,3);
W2(3,1) = Wsk2(2,1);

W4(1,1) = Wsk4(3,2);
W4(2,1) = Wsk4(1,3);
W4(3,1) = Wsk4(2,1);

Jw1 = jacobian(W1,dq);
Jw2 = jacobian(W2,dq);
Jw4 = jacobian(W4,dq);

%%%%%%%%% M matrix %%%%%%%%%%%%5
I1(1,1) = Ixx1;
I1(2,2) = Iyy1;
I1(3,3) = Izz1;

I2(1,1) = Ixx2;
I2(2,2) = Iyy2;
I2(3,3) = Izz2;

I4(1,1) = Ixx4;
I4(2,2) = Iyy4;
I4(3,3) = Izz4;

M = simplify(m1*(Jac_COM_1)'*Jac_COM_1 + Jw1'*R10*I1*R10'*Jw1 ...
           + m2*(Jac_COM_2)'*Jac_COM_2 + Jw2'*R20*I2*R20'*Jw2 ...
           + m3 *(Jac_COM_3)'*Jac_COM_3 + ...
           + m4 *(Jac_COM_4)'*Jac_COM_4 + Jw4'*R40*I4*R40'*Jw4);

%%%%%%%%%%% C matrix %%%%%%%%%%%
dM = diff(M,q1)*dq1 + diff(M,q2)*dq2 + diff(M,d3) * dd3 + diff(M,q4) * dq4;
C = simplify(dM * dq - 0.5 * jacobian(dq'*M*dq, q)');

%%%%%%%%%% G matrix %%%%%%%%
G = simplify(m1*(Jac_COM_1)'*[0; 0; g] + m2*(Jac_COM_2)'*[0; 0; g] + m3*(Jac_COM_3)'*[0; 0; g]+ m4*(Jac_COM_4)'*[0; 0; g]);

%%%%%%%% Matlab functions %%%%%%%%%
% matlabFunction(M,'File','M_scara','Optimize',true);
% matlabFunction(C,'File','C_scara','Optimize',true);
% matlabFunction(G,'File','G_scara','Optimize',true);