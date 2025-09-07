clc;
clear;
close all;

% Link parameters
L1 = 0.5; % base height
L2 = 1.0;
L3 = 1.0;
L5 = 0.2;

robot = rigidBodyTree('DataFormat','column','MaxNumBodies',5);

% Link1 - Revolute q1
body1 = rigidBody('Link1');
joint1 = rigidBodyJoint('joint1','revolute');
setFixedTransform(joint1,trvec2tform([0 0 L1]));
joint1.JointAxis = [0 0 1];
body1.Joint = joint1;
addBody(robot, body1, 'base');

% Link2 - Revolute q2
body2 = rigidBody('Link2');
joint2 = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint2,trvec2tform([L2 0 0]));
joint2.JointAxis = [0 0 1];
body2.Joint = joint2;
addBody(robot, body2, 'Link1');

% Link3 - Prismatic d3
body3 = rigidBody('Link3');
joint3 = rigidBodyJoint('joint3','prismatic');
setFixedTransform(joint3,trvec2tform([L3 0 0]));
joint3.JointAxis = [0 0 1];
body3.Joint = joint3;
addBody(robot, body3, 'Link2');

% Link4 - Revolute q4
body4 = rigidBody('Link4');
joint4 = rigidBodyJoint('joint4','revolute');
setFixedTransform(joint4,trvec2tform([0 0 0]));
joint4.JointAxis = [0 0 1];
body4.Joint = joint4;
addBody(robot, body4, 'Link3');

% End effector
ee = rigidBody('end_effector');
setFixedTransform(ee.Joint,trvec2tform([0 0 L5]));
addBody(robot, ee, 'Link4');

% Create figure
figure;
ax = axes;
hold on;
grid on;
view(3);
axis equal;
xlim([-2 2]);
ylim([-2 2]);
zlim([0 2]);

% Circular XY path
theta = linspace(0, pi/2, 100);
r = 1.5;
Xe = r * cos(theta);
Ye = r * sin(theta);
Ze = ones(size(Xe)) * 1.0;  % fixed Z height

for i = 1:length(Xe)
    x = Xe(i);
    y = Ye(i);
    z = Ze(i);
    
    % Planar inverse kinematics
    rxy = sqrt(x^2 + y^2);
    phi = atan2(y, x);
    cos_q2 = (rxy^2 - L2^2 - L3^2) / (2 * L2 * L3);
    cos_q2 = min(max(cos_q2, -1), 1);  % Clamp to avoid complex numbers
    sin_q2 = sqrt(1 - cos_q2^2);
    
    q2 = atan2(sin_q2, cos_q2);
    q1 = phi - atan2(L3*sin_q2, L2 + L3*cos_q2);
    d3 = L1 - z;
    q4 = 0;

    % Configuration
    config = [q1; q2; d3; q4];
    
    cla(ax);
    show(robot, config, 'Parent', ax, 'PreservePlot', false, 'Frames', 'off');
    title(sprintf('Step %d | q1=%.2f, q2=%.2f, d3=%.2f', i, q1, q2, d3));
    drawnow;
    pause(0.05);
end
