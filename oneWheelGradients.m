function [df] = oneWheelGradients(m, I, wheel, t, q, qd, u)
% Unpack variables
q_1 = q(1);
q_2 = q(2);
q_3 = q(3);

qd_1 = qd(1);
qd_2 = qd(2);
qd_3 = qd(3);

t = t;

u = u;

wheel_driveDir_1 = wheel.driveDir(1);
wheel_driveDir_2 = wheel.driveDir(2);
wheel_omegaMax = wheel.omegaMax;
wheel_pos_1 = wheel.pos(1);
wheel_pos_2 = wheel.pos(2);
wheel_r = wheel.r;
wheel_slipDir_1 = wheel.slipDir(1);
wheel_slipDir_2 = wheel.slipDir(2);
wheel_tauMax = wheel.tauMax;

% Conpute gradient
df = sparse(3,8);
df(1,4) = -(wheel_tauMax*(qd_2*wheel_slipDir_2^2*cos(2*q_3) - qd_2*wheel_slipDir_1^2*cos(2*q_3) + qd_1*wheel_slipDir_1^2*sin(2*q_3) - qd_1*wheel_slipDir_2^2*sin(2*q_3) + 2*qd_2*wheel_slipDir_1*wheel_slipDir_2*sin(2*q_3) + qd_3*wheel_pos_2*wheel_slipDir_2^2*sin(q_3) + 2*qd_1*wheel_slipDir_1*wheel_slipDir_2*cos(2*q_3) - qd_3*wheel_pos_1*wheel_slipDir_1^2*cos(q_3) - qd_3*wheel_pos_2*wheel_slipDir_1*wheel_slipDir_2*cos(q_3) + qd_3*wheel_pos_1*wheel_slipDir_1*wheel_slipDir_2*sin(q_3) + u*wheel_r*wheel_omegaMax*wheel_slipDir_1^2*wheel_driveDir_2*cos(q_3) + u*wheel_r*wheel_omegaMax*wheel_slipDir_2^2*wheel_driveDir_1*sin(q_3) - u*wheel_r*wheel_omegaMax*wheel_slipDir_1*wheel_slipDir_2*wheel_driveDir_2*sin(q_3) - u*wheel_r*wheel_omegaMax*wheel_slipDir_1*wheel_slipDir_2*wheel_driveDir_1*cos(q_3)))/(m*wheel_r^2*wheel_omegaMax*(wheel_slipDir_1*wheel_driveDir_2 - wheel_slipDir_2*wheel_driveDir_1)^2);
df(1,5) = -(wheel_tauMax*(wheel_slipDir_2*cos(q_3) + wheel_slipDir_1*sin(q_3))^2)/(m*wheel_r^2*wheel_omegaMax*(wheel_slipDir_1*wheel_driveDir_2 - wheel_slipDir_2*wheel_driveDir_1)^2);
df(1,6) = (wheel_tauMax*((wheel_slipDir_1^2*sin(2*q_3))/2 - (wheel_slipDir_2^2*sin(2*q_3))/2 + wheel_slipDir_1*wheel_slipDir_2*cos(2*q_3)))/(m*wheel_r^2*wheel_omegaMax*(wheel_slipDir_1*wheel_driveDir_2 - wheel_slipDir_2*wheel_driveDir_1)^2);
df(1,7) = (wheel_tauMax*(wheel_slipDir_2*cos(q_3) + wheel_slipDir_1*sin(q_3))*(wheel_pos_1*wheel_slipDir_1 + wheel_pos_2*wheel_slipDir_2))/(m*wheel_r^2*wheel_omegaMax*(wheel_slipDir_1*wheel_driveDir_2 - wheel_slipDir_2*wheel_driveDir_1)^2);
df(1,8) = -(wheel_tauMax*(wheel_slipDir_2*cos(q_3) + wheel_slipDir_1*sin(q_3)))/(m*wheel_r*(wheel_slipDir_1*wheel_driveDir_2 - wheel_slipDir_2*wheel_driveDir_1));
df(2,4) = (wheel_tauMax*(qd_1*wheel_slipDir_1^2*cos(2*q_3) - qd_1*wheel_slipDir_2^2*cos(2*q_3) + qd_2*wheel_slipDir_1^2*sin(2*q_3) - qd_2*wheel_slipDir_2^2*sin(2*q_3) - 2*qd_1*wheel_slipDir_1*wheel_slipDir_2*sin(2*q_3) + qd_3*wheel_pos_1*wheel_slipDir_1^2*sin(q_3) + 2*qd_2*wheel_slipDir_1*wheel_slipDir_2*cos(2*q_3) + qd_3*wheel_pos_2*wheel_slipDir_2^2*cos(q_3) + qd_3*wheel_pos_1*wheel_slipDir_1*wheel_slipDir_2*cos(q_3) + qd_3*wheel_pos_2*wheel_slipDir_1*wheel_slipDir_2*sin(q_3) + u*wheel_r*wheel_omegaMax*wheel_slipDir_2^2*wheel_driveDir_1*cos(q_3) - u*wheel_r*wheel_omegaMax*wheel_slipDir_1^2*wheel_driveDir_2*sin(q_3) + u*wheel_r*wheel_omegaMax*wheel_slipDir_1*wheel_slipDir_2*wheel_driveDir_1*sin(q_3) - u*wheel_r*wheel_omegaMax*wheel_slipDir_1*wheel_slipDir_2*wheel_driveDir_2*cos(q_3)))/(m*wheel_r^2*wheel_omegaMax*(wheel_slipDir_1*wheel_driveDir_2 - wheel_slipDir_2*wheel_driveDir_1)^2);
df(2,5) = (wheel_tauMax*((wheel_slipDir_1^2*sin(2*q_3))/2 - (wheel_slipDir_2^2*sin(2*q_3))/2 + wheel_slipDir_1*wheel_slipDir_2*cos(2*q_3)))/(m*wheel_r^2*wheel_omegaMax*(wheel_slipDir_1*wheel_driveDir_2 - wheel_slipDir_2*wheel_driveDir_1)^2);
df(2,6) = -(wheel_tauMax*(wheel_slipDir_1*cos(q_3) - wheel_slipDir_2*sin(q_3))^2)/(m*wheel_r^2*wheel_omegaMax*(wheel_slipDir_1*wheel_driveDir_2 - wheel_slipDir_2*wheel_driveDir_1)^2);
df(2,7) = -(wheel_tauMax*(wheel_slipDir_1*cos(q_3) - wheel_slipDir_2*sin(q_3))*(wheel_pos_1*wheel_slipDir_1 + wheel_pos_2*wheel_slipDir_2))/(m*wheel_r^2*wheel_omegaMax*(wheel_slipDir_1*wheel_driveDir_2 - wheel_slipDir_2*wheel_driveDir_1)^2);
df(2,8) = (wheel_tauMax*(wheel_slipDir_1*cos(q_3) - wheel_slipDir_2*sin(q_3)))/(m*wheel_r*(wheel_slipDir_1*wheel_driveDir_2 - wheel_slipDir_2*wheel_driveDir_1));
df(3,4) = (wheel_tauMax*(wheel_pos_1*wheel_slipDir_1 + wheel_pos_2*wheel_slipDir_2)*(qd_1*wheel_slipDir_1*cos(q_3) + qd_2*wheel_slipDir_2*cos(q_3) - qd_1*wheel_slipDir_2*sin(q_3) + qd_2*wheel_slipDir_1*sin(q_3)))/(I*wheel_r^2*wheel_omegaMax*(wheel_slipDir_1*wheel_driveDir_2 - wheel_slipDir_2*wheel_driveDir_1)^2);
df(3,5) = (wheel_tauMax*(wheel_slipDir_2*cos(q_3) + wheel_slipDir_1*sin(q_3))*(wheel_pos_1*wheel_slipDir_1 + wheel_pos_2*wheel_slipDir_2))/(I*wheel_r^2*wheel_omegaMax*(wheel_slipDir_1*wheel_driveDir_2 - wheel_slipDir_2*wheel_driveDir_1)^2);
df(3,6) = -(wheel_tauMax*(wheel_slipDir_1*cos(q_3) - wheel_slipDir_2*sin(q_3))*(wheel_pos_1*wheel_slipDir_1 + wheel_pos_2*wheel_slipDir_2))/(I*wheel_r^2*wheel_omegaMax*(wheel_slipDir_1*wheel_driveDir_2 - wheel_slipDir_2*wheel_driveDir_1)^2);
df(3,7) = -(wheel_tauMax*(wheel_pos_1*wheel_slipDir_1 + wheel_pos_2*wheel_slipDir_2)^2)/(I*wheel_r^2*wheel_omegaMax*(wheel_slipDir_1*wheel_driveDir_2 - wheel_slipDir_2*wheel_driveDir_1)^2);
df(3,8) = (wheel_tauMax*(wheel_pos_1*wheel_slipDir_1 + wheel_pos_2*wheel_slipDir_2))/(I*wheel_r*(wheel_slipDir_1*wheel_driveDir_2 - wheel_slipDir_2*wheel_driveDir_1));
end
