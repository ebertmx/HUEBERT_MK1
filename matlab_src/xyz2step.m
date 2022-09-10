function [S] = xyz2step(P)
%Matthew Ebert; 2021-05-25
%HUEBERT arm position
%give value in mm
%constants
%Dimension of the robot arm; names based on part names in solidworks
%assembly TECHRAM_MK3.SLDASM
OG = 80 /1000; %m
L1 = 255/ 1000; %m
CL1 = 141.42136/1000; %mxx
L1m = 110/1000; %m
L2 = 260.0 / 1000; %m
CL2 = 280.44607 /1000; %m
L2e = 80 /1000; %m
OL=109.37002/1000; %m
OX = 105.50000 /1000; %m
OY = 28.83664 /1000; %m
XC = 48/1000;
qO=0.26681587; %rads
%Q2t= (0.83+ Q2*(pi-0.83)/(780-0))

X = P(1)/1000;
Y = P(2)/1000;
Z = P(3)/1000;
%Map analog values to relative angle
% Q1 = 0.33+ Q1*(2.41-0.33)/(1023);
% Q2 = pi-(0.83+ Q2*(pi-0.83)/(780-0)) -0.88631-Q1-1.84818;
% Q3 = -pi + Q3*2*pi/1024;
RXZ = sqrt(X^2+Z^2)-XC;
alpha_YX = atan2 (Y,(RXZ));
R_XY = sqrt(RXZ^2 + Y^2);
beta_L1xRXY = acos(((R_XY^2)+(L1^2)-(L2^2))/(2*R_XY*L1));
Q1 =  beta_L1xRXY + alpha_YX;
Q2 = acos(((L2^2)+(L1^2)-(R_XY^2))/(2*L2*L1));
Q3 = atan2(Z,X);

%Find OG1 angle

q1_theta = pi-Q1-qO;
O_L1m = sqrt(L1m^2 +OL^2 - 2*L1m*OL*cos(q1_theta));
q1_beta =  acos( ((OG)^2  - (CL1)^2 + (O_L1m)^2)/(2*OG*O_L1m));
q1_alpha =  acos( ((OL)^2  - (L1m)^2 + (O_L1m)^2)/(2*OL*O_L1m));

qS1 = q1_beta+ q1_alpha; %Angle of OG1 referenced to the line between center of planetary gear and pivot of L1

%Find OG2 angle
R2 = sqrt(L1^2 +L2e^2 - 2*L1*L2e*cos(pi-Q2));
q_L1_OL2e = acos( (-(L2e)^2  + (R2)^2 + (L1)^2)/(2*R2*L1));
R1 =  sqrt(OL^2 + R2^2 - 2*OL*R2*cos(pi-Q1 - qO - q_L1_OL2e));
q2_beta =  acos( ((OG)^2  - (CL2)^2 + (R1)^2)/(2*OG*R1));
q2_alpha =  acos( ((OL)^2  - (R2)^2 + (R1)^2)/(2*OL*R1));

qS2 = q2_alpha + q2_beta;%Angle of OG2 referenced to the line between center of planetary gear and pivot of L1

qS3 = Q3; %Angle of assembly reference to front.

%Map angle to steps for Motor
S1 = round((3.442 - qS1) / (2 * pi) * 4 *4 * 200);
S2 =round( (4.114 - qS2) / (2 * pi) * 4 *4 * 200);
S3 = round((qS3 * 13*4) * 200 / (2 * pi));
S = [S1 S2 S3];
return
end

