%% General
g = 9.81; % m/s^2

%% Length
% length of arm1 from motor shaft to pivot of arm2
L1 = 0.1146; % m
% length of arm2 from pivot to the tip
L2 = 0.145; % m

% distance from motor shaft to CG of arm1 (from CAD)
l1 = 0.06151443; % m
% distance from arm2 pivot to CG of arm2 (from CAD)
l2 = 0.09155541; % m

%% Mass and volume for CAD density values
% mass of arm1 itself
m1_arm = 17.31e-3; % kg
v1_arm = 0.00003024; % m^3
% mass of arm2 itself
m2_arm = 9.09e-3; % kg
v2_arm = 0.00001234; % m^3
% mass of 4 nuts and 4 bolts at the end of the arm2
m2_nuts_bolts = 9.93e-3; % kg
v2_nuts_bolts = 0.00000158; % m^3
% mass of all encoder parts
m1_encoder_parts = 15.8e-3; % kg
v1_encoder_parts = 0.00000490; % m^3
% mass of ball bearing
m1_bearing = 10.9e-3; % kg
v1_bearing = 0.00000211; % m^3
% mass of shaft coupler
m1_shaft_coupler = 10e-3; % kg
v1_shaft_coupler = 0.00000129; % m^3

% total mass of arm1 at CG (from CAD)
m1 = 57.34349e-3; % kg 
% total mass of arm2 at CG (from CAD)
m2 = 18.99165e-3; % kg

%% Inertia tensor. Taken at the center of mass, aligned with main axes of the arm, negative tensor notation
% inertia tensor for arm1
J1xx = 1.25e-5;	J1xy = 4.1e-7;	J1xz = -2.326e-5;
J1yy = 1.2227e-04;	Lyz = 1e-7;
J1zz = 1.1482e-4; % kg-m^2s

% inertia tensor for arm2
J2xx = 8e-07; J2xy = 0; J2xz = -1.67e-06;
J2yy = 4.258e-05; J2yz = 0;
J2zz = 4.198e-05; % kg-m^2s


%% Friction
% dynamic friction coefficient of pololu-25d-4844 motor (Ivan)
% b1 = 0.0075423; 
% dynamic friction coefficient of amazon motor (Jacob RWIP)
b1 = 0.001535; 

% dynamic friction coefficient of arm2 pivot
b2 = 4e-5; % N-m-s

%% Electrical
%{
% pololu-25d-4844 motor (Ivan)
% motor inductance
Lm = 0.0469406; % Henry
% motor resistance
Rm = 4; % Ohm
% motor back emf (also torque) constant
Km = 0.0989468; % N-m/A
%}

% amazon motor (Jacob RWIP)
Km = 0.0794447; %%% LOOK AT ME, MATLAB CAME UP WITH SLIGHTLY DIFFERENT NUMBERS THAN THESE, GO TO MOTORCONSTANTSCALC.M and SOLVE THOSE EQUATIONS WITH YOUR CALCULATOR TO CONFIRM
Lm = 0.5728439;
Rm = 9;

% magnitude of maximum voltage the motor will receive
Vmax = 12; % Volts