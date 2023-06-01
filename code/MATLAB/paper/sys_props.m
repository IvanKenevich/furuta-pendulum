%% General
g = 9.81; % m/s^2

%% Length
% length of arm1 from motor shaft to pivot of arm2
L1 = 0.112; % m
% length of arm2 from pivot to the tip
L2 = 0.144; % m
% distance from motor shaft to CG of arm1 (from CAD)
l1 = 0.05113268; % m
% distance from arm2 pivot to CG of arm2 (from CAD)
l2 = 0.05447391; % m

%% Mass
% mass of arm1 itself
m1_arm = 21.24e-3; % kg
v1_arm = 0.00003267; % m^3
% mass of arm2 itself
m2_arm = 5.87e-3; % kg
v2_arm = 0.00000710; % m^3
% mass of 4 nuts and 4 bolts at the end of the arm2
m2_nuts_bolts = 9.93e-3; % kg
v2_nuts_bolts = 0.00000158; % m^3
% mass of all encoder parts
m1_encoder_parts = 15.8e-3; % kg
v1_encoder_parts = 0.00000490; % m^3
% mass of shaft on which arm2 pivots
m2_shaft = 4.63e-3; % kg
v2_shaft = 0.00000060; % m^3
% mass of 2 shaft collars
m2_shaft_collars = 9.65e-3; % kg
v2_shaft_collars = 0.00000140; % m^3
% mass of shaft coupler
m1_shaft_coupler = 10e-3; % kg
v1_shaft_coupler = 0.00000129; % m^3
% mass of 2 brass bushings
m1_bushing = 2.36e-3; % kg
v1_bushing = 0.00000036; % m^3

% total mass of arm1 at CG (from CAD)
m1 = 52.92863e-3; % kg
% total mass of arm2 at CG (from CAD)
m2 = 30.38172e-3; % kg

%% Inertia
% inertia tensor for arm1
J1xx = 0.01036427e-3; J1xy = 0.00034451e-3; J1xz = -0.01538809e-3; 
J1yy = 0.08669345e-3; J1yz = 0.00008695e-3;
J1zz = 0.08106350e-3; % kg-m^2

% inertia tensor for arm2
J2xx = 0.00798912e-3; J2xy = 0.00000031e-3; J2xz = -0.01628089e-3;
J2yy = 0.11481705e-3; J2yz = 0.00000007e-3;
J2zz = 0.10719887e-3;

%% Friction
% dynamic friction coefficient of motor
b1 = 0.0075423; 
% dynamic friction coefficient of arm2 pivot
b2 = 9e-5; % N-m-s

%% Electrical
% motor inductance
Lm = 0.0469406; % Henry
% motor resistance
Rm = 4; % Ohm
% motor back emf (also torque) constant
Km = 0.0989468; % N-m/A