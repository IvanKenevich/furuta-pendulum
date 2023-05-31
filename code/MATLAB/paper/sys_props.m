%% General
g = 9.81; % m/s^2

%% Length
% length of arm1 from motor shaft to pivot of arm2
L1 = 0.199; % m
% length of arm2 from pivot to the tip
L2 = 0.144; % m
% distance from motor shaft to CG of arm1
l1 = 0.064888; % m
% distance from arm2 pivot to CG of arm2
l2 = 0.020327; % m

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

% total mass of arm1 at CG
m1 = 48.022393e-3; % kg
% total mass of arm2 at CG
m2 = 19.588257e-3; % kg

%% Inertia
% inertia tensor for arm1
J1xx = 0.010807e-3; J1xy = -0.000018e-3; J1xz = -0.011989e-3; % kg-m^2
J1yy = 0.054190e-3; J1yz = 0;
J1zz = 0.048125e-3;

% inertia tensor for arm2
J2xx = 0.003293e-3; J2xy = 0; J2xz = -0.005240e-3;
J2yy = 0.034100e-3; J2yz = 0;
J2zz = 0.031078e-3;

%% Friction
% dynamic friction coefficient of motor
b1 = 0.0075423; 
% dynamic friction coefficient of arm2 pivot
b2 = 2.8e-4; % N-m-s

%% Electrical
% motor inductance
Lm = 0.0469406; % Henry
% motor resistance
Rm = 4; % Ohm
% motor back emf (also torque) constant
Km = 0.0989468; % N-m/A