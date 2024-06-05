%% FK BOGOBOT 3
rmpath('C:\Program Files\MATLAB\R2020b\toolbox\phased\phased');
syms q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 q11 q12 q13 q14 q15 q16 q17 q18 
syms L1 L2 L3 L4 L5 L6 L7

%% RIGHT LEG

TCOM1 = [rotz(-q12),[0;0;0]; 0 0 0 1];
T12 = [rotx(-q11),[0;0;0]; 0 0 0 1];
T23 = [roty(q10),[0;0;0]; 0 0 0 1];
T34 = [roty(q9),[0;0;-L3]; 0 0 0 1];
T45 = [roty(-q8),[0;0;-L4]; 0 0 0 1];
T56 = [rotx(q7),[0;0;0]; 0 0 0 1];


% Cinematica directa
TCOM6 = simplify(TCOM1*T12*T23*T34*T45*T56);

% Cinematica inversa
T16 = simplify(T12*T23*T34*T45*T56);
T16(:,4)

Res1 = simplify(expand(T16(1,4)^2) + expand(T16(2,4)^2) + expand(T16(3,4)^2));

%       se analiza de 2-6 
syms Px Py Pz
Td = [eye(3), [Px; Py; Pz]; 0 0 0 1];

T26 = simplify(simplify(T23*T34*T45)*T56);
Res2 = simplify(inv(T12)*Td);
[Res2(:,4) == expand(T26(:,4))]

%% LEFT LEG

TCOM1 = [rotz(-q6),[0;0;0]; 0 0 0 1];
T12 = [rotx(-q5),[0;0;0]; 0 0 0 1];
T23 = [roty(-q4),[0;0;0]; 0 0 0 1];
T34 = [roty(-q3),[0;0;-L3]; 0 0 0 1];
T45 = [roty(q2),[0;0;-L4]; 0 0 0 1];
T56 = [rotx(q1),[0;0;0]; 0 0 0 1];

% Cinematica directa
TCOM6 = simplify(TCOM1*T12*T23*T34*T45*T56);

% Cinematica inversa
T16 = simplify(T12*T23*T34*T45*T56)

%       se despeja q4 por pitagoras 
Res1 = simplify(expand(T16(1,4)^2) + expand(T16(2,4)^2) + expand(T16(3,4)^2))

%       se analiza de 2-6 
syms Px Py Pz
Td = [eye(3), [Px; Py; Pz]; 0 0 0 1];

T26 = simplify(simplify(T23*T34*T45)*T56);
Res2 = simplify(inv(T12)*Td);
[Res2(:,4) == expand(T26(:,4))]

%% Pruebas de Cinematica Inversa RIGHT LEG
clear all; clc;
alpha = 0; Px_alpha = 0; Py_alpha = -2; Pz_alpha = 0;
L3 = 18.5; L4 = 18.5;

% q12
q12 = alpha; c12 = cos(q12); s12 = sin(q12);
Px = Px_alpha*c12 - Px_alpha*s12;
Py = Py_alpha*s12 + Py_alpha*c12;
Pz = Pz_alpha;

P = [Px, Py, Pz]
%q9
c9 = (Px^2 + Py^2 + Pz^2 - L3^2 - L4^2) / (2*L3*L4);
c9 = max(-1,min(c9,1));
q9 = acos(c9); % + - Nota: debe ir positivo 
s9 = sin(q9);

% q11
q11 = atan2(-Py, -Pz); % o atan2(Py, Pz)
c11 = cos(q11); s11 = sin(q11);

% q10
q10 = atan2(-Px, -Pz*c11 -Py*s11) - atan2(L4*s9, L3 + L4*c9);
s10 = sin(q10); c10 = cos(q10);
q10 = atan2(s10, c10);

% q8
q8 = q9 + q10;

% q7
q7 = q11;

% PROBAR EN FK (ROBOT)
TCOM1 = [rotz(-q12),[0;0;0]; 0 0 0 1];
T12 = [rotx(-q11),[0;0;0]; 0 0 0 1];
T23 = [roty(q10),[0;0;0]; 0 0 0 1];
T34 = [roty(q9),[0;0;-L3]; 0 0 0 1];
T45 = [roty(-q8),[0;0;-L4]; 0 0 0 1];
T56 = [rotx(q7),[0;0;0]; 0 0 0 1];
TCOM6 = TCOM1*T12*T23*T34*T45*T56;

q = [q12, q11, q10, q9, q8, q7]
TCOM6(:,4)

%% Pruebas de Cinematica Inversa LEFT LEG
clear all; clc;
alpha = 0; Px_alpha = 0; Py_alpha = 0.01; Pz_alpha = -31;
L3 = 18.5; L4 = 18.5;

% q6
q6 = alpha; c6 = cos(q6); s6 = sin(q6);
Px = Px_alpha*c6 - Px_alpha*s6;
Py = Py_alpha*s6 + Py_alpha*c6;
Pz = Pz_alpha;

P = [Px, Py, Pz]
%q3
c3 = (Px^2 + Py^2 + Pz^2 - L3^2 - L4^2) / (2*L3*L4);
c3 = max(-1,min(c3,1));
q3 = - acos(c3); % + -
s3 = sin(q3);

% q5
q5 = atan2(-Py, -Pz); % o atan2(Py, Pz)
c5 = cos(q5); s5 = sin(q5);

% q4
q4 = atan2(Px, -Pz*c5 -Py*s5) - atan2(L4*s3, L3 + L4*c3);
s4 = sin(q4); c4 = cos(q4);
q4 = atan2(s4, c4);

% q2
q2 = + q3 + q4;

% q1
q1 = q5;

% PROBAR EN FK (ROBOT)
TCOM1 = [rotz(-q6),[0;0;0]; 0 0 0 1];
T12 = [rotx(-q5),[0;0;0]; 0 0 0 1];
T23 = [roty(-q4),[0;0;0]; 0 0 0 1];
T34 = [roty(-q3),[0;0;-L3]; 0 0 0 1];
T45 = [roty(q2),[0;0;-L4]; 0 0 0 1];
T56 = [rotx(q1),[0;0;0]; 0 0 0 1];
TCOM6 = TCOM1*T12*T23*T34*T45*T56;

q = [q6, q5, q4, q3, q2, q1]
TCOM6(:,4)

%% IK RIGHT ARM
syms L2 L3 q1 q3 q5
rmpath('C:\Program Files\MATLAB\R2020b\toolbox\phased\phased');

TCOM1 = [roty(-q1),[0;0;0]; 0 0 0 1];
T12 = [rotx(-q3),[0;0;0]; 0 0 0 1];
T23 = [rotx(q5),[0;-L2;0]; 0 0 0 1];
T34 = [eye(3),[0;-L3;0]; 0 0 0 1];

TCOM4 = simplify(TCOM1*T12*T23*T34)

Res1 = simplify(expand(TCOM4(1,4)^2) + expand(TCOM4(2,4)^2) + expand(TCOM4(3,4)^2))
Res2 = collect(- L2*cos(q3) - L3*cos(q3)*cos(q5) - L3*sin(q3)*sin(q5),[cos(q3), sin(q3)])


%% IK LEFT ARM
syms L2 L3 q2 q4 q6
rmpath('C:\Program Files\MATLAB\R2020b\toolbox\phased\phased');

TCOM1 = [roty(q2),[0;0;0]; 0 0 0 1];
T12 = [rotx(-q4),[0;0;0]; 0 0 0 1];
T23 = [rotx(q6),[0;0;L2]; 0 0 0 1];
T34 = [eye(3),[0;0;L3]; 0 0 0 1];

TCOM4 = simplify(TCOM1*T12*T23*T34);

Res1 = simplify(expand(TCOM4(1,4)^2) + expand(TCOM4(2,4)^2) + expand(TCOM4(3,4)^2));
Res2 = collect((L2*sin(q4) - L3*cos(q4)*sin(q6) + L3*cos(q6)*sin(q4)),[cos(q4), sin(q4)])





