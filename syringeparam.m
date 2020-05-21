%% Syringe Control Parameters

% Design Parameters
pos = 5; % percent overshoot
ts = 1;  % settling time
 
z = (-log(pos/100))/(sqrt(pi^2+log(pos/100)^2));% damping coefficient
wn = 4/(z*ts);
sig = wn*z;
wd = wn*sqrt(1-z^2);
 
s1 = sig-i*wd;
s2 = sig+i*wd;

% Plot Pole Allocation
rlocus(tf(1/((s+s1)*(s+s2)))); % place poles on root locus
sgrid(z,0); % damping angle
sigrid(sig);

b = 0.0035; % motor viscous friction constant
k = 0.0274; % motor torque constant
m1 = 19.5;  % weight of the push block in grams
m2 = 10;    % weight of water in grams

A = [0 1 0 0
    -k/m1 -b/m1 k/m1 b/m1
    0 0 0 1
    k/m2 b/m2 -k/m2 -b/m2];
B = [0; 0; 0; 1/m2];
C = [0 0 0 1];
D = 0;
Gsyx = ss(A,B,C,D); % state space equation

% Transfer Function
[num,den] = ss2tf(A,B,C,D);
Gsy = tf(num,den)

% Check Controllability
Pc = ctrb(A,B);
control = length(A)-rank(Pc);

%% Motor Plant Transfer Function
 
num = 150;          % numerator coefficients
den = [0.412 1];    % denominator coefficients
Gm = tf(num,den);
 
figure;
subplot(211)
rlocus(Gm);
title('Root Locus - Linear Model of DC Gear Motor');
grid on
 
subplot(212)
step(Gm)
title('Ideal Step Response of DC Gear Motor');
grid on

%% Push Block Test

t1 = [0:5:145];            % 5 sec interval
mm1 = [0 4 7 10 13 17 20 23 26 29 32 35 39 42 45 49 52 55 59 61.5 65 68 71.5 74 78 81 84.5 87 91 95];
 
t2 = [0:10:150];           % 10 sec interval
mm2 = [0 6 11 19 24.5 31 37 43 50 55 62 69 75 81 87.5 93];
 
t3 = [0 30 60 90 120 150]; % 30 sec interval
mm3 = [0 20 37 55 73 92];

figure;
plot(t1,mm1);
hold on
plot(t2,mm2);
plot(t3,mm3);
hold off
grid on

title('Push Block Position Test - 130 RPM Step');
xlabel('Time (sec)');
ylabel('Displacement (mm)');
legend('5 second interval','10 second interval','30 second interval');

%% Callback Buttons

% INITIALIZE   
    % Initialize Motor Driver Pins
    rpm = 0;
    pin6 = 1;
    pin7 = 1;

    % Initializing Sampling Time and Signal Filters
    Ts = 0.02;              % model sample time in seconds
    GR = 1/3591.84;         % gear ratio 75:1

    filt_const = 5*Ts;      % time constant of the first-order filter

    % Motor PI Controller Values
    Kp = 0.0153;
    Ki = 0.0922;

% INFUSE
    % Set Direction: Forward
    if pin6 == 1
    pin6 = 0;
    end
    if pin7 == 0
    pin7 = 1;
    end

    % User Input
    flow = input('ENTER INFUSION RATE (ml/s): ');

    % Convert to RPM
    rpm = flow*1083;
    mm_s = rpm/130*.6;  % mm per second
    runtime = 55/mm_s;  % relative runtime needed to infuse maximum

% RESET
    % Set Direction: Backwards
    if pin6 == 0
    pin6 = 1;
    end
    if pin7 == 1
    pin7 = 0;
    end

