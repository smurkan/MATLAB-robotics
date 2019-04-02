clear;
close all;
clc;
a = arduino('COM7', 'Uno', 'Libraries', 'Servo');
s1 = servo(a, 'D4', 'MinPulseDuration', 5.44e-04, 'MaxPulseDuration', 2.40e-03);
s2 = servo(a, 'D5', 'MinPulseDuration', 5.44e-04, 'MaxPulseDuration', 2.40e-03);
s3 = servo(a, 'D6', 'MinPulseDuration', 5.44e-04, 'MaxPulseDuration', 2.40e-03);
s4 = servo(a, 'D7', 'MinPulseDuration', 5.44e-04, 'MaxPulseDuration', 2.40e-03);
s5 = servo(a, 'D8', 'MinPulseDuration', 5.44e-04, 'MaxPulseDuration', 2.40e-03);
s6 = servo(a, 'D9', 'MinPulseDuration', 5.44e-04, 'MaxPulseDuration', 2.40e-03);

toPWM = @(x) x/100*0.5 + 0.5;
toPWMRadBig = @(x) x/(pi*+100/180 )*0.5 + 0.5; 
toPWMRadSmall = @(x) x/(pi*+90/180 )*0.5 + 0.5;
L = 6;
L1=12;
L2=12;
L3=12;
s = 'Tz(L) Rz(q1) Rx(q2) Tz(L1) Rx(q3) Tz(L2) Rz(q4) Rx(q5) Tz(L3) Rx(q6)';
dh = DHFactor(s);
dh.display
cmd = dh.command('5DOF_arm')
robot = eval(cmd)

robot.links(1,1).qlim = [-3.5/2 3.5/2];
robot.links(1,2).qlim = [-3.5/2 3.5/2];
robot.links(1,3).qlim = [-3.5/2 3.5/2];
robot.links(1,4).qlim = [-pi/2 pi/2];
robot.links(1,5).qlim = [-pi/2 pi/2];
robot.links(1,6).qlim = [-pi/2 pi/2];
%%
%robot.plot([0,0,0,0,0],'view','top','tilesize',3,'workspace',[-5 5 -3 3 -1 5]);
robot.teach()

for k=1:inf
    positions = robot.getpos()
    writePosition(s1, toPWMRadBig( positions(1)));
    writePosition(s2, toPWMRadBig( -positions(2)));
    writePosition(s3, toPWMRadBig( positions(3)));
    writePosition(s4, toPWMRadSmall( positions(4)));
    max(0,toPWMRadSmall( -(positions(5)-0.2)))
    writePosition(s5, min(1,toPWMRadSmall( -positions(5))));
    writePosition(s6, toPWMRadSmall( positions(6)));
    pause(1);
end