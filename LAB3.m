clear;
close all;
clc;
a = arduino('COM6', 'Uno', 'Libraries', 'Servo');
s1 = servo(a, 'D4', 'MinPulseDuration', 5.44e-04, 'MaxPulseDuration', 2.40e-03);
s2 = servo(a, 'D5', 'MinPulseDuration', 5.44e-04, 'MaxPulseDuration', 2.40e-03);
s3 = servo(a, 'D6', 'MinPulseDuration', 5.44e-04, 'MaxPulseDuration', 2.40e-03);
s4 = servo(a, 'D7', 'MinPulseDuration', 5.44e-04, 'MaxPulseDuration', 2.40e-03);
s5 = servo(a, 'D8', 'MinPulseDuration', 5.44e-04, 'MaxPulseDuration', 2.40e-03);
s6 = servo(a, 'D9', 'MinPulseDuration', 5.44e-04, 'MaxPulseDuration', 2.40e-03);

toPWM = @(x) x/100*0.5 + 0.5;
toPWMRadBig = @(x) x/(pi*+100/180 )*0.5 + 0.5; 
toPWMRadSmall = @(x) x/(pi*+90/180 )*0.5 + 0.5;

L1=12;
L2=12;
L3=14;
L=6;
s = 'Tz(L)Rz(q1) Rx(q2) Tz(L1) Rx(q3) Tz(L2) Rz(q4) Rx(q5)Tz(L3)Rx(q6)';
dh = DHFactor(s);
dh.display
cmd = dh.command('5DOF_arm')
robot = eval(cmd)

%big servo ~ 200 degrees (q1,q2,q3)
%small servo ~ 180 degrees (q4,q5)
robot.links(1,1).qlim = [-3.45/2 3.45/2];
robot.links(1,2).qlim = [-3.45/2 3.45/2];
robot.links(1,3).qlim = [-3.45/2 3.45/2];
robot.links(1,4).qlim = [-pi/2 pi/2];
robot.links(1,5).qlim = [-pi/2 pi/2];
%%
%robot.plot([0,0,0,0,0,0],'view','top','tilesize',5,'workspace',[-30 30 -30 30 -10 60]);
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
%%
q = positions;
%q=[0,0.977384381116825,1.256637061435917,0,-0.900590941226625,0];
%q=[0.39375,0.28,1.715,0,0.785398163397448,0];
T=robot.fkine(q);


    writePosition(s1, toPWMRadBig( q(1)));
    writePosition(s2, toPWMRadBig( -q(2)));
    writePosition(s3, toPWMRadBig( q(3)));
    writePosition(s4, toPWMRadSmall( q(4)));
    max(0,toPWMRadSmall( -(q(5)-0.2)))
    writePosition(s5, min(1,toPWMRadSmall( -q(5))));
    %writePosition(s6, toPWMRadSmall( q(6)));
    pause(1);
%robot.plot(q)
%%
T2=T
%T2.t(2) = T.t(2)-5;
T2.t(2) = T.t(2)-10;

%qs=robot.ikine(T2,'mask',[1 1 1 1 1 0]);
qs = robot.ikcon(T2,q);
robot.plot(qs)

writePosition(s1, toPWMRadBig( qs(1)));
    writePosition(s2, toPWMRadBig( -qs(2)));
    writePosition(s3, toPWMRadBig( qs(3)));
    writePosition(s4, toPWMRadSmall( qs(4)));
    writePosition(s5, min(1,toPWMRadSmall( -qs(5))));
    %writePosition(s6, toPWMRadSmall( qs(6)));
    pause(1);

    %%
   T3=T2;
T3.t(2) = T2.t(2)-10;
qs2=robot.ikcon(T3,qs);
%robot.plot(qs)

    writePosition(s1, toPWMRadBig( qs2(1)));
    writePosition(s2, toPWMRadBig( -qs2(2)));
    writePosition(s3, toPWMRadBig( qs2(3)));
    writePosition(s4, toPWMRadSmall( qs2(4)));
    max(0,toPWMRadSmall( -(qs2(5)-0.2)))
    writePosition(s5, min(1,toPWMRadSmall( -qs2(5))));
    %writePosition(s6, toPWMRadSmall( qs2(6)));
    pause(1);
    
%%
T4=T3;
T4.t(1) = T4.t(1)+5
qs3=robot.ikcon(T4)
%qs3=robot.ikine(T4)
%robot.plot(qs)


    writePosition(s1, toPWMRadBig( qs3(1)));
    writePosition(s2, toPWMRadBig( -qs3(2)));
    writePosition(s3, toPWMRadBig( qs3(3)));
    writePosition(s4, toPWMRadSmall( qs3(4)));
    max(0,toPWMRadSmall( -(qs3(5)-0.2)))
    writePosition(s5, min(1,toPWMRadSmall( -qs3(5))));
    %writePosition(s6, toPWMRadSmall( qs3(6)));
    pause(1);
    
%%
T5=T4;
T5.t(2) = T5.t(2)+5
qs4=robot.ikine(T5)
%robot.plot(qs)

    writePosition(s1, toPWMRadBig( qs4(1)));
    writePosition(s2, toPWMRadBig( -qs4(2)));
    writePosition(s3, toPWMRadBig( qs4(3)));
    writePosition(s4, toPWMRadSmall( qs4(4)));
    max(0,toPWMRadSmall( -(qs4(5)-0.2)))
    writePosition(s5, min(1,toPWMRadSmall( -qs4(5))));
    %writePosition(s6, toPWMRadSmall( qs3(6)));
    pause(1);
    
    
%%
    writePosition(s1, toPWMRadBig( q(1)));
    writePosition(s2, toPWMRadBig( -q(2)));
    writePosition(s3, toPWMRadBig( q(3)));
    writePosition(s4, toPWMRadSmall( q(4)));
    writePosition(s5, min(1,toPWMRadSmall( -q(5))));
    pause(1);
%%    
        writePosition(s1, toPWMRadBig( q2(1)));
    writePosition(s2, toPWMRadBig( -q2(2)));
    writePosition(s3, toPWMRadBig( q2(3)));
    writePosition(s4, toPWMRadSmall( q2(4)));
    writePosition(s5, min(1,toPWMRadSmall( -q2(5))));
    pause(1);
  %%  
        writePosition(s1, toPWMRadBig( q3(1)));
    writePosition(s2, toPWMRadBig( -q3(2)));
    writePosition(s3, toPWMRadBig( q3(3)));
    writePosition(s4, toPWMRadSmall( q3(4)));
    writePosition(s5, min(1,toPWMRadSmall( -q3(5))));
    pause(1);
      %%  
        writePosition(s1, toPWMRadBig( q4(1)));
    writePosition(s2, toPWMRadBig( -q4(2)));
    writePosition(s3, toPWMRadBig( q4(3)));
    writePosition(s4, toPWMRadSmall( q4(4)));
    writePosition(s5, min(1,toPWMRadSmall( -q4(5))));
    pause(1);

