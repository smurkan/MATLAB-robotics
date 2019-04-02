clear;
close all;
clc;
a = arduino('COM9', 'Uno', 'Libraries', 'Servo');
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
s = 'Tz(L)Rz(q1) Ry(q2) Tz(L1) Ry(q3) Tz(L2) Rz(q4) Ry(q5)Tz(L3)Rx(q6)';
dh = DHFactor(s);
dh.display
cmd = dh.command('5DOF arm')
robot = eval(cmd)

%big servo ~ 200 degrees (q1,q2,q3)
%small servo ~ 180 degrees (q4,q5)
robot.links(1,1).qlim = [-1.745 1.745];
robot.links(1,2).qlim = [-1.745 1.745];
robot.links(1,3).qlim = [-1.745 1.745];
robot.links(1,4).qlim = [-pi/2 pi/2];
robot.links(1,5).qlim = [-pi/2 pi/2];
robot.links(1,6).qlim = [-pi/2 pi/2];


%%
%q = positions;
%robot.teach()
%q=[0,0,0,0,0,0];
q=[-0.209439510239320,0.872664625997165,0.698131700797732,0,0,0];
T=robot.fkine(q);

robot.plot(q)
     A1 = toPWMRadBig( q(1));
     A2 = toPWMRadBig( -q(2));
     A3 = toPWMRadBig( q(3));
     A4 = toPWMRadSmall( q(4));
     A5 = toPWMRadSmall( -q(5));
     A6 =toPWMRadSmall( q(6));
    writePosition(s1, toPWMRadBig( q(1)));
    writePosition(s2, toPWMRadBig( -q(2)));
    writePosition(s3, toPWMRadBig( q(3)));
    writePosition(s4, toPWMRadSmall( q(4)));
    writePosition(s5, min(1,toPWMRadSmall( -q(5))));
    %writePosition(s6, toPWMRadSmall( q(6)));
    pause(1);
%robot.plot(q)
%%
T2=T;
T2.t(2) = T.t(2)+10;
%T2.t = [20 15 10]

%qs=robot.ikine(T2,'mask',[1 1 1 1 1 0]);
[qs err]= robot.ikcon(T2);
robot.plot(qs)
robot.fkine(qs)

A1 = toPWMRadBig( qs(1));
     A2 = toPWMRadBig( -qs(2));
     A3 = toPWMRadBig( qs(3));
     A4 = toPWMRadSmall( qs(4));
     A5 = toPWMRadSmall( -qs(5));
     A6 =toPWMRadSmall( qs(6));
    writePosition(s1, toPWMRadBig( qs(1)));
    writePosition(s2, toPWMRadBig( -qs(2)));
    writePosition(s3, toPWMRadBig( qs(3)));
    writePosition(s4, toPWMRadSmall( qs(4)));
    writePosition(s5, min(1,toPWMRadSmall( -qs(5))));
    %pause(1);

    %%
   T3=T2;
T3.t(2) = T2.t(2)+10;
T3.t(1) = T2.t(1)-6;

[qs2 err] =robot.ikcon(T3);
robot.plot(qs2)
robot.fkine(qs2)

A1 = toPWMRadBig( qs2(1));
     A2 = toPWMRadBig( -qs2(2));
     A3 = toPWMRadBig( qs2(3));
     A4 = toPWMRadSmall( qs2(4));
     A5 = toPWMRadSmall( -qs2(5));
     A6 =toPWMRadSmall( qs2(6));
    writePosition(s1, toPWMRadBig( qs2(1)));
    writePosition(s2, toPWMRadBig( -qs2(2)));
    writePosition(s3, toPWMRadBig( qs2(3)));
    writePosition(s4, toPWMRadSmall( qs2(4)));
    writePosition(s5, toPWMRadSmall( -qs2(5)));
    %writePosition(s6, toPWMRadSmall( qs2(6)));
    pause(1);
    
%%
T4 = T3;
T4.t(2) = T3.t(2)+10;
[qs3 err]=robot.ikcon(T4);
%qs3=robot.ikine(T4)
robot.plot(qs3)
robot.fkine(qs3)


A1 = toPWMRadBig( qs3(1));
     A2 = toPWMRadBig( -qs3(2));
     A3 = toPWMRadBig( qs3(3));
     A4 = toPWMRadSmall( qs3(4));
     A5 = toPWMRadSmall( -qs3(5));
     A6 =toPWMRadSmall( qs3(6));
    writePosition(s1, toPWMRadBig( qs3(1)));
    writePosition(s2, toPWMRadBig( -qs3(2)));
    writePosition(s3, toPWMRadBig(qs3(4)));
    writePosition(s4, toPWMRadSmall( qs3(4)));
    writePosition(s5, toPWMRadSmall( -qs3(5)));
    pause(1);
    

%%
% robot.teach()   %Used for setting up initial position 
% for k=1:inf
%     positions = robot.getpos()
%     writePosition(s1, toPWMRadBig( positions(1)));
%     writePosition(s2, toPWMRadBig( -positions(2)));
%     writePosition(s3, toPWMRadBig( positions(3)));
%     writePosition(s4, toPWMRadSmall( positions(4)));
%     max(0,toPWMRadSmall( -(positions(5)-0.2)))
%     writePosition(s5, min(1,toPWMRadSmall( -positions(5))));
%     writePosition(s6, toPWMRadSmall( positions(6)));
%     pause(1);
% end