%% Part 2
L1 = Link([0 0 1 0 0]);
L2 = Link([0 0 1 0 0]);
r2dp = SerialLink([L1 L2],'name','2DP');
r2dp.plot([0,0],'view','top','tilesize',3,'workspace',[-5 5 -3 3 -1 1]);
T = transl([2 0 0]);
p0 = [0 0 0 1];
pc = T * p0';
pc'
%% HM2
T=transl([-0.5 1.5 0]);
% for config elbow up
% q = r2dp.ikine(T,'mask',[1 1 0 0 0 0])
% for config elbow down
q = r2dp.ikine(T,'mask',[-1 1 0 0 0 0])
r2dp.plot(q,'view','top','tilesize',3,'workspace',[-5 5 -3 3 -1 1]);
figure
scatter(-0.5,1.5,'filled')
hold on
scatter(-0.83,0.56,'filled')
scatter(0,0,'filled')
grid on
hold off
legend('Link 2 joint: (-0.5,1.5)','Link 1 joint: (0.56,0.83)','base: (0,0)','location','northeast')
xlabel('x')
ylabel('y')
xlim([-2 2])
ylim([-1 2])
%% 
% The pose of the end-effector for q = [0 1] using FK
pe = r2dp.fkine([0 1])
% The joint angles that produces this pose using IFK
q = r2dp.ikine(pe, 'mask', [1 1 0 0 0 0])
% The Cartesian position of the end-effector
pc = pe.T * [0 0 0 1]';
pc'

%% HM3
%converts homogeneous transform to rotation and translation
[R,t] = tr2rt(pe)
tr2angvec(R)
trplot(T,'axis',[0 4 -1 3 -1 1],'frame','1')
hold on
trplot(pe,'frame','2')
%% Part 3
% Generate the joint space
% q1 in [0 pi/2]
% q2 in [0 pi]
r2dp.links(1,1).qlim = [0 pi/2];
r2dp.links(1,2).qlim = [0 pi];
q=[];
for i=0:pi/20:pi
    qt = jtraj([0 i], [pi/2 i], 20);
    q=[q;qt];
end
r2dp.plot(q,'view','top','tilesize',3,'workspace',[-5 5 -3 3 -1 1],'trail',{'r','linewidth',2})
%%
T0 = transl([2 0 0]);
T1 = transl([0 1 0]);
Ts = ctraj(T0, T1, 20);
qs = r2dp.ikine(Ts, 'mask', [1 1 0 0 0 0]);
%qs = r2dp.ikine(Ts);
r2dp.plot(qs,'view','top','tilesize',3,'workspace',[-5 5 -3 3 -1 1],'trail',{'b','linewidth',10})
dx = linspace(1,20,20);
figure;
plot(dx,qs)
%% HM4
%[q,qd,qdd] =jtraj([0 r2dp.ikine(T0,'mask',[2 0 0 0 0 0])],[0 r2dp.ikine(T1,'mask',[0 1 0 0 0 0])],20);
qs2 = r2dp.ikcon(Ts);
r2dp.plot(qs2,'view','top','tilesize',3,'workspace',[-5 5 -3 3 -1 1],'trail',{'b','linewidth',10})
figure;
plot(dx,qs2)
legend('q1','q2','location','northwest')
xlabel('Trajectory steps')
ylabel('Joint Angles')
xlim([0 20])
ylim([-1 2.5])
%% Part 4
r2dp.gravity = [0;9.81;0]

r2dp.links(1,1).m = 50;
r2dp.links(1,1).r = [-0.5,0,0];
r2dp.links(1,1).I = [0,0,0;0,0,0;0,0,10];
r2dp.links(1,1).G = 1;
r2dp.links(1,1).Jm = 0;
r2dp.links(1,1).dyn

r2dp.links(1,2).m = 50;
r2dp.links(1,2).r = [-0.5,0,0];
r2dp.links(1,2).I = [0,0,0;0,0,0;0,0,10];
r2dp.links(1,2).G = 1;
r2dp.links(1,2).Jm = 0;
r2dp.links(1,2).dyn

% Generate a trajectory between two positions
[q,qd,qdd] = jtraj([pi/4 0],[pi/4 pi/2],20);
%r2dp.plot(q,'view','top','tilesize',3,'workspace',[-5 5 -3 3 -1 1],'trail',{'b','linewidth',10})
r2dp.plot([pi/4 pi/3],'jvec')
figure
% Calculate the joints torques required to achieve
% the generated positions
Q = r2dp.rne(q,qd,qdd);

% Plot the torques and the joints angles
subplot(211)
plot(Q(:,1))
hold on
plot(Q(:,2),'r')
ylabel('Joints Torques')
legend('J1','J2')
subplot(212)
plot(q(:,1))
hold on
plot(q(:,2),'r')

ylabel('Joints Angles')
legend('q1','q2')
xlabel('Trajectory Steps')
% subplot(314)
% plot(qdd(:,1))
% hold on
% plot(qdd(:,2),'r')
% ylabel('Joints acc')
% legend('q1','q2')
% xlabel('Trajectory Steps')
%% HM 6
r2dp.links(1,2).m=50;
for i=[50 100 150]
    r2dp.links(1,1).m = i;
    Q = r2dp.rne(q,qd,qdd);
    plot(Q(:,1))
    hold on
    ylabel('Joints Torques')
    xlabel('Trajectory Steps')
end
plot(Q(:,2),'r')
legend('J1, m1=50','J1, m1=100','J1, m1=150','J2')

%%
r2dp.links(1,1).m=50;
for i=[50 100 150]
    r2dp.links(1,2).m = i;
    Q = r2dp.rne(q,qd,qdd);
    plot(Q(:,1))
    hold on
    plot(Q(:,2))
    ylabel('Joints Torques')
    xlabel('Trajectory Steps')
end
%plot(Q(:,2),'r')
legend('J1, m2=50','J2, m2=50','J1, m2=100','J2, m2=100','J1, m2=150','J2, m2=150')
%% HM7
Grav = r2dp.gravload(q);
plot(Q(:,1))
hold on
plot(Q(:,2),'r')
plot(Grav(:,1))
plot(Grav(:,2))
ylabel('Joints Torques')
xlabel('Trajectory Steps')
legend('J1','J2','G(q1)','G(q2)')
%%
