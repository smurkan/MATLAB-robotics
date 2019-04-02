
clc;
clear;
close all;

sample_rate = 10; %10hz

connector on robotik;

m = mobiledev;


m.AccelerationSensorEnabled = 1;
m.AngularVelocitySensorEnabled = 1;
m.OrientationSensorEnabled = 1;

m.Logging = 1;
pause(5);

m.Logging = 0;

[accel0, timestamp0] = accellog(m);
[orientation, timestamp1] = orientlog(m);
plot(timestamp0, accel0);
legend('x','y','z');



assert(length(accel0) <= length(orientation))
% orientation(i,1) = azimuth
% orientation(i,2) = pitch
% orientation(i,3) = roll
accelModified = zeros(length(accel0), 3);
for i = 1:length(accel0)
    clc;
    cyaw = cosd( orientation(i,1));
    syaw = sind( orientation(i,1));
    cpitch = cosd( orientation(i,2));
    spitch = sind( orientation(i,2));
    croll = cosd( orientation(i,3));
    sroll = sind( orientation(i,3));
    accelModified(i,1) = accel0(i,1)*cyaw*cpitch +  accel0(i,2)*(-syaw*croll + cyaw*spitch*sroll)+ accel0(i,3)*(syaw*sroll + cyaw*spitch*croll) - 9.9*sind(orientation(i,2))*cosd(orientation(i,3));
    accelModified(i,2) = accel0(i,1)*syaw*cpitch + accel0(i,2)*(cyaw*croll + syaw*spitch*sroll) + accel0(i,3)*(-cyaw*sroll + syaw*spitch*croll) - 9.9*sind(orientation(i,2))*sind(orientation(i,3));
    accelModified(i,3) = -accel0(i,1)*spitch + accel0(i,2)*cpitch*sroll + accel0(i,3)*cpitch*croll  - 9.9*cosd(orientation(i,2));
 
    
   
end

%FUSE = imufilter('SampleRate',10,'GyroscopeNoise',1e-6);

%pause(1)
%m.Logging = 0;
%accellog(m)
%angvellog(m)

%[accelReadings,gyroReadings] = imu(accellog(m),angvellog(m))

vel = cumtrapz(timestamp0, accelModified)
%dist = cumtrapz(timestamp0, vel)
%vel1 = trapz(k,a)




