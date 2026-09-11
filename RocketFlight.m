%% ANSYS Drag Studies Applied to Rocket Trajectory Calcuations + Flight Computer Data Viewer

clear
clc
close all

% Imports drag results from ANSYS for rocket, approximates altitude up to
% the time of apogee. Interpolates drag and thrust curves to generate
% results

NSamples = 100;
dt = 0.001;

%% Inputs
rho = 1.225; %kg/m^3
Area = 0.00159; %m^2
m = 0.328; %kg (avg weight assuming constant propellant burn)
g = 9.81; %m/s^2

%% ANSYS Drag Data

rawdata = importdata('DragVsVelocity.csv');
data = rawdata.data; 
datasize = size(data);
Velocity = data(:, 1)';
% Cd = data(:, 2)' Column has invalid numbers due to wrong ref area in
% ANSYS, Manually calculate instead
Drag = data(:, 3)';

Cd = Drag ./ (0.5*rho.*Velocity.^2*Area);

xq = linspace(0, 35, NSamples);

Cdinterp = interp1(Velocity, Cd, xq, 'spline');
% Ensures interpolated drag at zero velocity doesn't go negative
Drag = [0, Drag]; Velocity = [0, Velocity]; 
Draginterp = interp1(Velocity, Drag, xq, 'spline');

%% Estes D-12 Thrust Curve Data

d12rawdata = importdata("Estes_D12.csv");
d12data = d12rawdata.data;
Time = d12data(:,1)';
Thrust = d12data(:,2)';
BurnTimeCurve = Time(end);

tq = Time(1):dt:Time(end);

Thrust_interp = interp1(Time, Thrust, tq);

%% Apogee Calculation

% Preallocate with sufficient space for flight after burn end
t = zeros(1, length(tq)*100);
AltitudeCalc = zeros(1, length(tq)*100);
VelocityCalc = zeros(1, length(tq)*100);
LaunchStatus = false;
n = 2;

% Simulates flight assuming fixed average mass, constant atmospheric pressure
% Ends at apogee
while true
    if n > length(Thrust_interp)
        T = 0;
    else
        T = Thrust_interp(n-1);
    end
    D = interp1(Velocity, Drag, VelocityCalc(n-1), 'spline', 'extrap');
    F = T - D - m*g;
    if ~LaunchStatus && F < 0
        F = 0;
    else
        LaunchStatus = true;
    end
    da = F/m;
    VelocityCalc(n) = VelocityCalc(n-1) + da*dt;
    AltitudeCalc(n) = AltitudeCalc(n-1) + VelocityCalc(n-1)*dt;
    t(n) = t(n-1) + dt;
    if VelocityCalc(n) < 0
        break
    end
    n = n + 1;
end

% Trim Data up to apogee event
AltitudeCalc = AltitudeCalc(1:n);
VelocityCalc = VelocityCalc(1:n);
t = t(1:n);
ApogeeCalc = AltitudeCalc(end);
ApogeetimeCalc = t(end);

%% Plots

figure("Name", "Drag")
plot(xq, Draginterp, '--', Velocity, Drag, '*r')
xlabel("Velocity (m/s)")
ylabel("Drag (N)")
title("Interpolated Drag vs. Velocity")
legend("Interpolated Drag ", "Original Drag Points")

figure("Name", "Cd")
plot(xq, Cdinterp, '--', Velocity(2:end), Cd, "*r")
xlabel("Velocity (m/s)")
ylabel("Drag Coefficient (Cd)")
title("Interpolated Drag Coefficient vs. Velocity")
legend("Interpolated Drag Coefficient", "Original Cd Points")

figure("Name", "Thrust")
plot(Time, Thrust, 'b', tq, Thrust_interp, '--r')
xlabel("Time (s)")
ylabel("Thrust (N)")
title("Thrust vs. Time")
legend("Original Thrust", "Interpolated Thrust")

figure("Name", "Altitude")
plot(t, AltitudeCalc, '-')
xlabel("Time (s)")
ylabel("Altitude (m)")
title("Altitude vs. Time")

figure("Name","Velocity")
plot(t, VelocityCalc)
xlabel("Time (s)")
ylabel("Velocity (m/s)")
title("Rocket Velocity vs. Time")
grid on

%% Import Flight Computer Data

fid = fopen("FLIGHT00.csv");

C = textscan(fid,...
    '%f%f%f%f%f%f%f%f%f%f%f%f%f',...
    'Delimiter', ',', ...
    'HeaderLines', 1);

fclose(fid);

% Convert cell array to matrix
M = cell2mat(C);

%% Determine Time of Launch

altitude = M(:,6);
[apogeeheight, apogeeidx] = max(altitude);
launchidx = apogeeidx;
while altitude(launchidx) > 0;
    launchidx = launchidx - 1;
end

landidx = find(altitude(apogeeidx:end) < 0, 1) + apogeeidx - 1;

%% Assign variables

timestamp = M(launchidx-104:landidx+104,1) - M(launchidx-104, 1);

qw = M(launchidx-104:landidx+104,2);
qx = M(launchidx-104:landidx+104,3);
qy = M(launchidx-104:landidx+104,4);
qz = M(launchidx-104:landidx+104,5);

altitude = M(launchidx-104:landidx+104,6);

ax = M(launchidx-104:landidx+104,7);
ay = M(launchidx-104:landidx+104,8);
az = M(launchidx-104:landidx+104,9);

gx = M(launchidx-104:landidx+104,10);
gy = M(launchidx-104:landidx+104,11);
gz = M(launchidx-104:landidx+104,12);

temp = M(launchidx-104:landidx+104,13);

%% Convert timestamp to seconds
time = timestamp / 1e6;

%% Derived Quantities

accelMag = sqrt(ax.^2 + ay.^2 + az.^2);

gyroMag = sqrt(gx.^2 + gy.^2 + gz.^2);

%% Quaternion Math

earthAccel = zeros(length(ax),3);

for i = 1:length(ax)

    q = [qw(i), qx(i), qy(i), qz(i)];
    q = q / norm(q);

    % Rotation matrix from body frame to Earth frame
    R = quat2rotm(q);

    % Accelerometer measurement in Body frame
    aBody = [ax(i); ay(i); az(i)];

    % Rotate acceleration into Earth-frame
    aEarth = R * aBody;

    earthAccel(i,:) = aEarth';

end

% Earth-frame acceleration
axEarth = earthAccel(:,1);
ayEarth = earthAccel(:,2);
azEarth = earthAccel(:,3);

verticalAcceleration = azEarth - 9.807;

verticalVelocity = cumtrapz(time, verticalAcceleration);

%% Altitude

figure()
hold on
plot(time,altitude,'LineWidth',1.5)
xline(time(105), '--r', 'Launch')
xline(time(apogeeidx-launchidx+104), '--b', 'Apogee')
xline(time(landidx-launchidx+104), '--y', 'Touchdown')
grid on
xlabel('Time (s)')
ylabel('Altitude (m)')
title('Altitude')

%% Smoothed Altitude

% figure()
% plot(time,altitude,'g')
% hold on
% plot(time,smoothAltitude,'b-','LineWidth',2)
% xline(time(105), '--r', 'Launch')
% xline(time(apogeeidx-launchidx), '--b', 'Apogee')
% xline(time(landidx-launchidx), '--y', 'Touchdown')
% grid on
% legend('Raw','Smoothed')
% xlabel('Time (s)')
% ylabel('Altitude (m)')
% title('Altitude')

%% Accelerometer

figure()
plot(time,ax,'LineWidth',1.5)
hold on
plot(time,ay,'LineWidth',1.5)
plot(time,az,'LineWidth',1.5)
xline(time(105), '--r', 'Launch')
xline(time(apogeeidx-launchidx+104), '--b', 'Apogee')
xline(time(landidx-launchidx+104), '--y', 'Touchdown')
grid on
legend('Ax','Ay','Az')
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
title('Accelerometer')

%% Gyroscope

figure()
plot(time,gx,'LineWidth',1.5)
hold on
plot(time,gy,'LineWidth',1.5)
plot(time,gz,'LineWidth',1.5)
xline(time(105), '--r', 'Launch')
xline(time(apogeeidx-launchidx+104), '--b', 'Apogee')
xline(time(landidx-launchidx+104), '--y', 'Touchdown')
grid on
legend('Gx','Gy','Gz')
xlabel('Time (s)')
ylabel('Angular Velocity (deg/s)')
title('Gyroscope')

%% Quaternion

figure()
plot(time,qw,'LineWidth',1.5)
hold on
plot(time,qx,'LineWidth',1.5)
plot(time,qy,'LineWidth',1.5)
plot(time,qz,'LineWidth',1.5)
xline(time(105), '--r', 'Launch')
xline(time(apogeeidx-launchidx+105), '--b', 'Apogee')
xline(time(landidx-launchidx+105), '--y', 'Touchdown')
grid on
legend('Qw','Qx','Qy','Qz')
xlabel('Time (s)')
ylabel('Quaternion')
title('Madgwick Quaternion')

%% Acceleration Magnitude

figure()
plot(time,accelMag,'LineWidth',2)
hold on
xline(time(105), '--r', 'Launch')
xline(time(apogeeidx-launchidx+105), '--b', 'Apogee')
xline(time(landidx-launchidx+105), '--y', 'Touchdown')
grid on
xlabel('Time (s)')
ylabel('|a| (m/s^2)')
title('Acceleration Magnitude')

%% Angular Velocity Magnitude

figure()
plot(time,gyroMag,'LineWidth',2)
hold on
xline(time(105), '--r', 'Launch')
xline(time(apogeeidx-launchidx+105), '--b', 'Apogee')
xline(time(landidx-launchidx+105), '--y', 'Touchdown')
grid on
xlabel('Time (s)')
ylabel('|\omega| (deg/s)')
title('Angular Velocity Magnitude')

%% Vertical Velocity

figure()
plot(time,verticalVelocity,'LineWidth',2)
hold on
xline(time(105), '--r', 'Launch')
xline(time(apogeeidx-launchidx+105), '--b', 'Apogee')
xline(time(landidx-launchidx+105), '--y', 'Touchdown')
grid on
xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('Estimated Vertical Velocity')

%% Vertical Acceleration

figure()
plot(time,verticalAcceleration,'LineWidth',2)
hold on
xline(time(105), '--r', 'Launch')
xline(time(apogeeidx-launchidx+105), '--b', 'Apogee')
xline(time(landidx-launchidx+105), '--y', 'Touchdown')
grid on
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
title('Estimated Vertical Acceleration')

%% Temprature

figure()
plot(time,temp,'LineWidth',2)
hold on 
xline(time(105), '--r', 'Launch')
xline(time(apogeeidx-launchidx+105), '--b', 'Apogee')
xline(time(landidx-launchidx+105), '--y', 'Touchdown')
grid on
xlabel('Time (s)')
ylabel('Temperature (°C)')
title('Temperature')

%% Dashboard

figure("Name","Final Grid")

% Altitude
subplot(3,2,1)
plot(time,altitude,'LineWidth',1.5)
hold on
xline(time(105), '--r', 'Launch')
xline(time(apogeeidx-launchidx+105), '--b', 'Apogee')
xline(time(landidx-launchidx+105), '--y', 'Touchdown')
plot(time(apogeeidx - launchidx + 105), apogeeheight, "ro")
labels = compose('%.2f seconds after launch\n%.2fm apogee', ...
    time(apogeeidx - launchidx + 105) - time(105), apogeeheight);
text(time(apogeeidx - launchidx), apogeeheight, labels, ...
    'Position', [time(apogeeidx-launchidx)+5, apogeeheight])
yline(0,'g')
grid on
title('Altitude')
xlabel('Time(s)')
ylabel('Altitude(m)')

%Acceleration Magnitude
subplot(3,2,2)
plot(time,accelMag,'LineWidth',1.5)
hold on
xline(time(105), '--r', 'Launch')
xline(time(apogeeidx-launchidx+105), '--b', 'Apogee')
xline(time(landidx-launchidx+105), '--y', 'Touchdown')
yline(0, 'g')
grid on
title('Acceleration Magnitude')
xlabel('Time(s)')
ylabel('Accel(m/s$^2$)', 'Interpreter', 'Latex')

% Acceleration Components
subplot(3,2,3)
plot(time,ax)
hold on
plot(time,ay)
plot(time,az)
xline(time(105), '--r', 'Launch')
xline(time(apogeeidx-launchidx+105), '--b', 'Apogee')
xline(time(landidx-launchidx+105), '--y', 'Touchdown')
grid on
title('Accelerometer')
legend('Ax','Ay','Az')
xlabel('Time(s)')
ylabel('Accel(m/s$^2$)', 'Interpreter', 'Latex')

% Angular Velocity Components
subplot(3,2,4)
plot(time,gx)
hold on
plot(time,gy)
plot(time,gz)
xline(time(105), '--r', 'Launch')
xline(time(apogeeidx-launchidx+105), '--b', 'Apogee')
xline(time(landidx-launchidx+105), '--y', 'Touchdown')
grid on
title('Gyroscope')
legend('Gx','Gy','Gz')
xlabel('Time(s)')
ylabel('Angular Velocity (deg/s)')

% Vertical Velocity
subplot(3,2,5)
plot(time,verticalVelocity,'LineWidth',1.5)
[pks, locs] = max(verticalVelocity);
BurnTimeFlight = time(locs) - time(105);
labels = compose('%.2f seconds after launch\n%.2fm/s max velocity burn cutoff', ...
    BurnTimeFlight, pks);
grid on
hold on
plot(time(locs), pks, 'ro')
text(time(locs), pks, labels, 'Position', [time(locs)+2, pks - 2])
yline(0,'--g')
title('Vertical Velocity')
xlabel('Time (s)')
ylabel("Velocity(m/s)")

% Quaternions
subplot(3,2,6)
plot(time,qw,'LineWidth',1.5)
hold on
plot(time,qx,'LineWidth',1.5)
plot(time,qy,'LineWidth',1.5)
plot(time,qz,'LineWidth',1.5)
grid on
legend('Qw','Qx','Qy','Qz')
xlabel('Time (s)')
ylabel('Quaternion')
title('Madgwick Quaternion')

sgtitle('Flight 00 D-12 "Peter Trifin" Rocket Flight Summary - Leo P')

%% Calculated Vs Recorded Trajectory

figure("Name", "Calculated Vs. Recorded")
subplot(1, 2, 1);
recordedTime = time(105:apogeeidx-launchidx+150);
recordedAltitude = altitude(105:apogeeidx-launchidx+150);
recordedTime = recordedTime - recordedTime(1);
plot(t, AltitudeCalc, '-r', ...
     recordedTime, recordedAltitude, '-b')
xlabel("Time Since Launch (s)")
ylabel("Altitude (m)")
title("Calculated vs. Recorded Altitude")
legend("Calculated", "Recorded")
grid on
subplot(1,2,2);
recordedVelocity = verticalVelocity(105:apogeeidx-launchidx+150);
plot(t, VelocityCalc, '-r', ...
     recordedTime, recordedVelocity, '-b')
xlabel("Time Since Launch (s)")
ylabel("Velocity (m/s)")
title("Calculated vs. Recorded Velocity")
legend("Calculated", "Recorded")
grid on
sgtitle("Comparison of Calculated Vs. Recorded Performance")

%% Results Table

Parameter = {
    'Apogee (m)'
    'Time to Apogee (s)'
    'Maximum Velocity (m/s)'
    'Burn Time (s)'
    };

Calculated = {
    sprintf('%.2f', ApogeeCalc)
    sprintf('%.3f', ApogeetimeCalc)
    sprintf('%.2f', max(VelocityCalc))
    sprintf("%.2f", BurnTimeCurve)
    };

Recorded = {
    sprintf('%.2f', apogeeheight)
    sprintf('%.3f', time(apogeeidx-launchidx+105) - time(105))
    sprintf('%.2f', max(verticalVelocity))
    sprintf("%.2f", BurnTimeFlight)
    };

Difference = {
    sprintf('%+.2f', ApogeeCalc-apogeeheight)
    sprintf('%+.3f', ApogeetimeCalc-(time(apogeeidx-launchidx+105)-time(105)))
    sprintf('%+.2f', max(VelocityCalc)-max(verticalVelocity))
    sprintf('%+.3f', BurnTimeCurve - BurnTimeFlight)
    };

PercentError = {
    sprintf('%.1f%%', 100*abs((ApogeeCalc-apogeeheight)/apogeeheight))
    sprintf('%.1f%%', 100*abs((ApogeetimeCalc-(time(apogeeidx-launchidx+105)-time(105)))/(time(apogeeidx-launchidx+105)-time(105))))
    sprintf('%.1f%%', 100*abs((max(VelocityCalc)-max(verticalVelocity))/max(verticalVelocity)))
    sprintf('%.1f%%', 100*abs((BurnTimeCurve - BurnTimeFlight)/(BurnTimeFlight)))
    };

Results = table(Parameter, Calculated, Recorded, Difference, PercentError);
%writetable(Results, "PeterTrifinResults.csv")