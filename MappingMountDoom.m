[accel, gyro, attitude] = parsePowerSenseData('md1.csv');
load md1.mat
clf

%plotting position vs time of the left and right wheel values
%data taken from encoders
hold on
plot(dataset(:,1), dataset(:,2), 'b');
plot(dataset(:,1), dataset(:,3), 'r');
xlabel('Time(seconds)');
ylabel('Position');
title('Plotting Data from Encoders');
legend('left', 'right')
hold off

%plotting attitude vs time 
%data taken from phone accelerometer
figure
hold on
plot(attitude(:,1), attitude(:,2), 'b');
plot(attitude(:,1), attitude(:,3), 'r');
plot(attitude(:,1), attitude(:,4), 'g');
xlabel('Time(sec)');
ylabel('Attitude (degrees)');
title('Plotting Data from Accelerometer');
legend('yaw', 'pitch', 'roll');
hold off

%reorienting phone data to Neato coordinate system :)
Yawn = attitude(:,2) - 90;
Pitchn = attitude(:,4);
Rolln = attitude(:,3);

Accelx = accel(:,3);
Accely = -accel(:,2);
Accelz = accel(:,4);
Phone2Accel = [accel(:,1) Accelx Accely Accelz];

%QUESTIONS
%4 Rotation Matrix Validation
%prediction of acceleration values based on attitude data
roll = Rolln(1);
pitch = Pitchn(1);
yaw = Yawn(1);
Rroll = [1 0 0 ; 0 cosd(-pitch) -sind(-pitch); 0 sind(-pitch) cosd(-pitch)];
Rpitch = [cosd(roll) 0 sind(roll); 0 1 0; -sind(roll) 0 cosd(roll)];
Ryaw = [cosd(yaw) -sind(yaw) 0; sind(yaw) cosd(yaw) 0; 0 0 1];
RoomToNeato = Rroll * Rpitch * Ryaw;
gravity = [0; 0; 9.8];
rotated = RoomToNeato*gravity; %pretty close to accel data

%5 Attitude to Orientation
%class x, y, z are when roll, pitch, yaw are 0
vector = [1 ;0 ;0];
NeatoToRoom = inv(RoomToNeato);
orientation = NeatoToRoom * vector;

%MAPPING
heading = ones(6161, 3);
%converting attitudes from Neato coordinates to Room coordinates
for i = 1:6161
    roll = Rolln(i);
    pitch = Pitchn(i);
    yaw = Yawn(i);
    Rroll = [1 0 0 ; 0 cosd(-pitch) -sind(-pitch); 0 sind(-pitch) cosd(-pitch)];
    Rpitch = [cosd(roll) 0 sind(roll); 0 1 0; -sind(roll) 0 cosd(roll)];
    Ryaw = [cosd(yaw) -sind(yaw) 0; sind(yaw) cosd(yaw) 0; 0 0 1];
    RoomToNeato = Rroll * Rpitch * Ryaw;
    NeatoToRoom = inv(RoomToNeato);
    heading(i,:) = transpose(NeatoToRoom * vector);
end

%creating a heading matrix and appended times to the front
heading = [attitude(:,1) heading];
%started time at zero to line up time frames
heading(:,1) = heading(:,1) - heading(1,1);

%calculated speed from velocities of wheels
speed = (diff(dataset(:,2)) + diff(dataset(:,3))) / 2;
speed = [speed; 0];
speed = [dataset(:, 1) speed];
speed(:,1) = speed(:,1) - speed(1,1);

%calculated distance based on speed and time
distance = speed(1:end-1, 2) .* diff(speed(:,1));
distance = [dataset(1:end-1, 1) distance];

%found heading at each time stamp and corresponding distance
direction = ones(1276, 5);
for i = 1:1276
    [M, I] = min(abs(heading(:,1) - distance(i, 1))); 
    direction(i, :) = [distance(i,:) heading(I,2:4)];
end

%calculating the movement vectors based on distance and headings
movements = [direction(:, 2).* direction(:,3), direction(:, 2).* direction(:,4), direction(:, 2).* direction(:,5)]; 

%calculated position of the Neato based on cumulative movement vectors
positions = cumsum(movements, 1);

%plot our map :)
figure
plot3(positions(:,1), -positions(:,2), positions(:,3), '.'); %we inverted Y because Neato + axis is to the left
axis equal
title('Mount Doom Mapping');
xlabel('X');
ylabel('Y');
zlabel('Z');