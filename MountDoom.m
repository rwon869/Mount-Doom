% Mount Doom 
pub = rospublisher('/raw_vel');
acc = rossubscriber('/accel');
msg = rosmessage(pub);
sub_bump = rossubscriber('/bump');
omega = (0.5 + .05)/.2495; %calculate angular velocity (Vr-Vl/d)
%run 'rostopic echo /accel' in the command line to find initial accel
%values
threshold = 0.01;
while 1
    accel_message = receive(acc);
    x = accel_message.Data(1) - .2770; %subtract initial accel values
    y = accel_message.Data(2) - .0130;
    z = accel_message.Data(3) - .967;
    
    %stop when Neato gets to the top
    if abs(y) < threshold && abs(x) < threshold
        msg.Data = [0 0];
        send(pub, msg);
        pause(0.5);
    %if accelY is within threshold, drive Neato forward
    elseif abs(y) < threshold 
        msg.Data = [.05 .05];
        send(pub, msg);
        pause(0.75);
        
    %if accelY is greater than threshold, turn counterclockwise
    elseif y > threshold 
        msg.Data = [-.05, .05]; %turn counterclockwise
        send(pub,msg);
        time = abs((y * pi)/omega); %set time to theta/omega
        pause(time);
        
    %if accelY is less than threshold, turn clockwise
    elseif y < threshold
        msg.Data = [.05, -.05]; %turn clockwise
        send(pub,msg);
        time = abs((y * pi)/omega);
        pause(time);
    end
    
    %bump detection stops neato when bumped
    bumpMessage = receive(sub_bump);
    if any(bumpMessage.Data)
        msg.Data = [0.0, 0.0];
        send(pub, msg);
        pause(0.1);
        break;
    end
 end
msg.Data = [0,0];
send(pub,msg);

