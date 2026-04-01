close all                      
clear all                      
clc


[j, ball_image, alpha] = figure_setup_bike(); %where the background is read in 

% ================================================================
[bg_img,~,~] = imread('newbkg_v1.png');   % your road image
bg_img = flipud(bg_img);
bg_img = double(bg_img);

[rows, cols, ~] = size(bg_img);

% Define allowed colors (EDIT THESE)
colors = [
    172 191 105;   % green
    177 177 177;   % grey
    242 242 242;   %white
    65 65 67    %black
];

tol = 25;

mask = zeros(rows, cols);

for k = 1:size(colors,1)
    diff = sqrt( ...
        (bg_img(:,:,1) - colors(k,1)).^2 + ...
        (bg_img(:,:,2) - colors(k,2)).^2 + ...
        (bg_img(:,:,3) - colors(k,3)).^2 );

    mask = mask | (diff < tol);
end

mask = double(mask);
%============================================================================

global m c ux uy% declare global variables used in ODE
global gamestate x_next x_predict x_current
gamestate = "wait";

startscreen='start';
% Define constants
m = 10;             % kg, mass of the ball
c = 10;             % Ns/m, damping coefficient (slows the ball)
scale = 0.07;       % scaling factor for displaying the ball image

ux=0;                          % initialize joystick force along x-axis
uy=0;                          % initialize joystick force along y-axis

% initial conditions of mass
x0=[0;                         % m, initial x coordinate of mass
0;                             % m, initial y coordinate of mass
0;                             % m/s, initial velocity along x-axis
0];                            % m/s, initial velocity along y-axis

% Define time interval for solution
t0=0; tn=100;                  % s, time interval from [t0 to tn]
h=0.1;                         % s, step size used for RK4 integration
n=(tn-t0)/h;                   % number of time steps

x=zeros(4,n+1);                % preallocate RK4 solution matrix (4 states)
t=zeros(n+1,1);                % preallocate time vector

x(:,1)=x0;                     % set initial state
t(1)=t0;                       % set initial time

ux_history=zeros(n,1);           % store joystick force history (x direction)
uy_history=zeros(n,1);           % store joystick force history (y direction)

% Classic RK4 Method parameters
w1=1/6; w2=1/3; w3=1/3; w4=1/6;
c2=1/2; c3=1/2; c4=1;
a21=1/2; a31=0; a32=1/2; a41=0; a42=0; a43=1;

% Serial communication setup
port="/dev/cu.usbmodem1101";                   % COM port connected to Arduino
baud=115200;                   % baud rate (must match Arduino code)

arduino=serialport(port,baud); % create serial communication object
configureTerminator(arduino,"CR/LF"); % define line ending
flush(arduino);                % clear communication buffer
pause(2);                      % allow Arduino to reboot

% setup animation figure
[j,ball_image,alpha]=figure_setup(); % create figure window and load ball image
[b,a,~]=size(ball_image);              % determine dimensions of ball image
scale=0.07;                            % scaling factor for ball image

% draw initial ball location
H=image(ball_image,...
'Xdata',[0-scale*a/2,0+scale*a/2],...
'Ydata',[0-scale*b/2,0+scale*b/2],...
'AlphaData',alpha);

set(gca,'YLimMode','manual');          % fix y-axis limits to prevent auto scaling

% simulation loop
for i=1:n                              % loop through each time step

    writeline(arduino,int2str(i));     % send loop counter to Arduino

    while arduino.NumBytesAvailable==0 % wait until Arduino sends data
    end

    data=readline(arduino);            % read serial data from Arduino
    num=str2double(split(data,','));   % convert comma-separated string to numbers

    disp(num)


    %It's originally larger or equal to 3, I changed it to 4 so button bool
    %signal can be detected
    if length(num) >= 4 && num(1)==i     % verify data is valid and synchronized

        % Read joystick forces
        ux=(num(3)-512);             % convert joystick x value to force
        uy=(num(2)-512);             % convert joystick y value to force

        button=num(4);               %button signal
        ux_history(i)=ux;                % store x-direction input
        uy_history(i)=uy;                % store y-direction input

             if button == 1
                startscreen= "play";
             
            end
        continue   % skip physics until game starts
        end

       


        % RK4 integration step
        x(:,i+1)=RK4(t(i),x(:,i),h,...
        w1,w2,w3,w4,c2,c3,c4,...
        a21,a31,a32,a41,a42,a43);

        t(i+1)=t(i)+h;                 % update time

        % Update animation by deleting old ball and drawing new position
        delete(H);

        H=image(ball_image,...
        'Xdata',[x(1,i+1)-scale*a/2,x(1,i+1)+scale*a/2],...
        'Ydata',[x(2,i+1)-scale*b/2,x(2,i+1)+scale*b/2],...
        'AlphaData',alpha);

        drawnow                        % update figure immediately
    end
end



function [y_bike,x_bike, dy_bike] = update_bike(y_bike, dy_bike, ymin, ymax); %update bike 
function[y_bluecar, dy_bluecar]= update_cars(y_bluecar, dy_bluecar, ymin, ymax); %update car

   if dy_bike>0 
        direction= 'northbound';
    else 
        direction= 'southbound'; 
    end 

    collisionCode= mapping(mask1, maskbike, maskbluecar, maskoil, maskpothole);
    
function draw_bike(x_bike y_bike, direction, bike_northbound, bike_southbound, bike_alpha_northbound, bike_alpha_southbound, o, l);
%%function car 


clear arduino                         % close serial port connection

end 

% Runge-Kutta fourth order method
function x_new = RK4(ti,xi,h,...
                     w1,w2,w3,w4,...
                     c2,c3,c4,...
                     a21,a31,a32,a41,a42,a43)


k1 = h * f(ti,xi);
k2 = h * f(ti + c2*h, xi + a21*k1);
k3 = h * f(ti + c3*h, xi + a31*k1 + a32*k2);
k4 = h * f(ti + c4*h, xi + a41*k1 + a42*k2 + a43*k3);

x_new=xi+w1*k1+w2*k2+w3*k3+w4*k4;       % combine slopes to compute next state

end

% system dynamics function
function dxdt=f(t,x)

global m c ux uy            % access global parameters

dxdt=zeros(4,1);                        % initialize derivative vector

dxdt(1)=x(3);                           % dx/dt = velocity in x
dxdt(2)=x(4);                           % dy/dt = velocity in y

dxdt(3) = (-c/m)*x(3) + (ux/m);
dxdt(4) = (-c/m)*x(4) + (uy/m);

end

%%function to setup figure window 
function [j,ball_image,alpha_channel]=figure_setup()
% adjust figure to desired size&position, then use f.Position to get #s
%f=figure('position',[1661 -910 560 826]);
j=figure('position',[2 50 560 826]);
hold on % prevent axes from flipping y-axis when plotting images
axis('equal') % set aspect ratio so equal tick mark increments on each
% axis are equal in size
% to use up full figure window resize and reposition axes of plot
% to its normalized limits (0 to 1)
ax=gca; % get current axes handle
ax.Position=[0 0 1 1]; % normalized position of axes [left bottom width height]
set(gcf,'Toolbar','none','Menu','none'); % remove toolbar and menu
set(gca,'visible','off'); % remove axis labels
set(gcf,'color','w'); % make figure background white
ylim([-100 +100]) % set y axis limits
xlim([-100 +100]) %set x axis limits 
axis('manual') % freeze all axis limits for subsequent plots so they
% do not automatically adjust on the fly
% read image of black ball to track
% for fast performance, make image using as few pixels as possible
[ball_image,~,alpha_channel]=imread('circle_black_transparent.png');
ball_image=flipud(ball_image); % need to flip image so it is oriented correctly
% images are stored so that they face upwards when
% y axes are reversed (positive downwards),
% but our y axis is normal (postive upwards)
% so we need to flip the image so it faces upward
alpha_channel=flipud(alpha_channel);
end



%   x_current: current step
%   x_predict: RK4 predicted step (next step)
%   mask: 1=allowed, 0=wall
%   substeps: number of small steps for safe movement

function x_next = MovLimit(x_current, x_predict, mask, substeps)

    % Start from current state
    x_next = x_current;

    % Total displacement from RK4
    dx_total=x_predict(1)-x_current(1);
    dy_total=x_predict(2)-x_current(2);

    % Divide into small steps, makes the steps accurate so the moving obj won't skip the wall
    dx=dx_total/substeps;
    dy=dy_total/substeps;

    % Get mask size
    [rows, cols]=size(mask);

    % Step by step movement
    for k=1:substeps

        % Try small move
        x_trial=x_next;
        x_trial(1)=x_trial(1) + dx;
        x_trial(2)=x_trial(2) + dy;

        % Convert to pixel indices
        col=round(x_trial(1));
        row=round(x_trial(2));

        % Boundary check
        if row<1 || row>rows || col<1 || col>cols
            break
        end

        % Mask check
        if mask(row, col) == 1
            x_next = x_trial;
        else
            break % Collision happens and stop moving
        end

    end

    % Keep RK4 velocity only if movement succeeded
    if isequal(x_next(1:2), x_current(1:2))
        x_next(3)=0;
        x_next(4)=0;
    else
        x_next(3)=x_predict(3);
        x_next(4)=x_predict(4);
    end

end


