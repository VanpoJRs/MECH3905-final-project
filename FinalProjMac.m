close all                      
clear all                      
clc


[j, ball_image, alpha, mask1] = figure_setup_bike(); %where the background is read in 

[maskbike, maskbluecar, maskfollowers, hcar, hbike, numCars, offsets, l,o, p_4, q_4, y_bike, dy_bike, y_bluecar, dy_bluecar, maskoil] = setup_obstacles();
% ================================================================
[bg_img,~,~] = imread('newbkg_v1.png');
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
win_color=[235 226 253];   %finsih point RGB definition
win_tol=30;

%=====================================================================
global m c ux uy startscreen% declare global variables used in ODE
global gamestate x_next x_predict x_current
gamestate = "play";

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
port="COM3";                   % COM port connected to Arduino
baud=115200;                   % baud rate (must match Arduino code)

arduino=serialport(port,baud); % create serial communication object
configureTerminator(arduino,"CR/LF"); % define line ending
flush(arduino);                % clear communication buffer
pause(2);                      % allow Arduino to reboot

%===========setup ball image and screens==========================================================
%=================================================================================================
% setup animation figure
[j,ball_image,alpha]=figure_setup_bike(); % create figure window and load ball image
[b,a,~]=size(ball_image);              % determine dimensions of ball image
scale=0.07;                            % scaling factor for ball image

% draw initial ball location
H=image(ball_image,...
'Xdata',[0-scale*a/2,0+scale*a/2],...
'Ydata',[0-scale*b/2,0+scale*b/2],...
'AlphaData',alpha);

set(gca,'YLimMode','manual');          % fix y-axis limits to prevent auto scaling

[start_img,~,start_alpha] = imread('start_screen.jpg');
start_img=flipud(start_img);
start_alpha=flipud(start_alpha);

[end_img,~,end_alpha]=imread('collide.jpg');
end_img=flipud(end_img);
end_alpha=flipud(end_alpha);

[win_img,~,win_alpha]=imread('win_screen.jpg');
win_img=flipud(win_img);
win_alpha=flipud(win_alpha);

H_start = image(start_img,'XData',[-100 100],'YData',[-100 100],'AlphaData',start_alpha);

set(gca,'YLimMode','manual');
%=================================================================================================
%=================================================================================================


%=================================================================================================
%==================================MAIN LOOP======================================================
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

%------------------start-----------------------------------------------
        if strcmp(startscreen,'start')
             if button == 1
                startscreen= "play";
                delete(H_start);
             
            end
            drawnow
        continue   % skip physics until game starts
        end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


%-----------collision restart--------------------------------------------
        
        if strcmp(gamestate,'lose')
            if button==1
                gamestate='start';
                shrinkingscale =1; %reset scale here 
                x(:,i)=x0;
                H_screen=image(start_img,'XData',[-100 100],'YData',[-100 100],'AlphaData',start_alpha);
            end
            drawnow
            continue
        end    
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


%----------reach finish point and restart---------------------------------
        if strcmp(gamestate,'win')
            if button==1
                gamestate='start';
                x(:,i)=x0;
                H_screen=image(start_img,'XData',[-100 100],'YData',[-100 100],'AlphaData',start_alpha);
            end
            drawnow
            continue
        end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

collisionCode2= staticobstacles(mask1, maskoil, maskpothole); %call collision function to check if player is in oil.
switch  collisionCode2
    case 1 %oil puddle 
    input_scale = 0.4; %mofidy external force to feel less (go slower)
    damping= 0.02; %modified damping to make looser more ut of control 

    case 2 %pothole 
    gamestate= 'shrinking';
    shrinkingscale =1; %restart it/ initalize it 

    otherwise 
    input_scale= 1.0; 
    damping= 1.0; 
    end 
%---------------------RK4 physics------------------------------------------
     if strcmp(gamestate, 'shrinking')
          x(:, i+1) = x(:,i); %freeze motion
    else 
     % RK4 integration step normal motion 
        x(:,i+1)=RK4(t(i),x(:,i),h,...
        w1,w2,w3,w4,c2,c3,c4,...
        a21,a31,a32,a41,a42,a43);

        %call limiting motion function 
        x(:,i+1)=MovLimit(x(:,i),x_predict,mask,10);
    end 
       
        t(i+1)=t(i)+h;                 % update time
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
if strcmp(gamestate, 'shrinking')
    current_img=damaged_img; 
else
    current_img= ball_img; 
end 
    
 % Update animation by deleting old ball and drawing new position
        delete(H);
    scale_effective= scale * shrinkingscale;
        H=image(current_image,...
        'Xdata',[x(1,i+1)-scale_effective*a/2,x(1,i+1)+scale_effective*a/2],...
        'Ydata',[x(2,i+1)-scale_effective*b/2,x(2,i+1)+scale_effective*b/2],...
        'AlphaData',alpha);

        drawnow                        % update figure immediately


%===================================update animated obstacle===========================

[y_bike,x_bike, dy_bike] = update_bike(y_bike, dy_bike, ymin, ymax); %update bike position
[y_bluecar, dy_bluecar]= update_cars(y_bluecar, dy_bluecar, ymin, ymax); %update car position

%------------------------collision logic---------------------------
collisionCode= mapping(mask1, maskbike, maskbluecar); %collision code for animated obstacles 
    switch collisionCode
        case 1 %bike
            gamestate= 'lose'; 
        
        case 2 %car
            gamestate = 'lose'; 
        
        otherwise %nothing
end 

        if isequal(x(:,i+1),x(:,i))
            gamestate='lose';
            H_screen=image(end_img,'XData',[-100 100],'YData',[-100 100],'AlphaData',end_alpha);
        end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

%%=================================draw image animated obstacles============================
%% drawing function (image rendering for car)
%function draw_cars(hbluecar, hcar, x_bluecar, y_bluecar,numCars, ymin, ymax, p, q, p_4, q_4)
   



    delete(hbike); %clear bike for next image 
    delete(hbluecar); %clear bluecar image

    drawnow limitrate %limerate to skip iterations of loop if laggy
    pause(0.016) %pause for 60FPS

    clear arduino  %close serial port connection 


%--------------------finish point logic-----------------------------
        col=round((x(1,i+1)+100)/200*cols);
        row=round((x(2,i+1)+100)/200*rows);

        if row>=1 && row<=rows && col>=1 && col<=cols
            pixel=squeeze(bg_img(row,col,:))';
            diff=sqrt(sum((pixel-win_color).^2));

            if diff<win_tol
                gamestate='win';

                H_screen=image(win_img,'XData',[-100 100],'YData',[-100 100],'AlphaData',win_alpha);
            end
        end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      
    end
end 
%======================================================================================
%==========================MAIN LOOP ENDS==============================================


%%RK4 function
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
function dxdt=f(t,x, input_scale, damping) 

global m c ux uy            % access global parameters

dxdt=zeros(4,1);                        % initialize derivative vector

dxdt(1)=x(3);                           % dx/dt = velocity in x
dxdt(2)=x(4);                           % dy/dt = velocity in y

dxdt(3) = (-c*damping/m)*x(3) + (input_scale*ux/m);
dxdt(4) = (-c*dmping/m)*x(4) + (input_scale*uy/m);

end


% %%%move this inot function ?
% %%function to setup figure window 
% function [j,ball_image,alpha_channel]=figure_setup()
% % adjust figure to desired size&position, then use f.Position to get #s
% %f=figure('position',[1661 -910 560 826]);
% j=figure('position',[2 50 560 826]);
% hold on % prevent axes from flipping y-axis when plotting images
% axis('equal') % set aspect ratio so equal tick mark increments on each
% % axis are equal in size
% % to use up full figure window resize and reposition axes of plot
% % to its normalized limits (0 to 1)
% ax=gca; % get current axes handle
% ax.Position=[0 0 1 1]; % normalized position of axes [left bottom width height]
% set(gcf,'Toolbar','none','Menu','none'); % remove toolbar and menu
% set(gca,'visible','off'); % remove axis labels
% set(gcf,'color','w'); % make figure background white
% ylim([-100 +100]) % set y axis limits
% xlim([-100 +100]) %set x axis limits 
% axis('manual') % freeze all axis limits for subsequent plots so they
% % do not automatically adjust on the fly
% % read image of black ball to track
% % for fast performance, make image using as few pixels as possible
% [ball_image,~,alpha_channel]=imread('circle_black_transparent.png');
% ball_image=flipud(ball_image); % need to flip image so it is oriented correctly
% % images are stored so that they face upwards when
% % y axes are reversed (positive downwards),
% % but our y axis is normal (postive upwards)
% % so we need to flip the image so it faces upward
% alpha_channel=flipud(alpha_channel);



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
       col=round((x_trial(1)+100)/200*cols);
       row=round((x_trial(2)+100)/200*rows);
 
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


