close all                      
clear all                      
clc

% ==================================mask set up for rollback ==============================
bg_img = imread('newbkg_v4.jpg');
bg_img = flipud(bg_img);
[rows, cols, ~] = size(bg_img);
[mask, start_pos, finish_color] = createMazeLogic(bg_img);

%==========bakcgournd ======
[j, maskpothole, bg]= figure_setup_bike(); % create figure window and load ball image
hbg= image(bg, ...
    'Xdata', [-90 4000],...
    'YData', [0 2160]); 


%===============================Global varibles ======================================
global m c ux uy  % declare global variables used in ODE
global gamestate x_next x_predict x_current startscreen


m = 10;             % kg, mass of the ball
c = 10;             % Ns/m, damping coefficient (slows the ball)
scale = 0.07;       % scaling factor for displaying the ball image

ux=0;                          % initialize joystick force along x-axis
uy=0;                          % initialize joystick force along y-axis

x0=[start_pos(1);
    start_pos(2);
    0;
    0];

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

%=========================Serial communication setup
port="COM5";                   % COM port connected to Arduino
baud=115200;                   % baud rate (must match Arduino code)

arduino=serialport(port,baud); % create serial communication object
configureTerminator(arduino,"CR/LF"); % define line ending
flush(arduino);                % clear communication buffer
pause(2);                      % allow Arduino to reboot


%===========Ball image==========================================================

% player image
[ball_image,~,alpha_channel]=imread('ball_image.png');
ball_image=flipud(ball_image); 
alpha_channel=flipud(alpha_channel);
[b,a,~]=size(ball_image);              % determine dimensions of ball image
scale=0.13;   % scaling factor for ball image
mask1 = alpha_channel > 0; %creating bool mask for player character 
%draw initial ball location
H=image(ball_image,...
'Xdata',[0-scale*a/2,0+scale*a/2],...
'Ydata',[50-scale*b/2,50+scale*b/2],...
'AlphaData',alpha_channel);


%============start/end screens========
% [start_img,~,start_alpha] = imread('start_screen.png');
% start_img=flipud(start_img);
% start_alpha=flipud(start_alpha);
% [b3, a3, ~]= size(start_img); 
% H_start= image(start_img); 
% set(H_start, ...
%     'Xdata', [0-scale*a3/2,scale*a3/2 ],...
%     'Ydata',[50-scale*b3/2,50+scale*b3/2],...
% 'AlphaData',start_alpha);
% drawnow; 
% 
% 
% 
%%=====different screen states=============

start_img   = imread('start_screen.jpg');
start_img   = flipud(start_img);
start_alpha = uint8(255 * ones(size(start_img,1), size(start_img,2)));

win_img   = imread('win_screen.jpg');
win_img   = flipud(win_img);
win_alpha = uint8(255 * ones(size(win_img,1), size(win_img,2)));

% Display the start screen overlay (covers entire game window)
H_screen = image(start_img, 'XData', [-100 3840], 'YData', [0 2160], ...
    'AlphaData', start_alpha);


%=====winning=======================
[win_img,~,win_alpha]=imread('win_screen.png');
win_img=flipud(win_img);
win_alpha=flipud(win_alpha);

startscreen='start';
gamestate= 'play'; 

%=============================Finish line ============================================
win_color=finish_color;   %finsih point RGB definition
win_tol=30;


%==================================MAIN LOOP======================================================
% simulation loop
for i=1:n                              % loop through each time step

    writeline(arduino,int2str(i));     % send loop counter to Arduino

    while arduino.NumBytesAvailable==0
        pause(0.01); % wait until Arduino sends data
    end

    data=readline(arduino);            % read serial data from Arduino
    num=str2double(split(data,','));   % convert comma-separated string to numbers

    


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
                startscreen= 'play';
                delete(H_screen);
             
            end
            drawnow;
            continue;   % skip physics until game starts
        end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


%----------reach finish point and restart---------------------------------
        if strcmp(gamestate,'win')
            if button==1
                gamestate='start';
                x(:,i)=x0;
                H_screen=image(start_img,'XData',[-100 100],'YData',[-100 100],'AlphaData',start_alpha);
            end
            drawnow;
            continue;
        end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

%---------------------RK4 physics------------------------------------------
         % RK4 integration step normal motion 
            x_predict=RK4(t(i),x(:,i),h,...
            w1,w2,w3,w4,c2,c3,c4,...
            a21,a31,a32,a41,a42,a43);
    
            %call limiting motion function 
            x(:,i+1)=MovLimit(x(:,i),x_predict,mask,10);

    %properties for player 
    xB = x(1,i+1);   % center x
    yB = x(2,i+1);   % center y
    
    wB = a;   % width
    hB = b;   % height
           
    t(i+1)=t(i)+h;                 % update time
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    % Update animation by deleting old ball and drawing new position
    delete(H);
    H = image(ball_image,...
       'Xdata',[x(1,i+1)-scale*a/2, x(1,i+1)+scale*a/2],...
       'Ydata',[x(2,i+1)-scale*b/2, x(2,i+1)+scale*b/2],...
       'AlphaData',alpha_channel);
    drawnow;

    
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
function dxdt=f(t,x)
global m c ux uy            % access global parameters

dxdt=zeros(4,1);                        % initialize derivative vector

dxdt(1)=x(3);                           % dx/dt = velocity in x
dxdt(2)=x(4);                           % dy/dt = velocity in y

dxdt(3) = (-c/m)*x(3) + (ux/m);
dxdt(4) = (-c/m)*x(4) + (uy/m);
%removed damping and input)scale 

end


%==================function 1=================================
%==============movement limitation============================
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
%======================================================================


%================function 2-starting and ending position===============
function [mask, start_pos, finish_color] = createMazeLogic(bg_img)

[rows, cols, ~] = size(bg_img);

mask = ones(rows, cols);

wall_color   = [41 41 41];          % BLACK wall
start_color  = [247 83 84];      % YOUR START COLOR
finish_color = [0 115 84];       % YOUR FINISH COLOR

tol = 30;

start_found = false;

for i = 1:rows
    for j = 1:cols

        pixel = double(squeeze(bg_img(i,j,:))');

        
        if norm(pixel - wall_color) < tol
            mask(i,j) = 0;   % BLOCKED
        end


        if ~start_found && norm(pixel - start_color) < tol
            start_pos = [j; i];
            start_found = true;
        end

    end
end

if ~start_found
    error('Start point NOT found!');
end

end
%==========================================================