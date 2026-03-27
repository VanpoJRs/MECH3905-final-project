

%% animated obstacles function 
close all
clear all
clc 

%%animated obstacles 
%% 

%call the background 
[j, ball_image, alpha] = figure_setup_bike();

%-----------The following block is for boundary detecting-------------
%-----------Isolate the whole block if errors occur-------------------

[bg_img,~,~] = imread('maze.png');
bg_img = flipud(bg_img);
bg_img = double(bg_img);

[rows, cols, ~] = size(bg_img);

% Define allowed colors (EDIT THESE)
colors = [
    255 255 255;   % A
    200 200 200;   % B
    0   255   0;   % C
    0   0   255    % D
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
%------------------------------------------------------------------------
%------------------------------------------------------------------------



n=5000;
%% obstacles pictures 

%obstacle 1 call; biker 
scale_bike=0.9;

[bike_big, ~, bike_alpha_big]= imread('biker_image_v2.png'); 
bike_big=flipud(bike_big); %flip
bike_alpha_big=flipud(bike_alpha_big); %flip transperancy 

bike= imresize(bike_big, scale_bike); %reize big bike now 
bike_alpha= imresize(bike_alpha_big, scale_bike); %resize transperancy 
[l,o,~]=size(bike); %l=height, o=width 


%obstacle 2 call; car 
scale_bluecar=0.9;
scale_follower=0.7; 

[bluecar_big,~, bluecar_alpha_big] =imread('yellow_car_v2.png');
bluecar_big=flipud(bluecar_big); 
bluecar_alpha_big=flipud(bluecar_alpha_big);

%scaling bluecar size 
bluecar= imresize(bluecar_big, scale_bluecar); 
bluecar_alpha= imresize(bluecar_alpha_big, scale_bluecar); 
[p,q,~]= size(bluecar); 

%follower cars 
[car2, ~, alpha2]=imread('car4.png'); 
[car3, ~, alpha3]=imread('car4.png');
[car4_big, ~, alpha4_big]=imread('car4.png');
car2=flipud(car2); 
car3=flipud(car3); 
car4_big=flipud(car4_big);
alpha2=flipud(alpha2); 
alpha3=flipud(alpha3);
alpha4_big=flipud(alpha4_big);

car4=imresize(car4_big, scale_follower); 
alpha4=imresize(alpha4_big, scale_follower); 
[p_4, q_4,~]= size(car4); 

disp(p_4); 
%cell array 
car_imgs= {car2, car3, car4};
car_alphas= {alpha2, alpha3, alpha4}; 

numCars = length(car_imgs); 
hcar = gobjects(1, numCars); %gobjects initilizes space for graphics object array i think 

%cars rotated 
% car2_sb=imrotate(car2, 180); 
% car3_sb=imrotate(car3, 180); 
% car4_sb=imrotate(car4, 180);
% alpha2_sb=imrotate(alpha2, 180); 
% alpha3_sb=imrotate(alpha3, 180); 
% alpha4_sb=imrotate(alpha4, 180); 
% 
% car_imgs_rotated= {car2_sb, car3_sb, car4_sb}; 
% car_alphas_rotated= {alpha2_sb, alpha3_sb, alpha4_sb}; 
% 
% numCars_sb = length(car_imgs_rotated); 
% hcar_sb = gobjects(1, numCars_sb);


%vector for y position offset. Quantity of pixels away from lead car. 
offsets= [-1.5*p, -3*p, -5*p]; 

%% inital pos and equations 

 %intial y equations for bike 
y_bike= 80; %inital y
dy_bike= 10; %biker speed 

%inital equations for bluecar 
y_bluecar= 80; 
dy_bluecar= 7;
x_bluecar= 2450; 

%intial pos of bike 
 % hbike=image(bike, ...
 %    'Xdata', [2320-o/2, 2320+o/2], ...
 %    'Ydata', [2190-l/2, 2190+l/2], ...
 %    'AlphaData', bike_alpha); %%coordinates (x,y) when y=0.


%inital pos of bluecar 
hbluecar =image(bluecar, ...
    'Xdata', [2450-q/2,2450+q/2], ...
    'Ydata', [-110-p/2, -110+p/2], ...
    'AlphaData', bluecar_alpha); 

%initial pos for follower cars 
for i= 1:numCars
    hcar(i)=image(car_imgs{i}, ...
        'Xdata', [2450-q_4/2, 2450+q_4/2], ...
        'Ydata', [y_bluecar + offsets(i) -p_4/2, ...
                  y_bluecar + offsets(i) + p_4/2], ...
        'AlphaData', car_alphas{i}); 
end 

%% equations of motions

%equation of motion for bike
x_bike=2300+20*cos(0.7*y_bike/50);

%equation of motion for blue car 
y_bluecar = y_bluecar + dy_bluecar;

%boundaries should be the same for all 
ymax= 2160; %top boundary on figure window pop up
ymin=-50;  %bottom boundary on figure window pop up 
roadlength= ymax-ymin; %for follower cars finish the road after lead car loops back 

%% For turn vehicle turnaround
bike_northbound=bike; 
bike_alpha_northbound=bike_alpha; 

bike_southbound=imrotate(bike, 180); 
bike_alpha_southbound=imrotate(bike_alpha, 180);

% bluecar_northbound= bluecar; 
% bluecar_alpha_northbound= bluecar_alpha; 

% bluecar_southbound= imrotate(bluecar, 180);
% bluecar_alpha_southbound= imrotate(bluecar_alpha, 180); 
% 
% car_imgs_northbound= car_imgs; %cell array of rotated 
% car_imgs_southbound= car_imgs_rotated; %cell array of rotated


%% Main loop 
for k=1:n
    y_old_bike=y_bike; %sotring previous y 
    y_bike=y_bike+dy_bike; %update y 
  
    y_old_bluecar= y_bluecar; 
    y_bluecar= y_bluecar + dy_bluecar;
    x_bluecar=2450; %constant because going in straight line 

    %bike motion 
    x_bike=2310+20*cos(0.7*y_bike/50);
       
    %bluecar motion 
    y_bluecar= y_bluecar + dy_bluecar; 
      
     
    %boundary chec. Velocity (+)= Northbound, Velocity (-)= Southbound  
    if y_bike>ymax || y_bike< ymin
        dy_bike= -dy_bike;    
    end 
    %check boundary of car
    if y_bluecar>ymax 
        y_bluecar=ymin; 
    end


         
    %switch direction of bike 
    if dy_bike>0 
        direction= 'northbound';
    else 
        direction= 'southbound'; 
    end 
  
    switch direction
        case 'northbound'
            %image(bike_northbound); 
            hbike=image(bike_northbound, ...
                'Xdata', [x_bike-o/2, x_bike+o/2], ...
                'Ydata', [y_bike-l/2, y_bike+l/2], ...
                'AlphaData', bike_alpha_northbound); 

    
        case 'southbound'
            %image(bike_southbound);
             hbike=image(bike_southbound, ...
                'Xdata', [x_bike-o/2, x_bike+o/2], ...
                'Ydata', [y_bike-l/2, y_bike+l/2], ...
                'AlphaData', bike_alpha_southbound); 
        otherwise 
            
    end

  
    %switch direction of cars 
    % if y_bluecar>ymax || y_bluecar<ymin
    %    dy_bluecar=-dy_bluecar;
    % end 

    % if dy_bluecar>0
    %     direction= 'northbound'; 
    % else
    %     direction= 'southbound'; 
    % end
    % switch direction 
    %     case 'northbound'
    %         %image bluecar northbound 
    %         hbluecar =image(bluecar_northbound, ...
    %             'Xdata', [x_bluecar-q/2, x_bluecar+q/2], ...
    %             'Ydata', [y_bluecar-p/2, y_bluecar+p/2], ...
    %             'AlphaData', bluecar_alpha_northbound); 
    %         for i=1:numCars
    %         y_i = y_bluecar + offsets(i); 
    % 
    %         set(hcar(i), ...
    %             'Xdata', [2450-q/2, 2450+q/2], ...
    %             'Ydata', [y_i -p/2, ...
    %                       y_i + p/2], ...
    %             'AlphaData', car_alphas{i});
    %         end 
    % 
    %     case 'southbound'
    %         hbluecar =image(bluecar_southbound, ...
    %             'Xdata', [x_bluecar-q/2, x_bluecar+q/2], ...
    %             'Ydata', [y_bluecar-p/2, y_bluecar+p/2], ...
    %             'AlphaData', bluecar_alpha_southbound); 
    %         for i=1:numCars_sb
    %         y_i = y_bluecar + offsets(i); 
    % 
    %         set(hcar_sb(i), ...
    %             'Xdata', [2450-q/2, 2450+q/2], ...
    %             'Ydata', [y_i -p/2, ...
    %                       y_i + p/2], ...
    %             'AlphaData', car_alphas_rotated{i});
    %         end 
    % 
    % end 


%follower car 

       
%change direction bike figure if negative dy 
% if dy_bike>=0
%    current_img =bike_norm; 
%    current_alpha =bike_alpha; 
% else
%     current_img=bike_flipped; 
%     current_alpha= alpha_flipped;
% end
% 
% a= image(current_img); 
% 
% if y_bluecar>ymax || y_bluecar<ymin
%     dy_bluecar=-dy_bluecar;
% end 



%update bluecar image 
hbluecar =image(bluecar, ...
    'Xdata', [x_bluecar-q/2, x_bluecar+q/2], ...
    'Ydata', [y_bluecar-p/2, y_bluecar+p/2], ...
    'AlphaData', bluecar_alpha); 

%follower car 
    for i=1:numCars
        y_i = y_bluecar + offsets(i); 
        if y_i< ymin
            y_i=y_i + roadlength; 
        end 
        set(hcar(i), ...
            'Xdata', [2450-q_4/2, 2450+q_4/2], ...
            'Ydata', [y_i -p_4/2, ...
                      y_i + p_4/2], ...
            'AlphaData', car_alphas{i});
    end 

drawnow limitrate %limerate to skip iterations of loop if laggy 

pause(0.016) %pause for 60FPS

delete(hbike);
delete(hbluecar); 
end 
%%use same but offset? for other cars. 





