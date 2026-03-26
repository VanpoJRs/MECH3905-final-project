


%% animated obstacles function 
close all
clear all
clc 

%%animated obstacles 
%% 

%call the background 
[j, ball_image, alpha] = figure_setup_bike();

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

[bluecar_big,~, bluecar_alpha_big] =imread('yellow_car_v2.png');
bluecar_big=flipud(bluecar_big); 
bluecar_alpha_big=flipud(bluecar_alpha_big);

bluecar= imresize(bluecar_big, scale_bluecar); 
bluecar_alpha= imresize(bluecar_alpha_big, scale_bluecar); 
[p,q,~]= size(bluecar); 

%follower cars 
[car2, ~, alpha2]=imread('yellow_car_v2.png'); 
[car3, ~, alpha3]=imread('yellow_car_v2.png');
[car4, ~, alpha4]=imread('yellow_car_v2.png'); 

%cell array 
car_imgs= {car2, car3, car4};
car_alphas= {alpha2, alpha3, alpha4}; 

numCars = length(car_imgs); 
hcar = gobjects(1, numCars); %gobjects initilizes space for graphics object array i think 

%vector for y position offset. Quantity of pixels away from lead car. 
offsets= [-1.5*p, -3*p, -5*p]; 

%% inital pos and equations 

 %intial y equations for bike 
y_bike= 80; %inital y
dy_bike= 20; %biker speed 

%inital equations for bluecar 
y_bluecar= 80; 
dy_bluecar= 10;
x_bluecar= 2450; 


%intial pos of bike 
 hbike=image(bike, ...
    'Xdata', [2320-o/2, 2320+o/2], ...
    'Ydata', [2160-l/2, 2160+l/2], ...
    'AlphaData', bike_alpha); %%coordinates (x,y) when y=0.


%inital pos of bluecar 
hbluecar =image(bluecar, ...
    'Xdata', [2450-q/2,2450+q/2], ...
    'Ydata', [0-p/2, 0+p/2], ...
    'AlphaData', bluecar_alpha); 

%initial pos for follower cars 
for i= 1:numCars
    hcar(i)=image(car_imgs{i}, ...
        'Xdata', [2450-q/2, 2450+q/2], ...
        'Ydata', [y_bluecar + offsets(i) -p/2, ...
                  y_bluecar + offsets(i) + p/2], ...
        'AlphaData', car_alphas{i}); 
end 

%% equations of motion defined 

%equation of motion for bike
x_bike=2300+20*cos(0.7*y_bike/50);

%equation of motion for blue car 
y_bluecar = y_bluecar + dy_bluecar;

%boundaries should be the same for all 
ymax= 2160; %top boundary on figure window pop up
ymin=-50;  %bottom boundary on figure window pop up 


%% For turn around 
%trying to flip the bike (currently not working. Works along lines 
bike_northbound=bike; 
bike_alpha_northbound=bike_alpha; 

bike_southbound=imrotate(bike, 180); 
bike_alpha_southbound=imrotate(bike_alpha, 180);

%% Main loop 
for k=1:n
    y_old_bike=y_bike; %sotring previous y 
    y_bike=y_bike+dy_bike; %update y 
  

    y_old_bluecar= y_bluecar; 
    y_bluecar= y_bluecar + dy_bluecar;
    x_bluecar=2450; %constant because going in straight line 


      %bike motion 
      x_bike=2320+20*cos(0.7*y_bike/50);
       
      %bluecar motion 
      y_bluecar= y_bluecar + dy_bluecar; 
      
     

    %boundary check 
    if y_bike>ymax || y_bike< ymin
        dy_bike= -dy_bike;    
    end 


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


%update bike image 
% delete(hbike); 
% hbike=image(bike, ...
% 'Xdata', [x_bike-o/2, x_bike+o/2], ...
% 'Ydata', [y_bike-l/2, y_bike+l/2], ...
% 'AlphaData', bike_alpha); 

%update bluecar image 
delete(hbluecar); 
hbluecar =image(bluecar, ...
    'Xdata', [x_bluecar-q/2, x_bluecar+q/2], ...
    'Ydata', [y_bluecar-p/2, y_bluecar+p/2], ...
    'AlphaData', bluecar_alpha); 

%follower car 
    for i=1:numCars
        y_i = y_bluecar + offsets(i); 

        set(hcar(i), ...
            'Xdata', [2450-q/2, 2450+q/2], ...
            'Ydata', [y_i -p/2, ...
                      y_i + p/2], ...
            'AlphaData', car_alphas{i});
    end 

drawnow limitrate %limerate to skip iterations of loop if laggy 

pause(0.016) %pause for 60FPS

delete(hbike);   
end 
%%use same but offset? for other cars. 





