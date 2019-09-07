
jointidcs = 1:9;
bodies = create_worm(length(jointidcs)+1,1.5,[0,0]);


calc_pos_relative_angle = zeros(1,length(jointidcs));

figure(3);
xlim([-15 15]);
ylim([-15 15]);
ax = gca;
set(ax,'XTick',[-15:1:15]);
set(ax,'YTick',[-15:1:15]);

bodies = bodymovement(bodies, calc_pos_relative_angle);
printworm(bodies, 3);


%calc_pos_relative_angle(1) = -pi/2;
%bodies = bodymovement(bodies, calc_pos_relative_angle);
%printworm(bodies, 3);
% calc_pos_relative_angle(2) = -pi/2;
% bodies = bodymovement(bodies, calc_pos_relative_angle);
% printworm(bodies, 3);
% calc_pos_relative_angle(5) = -pi/2;
% bodies = bodymovement(bodies, calc_pos_relative_angle);
% printworm(bodies, 3);
% calc_pos_relative_angle(9) = -pi/2;
% bodies = bodymovement(bodies, calc_pos_relative_angle);
% printworm(bodies, 3);

% calc_pos_relative_angle(1) = pi/2;
% bodies = bodymovement(bodies, calc_pos_relative_angle);
% printworm(bodies, 3);
% calc_pos_relative_angle(2) = pi/2;
% bodies = bodymovement(bodies, calc_pos_relative_angle);
% printworm(bodies, 3);
% calc_pos_relative_angle(5) = pi/2;
% bodies = bodymovement(bodies, calc_pos_relative_angle);
% printworm(bodies, 3);
% calc_pos_relative_angle(9) = pi/2;
% bodies = bodymovement(bodies, calc_pos_relative_angle);
% printworm(bodies, 3);


% calc_pos_relative_angle(5) = +pi/2;
% bodies = bodymovement(bodies, calc_pos_relative_angle);
% printworm(bodies, 3);
% calc_pos_relative_angle(5) = +pi/2;
% bodies = bodymovement(bodies, calc_pos_relative_angle);
% printworm(bodies, 3);


% calc_pos_relative_angle(2) = pi/2;
% bodies = bodymovement(bodies, calc_pos_relative_angle);
% printworm(bodies, 3);
% calc_pos_relative_angle(2) = 0;
% calc_pos_relative_angle(8) = -pi/2;
% bodies = bodymovement(bodies, calc_pos_relative_angle);
% printworm(bodies, 3);


% calc_pos_relative_angle(8) = pi/2;
% bodies = bodymovement(bodies, calc_pos_relative_angle);
% printworm(bodies, 3);
% 
% calc_pos_relative_angle(8) = -pi/2;
% bodies = bodymovement(bodies, calc_pos_relative_angle);
% printworm(bodies, 3);

calc_pos_relative_angle(2) = pi/2;
calc_pos_relative_angle(8) = pi/2;
bodies = bodymovement(bodies, calc_pos_relative_angle);
printworm(bodies, 3);


calc_pos_relative_angle(2) = -pi/2;
calc_pos_relative_angle(8) = -pi/2;
bodies = bodymovement(bodies, calc_pos_relative_angle);
printworm(bodies, 3);


