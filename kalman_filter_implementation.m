%% KAlMAN filter basic implementation

%% step 1 : general parameters
del_t = 0.1;
num_points = 100;

%noice variances
%process noise
noise_process1 = wgn(1,num_points , 0.0001 , 'linear');
noise_process2 = wgn(1,num_points , 0.0001 , 'linear');
noise_process3 = wgn(1,num_points , 0.0001 , 'linear');
noise_process4 = wgn(1,num_points , 0.0001 , 'linear');

%process noice covariance initilize
epsilent_var =  [0.0001 0 0 0; 
                0 0.0001 0 0; 
                0 0 0.0001 0;
                0 0 0 0.0001];

%measuremnt noise
noise_measure1 = wgn(1,num_points , 0.001 , 'linear');
noise_measure2 = wgn(1,num_points , 0.001 , 'linear');

%measurement noise covariance initialize
delta_var =  [0.001 0;
              0 0.001];

%initilize position/vel for moving object
obj_x_pos = 1+noise_process1(1);
obj_y_pos = 1+noise_process2(1);
obj_x_vel = 1+noise_process3(1);
obj_y_vel = 1+noise_process4(1);

%initilize measurement for moving object
Z = [obj_x_pos ; obj_y_pos] + [noise_measure1(1);noise_measure2(1)];
%% step2 : Kalman parameters

%state matrix
A = [ 1 0 del_t 0;
      0 1 0 del_t;
      0 0 1 0;
      0 0 0 1];

%measurement/output matrix
C = [1 0 0 0; 0 1 0 0];

%no B matrix since we do not involve control input

%state vector initialze - mean of state vectors are used
u = [obj_x_pos ; obj_y_pos;obj_x_vel(1); obj_y_vel(1) ];

%covarice matrixinitilze(since we know the intial points no uncertatinity
sigma = [0 0 0 0;
         0 0 0 0;
         0 0 0 0;
         0 0 0 0];

%% plot and run algorithm

figure;
hold on;
for i=1:num_points-1

    %calculate new position and velocities
    pre_obj_x_pos = obj_x_pos;
    pre_obj_y_pos = obj_y_pos;
    obj_x_pos = obj_x_vel*del_t + pre_obj_x_pos + noise_process1(i+1);
    obj_y_pos = obj_y_vel*del_t + pre_obj_y_pos + noise_process2(i+1);
    obj_x_vel = obj_x_vel + noise_process3(i+1);
    obj_y_vel = obj_y_vel + noise_process4(i+1);
    
    %plot real moving path
    plot([pre_obj_x_pos ,obj_x_pos] , [pre_obj_y_pos ,obj_y_pos],'b');
    
    %save previous states fro plotting purposes
    pre_u = u;
    pre_Z = Z;
    pre_u_bar = u_bar;
    
    %update sensor readings with noise
    Z = [obj_x_pos ; obj_y_pos] + [noise_measure1(i+1);noise_measure2(i+1)];
    
    %Kalman filter
    u_bar = A*u;
    sigma_bar = A*sigma*A' + epsilent_var;
    K = sigma_bar*C'*inv(C*sigma_bar*C'+delta_var);
    u = u_bar + K*(Z -C*u_bar);
    sigma = (eye(4) - K*C)*sigma_bar;
    
    %plot sensor mean and state mean
    plot([pre_u(1),u(1)] , [pre_u(2),u(2)],'r' );
    plot([pre_Z(1),Z(1)] , [pre_Z(2),Z(2)],'g' );
     
    pause(0.05);
end






























