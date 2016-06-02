%% Initialize robot

clear all
close all
clc

puma560(1).joint = 'joint_1';
puma560(1).control_input_topic = '/puma560/joint_1_effort_controller/command';
puma560(1).state_topic = '/puma560/joint_states';

puma560(2).joint = 'joint_2';
puma560(2).control_input_topic = '/puma560/joint_2_position_controller/command';

puma560(3).joint = 'joint_3';
puma560(3).control_input_topic = '/puma560/joint_3_position_controller/command';

puma560(4).joint = 'joint_4';
puma560(4).control_input_topic = '/puma560/joint_4_position_controller/command';

puma560(5).joint = 'joint_5';
puma560(5).control_input_topic = '/puma560/joint_5_position_controller/command';


%%
Ts=0.001;

u=input.Data;
y=output.Data;

system=iddata(y,u,Ts);
systemN=detrend(system);

w = linspace(0.1,100,2048);
sys_np=spa(systemN,[],w);

%% Subscribe

% Retrieve joint data from ROS
clear current_state joints_actuated joints_current_pos joint_desired_pos

current_state = receive(rossubscriber(puma560(1).state_topic, ...
    rostopic('type', puma560(1).state_topic)));

joints_actuated = [2 3 4 5];
joints_desired_pos = [0 0 0 0];

joints_current_pos = zeros(length(joints_actuated),1);
vel_traj_bc = [0 0];
time_traj = 0:0.01:1;
joint_traj =  zeros(length(joints_actuated),length(time_traj));
for i = 1:length(joints_actuated)
    joints_current_pos(i) = current_state.Position(joints_actuated(i));
    pos_traj_bc = [joints_current_pos(i) joints_desired_pos(i)];
    joint_traj(i,:) =  spline([0 1],[vel_traj_bc(1) pos_traj_bc vel_traj_bc(2)], time_traj);
end

%% Publish 

clear topic_name 
i = 1;
tic
%Create the publish and message object
for j = 1:length(time_traj)
    for i = 1:length(joints_actuated)
        topic_name = puma560(joints_actuated(i)).control_input_topic;
        pub = rospublisher(topic_name, rostopic('type', topic_name));
        msg = rosmessage(rostopic('type', topic_name));
        msg.Data = joint_traj(i, j);
        pause(0.1);
        send(pub,msg)
    end
end

toc

