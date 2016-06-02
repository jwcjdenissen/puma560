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

%% Set joints to zero

joints_actuated = [2 3 4 5];
joints_desired_pos = [0 0 0 0];

clear topic_name 

for i = 1:length(joints_actuated)
    topic_name = puma560(joints_actuated(i)).control_input_topic;
    pub = rospublisher(topic_name, rostopic('type', topic_name));
    msg = rosmessage(rostopic('type', topic_name));
    msg.Data = joints_desired_pos(i);
    pause(1);
    send(pub,msg)
end

%%

Ts=0.0005;
open('puma560_simulink')
sim('puma560_simulink')

% open('pendulumtest_simulink')
% sim('pendulumtest_simulink')

%% Generate data
u=input.Data;
y=output.Data;

system=iddata(y,u,Ts);
system_normalized=detrend(system);

w = linspace(0.001,1000,2048);
sys_np=spa(system,[],w);

bode(sys_np)