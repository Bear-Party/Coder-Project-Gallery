
%--------------------------------------------------------------------------
% File Name: 倒立摆仿真实验
% Author: 方仕展
% Date: 2024/6/22
% MATLAB Version：2023b(simulink版本对不上无法自动使用，文件夹里面提供了其余版本，需要手动打开)
% Version: 1.4
% Description: 倒立摆仿真实验
% Modified：
%  1、2024/6/22 完成倒立摆数学模型建模、LQR控制、MPC控制
%  2、2024/6/23 多刚体仿真模块设计、RBF控制神经网络控制
%  3、2024/7/4-7/5 实现基于DDPG倒立摆控制 整理代码，添加注释
% Attention：
%      1、simulink文件自动打开的是Trolley_Pendulum_RBF_Supervisory.slx,为2023b版本matlab，
%           版本过低无法自动打开，请手动打开文件夹中另存的低版本后手动运行.m.slx文件
%      2、simulink文件需要使用到.m文件中参数，因而需要先运行.M文件，再运行simulink文件，否则会报错
%      3、K矩阵在.M文件中未进行取反、因而在simnulink文件中需要使用-K
%      4、 draw_picture_state用于控制是否绘图，1为绘制
%      
%--------------------------------------------------------------------------

%%
clc;
clear ;
close all;

draw_picture_state = 0;
%%
disp("**********************进行系统建模************************");
syms M m g 
syms J L b
syms F N P
syms x x_dt x_ddt 
syms xita xita_dt xita_ddt
syms fai fai_dt fai_ddt

%建立两组运动方程
sport_equation_set1 = [M*x_ddt == F - b*x_dt - N, ...
        N == m*x_ddt + m*L*xita_ddt*cos(xita) - m*L*xita_dt^2*sin(xita)];
sport_equation_set2 = [P - m*g == -m*L*xita_ddt*sin(xita) - m*L*xita_dt^2*cos(xita),...
        P*L*sin(xita) - N*L*cos(xita) == J*xita_ddt];
%获取中间变量
N_ = solve(sport_equation_set1(2), N); 
P_ = solve(sport_equation_set2(1), P); 

%获得最终运动方程
soprt_equation1 = simplify(subs(sport_equation_set1(1), N, N_)); 
sport_equation2 = simplify(subs(sport_equation_set2(2), [P, N], [P_, N_])); 

%线性化
spor_equation1_linear = subs(soprt_equation1, [cos(xita), xita_dt^2, xita_ddt], [-1, 0, fai_ddt]);
spor_equation2_linear = subs(sport_equation2, [xita_ddt, sin(xita), cos(xita)], [fai_ddt, fai, -1]);

%转换为状态空间表达式
x_ddt_ = solve(spor_equation1_linear, x_ddt); 
fai_ddt_ = solve(subs(spor_equation2_linear,x_ddt,x_ddt_), fai_ddt); 
decoupling_state(1) = diff(fai_ddt_, x_ddt); 
x_ddt_ = solve(subs(spor_equation1_linear,fai_ddt,fai_ddt_), x_ddt);
decoupling_state(2) = diff(x_ddt_, fai_ddt); 
disp("解耦状态");
disp(decoupling_state);

X_dot = [x_dt, x_ddt_, fai_dt, fai_ddt_];
X = [x, x_dt, fai, fai_dt];
U = F;
Y = [x, x_dt, fai, fai_dt];

A_syms = jacobian(X_dot,X);
B_syms = jacobian(X_dot,U);
C_syms = eye(4);
D_syms = 0;

%% 定义相关参数
M_ = 1.096;
m_ = 0.109; 
g_ = 9.8; 
J_ = 0.0034;
L_ = 0.25; 
b_ = 0.1; 
Time_Step = 0.03;
Time = 30;

%% 
% 获取离散状态空间方程
m_A_continue = double(subs(A_syms, [M, m, g, J, L, b], [M_, m_, g_, J_, L_, b_]));
m_B_continue = double(subs(B_syms, [M, m, g, J, L, b], [M_, m_, g_, J_, L_, b_]));
m_C_continue = C_syms;
m_D_continue = D_syms;

%建立系统
state_space_continue = ss(m_A_continue, m_B_continue, m_C_continue, m_D_continue);
state_space_discetization = c2d(state_space_continue, Time_Step, 'zoh');
m_A_disc = state_space_discetization.A;
m_B_disc = state_space_discetization.B;
m_C_disc = state_space_discetization.C;
m_D_disc = state_space_discetization.D;
disp(state_space_continue);

%%
disp("**********************进行LQR控制************************");
%设置LQR矩阵
Q = diag([400 10 500 10]);
R = 0.4;

%使用LQR计算K
[K, P] = dlqr(m_A_disc,m_B_disc,Q,R);

X_tar = [0; 0; 0; 0];

if(X_tar(2) ~= 0)
    K(1) = 0;
end

%计算加入反馈后的状态空间方程矩阵
m_A_disc_feed = m_A_disc - m_B_disc * K;
m_B_disc_feed = m_B_disc;
m_C_disc_feed = m_C_disc;
m_D_disc_feed = m_D_disc;

sys_feedback_LQR = ss(m_A_disc_feed, ...
                        m_B_disc_feed, ...
                        m_C_disc_feed, ...
                        m_D_disc_feed);

%%
%设置离散时间及步长
T = 0 : Time_Step : Time;

%设置系统初始状态
X0 = [2; 0; 0; 0];
X_err = X_tar - 0;

%设置输入
U = ones(size(T));
U_LQR = K * X_err * ones(size(T));

%使用lsim求离散系统在U输入下的响应
[Y_op, X_op] = dlsim(m_A_disc, m_B_disc, m_C_disc, m_D_disc, U, X0);
[Y_LQR_cl, X_LQR_cl] = dlsim(m_A_disc_feed, m_B_disc_feed, m_C_disc_feed, m_D_disc_feed, U_LQR, X0);

if draw_picture_state
%绘制图案
    figure
    
        subplot(2,1,1);
        
            plot(T, X_LQR_cl); % 闭环响应
            legend('x1', 'x2', 'x3', 'x4');
            xlabel('Time');
            ylabel('State Response');
            title('LQR控制下系统响应');
            axis([0 15 -10 15]);
        
        subplot(2,1,2);
        
            plot(T, X_op); % 开环响应
            legend('x1', 'x2', 'x3', 'x4');
            xlabel('Time');
            ylabel('State Response');
            title('无LQR控制下系统响应');
            axis([0 15 -10 15]);
end

%%  
disp("**********************进行MPC控制************************");  
  
% 获取离散化系统矩阵A和B的维度   
n = size(m_A_disc,1); % 状态变量的数量  
p = size(m_B_disc,2); % 控制输入的数量  
  
% 定义权重矩阵  
Q = diag([1000 50 500 50]);  
  
% F通常用于终端状态的惩罚，这里简化为与Q相同    
F = Q;  
  
% R是控制输入的惩罚权重矩阵  
R = 0.1;  
  
% 计算总的控制步数  
% Time是总的控制时间  
% Time_Step是每一步的时间间隔  
k_steps = Time / Time_Step;  
  
% 初始化状态序列X_K和控制序列U_K  
% X_K存储从当前时刻开始到未来k_steps步的状态预测  
% U_K存储对应的控制输入  
X_K = zeros(n,k_steps); % 初始化为全零矩阵  
X_K(:,1) = [0.1; 0; 0; 0]; % 假设初始状态为[0.1, 0, 0, 0]  
U_K = zeros(p,k_steps); % 控制输入初始化为全零  
  
% N是预测时域的长度，即MPC算法向前预测的步数  
N = 20;  
  
% 调用MPC_Matrices函数计算MPC问题所需的扩展矩阵E和H  
[E, H] = MPC_Matrices(m_A_disc, m_B_disc, Q, R, F, N);

if draw_picture_state
    for k = 1 : k_steps
        U_K(:,k) = Prediction(X_K(:,k),E,H,N,p);
        X_K(:,k+1)=(m_A_disc*X_K(:,k)+m_B_disc*U_K(:,k));
    end
    %%
    % 绘图
    figure
    plot(T, X_K'); 
    legend('x1', 'x2', 'x3', 'x4');
    xlabel('Time');
    ylabel('State Response');
    title('MPC控制下系统响应');
end

%%
disp("**********************进行DDPG控制************************");
%% 设置和配置强化学习环境  
%打开simulink模型
try
    mdl = 'Trolley_Pendulum_RBF_Supervisory';  
    open_system(mdl)  
catch ME
    % 异常处理代码
    fprintf('捕获到异常：simulink模型未能正常打开');
end

  
% 定义四个状态变量 
obsFeat = 4;  
obsInfo = rlNumericSpec([obsFeat 1]);   
obsInfo.Name = 'observations';  
obsInfo.Description = 'position x, time derivative position x dot, deflection theta, time derivative deflection theta dot';    
numObservations = obsInfo.Dimension(1);  
  
% 定义动作信息，这里动作只有一个力
actInfo = rlNumericSpec([1 1]);  
actInfo.Name = 'force';  
actInfo.LowerLimit = -50;  % 动作的最小值
actInfo.UpperLimit = 50;   % 动作的最大值 
numActions = actInfo.Dimension(1);  
  
% 创建一个rlSimulinkEnv对象，配置Simulink模型与RL agent之间的接口  
% 指定Simulink模型名称、RL agent的子系统名称（即Simulink中负责接收动作和输出观察的子系统）  
% 以及观察信息和动作信息的规格  
env = rlSimulinkEnv('Trolley_Pendulum_RBF_Supervisory',...  
                    'Trolley_Pendulum_RBF_Supervisory/偏离平衡点控制器（DDPG）/RL Agent',...  
                    obsInfo,actInfo);  
  
% 设置Simulink环境的仿真步长和总仿真时间  
Ts =  0.03;    
Tf = 20;       
  
% 设置随机数生成器的种子，以确保结果的可重复性  
rng(0)

%% 创建DDPG agent  
% 定义critic网络的state路径  
% 输入层：接收状态信息，不进行归一化  
% 第一个全连接层：50个神经元  
% ReLU激活层  
% 第二个全连接层：25个神经元  
statePath = [  
    featureInputLayer(numObservations,'Normalization','none','Name','State')  
    fullyConnectedLayer(50,'Name','CriticStateFC1')  
    reluLayer('Name','CriticRelu1')  
    fullyConnectedLayer(25,'Name','CriticStateFC2')];  
  
% 定义critic网络的action路径  
% 输入层：接收动作信息，不进行归一化  
% 第一个全连接层：25个神经元  
actionPath = [  
    featureInputLayer(numActions,'Normalization','none','Name','Action')  
    fullyConnectedLayer(25,'Name','CriticActionFC1')];  
  
% 定义critic网络的common路径  
% 将状态路径和动作路径的输出相加  
% ReLU激活层  
% 输出层：单个神经元，输出Q值  
commonPath = [  
    additionLayer(2,'Name','add')  
    reluLayer('Name','CriticCommonRelu')  
    fullyConnectedLayer(1,'Name','CriticOutput')];  
  
% 创建一个层图对象，并逐层添加state路径、action路径和common路径  
criticNetwork = layerGraph();  
criticNetwork = addLayers(criticNetwork,statePath);  
criticNetwork = addLayers(criticNetwork,actionPath);  
criticNetwork = addLayers(criticNetwork,commonPath);  
% 连接state路径和action路径到common层  
criticNetwork = connectLayers(criticNetwork,'CriticStateFC2','add/in1');  
criticNetwork = connectLayers(criticNetwork,'CriticActionFC1','add/in2');  
  
% 如果设置了draw_picture_state为true，则绘制critic网络图
if draw_picture_state  
    figure  
    plot(criticNetwork)  
end  
  
% 设置critic网络的选项，如学习率和梯度阈值  
criticOpts = rlRepresentationOptions('LearnRate',1e-03,'GradientThreshold',1);  
% 创建critic表示，指定输入输出名称  
critic = rlQValueRepresentation(criticNetwork,obsInfo,actInfo,'Observation',{'State'},'Action',{'Action'},criticOpts);  
  
% 定义actor网络  
% 输入层：接收状态信息，不进行归一化  
% Tanh激活层  
% 输出层：动作数量个神经元，输出动作  
actorNetwork = [  
    featureInputLayer(numObservations,'Normalization','none','Name','State')  
    fullyConnectedLayer(3, 'Name','actorFC')  
    tanhLayer('Name','actorTanh')  
    fullyConnectedLayer(numActions,'Name','Action')  
    ];  
  
actorNetworkGraph = layerGraph(actorNetwork); 

% 如果设置了draw_picture_state为true，则绘制Actor网络图
if draw_picture_state  
    figure  
    plot(actorNetworkGraph)  
end  

% 设置actor网络的选项  
actorOptions = rlRepresentationOptions('LearnRate',1e-04,'GradientThreshold',1);  
% 创建actor表示，指定输入输出名称  
actor = rlDeterministicActorRepresentation(actorNetwork,obsInfo,actInfo,'Observation',{'State'},'Action',{'Action'},actorOptions);  
  
% 创建DDPGagent的选项  
% 指定仿真步长、目标网络平滑因子、折扣因子、小批量大小和经验回放缓冲区长度  
agentOpts = rlDDPGAgentOptions(...  
    'SampleTime',Ts,...  
    'TargetSmoothFactor',1e-3,...  
    'DiscountFactor',1.0, ...  
    'MiniBatchSize',64, ...  
    'ExperienceBufferLength',1e6);   
% 设置噪声选项，包括噪声方差和方差衰减率  
agentOpts.NoiseOptions.Variance = 0.3;  
agentOpts.NoiseOptions.VarianceDecayRate = 1e-5;  
  
%%  
% 标志位，确认是否从已保存的模型继续训练  
resumeTraining = false;  
  
% 选择1继续训练模型
if resumeTraining  
    sprintf('- Resume training of: %s', 'SwingUpDDPG.mat');  
    load('SwingUpDDPG.mat','agent');  
else  
    % 创建一个新的DDPGagent  
    agent = rlDDPGAgent(actor, critic, agentOpts);  
end  
  
% 定义训练的最大episodes
maxepisodes = 1500;  
  
% 定义每集的最大步数。这里假设Tf是总时间，Ts是每步的时间间隔  
% 注意：Tf和Ts需要在之前被定义  
maxsteps = ceil(Tf/Ts);  
  
% 设置保存agent的目录路径  
% 使用了pwd（当前工作目录）和字符串拼接来构建完整路径  
agentdir = pwd + "/Pendulum on Cart Repository" + "/RL Agents";  

% 配置训练选项  
trainOpts = rlTrainingOptions(...  
    'MaxEpisodes', maxepisodes, ...% 最大训练集数  
    'MaxStepsPerEpisode', maxsteps,... % 每集最大步数  
    'ScoreAveragingWindowLength', 20,... % 分数平均窗口长度  
    'SaveAgentCriteria', 'EpisodeReward',... % 保存agent的条件（基于每episode的奖励）  
    'SaveAgentValue', 8,... % 当某集奖励达到此值时保存agent  
    'SaveAgentDirectory', 'Pendulum on Cart Repository/RL Agents/SwingUpDDPG.mat',... % 保存agent的目录  
    'Verbose', true, ... % 训练时是否显示详细信息  
    'Plots', 'training-progress',...  % 显示训练进度图  
    'StopTrainingCriteria', 'AverageReward',...  % 停止训练的条件（基于平均奖励）  
    'StopTrainingValue', 1.0000e+30); % 当平均奖励达到此值时停止训练
  
% 定义一个标志，用于控制是否执行训练过程  
doTraining = false;  
  
% 根据doTraining的值决定是训练新模型还是加载已保存的模型进行测试  
if doTraining  
    % 执行训练，并保存训练统计信息和最终训练的agent  
    trainingStats = train(agent, env, trainOpts); % 注意：env需要在之前被定义  
    save('SwingUpDDPG.mat', 'agent'); % 保存训练后的agent  
else  
    % 加载预训练的“最佳”agent  
    load('best\Agent111.mat', 'saved_agent');  
    % 使用加载的agent进行仿真  
    % 注意：'sim'函数和'Trolley_Pendulum_RBF_Supervisory'模型需要事先定义和配置  
    % 'StopTime'是传递给仿真模型的参数，用于控制仿真时长  
    try
        simOut = sim('Trolley_Pendulum_RBF_Supervisory', 'StopTime', '10');
    catch ME
        % 异常处理代码
        fprintf('捕获到异常：simulink模型未能正常运行');
    end
end
%%
disp("**********************实验结束************************");

%%
function u_k= Prediction(x_k,E,H,N,p)
    U_k = zeros(N*p,1); % NP x 1
    U_k = quadprog(H,E*x_k); 
    u_k = U_k(1:p,1); % 取第一个结果
end 

%%
function [E , H]=MPC_Matrices(A,B,Q,R,F,N)
    n=size(A,1);% A 是 n x n 矩阵, 得到 n
    p=size(B,2);% B 是 n x p 矩阵, 得到 p
    M=[eye(n);zeros(N*n,n)]; % 初始化 M 矩阵. M 矩阵是 (N+1)n x n的，它上面是 n x n 个 "I", 这一步先把下半部 % 分写成 0 
    C=zeros((N+1)*n,N*p); % 初始化 C 矩阵, 这一步令它有 (N+1)n x NP 个 0
    
    % 定义M 和 C 
    tmp=eye(n);%定义一个n x n 的 I 矩阵
    
    %　更新Ｍ和C
    for i=1:N % 循环，i 从 1到 N
        rows =i*n+(1:n); %定义当前行数，从i x n开始，共n行 
        C(rows,:)=[tmp*B,C(rows-n, 1:end-p)]; %将c矩阵填满
        tmp= A*tmp; %每一次将tmp左乘一次A
        M(rows,:)=tmp; %将M矩阵写满
    end
    
    % 定义Q_bar和R_bar
    Q_bar = kron(eye(N),Q);
    Q_bar = blkdiag(Q_bar,F);
    R_bar = kron(eye(N),R);
    
    % 计算G, E, H
    G=M'*Q_bar*M; % G: n x n
    E=C'*Q_bar*M; % E: NP x n
    H=C'*Q_bar*C+R_bar; % NP x NP 
end 
