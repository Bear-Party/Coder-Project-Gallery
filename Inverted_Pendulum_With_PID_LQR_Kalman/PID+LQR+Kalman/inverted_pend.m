
%--------------------------------------------------------------------------
% File Name: 倒立摆仿真实验
% Author: 方仕展
% Date: 2023/12/31
% MATLAB Version：2023b
% Version: 1.6
% Description: 倒立摆仿真实验
% Modified：
%   2023/11/2 1.0 建立状态空间表达式并离散化，加入lqr控制，通过draw_pend函数实现可视化
%   2023/11/18 1.1 根据计控建模方式重新建立状态空间表达式，加入系统能控性观测并整理代码
%   2023/11/24 1.1 根据现有建模方式，编写了Matlab_Appdesigner上位机
%   2023/12/4 1.2 增加系统特征方程计算，并进行系统性能参数获取分析
%   2023/12/9 1.3 将运动方程进行拉普拉斯变换，获得位置、角度传递函数，Matlab增加传递函数计算，
%                 将计算获得的传递函数导入simulink的PID模型
%   2023/12/12 1.4 simulink加入卡尔曼滤波
%   2023/12/18 1.5 搭建simscape多刚体仿真模块
%   2023/12/21 1.6 自主搭建卡尔曼滤波模块
% Attention：
%   1、simulink文件都使用了.m文件保存到基础工作区的变量，记得先运行.m文件，否则simulink会报错说找不到变量，
%      此外可以直接在.m文件中进行修改
%   2、代码分块，先整体运行一次，再根据需要运行节
%   3、状态空间方程直接使用了崔老师计算的结果，未使用模型数据，不过也有写了相关矩阵，可以试一试
%   4、simulink默认使用simscape多刚体仿真模块，若只需要使用状态空间表达式进行控制，
%      可以使用Ctrl+shift+X将多刚体仿真模块注释调，再将状态空间表达式部分用同样方式取消注释
%   5、运行.m文件前需要将PID_Ctr.slx打开，代码中需要使用到该文件的PID参数
%   6、默认不开启画图功能，可以修改draw_state和draw_car_state状态为1，再重新运行
%   7、.m文件调用的是2023版本的simulink文件，在438行sim("Inverted_Pendulum_2023b.slx")处，
%      若是低于该版本，需要手动在simulink中运行文件，压缩包中有保存不同版本（不同后缀）的simulink文件，
%      选择低于自身Matlab版本的simulink文件运行即可
%--------------------------------------------------------------------------
%%
clc;
clear;
%%
%是否进行画图
draw_state = 0;

%是否绘制小车
draw_car_state = 0;
%%
%%%%%%%%%%%%%%%%%%%%%%%模型参数设定%%%%%%%%%%%%%%%%%%%%%
%小车质量
M = 1.096;
%摆杆质量
m = 0.109;
%小车摩擦系数0
b = 0.1;
%摆杆转动惯量
I = 0.0034;
%重力加速度
g = 9.8;
%仿真步长
Time_step = 0.0001;
%仿真时间
Time = 20;
%摆杆转动轴心到杆质心之间的距离
L = 0.25;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%PID控制%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%%%%%%%%%%%%%%%传递函数%%%%%%%%%%%%

% 定义 s 为拉普拉斯变换的变量  
s = tf('s');  

% 位置传递函数  
position_tf = (((I + m * L^2) ) * s^2 - (m * g * L)) / ...
            (((M + m) * (I + m * L^2) - (m * L)^2)*s^4 + ...
            (b * (I + m * L^2)) * s^3 - ...
            ((M + m) * m * g * L) * s^2 - ...
            b * m * g * L * s );  
  
% 计算分子和分母的系数，保存到工作空间便于PID模型调用
%分子
num_position = [((I + m * L^2)), 0, - (m * g * L)];  
%分母
den_position = [((M + m) * (I + m * L^2) - (m * L)^2), (b * (I + m * L^2)), - ((M + m) * m * g * L), - b * m * g * L , 0];  
  
% 角度传递函数
angle_tf = (m * L * s ) / (((M + m) * (I + m * L^2) - (m * L)^2) * s^3 + ...
                (b * (I + m * L^2)) * s^2 - ...
                ((M + m) * m * g * L) * s - ...
                 b * m * g * L );  
  
% 计算分子和分母的系数，保存到工作空间便于PID模型调用
%分子
num_angle = [m * L, 0];  
%分母
den_angle = [((M + m) * (I + m * L^2) - (m * L)^2), (b * (I + m * L^2)), - ((M + m) * m * g * L), - b * m * g * L];  

% 将小车和吊索的传递函数组合成一个 2x1 的传递函数 sys_tf
pos_ang_tf = [position_tf ; angle_tf];
  
%离散化
angle_tf_disc = c2d(angle_tf, Time_step, 'zoh'); 
num_angle_disc = cell2mat(angle_tf_disc.Numerator);
den_angle_disc = cell2mat(angle_tf_disc.Denominator);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
%%%%%%%%%%%%PID控制器%%%%%%%%%%%
% 定义时间向量 t，用于模拟系统的响应  
 disp("**********************进行PID实验************************");
T_PID = 0:Time_step:2;

% 定义输入信号 u 为单位阶跃信号，即从0开始逐渐增加到1的信号  
U_PID = ones(size(T_PID)); 

% 使用 lsim 函数来模拟系统的响应，输出结果 y 和时间向量 t  
[Y_PID_f,T_PID_f] = lsim(pos_ang_tf,U_PID,T_PID);  

% 使用 plot 函数来绘制系统的响应曲线，其中 title 和 axis 函数分别用来设置图形的标题和坐标轴范围  
if draw_state
    figure   
    plot(T_PID_f,Y_PID_f)   
    title('未添加PID控制器下的阶跃响应')   
    axis([0 1 0 Time])   
    legend('x','phi')  
end
try
    % 设置连续控制器的参数 Kp、Ki 和 Kd
    Kp_con = str2double(get_param('Inverted_Pendulum_2023b/Angle_con_PID','P'));  
    Ki_con = str2double(get_param('Inverted_Pendulum_2023b/Angle_con_PID','I'));  
    Kd_con = str2double(get_param('Inverted_Pendulum_2023b/Angle_con_PID','D'));    
catch
    disp("未打开Inverted_Pendulum_2023b文件，已加载默认连续PID参数");
    Kp_con =   143.8476;  
    Ki_con =   230.4503;  
    Kd_con =    16.9204;    
end
PID_Ctr = pid(Kp_con,Ki_con,Kd_con); 

% 使用 feedback 函数求连续系统添加PID控制器后的响应
angle_tf_OL_PID = series(PID_Ctr, angle_tf);
angle_tf_PID = feedback(angle_tf_OL_PID, 1);

if draw_state
 figure
     impulse(angle_tf_PID,T_PID); 
     title('PID控制器下连续系统的脉冲响应')   
     drawnow
 figure
     step(angle_tf_PID,T_PID);
     title('PID控制器下连续系统的阶跃响应')   
     drawnow
end

try
    % 设置离散控制器的参数 Kp、Ki 和 Kd
    Kp_disc = str2double(get_param('Inverted_Pendulum_2023b/Angle_disc_PID','P'));    
    Ki_disc = str2double(get_param('Inverted_Pendulum_2023b/Angle_disc_PID','I'));  
    Kd_disc = str2double(get_param('Inverted_Pendulum_2023b/Angle_disc_PID','D'));    
catch
    disp("未打开Inverted_Pendulum_2023b文件，已加载默认离散PID参数");
    Kp_disc = 187.6061;  
    Ki_disc = 461.5129;  
    Kd_disc = 19.0656;    
end

PID_Ctr_disc = pid(Kp_disc, Ki_disc, Kd_disc, Ts = Time_step);

% 使用 feedback 函数求离散系统添加PID控制器后的响应
angle_tf_OL_PID_disc = series(PID_Ctr_disc, angle_tf_disc);
angle_tf_PID_disc = feedback(angle_tf_OL_PID_disc, 1);
if draw_state
    figure
        impulse(angle_tf_PID_disc, T_PID);
        title('PID控制器下离散系统的脉冲响应'); drawnow
    figure
        step(angle_tf_PID_disc, T_PID);
        title('PID控制器下离散系统的阶跃响应'); drawnow
        [angle_tf_PID_disc_step_info]= stepinfo(angle_tf_PID_disc);
        fprintf('上升时间：%d\n', angle_tf_PID_disc_step_info.RiseTime);
        fprintf('峰值时间：%d\n', angle_tf_PID_disc_step_info.PeakTime);
        fprintf('调节时间：%d\n', angle_tf_PID_disc_step_info.SettlingTime);
        fprintf('峰值：%d\n', angle_tf_PID_disc_step_info.Peak);
end

%PID控制添加前后极点分析
angle_tf_disc_poles = eig(angle_tf_disc);
if(find(abs(angle_tf_disc_poles) > 1))
    fprintf("未加PID控制器下的力到角度的传递函数是不稳定的\n");
else
    fprintf("未加PID控制器下的力到角度的传递函数是稳定的\n");
end
angle_tf_PID_disc_poles = eig(angle_tf_PID_disc);
if(find(abs(angle_tf_PID_disc_poles) > 1))
    fprintf("PID控制器下的力到角度的传递函数是不稳定的\n");
else
    fprintf("PID控制器下的力到角度的传递函数是稳定的\n");
end
%%
%PID控制下的频域分析
[Gm, Pm, Wcg, Wcp] = margin(angle_tf_OL_PID_disc);
fprintf("相位裕度：%d\n 幅值裕度: %d\n 相位穿越频率: %d\n 幅值穿越频率:%d\n", Gm, Pm, Wcg, Wcp);
% %%
% %PID控制下离散状态空间表达式的频域分析
% [Gm, Pm, Wcg, Wcp] = margin(linsys1(1));
% fprintf("相位裕度：%d\n 幅值裕度: %d\n 相位穿越频率: %d\n 幅值穿越频率:%d\n", Gm, Pm, Wcg, Wcp);
% [Gm, Pm, Wcg, Wcp] = margin(linsys1(2));
% fprintf("相位裕度：%d\n 幅值裕度: %d\n 相位穿越频率: %d\n 幅值穿越频率:%d\n", Gm, Pm, Wcg, Wcp);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 disp("**********************PID实验结束************************");
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%LQR控制%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 disp("**********************进行LQR控制************************");
%%%%%%%%%%%%建立状态空间表达式%%%%%%%%%%%
%分母
p = I * (M + m) + M * m * L^2;
% 状态方程相关矩阵
A_con = [0 1 0 0;
        0 -(I + m * L^2) * b / p, (m^2 * g * L^2) / p, 0;
        0 0 0 1;
        0. -(m * L * b) / p, m * g * L * (M + m) / p, 0];
B_con = [0; (I + m * L^2) / p; 0; m * L / p];
C_con = eye(4);
D_con = [0; 0; 0; 0];
sys_con = ss(A_con, B_con, C_con, D_con);

%开环连续系统离散化
sys_disc = c2d(sys_con, Time_step,'zoh');
A_disc = sys_disc.A;
B_disc = sys_disc.B;
C_disc = sys_disc.C;
D_disc = sys_disc.D;

% %状态方程相关矩阵（外部计算得）
% A = [0  1        0        0;
%     0  -0.0883  0.6293   0;
%     0  0        0        1;
%     0  -0.2357  27.8285  0];
% 
% B = [0; 0.8832; 0; 2.3566];
% 
% C = [1 0 0 0;
%     0 0 1 0];
% 
% D = [0; 0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%%%%%%%%%%%%%%%系统性能分析%%%%%%%%%%%%%%%%%

%稳定性分析
syms S
%系统在离散和连续状态下的极点和特征方程
poles_con = eig(A_con);
characteristic_equation_con = vpa((S - poles_con(1)) * (S - poles_con(2)) * (S - poles_con(3)) * (S - poles_con(4)), 4);
disp('离散系统特征方程为：');
disp(characteristic_equation_con);
if(find(real(poles_con) > 1))
    disp('离散状态下的系统不稳定：');
else
    disp('离散状态下的系统稳定：');
end

poles_disc = eig(A_disc);
characteristic_equation_disc = vpa((S - poles_disc(1)) * (S - poles_disc(2)) * (S - poles_disc(3)) * (S - poles_disc(4)), 4);
disp('连续系统特征方程为：');
disp(characteristic_equation_disc);
if(find(abs(poles_disc) > 0))
    disp('连续状态下的系统不稳定：');
else
    disp('连续状态下的系统稳定：');
end

%能控性和能观测性分析
disp('判断系统能控性和能观性');
%连续系统
% 判断可控性
rank_ctrb_con = rank(ctrb(A_con, B_con));
if rank_ctrb_con == size(A_con, 1)
    disp('连续系统为可控的');
else
    disp('连续系统为不可控的');
end
% 判断可观性
rank_obsv_con = rank(obsv(A_con, C_con));
if rank_obsv_con == size(A_con, 1)
    disp('系统连续为可观的');
else
    disp('系统连续为不可观的');
end

%连续系统
% 判断可控性
rank_ctrb_disc = rank(ctrb(A_disc, B_disc));
if rank_ctrb_disc == size(A_disc, 1)
    disp('离散系统为可控的');
else
    disp('离散系统为不可控的');
end
% 判断可观性
rank_obsv_disc = rank(obsv(A_disc, C_disc));
if rank_obsv_disc == size(A_disc, 1)
    disp('离散系统为可观的');
else
    disp('离散系统为不可观的');
end

%绘图
if draw_state
    %绘制阶跃响应，为发散
    figure
        t = 0:0.2:10; % 时间向量，步长为 0.1
        step(sys_disc, t);
        grid on; title('阶跃响应'); xlabel('Time'); ylabel('Amplitude'); drawnow
    %绘制脉冲响应，为发散
    figure
        t = 0:0.2:10; % 时间向量，步长为 0.1
        impulse(sys_disc, t);
        grid on; title('脉冲响应'); xlabel('Time'); ylabel('Amplitude'); drawnow
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%%%%%%%%%%%%LQR最优控制%%%%%%%%%%%%%

%设置LQR矩阵
Q = diag([400 10 500 10]);
R = 0.4;

%使用LQR计算K
[K, P] = dlqr(A_disc,B_disc,Q,R);

X_tar = [0; 0; 0; 0];

if(X_tar(2) ~= 0)
    K(1) = 0;
end

%计算加入反馈后的状态空间方程矩阵
A_disc_feed = A_disc - B_disc * K;
B_disc_feed = B_disc;
C_disc_feed = C_disc;
D_disc_feed = D_disc;

sys_feedback_LQR = ss(A_disc_feed, ...
                        B_disc_feed, ...
                        C_disc_feed, ...
                        D_disc_feed);
%% 系统性能分析
%系统在离散和连续状态下的极点和特征方程
state_space_poles_disc_feedback_LQR = eig(A_disc_feed);
disp('LQR控制下离散系统极点为：');
disp(state_space_poles_disc_feedback_LQR);
characteristic_equation_con = vpa((S - state_space_poles_disc_feedback_LQR(1)) *...
                                    (S - state_space_poles_disc_feedback_LQR(2)) * ...
                                    (S - state_space_poles_disc_feedback_LQR(3)) * ...
                                    (S - state_space_poles_disc_feedback_LQR(4)), 4);
disp('LQR控制下离散状态空间表达式特征方程为：');
disp(characteristic_equation_con);

if(find(real(state_space_poles_disc_feedback_LQR) > 1))
    disp('LQR控制下离散状态下的系统不稳定：');
else
    disp('LQR控制下离散状态下的系统稳定：');
end

%LQR控制下的频域分析
[Gm, Pm, Wcg, Wcp] = margin(angle_tf_OL_PID_disc);
fprintf("相位裕度：%d\n 幅值裕度: %d\n 相位穿越频率: %d\n 幅值穿越频率:%d\n", Gm, Pm, Wcg, Wcp);

%%
%设置离散时间及步长
T = 0 : Time_step : Time;

%设置系统初始状态

X0 = [2; 0; 0; 0];

X_err = X_tar - 0;

%设置输入
U = ones(size(T));
% U(1) = 0;

U_LQR = K * X_err * ones(size(T));

%使用lsim求离散系统在U输入下的响应
[Y_op, X_op] = dlsim(A_disc, B_disc, C_disc, D_disc, U, X0);
[Y_LQR_cl, X_LQR_cl] = dlsim(A_disc_feed, B_disc_feed, C_disc_feed, D_disc_feed, U_LQR, X0);

if draw_state
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
%绘制小车
if draw_car_state
    figure
    
        for k=1:length(T)
            draw_pend(X_LQR_cl(k, :), m, M, L * 2);
        end
    
    hold on
end
 disp("**********************LQR实验结束************************");

%%
%卡尔曼滤波参数设置
angle_to_radian = pi/180;
Measure_noise_position = 0.02^2;%位置测量存在2cm误差
Measure_noise_speed = 1e-6;%速度测量误差
Measure_noise_angle = (0.2*angle_to_radian)^2;%角度测量存在0.2°误差
Measure_noise_angle_vel = 1e-6;%角速度测量误差
%模型噪声方差
Wk = 0.0000001;
%测量噪声方差
Vk = [Measure_noise_position, ...
            Measure_noise_speed, ...
            Measure_noise_angle, ...
            Measure_noise_angle_vel];

%模型误差
Q_kalman = 0.000001;

%测量误差
R_kalman = [Measure_noise_position, ...
            Measure_noise_speed, ...
            Measure_noise_angle, ...
            Measure_noise_angle_vel];
% %%
% %%运行simulink文件
% try
% sim("Inverted_Pendulum_2023b.slx")
% catch
%     disp("未打开Inverted_Pendulum_2023b文件，请手动打开");
% end

%%
%%%%%%%%%%%%%%%%小车绘制函数%%%%%%%%%%%%%%%%%%%%%%
%画图函数
function draw_pend(y, m, M, L)
    %获取位置、角度数据
    x = y(1);
    th = y(3);
    
    % 车子宽度
    W = 1 * sqrt(M / 5);
    
    % 车子高度
    H = .5 * sqrt(M / 5);
    
    % 轮胎半径
    wr = .2;
    
    % 小球半径
    mr = .3 * sqrt(m);
    
    % 位置高度
    y = wr / 2 + H / 2;
    
    %两个轮子起始点坐标
    w1x = x - 0.9 * W / 2;
    w1y = 0;
    w2x = x + 0.9 * W / 2 - wr;
    w2y = 0;
    
    %杆终点坐标
    px = x + L * sin(th);
    py = y + L * cos(th);
    
    plot([-20 20], [0 0], 'k', 'LineWidth', 2)
    hold on
    
    %绘制小车
    rectangle('Position', [x - W/2, y - H/2, W, H], 'Curvature', 0.1, 'FaceColor', [0.8 1 0.8], 'LineWidth', 1.5)
    rectangle('Position', [w1x, w1y, wr, wr], 'Curvature', 1, 'FaceColor', [0.5 0.5 0.7], 'LineWidth', 1.5)
    rectangle('Position', [w2x, w2y, wr, wr], 'Curvature', 1, 'FaceColor', [0.5 0.5 0.7], 'LineWidth', 1.5)
    
    %绘制摆臂
    plot([x px], [y py], 'k', 'LineWidth', 2)
    
    %绘制
    rectangle('Position', [px - mr/2, py - mr/2, mr, mr], 'Curvature', 1, 'FaceColor', [1 0.1 .1], 'LineWidth', 1.5)
    
    % %   xlim([-10 10]);
    xlim([x-2 x+2]);
    % %   ylim([-2 2.5]);
    %ylim([y-5 y+5]);
    
    axis equal
    
    drawnow
    hold off

end


