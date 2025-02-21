%% 清屏
clear;
close all;
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 仿真时间
Time_step = 0.03;%测试0.02S控制下倒立摆发散，而0.03S不会，因而选择0.03S
%% 第一步，定义状态空间矩阵
M = 1.096;
m = 0.109;
b = 0.1;
I = 0.0034;
g = 9.8;
L = 0.25;
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

n= size (A_disc,1);
p = size(B_disc,2);

%% 定义Q矩阵，n x n 矩阵

Q = diag([500 50 500 50]);

%% 定义F矩阵，n x n 矩阵

F = Q;

%% 定义R矩阵，p x p 矩阵

R=0.1;

%% 定义step数量k

k_steps=500;

%% 定义矩阵 X_K， n x k 矩 阵

X_K = zeros(n,k_steps);

%% 初始状态变量值， n x 1 向量

X_K(:,1) =[0.1;0;0;0];

%% 定义输入矩阵 U_K， p x k 矩阵

U_K=zeros(p,k_steps);

%% 定义预测区间K

N=20;

%% Call MPC_Matrices 函数 求得 E,H矩阵 

[E, H]=MPC_Matrices(A_disc,B_disc,Q,R,F,N);

%% 计算每一步的状态变量的值

for k = 1 : k_steps

    %% 求得U_K(:,k)
    
    U_K(:,k) = Prediction(X_K(:,k),E,H,N,p);
    
    %% 计算第k+1步时状态变量的值
    
    X_K(:,k+1)=(A_disc*X_K(:,k)+B_disc*U_K(:,k));

end
%% LQR控制
%设置LQR矩阵
%Q = diag([400 10 500 10]);
Q = diag([100 10 20 10]);
R = 0.8;

%使用LQR计算K
[K, P] = dlqr(A_disc,B_disc,Q,R);

%% 绘制状态变量和输入的变化

subplot(2, 1, 1);

hold;

for i =1 :size (X_K,1)

    plot (X_K(i,:));

end

legend("pos","speed","angle","angle—vel")
title("系统响应")

hold off;

subplot (2, 1, 2);

hold;

for i =1 : size (U_K,1)

    plot (U_K(i,:));

end
title("系统输入")
legend("u1") 
