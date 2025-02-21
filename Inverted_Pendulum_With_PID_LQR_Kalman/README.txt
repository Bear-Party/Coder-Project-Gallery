各个文件作用：

LQR上位机：
inverted_pendulum.mlapp:LQR上位机代码文件，可使用MATLAB中的APP工具箱打开
倒立摆上位机演示.mp4: 倒立摆上位机演示视频

MPC控制文件夹下：
MPC.m:MATLAB编程代码文件、代码基于MPC_Ctr_2023b.slx文件进行
MPC_Matrices.m:MPC的E、H矩阵计算函数
Prediction.m:MPC模型预测函数
MPC_Ctr_2023b.slx matlab2023b版本的simulink文件
MPC_Ctr_2020b.mdl matlab2016b版本的simulink文件
MPC_Ctr_2016b.mdl matlab2016b版本的simulink文件

PID+LQR+Kalman文件夹下：
inverted_pend.m:存储倒立摆matlab编程代码文件,代码中基于inverted_pend_2023b文件进行
inverted_pend_2023b.slx:matlab2023b版本的simulink文件
inverted_pend_2021b.mdl:matlab2021b版本的simulink文件
inverted_pend_2019b.mdl:matlab2019b版本的simulink文件
inverted_pend_2016b.mdl:matlab2016b版本的simulink文件
脉冲干扰LQR_Ctr.mp4:脉冲干扰下倒立摆系统响应
正弦干扰LQR_Ctr.mp4:正弦干扰下倒立摆系统响应

平衡小车文件夹下：
平衡小车keil工程代码：编写的用于控制基于stm32c8t6主控的平衡小车

文件打开逻辑：
先打开打开.m及.slx文件，先运行.m文件，再运行.slx文件，.m文件部分代码参数使用的是inverted_pend_2023b.slx的simulink文件，
若无法打开该文件（MATLAB版本太低），可以使用比自己MATLAb版本低的simulink文件，不用担心.m文件的参数获取，
因为若没有打开2023b版本文件，代码有进行异常处理，会使用默认参数
