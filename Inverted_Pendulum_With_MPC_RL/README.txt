一阶倒立摆仿真实验采用了系统建模、LQR、MPC、RBF在平衡点进行控制，
以上控制方式为model_based，依赖在平衡点线性化的模型，
对于偏离平衡点状态控制效果差，因而使用基于强化学习DDPG（model_free）
对于偏离平衡点的倒立摆进行控制

各个文件及文件夹作用：
best：存储训练过程中表现良好的agent
Pendulum on Cart Repository: 保存训练过程中经验值达到预期的agent
slprj： Matlab编译过程文件
视频：保存演示视频，为avi格式，需使用支持avi格式播放器
inverted_pendulum.m: 倒立摆模型建立、LQR、MPC、DDPG代码
Trolley_Pendulum_RBF_Supervisory.slx ：simulink文件

文件打开逻辑：
先打开打开inverted_pendulum.m及.slx文件（2023b及以上会自动打开），先运行.m文件，再运行.slx文件（Matlab2023b及以上自动运行），
.m文件部分代码参数使用的是Trolley_Pendulum_RBF_Supervisory.slx的simulink文件，
若无法打开该文件（MATLAB版本太低），可以使用比自己MATLAb版本低的simulink文件，不用担心.m文件的参数获取，
因为若没有打开2023b版本文件，代码有进行异常处理，会使用默认参数
