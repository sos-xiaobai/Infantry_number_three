开发者日志：
- 2022.7.27 修改人：金旭峰
    1. 新增云台can2的裁判系统读取枪口热量和热量限制，防止自瞄时热量超限。
    
2022.6.5 杨宗键
为了方便上位机调试，单发和十连发互换位置，修改单发为左下右下，十连发为左下右上


2022.6.5 李紫璇
依据肉眼和编码器，单发走的角度应当是没问题的。（要我说就机械限位设计的不行
打算走一个位置回一点


2022.6.4 李紫璇
云台往右飘的问题已解决。
用编码器的角度值来进行PID，避免了陀螺仪飘的问题。（陀螺仪经过校准，漂移情况有改善）
改的地方在RC.c以及gimbal_pid这两个地方
单发无法实现

2022.6.3 罗俊宇
1.修改了bsp_uart中的SHOOT_DIRECTION角度的宏定义，pitch角度每一项都加了9，因为喵准正中心的pitch角度是-9。
右上右下的yaw角每个加了6，测试中角度飘逸有导致测出来的不准。
2.解除了key_control_data函数中One_Shoot_flag=0;这句话的注释，不然这个变量置1后没法置0
3.systick中断中假如sleep_cnt倒计时值，打子弹时倒计时赋值，当倒计时到1时让云台归零，逻辑没问题但功能实测没效果。
4.待解决问题：
云台一直往右边飘，可以尝试加一个恒定的电流平衡。


2020.6.2 李紫璇
依据上位机要求修改了 bsp_uasrt 部分的接收与发送函数 接收数变为两个 一个是发射时间以秒为单位 一个是四个射击位置的标识符
例如接收的是 1.2 2 便是1.2秒后发射右下位置；   发送数值就一个就是 大风车是顺时针转还是逆时针转
计时部分改在xxit.h里的系统计时中断  射击位置初始值和发送数值在RC.c里改动   