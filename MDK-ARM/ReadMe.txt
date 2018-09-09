/**********************电机ID分配*********************/

定义在drivers_canmotor_user.h：

//RxID
//CM为底盘电机
#define CMFL_RXID 0x202u
#define CMFR_RXID 0x201u
#define CMBL_RXID 0x203u
#define CMBR_RXID 0x204u
//AM为机械臂电机
#define AM1L_RXID 0x205u
#define AM1R_RXID 0x206u
//GM为云台电机
#define GMYAW_RXID 0x20Au
#define GMPITCH_RXID 0x209u
//PM为推弹电机
#define PM1_RXID 0x201u
#define PM2_RXID 0x202u
#define PM3_RXID 0x203u

//TxID
#define CM_TXID 0x200u	//CAN1
#define AM1_TXID 0x1FFu	//CAN2
#define GM_TXID 0x2FFu	//CAN1
#define PM_TXID 0x200u	//CAN2

/***********************操作说明**********************/

键鼠模式（右拨杆 中间位置）

鼠标：
移动 旋转俯仰（灵敏度小，用于瞄准）
左键 发射（受热量限制）
右键 点击一下 开摩擦轮
        【长按】        自瞄

键盘：
WASD 移动
QE  旋转（灵敏度大，用于转向、掉头）
【长按】shift 慢速模式（上坡、怼车）
【长按】ctrl 自动取弹（取弹状态下使用）
【长按】 R 扭腰（效果不好，不建议使用）
G 取弹/正常状态切换
Z 机械爪复位（已添加自动复位，正常状态下无需使用）
X 取弹紧急中止
C 取单弹药箱

V 关摩擦轮  (比赛中无需使用）

紧急STOP（右拨杆 最下位置）

遥控模式（右拨杆 最上位置）（与工程对接时建议使用）
						   
/**************************键位表*******************/
鼠标键盘的设置 243行的 MouseKeyControlProcess(Mouse *mouse, Key *key) 键位是key->v，键位表如下：
//Bit0-----W
//Bit1-----S
//Bit2-----A
//Bit3-----D
//Bit4-----Shift
//Bit5-----Ctrl
//Bit6-----Q
//Bit7-----E
//Bit8-----R
//Bit9-----F
//Bit10-----G
//Bit11-----Z
//Bit12-----X
//Bit13-----C
//Bit14-----V
//Bit15-----B

/************************文件介绍*******************/
Drivers/CMSIS,Application/MDK-ARM,Drivers/STM32F4xx_HAL_Driver:
这些是ST官方提供的固件库，用以包装芯片底层驱动的库函数，不同系列的芯片库函数不一样，Cube配置时会自动生成。
不知道相应功能的库函数是什么时可以百度，可以查说明书，也可以在相应外设的库函数文件里面找

Middlewares/FreeRTOS:
FreeRTOS操作系统的程序文件，Cube配置时自动生成其中的main函数

Middlewares/USB_Device_Library:
USB的配置库函数文件，主控板上有一个miniUSB接口，可以用这种方式通信，但我们目前没有开发

Application/User：
外设的配置，Cube生成
其中的main函数没有用到，但是里面的看门狗初始化要注释掉

Framework/RTOS:
rtos_init.c:系统的初始化函数，对板子上的外设模块进行初始化。要区别芯片上的外设初始化，那个在这个之前，是Cube生成的。
            这个要自己写，以后添加什么新硬件需要初始化的话在这里面加。
rtos_semaphore.c:系统进程信号量的初始化，添加信号量参照这个格式添加
rtos_task.c:系统进程初始化，添加任务参照这个格式添加

Framework/Peripheral:
peripheral_define.h: 定时器，串口，CAN的重命名，提高可读性
peripheral_gpio.c：外部中断读取MPU6050数据
peripheral_laser.h：激光瞄准器的开关函数
peripheral_sov.h：电磁阀的开关函数

Framework/Utilities:
utilities_debug.c：重定向C标准库函数printf到串口DEBUG_UART,可以直接用printf输出信息到串口，方便调试
utilities_iopool.c：IOPOOL的定义，一种相比于全局变量，在线程间安全通信的方式，具体使用看C文件注释
utilities_minmax.h：求最大最小值的函数和角度规范化的函数
utilities_tim.c：使用1ms定时器获得上电后ms单位的时间
peripheral_tim.c：摩擦轮、舵机PWM所需定时器初始化函数
visualscope.c：用来看波形的上位机，可以同时显示四个变量，现在把这个中断关掉了

Framework/Drives:
drivers_led.c：红灯和绿灯的开关函数，以及两个亮灯的任务
drivers_imu.c：板载IMU传感器数据的读取函数，以及IMU数据的刷新任务
drivers_buzzer.c：蜂鸣器的执行函数，包括开机音乐
drivers_canmotor.c：CAN的底层驱动，CAN的接收是通过中断的方式，用以接收反馈的编码器位置、速度信息。
                    CAN的发送用来驱动电机
drivers_uart.c：所有串口的中断函数，包括遥控器串口，妙算串口，裁判系统串口
drivers_uartrc.c：遥控器串口接收函数，主要包括拨杆的模式设置，以及发射摩擦轮的启动逻辑
drivers_uartupper.c：妙算的通信函数，旧步兵只回传一个大神符的击打数字
drivers_flash.c：flash写入和读取的函数，用于掉电保存，但我们目前没用到
drivers_sonar.c：超声波测距的驱动，基地上用，步兵没有用到
drivers_ramp.h：斜坡函数
pid_regulator.c：PID函数的实现，最基本的PID函数，更高级的算法要改这里complete_PID为完整的PID，另外一个算法有误
pwm_server_motor：不同PWM控制不同的舵机角度，原先用来控制弹仓开关的，现在没有用到
drivers_uartjudge.c：裁判系统串口的中断函数，将裁判系统的数据帧解析成信息
drivers_platemotor.c：拨盘电机速度/位置控制,英雄中这个已经没用了
drivers_cmpower.c：根据能量槽剩余做动态上限，单独限制 + 总和比例限制，仿桂电方案
UserProtoca.c：外接陀螺仪数据帧的用户协议，没有用到
drivers_uartgyro.c:外界陀螺仪数据读取

Framework/Applications：主体的任务都在这边，比较重要
tasks_motor.c：底盘和云台的控制任务，主要要调的底盘PID和云台PID都在这边
application_motorcontrol.c：执行电机CAN信号控制的函数
tasks_upper.c：妙算通信任务
application_quaternion：更新四元数的函数，但我们目前没有用到
tasks_remotecontrol.c：遥控器控制的任务函数，比较重要，遥控器拨杆、摇杆，键盘鼠标操作方式都在这边改
tasks_timed.c 2ms周期的任务，状态机切换
application_waveform.c 上位机观察电机信号波形，之前没有用到
tasks_platemotor.c 旧步兵的拨盘电机的任务，在英雄上已经没用了
tasks_arm.c：机械臂和推弹电机控制任务
tasks_hero.c:主要用于进行带有osDelay的操作，主要包括取弹、推弹
