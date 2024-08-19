# 步兵电控代码开源文档

> 此代码用于2024赛季Robomaster超级对抗赛山海Mas战队4号步兵。
>
> 步兵代码V1.0

* 第一次修改 2024.8.19 修改**写在最后**

## 一、概述

本代码从2023年8月下旬正式建立，采用标准库裸机开发，并正式应用于2024赛季高校联盟赛和超级对抗赛。在高校联盟赛时并没有加入功率控制导致频繁超功率，超级对抗赛又由于俯仰角问题没办法大展身手，算是一种遗憾吧。

## 二、接线图

![接线图.png](https://github.com/zpclyn/Mas_Infantry_Control/blob/main/IMAGE/%E6%8E%A5%E7%BA%BF%E5%9B%BE.png?raw=true)

## 三、Keil文件说明

### 底盘

### 云台

## 四、功能说明

控制方式具有图传链路、遥控器两种模式，目前在模式切换略有问题，在上电后注意不要更改控制方式。

控制映射为：

| 输入         | 输出                         | 输入         | 输出                               |
| ------------ | ---------------------------- | ------------ | ---------------------------------- |
| W            | 前进                         | A            | 左移                               |
| S            | 后退                         | D            | 右移                               |
| Q*           | 摩擦轮                       | E            | 拨弹盘反转                         |
| Shift*       | 超电                         | Ctrl*        | 小陀螺                             |
| 鼠标左右     | 云台左右旋转                 | 鼠标前后     | 云台俯仰                           |
| 鼠标左键     | 射击                         |              |                                    |
| 遥控器左拨码 | 1-开摩擦轮，其他-关摩擦轮    | 遥控器右拨码 | 1-反向小陀螺，3-正常移动，2-小陀螺 |
| 遥控器左摇杆 | 上下-云台俯仰，左右-云台旋转 | 遥控器右摇杆 | 上下-前后移动，左右-左右平移       |
| 遥控器拨轮   | 前拨-发射，反拨-拨弹盘反转   |              |                                    |

> 标*的为开关形式，按一下开，按一下关
>
> 此处前后左右均以云台坐标系为基准
>
> 反向小陀螺仅在检录时使用，相比正向小陀螺效果略差
>
> 鼠标左键仅在摩擦轮开启时有效
>
> 鼠标右键具有自瞄功能，但由于视觉组摆烂，只是空有架子，没有实际作用

此代码同时具有自定义UI用以显示当前摩擦轮、小陀螺和超电是否开启。

在没有裁判系统时，需要注释RefereeSystem.c文件中RefereeSystem_Init函数的`while(RefereeSystem_RobotID==0);`语句，同时功率控制中会采用默认功率上限80W，缓冲能量默认30J，发射机构默认上电。

## 五、代码说明

### 1. 底盘代码说明

底盘代码主要分为四个函数

```C
void Mecanum_Init(void);//麦轮初始化
void Mecanum_ControlSpeed(int16_t LeftFrontSpeed,int16_t RightFrontSpeed,int16_t LeftRearSpeed,int16_t RightRearSpeed);//麦轮速度控制
void Mecanum_InverseMotionControl(float v_x,float v_y,float w);//麦轮逆运动解算
void Mecanum_PowerMoveControl(void);//麦轮功率控制
```

其中`void Mecanum_Init(void);`进行PID的初始化，`void Mecanum_ControlSpeed(int16_t LeftFrontSpeed,int16_t RightFrontSpeed,int16_t LeftRearSpeed,int16_t RightRearSpeed);`进行PID计算，闭环周期为2ms。

而剩下两个函数就是底盘代码的核心。

#### 1.1 底盘逆运动解算

对于底盘坐标系的建立如下图所示

![底盘麦轮解算简图.png](https://github.com/zpclyn/Mas_Infantry_Control/blob/main/IMAGE/%E5%BA%95%E7%9B%98%E9%BA%A6%E8%BD%AE%E8%A7%A3%E7%AE%97%E7%AE%80%E5%9B%BE.png?raw=true)

稍微有一点反直觉，但是无妨，在此坐标系下，麦克纳姆轮的逆运动解算公式如下

$$
\begin{align}
v_{左前轮}&=-v_{x}-v_{y}-w(r_{x}+r_{y})\\
v_{右前轮}&=v_{x}-v_{y}-w(r_{x}+r_{y})\\
v_{左后轮}&=-v_{x}+v_{y}-w(r_{x}+r_{y})\\
v_{右后轮}&=v_{x}+v_{y}-w(r_{x}+r_{y})
\end{align}
$$

此部分代码在`void Mecanum_InverseMotionControl(float v_x,float v_y,float w);`中实现，如果考虑云台和底盘的相对角度，云台坐标系和底盘坐标系之间存在坐标变换（显然，图传链路和遥控器控制均以云台坐标系为基准）。

![底盘云台向量变换简图.png](https://github.com/zpclyn/Mas_Infantry_Control/blob/main/IMAGE/%E5%BA%95%E7%9B%98%E4%BA%91%E5%8F%B0%E5%90%91%E9%87%8F%E5%8F%98%E6%8D%A2%E7%AE%80%E5%9B%BE.png?raw=true)

坐标变换公式为

$$
\begin{align}
v_{x底盘}&=v_{x云台}cos\theta-v_{y云台}sin\theta\\
v_{y底盘}&=v_{x云台}sin\theta+v_{y云台}cos\theta
\end{align}
$$

此部分代码在`void Mecanum_PowerMoveControl(void);`中实现，具体代码段为

```C
	int16_t Raw_Theta=Yaw_GM6020PositionValue-GM6020_MotorStatus[0].Angle;//获取底盘云台相对角度原始数据
	if(Raw_Theta<0)Raw_Theta+=8192;
	//Mecanum_YawTheta=Raw_Theta/8192.0f*2.0f*3.141592653589793238462643383279f;
	Mecanum_YawTheta=Raw_Theta*0.000766990393942820614859043794746f;//获取底盘云台相对角度
	
	float vx_=vx,vy_=vy;
	vx=vx_*cosf(Mecanum_YawTheta)-vy_*sinf(Mecanum_YawTheta);
	vy=vx_*sinf(Mecanum_YawTheta)+vy_*cosf(Mecanum_YawTheta);//根据底盘云台相对角度修正xy轴速度
```

最终达到效果，不管云台和底盘相对角度如何，均以云台坐标系为基准进行平移旋转。

#### 1.2 功率控制

由`void Mecanum_PowerMoveControl(void);`函数实现功率控制，具体实现由下方功率控制代码介绍。

### 2. 云台代码说明

云台代码主要分为如下四个函数

```c
void Gimbal_PitchControl(void);//云台Pitch轴控制
void Gimbal_YawControl(void);//云台Yaw轴控制
void Gimbal_FiringMechanismControl(void);//摩擦轮控制
void Gimbal_Rammer(void);//拨弹盘控制
```

云台Pitch轴控制和Yaw轴控制主要是对两个6020电机的串级PID保证云台陀螺仪保持期望姿态，并通过遥控器可以对姿态进行改变。云台Pitch轴控制中包含了视觉补偿代码，具体介绍见下方视觉联调代码介绍。由于机器人云台Yaw轴控制一直保持为陀螺仪角度控制，于是没必要考虑小陀螺状态时的特殊处理。

摩擦轮控制和拨弹盘控制均为一个简单的速度环PID，不过注意在摩擦轮未开启时无法进行拨弹盘控制，同时在比赛开始发射机构未上电时也无法控制摩擦轮和拨弹盘。

### 3. 视觉联调说明

视觉这个部分很鸡肋，在代码里找了个地方留了个接口，但是一直到比赛前，视觉一直摆烂毫无成果，导致视觉联调完全没做，后来也懒得改了，接口就一直留在俯仰控制的函数里了。

如果要做视觉联调，首先通讯数据帧如下

> 帧头				数据（高位在前）														帧尾
>
> 0x09	0x14	Visual_Yaw\*100(2Byte)	Visual_Pitch\*100(2Byte)	0x18

采用了一个比较简单的包结构，方便自己方便队友。

在做视觉联调时可以考虑把视觉补偿拿出来单独作为一个函数。

### 4. 功率控制代码说明

此功率控制模型为山海Mas战队自研，从功率上限、速度、电流进行三闭环控制，故简称为**PVI模型**。

PVI功率模型主要分为**三轴速度获取**、**小陀螺处理**、**功率上限处理**、**功率控制**和**运动控制**五部分。

#### 4.1 三轴速度获取

```C
	float vx=1024-Remote_RxData.Remote_R_UD;
	float vy=1024-Remote_RxData.Remote_R_RL;
	float w=1024-Remote_RxData.Remote_L_RL;//获取三个轴的速度参量
	float sigma=sqrtf(vx*vx+vy*vy);//获取xy轴速度归一化系数
	if(sigma!=0)//x,y轴速度归一化(正交合成速度不变为标准值)
	{
		vx=vx/sigma*Mecanum_PowerControlSpeedNormalizationValue;
		vy=vy/sigma*Mecanum_PowerControlSpeedNormalizationValue;
	}
	
	int16_t Raw_Theta=Yaw_GM6020PositionValue-GM6020_MotorStatus[0].Angle;//获取底盘云台相对角度原始数据
	if(Raw_Theta<0)Raw_Theta+=8192;
	//Mecanum_YawTheta=Raw_Theta/8192.0f*2.0f*3.141592653589793238462643383279f;
	Mecanum_YawTheta=Raw_Theta*0.000766990393942820614859043794746f;//获取底盘云台相对角度
	
	float vx_=vx,vy_=vy;
	vx=vx_*cosf(Mecanum_YawTheta)-vy_*sinf(Mecanum_YawTheta);
	vy=vx_*sinf(Mecanum_YawTheta)+vy_*cosf(Mecanum_YawTheta);//根据底盘云台相对角度修正xy轴速度
```

此处代码首先通过遥控器获取三轴速度（以云台坐标系为基准），然后对 $v_x$ 和 $v_{y}$ 进行归一化处理，方便后方速度增益的一致性，然后进行坐标变换获取底盘坐标系的 $v_x$ 和 $v_{y}$ 。

#### 4.2 小陀螺处理

```C
	if(Yaw_GM6020PositionValue<4096)//正向小陀螺为逆时针
	{
		if((Remote_RxData.Remote_RS==2 || Remote_RxData.Remote_KeyPush_Ctrl==1) || (Mecanum_GyroScopeFlag==1 && Mecanum_GyroScopeCloseFlag==1 && GM6020_MotorStatus[0].Position<Yaw_GM6020PositionValue+500))//小陀螺模式
		{
			w=Mecanum_GyroScopeAngularVelocity;
			Mecanum_GyroScopeFlag=1;//处于小陀螺状态
			Mecanum_GyroScopeCloseFlag=1;//小陀螺处于待关闭状态
		}
		else if(Mecanum_GyroScopeFlag==1 && ((GM6020_MotorStatus[0].Angle>Yaw_GM6020PositionValue+100 || GM6020_MotorStatus[0].Angle<Yaw_GM6020PositionValue-100) || (GM6020_MotorStatus[0].Speed>2||GM6020_MotorStatus[0].Speed<-2)))//小陀螺关闭状态
		{
			PID_PositionSetOUTRange(&Mecanum_TrackPID,-2,2);
			PID_PositionCalc(&Mecanum_TrackPID,GM6020_MotorStatus[0].Position);//底盘跟云台
			w=-Mecanum_TrackPID.OUT;
			Mecanum_GyroScopeCloseFlag=0;//小陀螺未处于待关闭状态
		}
		else if(Remote_RxData.Remote_RS==1)//反向小陀螺模式(仅检录使用)
			w=-Mecanum_GyroScopeAngularVelocity;
		else
		{
			PID_PositionSetOUTRange(&Mecanum_TrackPID,-5,5);
			Mecanum_GyroScopeFlag=0;
			PID_PositionCalc(&Mecanum_TrackPID,GM6020_MotorStatus[0].Position);//底盘跟云台
			w=-Mecanum_TrackPID.OUT;
		}
	}
	else//正向小陀螺为顺时针
	{
		if((Remote_RxData.Remote_RS==2 || Remote_RxData.Remote_KeyPush_Ctrl==1) || (Mecanum_GyroScopeFlag==1 && Mecanum_GyroScopeCloseFlag==1 && GM6020_MotorStatus[0].Position>Yaw_GM6020PositionValue-500))//小陀螺模式
		{
			w=-Mecanum_GyroScopeAngularVelocity;
			Mecanum_GyroScopeFlag=1;//处于小陀螺状态
			Mecanum_GyroScopeCloseFlag=1;//小陀螺处于待关闭状态
		}
		else if(Mecanum_GyroScopeFlag==1 && ((GM6020_MotorStatus[0].Angle>Yaw_GM6020PositionValue+5 || GM6020_MotorStatus[0].Angle<Yaw_GM6020PositionValue-5) || (GM6020_MotorStatus[0].Speed>1||GM6020_MotorStatus[0].Speed<-1)))//小陀螺关闭状态
		{
			PID_PositionSetOUTRange(&Mecanum_TrackPID,-2,2);
			PID_PositionCalc(&Mecanum_TrackPID,GM6020_MotorStatus[0].Position);//底盘跟云台
			w=-Mecanum_TrackPID.OUT;
			Mecanum_GyroScopeCloseFlag=0;//小陀螺未处于待关闭状态
		}
		else if(Remote_RxData.Remote_RS==1)//反向小陀螺模式(仅检录使用)
			w=Mecanum_GyroScopeAngularVelocity;
		else
		{
			PID_PositionSetOUTRange(&Mecanum_TrackPID,-5,5);
			Mecanum_GyroScopeFlag=0;
			PID_PositionCalc(&Mecanum_TrackPID,GM6020_MotorStatus[0].Position);//底盘跟云台
			w=-Mecanum_TrackPID.OUT;
		}
	}
```

小陀螺的处理主要分为三部分：**小陀螺状态**、**正常状态**和**过渡状态**，过渡状态应用于小陀螺状态转换正常状态时，主要为了避免此时发生的电机换向产生的大电流，为了更好的过渡状态，由底盘云台同向时的Yaw轴6020编码器值区分正向小陀螺为顺时针或逆时针，同时，对过渡状态和正常状态的底盘跟随角速度输出限幅进行区别防止过渡状态超调。

#### 4.3 功率上限处理

**此处为PVI模型的第一环——功率环。**

功率上限区别于裁判系统中的功率上限，大多数情况下功率上限高于裁判系统的功率上限。对功率上限的约束来自于**缓冲能量**、**运动状态**和**超电**。

##### 4.3.1 缓冲能量约束

```C
	float Mecanum_PowerRef=1.0f/(60.0f-Mecanum_PowerControl_UseBuffer)*RefereeSystem_Buffer;//功率增益
	if(Mecanum_PowerRef>Mecanum_PowerControl_PowerMax)Mecanum_PowerRef=Mecanum_PowerControl_PowerMax;
	Mecanum_PowerLimit=Mecanum_PowerRef*RefereeSystem_Ref;
```

缓冲能量约束为第一级约束，与其说是约束，不如说是获取，在此处通过如下公式获取最初始的功率上限值。

$$
PowerLimit=\dfrac{Buffer}{60-UseBuffer}\times PowerRef
$$

对功率增益系数 $K_{buffer}=\dfrac{Buffer}{60-UseBuffer}$ 作图，可以得到

![缓冲能量功率增益.png](https://github.com/zpclyn/Mas_Infantry_Control/blob/main/IMAGE/%E7%BC%93%E5%86%B2%E8%83%BD%E9%87%8F%E5%8A%9F%E7%8E%87%E5%A2%9E%E7%9B%8A.png?raw=true)

此公式实现了缓冲能量越多，功率越大，在 $Buffer=60-UseBuffer$ 时，功率增益为1，在此处稳定下来。同时缓冲能量利用越多，功率越大，充分利用缓冲能量提高功率。同时在飞坡后也有很好的效果，会有更高的功率增益。

##### 4.3.2 运动状态约束

```C
	float Scale=sigma/660.0f;
	if(Scale<1 && Scale>0)Mecanum_PowerLimit*=Scale;//平移约束
	if(Mecanum_GyroScopeFlag==1)Mecanum_PowerLimit=Mecanum_PowerRef*RefereeSystem_Ref;//小陀螺约束
	if(Mecanum_PowerLimit>150.0f)Mecanum_PowerLimit=150.0f;//限幅约束
```

此处约束主要针对遥控器摇杆，摇杆拨动程度不同，功率上限成比例放缩。同时对于小陀螺状态，功率上限不会放缩。

##### 4.3.3 超电约束

```C
	if(Remote_RxData.Remote_KeyPush_Shift==1)//开启超电
	{
		if(Mecanum_PowerLimit>0.0f)
		{
			if(Mecanum_PowerLimit<RefereeSystem_Ref)Mecanum_PowerLimit+=Mecanum_PowerControl_UltraCAPPower;
			else Mecanum_PowerLimit=RefereeSystem_Ref+Mecanum_PowerControl_UltraCAPPower;
		}
		Ultra_CAP_SetPower(RefereeSystem_Ref);
	}
	else
		Ultra_CAP_SetPower(Mecanum_PowerLimit);
```

未开启超电时，使超电的功率上限和软件功率上限一致，使超电无法工作，保留电量。开启超电时，使软件上限直接增大`Mecanum_PowerControl_UltraCAPPower`，并使超电功率上限等于裁判系统功率上限，使底盘处于超功率运行。

#### 4.4 功率控制

**此处为PVI模型的第二三环——速度环和电流环。**

##### 4.4.1 速度环

```C
	if(Count==Mecanum_PowerControl_T)//一个控制周期
	{
		Count=0;//计数器清零
		float Power_avg=PowerSum/Mecanum_PowerControl_T;
		PowerSum=0;
		
		if(Power_avg<Mecanum_PowerLimit/2.0f)
		{
			GainCoefficient+=0.05f;//增益系数递减
			if(GainCoefficient>2)GainCoefficient=2;//增益系数限上幅2
		}
		else if(Power_avg<Mecanum_PowerLimit-5)
		{
			GainCoefficient+=0.02f;//增益系数递减
			if(GainCoefficient>2)GainCoefficient=2;//增益系数限上幅2
		}
		else if(Power_avg>Mecanum_PowerLimit+5)
		{
			if(RefereeSystem_Buffer<60.0f-Mecanum_PowerControl_UseBuffer)GainCoefficient-=0.05f;//增益系数递增
			else GainCoefficient-=0.02f;//增益系数递增
			if(GainCoefficient<Mecanum_PowerControlGainCoefficientInitialValue/2.0f)GainCoefficient=Mecanum_PowerControlGainCoefficientInitialValue/2.0f;//增益系数限下幅
		}
	}
	else if(vx!=0 || vy!=0 || Mecanum_GyroScopeFlag==1)//正常运动
		Count++;
	else//停止运动
		GainCoefficient=Mecanum_PowerControlGainCoefficientInitialValue;//增益系数回归初始值
	if(Mecanum_GyroScopeFlag==1 && (vx!=0 || vy!=0)){w*=0.6f;vx*=0.5f;vy*=0.5f;}//小陀螺移动直接降速,保证移动瞬间为缓冲能量增加
```

速度环的主要实现方式为速度增益系数。通过速度乘上增益系数改变速度大小从而改变功率，同时速度增益系数变化速度跟功率与功率上限的差值有关，相差越多变化越快。

同时在停止运动时使增益系数回归初始状态保证闭环的持续性。

在小陀螺状态下，通过降速来抑制换向大电流，同时由于功率的下降导致缓冲能量回复。

##### 4.4.2 电流环

```C
	float Mecanum_BufferCurrent=Mecanum_PowerLimit*512.0f/15.0f;
	float Mecanum_PowerCurrent=Mecanum_PowerLimit*512.0f/15.0f*5.0f;
	float B,C=0.9f;
	if(60.0f-Mecanum_PowerControl_UseBuffer-10.0f>10.0f)B=60.0f-Mecanum_PowerControl_UseBuffer-10.0f;
	else B=10.0f;
	if(RefereeSystem_Buffer>B)
	{
		float PowerScale=0;
		if(Mecanum_Power>C*Mecanum_PowerLimit)
		{
			if(Mecanum_Power>Mecanum_PowerLimit)PowerScale=0;
			else PowerScale=(Mecanum_PowerLimit-Mecanum_Power)/(Mecanum_PowerLimit-C*Mecanum_PowerLimit);
		}
		else PowerScale=1;
		Mecanum_CurrentLimit=Mecanum_BufferCurrent+Mecanum_PowerCurrent*PowerScale;
	}
	else
	{
		if(RefereeSystem_Buffer<5.0f)Mecanum_CurrentLimit=Mecanum_BufferCurrent*0.5f;
		else Mecanum_CurrentLimit=Mecanum_BufferCurrent*(RefereeSystem_Buffer+B-10.0f)/(2*B-10.0f);
	}
	if(Mecanum_CurrentLimit>16384*4)Mecanum_CurrentLimit=16384*4;
```

电流环的实现主要通过对电流的限幅来进行功率控制，在此仅讨论电流限幅大小的确定逻辑。

电流环的运行逻辑如下所示

![电流环.png](https://github.com/zpclyn/Mas_Infantry_Control/blob/main/IMAGE/%E7%94%B5%E6%B5%81%E7%8E%AF.png?raw=true)

转换成数学表达式为

$$
I_{Limit}=\begin{cases}
\dfrac{1}{2}I_{Buffer},&Buffer < 5\\
\dfrac{Buffer+B-10}{2B-10}\times I_{Buffer},&5 \leqslant Buffer \leqslant B\\
I_{Buffer}+I_{Power},&B < Buffer,P \leqslant 0.9P_{Limit}\\
I_{Buffer}+\dfrac{P_{Limit}-P}{P_{Limit}-0.9P}\times I_{Power},&B < Buffer,0.9P_{Limit} < P \leqslant P_{Limit}\\
I_{Buffer},&B < Buffer,P_{Limit} < P
\end{cases}\\
\\\
\text{其中},I_{Buffer}=\dfrac{512}{15}P_{Limit},I_{Power}=\dfrac{512\times 5}{15}P_{Limit}
$$

而其中电流与功率的对应比例关系为

$$
P=UI_{实际}=24V\times(\dfrac{I}{16384}\times20A)=\dfrac{15}{512}\times I
$$

故电流限幅可用功率增益系数等效表示。

电流功率增益系数表达式为

$$
K_I=
\begin{cases}
\dfrac{1}{2},&Buffer < 5\\
\dfrac{Buffer+B-10}{2B-10},&5 \leqslant Buffer \leqslant B\\
6,&B < Buffer,P \leqslant 0.9P_{Limit}\\
1+5\times\dfrac{P_{Limit}-P}{P_{Limit}-0.9P},&B < Buffer,0.9P_{Limit} < P \leqslant P_{Limit}\\
1,&B < Buffer,P_{Limit} < P
\end{cases}
$$

由此可做出两张图像来更直观的表现电流功率增益系数

图1： $Buffer<=B时,K_I-Buffer曲线$ 

![电流功率增益系数1.png](https://github.com/zpclyn/Mas_Infantry_Control/blob/main/IMAGE/%E7%94%B5%E6%B5%81%E5%8A%9F%E7%8E%87%E5%A2%9E%E7%9B%8A%E7%B3%BB%E6%95%B01.png?raw=true)

图2： $Buffer>B时,K_I-P曲线$ （假定 $P_{Limit}=1$ ）

![电流功率增益系数2.png](https://github.com/zpclyn/Mas_Infantry_Control/blob/main/IMAGE/%E7%94%B5%E6%B5%81%E5%8A%9F%E7%8E%87%E5%A2%9E%E7%9B%8A%E7%B3%BB%E6%95%B02.png?raw=true)

注意，由于电压并非24V，故电流功率对应比例关系并不准确，而且电流环的公式并非最优解，有大量的优化空间。

#### 4.5运动控制

```C
	vx*=GainCoefficient;vy*=GainCoefficient;
	if(Mecanum_GyroScopeFlag==1)w*=GainCoefficient;
	int16_t LeftFrontSpeed=(int16_t)((-vx-vy-w*(Mecanum_rx+Mecanum_ry)/100.0f)/Mecanum_WheelRadius*18143.663512f);//左前
	int16_t RightFrontSpeed=(int16_t)((vx-vy-w*(Mecanum_rx+Mecanum_ry)/100.0f)/Mecanum_WheelRadius*18143.663512f);//右前
	int16_t LeftRearSpeed=(int16_t)((-vx+vy-w*(Mecanum_rx+Mecanum_ry)/100.0f)/Mecanum_WheelRadius*18143.663512f);//左后
	int16_t RightRearSpeed=(int16_t)((vx+vy-w*(Mecanum_rx+Mecanum_ry)/100.0f)/Mecanum_WheelRadius*18143.663512f);//右后
	
	Mecanum_ControlSpeed(LeftFrontSpeed,RightFrontSpeed,LeftRearSpeed,RightRearSpeed);
```

此处即为麦克纳姆轮逆运动解算，见上方底盘代码介绍。

### 5.官方姿态解算代码移植说明

最开始我采用了软件教程文档中的姿态解算代码及PI参数，但效果很不理想，最后通过移植官方姿态解算代码获得了极大的改善。下方是移植姿态解算代码的方法。

1. 将AHRS文件夹移入工程文件夹。
2. 在Keil工程中导入**AHRS.lib**、**ahrs_lib.h**、**AHRS_middleware.c**、**AHRS_middleware.h**、**user_lib.c**和**user_lib.h**六个文件。
3. 在**Options->C/C++**中的**Include Paths**引入AHRS文件夹，**Define**中加入**,ARM_MATH_CM4,__FPU_PRESENT=1U**。
4. 注释**stm32f4xx.h**中183行的`#define __FPU_PRESENT             1       /*!< FPU present                                   */`语句。

### 6.时钟修改说明

为了使定时器时钟正确，需要根据下图时钟树对标准库代码进行一定的修改。

![C板时钟树.jpg](https://github.com/zpclyn/Mas_Infantry_Control/blob/main/IMAGE/C%E6%9D%BF%E6%97%B6%E9%92%9F%E6%A0%91.jpg?raw=true)

* 将**stm32f4xx.h**的137行中**HSE**频率改为12MHz。
* 将**system_stm32f4xx.h**的364行**PLL_M**改为6，377行**PLL_Q**改为4，394行**PLL_N**改为168，396行**PLL_P**改为2。

## 六、注意事项和改进

1. 定时器和中断优先级尽量不要修改，裸机开发的弊病，改不好容易疯车。
2. 其实在底盘跟随时，由于功率控制的一些限定，跟随速度很慢，在后期更改中需要完善。底盘跟随的逻辑还可以优化一下，跑动起来之后在转弯时有些扭动。
3. 鼠标控制也具有一定的问题，延迟比较大，可能逻辑上或时序上有些问题，还需完善。
4. 在飞坡之后的250J缓冲能量，还可以有更好的控制方法，根据下面的赛规，感觉有更好的玩法。

> 英雄机器人、步兵机器人或哨兵机器人触发飞坡增益后，缓冲能量增加至 250J。后续若缓冲能量消耗至 60J 以下，缓冲能量最高可恢复至 60J。

5. Pitch轴6020容易卡住小弹丸导致堵转，南部赛区的时候真的发生了，可以考虑像工程一样做堵转保护。
6. 反向小陀螺还可以改一改，加一下过渡过程。
7. 遥控器和图传链路的兼容还有问题，不能做到比赛中两者随意更换。图传链路的检测时间也没精调。
8. Pitch轴和Yaw轴的6020还有一些小幅的低频抖动，还需要精调一下PID或者滤一下波。
9. IST8310初始化有时过不去会报警，后续可以做一些处理。

## 写在最后

今年是我入队的第二年，正式做车的第一年，当时的我还懵懂无知，刚刚转动一个3508便兴奋异常，到现在虽然还是很菜，但也能独当一面（了吧？）。回望这一年，感触颇深，我仍然记得当时赛规出来的时候，人员稀少（虽然现在也是），我曾萌生了退缩之意，打个3V3得了，不过在Co总的号召下，大家还是最后决定拼一把7V7，对于一个第二年的队伍，7V7的压力可谓颇大，这一年真的是不分昼夜，用命在拼搏。不过现在想想，对这个决定还是有点后悔的，因为队里没有电控，我一个人承担了6辆机器人代码的编写，当时还被Co总骗了，从头开始写的代码，当时真的压力山大，也闹过几次情绪，不过感谢Co总的耐心疏导和队友的包容。最后在3V3中期之前也是赶制出了步兵、哨兵和工程的代码并顺利完成了中期的录制，不过因为视觉组摆烂，哨兵打算完全复原去年学长的代码（学长太强了），然后还复原不出来，因为今年枪管上面多了个图传，可能在好多地方都有影响，然后步兵代码也很不完善，有很多问题而且没有功率控制，辽宁站大败而归，当时真的很难过，好多人自己偷偷抹眼泪，太多心血了。回来之后痛改前非，完成了现在的步兵代码V1.0，完成了第一套能上场的完整的步兵代码。

这一年真的太累了，已经记不清有多少个夜晚通宵改代码，记不清看过多少日出，好多次想要放弃，摆烂算了，就像视觉组一样，不过一直难以割舍，或许这就是RM的魅力，为了热爱，奋不顾身。感谢Co总，引导我不断成长，感谢队友，陪我日日夜夜，感谢RM，让我们相遇。

在火车上写的，比较杂碎，不过，RMer，懂的都懂😏。

最后还得骂一句，为什么视觉组那种摆子都能是研发代表，我写了七个车的代码都不是！

> PS：
>
> **本开源最终解释权为河北工业大学山海Mas战队所有，不得用于任何商用行为。**
>
> 欢迎交流，如有任何问题，请联系QQ：**zpclyn 2297651490**。
