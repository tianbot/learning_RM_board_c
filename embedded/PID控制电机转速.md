# Class 9 PID控制电机转速
[【嵌入式小白的学习之路】9.1 PID控制电机的转速（一）](https://www.bilibili.com/video/BV1tw411D7Mz/)
[【嵌入式小白的学习之路】9.2 PID控制电机的转速（二）](https://www.bilibili.com/video/BV1x8411i7Ed/)
[【嵌入式小白的学习之路】9.3 PID控制电机的转速（三）](https://www.bilibili.com/video/BV13w411y7pc/)
[【嵌入式小白的学习之路】9.4 PID控制电机转速（四）](https://www.bilibili.com/video/BV1Zh4y1B7rD/)

## 今天的目标

从上一课的CAN通信与电机驱动可以知道，我们是通过控制电机电流大小来控制电机转速的，但电流的驱动能力和电机的负载有关。当我们想要使得电机转速恒定时，怎么办？
接下来我们将介绍PID的方法，闭环控制我们电机转速在有扰动的情况下维持在一个稳定值。
## 原理解析

### PID原理
理解PID，只需要理解下面这个简单的公式：

$u(t)=K_{P}\left[e(t)+\frac{1}{T_{i}} \int_{0}^{t} e(t) d t+T_{D} \frac{d e(t)}{d t}\right]$

其中：$e(t)$为误差值，$K_P$为比例系数, $\frac{K_{P}}{T_{I}}$为积分系数,$K_{P}T_{D}$为微分系数，它们是PID重要的三个参数。
比例项Kp：控制器比例项输出值和误差值保持线性关系，误差值放大一倍则输出值也同样放大一倍，误差值缩小一倍则输出值也同样缩小一倍。只依靠比例项进行控制的方法称为比例控制，比例控制可以很简单的实现控制器的基本功能， 但往往存在静差以及过大引起系统振荡的问题。
积分项Ki：控制器积分项输出值与误差值的积分值成线性关系，即误差值的累计值乘以一个常数。积分项科研加速系统趋近设定值的过程，但积分增益过大容易引起积分超调的现象。
微分项Kd：微分项的大小和输出值的变化量成正相关，微分项计算误差的一阶导数，并和一个常数相乘，得到微分项的输出值。微分项可以对系统的改变做出反应，对系统的短期改变很有帮助。
### 离散化PID（位置型PID）
由于数字系统是离散的，在单片机中实现PID控制算法时，需要将PID控制器的输出表达式改写成离散形式，其具体的做法就是将输出u(t)和误差e(t)由函数改成数组u(k)和e(k)，积分换成求和，微分换成差分。离散化后的PID表达式如下：
$u(k)=K_p e(k)+K_i  \sum_{i=0}^k e(i)+K_d [e(k)-e(k-1)]$

### 差分PID
PID控制器可以分为增量式PID控制器和位置式PID控制器，上文介绍的都是位置式PID控制器，即误差值直接决定最后的输出，而增量式PID控制器则用误差值来控制每次输出的改变量Δu

$\Delta u ( k ) = u ( k ) - u ( k - 1 )$

表达式为：

$\Delta u(k)=K_p [e(k)-e(k-1)]+K_i e(i)+K_d [e(k)-2e(k-1)+e(k-2)]$

相比位置式PID，增量式PID控制有以下优点：

- 不需要累加计算累加，输出增量只和前三次误差采样值有关，参数更容易调节
- 每次只输出控制增量，故发生故障时产生的影响较小

使用增量式PID时需要记忆上一次的输出值，将上一次的输出值和增量相加才能得到本次输出值。
本实验会着重介绍位置式PID控制器的使用。

## 实践
### PID实现
#### PID.h/.c
对大疆的PID代码进行研究：
pid.h中声明了一个枚举、一个结构体、三个函数。
枚举用于表示PID模式的选择
结构体 `pid_type_def`用于储存PID相关参数
三个函数分别是PID初始化，PID计算函数，PID清空函数。
```c
#ifndef PID_H
#define PID_H
#include "struct_typedef.h"
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

#endif
```
pid.c
这里首先用define定义了一个函数LimitMax，用于保障我们输出的值不会越界，接着是我们前面声明的对应函数的定义。待会在PID算法运行的过程中会有计算。
初始化函数 `PID_init`：
首先是向pid指针里装载我们的输入参数，PID的模式（普通还是差分）、PID的kp、ki、kd、PID的最大输出和最大积分输出。之后我们的PID调参也将是调这三个参数。
PID计算函数 `PID_calc`:
参考值 `ref`是这次读进来的值（例如电调反馈的当前电机转速），目标值是我们想要达到的目标 `set`（例如目标电机转速）。结构体 `pid` 中会放置了PID算法所涉及到的 所有值。
在算法运行过程中，如果PID结构体的指针不存在，将直接返回0. 首先将更新误差值，目标值，参考值。接下来，当使用普通PID时，将直接计算我们PID的值，对应计算比例项、微分项、积分项（限制范围），再对应的加起来得到输出（限制范围）。；使用差分PID时，我们也会同样对应计算比例项、微分项、积分项，再对应的加起来。
PID 清空函数 `PID_clear`：
用于清空PID结构体指针。

```c
#include "pid.h"
#include "main.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}
```


#### PID的使用

这里我们直接在 `main.c` 中对上述PID代码进行应用，需要注意的是，这一部分的PID示例建立在DT7遥控和CAN通信两节课的基础上，使用的同一个项目。


```c
/* USER CODE BEGIN Includes */
#include "pid.h"
/* USER CODE END Includes */
```


PID参数设置
```c
/* USER CODE BEGIN PD */
#define PID_KP        800.0f
#define PID_KI        0.5f
#define PID_KD        0.0f
#define PID_MAX_OUT  10000.0f
#define PID_MAX_IOUT 9000.0f
/* USER CODE END PD */
```

两位函数的声明
```c
/* USER CODE BEGIN PFP */
void Speed_motor_2006_init(void);
int16_t Speed_motor_2006_loop(int16_t motor_2006_speed_set);
/* USER CODE END PFP */
```

参数、结构体声明
```c
/* USER CODE BEGIN 0 */
pid_type_def motor_2006_pid;
int16_t speed_set = 0;   //rpm
int16_t motor_2006_given_current = 0;
/* USER CODE END 0 */
```

初始化以及速度初始化设置
```c
/* USER CODE BEGIN 2 */
Speed_motor_2006_init();
speed_set = 200;
/* USER CODE END 2 */
```

循环
```c
while (1)
{
    /* USER CODE BEGIN 3 */
    motor_2006_given_current = Speed_motor_2006_loop(speed_set);
    CAN_cmd_gimbal(0, 0, motor_2006_given_current, 0);
    usart_printf("shoot:%d,%d,%d \n",local_trigger->ecd,local_trigger->speed_rpm,local_trigger->given_current);
    HAL_Delay(10);
}
/* USER CODE END 3 */
```


两个函数的定义

```c
/* USER CODE BEGIN 4 */
void Speed_motor_2006_init(void){
    static const fp32 motor_2006_speed_pid[3] = {PID_KP, PID_KI, PID_KD};
    //初始化PID
    PID_init(&motor_2006_pid, PID_POSITION, motor_2006_speed_pid, PID_MAX_OUT, PID_MAX_IOUT);
}

int16_t Speed_motor_2006_loop(int16_t motor_2006_speed_set){
    PID_calc(&motor_2006_pid, (fp32)(local_trigger->speed_rpm), (fp32)(motor_2006_speed_set));
    motor_2006_given_current = (int16_t)(motor_2006_pid.out);
    return motor_2006_given_current;
}

/* USER CODE END 4 */
```



#### 遥控按键与PID调速

**调节出一个可以用的PID，套上遥控和按键调速**

1. 在cubeMX中添加Key

2. 添加代码

```c
/* USER CODE BEGIN 0 */
static int key_sta = 0;
int speed_step_sign = +1;
#define SpeedStep 500

void Key_Scan(){
		if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == GPIO_PIN_RESET){		
			if(key_sta == 0){	
				key_sta = 1;
				speed_set += SpeedStep*speed_step_sign;
				if(speed_set>8000)
				{
					speed_step_sign = -1;
				}
				if(speed_set<=0){	
					speed_set = 0;
					speed_step_sign = 1;	
				}
			}
		}else{
			key_sta = 0;
		}
}

/* USER CODE END 0 */

```

添加遥控器控制有关代码
```c
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(HAL_GetTick() > 500){   //如果500ms都没有收到遥控器数据，证明遥控器可能已经离线，切换到按键控制模式。
      Key_Scan();
    }else{		
      speed_set = local_rc_ctrl->rc.ch[3]*8000/660;
    }
    /* USER CODE END WHILE */
  }
```

测试并且调一下PID。可以参考视频中结合VOFA+的可视化展示。

另外对代码进行部分注释补充
```c

    motor_2006_given_current = (int16_t)PID_calc(&motor_2006_pid, (fp32)(local_trigger->speed_rpm), (fp32)(speed_set));//根据设定值进行PID计算,得到电流
    CAN_cmd_gimbal(0, 0, motor_2006_given_current, 0);//设置电流
    usart_printf("shoot:%d,%d,%d,%d\n",local_trigger->ecd,local_trigger->speed_rpm,local_trigger->given_current,speed_set);//打印到串口
    
    HAL_Delay(10);//PID控制频率100HZ
```

