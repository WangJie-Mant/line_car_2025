/*
 * 循迹PID参数调试指南
 * 文件名：line_follow_tuning_guide.md
 * 
 * 本文档说明如何调试循迹PID参数以获得最佳控制效果
 */

# 循迹PID控制参数调试指南

## 1. 理论基础

### 1.1 PID控制原理
- **P (比例)**：响应当前偏差，决定系统的响应速度
- **I (积分)**：消除稳态误差，但过大会引起振荡
- **D (微分)**：预测未来趋势，抑制振荡，提高稳定性

### 1.2 循迹控制逻辑
```
偏差值 → PID控制器 → 转向修正量 → 左右轮速度差 → 小车转向
```

## 2. 传感器值分析

### 2.1 传感器布局（从左到右）
```
HW1  HW2  HW3  HW4
 ○    ○    ○    ○
```

### 2.2 偏差值含义
- `line_num = 0`：小车在线上，无需转向
- `line_num > 0`：小车偏左，需要向右转
- `line_num < 0`：小车偏右，需要向左转

### 2.3 偏差值大小
- `±10`：轻微偏离
- `±15`：中等偏离  
- `±20`：严重偏离

## 3. PID参数调试步骤

### 3.1 初始参数设置
```c
// 推荐的初始参数
Kp = 2.0    // 比例系数
Ki = 0.1    // 积分系数
Kd = 0.5    // 微分系数
base_speed = 80.0  // 基础速度
```

### 3.2 逐步调试方法

#### 第一步：只调P参数
1. 设置 Ki=0, Kd=0
2. 从 Kp=1.0 开始测试
3. 观察现象：
   - Kp太小：响应慢，转向不足
   - Kp太大：振荡，转向过度
   - Kp合适：能跟踪线路，略有振荡

#### 第二步：添加D参数
1. 在合适的Kp基础上，添加微分项
2. 从 Kd=0.3 开始测试
3. 观察现象：
   - Kd太小：仍有振荡
   - Kd太大：反应迟钝
   - Kd合适：振荡减小，响应平稳

#### 第三步：添加I参数
1. 在PD参数基础上，添加积分项
2. 从很小的值开始 Ki=0.05
3. 观察现象：
   - Ki太小：可能有稳态偏差
   - Ki太大：积分饱和，系统不稳定
   - Ki合适：消除稳态偏差

## 4. 调试参考参数

### 4.1 不同速度下的参考参数

#### 低速模式 (base_speed = 60)
```c
Kp = 1.5
Ki = 0.08  
Kd = 0.4
max_correction = 30
```

#### 中速模式 (base_speed = 100) 
```c
Kp = 2.5
Ki = 0.15
Kd = 0.8
max_correction = 50
```

#### 高速模式 (base_speed = 150)
```c
Kp = 3.5
Ki = 0.2
Kd = 1.2  
max_correction = 70
```

### 4.2 不同路况的参数调整

#### 直线为主的路况
- 增大Kd，减小Ki
- 提高base_speed

#### 弯道较多的路况  
- 增大Kp，适当增大Ki
- 降低base_speed

#### 路面不平整
- 增大滤波，适当增大Kd
- 降低base_speed

## 5. 常见问题及解决方案

### 5.1 小车不跟线
**现象**：传感器检测到线路，但小车不转向
**可能原因**：
- Kp太小
- base_speed设置错误
- 电机方向错误

**解决方案**：
- 增大Kp值
- 检查电机接线
- 确认传感器值正确

### 5.2 小车振荡
**现象**：小车左右摆动严重
**可能原因**：
- Kp过大
- Kd太小
- 积分饱和

**解决方案**：
- 减小Kp
- 增大Kd  
- 重置积分项或减小Ki

### 5.3 转弯性能差
**现象**：直线跟踪好，但过不了弯
**可能原因**：
- max_correction太小
- base_speed过高
- 传感器间距不合适

**解决方案**：
- 增大max_correction
- 降低转弯时的速度
- 调整传感器布局

### 5.4 稳态偏差
**现象**：小车始终偏向一侧
**可能原因**：
- 传感器安装偏差
- 电机性能不一致
- Ki太小

**解决方案**：
- 校准传感器位置
- 增加Ki项
- 软件补偿机械偏差

## 6. 调试工具和技巧

### 6.1 串口调试输出
```c
// 在line_follow_control函数中添加调试输出
printf("line_error:%d, pid_output:%.2f, left_speed:%.2f, right_speed:%.2f\n", 
       line_error, pid_correction, left_motor_speed, right_motor_speed);
```

### 6.2 LED指示
```c
// 用LED显示当前状态
if(line_error > 0) 
    HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, GPIO_PIN_SET);
else if(line_error < 0)
    HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, GPIO_PIN_SET);
```

### 6.3 分段测试
1. 先在直线上测试
2. 然后测试大弯道  
3. 最后测试复杂路径

## 7. 高级优化技巧

### 7.1 变速控制
```c
// 根据偏差大小动态调整速度
if(abs(line_error) > 15) {
    current_base_speed = base_speed * 0.7;  // 大偏差时减速
} else {
    current_base_speed = base_speed;        // 正常速度
}
```

### 7.2 前馈控制
```c
// 根据路况预测添加前馈量
if(detection_curve_ahead()) {
    feed_forward = calculate_curve_compensation();
    pid_correction += feed_forward;
}
```

### 7.3 自适应参数
```c
// 根据运行状态自动调整参数
if(error_history_variance > threshold) {
    Kd *= 1.1;  // 增强抗干扰能力
}
```

## 8. 测试验证

### 8.1 性能指标
- 直线跟踪精度：±2mm
- 弯道通过速度：>80% base_speed
- 振荡频率：<1Hz
- 稳定时间：<0.5s

### 8.2 测试用例
1. 直线跟踪测试
2. S形弯道测试  
3. 急转弯测试
4. 断线重连测试
5. 干扰抗性测试
