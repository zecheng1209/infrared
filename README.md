# STM32F1 红外通信模块

基于STM32F1的红外通信系统，支持红外发送/接收、CRC校验、ACK确认机制以及CAN总线桥接功能。

## 功能特性

- **红外通信**：基于改进的NEC协议（时序为标准NEC的一半）
- **38kHz载波**：使用TIM1 PWM生成38kHz红外载波
- **CRC8校验**：数据传输完整性校验
- **ACK/NACK机制**：双向通信确认
- **超时重传**：提高通信可靠性
- **CAN总线桥接**：CAN与红外数据双向转换
- **模块ID识别**：支持多模块组网

## 硬件资源

### 引脚分配

| 功能 | 引脚 | 说明 |
|------|------|------|
| 红外发射 | PA8 | TIM1_CH1 - 38kHz PWM |
| 红外接收 | PA15 | TIM2_CH1 - 输入捕获 |
| CAN RX | PA11 | CAN1接收 |
| CAN TX | PA12 | CAN1发送 |

### 定时器配置

- **TIM1**：38kHz PWM载波生成
- **TIM2**：输入捕获，用于红外接收
- **TIM3**：时序控制，用于红外发送

## 通信协议

### 红外时序

- 起始信号：4.5ms高 + 2.25ms低
- 数据位0：280us高 + 280us低（总计560us）
- 数据位1：280us高 + 840us低（总计1120us）
- 载波频率：38kHz

### 数据帧格式

| 字节0-7 | 字节8 |
|---------|-------|
| 用户数据 | CRC8校验 |

### ACK帧格式

| 字节0 | 字节1 |
|-------|-------|
| 0xA5 | 0xA5 | (ACK) 或 0x5A 0x5A (NACK) |

## API接口

### 初始化

```c
void IR_Init(void);
```

### 数据发送

```c
// 发送单帧数据
bool IR_SendData(uint8_t *data, uint8_t length);

// 带重传发送
bool IR_SendDataWithRetry(uint8_t *data, uint8_t length, uint8_t max_retry);

// 带ACK确认发送
bool IR_SendDataAndWaitAck(uint8_t *data, uint8_t length, uint8_t max_retry);
```

### 接收处理

```c
// 在主循环中检查接收超时
void IR_CheckRxTimeout(void);

// 处理接收到的数据帧
void IR_ProcessReceivedFrame(uint8_t *data, uint8_t length);
```

### CAN接口

```c
// 发送CAN数据
bool CAN_SendData(uint8_t *data, uint8_t length, uint32_t std_id);
```

## 使用说明

### CAN → 红外

1. 发送CAN帧，ID设为目标模块ID
2. 数据区为8字节用户数据
3. 模块自动通过红外发送并等待ACK确认

### 红外 → CAN

1. 红外接收数据后自动验证CRC
2. CRC正确则发送ACK，并通过CAN转发（ID=源模块ID）
3. CRC错误则发送NACK，不转发

### 模块配置

在 `infrared.h` 中修改：
```c
#define IR_MODULE_ID    0x01   // 修改为唯一模块ID
```

## 编译环境

- STM32CubeMX
- Keil MDK-ARM / STM32CubeIDE
- STM32F1xx HAL库

## 注意事项

- 两个相同模块通信时，时序参数不宜过低
- 红外发射/接收头需对准，避免遮挡
- 模块ID在同一网络中必须唯一

- ##以上为AI生成，仅供参考
