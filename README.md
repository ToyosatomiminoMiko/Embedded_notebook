*这个笔记本因为不小心的误操作清除了内容,但通过VScode的本地历史记录恢复了,故避免以后再次出现此类情况,现公开该笔记本(2024.10.24.03:53:41)*

##### FPAG 制造商

- intel/Altera
- AMD/Xilinx

##### MCU 制造商

- STMicroelectronics
- Texas Instruments

# `STM32F103ZET6`

- Flash 512 kBytes
- RAM 64 kBytes
- IO 112
- Frequecy(Max) 72 MHz

**Linker Settings**
default:

- Heap: 0x200
- Stack: 0x400

## § 0x00 名词解释

### **COM**

*Cluster Communication Port*

### *simplex*/*half-duplex*/*full-duplex*

单工/半双工/全双工

### TTL

*Transistor-Transistor Logic*
晶体管-晶体管逻辑

### TLE

*Time Limit Exceeded*
时间超限

### NRND

*Not Recommended for New Design*
不推荐用于新设计

1. 器件已经计划停产,或者已经停产有少量库存.
2. 有替代或升级的型号.

### PID

*proportional-integral-differential*
比例-积分-微分

### FFT

*Fast Fourier Transform*
快速傅里叶变换

### PWM

*Pulse Width Modulation*
脉冲宽度调制

### HID

*Human Interface Device*
人机接口设备

### HAL

**Hardware Abstraction Layer**
硬件抽象层

### BSP

**board support package**
板级支持包

### IRQ

**Interrupt Request**
中断请求

### DMA

**Direct Memory Access**
直接内存访问

## § 0x01 CubeIDE 疑难解答

微软输入法繁简切换`Ctrl`+`Shift`+`F`

### 生成HEX文件

1. **Project** ->
2. **Properties** ->
3. **C/C++ Build** ->
4. **Settings** ->
5. **MCU/MPU Post build outputs** ->
6. **Convert to Intel Hex file (-O ihex)**

### *The float formatting support is not enabled*

1. **Project** ->
2. **Properties** ->
3. **C/C++ Build** ->
4. **Settings** ->
5. **Tool Settings** ->
6. **MCU Settings** ->
7. **Use float with printf from newlib-nano (-u printf float)**

### 调试模式(Dubug)

- **step into**:进入子函数执行
- **step over**:不进入子函数执行
- **step out**:进入子函数后执行剩余部分返回上一层

### 路径包含设置

1. *Project_name* ->
2. **Properties**->
3. **C/C++ General** ->
4. **Paths and Symbols** ->
5. **Includes**

## 协议

#### 差分信号

振幅相同,相位相反

### **UART**

- *Universal Asynchronous Receiver Transmitter*
- 通用异步串行接收发送器
- 位识别方式: 波特率
- 半双工

### **USART**

- *Universal Synchronous Asynchronous Receiver Transmitter*
- 通用同步异步串行接收发送器
- 位识别方式: 时钟

### **RS232**

- *Recommended Standard 232*
- 位识别方式: 波特率

### **I2C**

- *Inter-Integrated Circuit*
- 位识别方式: 时钟电平变化 高电平有效
- 半双工

### **SPI**

- *Serial Peripheral interface*
- 位识别方式: 时钟电平变化 电平跳变时有效
- 全双工

### **RS485**

- *Recommended Standard 485*
- 位识别方式: 差分信号 波特率

### **CAN**

- *Controller Area Network*
- 控制器局域网
- 位识别方式: 差分信号 波特率

### **USB**

- *Universal Serial Bus*
- 位识别方式: 差分信号 波特率

## § 外设与 HAL 库

### C 整数类型

|       type       |  size  |          range           |
| :--------------: | :----: | :----------------------: |
| `unsigned char`  | 1 Byte |         [0,255]          |
|  `signed char`   | 1 Byte |        [-128,127]        |
| `unsigned short` | 2 Byte |        [0,65535]         |
|  `signed short`  | 2 Byte |      [-32768,32767]      |
|  `unsigned int`  | 4 Byte |      [0,4294967295]      |
|   `signed int`   | 4 Byte | [-2147483648,2147483647] |

#### GPIO写入

```c
void HAL_GPIO_WritePin(GPIOx, GPIO_Pin_x, GPIO_PIN_SET);
//high:   GPIO_PIN_SET
//low:    GPIO_PIN_RESET
```

#### GPIO读取

```c
GPIO_PinState HAL_GPIO_ReadPin(GPIOx, GPIO_Pin_x);
```

#### 延时

```c
void HAL_Delay(Delay);
// Delay: 延迟时间 ms

#### 未使用变量处理
```c
UNUSED(v);
```

### TIM 定时器

```c
__weak void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
```

定时器中断回调函数

#### PWM 控制

$$
frequency=\frac{ClockSource}{Prescaler}/InternalClockDivision
$$

```c
// 开启PWM
HAL_StatusTypeDef HAL_TIM_PWM_Start(&htimx, TIM_CHANNEL_x);
// htimx: 定时器; TIM_CHANNEL_x: 通道
htimx.Instance->CCRx = CCR;
// CCRx: 占空比(该寄存器共4个,每个对应一个通道)
__HAL_TIM_SET_AUTORELOAD(&htimx, 1); // 设置自动重装载值 频率
__HAL_TIM_SET_COMPARE(&htimx, TIM_CHANNEL_x, 50); // 设置占空比
htimx.Instance->CNT
// 外部传入高电平计数
```

#### encoder 编码器模式

开启编码器

```c
HAL_TIM_Encoder_Start(&htim, TIM_CHANNEL_ALL);
```

获取编码器值

```c
__HAL_TIM_GET_COUNTER(&htim1);
```

### UART

- `USART1`: host
- `USART2`: RS485
- `USART3`: I2C2
- `UART4`: SDIO
- `UART5`: SDIO

#### UART读写

```c
HAL_UART_Receive(&huartx, Data, 40, 1000);
HAL_UART_Transmit(&huartx, Data, 40, 1000);
/*
huartx:uart端口;需要读写的数据缓冲区Data;缓冲区大小,超时时间
*/
```

#### 串口打印

```c
char s[16];
uint8_t i = 0;
while (1) {
    sprintf(s, "test=%i;", i);
    HAL_UART_Transmit(&huart1, (uint8_t*) s, sizeof(s), 1000);
    HAL_Delay(1000);
    i++;
}
```

- Sample text:
    >*The Regents of the University of California.*

    (加州大学董事会)

*temperature* 温度

#### UART读写(中断模式)

```c
HAL_UART_Receive_IT(&huartx, Data, x);
HAL_UART_Transmit_IT(&huartx, Data, x);
/*
huartx:uart端口;需要读写的数据缓冲区Data;缓冲区大小
*/
```

#### uart中断回调函数

位于`stm32f1xx_hal_uart.h`中,该函数是`__weak`弱定义可重写

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
```

内容自定义,记得清除中断位接收下一次

#### uart不定长接收

位于`stm32f1xx_hal_uart.h`中,该函数是`__weak`弱定义可重写

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
```

### I2C

#### 写寄存器

```c
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
/*
hi2c i2c句柄指针
DevAddress 设备地址
MemAddress 寄存器地址
MemAddSize 寄存器地址之大小
pData 写入数据指针
Size 写入数据之大小
Timeout 超时时间
*/
```

#### 检查I2C操作是否失败

```c
if (HAL_I2C_Mem_Write(&hi2c, uint16_t DevAddress, MemAddress, MemAddSize, &pData, Size, Timeout) != HAL_OK)
    {
        HAL_UART_Transmit(&huart, (uint8_t*) "error!", 6, 10);
    }
```

### TFT-LCD

*Thin film transistor liquid crystal display*
**薄膜晶体管液晶显示器**

#### RGB565

`color = ((R << 11) | (G << 5) | B)`

#### `NT5510`(马老师的,坏掉了)

- 触控: `GT917S`
- resolution: `800*480`
  - `x_max = 480 - 1;`
  - `y_max = 800 - 1;`

#### `ST7789`

- 触控: none
- resolution: `320*240`
  - `x_max = 320 - 1;`
  - `y_max = 240 - 1;`

#### TFT-LCD: FSMC Mode and Configuration

- **FSMC**/Mode/NOR 1
- **Chip Select**: `FSMC_NE4`;
- **Memory type**: "LCD Interface";
- **LCD Rigster Select**: `FSMC_A10`(`RS`pin);
- **Data**: "16 bits"
- **GPIO**: "PB0->`LCD_BL`(`GPIO_PIN_SET`);
- **RESET**: auto, 开发板电路已连接

- NOR/PSRAM control
    Extended mode: Enabled
- NOR/PSRAM timing
    address setup time in..: 0
    Data setup time in HC..: 15
    Bus turn around time ..: 15
    Access mode: A
- NOR/PSRAM timing for write accsses
    Extended address setup..: 0
    Extended data setup time: 1
    Extended bus turn around: 15
    Extended access mode: A

#### lcd函数

初始化函数

```c
// 定义
void lcd_init(void);
// 调用
lcd_init();
```

#### 绘制图案

##### 雷达扫描

画线清除之前的点

```c
while (1)
{
    if (d <= 0)
        d = 200;
    line_y = sin((d - 1) * 1.8 * PI / 180) * (double) 101;
    line_x = cos((d - 1) * 1.8 * PI / 180) * (double) 101;
    lcd_draw_line(Ox, Oy, Ox - line_x, Oy - line_y, WHITE);
    HAL_UART_Receive(&huart2, receive_buffer, sizeof(receive_buffer), 0xFF);
    for (uint8_t i = 0; i < sizeof(receive_buffer); i++)
    {
        if (receive_buffer[i] == 'd' && receive_buffer[i + 1] == ':')
        {
            sprintf(s, "%c%c%c%c", receive_buffer[i + 2],
                    receive_buffer[i + 3], receive_buffer[i + 4],
                    receive_buffer[i + 5]);
            length = atoi(s);
            if (length > 2000)
                length = 2000;
            length = length / 20;
            point_y = sin(d * 1.8 * PI / 180) * (double) length;
            point_x = cos(d * 1.8 * PI / 180) * (double) length;
            lcd_draw_point(Ox - point_x, Oy - point_y, GREEN);
            d--;
            lcd_draw_line(Ox, Oy, Ox - line_x, Oy - line_y, BLACK);
            //sprintf(s0, "%d\r\n", length);
            //HAL_UART_Transmit(&huart1, (uint8_t*) s0, sizeof(s0), 0xFF);

        }
    }
}
```

### MPU6050

#### MPU6050 i2c address: `0xD0`

|   7   |   6   |   5   |   4   |   3   |   2   |   1   |  R/W  |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
|   1   |   1   |   0   |   1   |   0   |   0   |   0   |  1/0  |

視頻教程:

- `BV13N4y1y7Js`
- `BV1z84y1R7JC`
- `BV11w411z7o8`

### EEPROM:24C02

#### size

`256 Byte`

#### 24C02 i2c address: `0xA0`

|   7   |   6   |   5   |   4   |  A2   |  A1   |  A0   |  R/W  |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
|   1   |   0   |   1   |   0   |   0   |   0   |   0   |  1/0  |

`A0`,`A1`,`A2`>>`GND`

### 内部Flash

#### size

`512 kByte`
512 kByte = 2 kByte * 256 page

#### 主存储器

start: `0x08000000`(B0, B1 >> GND)
stop: `0x0807FFFF`

`HAL_FLASH`系列函数

### Flash:W25Q128

#### size

`16 MByte`
128 Mbit = 16 MByte = 256 Block *64 kByte
1 Block = 16 Sector* 4 kByte
1 Sector = 4 kByte
**最小擦除单位为一个扇区(Sector)**

*Device ID: `0x5217`*
> *When /CS is high the device is deselected*
>
> 当`CS`为高电平时设备不选中

#### Instruction

1. Write Enable 写使能
`0x06`
2. Write Enable for Volatile Status Register 易失性状态寄存器写使能
`0x50`
3. Write Disable 写禁用
`0x04`
4. Read Status Register-1, Status Register-2 & Status Register-3 读状态寄存器1,2,3
r1`0x05`;r2`0x35`;r3`0x15`
5. Write Status Register-1, Status Register-2 & Status Register-3 写状态寄存器1,2,3
r1`0x01`;r2`0x31`;r3`0x11`
6. Read Data 读数据
`0x03`
7. Fast Read 快速读
`0x0B`
8. Fast Read Dual Output 快读双输出
`0x3B`
9. Fast Read Quad Output 快读四输出
`0x6B`

### DAC

**analog** = **analogue**

STM32的DAC输出缓存做的有些不好,如果使能的话,虽然输出能力强一点,但是输出没法到0,这是个很严重的问题.
所以本章我们不使用输出缓存.即设置该位为1.

```c
DACCHx_Config.DAC_Trigger=DAC_TRIGGER_NONE; // 不使用触发功能
DACCHx_Config.DAC_OutputBuffer=DAC_OUTPUTBUFFER_DISABLE; // 不使用输出缓存
```

使能DAC转换通道

```c
HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef* hdac, uint32_t Channel);
```

设置DAC的输出值

```c
HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef* hdac,
uint32_t Channel, uint32_t Alignment, uint32_t Data);
```

Alignment:

1. `DAC_ALIGN_8B_R`
2. `DAC_ALIGN_12B_L`
3. `DAC_ALIGN_12B_R`
单DAC通道1,采用12位右对齐格式,用户将数据写入`DAC_DHR12Rx[11:0]`位
(实际是存入`DHRx[11:0]`位)

### ADC

不要让ADC的时钟超过14M,否则将导致结果准确度下降

```c
double adc_v;
char s[8];
while (1)
{
   HAL_ADC_Start(&hadc);
   if (HAL_ADC_PollForConversion(&hadc, 0xFF) == HAL_OK)
   {
      adc_v = HAL_ADC_GetValue(&hadc) * (3.3 / 4096);
   }
   sprintf(s, "%5.3f\r\n", adc_v);
   HAL_UART_Transmit(&huart1, (uint8_t *)s, sizeof(s), 0xFF);
}
```

### MS53L0M/vl53l0x

#### vl53l0x i2c address: `0x52`

|   7   |   6   |   5   |   4   |   3   |   2   |   1   |  R/W  |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
|   0   |   1   |   0   |   1   |   0   |   0   |   1   |  1/0  |

### CRC (Cyclic Redundancy Check, 循环冗余校验)

1. **Categories** ->
2. **Computing** ->
3. **CRC** ->
4. **Activated**

```c
uint32_t HAL_CRC_Accumulate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength);
uint32_t HAL_CRC_Calculate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength)
```

`HAL_CRC_Accumulate`不会复位,每次结果都不同

### 晶振 (Crytal)

| Crytal | frequency  |    (Hz) |
| :----: | :--------: | ------: |
|  `Y1`  | 32.768 kHz |   32768 |
|  `Y2`  | 8.000 MHz  | 8000000 |

### OLED:SSD1306

#### size

8 bit *128* 8
`1024 Byte`

1Byte 对应像素的结构
<table border="1">
    <tbody>
        <tr>
            <td><code>7</code></td>
        </tr>
        <tr>
            <td><code>6</code></td>
        </tr>
        <tr>
            <td><code>5</code></td>
        </tr>
        <tr>
            <td><code>4</code></td>
        </tr>
        <tr>
            <td><code>3</code></td>
        </tr>
        <tr>
            <td><code>2</code></td>
        </tr>
        <tr>
            <td><code>1</code></td>
        </tr>
        <tr>
            <td><code>0</code></td>
        </tr>
    </tbody>
</table>

#### SSD1306 i2c address: `0x78`

|   7   |   6   |   5   |   4   |   3   |   2   |   1   |  R/W  |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
|   0   |   1   |   1   |   1   |   1   |   0   |   0   |  1/0  |

#### 提高帧率

设置I2C快速模式
**Master Features**/
**I2C Speed Mode: Fast Mode**

### VFD:CIG25-1605N

(5 pixel *7 pixel )* 16 *2
16 Byte* 16 = 256 Byte

#### SPI config

1. > *Display control command and data are written by an 8-bit serial transfer*<br>
   > 显示控制命令和数据由**8位**串行传输写入.

2. > *Setting the CS pin to "Low" level enables a data transfer.*<br>
   > 将CS引脚设置为**低电平**可启用数据传输.

3. > *Data is 8 bits and is sequentially input into the DA pin from LSB (LSB first).*<br>
   > 数据为8位,从LSB顺序输入到DA引脚(**LSB优先**).

4. (见图)时钟线空闲时间为**高电平**. **第二边沿**(由低到高)

5. > *When data is written to RAM (DCRAM, ADRAM, CGRAM) continuously, addresses are internally incremented automatically.*<br>
   > 当数据连续写入RAM(DCRAM,ADRAM,CGRAM)时,地址会在内部自动递增.

#### Pin

- `GND`: -
- `VCC`: +5v
- ~~`EN`~~: nonuse (不关注)
- `RST`: Reset
- `CS`: `NSS` 空闲为高
- `CP`: `SCK` 空闲为高
- `DA`: `MOSI` 空闲为低

#### MOSI 发送 1 Byte 函数

```c
void MOSI_Byte(uint8_t byte)
{
 for (uint8_t i = 0; i < 8; i++)
 {
  CLK_RESET;
  if (byte & 0x01)
   DATA_SET;
  else
   DATA_RESET;
  byte = byte >> 1;
  HAL_Delay(1);
  CLK_SET;
  HAL_Delay(1);
 }
}
```

- Sample text:
    > *Democratic Republic of North Japan*
    >
    > *Шойгу! Герасимов! где сука боеприпасы?*
    >
    > *Shoigu gerasimov gdie suka boiepripacy*

### SDIO

#### `SDIO_CMD`

SDIO的所有命令和响应都是通过`SDIO_CMD`引脚传输的,任何命令的长度都是固定为**48**位.

SDIO command report:
<table border="1">
    <thead align="center">
        <tr>
            <th>bit</th>
            <th>7</th>
            <th>6</th>
            <th>5</th>
            <th>4</th>
            <th>3</th>
            <th>2</th>
            <th>1</th>
            <th>0</th>
        </tr>
    </thead>
    <tbody align="center">
        <tr>
            <td><b>5</b></td>
            <td><code>start</code></td>
            <td><code>transmission</code></td>
            <td colspan="6"><code>command_index</code></td>
        </tr>
        <tr>
            <td><b>4</b></td>
            <td colspan="8" rowspan="4"><code>arguments</code></td>
        </tr>
        <tr>
            <td><b>3</b></td>
        </tr>
        <tr>
            <td><b>2</b></td>
        </tr>
        <tr>
            <td><b>1</b></td>
        </tr>
        <tr>
            <td><b>0</b></td>
            <td colspan="7"><code>CRC_7</code></td>
            <td><code>stop</code></td>
        </tr>
    </tbody>
</table>

所有的命令都是由STM32F1发出,其中**开始位**,**传输位**,**CRC7**和**结束位**由SDIO硬件控制,我们需要设置的就只有**命令索引**和**参数**部分

#### configuration

初始化:

```c
hsd.Instance = SDIO;
hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_ENABLE;
hsd.Init.ClockDiv = 0x06;
```

- `hsd.Init.BusWide` 在初始化时必须保持 1 位宽(`SDIO_BUS_WIDE_1B`)
    位宽可在初始化完成后修改

    ```c
    /* USER CODE BEGIN SDIO_Init 2 */
    if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK)
    {
    Error_Handler();
    }
    /* USER CODE END SDIO_Init 2 */
    ```

- `hsd.Init.ClockDiv`公式:*CLKDIV*
    $$ SDIO\_CK =\frac{SDIOCLK}{(2 + CLKDIV)} $$
SD卡基本信息:

```t
LCD_ID: 5510
Card_ManufacturerID: 3
Card_RCA: 58916
ID: 3
Card_Capacity: 29 MB
Card_BlockSize: 512
```

### Motor

#### motor i2c address: `0x68`

|   7   |   6   |   5   |   4   |   3   |   2   |   1   |  R/W  |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
|   1   |   1   |   0   |   1   |   1   |   0   |   0   |  1/0  |

### RS485

`USART2`

- 接收模式: `RS485_RE` = 0
- 发送模式: `RS485_RE` = 1

### USB/HID

#### USB

1. **Connectivity** ->
2. **USB** ->
3. **Mode** ->
4. **Device (FS)**

#### HID

1. **Middleware and Software Packs** ->
2. **USB_DEVICE** ->
3. **Mode** ->
4. **Class For FS IP** ->
5. **Human Interface Device Class (HID)**
包含中间件

```c
#include "usbd_hid.h"
extern USBD_HandleTypeDef hUsbDeviceFS;
```

#### Joystick(摇杆)

**ADC** + **USB**

#### USB HID report

<table border="1">
    <thead align="center">
        <tr>
            <th>Byte</th>
            <th>function</th>
            <th>0</th>
            <th>1</th>
            <th>2</th>
            <th>3</th>
            <th>4</th>
            <th>5</th>
            <th>6</th>
            <th>7</th>
        </tr>
    </thead>
    <tbody align="center">
        <tr>
            <td><b>0</b></td>
            <td><b>button</b></td>
            <td><code>left</code></td>
            <td><code>right</code></td>
            <td><code>central</code></td>
            <td colspan="5"><code>null</code></td>
        </tr>
        <tr>
            <td><b>1</b></td>
            <td><b>X-axis movement</b></td>
            <td colspan="8"><code>left[-127,+127]right</code></td>
        </tr>
        <tr>
            <td><b>2</b></td>
            <td><b>Y-axis movement</b></td>
            <td colspan="8"><code>up[-127,+127]down</code></td>
        </tr>
        <tr>
            <td><b>3</b></td>
            <td><b>wheel movement</b></td>
            <td colspan="8"><code>down[-127,+127]up</code></td>
        </tr>
    </tbody>
</table>

1. 第一个字节表示按键,bit0对应左键,bit1对应右键,bit3对应中键;0表示未按,1表示按下;
2. 第二个字节表示x轴(即鼠标左右移动,0表示不动,正值表示往右移,负值表示往左移,范围-127-127,绝对值对应了移动量大小);
3. 第三个字节表示y轴(即鼠标上下移动,0表示不动,正值表示往下移,负值表示往上移,范围-127-127,绝对值对应了移动量大小);
4. 第四个字节表示鼠标滚轮(正值为往上滚动,负值为往下滚动,-127-127,绝对值对应了移动量大小).
5. 设置完成后将报文发送

    ```c
    USBD_HID_SendReport(&hUsbDeviceFS, HID_buf, 4);
    ```

### CAN

收发芯片: `JTA1050`

### RTC

#### RTC configuration

1. **Timers** ->
2. **RTC** ->
3. **Mode** ->
4. **Active Clock Source**,**Active Calendar**
5. **RTC OUT** ->
6. **RTC Output on the Tamper pin**

```c
// 定义保存日期和时间的类
RTC_DateTypeDef date;
RTC_TimeTypeDef time;
// 获取时间和日期到类
HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
```

格式化输出

```c
sprintf(s_date, "20%02d.%02d.%02d", date.Year, date.Month, date.Date);
sprintf(s_time, "%02d:%02d:%02d", time.Hours, time.Minutes, time.Seconds);
```

#### 时间日期取值范围

- Year     [0,99]
- Month    [1,12]
- Date     [1,31]
- Hour     [0,23]
- Minutes  [0,59]
- Seconds  [0,59]
hex:
`59` = `0x3b`
`23` = `0x17`
`31` = `0x1F`
`99` = `0x63`

### ST7302

- resolution: 250*122

125 * 0x21 = 4125
1Byte 对应像素的结构

<table border="1">
    <tbody>
        <tr>
            <td><code>7</code></td>
            <td><code>6</code></td>
        </tr>
        <tr>
            <td><code>5</code></td>
            <td><code>4</code></td>
        </tr>
        <tr>
            <td><code>3</code></td>
            <td><code>2</code></td>
        </tr>
        <tr>
            <td><code>1</code></td>
            <td><code>0</code></td>
        </tr>
    </tbody>
</table>

31 * 4 = 124
最后两行在屏幕外面

- Sample text:
    >*「昏睡レイプ！野獣と化した先輩」*

- `CS`: low enable

### GC9A01

HSB first

- resolution: 240*240
圆形屏幕

### Memory Management

内存池:

内存管理表:
内存管理表的每一个项对应内存池的一块内存
内存管理表的项值:

- 当该项值为 0 的时候,代表对应的内存块未被占用
- 当该项值非零的时候,代表该项对应的内存块已经被占用,其数值则代表被连续占用的内存块数

```c
/*
定义
memx: 
SRAMIN 内部 SRAM (64 kB);
SRAMEX 外部扩展 SRAM (本开发板不支持);
*/
void my_mem_init(uint8_t memx);
// 调用
my_mem_init(SRAMIN);
```

### 示波器

oscillo-meter
oscillo-graph
oscillo-scope
示波器的本质: ADC(电压表)+屏幕

### FATFS

#### Configuration

1. **Middleware and Software Packs** /
2. **FATFS** / **Mode**
3. **User-defined** | **SD Dard**(只有SD卡可选SD)
4. `USE_LABEL`: `Enable` (支持卷标, 设置磁盘名称)
   `CODE_PAGE`: (选择语言,支持简体中文)
   `USE_LFN`: `...HEAP`(支持长文件名)
   `VOLUMES`: `2`(支持多个磁盘)

#### 扩大堆区

1. **Project Manager**
2. **Project**
3. **Linker Setting**
4. **Minimum Heap Size** = `0x1000`

SSE 600519 605337
*Mis días contigo son los más felices que he tenido*

### 步进电机(S42)

**`2MD5050`驱动器**

- `ENA`: 信号有效,输出关闭,电机线圈电流为零,电机处于无力矩状态,可自由转动
- `DIR`: 信号有效,电机顺时针(clockwise)转动;信号无效,电机逆时针(anticlockwise)转动
- `PUL`: 接收到信号后,按照设定方向转动(输入频率 <= 200 kHz)

### SR04

超声波测距

#### config

**Timers**/TIMx/Mode/
**Clock Source**/**Internal Clock**
**Channex**/**Input Capture direct mode**&
**Input Capture indirect mode**

Internal Clock: 72 MHz
Prescaler: 72
**NVIC Settings**/**TIMx capture compare interrupt**
开启定时器与通道捕获

```c
HAL_TIM_Base_Start(&htimx);
HAL_TIM_IC_Start(&htimx, TIM_CHANNEL_x); // direct
HAL_TIM_IC_Start_IT(&htimx, TIM_CHANNEL_x); // indirect
```

触发模块启动

```c
void SR04_Trigger()
{
 HAL_GPIO_WritePin(SR04_TRIG_GPIO_Port, SR04_TRIG_Pin, GPIO_PIN_SET);
 HAL_Delay(1);
 HAL_GPIO_WritePin(SR04_TRIG_GPIO_Port, SR04_TRIG_Pin, GPIO_PIN_RESET);
 __HAL_TIM_SET_COUNTER(&htimx, 0);
}
```

温度和音速的关系(近似):
$$
V= 331 \times \sqrt {\frac {1+T} {273}}
$$
T: 温度(temperature)
V: 音速(speed of sound)

### 封装方式

- QFP
    *Quad Flat Package*
    四侧引脚扁平封装
  - LQFP
  - TQFP
- BGA
*Ball Grid Array*
- LGA
*Land Grid Array*
- PGA
*Pin Grid Array*

### REMOTE

|-|-|-|
|:-:|:-:|:-:|
|power|up|alientek|
|left|play|right|
|vol-|down|vol+|
|1|2|3|
|4|5|6|
|7|8|9|
|0||delete|

## 显示技术

- EL
*Electro-Luminescent*
电致发光
- LCD
*Liquid Crystal Display*
液晶显示
- OLED
*Organic Light-Emitting Diode*
有机发光二极管
- LED
*Light Emitting Diode*
发光二极管
- CRT
*Cathode Ray Tube*
阴极射线管
- VFD
*Vacuum Fluorescent Display*
真空荧光显示器
- NIXIE
*Nixie tube*
辉光管

# Turing Completa

## Assembly

- `0b00`: Immediate 立即数
- `0b01`: Compute 计算
- `0b10`: Copy 复制
- `0b11`: Condition 条件
*Internet Software Infrastructure*
