# Remote 遥控接收驱动

本目录有 S.BUS 和 DBUS 两套不同协议，以及复用 S.BUS 的 WFLY 通道适配。`Control_Task` 在 `RobotCmd_Init()` 后通过 `RemoteInput_Init()` 把 UART5 绑定为 S.BUS；每周期从完整帧快照更新 `InputState(Remote)`，由 RobotCmd 的固定仲裁使用。DBUS 仍未绑定 UART。

| 驱动 | 协议与 UART | 接收行为 | 在线/失控语义 |
| --- | --- | --- | --- |
| `remote_control.c/.h` | DJI DT7/DR16 DBUS，18 字节；具体 UART 波特率、DMA 接线需按遥控器和 CubeMX 配置核对 | 回调只取本次 chunk 最后 18 字节并校验通道与开关；不跨回调拼帧，拆包或合包中的较早帧会丢失 | 最近 100 ms 有合法帧视为在线；DBUS 代码无独立 failsafe 位解析 |
| `sbus.cpp/.h` | 标准 S.BUS，25 字节；当前 UART5 配为 100000 baud、偶校验、2 停止位、RX DMA（HAL `UART_WORDLENGTH_9B`） | 跨回调累积、查找帧头并在坏字节后重同步，解码 16 路模拟通道及 CH17/18 | 最近 100 ms 收到结构合法完整帧为在线；`SBUS_IsHealthy()` 另要求 frame-lost/failsafe 均未置位 |
| `wfly.cpp/.h` | WFLY 普通 W.BUS/S.BUS 通道语义；复用上面的接收驱动 | `WFLY_ReadLatest()` 从同一 S.BUS 快照转换摇杆、开关与旋钮，不另绑 UART 或注册 Daemon | 原始时戳、序号和失控位保存在 `state.sbus`；读取成功不表示健康或已解锁 |

两者不可互换输入。BSP 的 ReceiveToIdle 回调只交付 DMA chunk，不保证一回调等于一帧；DBUS 如需实车接入，应先补有界流式解析，并验证断线输出。S.BUS 的 CH1/CH2/CH7/CH10 已映射底盘调试目标；50 ms 帧时效、failsafe 和 200 ms 回中去抖由 RemoteInput 与仲裁检查。它不直接发布 Topic。

S.BUS 初始化时注册静态 100 ms Daemon，注册失败返回 false 并保持不可用；完整帧通过 header/footer 检查后才 Feed，噪声和不完整帧不 Feed。`SBUS_IsOnline()` 统一使用 Daemon 当前时间判断（age<100 ms），`IsDataValid()` 等价于 Online；frame-lost/failsafe 帧仍维持 Online，但 `IsHealthy()` 为 false。`latest_frame.timestamp_ms` 保留给 RemoteInput 的 50 ms 实时时效和 200 ms 回中恢复，Daemon 不接管 RobotCmd 安全许可，也不重启 UART。StatusTask 100 Hz CheckAll，UART 恢复仍由 BSP 负责。参见 [Daemon](../../../System/Daemon/README.md)。

## WFLY 接入与协议依据

参考工程 `JiuXian_SP_gimbal_Xpp` 的 `Drv/WFLY` 实现了解码与通道分配，但未校验帧尾、
未处理 CH17/18 和失控位、不跨回调拼帧；RemoteControl 回调在解析失败时也刷新在线计时。
本框架复用现有 S.BUS 流式解析和 Daemon，WFLY 层只做完整快照到设备语义的转换。

- [天地飞 ET16 Pro 原厂说明书 V1.01（镜像）](https://manuals.plus/m/a80773a57a2b8629a10b1a860d55e87ecb930352ff3315d8a4b2bf6b227ff1dd)：RF209S 的 W.BUS 兼容 S.BUS、2048 分辨率；端口、通道和失控保护可以设置。
- [Bolder Flight S.BUS 格式说明](https://github.com/bolderflight/sbus)：100000 baud、反相 8E2，25 字节；`0x0F` 帧头、22 字节连续的 16×11 bit 通道、状态字节、帧尾。状态字节 bit0/1 为 CH17/18，bit2 为 frame-lost，bit3 为 failsafe。

这里只支持普通 W.BUS/S.BUS、帧尾 `0x00`；不支持 W.BUS2、S.BUS2 或 200000 baud Fast S.BUS。
S.BUS 没有帧级 CRC，header/footer 检查只确认结构，不能保证检测所有传输错误。
接收机的精确型号和固件未从参考源码确定，不能据此认定所有 WFLY 产品具有相同通道配置。

### 公开接口与通道

先由唯一所有者 `RemoteInput_Init()` 调用 `SBUS_Init(&huart5)`，之后在任务中读取：

```cpp
#include "wfly.h"

void ReadReceiverExample()
{
    Struct_WFLY_State state{};
    if (WFLY_ReadLatest(&state))
    {
        // state.sbus.channels 已减 1024；加回 1024 可还原 0～2047 的线上值。
        // state.sbus.sequence/timestamp_ms/frame_lost/failsafe 供输入适配判断新鲜度和许可。
        // 不在这里重复绑定 UART，也不直接向电机写输出。
    }
}
```

`WFLY_ReadLatest()` 不消费帧，也不为旧帧更新时间；无完整帧或空指针时返回 false 并保持输出不变。
已经过期或带失控位的完整帧仍可读，用于诊断；控制链须同时检查实际接收年龄和失控标志。

| 通道（1 基） | `Struct_WFLY_State` 字段 | 转换 |
| --- | --- | --- |
| CH4 / CH2 | `left_remote_x` / `left_remote_y` | X 正向，Y 取反；[-1,1] |
| CH1 / CH3 | `right_remote_x` / `right_remote_y` | 正向；[-1,1] |
| CH5/6/7/8/11/13 | `switches[0..5]` | 依次为六个三档开关；低/中/高 |
| CH12 / CH14 | `left_bool_switch` / `right_bool_switch` | 原始值 >1024 为 true，否则 false |
| CH9 / CH10 | `knob_up_left` / `knob_up_right` | [0,2]，中点 1 |
| CH15 / CH16 | `knob_down_left` / `knob_down_right` | [-1,1]，中点 0 |

通道分配来自参考工程，是一套具体的遥控器配置，不是 W.BUS 固定的物理控件分配。
按本次选择统一中点 1024、半量程 671（对应 353～1695），不保留参考工程 CH4 中点 1031 的偏置。
模拟量先除以半量程并限幅；上方旋钮再加 1。设备层不加死区，底盘输入适配添加 0.04 死区。
三档开关原始值 <924 为 `WFLY_SWITCH_LOW`，>1124 为 `WFLY_SWITCH_HIGH`，包含两端的中间区间为 `WFLY_SWITCH_MIDDLE`。
这些量程初值不代表所有 WFLY 接收机的出厂标定，接入前核对发射机通道配置与实际反馈。

### 启用与接线

`H7_REMOTE_WFLY` 默认 OFF。SingleBoard/GimbalBoard 编入 WFLY 读取接口；打开选项后才切换
`RemoteInput` 的底盘映射。ChassisBoard 没有本地遥控输入链。当前 SingleBoard 启用示例：

```sh
cmake --preset SingleBoard -DH7_REMOTE_WFLY=ON
cmake --build --preset SingleBoard
```

恢复旧映射时以同一 preset 配置 `-DH7_REMOTE_WFLY=OFF`。不需要修改 IOC 或 UART DMA 缓冲。
具体底盘方向、限幅和解锁条件统一维护在 [Input 指南](../../../Application/Input/README.md#可选-wfly-底盘映射)。

UART5 RX 为 PD2，当前配置为 100000 baud、8 数据位加偶校验（HAL 9B）、2 停止位、RX DMA。
标准 S.BUS 使用反相电平，但当前 UART5 未启用硬件 RX 反相；须核对 MC-02 接口是否已有反相电路、
接收机实际输出极性与电平后再决定接线或板级配置，不能直接叠加一次反相。
接收机端口须设为普通 W.BUS 输出，不能把 PWM、PPM 或 W.BUS2 端口当作本实现的输入。
UART 错误恢复继续由现有 BSP 管理。

## 实机验收

固件构建沿用当前选中的板型、构建配置和应用开关。实机还需验证 UART 极性/电平、
全部通道方向和端点、发射机关机后的失控位/输出、拔线停机和回中恢复；主机回归与编译不能替代这些检查。

旧资料中的拨轮、左右开关与键鼠/视觉模式映射是**历史设计示例**。当前 VTM/键鼠/Vision 只有安全禁用的 InputState 接口，尚未配置实机模式开关和接收协议。
