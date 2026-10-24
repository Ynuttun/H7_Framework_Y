# 裁判系统协议模块

## 当前状态

本目录提供裁判系统帧解析、UI 绘制与 VTM 解析代码。`H7_APP_CLIENT_UI` 默认关闭；
启用时 `System_Init()` 按 `BoardHardware::referee_uart` 统一绑定 Referee，StatusTask
非阻塞绘制 [ClientUI HUD 示例](../../../Application/ClientUI/README.md)。
裁判缓冲能量、17 mm 热量/上限及弹量已接入 UI 显示；功率控制和发射互锁仍未接入，VTM 不随 UI 自动绑定。

## 协议解析与 UART 接入

调用 `RefereeInit(UART_HandleTypeDef*)` 才会注册 `RefereeRxCallback()`；传入的 UART 需已具备 ReceiveToIdle RX DMA。BSP 回调参数是一段 DMA chunk，可能包含半帧、多帧或杂字节。`JudgeReadData()` 用 320 字节静态缓存跨 chunk 重组帧，验证帧头 CRC8 和整帧 CRC16，然后按 CmdID 复制到 `referee_info_t`。`RefereeReceiveData()` 也调用相同解析器。VTM 使用本目录的独立解析入口。

已知固定长度命令要求声明长度、协议常量和目标结构大小完全一致；超长帧被拒绝，CRC 错帧逐字节重同步。解析状态由 UART 回调持有；`RefereeReceiveData()` 不应与 UART 中断并发调用。

## 在线状态与发送

`RefereeIsOnline()` 以最近一次 CRC 校验通过的完整帧为依据，超时门限为 500 ms；它不保证每个业务字段都已更新或有效。`RefereeSend()` 在 UART 提交成功后调用 `osDelay(115)`，只能在允许阻塞的任务上下文调用，不能从 ISR 或 1 kHz 控制周期调用。调用它之前必须先完成 `RefereeInit()`。

Referee 与 VTM 分别持有静态 Daemon，C 解析器通过 `referee_daemon.h/.cpp` 的薄 C 接口接入；Init 注册失败返回 NULL 并保持 Disabled，不使用动态内存。删除 Referee 的独立 `referee_last_valid_tick`，Online 统一按 Daemon 的绝对时间窗判断；StatusTask 100 Hz CheckAll 只记录活性与跃迁，不承担整车安全策略或 UART 重启。

Referee 只有完整 CRC8/CRC16、长度检查通过的帧才 Feed；已知命令的载荷长度仍必须匹配，未知但合法的命令可证明链路在线，不写入已知业务数据结构。超时仍为 500 ms，统一边界为 age<500 ms；Enabled 表示 UART/Daemon 已注册，DataValid 当前等价于 Online，Healthy 为 Enabled && Online。

VTM 的 `VTMIsEnabled/Online/DataValid/Healthy()` 与上述通信设备语义一致。完整 CRC16 合法遥控帧，或 CRC8/CRC16 与完整声明长度校验通过的图传链路帧才 Feed；坏帧、半帧不 Feed。图传业务载荷仍未实现，DataValid/Online 不保证遥控字段都已更新。门限暂定 300 ms，需按实际图传帧周期上车验证。保留当前 chunk 内解析方式，本轮不实现跨 chunk 重组；长度计算使用宽整数，避免溢出后错误地 Feed。未来业务接入仍须独立检查对应载荷 freshness。参见 [Daemon](../../../System/Daemon/README.md)。

## 非阻塞 UI 接口

`referee_ui_sender.h` 提供 `RefereeUITryDraw`（1/2/5/7 图元）、`RefereeUITryText`
和 `RefereeUITryDeleteLayer`。组包复用现有协议布局与 CRC，序号只在提交成功后递增。
底层 `RefereeTrySend()` 要求 TX DMA，返回 0 时未接受，不会走阻塞发送回退。
仅允许单一任务调用；接口不含队列、延时或自动限频，调用方负责速率与失败重试，
不要与旧阻塞发送器争用串口。DMA BSP 复制内容，返回后可复用调用方缓冲。

`RefereeReadUIRobotId()` 在短 PRIMASK 临界区复制身份与时效，仅合法且固定长度匹配的
`0x0201` 更新；其他合法帧不能续期身份，age < 500 ms，首版支持红蓝步兵 3/4/5。
失败保持输出不变。它不是整个 `referee_info_t` 的一致业务快照。
`RefereeUIIsReady()` 仅表示已绑定且有 TX DMA，不等于裁判在线或客户端收到。

## UI 裁判显示快照

`RefereeReadUISnapshot(referee_ui_snapshot_t*)` 是 C 兼容读取接口，在短 PRIMASK 临界区
复制显示数据及时间戳，并恢复原中断状态；返回成功只表示快照读取成功。
未初始化或空指针返回 0 且不修改输出。三个 received 标志区分从未收到与有效零值：

| 来源 | 字段 | 源时间 |
| --- | --- | --- |
| 0x0201，13 字节 | barrel_heat_limit | status_ms |
| 0x0202，14 字节 | buffer_energy_j、barrel_heat（17 mm） | heat_ms |
| 0x0208，8 字节 | projectile_allowance（17 mm） | allowance_ms |

只有对应命令的长度和 CRC 均合法时才更新该组数值及时间。接口返回原始时间，
不自动续期或将过期值置零；UI 分别按 HAL tick 无符号差 <500 ms 判断有效，支持 tick 回绕。
初始化重置全部 received 标志。底层协议长度保持现有定义，不自动兼容老车其他版本。
通用 Capture 将此快照复制到显示 Topic；热量或允许量不进入发射许可决策。

## Known Issues

- `referee_info_t` 仍由中断更新，任务直接读取多个字段可能跨帧；UI 使用上述专用快照，其他业务消费前仍需增加相应快照。
- 裁判业务数据尚未进入控制 Topic、Chassis 或 Shoot；ClientUI 有独立显示 Topic，无独立 UI 任务。
- 旧 UI 发送接口仍有 115 ms 延时；新 HUD 仅走非阻塞接口，实际 UART/客户端行为仍须实机验证。

## 历史 UI 示例

旧框架使用 `MyUIInit()`、`MyUIRefresh()`、`UICharDraw()`、`UIRectangleDraw()`、`UIFloatDraw()`、`UILineDraw()` 与 `UIRefresh()` 绘制功率条，并在独立 UI 任务以约 10 Hz 刷新。它仅是移植参考；当前板级任务没有创建该任务，示例中的旧类型和数据源不可直接复制到生产代码。
