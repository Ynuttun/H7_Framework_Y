#include "remote_input.h"

#include "input_state.h"
#include "sbus.h"
#include "usart.h"
#if H7_REMOTE_WFLY
#include "wfly.h"
#endif

namespace
{
#if !H7_REMOTE_WFLY
/* S.BUS 通道使用零基索引；CH5 跟随须有底盘朝向反馈后再接入。 */
constexpr unsigned TRANSLATE_X = 1U; // CH2
constexpr unsigned TRANSLATE_Y = 0U; // CH1
constexpr unsigned SPEED_GEAR = 6U;  // CH7
constexpr unsigned ROTATION = 9U;    // CH10，负半轴为手动旋转
constexpr float CHANNEL_RANGE = 784.0f;
#endif
constexpr int16_t NEUTRAL_THRESHOLD = 50;
constexpr uint32_t FRAME_FRESH_MS = 50U;
constexpr uint32_t RECOVERY_MS = 200U;

bool receiver_ready;
bool armed;
uint32_t last_unhealthy_ms;

#if H7_REMOTE_WFLY
float Deadzone(float value)
{
    return value > -0.04f && value < 0.04f ? 0.0f : value;
}
#else
float Clamp(float value, float minimum, float maximum)
{
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

float Axis(int16_t channel)
{
    // 原始通道先限幅到 [-1, 1]，再设置中心死区；物理速度量程由调用处换算。
    const float normalized = Clamp(static_cast<float>(channel) / CHANNEL_RANGE,
                                   -1.0f, 1.0f);
    return normalized > -0.04f && normalized < 0.04f ? 0.0f : normalized;
}
#endif

bool Neutral(const Struct_SBUS_Frame &frame)
{
#if H7_REMOTE_WFLY
    // WFLY 的 CH15 是双向旋转目标，必须和四个摇杆轴一起回中。
    constexpr unsigned axes[] = {0U, 1U, 2U, 3U, 14U};
#else
    /* 解锁必须先松开平移和云台两轴；CH10 在旧遥控上是偏置开关。 */
    constexpr unsigned axes[] = {0U, 1U, 2U, 3U};
#endif
    for (unsigned index : axes)
    {
        if (frame.channels[index] < -NEUTRAL_THRESHOLD ||
            frame.channels[index] > NEUTRAL_THRESHOLD)
        {
            return false;
        }
    }
    return true;
}
}

bool RemoteInput_Init(void)
{
    armed = false;
    last_unhealthy_ms = HAL_GetTick();
    InputState_Reset();
    InputState_SetTime(last_unhealthy_ms);
    // S.BUS Device 负责协议解析，并通过 BSP UART 接收；应用只读取完整帧快照。
    receiver_ready = SBUS_Init(&huart5);
    return receiver_ready;
}

void RemoteInput_Update(void)
{
    const uint32_t now = HAL_GetTick();
    InputState_SetTime(now);
#if H7_REMOTE_WFLY
    Struct_WFLY_State wfly{};
    const bool available = receiver_ready && WFLY_ReadLatest(&wfly);
    const Struct_SBUS_Frame &frame = wfly.sbus;
#else
    Struct_SBUS_Frame frame{};
    const bool available = receiver_ready && SBUS_ReadLatest(&frame);
#endif
    const bool healthy = available &&
                         now - frame.timestamp_ms <= FRAME_FRESH_MS &&
                         !frame.frame_lost && !frame.failsafe;
    if (!healthy)
    {
        last_unhealthy_ms = now;
        armed = false;
        InputState_SubmitRemote({});
        return;
    }

    if (!armed)
    {
        // 健康帧且摇杆连续回中 200 ms 才解锁；失联或未回中都会重新计时。
        if (!Neutral(frame))
        {
            last_unhealthy_ms = now;
            InputState_SubmitRemote({});
            return;
        }
        if (now - last_unhealthy_ms < RECOVERY_MS)
        {
            InputState_SubmitRemote({});
            return;
        }
        armed = true;
    }

    ChassisCmd chassis{};
#if H7_REMOTE_WFLY
    // 上左旋钮只缩放平移；下左旋钮独立控制双向旋转，坐标系固定为底盘。
    const float gear = wfly.knob_up_left * 0.5f;
    chassis.velocity_x_m_s = Deadzone(wfly.left_remote_y) * gear * INPUT_MAX_TRANSLATION_M_S;
    chassis.velocity_y_m_s = -Deadzone(wfly.left_remote_x) * gear * INPUT_MAX_TRANSLATION_M_S;
    chassis.angular_velocity_rad_s = Deadzone(wfly.knob_down_left) * INPUT_MAX_ROTATION_RAD_S;
#else
    // 速度档映射到 [0, 1]，只缩放 SI 速度目标；此处尚未接入云台和发射通道。
    const float gear = Clamp((static_cast<float>(frame.channels[SPEED_GEAR]) +
                              CHANNEL_RANGE) / (2.0f * CHANNEL_RANGE), 0.0f, 1.0f);
    chassis.velocity_x_m_s = Axis(frame.channels[TRANSLATE_X]) * gear * INPUT_MAX_TRANSLATION_M_S;
    chassis.velocity_y_m_s = -Axis(frame.channels[TRANSLATE_Y]) * gear * INPUT_MAX_TRANSLATION_M_S;
    if (frame.channels[ROTATION] < 0)
    {
        chassis.angular_velocity_rad_s = Axis(frame.channels[ROTATION]) * gear *
                                         INPUT_MAX_ROTATION_RAD_S;
    }
#endif
    if (chassis.velocity_x_m_s != 0.0f || chassis.velocity_y_m_s != 0.0f ||
        chassis.angular_velocity_rad_s != 0.0f)
    {
        chassis.mode = ChassisMode::NO_FOLLOW;
    }
    ControlInput remote{};
    remote.chassis = chassis;
    remote.received_ms = frame.timestamp_ms;
    remote.valid = true;
    InputState_SubmitRemote(remote);
}
