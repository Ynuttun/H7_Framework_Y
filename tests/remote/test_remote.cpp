#include "RobotCmd.h"
#include "bsp_uart.h"
#include "input_state.h"
#include "message_center.h"
#include "remote_input.h"
#include "source_arbitration.h"
#include "wfly.h"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

uint32_t test_primask;
UART_HandleTypeDef huart5{};
static uint32_t now_ms;
static unsigned uart_init_count;
static UART_Callback rx_callback;
static int dma;

extern "C" uint32_t HAL_GetTick(void)
{
    return now_ms;
}

extern "C" uint64_t SYS_Timestamp_Get_Microsecond(void)
{
    return static_cast<uint64_t>(now_ms) * 1000U;
}

void UART_Init(UART_HandleTypeDef* uart, UART_Callback callback)
{
    assert(uart == &huart5);
    ++uart_init_count;
    rx_callback = callback;
}

namespace
{
using Channels = std::array<uint16_t, SBUS_CHANNEL_COUNT>;
using Frame = std::array<uint8_t, SBUS_FRAME_SIZE>;

Channels Centered()
{
    Channels channels;
    channels.fill(1024U);
    return channels;
}

void ConfigureUart()
{
    huart5 = {{100000U, UART_WORDLENGTH_9B, UART_STOPBITS_2, UART_PARITY_EVEN, UART_MODE_RX}, &dma};
}

Frame Encode(const Channels& channels, uint8_t flags = 0U)
{
    Frame bytes{};
    bytes[0] = 0x0FU;
    for (unsigned channel = 0U; channel < channels.size(); ++channel)
    {
        for (unsigned bit = 0U; bit < 11U; ++bit)
        {
            const unsigned offset = channel * 11U + bit;
            bytes[1U + offset / 8U] |= ((channels[channel] >> bit) & 1U) << (offset % 8U);
        }
    }
    bytes[23] = flags;
    return bytes;
}

void Receive(const Channels& channels, uint8_t flags = 0U)
{
    Frame bytes = Encode(channels, flags);
    assert(rx_callback != nullptr);
    rx_callback(bytes.data(), bytes.size());
}

Struct_SBUS_Frame Latest()
{
    Struct_SBUS_Frame frame{};
    assert(SBUS_ReadLatest(&frame));
    return frame;
}

Struct_WFLY_State Wfly()
{
    Struct_WFLY_State state{};
    const unsigned init_count = uart_init_count;
    assert(WFLY_ReadLatest(&state));
    assert(init_count == uart_init_count);
    return state;
}

void Near(float actual, float expected)
{
    assert(std::fabs(actual - expected) < 0.00001f);
}

void TestInit()
{
    Struct_WFLY_State state{};
    state.left_remote_x = 123.0f;
    assert(!WFLY_ReadLatest(&state));
    Near(state.left_remote_x, 123.0f);
    assert(!WFLY_ReadLatest(nullptr));
    assert(!SBUS_ReadLatest(nullptr));
    assert(!SBUS_Init(nullptr));
    ConfigureUart();
    const UART_HandleTypeDef valid = huart5;
    huart5.hdmarx = nullptr;
    assert(!SBUS_Init(&huart5));
    for (uint32_t* field : {&huart5.Init.BaudRate, &huart5.Init.WordLength,
                            &huart5.Init.StopBits, &huart5.Init.Parity, &huart5.Init.Mode})
    {
        huart5 = valid;
        *field = 0U;
        assert(!RemoteInput_Init());
        RemoteInput_Update();
        assert(!InputState_Read().remote.valid);
        assert(!SourceArbitration_Resolve(InputState_Read()).armed);
    }
    assert(uart_init_count == 0U);
    huart5 = valid;
    assert(SBUS_Init(&huart5));
    assert(SBUS_IsEnabled());
    assert(!SBUS_IsOnline());
    assert(!SBUS_IsHealthy());
    assert(!WFLY_ReadLatest(&state));
    // Nested critical sections must restore the caller's interrupt mask.
    test_primask = 1U;
    Receive(Centered());
    assert(SBUS_IsHealthy());
    Wfly();
    SBUS_GetDiagnostics(nullptr);
    assert(test_primask == 1U);
    test_primask = 0U;
}

void TestProtocol()
{
    ConfigureUart();
    assert(SBUS_Init(&huart5));
    Channels channels = Centered();
    for (unsigned index = 0U; index < channels.size(); ++index)
    {
        for (unsigned bit = 0U; bit < 11U; ++bit)
        {
            channels = Centered();
            channels[index] = 1U << bit;
            Receive(channels);
            const auto frame = Latest();
            for (unsigned other = 0U; other < channels.size(); ++other)
            {
                assert(frame.channels[other] == static_cast<int>(channels[other]) - 1024);
            }
        }
    }
    for (unsigned flags = 0U; flags < 16U; ++flags)
    {
        Receive(channels, flags);
        const auto frame = Latest();
        assert(frame.channel_17 == ((flags & 1U) != 0U));
        assert(frame.channel_18 == ((flags & 2U) != 0U));
        assert(frame.frame_lost == ((flags & 4U) != 0U));
        assert(frame.failsafe == ((flags & 8U) != 0U));
        assert(SBUS_IsOnline());
        assert(SBUS_IsHealthy() == ((flags & 12U) == 0U));
    }
    for (unsigned split = 1U; split < SBUS_FRAME_SIZE; ++split)
    {
        assert(SBUS_Init(&huart5));
        auto bytes = Encode(channels);
        rx_callback(bytes.data(), split);
        assert(!SBUS_IsOnline());
        Struct_SBUS_Frame partial{};
        assert(!SBUS_ReadLatest(&partial));
        rx_callback(bytes.data() + split, bytes.size() - split);
        assert(Latest().sequence == 1U);
    }
    assert(SBUS_Init(&huart5));
    auto bytes = Encode(channels);
    for (uint8_t& byte : bytes)
    {
        rx_callback(&byte, 1U);
    }
    assert(Latest().sequence == 1U);
    std::vector<uint8_t> merged(bytes.begin(), bytes.end());
    channels[0] = 353U;
    bytes = Encode(channels);
    merged.insert(merged.end(), bytes.begin(), bytes.end());
    rx_callback(merged.data(), merged.size());
    assert(Latest().sequence == 3U);
    assert(Latest().channels[0] == -671);

    // A false header followed by a real frame must recover at the embedded header.
    assert(SBUS_Init(&huart5));
    channels = Centered();
    bytes = Encode(channels);
    std::vector<uint8_t> noisy{0x55U, 0x0FU, 0xAAU, 0xBBU};
    noisy.insert(noisy.end(), bytes.begin(), bytes.end());
    rx_callback(noisy.data(), noisy.size());
    assert(Latest().sequence == 1U);
    assert(Latest().channels[0] == 0);
    const uint32_t timestamp = Latest().timestamp_ms;
    now_ms += 99U;
    auto bad = bytes;
    bad.back() = 0x55U;
    rx_callback(bad.data(), bad.size());
    assert(Latest().sequence == 1U);
    assert(Latest().timestamp_ms == timestamp);
    assert(SBUS_IsOnline());
    ++now_ms;
    rx_callback(nullptr, 25U);
    rx_callback(bytes.data(), 0U);
    rx_callback(bytes.data(), bytes.size() - 1U);
    assert(!SBUS_IsOnline());
    assert(Latest().sequence == 1U);
    rx_callback(&bytes.back(), 1U);
    assert(SBUS_IsOnline());
    assert(Latest().sequence == 2U);
    Struct_SBUS_Diagnostics stats{};
    SBUS_GetDiagnostics(&stats);
    assert(stats.valid_frames == 2U);
    assert(stats.invalid_candidates >= 2U);
    assert(stats.resync_events >= 2U);
}

void TestMapping()
{
    ConfigureUart();
    assert(SBUS_Init(&huart5));
    Channels channels = Centered();
    Receive(channels);
    auto state = Wfly();
    Near(state.left_remote_x, 0.0f);
    Near(state.left_remote_y, 0.0f);
    Near(state.right_remote_x, 0.0f);
    Near(state.right_remote_y, 0.0f);
    Near(state.knob_up_left, 1.0f);
    Near(state.knob_up_right, 1.0f);
    Near(state.knob_down_left, 0.0f);
    Near(state.knob_down_right, 0.0f);
    const unsigned axes[] = {3U, 1U, 0U, 2U, 8U, 9U, 14U, 15U};
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        for (uint16_t raw : {0U, 353U, 1024U, 1031U, 1695U, 2047U})
        {
            channels = Centered();
            channels[axes[axis]] = raw;
            Receive(channels, 0x0FU);
            state = Wfly();
            const float values[] = {state.left_remote_x, state.left_remote_y,
                                    state.right_remote_x, state.right_remote_y,
                                    state.knob_up_left, state.knob_up_right,
                                    state.knob_down_left, state.knob_down_right};
            float expected = raw <= 353U ? -1.0f : (raw >= 1695U ? 1.0f : (static_cast<float>(raw) - 1024.0f) / 671.0f);
            if (axis == 1U)
            {
                expected = -expected;
            }
            if (axis == 4U || axis == 5U)
            {
                expected += 1.0f;
            }
            Near(values[axis], expected);
            for (unsigned other = 0U; other < 8U; ++other)
            {
                if (other != axis)
                {
                    Near(values[other], other == 4U || other == 5U ? 1.0f : 0.0f);
                }
            }
            assert(state.sbus.sequence == Latest().sequence);
            assert(state.sbus.timestamp_ms == now_ms);
            assert(state.sbus.frame_lost && state.sbus.failsafe);
            assert(state.sbus.channel_17 && state.sbus.channel_18);
        }
    }
    constexpr unsigned switches[] = {4U, 5U, 6U, 7U, 10U, 12U};
    for (unsigned index = 0U; index < WFLY_SWITCH_COUNT; ++index)
    {
        for (uint16_t raw : {923U, 924U, 1024U, 1124U, 1125U})
        {
            channels = Centered();
            channels[switches[index]] = raw;
            Receive(channels);
            state = Wfly();
            for (unsigned other = 0U; other < WFLY_SWITCH_COUNT; ++other)
            {
                assert(state.switches[other] == (other != index ? WFLY_SWITCH_MIDDLE : (raw < 924U ? WFLY_SWITCH_LOW : (raw > 1124U ? WFLY_SWITCH_HIGH : WFLY_SWITCH_MIDDLE))));
            }
        }
    }
    for (unsigned index : {11U, 13U})
    {
        for (uint16_t raw : {1023U, 1024U, 1025U})
        {
            channels = Centered();
            channels[index] = raw;
            Receive(channels);
            state = Wfly();
            assert(state.left_bool_switch == (index == 11U && raw > 1024U));
            assert(state.right_bool_switch == (index == 13U && raw > 1024U));
        }
    }
}

LocalPublisher<GimbalCmd> gimbal_output(MessageCenter::Gimbal_Command_Topic);
LocalPublisher<ChassisCmd> chassis_output(MessageCenter::Chassis_Command_Topic);
LocalPublisher<ShootCmd> shoot_output(MessageCenter::Shoot_Command_Topic);

void StartInput()
{
    ConfigureUart();
    assert(RobotCmd_Init(gimbal_output.Bind(), chassis_output.Bind(), shoot_output.Bind(), true));
    assert(RemoteInput_Init());
}

ControlInput Update(const Channels& channels, uint8_t flags = 0U)
{
    Receive(channels, flags);
    RemoteInput_Update();
    RobotCmd_Update();
    return InputState_Read().remote;
}

void AssertSafeOutputs()
{
    GimbalCmd gimbal{};
    ChassisCmd chassis{};
    ShootCmd shoot{};
    assert(MessageCenter::Gimbal_Command_Topic.Read(gimbal));
    assert(MessageCenter::Chassis_Command_Topic.Read(chassis));
    assert(MessageCenter::Shoot_Command_Topic.Read(shoot));
    assert(gimbal.mode == GimbalMode::DISABLED);
    assert(chassis.mode == ChassisMode::ZERO_FORCE);
    Near(chassis.velocity_x_m_s, 0.0f);
    Near(chassis.velocity_y_m_s, 0.0f);
    Near(chassis.angular_velocity_rad_s, 0.0f);
    assert(shoot.shoot_mode == ShootMode::OFF);
    assert(shoot.friction_mode == FrictionMode::OFF);
    assert(shoot.loader_mode == LoaderMode::STOP);
    assert(MessageCenter::Shoot_Event_Queue.Empty());
}

void Arm(Channels channels)
{
    // Exercise the same 1 ms input/control schedule used by the firmware.
    const uint32_t start = now_ms;
    RemoteInput_Update();
    RobotCmd_Update();
    assert(!Update(channels).valid);
    for (unsigned elapsed = 1U; elapsed < 200U; ++elapsed)
    {
        now_ms = start + elapsed;
        assert(!Update(channels).valid);
    }
    now_ms = start + 200U;
    assert(Update(channels).valid);
}

void TestInput()
{
    StartInput();
    Channels channels = Centered();
    Arm(channels);
    auto input = InputState_Read().remote;
    assert(input.chassis.mode == ChassisMode::ZERO_FORCE);
    assert(input.gimbal.mode == GimbalMode::DISABLED);
    assert(input.shoot.shoot_mode == ShootMode::OFF);
    assert(SourceArbitration_Resolve(InputState_Read()).armed);
#if H7_REMOTE_WFLY
    channels[8] = 1695U;
    channels[1] = 353U;
    channels[3] = 1695U;
    channels[14] = 1695U;
#else
    channels[6] = 1808U;
    channels[1] = 1808U;
    channels[0] = 1808U;
    channels[9] = 240U;
#endif
    input = Update(channels);
    Near(input.chassis.velocity_x_m_s, 0.5f);
    Near(input.chassis.velocity_y_m_s, -0.5f);
#if H7_REMOTE_WFLY
    Near(input.chassis.angular_velocity_rad_s, 1.0f);
    channels[14] = 353U;
#else
    Near(input.chassis.angular_velocity_rad_s, -1.0f);
    channels[9] = 1808U;
#endif
    input = Update(channels);
    assert(input.chassis.mode == ChassisMode::NO_FOLLOW);
#if H7_REMOTE_WFLY
    Near(input.chassis.angular_velocity_rad_s, -1.0f);
    channels[8] = 1024U;
    input = Update(channels);
    Near(input.chassis.velocity_x_m_s, 0.25f);
    Near(input.chassis.velocity_y_m_s, -0.25f);
    Near(input.chassis.angular_velocity_rad_s, -1.0f);
    channels[8] = 353U;
    input = Update(channels);
    Near(input.chassis.velocity_x_m_s, 0.0f);
    Near(input.chassis.angular_velocity_rad_s, -1.0f);
#else
    Near(input.chassis.angular_velocity_rad_s, 0.0f);
#endif
    channels = Centered();
    channels[1] += 26U;
    input = Update(channels);
    Near(input.chassis.velocity_x_m_s, 0.0f);
    channels[1] += 6U;
    assert(Update(channels).chassis.velocity_x_m_s != 0.0f);
    channels = Centered();
#if H7_REMOTE_WFLY
    channels[8] = 2047U;
    channels[3] = 0U;
    channels[14] = 2047U;
    input = Update(channels);
    Near(input.chassis.velocity_y_m_s, 0.5f);
    Near(input.chassis.angular_velocity_rad_s, 1.0f);
#endif
    assert(RobotCmd_PushShootEvent({}));
    // Old data retains its real receive time; a periodic read cannot extend its life.
    const uint32_t received_ms = input.received_ms;
    now_ms += 50U;
    RemoteInput_Update();
    assert(InputState_Read().remote.valid);
    assert(InputState_Read().remote.received_ms == received_ms);
    ++now_ms;
    RemoteInput_Update();
    RobotCmd_Update();
    assert(!InputState_Read().remote.valid);
    assert(!SourceArbitration_Resolve(InputState_Read()).armed);
    AssertSafeOutputs();
    assert(!RobotCmd_PushShootEvent({}));
}

void TestRecovery()
{
    StartInput();
    Channels channels = Centered();
    Arm(channels);
    for (uint8_t flags : {4U, 8U})
    {
        assert(RobotCmd_PushShootEvent({}));
        assert(!Update(channels, flags).valid);
        assert(SBUS_IsOnline() && !SBUS_IsHealthy());
        AssertSafeOutputs();
        ++now_ms;
        Arm(channels);
    }
    Update(channels, 8U);
    ++now_ms;
    const uint32_t start = now_ms;
#if H7_REMOTE_WFLY
    channels[14] += 51U;
#else
    channels[0] += 51U;
#endif
    for (unsigned elapsed = 0U; elapsed <= 300U; ++elapsed)
    {
        now_ms = start + elapsed;
        assert(!Update(channels).valid);
    }
    channels = Centered();
    // The inclusive neutral boundary is allowed, but it still needs 200 ms.
#if H7_REMOTE_WFLY
    channels[14] += 50U;
#else
    channels[0] -= 50U;
#endif
    for (unsigned elapsed = 1U; elapsed < 200U; ++elapsed)
    {
        now_ms = start + 300U + elapsed;
        assert(!Update(channels).valid);
    }
    now_ms = start + 500U;
    assert(Update(channels).valid);
    // Default CH10 remains a biased switch and is excluded from neutral checks.
#if !H7_REMOTE_WFLY
    assert(RemoteInput_Init());
    channels = Centered();
    channels[9] = 240U;
    Arm(channels);
#endif
    // Exercise uint32_t receive-time rollover while callbacks and updates continue.
    now_ms = UINT32_MAX - 100U;
    assert(RemoteInput_Init());
    channels = Centered();
    Arm(channels);
    assert(now_ms == 99U);
    assert(SourceArbitration_Resolve(InputState_Read()).armed);
}
} // namespace

int main(int argc, char** argv)
{
    assert(argc == 2);
    if (std::strcmp(argv[1], "init") == 0)
        TestInit();
    else if (std::strcmp(argv[1], "protocol") == 0)
        TestProtocol();
    else if (std::strcmp(argv[1], "mapping") == 0)
        TestMapping();
    else if (std::strcmp(argv[1], "input") == 0)
        TestInput();
    else if (std::strcmp(argv[1], "recovery") == 0)
        TestRecovery();
    else
        assert(false);
    assert(test_primask == 0U);
    std::printf("PASS %s (%s)\n", argv[1], H7_REMOTE_WFLY ? "WFLY" : "default");
}
