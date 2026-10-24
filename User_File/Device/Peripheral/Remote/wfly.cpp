#include "wfly.h"

namespace
{
float Axis(int16_t channel)
{
    const float value = static_cast<float>(channel) / WFLY_CHANNEL_RANGE;
    return value < -1.0f ? -1.0f : (value > 1.0f ? 1.0f : value);
}

Enum_WFLY_Switch_Position Switch(int16_t channel)
{
    return channel > 100 ? WFLY_SWITCH_HIGH : (channel < -100 ? WFLY_SWITCH_LOW : WFLY_SWITCH_MIDDLE);
}
} // namespace

extern "C" bool WFLY_ReadLatest(Struct_WFLY_State* state)
{
    Struct_WFLY_State decoded{};
    if (state == nullptr || !SBUS_ReadLatest(&decoded.sbus))
    {
        return false;
    }

    const int16_t* channels = decoded.sbus.channels;
    decoded.left_remote_x = Axis(channels[3]);
    decoded.left_remote_y = -Axis(channels[1]);
    decoded.right_remote_x = Axis(channels[0]);
    decoded.right_remote_y = Axis(channels[2]);
    constexpr unsigned switch_channels[] = {4U, 5U, 6U, 7U, 10U, 12U};
    for (unsigned index = 0U; index < WFLY_SWITCH_COUNT; ++index)
    {
        decoded.switches[index] = Switch(channels[switch_channels[index]]);
    }
    decoded.left_bool_switch = channels[11] > 0;
    decoded.right_bool_switch = channels[13] > 0;
    decoded.knob_up_left = Axis(channels[8]) + 1.0f;
    decoded.knob_up_right = Axis(channels[9]) + 1.0f;
    decoded.knob_down_left = Axis(channels[14]);
    decoded.knob_down_right = Axis(channels[15]);
    *state = decoded;
    return true;
}
