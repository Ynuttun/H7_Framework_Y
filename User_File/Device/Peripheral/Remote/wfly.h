/**
 * @file wfly.h
 * @brief WFLY 通道语义适配；接收、帧校验与在线判断复用 S.BUS。
 */
#ifndef WFLY_H
#define WFLY_H

#include "sbus.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define WFLY_CHANNEL_RANGE 671
#define WFLY_SWITCH_COUNT 6U

    typedef enum
    {
        WFLY_SWITCH_LOW = 0,
        WFLY_SWITCH_MIDDLE,
        WFLY_SWITCH_HIGH,
    } Enum_WFLY_Switch_Position;

    typedef struct
    {
        Struct_SBUS_Frame sbus;                                ///< channels 为减去 1024 后的值；含接收时戳、序号及失控标志。
        float left_remote_x;                                   ///< CH4，[-1,1]。
        float left_remote_y;                                   ///< CH2 取反，[-1,1]。
        float right_remote_x;                                  ///< CH1，[-1,1]。
        float right_remote_y;                                  ///< CH3，[-1,1]。
        Enum_WFLY_Switch_Position switches[WFLY_SWITCH_COUNT]; ///< CH5/6/7/8/11/13，低/中/高。
        bool left_bool_switch;                                 ///< CH12 > 1024。
        bool right_bool_switch;                                ///< CH14 > 1024。
        float knob_up_left;                                    ///< CH9，[0,2]。
        float knob_up_right;                                   ///< CH10，[0,2]。
        float knob_down_left;                                  ///< CH15，[-1,1]。
        float knob_down_right;                                 ///< CH16，[-1,1]。
    } Struct_WFLY_State;

    /**
     * @brief 在任务上下文读取并转换最新完整帧；须先由唯一所有者调用 SBUS_Init。
     * @return 无完整帧或 state 为空时返回 false，保持输出不变。
     * @note true 不表示新帧、在线或可控制；调用方检查 sbus 的时戳和失控标志。
     *       通道分配来自参考工程，统一中点 1024、半量程 671，须按实际遥控器核对。
     */
    bool WFLY_ReadLatest(Struct_WFLY_State* state);

#ifdef __cplusplus
}
#endif

#endif
