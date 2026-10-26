#include "ClientUI.h"

#include "ClientUI_Layout.h"
#include "message_center.h"
#include "referee_ui_sender.h"
#include "referee_26.h"

#include <cstring>
#include <cstdio>

namespace
{
bool initialized;
uint16_t configured_robot_id;
uint16_t client_robot_id;
uint32_t last_send_ms;
uint32_t rebuild_ms;
uint8_t rebuild_slot;
uint8_t dynamic_visible;
uint8_t text_pending;
bool rebuilding;
bool draw_dynamic;
bool readd_dynamic;
uint32_t dynamic_readd_ms;
uint32_t value_sent_ms[2];
uint8_t auxiliary_slot;
uint8_t last_referee_validity;
uint8_t mode_remaining[2];
char requested_mode[30];
Enum_Client_UI_FireMode requested_fire_mode;
char sent_text[6][30];
bool sent_warning[3];

bool DrawStatic(uint16_t robot_id)
{
    using namespace ClientUILayout;
    if (rebuild_slot < kStaticGroups)
    {
        const uint8_t count = rebuild_slot == kStaticGroups - 1U ? 1U : 7U;
        return RefereeUITryDraw(robot_id, kStaticFigures + rebuild_slot * 7U, count);
    }
    const uint8_t index = rebuild_slot - kStaticGroups;
    const char* labels[kTextCount] = {"F", "E", "A B C", "CAP N/A", "kRPM", "LOAD", "POWER W", "1", "3"};
    const uint16_t x[kTextCount] = {697U, 605U, 910U, 300U, 1670U, 1535U, 260U, 579U, 548U};
    const uint16_t y[kTextCount] = {775U, 550U, 275U, 700U, 730U, 614U, 665U, 862U, 818U};
    ext_client_custom_character_t text{};
    text.graphic_data_struct = Figure(150U + index, figure_type_Char, color_White, 1U, x[index], y[index]);
    text.graphic_data_struct.details_a = index < 3U ? 20U : index >= 7U ? 16U : 14U;
    const size_t length = std::strlen(labels[index]);
    text.graphic_data_struct.details_b = length;
    std::memcpy(text.data, labels[index], length);
    return RefereeUITryText(robot_id, &text);
}

bool DrawDynamic(uint16_t robot_id, const Struct_Client_UI_State& view, uint32_t now_ms)
{
    interaction_figure_t graphs[7]{};
    ClientUILayout::BuildDynamic(view, graphs, (now_ms / ClientUIConfig::kTargetBlinkPhaseMs) % 2U == 0U);
    uint8_t next_visible = 0U;
    for (uint8_t i = 0U; i < 7U; ++i)
    {
        if (graphs[i].operate_type == operate_type_Del)
            continue;
        const uint8_t bit = 1U << i;
        next_visible |= bit;
        if (!readd_dynamic && (dynamic_visible & bit) != 0U)
            graphs[i].operate_type = operate_type_Change;
    }
    if (!RefereeUITryDraw(robot_id, graphs, 7U))
        return false;
    dynamic_visible = next_visible;
    readd_dynamic = false;
    return true;
}

bool DrawText(uint16_t robot_id, uint8_t index, const char* label, bool warning)
{
    ext_client_custom_character_t text{};
    const uint16_t x[] = {0U, 0U, 0U, 1510U, 40U, 40U};
    const uint16_t y[] = {820U, 790U, 760U, 560U, 640U, 670U};
    text.graphic_data_struct = ClientUILayout::Figure(170U + index, figure_type_Char,
                                                      index >= 3U ? color_White : warning ? color_Yellow : color_Orange,
                                                      2U, index < 3U ? 960U - 9U * std::strlen(label) : x[index], y[index]);
    text.graphic_data_struct.details_a = 18U;
    text.graphic_data_struct.operate_type = *label == '\0' ? operate_type_Del
                                            : sent_text[index][0] == '\0' || (text_pending & (1U << index)) != 0U
                                                ? operate_type_ADD : operate_type_Change;
    if (*label != '\0' && index == 3U)
        text.graphic_data_struct.operate_type = (mode_remaining[0] & 1U) == 0U
                                                   ? operate_type_ADD : operate_type_Change;
    const size_t length = std::strlen(label);
    text.graphic_data_struct.details_b = length;
    std::memcpy(text.data, label, length);
    return RefereeUITryText(robot_id, &text);
}

bool Expired(uint64_t timestamp_us, uint64_t now_us)
{
    return now_us < timestamp_us || now_us - timestamp_us > ClientUIConfig::kFeedbackMaxAgeUs;
}
} // namespace

bool ClientUI_Init(uint16_t robot_id)
{
    // 仅由 StatusTask 初始化和消费私有刷新状态，不与 Capture 共享可变缓存。
    initialized = RefereeUIIsReady() && (robot_id == 0U || RefereeUIRobotIdIsValid(robot_id));
    configured_robot_id = robot_id;
    client_robot_id = 0U;
    last_send_ms = static_cast<uint32_t>(SYS_Timestamp_Get_Microsecond() / 1000U);
    return initialized;
}

void ClientUI_Update(void)
{
    if (!initialized)
        return;
    uint16_t robot_id = configured_robot_id;
    if (robot_id == 0U && !RefereeReadUIRobotId(&robot_id))
    {
        // 身份恢复后即使 ID 相同，也重新建立本 UI 图层。
        client_robot_id = 0U;
        return;
    }
    const uint64_t now_us = SYS_Timestamp_Get_Microsecond();
    const uint32_t now_ms = static_cast<uint32_t>(now_us / 1000U);
    if (now_ms - last_send_ms < ClientUIConfig::kSendIntervalMs)
        return;
    if (robot_id != client_robot_id)
    {
        if (!RefereeUITryDeleteLayer(robot_id, 9U))
            return;
        client_robot_id = robot_id;
        rebuild_slot = dynamic_visible = text_pending = 0U;
        rebuilding = draw_dynamic = readd_dynamic = true;
        std::memset(sent_text, 0, sizeof(sent_text));
        std::memset(sent_warning, 0, sizeof(sent_warning));
        std::memset(requested_mode, 0, sizeof(requested_mode));
        requested_fire_mode = Enum_Client_UI_FireMode::Unknown;
        std::memset(mode_remaining, 0, sizeof(mode_remaining));
        std::memset(value_sent_ms, 0, sizeof(value_sent_ms));
        auxiliary_slot = last_referee_validity = 0U;
        rebuild_ms = dynamic_readd_ms = last_send_ms = now_ms;
        return;
    }
    if (!rebuilding && now_ms - rebuild_ms >= ClientUIConfig::kRebuildIntervalMs)
    {
        rebuilding = true;
        rebuild_slot = 0U;
        readd_dynamic = true;
        text_pending = 0x7FU;
        rebuild_ms = now_ms;
    }
    if (now_ms - dynamic_readd_ms >= ClientUIConfig::kDynamicReaddIntervalMs)
        readd_dynamic = true;
    Struct_Client_UI_State view{};
    (void) MessageCenter::Client_UI_State_Topic.ReadFresh(view, ClientUIConfig::kSnapshotMaxAgeUs);
    // 保留源时间：即使采集刚停止，也不额外延长 100 ms 的反馈有效期。
    if (Expired(view.pitch_timestamp_us, now_us))
        view.pitch_valid = false;
    if (Expired(view.shoot_timestamp_us, now_us))
    {
        view.speed_valid = false;
        view.friction = view.loader = Enum_Client_UI_Motion::Unknown;
    }
    if (Expired(view.chassis_timestamp_us, now_us))
    {
        view.chassis = Enum_Client_UI_Chassis::Unknown;
        std::strcpy(view.mode_text, "UNKNOWN");
    }
#if GIMBAL
    view.initializing |= !view.pitch_valid;
#endif
#if CHASSIS
    view.initializing |= view.chassis == Enum_Client_UI_Chassis::Unknown || view.chassis == Enum_Client_UI_Chassis::Waiting;
#endif
#if SHOOT
    view.initializing |= Expired(view.shoot_timestamp_us, now_us);
#endif
    view.mode_text[29] = '\0';
    for (auto& line : view.alert_text)
        line[29] = '\0';

    if (now_us < view.power_timestamp_us || now_us - view.power_timestamp_us >= ClientUIConfig::kPowerMaxAgeUs)
        view.power_valid = false;
    if (now_us < view.fire_mode_timestamp_us || now_us - view.fire_mode_timestamp_us > ClientUIConfig::kFireModeMaxAgeUs)
        view.fire_mode = Enum_Client_UI_FireMode::Unknown;
    const uint32_t tick_ms = HAL_GetTick();
    view.target_detected &= tick_ms - view.target_received_ms <= ClientUIConfig::kTargetMaxAgeMs;
    if (Expired(view.autoaim_timestamp_us, now_us))
        view.autoaim = Enum_Client_UI_AutoAim::Unknown;
    else if (view.autoaim == Enum_Client_UI_AutoAim::Active && !view.target_detected)
        view.autoaim = Enum_Client_UI_AutoAim::Waiting;
    view.referee_status_valid &= tick_ms - view.referee_status_ms < ClientUIConfig::kRefereeMaxAgeMs;
    view.referee_heat_valid &= tick_ms - view.referee_heat_ms < ClientUIConfig::kRefereeMaxAgeMs;
    view.referee_allowance_valid &= tick_ms - view.referee_allowance_ms < ClientUIConfig::kRefereeMaxAgeMs;
    const uint8_t referee_validity = (view.referee_status_valid ? 1U : 0U) |
                                     (view.referee_heat_valid ? 2U : 0U) |
                                     (view.referee_allowance_valid ? 4U : 0U);
    char labels[6][30]{};
    for (uint8_t i = 0U; i < 3U; ++i)
        std::memcpy(labels[i], view.alert_text[i], sizeof(labels[i]));
    std::memcpy(labels[3], view.mode_text, sizeof(labels[3]));
    if (view.referee_heat_valid)
        std::snprintf(labels[4], sizeof(labels[4]), "BUFFER %u J", unsigned(view.buffer_energy_j));
    if (view.referee_heat_valid && view.referee_status_valid)
        std::snprintf(labels[5], sizeof(labels[5]), "HEAT %u/%u", unsigned(view.barrel_heat), unsigned(view.barrel_heat_limit));
    else if (view.referee_heat_valid)
        std::snprintf(labels[5], sizeof(labels[5]), "HEAT %u", unsigned(view.barrel_heat));
    else if (view.referee_status_valid)
        std::snprintf(labels[5], sizeof(labels[5]), "HEAT -/%u", unsigned(view.barrel_heat_limit));
    if (view.referee_allowance_valid)
    {
        const size_t length = std::strlen(labels[5]);
        std::snprintf(labels[5] + length, sizeof(labels[5]) - length, "%sAMMO %u", length ? " " : "", unsigned(view.projectile_allowance));
    }
    // 每轮观察最新目标；中途变化取消旧目标的剩余补发。
    if (std::strcmp(labels[3], requested_mode) != 0 || (text_pending & (1U << 3U)) != 0U)
    {
        std::memcpy(requested_mode, labels[3], sizeof(requested_mode));
        mode_remaining[0] = 4U;
        text_pending &= ~(1U << 3U);
    }
    if (view.fire_mode != requested_fire_mode || (text_pending & (1U << 6U)) != 0U)
    {
        requested_fire_mode = view.fire_mode;
        mode_remaining[1] = 4U;
        text_pending &= ~(1U << 6U);
    }
    bool submitted = false;
    bool auxiliary = false;
    if (!draw_dynamic)
    {
        for (uint8_t i = 0U; i < 3U; ++i)
        {
            if ((text_pending & (1U << i)) == 0U && std::strcmp(labels[i], sent_text[i]) == 0 &&
                view.alert_warning[i] == sent_warning[i])
                continue;
            auxiliary = true;
            submitted = DrawText(robot_id, i, labels[i], view.alert_warning[i]);
            if (submitted)
            {
                std::memcpy(sent_text[i], labels[i], sizeof(sent_text[i]));
                sent_warning[i] = view.alert_warning[i];
                text_pending &= ~(1U << i);
            }
            break;
        }
        // 有界轮转；报警以外的持续数值变化不能饿死模式或静态补绘。
        for (uint8_t offset = 0U; !auxiliary && offset < 5U; ++offset)
        {
            const uint8_t slot = (auxiliary_slot + offset) % 5U;
            if (slot == 4U)
            {
                if (!rebuilding)
                    continue;
                auxiliary = true;
                submitted = DrawStatic(robot_id);
                if (submitted && ++rebuild_slot == ClientUILayout::kStaticGroups + ClientUILayout::kTextCount)
                    rebuilding = false;
            }
            else if (slot == 3U)
            {
                if (mode_remaining[1] == 0U)
                    continue;
                auto graph = ClientUILayout::FirePointer(view.fire_mode);
                if (graph.operate_type != operate_type_Del)
                    graph.operate_type = (mode_remaining[1] & 1U) == 0U ? operate_type_ADD : operate_type_Change;
                auxiliary = true;
                submitted = RefereeUITryDraw(robot_id, &graph, 1U);
                if (submitted)
                    --mode_remaining[1];
            }
            else
            {
                const uint8_t index = 3U + slot;
                const bool mode = slot == 0U;
                const bool pending = (text_pending & (1U << index)) != 0U;
                if (mode ? mode_remaining[0] == 0U : !pending && std::strcmp(labels[index], sent_text[index]) == 0)
                    continue;
                // 部分热量字段失效/恢复也绕过限频，不显示过期字段。
                const bool validity_changed = slot == 2U && referee_validity != last_referee_validity;
                if (!mode && !pending && !validity_changed && labels[index][0] != '\0' && sent_text[index][0] != '\0' &&
                    now_ms - value_sent_ms[slot - 1U] < ClientUIConfig::kValueIntervalMs)
                    continue;
                auxiliary = true;
                submitted = DrawText(robot_id, index, labels[index], false);
                if (submitted)
                {
                    std::memcpy(sent_text[index], labels[index], sizeof(sent_text[index]));
                    text_pending &= ~(1U << index);
                    if (mode)
                        --mode_remaining[0];
                    else
                        value_sent_ms[slot - 1U] = now_ms;
                    if (slot == 2U)
                        last_referee_validity = referee_validity;
                }
            }
            if (submitted)
                auxiliary_slot = (slot + 1U) % 5U;
        }
    }
    if (!auxiliary)
    {
        const bool readding = readd_dynamic;
        submitted = DrawDynamic(robot_id, view, now_ms);
        if (submitted && readding)
            dynamic_readd_ms = now_ms;
    }
    if (!submitted)
        return;
    last_send_ms = now_ms;
    draw_dynamic = auxiliary;
}
