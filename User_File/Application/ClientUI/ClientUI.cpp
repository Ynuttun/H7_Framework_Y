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
char requested_mode[2][30];
char sent_text[7][30];
bool sent_warning[3];

bool DrawStatic(uint16_t robot_id)
{
    using namespace ClientUILayout;
    if (rebuild_slot < kStaticGroups)
    {
        const uint8_t count = rebuild_slot == kStaticGroups - 1U ? 2U : 7U;
        return RefereeUITryDraw(robot_id, kStaticFigures + rebuild_slot * 7U, count);
    }
    const uint8_t index = rebuild_slot - kStaticGroups;
    const char* labels[kTextCount] = {"F", "E", "A   B   C", "CAP N/A", "kRPM", "LOAD", "POWER W"};
    const uint16_t x[kTextCount] = {697U, 605U, 912U, 440U, 1670U, 1535U, 400U};
    const uint16_t y[kTextCount] = {775U, 550U, 275U, 700U, 730U, 614U, 665U};
    ext_client_custom_character_t text{};
    text.graphic_data_struct = Figure(150U + index, figure_type_Char, color_White, 1U, x[index], y[index]);
    text.graphic_data_struct.details_a = index < 3U ? 20U : 14U;
    const size_t length = std::strlen(labels[index]);
    text.graphic_data_struct.details_b = length;
    std::memcpy(text.data, labels[index], length);
    return RefereeUITryText(robot_id, &text);
}

bool DrawDynamic(uint16_t robot_id, const Struct_Client_UI_State& view)
{
    interaction_figure_t graphs[7]{};
    ClientUILayout::BuildDynamic(view, graphs);
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
    const uint16_t x[] = {60U, 60U, 60U, 1510U, 40U, 40U, 40U};
    const uint16_t y[] = {800U, 770U, 740U, 560U, 640U, 670U, 700U};
    text.graphic_data_struct = ClientUILayout::Figure(170U + index, figure_type_Char,
                                                      index >= 3U ? color_White : warning ? color_Yellow : color_Orange,
                                                      2U, x[index], y[index]);
    text.graphic_data_struct.details_a = 18U;
    text.graphic_data_struct.operate_type = *label == '\0' ? operate_type_Del
                                            : sent_text[index][0] == '\0' || (text_pending & (1U << index)) != 0U
                                                ? operate_type_ADD : operate_type_Change;
    if (*label != '\0' && (index == 3U || index == 6U))
        text.graphic_data_struct.operate_type = (mode_remaining[index == 3U ? 0U : 1U] & 1U) == 0U
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
    view.mode_text[29] = '\0';
    for (auto& line : view.alert_text)
        line[29] = '\0';

    if (now_us < view.power_timestamp_us || now_us - view.power_timestamp_us >= ClientUIConfig::kPowerMaxAgeUs)
        view.power_valid = false;
    if (now_us < view.fire_mode_timestamp_us || now_us - view.fire_mode_timestamp_us > ClientUIConfig::kFireModeMaxAgeUs)
        view.fire_mode = Enum_Client_UI_FireMode::Unknown;
    const uint32_t tick_ms = HAL_GetTick();
    view.referee_status_valid &= tick_ms - view.referee_status_ms < ClientUIConfig::kRefereeMaxAgeMs;
    view.referee_heat_valid &= tick_ms - view.referee_heat_ms < ClientUIConfig::kRefereeMaxAgeMs;
    view.referee_allowance_valid &= tick_ms - view.referee_allowance_ms < ClientUIConfig::kRefereeMaxAgeMs;
    const uint8_t referee_validity = (view.referee_status_valid ? 1U : 0U) |
                                     (view.referee_heat_valid ? 2U : 0U) |
                                     (view.referee_allowance_valid ? 4U : 0U);
    char labels[7][30]{};
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
    const char* fire = view.fire_mode == Enum_Client_UI_FireMode::Single ? "FIRE SINGLE"
                      : view.fire_mode == Enum_Client_UI_FireMode::Triple ? "FIRE TRIPLE"
                      : view.fire_mode == Enum_Client_UI_FireMode::Burst ? "FIRE BURST" : "";
    std::strcpy(labels[6], fire);
    // 每轮观察目标，即使新模式先变回已发送内容，也取消旧目标的剩余补发。
    for (uint8_t i = 0U; i < 2U; ++i)
    {
        const uint8_t index = i == 0U ? 3U : 6U;
        if (std::strcmp(labels[index], requested_mode[i]) != 0 || (text_pending & (1U << index)) != 0U)
        {
            std::memcpy(requested_mode[i], labels[index], sizeof(requested_mode[i]));
            mode_remaining[i] = 4U;
            text_pending &= ~(1U << index);
        }
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
            else
            {
                const uint8_t index = 3U + slot;
                const bool mode = slot == 0U || slot == 3U;
                const uint8_t mode_index = slot == 0U ? 0U : 1U;
                const bool pending = (text_pending & (1U << index)) != 0U;
                if (mode ? mode_remaining[mode_index] == 0U : !pending && std::strcmp(labels[index], sent_text[index]) == 0)
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
                        --mode_remaining[mode_index];
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
        submitted = DrawDynamic(robot_id, view);
        if (submitted && readding)
            dynamic_readd_ms = now_ms;
    }
    if (!submitted)
        return;
    last_send_ms = now_ms;
    draw_dynamic = auxiliary;
}
