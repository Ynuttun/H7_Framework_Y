#include "ClientUI.h"
#include "ClientUI_Layout.h"
#include "bsp_uart.h"
#include "message_center.h"
#include "referee_26.h"
#include "referee_daemon.h"
#include "referee_ui_sender.h"
extern "C"
{
#include "crc_ref.h"
}

#include <array>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

uint32_t test_primask;
static uint64_t now_us;
static uint8_t tx_status = HAL_OK;
static bool register_ok = true;
static unsigned init_count;
static unsigned feeds;
static UART_HandleTypeDef uart{&init_count, &init_count};
static void (*rx_callback)(uint8_t*, uint16_t);
struct Packet
{
    uint32_t ms;
    uint8_t status;
    std::vector<uint8_t> bytes;
};
static std::vector<Packet> packets;
extern "C" uint64_t SYS_Timestamp_Get_Microsecond(void)
{
    return now_us;
}
extern "C" uint32_t HAL_GetTick(void)
{
    return static_cast<uint32_t>(now_us / 1000U);
}
extern "C" void UART_Init(UART_HandleTypeDef* handle, void (*callback)(uint8_t*, uint16_t))
{
    assert(handle == &uart);
    rx_callback = callback;
    ++init_count;
}
extern "C" uint8_t UART_Transmit_Data(UART_HandleTypeDef* handle, uint8_t* data, uint16_t size)
{
    assert(handle == &uart && handle->hdmatx != nullptr);
    packets.push_back({HAL_GetTick(), tx_status, {data, data + size}});
    return tx_status;
}
extern "C" void osDelay(uint32_t)
{
    assert(false && "UI must never block");
}
extern "C" bool RefereeDaemonRegister(void)
{
    return register_ok;
}
extern "C" void RefereeDaemonFeed(void)
{
    ++feeds;
}
extern "C" bool RefereeDaemonIsOnline(void)
{
    return feeds != 0U;
}

static uint16_t U16(const std::vector<uint8_t>& data, size_t offset)
{
    return data[offset] | (uint16_t(data[offset + 1U]) << 8U);
}
// Independent bitwise CRC oracle, rather than comparing the encoder with itself.
static uint16_t CRC(const uint8_t* data, size_t length, uint16_t initial, uint16_t polynomial)
{
    uint16_t crc = initial;
    for (size_t i = 0; i < length; ++i)
    {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8U; ++bit)
            crc = (crc >> 1U) ^ ((crc & 1U) ? polynomial : 0U);
    }
    return crc;
}
static void CheckPacket(const Packet& packet, uint16_t robot, uint16_t content, size_t length)
{
    const auto& data = packet.bytes;
    assert(data.size() == length + 15U && data[0] == 0xA5U);
    assert(U16(data, 1U) == length + 6U && U16(data, 5U) == 0x0301U);
    assert(U16(data, 7U) == content && U16(data, 9U) == robot && U16(data, 11U) == robot + 0x100U);
    assert(data[4] == CRC(data.data(), 4U, 0xFFU, 0x8CU));
    assert(U16(data, data.size() - 2U) == CRC(data.data(), data.size() - 2U, 0xFFFFU, 0x8408U));
}
static std::vector<uint8_t> Frame(uint16_t command, uint16_t length, uint8_t id)
{
    std::vector<uint8_t> frame(length + 9U);
    frame[0] = 0xA5U;
    frame[1] = length;
    frame[2] = length >> 8U;
    frame[5] = command;
    frame[6] = command >> 8U;
    frame[7] = id;
    Append_CRC8_Check_Sum(frame.data(), 5U);
    Append_CRC16_Check_Sum(frame.data(), frame.size());
    return frame;
}
static void ReceiveId(uint8_t id)
{
    auto frame = Frame(0x0201U, 13U, id);
    rx_callback(frame.data(), frame.size());
}
static void Start(uint16_t id = 3U)
{
    assert(RefereeInit(&uart) != nullptr);
    assert(ClientUI_Init(id));
}
static bool Dynamic(const Packet& p)
{
    return U16(p.bytes, 7U) == 0x0104U && p.bytes[15] == 100U;
}
static interaction_figure_t FigureAt(const Packet& p, unsigned index)
{
    interaction_figure_t figure;
    std::memcpy(&figure, p.bytes.data() + 13U + 15U * index, sizeof(figure));
    return figure;
}
static Struct_Client_UI_State LiveState()
{
    Struct_Client_UI_State state{};
    state.pitch_valid = state.speed_valid = true;
    state.pitch_timestamp_us = state.shoot_timestamp_us = state.chassis_timestamp_us = now_us;
    state.pitch_rad = 0.2f;
    state.left_speed_rad_s = 100.0f;
    state.right_speed_rad_s = 200.0f;
    state.friction = Enum_Client_UI_Motion::Moving;
    state.loader = Enum_Client_UI_Motion::Stopped;
    state.chassis = Enum_Client_UI_Chassis::Spin;
    std::strcpy(state.mode_text, "SPIN");
    return state;
}
static void Tick(uint64_t milliseconds, bool live = true)
{
    now_us = milliseconds * 1000U;
    if (live)
        MessageCenter::Client_UI_State_Topic.Publish(LiveState());
    const size_t before = packets.size();
    ClientUI_Update();
    assert(packets.size() - before <= 1U);
}

static void Protocol()
{
    assert(!RefereeUITryDeleteLayer(3U, 9U));
    assert(packets.empty());
    Start();
    interaction_figure_t graphs[7]{};
    graphs[0] = ClientUILayout::Line(1U, color_White, 2U, 10U, 20U, 30U, 40U);
    const uint8_t counts[] = {1U, 2U, 5U, 7U};
    for (unsigned i = 0; i < 4U; ++i)
    {
        assert(RefereeUITryDraw(103U, graphs, counts[i]));
        CheckPacket(packets.back(), 103U, 0x0101U + i, counts[i] * 15U);
        assert(packets.back().bytes[3] == i);
    }
    // Little-endian packed graphic bit layout: name + word0 + word1 + word2.
    const auto& bytes = packets.front().bytes;
    assert(bytes[13] == 'H' && bytes[14] == 'U' && bytes[15] == 1U);
    const uint32_t expected[3] = {1U | (9U << 6U) | (8U << 10U), 2U | (10U << 10U) | (20U << 21U), (30U << 10U) | (40U << 21U)};
    for (unsigned word = 0; word < 3U; ++word)
        for (unsigned byte = 0; byte < 4U; ++byte)
            assert(bytes[16U + 4U * word + byte] == ((expected[word] >> (8U * byte)) & 0xFFU));
    for (uint8_t status : {HAL_BUSY, HAL_ERROR})
    {
        tx_status = status;
        assert(!RefereeUITryDeleteLayer(3U, 9U));
        const auto failed = packets.back().bytes;
        tx_status = HAL_OK;
        assert(RefereeUITryDeleteLayer(3U, 9U));
        assert(failed == packets.back().bytes);
    }
    ext_client_custom_character_t text{};
    assert(RefereeUITryText(105U, &text));
    CheckPacket(packets.back(), 105U, 0x0110U, 45U);
    size_t before = packets.size();
    assert(!RefereeUITryDraw(3U, graphs, 3U));
    assert(!RefereeUITryDraw(3U, nullptr, 1U));
    assert(!RefereeUITryText(3U, nullptr));
    assert(!RefereeUITryDeleteLayer(3U, 10U));
    assert(!RefereeUITryDeleteLayer(0U, 9U));
    assert(!RefereeUITryDeleteLayer(7U, 9U));
    assert(!RefereeTrySend(nullptr, 1U));
    uart.hdmatx = nullptr;
    assert(!RefereeUITryDeleteLayer(3U, 9U) && !ClientUI_Init(3U));
    assert(packets.size() == before);
    uart.hdmatx = &init_count;
    assert(!ClientUI_Init(101U));
    register_ok = false;
    assert(RefereeInit(&uart) == nullptr && !ClientUI_Init());
    // Failed initialization stays harmless.
    Tick(1000U);
    assert(packets.size() == before);
}

static void Identity()
{
    Start(0U);
    uint16_t id = 999U;
    assert(!RefereeReadUIRobotId(&id) && id == 999U);
    auto bad = Frame(0x0201U, 13U, 3U);
    bad.back() ^= 1U;
    RefereeReceiveData(bad.data(), bad.size());
    auto short_frame = Frame(0x0201U, 12U, 3U);
    RefereeReceiveData(short_frame.data(), short_frame.size());
    assert(!RefereeReadUIRobotId(&id));
    auto good = Frame(0x0201U, 13U, 104U);
    RefereeReceiveData(good.data(), 4U);
    assert(!RefereeReadUIRobotId(&id));
    RefereeReceiveData(good.data() + 4U, good.size() - 4U);
    test_primask = 1U;
    assert(RefereeReadUIRobotId(&id) && id == 104U && test_primask == 1U);
    test_primask = 0U;
    now_us = 499000U;
    assert(RefereeReadUIRobotId(&id));
    auto unrelated = Frame(0x7777U, 1U, 3U);
    RefereeReceiveData(unrelated.data(), unrelated.size());
    now_us = 500000U;
    assert(!RefereeReadUIRobotId(&id) && id == 104U);
    for (uint8_t valid : {3U, 4U, 5U, 103U, 104U, 105U})
    {
        ReceiveId(valid);
        assert(RefereeReadUIRobotId(&id) && id == valid);
    }
    ReceiveId(7U);
    assert(!RefereeReadUIRobotId(&id));
    now_us = (uint64_t(UINT32_MAX) - 20U) * 1000U;
    ReceiveId(3U);
    now_us += 499000U;
    assert(RefereeReadUIRobotId(&id));
    now_us += 1000U;
    assert(!RefereeReadUIRobotId(&id));
}

static void Capture()
{
    using namespace MessageCenter;
    ClientUI_Capture();
    auto view = Client_UI_State_Topic.ReadWithMeta();
    assert(view.valid && !view.data.pitch_valid && !view.data.speed_valid);
    assert(view.data.autoaim == Enum_Client_UI_AutoAim::Unknown && !view.data.yaw_valid);
    const auto sequence = view.sequence;
    now_us = 9999U;
    ClientUI_Capture();
    assert(Client_UI_State_Topic.Sequence() == sequence);
    now_us = 10000U;
    GimbalFeedback gimbal{};
    gimbal.pitch_rad = 0.5f;
    gimbal.yaw_rad = 1.5f;
    gimbal.ins_valid = true;
    ShootFeedback shoot{};
    shoot.online = true;
    shoot.friction_left_speed_rad_s = -100.0f;
    shoot.friction_right_speed_rad_s = 200.0f;
    shoot.loader_speed_rad_s = -2.0f;
    Gimbal_Feedback_Topic.Publish(gimbal);
    Shoot_Feedback_Topic.Publish(shoot);
    ChassisFeedback chassis{};
    chassis.online = chassis.enabled = true;
    Chassis_Feedback_Topic.Publish(chassis);
    ChassisCmd command{};
    command.mode = ChassisMode::ROTATE;
    Chassis_Command_Topic.Publish(command);
    ClientUI_Capture();
    view = Client_UI_State_Topic.ReadWithMeta();
    assert(view.data.pitch_valid && view.data.pitch_rad == 0.5f && !view.data.yaw_valid);
    assert(view.data.speed_valid && view.data.left_speed_rad_s == 100.0f);
    assert(view.data.loader == Enum_Client_UI_Motion::Moving);
    assert(view.data.chassis == Enum_Client_UI_Chassis::Spin && std::strcmp(view.data.mode_text, "SPIN") == 0);
    now_us = 110000U;
    ClientUI_Capture();
    view = Client_UI_State_Topic.ReadWithMeta();
    assert(view.data.pitch_valid && view.data.pitch_timestamp_us == 10000U);
    now_us = 120000U;
    ClientUI_Capture();
    view = Client_UI_State_Topic.ReadWithMeta();
    assert(!view.data.pitch_valid && !view.data.speed_valid && view.data.chassis == Enum_Client_UI_Chassis::Unknown);
    for (unsigned step = 0U; step < 5U; ++step)
    {
        now_us += 10000U;
        gimbal.pitch_rad = step == 0U ? std::numeric_limits<float>::quiet_NaN() : 0.3f;
        gimbal.ins_valid = step != 1U;
        shoot.friction_left_speed_rad_s = std::numeric_limits<float>::infinity();
        shoot.loader_speed_rad_s = 0.0f;
        shoot.online = step != 1U;
        Gimbal_Feedback_Topic.Publish(gimbal);
        Shoot_Feedback_Topic.Publish(shoot);
        chassis.enabled = step == 4U;
        chassis.online = step != 0U;
        command.mode = step == 2U ? ChassisMode::ZERO_FORCE : ChassisMode::NO_FOLLOW;
        Chassis_Feedback_Topic.Publish(chassis);
        Chassis_Command_Topic.Publish(command);
        ClientUI_Capture();
        const auto state = Client_UI_State_Topic.ReadWithMeta().data;
        assert(state.pitch_valid == (step >= 2U) && !state.speed_valid);
        assert(state.loader == (step == 1U ? Enum_Client_UI_Motion::Unknown : Enum_Client_UI_Motion::Stopped));
        const Enum_Client_UI_Chassis expected[] = {Enum_Client_UI_Chassis::Unknown, Enum_Client_UI_Chassis::Waiting,
                                                   Enum_Client_UI_Chassis::Stopped, Enum_Client_UI_Chassis::Waiting, Enum_Client_UI_Chassis::Active};
        assert(state.chassis == expected[step]);
    }
}

static void Layout()
{
    interaction_figure_t graphs[7]{}, expected[7]{};
    auto state = LiveState();
    state.pitch_rad = 40.0f / ClientUILayout::kRadToDeg;
    state.left_speed_rad_s = state.right_speed_rad_s = 7000.0f / ClientUILayout::kRadSToRpm;
    ClientUILayout::BuildDynamic(state, expected);
    state.pitch_rad = 100.0f;
    state.left_speed_rad_s = state.right_speed_rad_s = 100000.0f;
    ClientUILayout::BuildDynamic(state, graphs);
    assert(std::memcmp(&graphs[1], &expected[1], sizeof(graphs[1])) == 0);
    assert(graphs[4].start_y == 777U && graphs[5].start_y == 777U);
    assert(graphs[3].details_a == 0U && graphs[3].details_b == 360U);
    assert(graphs[2].operate_type == operate_type_Del && graphs[0].operate_type == operate_type_Del);
    state.pitch_rad = std::numeric_limits<float>::quiet_NaN();
    state.left_speed_rad_s = -1.0f;
    state.right_speed_rad_s = std::numeric_limits<float>::infinity();
    state.yaw_valid = true;
    state.relative_yaw_rad = std::numeric_limits<float>::max();
    ClientUILayout::BuildDynamic(state, graphs);
    assert(graphs[1].operate_type == operate_type_Del && graphs[4].operate_type == operate_type_Del && graphs[5].operate_type == operate_type_Del);
    assert(graphs[3].details_a < 360U && graphs[3].details_b < 360U);
}

static void Schedule()
{
    Start();
    for (unsigned ms = 0; ms < 40U; ms += 10U)
        Tick(ms);
    assert(packets.empty());
    tx_status = HAL_BUSY;
    Tick(40U);
    const auto failed = packets.back().bytes;
    tx_status = HAL_OK;
    Tick(50U);
    assert(packets.back().bytes == failed);
    Tick(89U);
    assert(packets.size() == 2U);
    Tick(90U);
    assert(Dynamic(packets.back()) && FigureAt(packets.back(), 1U).operate_type == operate_type_ADD);
    // On a failed dynamic update the same ADD/Change state and sequence must be retried.
    Tick(130U);
    tx_status = HAL_ERROR;
    Tick(170U);
    const auto dynamic_failed = packets.back().bytes;
    tx_status = HAL_OK;
    Tick(180U);
    assert(packets.back().bytes == dynamic_failed && FigureAt(packets.back(), 1U).operate_type == operate_type_Change);
    for (unsigned ms = 190U; ms <= 9000U; ms += 10U)
        Tick(ms);
    uint32_t last_ms = 0U;
    bool sent = false, readded = false;
    unsigned clears = 0U;
    std::array<bool, 37> static_seen{};
    std::array<bool, 7> labels_seen{};
    for (const auto& packet : packets)
    {
        if (packet.status != HAL_OK)
            continue;
        assert(!sent || packet.ms - last_ms >= 40U);
        sent = true;
        last_ms = packet.ms;
        const auto content = U16(packet.bytes, 7U);
        if (content == 0x0100U)
            ++clears;
        else if (Dynamic(packet))
            readded |= packet.ms >= 5110U && FigureAt(packet, 1U).operate_type == operate_type_ADD;
        else if (content == 0x0104U || content == 0x0102U)
        {
            const unsigned count = content == 0x0104U ? 7U : 2U;
            for (unsigned i = 0; i < count; ++i)
            {
                const auto figure = FigureAt(packet, i);
                assert(figure.layer == 9U && figure.figure_name[2] < static_seen.size());
                static_seen[figure.figure_name[2]] = true;
            }
        }
        else if (content == 0x0110U)
        {
            const uint8_t id = packet.bytes[15];
            if (id >= 150U && id < 157U)
                labels_seen[id - 150U] = true;
        }
    }
    assert(clears == 1U && readded);
    for (bool seen : static_seen)
        assert(seen);
    for (bool seen : labels_seen)
        assert(seen);
}

static void Expiry()
{
    Start();
    Tick(100U);
    Tick(200U);
    assert(FigureAt(packets.back(), 1U).operate_type == operate_type_ADD);
    // Snapshot still <200 ms old, but its feedback has aged past 100 ms.
    Tick(300U, false);
    Tick(400U, false);
    assert(Dynamic(packets.back()));
    assert(FigureAt(packets.back(), 1U).operate_type == operate_type_Del);
    assert(FigureAt(packets.back(), 4U).operate_type == operate_type_Del);
    assert(FigureAt(packets.back(), 6U).color == color_White);
    Tick(500U);
    Tick(600U);
    assert(Dynamic(packets.back()) && FigureAt(packets.back(), 1U).operate_type == operate_type_ADD);
    // Explicit extension inputs disappear when their overall snapshot expires.
    now_us = 700000U;
    auto view = LiveState();
    view.yaw_valid = true;
    view.relative_yaw_rad = 0.3f;
    view.autoaim = Enum_Client_UI_AutoAim::Active;
    std::memset(view.alert_text[0], 'A', sizeof(view.alert_text[0]));
    MessageCenter::Client_UI_State_Topic.Publish(view);
    ClientUI_Update();
    assert(U16(packets.back().bytes, 7U) == 0x0110U && FigureAt(packets.back(), 0U).details_b == 29U);
    Tick(800U, false);
    assert(FigureAt(packets.back(), 2U).operate_type == operate_type_ADD);
    Tick(910U, false);
    Tick(1010U, false);
    assert(Dynamic(packets.back()) && FigureAt(packets.back(), 2U).operate_type == operate_type_Del);
    assert(FigureAt(packets.back(), 3U).details_b == 360U);
}

static void Recovery()
{
    Start(0U);
    Tick(100U);
    assert(packets.empty());
    ReceiveId(3U);
    ClientUI_Update();
    CheckPacket(packets.back(), 3U, 0x0100U, 2U);
    Tick(200U);
    Tick(600U); // exactly 500 ms since the identity frame
    size_t before = packets.size();
    Tick(700U);
    assert(packets.size() == before);
    ReceiveId(3U);
    tx_status = HAL_BUSY;
    ClientUI_Update();
    const auto failed = packets.back().bytes;
    tx_status = HAL_OK;
    ClientUI_Update();
    assert(failed == packets.back().bytes);
    CheckPacket(packets.back(), 3U, 0x0100U, 2U);
    now_us = 800000U;
    ReceiveId(105U);
    ClientUI_Update();
    CheckPacket(packets.back(), 105U, 0x0100U, 2U);
    Tick(900U);
    assert(FigureAt(packets.back(), 1U).operate_type == operate_type_ADD);
    // The sender sequence wraps naturally and does not stall scheduling.
    const uint8_t seq = packets.back().bytes[3];
    for (unsigned i = 0; i < 256U; ++i)
        assert(RefereeUITryDeleteLayer(3U, 9U));
    assert(packets.back().bytes[3] == seq);
}


template <typename T>
static void ReceivePayload(uint16_t command, const T& payload)
{
    auto frame = Frame(command, sizeof(payload), 0U);
    std::memcpy(frame.data() + 7U, &payload, sizeof(payload));
    Append_CRC16_Check_Sum(frame.data(), frame.size());
    RefereeReceiveData(frame.data(), frame.size());
}

static std::string Text(const Packet& p)
{
    return std::string(reinterpret_cast<const char*>(p.bytes.data() + 28U), FigureAt(p, 0U).details_b);
}

static void RefereeSnapshot()
{
    referee_ui_snapshot_t snapshot{};
    snapshot.barrel_heat = 99U;
    assert(!RefereeReadUISnapshot(&snapshot) && snapshot.barrel_heat == 99U);
    Start();
    assert(!RefereeReadUISnapshot(nullptr));
    assert(RefereeReadUISnapshot(&snapshot) && !snapshot.heat_received);
    robot_status_t status{};
    status.robot_id = 3U;
    status.shooter_barrel_heat_limit = 240U;
    now_us = 10000U;
    ReceivePayload(0x0201U, status);
    power_heat_data_t heat{};
    heat.buffer_energy = 0U;
    heat.shooter_17mm_barrel_heat = 120U;
    now_us = 20000U;
    ReceivePayload(0x0202U, heat);
    projectile_allowance_t ammo{};
    ammo.projectile_allowance_17mm = 45U;
    now_us = 30000U;
    ReceivePayload(0x0208U, ammo);
    test_primask = 1U;
    assert(RefereeReadUISnapshot(&snapshot) && test_primask == 1U);
    test_primask = 0U;
    assert(snapshot.status_received && snapshot.heat_received && snapshot.allowance_received);
    assert(snapshot.status_ms == 10U && snapshot.heat_ms == 20U && snapshot.allowance_ms == 30U);
    assert(snapshot.barrel_heat_limit == 240U && snapshot.barrel_heat == 120U && snapshot.projectile_allowance == 45U);
    const auto saved = snapshot;
    now_us = 40000U;
    for (uint16_t command : {0x0201U, 0x0202U, 0x0208U})
    {
        const unsigned size = command == 0x0201U ? 13U : command == 0x0202U ? 14U : 8U;
        auto bad = Frame(command, size, 3U);
        bad.back() ^= 1U;
        RefereeReceiveData(bad.data(), bad.size());
        auto short_frame = Frame(command, size - 1U, 3U);
        RefereeReceiveData(short_frame.data(), short_frame.size());
    }
    auto unrelated = Frame(0x7777U, 1U, 3U);
    RefereeReceiveData(unrelated.data(), unrelated.size());
    assert(RefereeReadUISnapshot(&snapshot));
    assert(std::memcmp(&snapshot, &saved, sizeof(snapshot)) == 0);
    auto split = Frame(0x0202U, 14U, 0U);
    RefereeReceiveData(split.data(), 10U);
    assert(RefereeReadUISnapshot(&snapshot) && snapshot.heat_ms == 20U);
    RefereeReceiveData(split.data() + 10U, split.size() - 10U);
    assert(RefereeReadUISnapshot(&snapshot) && snapshot.heat_ms == 40U && snapshot.barrel_heat == 0U);
    ClientUI_Capture();
    const auto view = MessageCenter::Client_UI_State_Topic.ReadWithMeta().data;
    assert(view.referee_status_ms == 10U && view.referee_heat_ms == 40U && view.referee_allowance_ms == 30U);
    assert(view.referee_heat_valid && view.buffer_energy_j == 0U);
    assert(!view.power_valid && view.fire_mode == Enum_Client_UI_FireMode::Unknown);
    assert(RefereeInit(&uart));
    assert(RefereeReadUISnapshot(&snapshot) && !snapshot.status_received && !snapshot.heat_received && !snapshot.allowance_received);
}

static void PublishExtended()
{
    auto state = LiveState();
    state.power_valid = true;
    state.power_w = 100.5f;
    state.power_timestamp_us = state.fire_mode_timestamp_us = now_us;
    state.fire_mode = Enum_Client_UI_FireMode::Triple;
    state.referee_status_valid = state.referee_heat_valid = state.referee_allowance_valid = true;
    state.referee_status_ms = state.referee_heat_ms = state.referee_allowance_ms = HAL_GetTick();
    state.barrel_heat = HAL_GetTick() / 10U;
    state.barrel_heat_limit = 240U;
    state.projectile_allowance = 45U;
    state.buffer_energy_j = HAL_GetTick() / 10U;
    MessageCenter::Client_UI_State_Topic.Publish(state);
}

static void Fairness()
{
    Start();
    for (uint32_t ms = 0U; ms <= 12000U; ms += 10U)
    {
        now_us = uint64_t(ms) * 1000U;
        PublishExtended();
        const auto before = packets.size();
        ClientUI_Update();
        assert(packets.size() - before <= 1U);
    }
    uint32_t dynamic_ms = 0U, add_ms = 0U, buffer_ms = 0U;
    unsigned dynamics = 0U, adds = 0U;
    std::array<bool, 37> figures{};
    std::array<bool, 7> labels{};
    for (size_t n = 0U; n < packets.size(); ++n)
    {
        const auto& p = packets[n];
        assert(p.bytes.size() <= 120U);
        if (n) assert(p.ms - packets[n - 1U].ms >= 40U);
        size_t bytes = 0U;
        for (size_t j = n; j < packets.size() && packets[j].ms - p.ms < 1000U; ++j)
            bytes += packets[j].bytes.size();
        assert(bytes <= 3000U);
        if (Dynamic(p))
        {
            if (dynamics++) assert(p.ms - dynamic_ms <= 80U);
            dynamic_ms = p.ms;
            if (FigureAt(p, 1U).operate_type == operate_type_ADD)
            {
                if (adds++) assert(p.ms - add_ms <= 1080U);
                add_ms = p.ms;
            }
        }
        else if (U16(p.bytes, 7U) == 0x0110U)
        {
            const auto f = FigureAt(p, 0U);
            const auto id = f.figure_name[2];
            if (id == 174U)
            {
                if (buffer_ms && f.operate_type != operate_type_ADD) assert(p.ms - buffer_ms >= 200U);
                buffer_ms = p.ms;
            }
            if (p.ms < 5000U && id >= 150U && id < 157U) labels[id - 150U] = true;
        }
        else if (U16(p.bytes, 7U) != 0x0100U && p.ms < 5000U)
        {
            const unsigned count = U16(p.bytes, 7U) == 0x0104U ? 7U : 2U;
            for (unsigned i = 0U; i < count; ++i) figures[FigureAt(p, i).figure_name[2]] = true;
        }
    }
    for (bool seen : figures) assert(seen);
    for (bool seen : labels) assert(seen);
    assert(dynamics >= 149U && adds >= 11U);
}

static void Loss()
{
    // Client model: ADD creates only missing figures; Change updates only existing ones.
    for (bool existing : {false, true})
        for (unsigned dropped = 0U; dropped < 4U; ++dropped)
        {
            Start();
            unsigned counts[2]{};
            bool present[2] = {existing, existing};
            std::string displayed[2] = {"OLD", "OLD"};
            for (unsigned step = 0U; step < 400U && (counts[0] < 4U || counts[1] < 4U); ++step)
            {
                now_us += 10000U;
                PublishExtended();
                const size_t before = packets.size();
                ClientUI_Update();
                if (before == packets.size()) continue;
                const auto& p = packets.back();
                if (U16(p.bytes, 7U) != 0x0110U) continue;
                const auto f = FigureAt(p, 0U);
                if (f.figure_name[2] != 173U && f.figure_name[2] != 176U) continue;
                const unsigned i = f.figure_name[2] == 173U ? 0U : 1U;
                const unsigned count = counts[i]++;
                assert(count < 4U);
                assert(f.operate_type == (count % 2U ? operate_type_Change : operate_type_ADD));
                if (count == dropped) continue;
                if ((f.operate_type == operate_type_ADD && !present[i]) ||
                    (f.operate_type == operate_type_Change && present[i]))
                {
                    displayed[i] = Text(p);
                    present[i] = true;
                }
            }
            assert(counts[0] == 4U && counts[1] == 4U);
            assert(present[0] && displayed[0] == "SPIN");
            assert(present[1] && displayed[1] == "FIRE TRIPLE");
            counts[0] = counts[1] = 0U;
            for (unsigned step = 0U; step < 400U && (counts[0] < 4U || counts[1] < 4U); ++step)
            {
                now_us += 10000U;
                auto view = LiveState();
                view.mode_text[0] = '\0';
                MessageCenter::Client_UI_State_Topic.Publish(view);
                const size_t before = packets.size();
                ClientUI_Update();
                if (before == packets.size()) continue;
                const auto& p = packets.back();
                if (U16(p.bytes, 7U) != 0x0110U) continue;
                const auto f = FigureAt(p, 0U);
                if (f.figure_name[2] != 173U && f.figure_name[2] != 176U) continue;
                const unsigned i = f.figure_name[2] == 173U ? 0U : 1U;
                const unsigned count = counts[i]++;
                assert(count < 4U && f.operate_type == operate_type_Del);
                if (count != dropped) present[i] = false;
            }
            assert(counts[0] == 4U && counts[1] == 4U && !present[0] && !present[1]);
        }
    // Busy does not consume a repeat; changing targets drops pending old repeats.
    Start();
    bool changed = false, retried = false;
    unsigned new_count = 0U;
    for (unsigned step = 0U; step < 400U && new_count < 4U; ++step)
    {
        now_us += 10000U;
        auto view = LiveState();
        std::strcpy(view.mode_text, changed ? "FREE" : "FOLLOW");
        MessageCenter::Client_UI_State_Topic.Publish(view);
        const auto before = packets.size();
        if (changed && !retried) tx_status = HAL_BUSY;
        ClientUI_Update();
        if (before == packets.size()) continue;
        const auto p = packets.back();
        if (p.status == HAL_BUSY)
        {
            tx_status = HAL_OK;
            ClientUI_Update();
            assert(p.bytes == packets.back().bytes);
            retried = true;
        }
        if (U16(p.bytes, 7U) == 0x0110U && p.bytes[15] == 173U)
        {
            if (!changed) changed = true;
            else
            {
                assert(Text(p) == "FREE");
                assert(FigureAt(p, 0U).operate_type == (new_count % 2U ? operate_type_Change : operate_type_ADD));
                ++new_count;
            }
        }
    }
    assert(retried && new_count == 4U);
}

static void Values()
{
    Start();
    // Exact power boundary, invalid values, and packed zero/fractional float representation.
    interaction_figure_t figures[7]{};
    auto state = LiveState();
    state.power_valid = true;
    for (float value : {0.0f, 100.5f, 3276.75f, -1.0f, 3276.76f,
                        std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()})
    {
        state.power_w = value;
        ClientUILayout::BuildDynamic(state, figures);
        const bool valid = std::isfinite(value) && value >= 0.0f && value <= 3276.75f;
        assert((figures[0].operate_type != operate_type_Del) == valid);
        if (valid)
        {
            const uint32_t encoded = figures[0].details_c | (figures[0].details_d << 10U) | (figures[0].details_e << 21U);
            assert(encoded == uint32_t(std::lround(value * 1000.0f)));
        }
    }
    // Staggered frame ages: independently remove limit, then heat/buffer, then ammo.
    std::string last_buffer, last_combat, last_fire;
    unsigned fire_deletes = 0U;
    bool power_deleted = false, power_restored = false;
    for (uint32_t ms = 0U; ms <= 3600U; ms += 10U)
    {
        now_us = uint64_t(ms) * 1000U;
        auto view = LiveState();
        view.power_valid = true;
        view.power_w = 0.0f;
        view.power_timestamp_us = ms < 2000U ? now_us : 1990000U;
        if (ms >= 3000U) view.power_timestamp_us = now_us;
        view.fire_mode = Enum_Client_UI_FireMode::Single;
        view.fire_mode_timestamp_us = ms < 1600U ? now_us : 1590000U;
        view.referee_status_valid = view.referee_heat_valid = view.referee_allowance_valid = true;
        view.referee_status_ms = ms < 1000U ? ms : 990U;
        view.referee_heat_ms = ms < 1600U ? ms : 1590U;
        view.referee_allowance_ms = ms < 2200U ? ms : 2190U;
        view.barrel_heat_limit = 240U;
        view.projectile_allowance = 45U;
        MessageCenter::Client_UI_State_Topic.Publish(view);
        const auto before = packets.size();
        ClientUI_Update();
        if (before != packets.size())
        {
            const auto& p = packets.back();
            if (Dynamic(p))
            {
                if (ms < 2090U) assert(FigureAt(p, 0U).operate_type != operate_type_Del);
                if (ms >= 2090U && ms < 3000U)
                {
                    assert(FigureAt(p, 0U).operate_type == operate_type_Del);
                    power_deleted = true;
                }
                if (ms >= 3000U && !power_restored)
                {
                    assert(FigureAt(p, 0U).operate_type == operate_type_ADD);
                    power_restored = true;
                }
            }
            if (U16(p.bytes, 7U) == 0x0110U)
            {
                const uint8_t id = p.bytes[15];
                if (id == 174U) last_buffer = Text(p);
                if (id == 175U) last_combat = Text(p);
                if (id == 176U)
                {
                    last_fire = Text(p);
                    if (FigureAt(p, 0U).operate_type == operate_type_Del) ++fire_deletes;
                }
            }
        }
        if (ms == 1400U) assert(last_buffer == "BUFFER 0 J" && last_combat == "HEAT 0/240 AMMO 45");
        if (ms == 2000U) assert(last_combat == "HEAT 0 AMMO 45");
        if (ms == 2600U) assert(last_buffer.empty() && last_combat == "AMMO 45");
    }
    assert(last_combat.empty() && last_fire.empty() && fire_deletes == 4U);
    assert(power_deleted && power_restored);
}

static void Boundaries()
{
    for (uint64_t age : {99999U, 100000U})
    {
        now_us = 0U;
        Start();
        Tick(40U);
        now_us = age;
        auto view = LiveState();
        view.power_valid = true;
        view.power_w = 0.0f;
        view.power_timestamp_us = 0U;
        MessageCenter::Client_UI_State_Topic.Publish(view);
        ClientUI_Update();
        assert(Dynamic(packets.back()));
        assert((FigureAt(packets.back(), 0U).operate_type == operate_type_Del) == (age == 100000U));
    }
    for (unsigned age_extra : {0U, 1U})
    {
        now_us = 1000000U;
        Start();
        bool buffer_seen = false, combat_seen = false, fire_seen = false;
        for (unsigned step = 0U; step < 300U; ++step)
        {
            now_us += 10000U;
            auto view = LiveState();
            view.referee_heat_valid = view.referee_status_valid = view.referee_allowance_valid = true;
            view.referee_heat_ms = view.referee_status_ms = view.referee_allowance_ms = HAL_GetTick() - 499U - age_extra;
            view.barrel_heat_limit = view.barrel_heat = view.projectile_allowance = 65535U;
            view.fire_mode = Enum_Client_UI_FireMode::Burst;
            view.fire_mode_timestamp_us = now_us - 200000U - age_extra;
            MessageCenter::Client_UI_State_Topic.Publish(view);
            const size_t before = packets.size();
            ClientUI_Update();
            if (before == packets.size()) continue;
            const auto& p = packets.back();
            if (U16(p.bytes, 7U) != 0x0110U) continue;
            if (p.bytes[15] == 174U) buffer_seen = Text(p) == "BUFFER 0 J";
            if (p.bytes[15] == 175U) combat_seen = Text(p) == "HEAT 65535/65535 AMMO 65535";
            if (p.bytes[15] == 176U) fire_seen = Text(p) == "FIRE BURST";
        }
        assert(buffer_seen == (age_extra == 0U));
        assert(combat_seen == (age_extra == 0U));
        assert(fire_seen == (age_extra == 0U));
    }
}

static void Rollover()
{
    now_us = (uint64_t(UINT32_MAX) - 100U) * 1000U;
    Start();
    for (unsigned step = 0U; step < 300U; ++step)
    {
        now_us += 10000U;
        PublishExtended();
        ClientUI_Update();
    }
    for (size_t i = 1U; i < packets.size(); ++i)
        assert(uint32_t(packets[i].ms - packets[i - 1U].ms) == 40U);
    assert(packets.size() >= 74U);
    now_us += 10000000U;
    PublishExtended();
    ClientUI_Update();
    const auto before = packets.size();
    for (unsigned i = 0U; i < 10U; ++i) ClientUI_Update();
    assert(packets.size() == before); // No catch-up burst.
}

int main(int argc, char** argv)
{
    assert(argc == 2);
    const std::string name = argv[1];
    if (name == "protocol")
        Protocol();
    else if (name == "identity")
        Identity();
    else if (name == "capture")
        Capture();
    else if (name == "layout")
        Layout();
    else if (name == "schedule")
        Schedule();
    else if (name == "expiry")
        Expiry();
    else if (name == "recovery")
        Recovery();
    else if (name == "referee") RefereeSnapshot();
    else if (name == "fairness") Fairness();
    else if (name == "loss") Loss();
    else if (name == "values") Values();
    else if (name == "rollover") Rollover();
    else if (name == "boundaries") Boundaries();
    else
        assert(false);
    std::printf("PASS %s\n", argv[1]);
}
