#ifndef CLIENT_UI_CONFIG_H
#define CLIENT_UI_CONFIG_H

#include <cstdint>

namespace ClientUIConfig
{
// 0：从裁判 0x0201 获取；只接 TX 调试时显式改为红/蓝步兵 3/4/5。
constexpr uint16_t kRobotId = 0U;
constexpr uint64_t kFeedbackMaxAgeUs = 100000U;
constexpr uint64_t kSnapshotMaxAgeUs = 200000U;
constexpr uint32_t kTargetMaxAgeMs = 100U;
constexpr uint32_t kTargetBlinkPhaseMs = 250U;
constexpr uint32_t kSendIntervalMs = 40U;
constexpr uint32_t kDynamicReaddIntervalMs = 1000U;
constexpr uint32_t kValueIntervalMs = 200U;
constexpr uint32_t kRefereeMaxAgeMs = 500U;
constexpr uint64_t kPowerMaxAgeUs = 100000U;
constexpr uint64_t kFireModeMaxAgeUs = 200000U;
constexpr uint32_t kRebuildIntervalMs = 5000U;
} // namespace ClientUIConfig

#endif
