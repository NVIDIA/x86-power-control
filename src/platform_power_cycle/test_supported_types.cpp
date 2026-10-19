#include "platform_power_cycle.hpp"

#include <gtest/gtest.h>

namespace nvidia::platform_power_cycle
{
namespace
{
TEST(SupportedTypes, ReportsOnlyInstalledFixedUnits)
{
    const std::vector<UnitFile> unitFiles = {
        {"/usr/lib/systemd/system/nvidia-aux-power.service", "disabled"},
        {"/usr/lib/systemd/system/nvidia-full-power-cycle.service", "static"},
        {"/usr/lib/systemd/system/unrelated.service", "enabled"},
    };

    EXPECT_EQ(getSupportedPowerCycleTypes(unitFiles),
              (std::vector{PowerCycleType::AuxPowerCycle,
                           PowerCycleType::FullPowerCycle}));
}

TEST(SupportedTypes, DoesNotReportMissingOrMaskedUnits)
{
    const std::vector<UnitFile> unitFiles = {
        {"/usr/lib/systemd/system/nvidia-full-power-cycle.service", "bad"},
        {"/usr/lib/systemd/system/nvidia-aux-power.service", "masked"},
        {"/run/systemd/system/nvidia-aux-power-force.service",
         "masked-runtime"},
    };

    EXPECT_TRUE(getSupportedPowerCycleTypes(unitFiles).empty());
}

TEST(SupportedTypes, ReportsAuxPowerCycleForceWhenInstalled)
{
    const std::vector<UnitFile> unitFiles = {
        {"/usr/lib/systemd/system/nvidia-aux-power-force.service", "enabled"},
    };

    EXPECT_EQ(getSupportedPowerCycleTypes(unitFiles),
              (std::vector{PowerCycleType::AuxPowerCycleForce}));
}

TEST(SupportedTypes, DoesNotAcceptUnitNameSuffixes)
{
    const std::vector<UnitFile> unitFiles = {
        {"/usr/lib/systemd/system/not-nvidia-aux-power.service", "enabled"},
    };

    EXPECT_TRUE(getSupportedPowerCycleTypes(unitFiles).empty());
}
} // namespace
} // namespace nvidia::platform_power_cycle
