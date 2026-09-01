#pragma once

#include <com/nvidia/Control/Platform/PowerCycle/server.hpp>
#include <sdbusplus/bus.hpp>

#include <array>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace nvidia::platform_power_cycle
{

using Interface = sdbusplus::server::com::nvidia::control::platform::PowerCycle;
using PowerCycleType = Interface::PowerCycleType;
using UnitFile = std::tuple<std::string, std::string>;

inline constexpr std::string_view busName =
    "com.nvidia.Control.Platform.PowerCycle";
inline constexpr std::string_view objectPath =
    "/xyz/openbmc_project/control/power_cycle/host0";

struct PowerCycleUnit
{
    PowerCycleType type;
    std::string_view unit;
};

inline constexpr std::array powerCycleUnits = {
    PowerCycleUnit{PowerCycleType::AuxPowerCycle, "nvidia-aux-power.service"},
    PowerCycleUnit{PowerCycleType::AuxPowerCycleForce,
                   "nvidia-aux-power-force.service"},
    PowerCycleUnit{PowerCycleType::FullPowerCycle,
                   "nvidia-full-power-cycle.service"},
};

std::vector<PowerCycleType> getSupportedPowerCycleTypes(
    const std::vector<UnitFile>& unitFiles);

class PlatformPowerCycle : public Interface
{
  public:
    PlatformPowerCycle(sdbusplus::bus_t& bus,
                       const std::vector<UnitFile>& unitFiles);

    void requestPowerCycle(PowerCycleType type) override;

  private:
    sdbusplus::bus_t& bus;
    std::vector<PowerCycleType> supportedTypes;
};

std::vector<UnitFile> listUnitFiles(sdbusplus::bus_t& bus);

} // namespace nvidia::platform_power_cycle
