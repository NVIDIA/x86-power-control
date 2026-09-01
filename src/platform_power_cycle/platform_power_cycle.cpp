#include "platform_power_cycle.hpp"

#include <xyz/openbmc_project/Common/error.hpp>

#include <algorithm>

namespace nvidia::platform_power_cycle
{
namespace
{
constexpr std::string_view systemdBus = "org.freedesktop.systemd1";
constexpr std::string_view systemdPath = "/org/freedesktop/systemd1";
constexpr std::string_view systemdManager = "org.freedesktop.systemd1.Manager";

std::string_view baseName(std::string_view path)
{
    const std::size_t separator = path.find_last_of('/');
    if (separator == std::string_view::npos)
    {
        return path;
    }
    return path.substr(separator + 1);
}

bool isStartableState(std::string_view state)
{
    return state != "bad" && state != "masked" && state != "masked-runtime";
}

const PowerCycleUnit* findPowerCycleUnit(PowerCycleType type)
{
    const auto unit =
        std::ranges::find(powerCycleUnits, type, &PowerCycleUnit::type);
    return unit == powerCycleUnits.end() ? nullptr : &*unit;
}
} // namespace

std::vector<PowerCycleType> getSupportedPowerCycleTypes(
    const std::vector<UnitFile>& unitFiles)
{
    std::vector<PowerCycleType> supported;
    for (const PowerCycleUnit& candidate : powerCycleUnits)
    {
        const auto installed = std::ranges::find_if(
            unitFiles, [&candidate](const UnitFile& unitFile) {
                const auto& [path, state] = unitFile;
                return baseName(path) == candidate.unit &&
                       isStartableState(state);
            });
        if (installed != unitFiles.end())
        {
            supported.push_back(candidate.type);
        }
    }
    return supported;
}

std::vector<UnitFile> listUnitFiles(sdbusplus::bus_t& bus)
{
    auto request = bus.new_method_call(systemdBus.data(), systemdPath.data(),
                                       systemdManager.data(), "ListUnitFiles");
    auto response = bus.call(request);
    std::vector<UnitFile> unitFiles;
    response.read(unitFiles);
    return unitFiles;
}

PlatformPowerCycle::PlatformPowerCycle(sdbusplus::bus_t& bus,
                                       const std::vector<UnitFile>& unitFiles) :
    Interface(bus, objectPath.data()), bus(bus),
    supportedTypes(getSupportedPowerCycleTypes(unitFiles))
{
    supportedPowerCycleTypes(supportedTypes, true);
    emit_added();
}

void PlatformPowerCycle::requestPowerCycle(PowerCycleType type)
{
    using sdbusplus::xyz::openbmc_project::Common::Error::InvalidArgument;
    using sdbusplus::xyz::openbmc_project::Common::Error::Unavailable;

    const PowerCycleUnit* requested = findPowerCycleUnit(type);
    if (requested == nullptr)
    {
        throw InvalidArgument();
    }
    if (std::ranges::find(supportedTypes, type) == supportedTypes.end())
    {
        throw Unavailable();
    }

    try
    {
        auto request =
            bus.new_method_call(systemdBus.data(), systemdPath.data(),
                                systemdManager.data(), "StartUnit");
        request.append(std::string(requested->unit), std::string("replace"));
        bus.call(request);
    }
    catch (const sdbusplus::exception_t&)
    {
        throw Unavailable();
    }
}

} // namespace nvidia::platform_power_cycle
