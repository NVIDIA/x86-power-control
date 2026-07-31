// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "config.h"

#include "bmc_state_manager.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/bus.hpp>

#include <cstdlib>
#include <exception>

using BMCState = sdbusplus::server::xyz::openbmc_project::state::BMC;

int main()
try
{
    auto bus = sdbusplus::bus::new_default();

    // For now, we only have one instance of the BMC
    // 0 is for the current instance
    const auto* BMCName = BMCState::namespace_path::bmc;
    const auto* objPath = BMCState::namespace_path::value;
    std::string objPathInst = sdbusplus::object_path(objPath) / BMCName;

    // Add sdbusplus ObjectManager.
    sdbusplus::server::manager_t objManager(bus, objPath);

    phosphor::state::manager::BMC manager(bus, objPathInst.c_str());

    bus.request_name(BMCState::interface);

    while (true)
    {
        bus.process_discard();
        bus.wait();
    }
}
catch (const std::exception& e)
{
    lg2::error("bmc-state-manager: unhandled exception in main: {ERR}", "ERR",
               e.what());
    return EXIT_FAILURE;
}
catch (...)
{
    lg2::error("bmc-state-manager: unknown unhandled exception in main");
    return EXIT_FAILURE;
}
