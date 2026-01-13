/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#include "e5010_power_control.hpp"

namespace power_control
{

// Constructor: E5010 has no platform-specific GPIOs (no PDB)
E5010PowerControl::E5010PowerControl(
    boost::asio::io_context& ioContext,
    std::shared_ptr<sdbusplus::asio::connection> conn,
    const std::string& configFilePath, const std::string& node,
    PersistentState& appState) :
    VRPowerControl(ioContext, conn, configFilePath, node,
                   appState) // Call parent constructor (registers VR GPIOs)
{
    // powerSignalMap is now populated by base class
    // PowerControl::loadConfigValues() VR handlers already added to
    // gpioHandlerMap by VRPowerControl constructor E5010 has no
    // platform-specific GPIOs (no PDB), so no handlers to add

    // call validateRequiredSignals() to validate all required signals
    validateRequiredSignals();

    // call validateTimerConfigs() to validate all required timers
    validateTimerConfigs();

    // call setDefaultValues() to set the default values for output signals
    setDefaultValues();

    // Register all GPIO handlers (from base and VR only)
    registerGPIOHandlers();
}

std::function<void(Event)> E5010PowerControl::getPowerStateHandler()
{
    // E5010 does not define new PowerState values, so delegate everything
    // to VRPowerControl which handles all VR and upstream states
    switch (powerState)
    {
        // TODO: Add E5010-specific states here, that are overridden by
        // E5010PowerControl

        // Delegate all states to parent VRPowerControl
        default:
            return VRPowerControl::getPowerStateHandler();
    }
}

void E5010PowerControl::handlePowerStateOn(Event event)
{
    // TODO: Move E5010-specific powerStateOn() implementation here
}

void E5010PowerControl::handlePowerStateOff(Event event)
{
    // TODO: Move E5010-specific powerStateOff() implementation here
}

void E5010PowerControl::handleWaitForHPMPowerGoodDeAssert(Event event)
{
    // TODO: Move E5010-specific powerStateWaitForHPMPowerGoodDeAssert()
    // implementation here
}

void E5010PowerControl::validateRequiredSignals()
{
    // E5010 has no PDB, so no platform-specific signals to validate
    // Just call VRPowerControl to validate common VR signals
    VRPowerControl::validateRequiredSignals();

    lg2::info("E5010 signal validation complete");
}

void E5010PowerControl::validateTimerConfigs()
{
    // E5010 has no PDB, so no platform-specific timers
    // Just call VRPowerControl to validate common VR timers
    VRPowerControl::validateTimerConfigs();

    lg2::info("E5010 timer configuration validation complete");
}

void E5010PowerControl::setDefaultValues()
{
    // E5010 has no PDB, so no platform-specific defaults to set
    // Just call parent to set common VR/HPM defaults
    lg2::info("Setting E5010 default values (calling parent for VR defaults)");
    VRPowerControl::setDefaultValues();
    lg2::info("E5010 default values set successfully");
}

} // namespace power_control
