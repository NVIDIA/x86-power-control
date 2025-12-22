/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#include "e5010_power_control.hpp"

// External references to global variables from power_control.cpp
namespace power_control
{
    extern PowerState powerState;
}

namespace power_control
{

// Constructor: E5010 has no platform-specific GPIOs (no PDB)
E5010PowerControl::E5010PowerControl(boost::asio::io_context& ioContext,
                                     std::shared_ptr<sdbusplus::asio::connection> conn,
                                     const std::string& node)
    : VRPowerControl(ioContext, conn, node)  // Call parent constructor (registers VR GPIOs)
{
    // powerSignalMap is now populated by base class PowerControl::loadConfigValues()
    // VR handlers already added to gpioHandlerMap by VRPowerControl constructor
    // E5010 has no platform-specific GPIOs (no PDB), so no handlers to add

    // call validateRequiredSignals() to validate all required signals
    validateRequiredSignals();

    // Register all GPIO handlers (from base and VR only)
    registerGPIOHandlers();
}

std::function<void(Event)> E5010PowerControl::getPowerStateHandler(PowerState state)
{
    // E5010 does not define new PowerState values, so delegate everything
    // to VRPowerControl which handles all VR and upstream states
    switch (state)
    {
        // TODO: Add E5010-specific states here, that are overriden by E5010PowerControl
        
        // Delegate all states to parent VRPowerControl
        default:
            return VRPowerControl::getPowerStateHandler(state);
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
    // TODO: Move E5010-specific powerStateWaitForHPMPowerGoodDeAssert() implementation here
}

void E5010PowerControl::validateRequiredSignals()
{
    // E5010 has no PDB, so no platform-specific signals to validate
    // Just call VRPowerControl to validate common VR signals
    VRPowerControl::validateRequiredSignals();
    
    lg2::info("E5010 signal validation complete");
}

void E5010PowerControl::setGPIOsForHostStateOn()
{
    // E5010 has no PDB, just call parent to set VR GPIOs
    lg2::info("Setting E5010 GPIOs for host state ON");
    VRPowerControl::setGPIOsForHostStateOn();
}

void E5010PowerControl::setGPIOsForHostStateOff()
{
    // E5010 has no PDB, just call parent to set VR GPIOs
    lg2::info("Setting E5010 GPIOs for host state OFF");
    VRPowerControl::setGPIOsForHostStateOff();
}

} // namespace power_control

