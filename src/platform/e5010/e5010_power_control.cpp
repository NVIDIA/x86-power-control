/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#include "e5010_power_control.hpp"

// External references to global variables from power_control.cpp
namespace power_control
{
    extern PowerState powerState;
    extern Context powerContext;
}

namespace power_control
{

// Constructor: E5010 has no platform-specific GPIOs (no PDB)
E5010PowerControl::E5010PowerControl(boost::asio::io_context& ioContext)
    : VRPowerControl(ioContext)  // Call parent constructor (registers VR GPIOs)
{
    // powerSignalMap is now populated by base class PowerControl::loadConfigValues()
    // VR handlers already assigned and registered by VRPowerControl constructor
    // E5010 has no platform-specific GPIOs (no PDB), so nothing to register here
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

} // namespace power_control

