/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#include "c2_power_control.hpp"

namespace power_control
{

std::function<void(Event)> C2PowerControl::getPowerStateHandler(PowerState state)
{
    // C2 does not define new PowerState values, so delegate everything
    // to VRPowerControl which handles all VR and upstream states
    switch (state)
    {
        // C2 modifies the handleWaitForHPMPowerGoodDeAssert and handlePowerStateOff handlers to handle the C2 PDB 12V rails.
        case PowerState::handleWaitForHPMPowerGoodDeAssert:
            return [this](Event e) { this->handleWaitForHPMPowerGoodDeAssert(e); };
        case PowerState::handlePowerStateOff:
            return [this](Event e) { this->handlePowerStateOff(e); };
        // Delegate all states to parent VRPowerControl
        default:
            return VRPowerControl::getPowerStateHandler(state);
    }
}

void C2PowerControl::handlePowerStateOn(Event event)
{
    // TODO: Move C2-specific powerStateOn() implementation here
}

void C2PowerControl::handlePowerStateOff(Event event)
{
    // TODO: Move C2-specific powerStateOff() implementation here
}

void C2PowerControl::handleWaitForPDBMainPowerOk(Event event)
{
    // TODO: Move C2-specific powerStateWaitForPDBMainPowerOk() implementation here
}

void C2PowerControl::handleWaitForHPMPowerGoodDeAssert(Event event)
{
    // TODO: Move C2-specific powerStateWaitForHPMPowerGoodDeAssert() implementation here
}
} // namespace power_control

