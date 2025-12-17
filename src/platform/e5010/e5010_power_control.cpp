/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#include "e5010_power_control.hpp"

namespace power_control
{

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

