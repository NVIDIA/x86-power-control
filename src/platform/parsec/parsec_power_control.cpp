/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#include "parsec_power_control.hpp"

namespace power_control
{

std::function<void(Event)> ParsecPowerControl::getPowerStateHandler(PowerState state)
{
    // Parsec does not define new PowerState values, so delegate everything
    // to VRPowerControl which handles all VR and upstream states
    switch (state)
    {
        // TODO: Add Parsec-specific states here, that are overriden by ParsecPowerControl
        
        // Delegate all states to parent VRPowerControl
        default:
            return VRPowerControl::getPowerStateHandler(state);
    }
}

void ParsecPowerControl::handlePowerStateOn(Event event)
{
    // TODO: Move Parsec-specific powerStateOn() implementation here
}

void ParsecPowerControl::handlePowerStateOff(Event event)
{
    // TODO: Move Parsec-specific powerStateOff() implementation here
}

void ParsecPowerControl::handleWaitForPDBMainPowerOk(Event event)
{
    // TODO: Move Parsec-specific powerStateWaitForPDBMainPowerOk() implementation here
}

} // namespace power_control

