/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#include "nvl144_power_control.hpp"

namespace power_control
{

// NVL144PowerControl implementation
// 
// This class inherits all behavior from VRPowerControl, which already implements
// NVL144 default behavior. No overrides are needed unless NVL144 requires
// platform-specific variations.

std::function<void(Event)> NVL144PowerControl::getPowerStateHandler(PowerState state)
{
    // NVL144 does not define new PowerState values, so delegate everything
    // to VRPowerControl which handles all VR and upstream states
    switch (state)
    {
        // No NVL144-specific states (empty switch)
        
        // Delegate all states to parent VRPowerControl
        default:
            return VRPowerControl::getPowerStateHandler(state);
    }
}

void NVL144PowerControl::handlePowerStateOn(Event event)
{
    // TODO: Move NVL144-specific powerStateOn() implementation here
    //
    // Currently NVL144 uses VRPowerControl default implementation which already
    // implements NVL144 behavior. This override exists for consistency and can be
    // customized if NVL144 requires platform-specific on-state monitoring.
    //
    // For now, call base class:
    // VRPowerControl::handlePowerStateOn(event);
}

} // namespace power_control

