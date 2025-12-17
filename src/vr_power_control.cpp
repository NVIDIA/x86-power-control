/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#include "vr_power_control.hpp"

namespace power_control
{

std::function<void(Event)> VRPowerControl::getPowerStateHandler(PowerState state)
{
    // Map VR-specific PowerState values to their handler functions
    switch (state)
    {
        // VR-specific states
        case PowerState::waitForPDBMainPowerOk:
            return [this](Event e) { this->handleWaitForPDBMainPowerOk(e); };
        
        case PowerState::waitForPDBMainPowerOff:
            return [this](Event e) { this->handleWaitForPDBMainPowerOff(e); };
        
        case PowerState::waitForHPMPowerGoodAssert:
            return [this](Event e) { this->handleWaitForHPMPowerGoodAssert(e); };
        
        case PowerState::waitForHPMPowerGoodDeAssert:
            return [this](Event e) { this->handleWaitForHPMPowerGoodDeAssert(e); };
        
        case PowerState::waitForCPUResetAssert:
            return [this](Event e) { this->handleWaitForCPUResetAssert(e); };
        
        case PowerState::waitForCPUResetDeAssert:
            return [this](Event e) { this->handleWaitForCPUResetDeAssert(e); };
        
        case PowerState::waitForCPUShutdownOk:
            return [this](Event e) { this->handleWaitForCPUShutdownOk(e); };
        
        // Delegate upstream states to base class
        default:
            return PowerControl::getPowerStateHandler(state);
    }
}

void VRPowerControl::handlePowerStateOn(Event event)
{
    // TODO: Move NVL144-specific powerStateOn() implementation here

}

void VRPowerControl::handlePowerStateOff(Event event)
{
    // TODO: Move NVL144-specific powerStateOff() implementation here

}

void VRPowerControl::handleWaitForPDBMainPowerOk(Event event)
{
    // TODO: Move NVL144-specific powerStateWaitForPDBMainPowerOk() implementation here

}

void VRPowerControl::handleWaitForPDBMainPowerOff(Event event)
{
    // TODO: Move powerStateWaitForPDBMainPowerOff() implementation here

}

void VRPowerControl::handleWaitForHPMPowerGoodAssert(Event event)
{
    // TODO: Move powerStateWaitForHPMPowerGoodAssert() implementation here

}

void VRPowerControl::handleWaitForHPMPowerGoodDeAssert(Event event)
{
    // TODO: Move powerStateWaitForHPMPowerGoodDeAssert() implementation here

}

void VRPowerControl::handleWaitForCPUResetAssert(Event event)
{
    // TODO: Move powerStateWaitForCPUResetAssert() implementation here
 
}

void VRPowerControl::handleWaitForCPUResetDeAssert(Event event)
{
    // TODO: Move powerStateWaitForCPUResetDeAssert() implementation here
 

void VRPowerControl::handleWaitForCPUShutdownOk(Event event)
{
    // TODO: Move powerStateWaitForCPUShutdownOk() implementation here

}

} // namespace power_control

