/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#include "nvl144_power_control.hpp"

// External references to global variables/functions from power_control.cpp
namespace power_control
{
    extern std::map<std::string, ConfigData*> powerSignalMap;
    extern PowerState powerState;
    extern Context powerContext;
    extern ConfigType;
    
    extern bool requestGPIOEvents(
        const std::string& lineName,
        std::function<void(bool)> handler,
        gpiod::line& gpioLine,
        boost::asio::posix::stream_descriptor& gpioEventDescriptor
    );
}

namespace power_control
{

// Constructor: Adds NVL144-specific ConfigData entries to powerSignalMap
NVL144PowerControl::NVL144PowerControl(boost::asio::io_context& ioContext)
    : VRPowerControl(ioContext),              // ← MUST call parent constructor FIRST
      nvl144pdbMainPowerOkEvent(ioContext)    // Then initialize own event descriptors
{
    // Add NVL144-specific signals to powerSignalMap
    powerSignalMap["NVL144PDBMainPowerOk"] = &nvl144pdbMainPowerOkConfig;
    powerSignalMap["NVL144PDBMainPowerEnable"] = &nvl144pdbMainPowerEnableConfig;
    powerSignalMap["E1SPowerEnable"] = &e1sPowerEnableConfig;
    powerSignalMap["BMCSSDReset"] = &bmcSSDResetConfig;
}

// Initialize NVL144-specific GPIO events (PHASE 2: After loadConfigValues())
void NVL144PowerControl::initializeGPIO()
{
    // First, initialize common VR/HPM GPIOs (parent implementation)
    VRPowerControl::initializeGPIO();
    
    // Register NVL144-specific GPIO event handlers
    
    // NVL144 PDB Main Power OK
    if (powerContext.presence.nvl144_pdb && 
        nvl144pdbMainPowerOkConfig.type == ConfigType::GPIO)
    {
        if (!requestGPIOEvents(nvl144pdbMainPowerOkConfig.lineName,
                              [this](bool state) { this->nvl144pdbMainPowerOkHandler(state); },
                              nvl144pdbMainPowerOkLine,
                              nvl144pdbMainPowerOkEvent))
        {
            throw std::runtime_error("NVL144: Failed to register PDB Main Power OK GPIO events");
        }
    }
}

// NVL144-specific GPIO handler implementations
void NVL144PowerControl::nvl144pdbMainPowerOkHandler(bool state)
{
    Event powerControlEvent = (state == nvl144pdbMainPowerOkConfig.polarity)
                                  ? Event::nvl144pdbMainPowerOkAssert
                                  : Event::nvl144pdbMainPowerOkDeAssert;
    this->sendPowerControlEvent(powerControlEvent, powerState);
}

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

