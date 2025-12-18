/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#include "nvl144_power_control.hpp"

// External references to global variables from power_control.cpp
namespace power_control
{
    extern PowerState powerState;
    extern Context powerContext;
}

namespace power_control
{

// Constructor: Assigns handlers and registers events for NVL144-specific GPIOs
NVL144PowerControl::NVL144PowerControl(boost::asio::io_context& ioContext)
    : VRPowerControl(ioContext)  // Call parent constructor (registers VR GPIOs)
{
    // powerSignalMap is now populated by base class PowerControl::loadConfigValues()
    // VR handlers already assigned and registered by VRPowerControl constructor
    // Need to register NVL144 handlers for NVL144-specific signals
    
    // NVL144 PDB Main Power OK
    if (auto it = powerSignalMap.find("NVL144PDBMainPowerOk"); it != powerSignalMap.end())
    {
        if (powerContext.presence.nvl144_pdb)
        {
            it->second->gpioHandler = [this](bool state) { this->nvl144pdbMainPowerOkHandler(state); };
            if (!requestGPIOEvents(*it->second))
            {
                lg2::error("Failed to register GPIO events for NVL144 PDB Main Power OK");
                throw std::runtime_error("NVL144: Failed to register PDB Main Power OK GPIO events");
            }
        }
    }
    else
    {
        // NVL144 requires this signal if PDB is present
        if (powerContext.presence.nvl144_pdb)
        {
            lg2::error("NVL144PDBMainPowerOk not found in config");
            throw std::runtime_error("NVL144: Required signal NVL144PDBMainPowerOk missing from config");
        }
    }
}

// NVL144-specific GPIO handler implementations
void NVL144PowerControl::nvl144pdbMainPowerOkHandler(bool state)
{
    // Lookup config for polarity (guaranteed to exist since handler was registered)
    auto& config = *powerSignalMap["NVL144PDBMainPowerOk"];
    
    Event powerControlEvent = (state == config.polarity)
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

