/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#include "c2_power_control.hpp"

// External references to global variables from power_control.cpp
namespace power_control
{
    extern PowerState powerState;
    extern Context powerContext;
}

namespace power_control
{

// Constructor: Assigns handlers and registers events for C2-specific GPIOs
C2PowerControl::C2PowerControl(boost::asio::io_context& ioContext)
    : VRPowerControl(ioContext)  // Call parent constructor (registers VR GPIOs)
{
    // powerSignalMap is now populated by base class PowerControl::loadConfigValues()
    // VR handlers already assigned and registered by VRPowerControl constructor
    // Need to register C2 handlers for C2-specific signals
    
    // C2 PDB PSU Power OK
    if (auto it = powerSignalMap.find("C2PDBPSUPowerOk"); it != powerSignalMap.end())
    {
        if (powerContext.presence.c2_pdb)
        {
            it->second->gpioHandler = [this](bool state) { this->c2pdbPSUPowerOkHandler(state); };
            if (!requestGPIOEvents(*it->second))
            {
                lg2::error("Failed to register GPIO events for C2 PDB PSU Power OK");
                throw std::runtime_error("C2: Failed to register PDB PSU Power OK GPIO events");
            }
        }
    }
    else
    {
        // C2 requires this signal if PDB is present
        if (powerContext.presence.c2_pdb)
        {
            lg2::error("C2PDBPSUPowerOk not found in config");
            throw std::runtime_error("C2: Required signal C2PDBPSUPowerOk missing from config");
        }
    }
    
    // Note: C2PDB_Type no requestGPIOEvents call needed
}

// C2-SPECIFIC GPIO EVENT HANDLERS

void C2PowerControl::c2pdbPSUPowerOkHandler(bool state)
{
    // Lookup config for polarity (guaranteed to exist since handler was registered)
    auto& config = *powerSignalMap["C2PDBPSUPowerOk"];
    
    Event powerControlEvent = (state == config.polarity)
                                  ? Event::c2pdbPSUPowerOkAssert
                                  : Event::c2pdbPSUPowerOkDeAssert;
    this->sendPowerControlEvent(powerControlEvent, powerState);
}

std::function<void(Event)> C2PowerControl::getPowerStateHandler(PowerState state)
{

    switch (state)
    {
        // C2 modifies the handleWaitForHPMPowerGoodDeAssert and handlePowerStateOff handlers to handle the C2 PDB 12V rails.
        case PowerState::handleWaitForHPMPowerGoodDeAssert:
            return [this](Event e) { this->handleWaitForHPMPowerGoodDeAssert(e); };
        case PowerState::handlePowerStateOff:
            return [this](Event e) { this->handlePowerStateOff(e); };
        
        // TODO: Add any other C2-specific states here...

        // Delegate all states note overridden by C2 to parent VRPowerControl
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

