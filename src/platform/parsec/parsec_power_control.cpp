/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#include "parsec_power_control.hpp"

// External references to global variables/functions from power_control.cpp
namespace power_control
{
    // External global variables (runtime variables, not types)
    extern PowerState powerState;
    extern Context powerContext;
    
    // TODO: Implement the requestInputEvents function in the Base PowerControl class
    // extern bool requestInputEvents(
    //     const std::string& deviceName,
    //     const std::string& signalName,
    //     uint16_t keyCode,
    //     const std::function<void(bool)>& handler,
    //     boost::asio::posix::stream_descriptor& eventDescriptor,
    //     int* stateTracker
    // );
}

namespace power_control
{

// Constructor: Adds Parsec-specific ConfigData entries to powerSignalMap
ParsecPowerControl::ParsecPowerControl(boost::asio::io_context& ioContext)
    : VRPowerControl(ioContext) // Call parent constructor (registers VR GPIOs)
{

    // GB300 PDB Main Power OK
    if (auto it = powerSignalMap.find("GB300PDBMainPowerOk"); it != powerSignalMap.end())
    {
        // GB300 requires this signal if PDB is present
        if (powerContext.presence.gb300_pdb)
        {
            it->second->gpioHandler = [this](bool state) { this->gb300pdbMainPowerOkHandler(state); };
            
            // TODO: Call base class's requestInputEvents function to register the input event
        }
        else
        {
            lg2::error("GB300PDBMainPowerOk not found in config");
            throw std::runtime_error("Parsec: Required signal GB300PDBMainPowerOk missing from config");
        }
    }
    // powerSignalMap is now populated by base class PowerControl::loadConfigValues()
}

// PARSEC-SPECIFIC GPIO EVENT HANDLERS

void ParsecPowerControl::gb300pdbMainPowerOkHandler(bool state)
{
    // Lookup config for polarity (guaranteed to exist since handler was registered)
    auto& config = *powerSignalMap["GB300PDBMainPowerOk"];
    
    Event powerControlEvent = (state == config.polarity)
                                  ? Event::gb300pdbMainPowerOkAssert
                                  : Event::gb300pdbMainPowerOkDeAssert;
    this->sendPowerControlEvent(powerControlEvent, powerState);
}

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

