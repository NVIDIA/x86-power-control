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
ParsecPowerControl::ParsecPowerControl(boost::asio::io_context& ioContext,
                                       std::shared_ptr<sdbusplus::asio::connection> conn,
                                       const std::string& node)
    : VRPowerControl(ioContext, conn, node) // Call parent constructor (registers VR GPIOs)
{

    // call validateRequiredSignals() to validate all required signals
    validateRequiredSignals();
    
    auto it = powerSignalMap.find("GB300PDBMainPowerOk");
    it->second->gpioHandler = [this](bool state) { this->gb300pdbMainPowerOkHandler(state); };

    if(!requestGPIOEvents(*it->second))
    {
        lg2::error("Failed to register GPIO events for GB300 PDB Main Power OK");
        throw std::runtime_error("Parsec: Failed to register GB300 PDB Main Power OK GPIO events");
    }
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

void ParsecPowerControl::validateRequiredSignals()
{
    // Validate Parsec GB300 PDB signals (always required for Parsec platform)
    for (const auto& signalName : requiredSignals)
    {
        if (powerSignalMap.find(signalName) == powerSignalMap.end())
        {
            lg2::error("Required Parsec GB300 PDB signal '{SIGNAL}' not found in config", 
                      "SIGNAL", signalName);
            throw std::runtime_error("Parsec: Required GB300 PDB signal missing from config: " + signalName);
        }
    }
    
    // Call VRPowerControl to validate common VR signals
    VRPowerControl::validateRequiredSignals();
    
    lg2::info("Parsec signal validation complete");
}

void ParsecPowerControl::setGPIOsForHostStateOn()
{
    // TODO: Set Parsec GB300 PDB control GPIOs for host state "on"
    // - Assert GB300PDBMainPowerEnable
    
    lg2::info("Setting Parsec GPIOs for host state ON");
    
    // Call parent to set VR GPIOs
    VRPowerControl::setGPIOsForHostStateOn();
}

void ParsecPowerControl::setGPIOsForHostStateOff()
{
    // TODO: Set Parsec GB300 PDB control GPIOs for host state "off"
    // - De-assert GB300PDBMainPowerEnable
    
    lg2::info("Setting Parsec GPIOs for host state OFF");
    
    // Call parent to set VR GPIOs
    VRPowerControl::setGPIOsForHostStateOff();
}

} // namespace power_control

