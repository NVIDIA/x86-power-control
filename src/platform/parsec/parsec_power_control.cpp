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
}

namespace power_control
{

// Constructor: Adds Parsec-specific ConfigData entries to powerSignalMap
ParsecPowerControl::ParsecPowerControl(boost::asio::io_context& ioContext,
                                       std::shared_ptr<sdbusplus::asio::connection> conn,
                                       const std::string& configFilePath,
                                       const std::string& node,
                                       PersistentState& appState)
    : VRPowerControl(ioContext, conn, configFilePath, node, appState) // Call parent constructor (registers VR GPIOs)
{
    // powerSignalMap is now populated by base class PowerControl::loadConfigValues()
    // VR handlers already added to gpioHandlerMap by VRPowerControl constructor
    // Now add Parsec-specific handlers to the map

    // call validateRequiredSignals() to validate all required signals
    validateRequiredSignals();
    
    // Add Parsec-specific GPIO handler to the map
    gpioHandlerMap["GB300PDBMainPowerOk"] = [this](bool state) { this->gb300pdbMainPowerOkHandler(state); };

    // Register all GPIO handlers (from base, VR, and Parsec)
    registerGPIOHandlers();
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

std::function<void(Event)> ParsecPowerControl::getPowerStateHandler()
{
    // Parsec does not define new PowerState values, so delegate everything
    // to VRPowerControl which handles all VR and upstream states
    switch (powerState)
    {
        // TODO: Add Parsec-specific states here, that are overridden by ParsecPowerControl
        
        // Delegate all states to parent VRPowerControl
        default:
            return VRPowerControl::getPowerStateHandler();
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
    
    // Configure GB300PDBMainPowerOk for input event monitoring (gpio_keys_polled driver)
    auto gb300pdbMainPowerOkIt = powerSignalMap.find("GB300PDBMainPowerOk");
    if (gb300pdbMainPowerOkIt != powerSignalMap.end())
    {
        auto& configData = gb300pdbMainPowerOkIt->second;
        
        // Enable input event monitoring
        configData->useInputEvents = true;
        
        // Configure input event parameters
        InputEventConfig inputConfig;
        inputConfig.deviceName = "gpio_keys_gb300";
        inputConfig.signalName = "GB300_PDB_MAIN_PWR_OK_Mon";
        inputConfig.keyCode = 0x102;  // BTN_2 key code
        inputConfig.stateTracker = &gb300pdbMainPowerOkState;
        
        configData->inputEventConfig = inputConfig;
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

