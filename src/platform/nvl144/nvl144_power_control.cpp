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
NVL144PowerControl::NVL144PowerControl(boost::asio::io_context& ioContext,
                                       std::shared_ptr<sdbusplus::asio::connection> conn,
                                       const std::string& node)
    : VRPowerControl(ioContext, conn, node)  // Call parent constructor (registers VR GPIOs)
{
    // powerSignalMap is now populated by base class PowerControl::loadConfigValues()
    // VR handlers already assigned and registered by VRPowerControl constructor
    // Need to register NVL144 handlers for NVL144-specific signals

    // call validateRequiredSignals() to validate all required signals
    validateRequiredSignals();

    auto it = powerSignalMap.find("NVL144PDBMainPowerOk");
    it->second->gpioHandler = [this](bool state) { this->nvl144pdbMainPowerOkHandler(state); };

    if(!requestGPIOEvents(*it->second))
    {
        lg2::error("Failed to register GPIO events for NVL144 PDB Main Power OK");
        throw std::runtime_error("NVL144: Failed to register PDB Main Power OK GPIO events");
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

void NVL144PowerControl::handlePowerStateOff(Event event)
{
    // TODO: Move NVL144-specific powerStateOff() implementation here
    switch (event)
    {
        case Event::powerOnRequest:
            // Determine if 1P or 2P sequencing being performed
            // log power on request received
            // check if Run Power is already enabled (PDB Main Power OK & HPM Board 0 Run Power Good & HPM Board 1 Run Power Good are ALL asserted)
            //  if not, enable NVL144 PDB Main Power Enable, transition to PowerState::waitForPDBMainPowerOk
            //  if yes, log that Run Power is already enabled, and transition to PowerState::On
            auto board0RunPowerPG = powerSignalMap.find("Board0RunPowerPG");
            auto nvl144pdbMainPowerOk = powerSignalMap.find("NVL144PDBMainPowerOk");
            auto nvl144pdbMainPowerEnable = powerSignalMap.find("NVL144PDBMainPowerEnable");

            if(board0RunPowerPG->second->gpioLine.get_value() == board0RunPowerPG->second->polarity 
                && nvl144pdbMainPowerOk->second->gpioLine.get_value() == nvl144pdbMainPowerOk->second->polarity)
            {
                lg2::info("PDB Main Power and HPM Run Power is already enabled. Setting GPIOs for host state ON and transitioning to PowerState::On");
                setGPIOsForHostStateOn(); // TODO: fill function implementation
                powerContext.action = PowerAction::NONE; // TODO: replace with Aushim's implementation for tracking which power action is in effect
                
                setPowerState(PowerState::on);
            }
            else
            {
                lg2::info("Asserting NVL144 PDB Main Power Enable. Starting PDB Main Power OK Watchdog Timer. Transitioning to PowerState::waitForPDBMainPowerOk");

                setGPIOOutput(nvl144pdbMainPowerEnable->second, nvl144pdbMainPowerEnable->second->polarity);
                startTimer(TimerMap["PDBMainPowerOkWatchdogTimer"], pdbMainPowerOkWatchdogTimer, Event::pdbMainPowerOkWatchdogTimerExpired);
                setPowerState(PowerState::waitForPDBMainPowerOk);
            }
            break;
        case Event::powerCycleRequest:
            break;
        case Event::powerButtonPressed:
            break;
        case Event::resetRequest:
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

void NVL144PowerControl::handleWaitForPDBMainPowerOk(Event event)
{
    switch (event)
    {
        case Event::nvl144pdbMainPowerOkAssert:
            pdbMainPowerOkWatchdogTimer.cancel(); // Cancel the PDB Main Power OK watchdog timer
            lg2::info("NVL144 PDB Main Power OK Asserted. Conducting HPM Board Power Sequencing. Asserting HPM Board Pre System Reset, E1S Power Enable, de-asserting BMC SDD Reset, and asserting Run Power Enable Lines. Starting HPM Power Good Watchdog Timer. Transitioning to PowerState::waitForHPMPowerGoodAssert.");

            auto board0RunPowerEnable = powerSignalMap.find("Board0RunPowerEnable");
            auto board0PreSystemReset = powerSignalMap.find("Board0PreSystemReset");
            auto usbPowerEnable = powerSignalMap.find("USBPowerEnable");
            auto e1sPowerEnable = powerSignalMap.find("E1SPowerEnable");
            auto bmcSSDReset = powerSignalMap.find("BMCSSDReset");

            std::map<std::string, std::shared_ptr<ConfigData>>::iterator board1RunPowerEnable;
            std::map<std::string, std::shared_ptr<ConfigData>>::iterator board1PreSystemReset;

            if (boardPresence.board1Present){
                board1RunPowerEnable = powerSignalMap.find("Board1RunPowerEnable");
                board1PreSystemReset = powerSignalMap.find("Board1PreSystemReset");
            }

            setGPIOOutput(board0PreSystemReset->second, board0PreSystemReset->second->polarity);
            if(boardPresence.board1Present) 
            {
                setGPIOOutput(board1PreSystemReset->second, board1PreSystemReset->second->polarity);
            }

            
            setGPIOOutput(e1sPowerEnable->second, e1sPowerEnable->second->polarity);
            setGPIOOutput(usbPowerEnable->second, usbPowerEnable->second->polarity);
            setGPIOOutput(bmcSSDReset->second, !bmcSSDReset->second->polarity); // de-assert BMC SDD Reset

            setGPIOOutput(board0RunPowerEnable->second, board0RunPowerEnable->second->polarity);
            if(boardPresence.board1Present) 
            {
                setGPIOOutput(board1RunPowerEnable->second, board1RunPowerEnable->second->polarity);
            }

            startTimer(TimerMap["HPMPowerGoodWatchdogTimer"], hpmPowerGoodWatchdogTimer, Event::hpmPowerGoodWatchdogTimerExpired);
            setPowerState(PowerState::waitForHPMPowerGoodAssert);
           
            break;
        case Event::pdbMainPowerOkWatchdogTimerExpired:
            lg2::error("PDB Main Power OK watchdog timer expired. PDB Main Power On Sequence Failed. Host Power On sequence failed. Conducting Cleanup Sequence: Setting GPIO states to match Host State OFF. Setting Host Power State to Off.");

            powerContext.action = PowerAction::NONE; // TODO: replace with Aushim's implementation for tracking which power action is in effect
            setGPIOsForHostStateOff(); // TODO: fill function implementation

            setPowerState(PowerState::off);
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

void NVL144PowerControl::validateRequiredSignals()
{
    // Validate NVL144 PDB signals (always required for NVL144 platform)
    for (const auto& signalName : requiredSignals)
    {
        if (powerSignalMap.find(signalName) == powerSignalMap.end())
        {
            lg2::error("Required NVL144 PDB signal '{SIGNAL}' not found in config", 
                      "SIGNAL", signalName);
            throw std::runtime_error("NVL144: Required PDB signal missing from config: " + signalName);
        }
    }
    
    // Call VRPowerControl to validate common VR signals
    VRPowerControl::validateRequiredSignals();
    
    lg2::info("NVL144 signal validation complete");
}

void NVL144PowerControl::setGPIOsForHostStateOn()
{
    // TODO: Set NVL144 PDB control GPIOs for host state "on"
    // - Assert NVL144PDBMainPowerEnable
    // - Assert E1S Power Enable (if applicable)
    // - Assert BMC SSD Reset (if applicable)
    
    lg2::info("Setting NVL144 GPIOs for host state ON");
    
    // Call parent to set VR GPIOs
    VRPowerControl::setGPIOsForHostStateOn();
}

void NVL144PowerControl::setGPIOsForHostStateOff()
{
    // TODO: Set NVL144 PDB control GPIOs for host state "off"
    // - De-assert NVL144PDBMainPowerEnable
    // - De-assert E1S Power Enable (if applicable)
    // - De-assert BMC SSD Reset (if applicable)
    
    lg2::info("Setting NVL144 GPIOs for host state OFF");
    
    // Call parent to set VR GPIOs
    VRPowerControl::setGPIOsForHostStateOff();
}

} // namespace power_control

