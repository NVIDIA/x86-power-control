/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#include "nvl144_power_control.hpp"

// External references to global variables from power_control.cpp
namespace power_control
{
    extern PowerState powerState;
}

namespace power_control
{

// Constructor: Assigns handlers and registers events for NVL144-specific GPIOs
NVL144PowerControl::NVL144PowerControl(boost::asio::io_context& ioContext, const std::string& configFilePath, std::string node = "0")
    : VRPowerControl(ioContext, configFilePath, node)  // Call parent constructor (registers VR GPIOs)
{
    // powerSignalMap is now populated by base class PowerControl::loadConfigValues()
    // VR handlers already added to gpioHandlerMap by VRPowerControl constructor
    // Now add NVL144-specific handlers to the map

    // call validateRequiredSignals() to validate all required signals
    validateRequiredSignals();

    // Add NVL144-specific GPIO handler to the map
    gpioHandlerMap["NVL144PDBMainPowerOk"] = [this](bool state) { this->nvl144pdbMainPowerOkHandler(state); };

    // Register all GPIO handlers (from base, VR, and NVL144)
    registerGPIOHandlers();
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
        case PowerState::off:
            return [this](Event e) { this->handlePowerStateOff(e); };
        case PowerState::on:
            return [this](Event e) { this->handlePowerStateOn(e); };
        case PowerState::waitForPDBMainPowerOk:
            return [this](Event e) { this->handleWaitForPDBMainPowerOk(e); };
        case PowerState::waitForPDBMainPowerOff:
            return [this](Event e) { this->handleWaitForPDBMainPowerOff(e); };
        case PowerState::waitForCPUResetAssert:
            return [this](Event e) { this->handleWaitForCPUResetAssert(e); };
        case PowerState::waitForHPMPowerGoodDeAssert:
            return [this](Event e) { this->handleWaitForHPMPowerGoodDeAssert(e); };
        // Add more as Power State Handlers are overrident and implemented by NVL144PowerControl
        default:
            return VRPowerControl::getPowerStateHandler(state);
    }
}

// ============================================================================
// HELPER FUNCTIONS for handlePowerStateOn
// ============================================================================

// Helper function: Handle shutdown requests (force or graceful)
void NVL144PowerControl::handleShutdownRequest(Event event)
{
    // Determine which shutdown signal to use based on event type
    const char* shutdownSignalName = (event == Event::powerOffRequest) ? 
        "Board0CpuShutdownForce" : "Board0CpuShutdownRequest";
    
    const char* shutdownType = (event == Event::powerOffRequest) ? 
        "Forceful" : "Graceful";
    
    const char* shutdownAction = (event == Event::powerOffRequest) ? 
        "Shutdown Force" : "Shutdown Request";
    
    lg2::info("{SHUTDOWN_TYPE} Power Off Request received. Commencing Host Main {SHUTDOWN_TYPE} Shutdown sequence.", 
              "SHUTDOWN_TYPE", shutdownType);

    auto board0RunPowerPG = powerSignalMap.find("Board0RunPowerPG");
    auto nvl144pdbMainPowerOk = powerSignalMap.find("NVL144PDBMainPowerOk");
    auto shutdownSignal = powerSignalMap.find(shutdownSignalName);

    // Check if power is already off
    if(board0RunPowerPG->second->gpioLine.get_value() == !board0RunPowerPG->second->polarity 
        && nvl144pdbMainPowerOk->second->gpioLine.get_value() == !nvl144pdbMainPowerOk->second->polarity)
    {
        lg2::info("PDB Main Power and HPM Run Power is already disabled. Setting GPIOs for host state OFF and transitioning to PowerState::Off");
        setGPIOsForHostStateOff(); // TODO: fill function implementation
        action = PowerAction::NONE;
        setPowerState(PowerState::off);
    }
    else
    {
        lg2::info("Asserting Board 0 CPU {SHUTDOWN_ACTION}. Starting CPU Shutdown OK Watchdog Timer. Transitioning to PowerState::waitForCPUShutdownOk",
                  "SHUTDOWN_ACTION", shutdownAction);
        setGPIOOutput(shutdownSignal->second, !shutdownSignal->second->polarity);
        startTimer(TimerMap["CpuShutdownOkWatchdogMs"], cpuShutdownOkWatchdogTimer, Event::cpuShutdownOkWatchdogTimerExpired);
        setPowerState(PowerState::waitForCPUShutdownOk);
    }
}

// ============================================================================
// handlePowerStateOn state handler
// ============================================================================

void NVL144PowerControl::handlePowerStateOn(Event event)
{
    // TODO: Move NVL144-specific powerStateOn() implementation here
    switch (event)
    {
        case Event::powerOffRequest:
        case Event::gracefulPowerOffRequest:
            handleShutdownRequest(event);
            break;
            
        case Event::powerCycleRequest:
            break;
        case Event::resetRequest:
            break;
        case Event::powerButtonPressed:
            break;
        case Event::gracefulPowerCycleRequest:
        default:
            lg2::info("No action taken.");
            break;
    }
}

// ============================================================================
// HELPER FUNCTIONS for handlePowerStateOff
// ============================================================================

// Helper function: Handle power on request
void NVL144PowerControl::handlePowerOnRequest()
{
    lg2::info("Power On Request received. Commencing Host Main Power On sequence.");

    auto board0RunPowerPG = powerSignalMap.find("Board0RunPowerPG");
    auto nvl144pdbMainPowerOk = powerSignalMap.find("NVL144PDBMainPowerOk");
    auto nvl144pdbMainPowerEnable = powerSignalMap.find("NVL144PDBMainPowerEnable");

    // Check if power is already on
    if(board0RunPowerPG->second->gpioLine.get_value() == board0RunPowerPG->second->polarity 
        && nvl144pdbMainPowerOk->second->gpioLine.get_value() == nvl144pdbMainPowerOk->second->polarity)
    {
        lg2::info("PDB Main Power and HPM Run Power is already enabled. Setting GPIOs for host state ON and transitioning to PowerState::On");
        setGPIOsForHostStateOn(); // TODO: fill function implementation
        action = PowerAction::NONE;
        setPowerState(PowerState::on);
    }
    else
    {
        lg2::info("Asserting NVL144 PDB Main Power Enable. Starting PDB Main Power OK Watchdog Timer. Transitioning to PowerState::waitForPDBMainPowerOk");
        setGPIOOutput(nvl144pdbMainPowerEnable->second, nvl144pdbMainPowerEnable->second->polarity);
        startTimer(TimerMap["PDBMainPowerOkWatchdogTimer"], pdbMainPowerOkWatchdogTimer, Event::pdbMainPowerOkWatchdogTimerExpired);
        setPowerState(PowerState::waitForPDBMainPowerOk);
    }
}

// ============================================================================
// handlePowerStateOff state handler
// ============================================================================

void NVL144PowerControl::handlePowerStateOff(Event event)
{
    // TODO: Move NVL144-specific powerStateOff() implementation here
    switch (event)
    {
        case Event::powerOnRequest:
            handlePowerOnRequest();
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

// ============================================================================
// HELPER FUNCTIONS for handleWaitForPDBMainPowerOk
// ============================================================================

// Helper function: Assert HPM board power sequence during power-on
void NVL144PowerControl::assertHPMBoardPowerSequence()
{
    auto board0RunPowerEnable = powerSignalMap.find("Board0RunPowerEnable");
    auto board0PreSystemReset = powerSignalMap.find("Board0PreSystemReset");
    auto usbPowerEnable = powerSignalMap.find("USBPowerEnable");
    auto e1sPowerEnable = powerSignalMap.find("E1SPowerEnable");
    auto bmcSSDReset = powerSignalMap.find("BMCSSDReset");

    // Assert Pre System Reset for Board 0 and Board 1 (if present)
    setGPIOOutput(board0PreSystemReset->second, board0PreSystemReset->second->polarity);
    if(boardPresence.board1Present) 
    {
        auto board1PreSystemReset = powerSignalMap.find("Board1PreSystemReset");
        setGPIOOutput(board1PreSystemReset->second, board1PreSystemReset->second->polarity);
    }

    // Assert peripheral power and de-assert BMC SSD Reset
    setGPIOOutput(e1sPowerEnable->second, e1sPowerEnable->second->polarity);
    setGPIOOutput(usbPowerEnable->second, usbPowerEnable->second->polarity);
    setGPIOOutput(bmcSSDReset->second, !bmcSSDReset->second->polarity); // de-assert BMC SSD Reset

    // Assert Run Power Enable for Board 0 and Board 1 (if present)
    setGPIOOutput(board0RunPowerEnable->second, board0RunPowerEnable->second->polarity);
    if(boardPresence.board1Present) 
    {
        auto board1RunPowerEnable = powerSignalMap.find("Board1RunPowerEnable");
        setGPIOOutput(board1RunPowerEnable->second, board1RunPowerEnable->second->polarity);
    }
}

// Helper function: Transition to HPM Power Good assert wait state
void NVL144PowerControl::transitionToHPMPowerGoodAssertState()
{
    pdbMainPowerOkWatchdogTimer.cancel();
    
    lg2::info("NVL144 PDB Main Power OK Asserted. Conducting HPM Board Power Sequencing. Asserting HPM Board Pre System Reset, E1S Power Enable, de-asserting BMC SDD Reset, and asserting Run Power Enable Lines. Starting HPM Power Good Watchdog Timer. Transitioning to PowerState::waitForHPMPowerGoodAssert.");
    
    assertHPMBoardPowerSequence();
    startTimer(TimerMap["HPMPowerGoodWatchdogTimer"], hpmPowerGoodWatchdogTimer, Event::hpmPowerGoodWatchdogTimerExpired);
    setPowerState(PowerState::waitForHPMPowerGoodAssert);
}

// ============================================================================
// handleWaitForPDBMainPowerOk state handler
// ============================================================================

void NVL144PowerControl::handleWaitForPDBMainPowerOk(Event event)
{
    switch (event)
    {
        case Event::nvl144pdbMainPowerOkAssert:
            transitionToHPMPowerGoodAssertState();
            break;
            
        case Event::pdbMainPowerOkWatchdogTimerExpired:
            lg2::error("PDB Main Power OK watchdog timer expired. PDB Main Power On Sequence Failed. Host Power On sequence failed. Conducting Cleanup Sequence: Setting GPIO states to match Host State OFF. Setting Host Power State to Off.");

            action = PowerAction::NONE;
            setGPIOsForHostStateOff(); // TODO: fill function implementation
            setPowerState(PowerState::off);
            break;
            
        default:
            lg2::info("No action taken.");
            break;
    }
}

// ============================================================================
// HELPER FUNCTIONS for handleWaitForPDBMainPowerOff
// ============================================================================

// Helper function: Complete shutdown and transition to off state
void NVL144PowerControl::completeShutdownAndTransitionToOff(bool success)
{
    pdbMainPowerOkWatchdogTimer.cancel();
    
    if (success)
    {
        // Log success based on shutdown type
        if(action == PowerAction::FORCE_OFF)
        {
            lg2::info("NVL144 PDB Main Power OK De-Asserted. Host Forceful Shutdown Sequence Completed Successfully. Transitioning to PowerState::off.");
        }
        else if(action == PowerAction::GRACE_OFF)
        {
            lg2::info("NVL144 PDB Main Power OK De-Asserted. Host Graceful Shutdown Sequence Completed Successfully. Transitioning to PowerState::off.");
        }
    }
    else
    {
        // Log failure
        lg2::error("PDB Main Power OK watchdog timer expired. PDB Main Power Off Sequence Failed. Host Power Off sequence failed. Conducting Cleanup Sequence: Setting GPIO states to match Host State OFF. Setting Host Power State to Off.");
    }

    action = PowerAction::NONE;
    setGPIOsForHostStateOff(); // TODO: fill function implementation
    setPowerState(PowerState::off);
}

// ============================================================================
// handleWaitForPDBMainPowerOff state handler
// ============================================================================

void NVL144PowerControl::handleWaitForPDBMainPowerOff(Event event)
{
    switch (event)
    {
        case Event::nvl144pdbMainPowerOkDeAssert:
            completeShutdownAndTransitionToOff(true);
            break;
            
        case Event::pdbMainPowerOkWatchdogTimerExpired:
            completeShutdownAndTransitionToOff(false);
            break;
            
        default:
            lg2::info("No action taken.");
            break;
    }
}

// ============================================================================
// HELPER FUNCTIONS for handleWaitForCPUResetAssert
// ============================================================================

// Helper function: De-assert HPM power and peripherals when CPUs are in reset
void NVL144PowerControl::deassertHPMPowerAndPeripherals()
{
    auto board0RunPowerEnable = powerSignalMap.find("Board0RunPowerEnable");
    auto e1sPowerEnable = powerSignalMap.find("E1SPowerEnable");
    auto usbPowerEnable = powerSignalMap.find("USBPowerEnable");
    auto bmcSSDReset = powerSignalMap.find("BMCSSDReset");

    // De-assert Board 0 Run Power Enable
    setGPIOOutput(board0RunPowerEnable->second, !board0RunPowerEnable->second->polarity);
    
    // De-assert Board 1 Run Power Enable if present
    if (boardPresence.board1Present)
    {
        auto board1RunPowerEnable = powerSignalMap.find("Board1RunPowerEnable");
        setGPIOOutput(board1RunPowerEnable->second, !board1RunPowerEnable->second->polarity);
    }

    // De-assert peripheral power and assert BMC SSD Reset
    setGPIOOutput(e1sPowerEnable->second, !e1sPowerEnable->second->polarity);
    setGPIOOutput(usbPowerEnable->second, !usbPowerEnable->second->polarity);
    setGPIOOutput(bmcSSDReset->second, bmcSSDReset->second->polarity);
}

// Helper function: Transition to HPM Power Good de-assert wait state
void NVL144PowerControl::transitionToHPMPowerGoodDeAssertState()
{
    cpuResetWatchdogTimer.cancel();
    
    lg2::info("CPU Reset Indicator Asserted. CPUs are in reset. De-asserting Run Power Enable, E1S Power Enable, USB Power Enable, and asserting BMC SSD Reset lines. Starting HPM Power Good Watchdog Timer. Transitioning to PowerState::waitForHPMPowerGoodDeAssert.");
    
    deassertHPMPowerAndPeripherals();
    startTimer(TimerMap["HPMPowerGoodWatchdogTimer"], hpmPowerGoodWatchdogTimer, Event::hpmPowerGoodWatchdogTimerExpired);
    setPowerState(PowerState::waitForHPMPowerGoodDeAssert);
}

// ============================================================================
// handleWaitForCPUResetAssert state handler
// ============================================================================

void NVL144PowerControl::handleWaitForCPUResetAssert(Event event)
{
    switch (event)
    {
        case Event::cpuResetIndicatorAssert:
            transitionToHPMPowerGoodDeAssertState();
            break;
            
        case Event::cpuResetWatchdogTimerExpired:
            lg2::error("CPU Reset Watchdog Timer Expired. CPUs are not in reset. Host Shutdown sequence failed {reccomend checking CPLD  status}. Conducting Cleanup Sequence: Setting GPIO states to match Host State OFF. Setting Host Power State to Off.");

            action = PowerAction::NONE;
            setGPIOsForHostStateOff(); // TODO: fill function implementation
            setPowerState(PowerState::off);
            break;
            
        default:
            lg2::info("No action taken.");
            break;
    }
}

// ============================================================================
// HELPER FUNCTIONS for handleWaitForHPMPowerGoodDeAssert
// ============================================================================

// Helper function: De-assert Pre System Resets and PDB Main Power
void NVL144PowerControl::deassertPreSystemResetsAndPDBMainPower()
{
    auto board0PreSystemReset = powerSignalMap.find("Board0PreSystemReset");
    auto nvl144pdbMainPowerEnable = powerSignalMap.find("NVL144PDBMainPowerEnable");

    // De-assert Board 0 Pre System Reset
    setGPIOOutput(board0PreSystemReset->second, !board0PreSystemReset->second->polarity);
    
    // De-assert Board 1 Pre System Reset if present
    if (boardPresence.board1Present)
    {
        auto board1PreSystemReset = powerSignalMap.find("Board1PreSystemReset");
        setGPIOOutput(board1PreSystemReset->second, !board1PreSystemReset->second->polarity);
    }

    // De-assert NVL144 PDB Main Power Enable
    setGPIOOutput(nvl144pdbMainPowerEnable->second, !nvl144pdbMainPowerEnable->second->polarity);
}

// Helper function: Transition to PDB Main Power Off wait state
void NVL144PowerControl::transitionToPDBMainPowerOffState()
{
    hpmPowerGoodWatchdogTimer.cancel();
    
    lg2::info("HPM Board 0 Run Power Good de-asserted. De-asserting Pre System Reset lines. De-asserting NVL144 PDB Main Power Enable, Starting PDB Main Power OK Watchdog Timer. Transitioning to PowerState::waitForPDBMainPowerOff.");
    
    deassertPreSystemResetsAndPDBMainPower();
    startTimer(TimerMap["PDBMainPowerOkWatchdogTimer"], pdbMainPowerOkWatchdogTimer, Event::pdbMainPowerOkWatchdogTimerExpired);
    setPowerState(PowerState::waitForPDBMainPowerOff);
}

// ============================================================================
// handleWaitForHPMPowerGoodDeAssert state handler
// ============================================================================

void NVL144PowerControl::handleWaitForHPMPowerGoodDeAssert(Event event)
{
    switch (event)
    {
        case Event::board0RunPowerPGDeAssert:
            transitionToPDBMainPowerOffState();
            break;
            
        case Event::hpmPowerGoodWatchdogTimerExpired:
            // TODO: determine if this is the correct fault handling for No Run Power Good De-assertion during shutdown sequences
            lg2::error("HPM Power Good Watchdog Timer Expired. Host Forceful Shutdown sequence failed! Conducting Cleanup Sequence: Setting GPIO states to match Host State ON. Setting Host Power State to On.");

            action = PowerAction::NONE;
            setGPIOsForHostStateOn(); // TODO: fill function implementation
            setPowerState(PowerState::on);
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

