/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#include "nvl144_power_control.hpp"

#include <phosphor-logging/lg2.hpp>

// External references to global variables from power_control.cpp
namespace power_control
{
extern PowerState powerState;
}

namespace power_control
{
// Type aliases for convenience
using Event = PowerControl::Event;

// Constructor: Assigns handlers and registers events for NVL144-specific GPIOs
NVL144PowerControl::NVL144PowerControl(
    boost::asio::io_context& ioContext,
    std::shared_ptr<sdbusplus::asio::connection> conn,
    const std::string& configFilePath, const std::string& node,
    PersistentState& appState) :
    VRPowerControl(ioContext, conn, configFilePath, node,
                   appState), // Call parent constructor (registers VR GPIOs)
    pdbMainPowerOkWatchdogTimer(ioContext)
{
    // powerSignalMap is now populated by base class
    // PowerControl::loadConfigValues() VR handlers already added to
    // gpioHandlerMap by VRPowerControl constructor Now add NVL144-specific
    // handlers to the map

    // call validateRequiredSignals() to validate all required signals
    validateRequiredSignals();

    // call validateTimerConfigs() to validate all required timers
    validateTimerConfigs();

    // call setDefaultValues() to set the default values for output signals
    setDefaultValues();

    // Add NVL144-specific GPIO handler to the map
    gpioHandlerMap["NVL144PDBMainPowerOk"] = [this](bool state) {
        this->nvl144pdbMainPowerOkHandler(state);
    };

    // Register all GPIO handlers (from base, VR, and NVL144)
    registerGPIOHandlers();
}

// NVL144-specific GPIO handler implementations
void NVL144PowerControl::nvl144pdbMainPowerOkHandler(bool state)
{
    auto it = powerSignalMap.find("NVL144PDBMainPowerOk");
    if (it == powerSignalMap.end())
    {
        throw std::runtime_error(
            "NVL144PDBMainPowerOk signal not found in powerSignalMap");
    }

    auto& config = *it->second;
    Event powerControlEvent = (state == config.polarity)
                                  ? Event::nvl144pdbMainPowerOkAssert
                                  : Event::nvl144pdbMainPowerOkDeAssert;
    this->sendPowerControlEvent(powerControlEvent);
}

std::function<void(Event)> NVL144PowerControl::getPowerStateHandler()
{
    // NVL144 does not define new PowerState values, so delegate everything
    // to VRPowerControl which handles all VR and upstream states
    switch (powerState)
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
            return
                [this](Event e) { this->handleWaitForHPMPowerGoodDeAssert(e); };
        // Add more as Power State Handlers are overridden and implemented by
        // NVL144PowerControl
        default:
            return VRPowerControl::getPowerStateHandler();
    }
}

// ============================================================================
// HELPER FUNCTIONS for handlePowerStateOn
// ============================================================================

// Helper function: Handle shutdown requests (force or graceful)
void NVL144PowerControl::handleShutdownRequest(Event event)
{
    // Determine which shutdown signal to use based on event type
    const char* shutdownSignalName = (event == Event::powerOffRequest)
                                         ? "Board0CpuShutdownForce"
                                         : "Board0CpuShutdownRequest";

    const char* shutdownType =
        (event == Event::powerOffRequest) ? "Forceful" : "Graceful";

    int shutdownOkTimeout = event == Event::powerOffRequest ? TimerMap["ForcefulCpuShutdownOkWatchdogMs"] : TimerMap["GracefulCpuShutdownOkWatchdogMs"];

    const char* shutdownAction = (event == Event::powerOffRequest)
                                     ? "Shutdown Force"
                                     : "Shutdown Request";

    // Only set action if it's not already POWER_CYCLE (to preserve power cycle context)
    if (action != PowerAction::POWER_CYCLE)
    {
        action = (event == Event::powerOffRequest) ? PowerAction::FORCE_OFF
                                                    : PowerAction::GRACE_OFF;
    }

    lg2::info(
        "{SHUTDOWN_TYPE} Power Off Request received. Commencing Host Main {SHUTDOWN_TYPE} Shutdown sequence.",
        "SHUTDOWN_TYPE", shutdownType);

    auto board0RunPowerPG = powerSignalMap.find("Board0RunPowerPG");
    if (board0RunPowerPG == powerSignalMap.end())
    {
        throw std::runtime_error(
            "Board0RunPowerPG signal not found in powerSignalMap");
    }

    auto nvl144pdbMainPowerOk = powerSignalMap.find("NVL144PDBMainPowerOk");
    if (nvl144pdbMainPowerOk == powerSignalMap.end())
    {
        throw std::runtime_error(
            "NVL144PDBMainPowerOk signal not found in powerSignalMap");
    }

    auto shutdownSignal = powerSignalMap.find(shutdownSignalName);
    if (shutdownSignal == powerSignalMap.end())
    {
        throw std::runtime_error(
            std::string(shutdownSignalName) + " signal not found in powerSignalMap");
    }

    // Check if power is already off
    if (board0RunPowerPG->second->gpioLine.get_value() ==
            !board0RunPowerPG->second->polarity &&
        nvl144pdbMainPowerOk->second->gpioLine.get_value() ==
            !nvl144pdbMainPowerOk->second->polarity)
    {
        lg2::info(
            "PDB Main Power and HPM Run Power is already disabled. Setting GPIOs for host state OFF and transitioning to PowerState::Off");
        setGPIOsForHostStateOff();
        action = PowerAction::NONE;
        setPowerState(PowerState::off);
    }
    else
    {
        lg2::info(
            "Asserting Board 0 CPU {SHUTDOWN_ACTION}. Starting CPU Shutdown OK Watchdog Timer. Transitioning to PowerState::waitForCPUShutdownOk",
            "SHUTDOWN_ACTION", shutdownAction);
        setGPIOOutput(shutdownSignal->second,
                      !shutdownSignal->second->polarity);
        startTimer(shutdownOkTimeout,
                   cpuShutdownOkWatchdogTimer,
                   Event::cpuShutdownOkWatchdogTimerExpired);
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
        // case Event::board0ShutdownOkAsserted
        // case Event::board1ShutdownOkAsserted:
            // call method for 
            // log host initiated shutdown request received
            // assert pre system reset
            // de-assert run power enables, USB Power Enable, E1S Power Enable, BMC SSD Reset
            // 
            // check CPU Boot Done assertion (only accept when OS is booted).
            // host initiated shutdown request
        case Event::powerOffRequest:
        case Event::gracefulPowerOffRequest:
            handleShutdownRequest(event);
            break;

        case Event::powerCycleRequest:
            lg2::info("Forceful Power Cycle Request received. Initiating forceful shutdown");
            action = PowerAction::POWER_CYCLE;
            handleShutdownRequest(Event::powerOffRequest);  // Reuse forceful shutdown
            break;
        case Event::resetRequest:
            break;
        case Event::powerButtonPressed:
            break;
        case Event::gracefulPowerCycleRequest:
            // handleGracefulPowerCycleRequest(event);
            break;
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
    lg2::info(
        "Power On Request received. Commencing Host Main Power On sequence.");

    auto board0RunPowerPG = powerSignalMap.find("Board0RunPowerPG");
    if (board0RunPowerPG == powerSignalMap.end())
    {
        throw std::runtime_error(
            "Board0RunPowerPG signal not found in powerSignalMap");
    }

    auto nvl144pdbMainPowerOk = powerSignalMap.find("NVL144PDBMainPowerOk");
    if (nvl144pdbMainPowerOk == powerSignalMap.end())
    {
        throw std::runtime_error(
            "NVL144PDBMainPowerOk signal not found in powerSignalMap");
    }

    auto nvl144pdbMainPowerEnable =
        powerSignalMap.find("NVL144PDBMainPowerEnable");
    if (nvl144pdbMainPowerEnable == powerSignalMap.end())
    {
        throw std::runtime_error(
            "NVL144PDBMainPowerEnable signal not found in powerSignalMap");
    }

    // Check if power is already on
    if (board0RunPowerPG->second->gpioLine.get_value() ==
            board0RunPowerPG->second->polarity &&
        nvl144pdbMainPowerOk->second->gpioLine.get_value() ==
            nvl144pdbMainPowerOk->second->polarity)
    {
        lg2::info(
            "PDB Main Power and HPM Run Power is already enabled. Setting GPIOs for host state ON and transitioning to PowerState::On");
        setGPIOsForHostStateOn(); // TODO: fill function implementation
        action = PowerAction::NONE;
        setPowerState(PowerState::on);
    }
    else
    {
        lg2::info(
            "Asserting NVL144 PDB Main Power Enable. Starting PDB Main Power OK Watchdog Timer. Transitioning to PowerState::waitForPDBMainPowerOk");
        action = PowerAction::POWER_ON;
        setGPIOOutput(nvl144pdbMainPowerEnable->second,
                      nvl144pdbMainPowerEnable->second->polarity);
        startTimer(TimerMap["NVL144PdbMainPowerOkWatchdogMs"],
                   pdbMainPowerOkWatchdogTimer,
                   Event::pdbMainPowerOkWatchdogTimerExpired);
        setPowerState(PowerState::waitForPDBMainPowerOk);
    }
}

// Helper function: Handle power cycle request when in off state
void NVL144PowerControl::handlePowerCycleWhenOff()
{
    lg2::info("Power Cycle Request received while in off state");

    auto board0RunPowerPG = powerSignalMap.find("Board0RunPowerPG");
    if (board0RunPowerPG == powerSignalMap.end())
    {
        throw std::runtime_error(
            "Board0RunPowerPG signal not found in powerSignalMap");
    }

    // Verify power is actually off
    if (board0RunPowerPG->second->gpioLine.get_value() ==
        !board0RunPowerPG->second->polarity)
    {
        lg2::info("Verified Board 0 Run Power PG is de-asserted. Initiating Host Power On sequence");
        action = PowerAction::POWER_CYCLE;
        handlePowerOnRequest();
    }
    else
    {
        lg2::warning(
            "Power cycle requested but Board 0 Run Power PG is not de-asserted. Initiating Host Forceful Shutdown first");
        action = PowerAction::POWER_CYCLE;
        setPowerState(PowerState::on);
        handleShutdownRequest(Event::powerOffRequest);
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
            // Power cycle requested when already off
            handlePowerCycleWhenOff();
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
    if (board0RunPowerEnable == powerSignalMap.end())
    {
        throw std::runtime_error(
            "Board0RunPowerEnable signal not found in powerSignalMap");
    }

    auto board0PreSystemReset = powerSignalMap.find("Board0PreSystemReset");
    if (board0PreSystemReset == powerSignalMap.end())
    {
        throw std::runtime_error(
            "Board0PreSystemReset signal not found in powerSignalMap");
    }

    auto usbPowerEnable = powerSignalMap.find("USBPowerEnable");
    if (usbPowerEnable == powerSignalMap.end())
    {
        throw std::runtime_error(
            "USBPowerEnable signal not found in powerSignalMap");
    }

    auto e1sPowerEnable = powerSignalMap.find("E1SPowerEnable");
    if (e1sPowerEnable == powerSignalMap.end())
    {
        throw std::runtime_error(
            "E1SPowerEnable signal not found in powerSignalMap");
    }

    auto bmcSSDReset = powerSignalMap.find("BMCSSDReset");
    if (bmcSSDReset == powerSignalMap.end())
    {
        throw std::runtime_error(
            "BMCSSDReset signal not found in powerSignalMap");
    }

    // Assert Pre System Reset for Board 0 and Board 1 (if present)
    setGPIOOutput(board0PreSystemReset->second,
                  board0PreSystemReset->second->polarity);
    if (boardPresence.board1Present)
    {
        auto board1PreSystemReset = powerSignalMap.find("Board1PreSystemReset");
        if (board1PreSystemReset == powerSignalMap.end())
        {
            throw std::runtime_error(
                "Board1PreSystemReset signal not found in powerSignalMap");
        }
        setGPIOOutput(board1PreSystemReset->second,
                      board1PreSystemReset->second->polarity);
    }

    // Assert peripheral power and de-assert BMC SSD Reset
    setGPIOOutput(e1sPowerEnable->second, e1sPowerEnable->second->polarity);
    setGPIOOutput(usbPowerEnable->second, usbPowerEnable->second->polarity);
    setGPIOOutput(bmcSSDReset->second,
                  !bmcSSDReset->second->polarity); // de-assert BMC SSD Reset

    // Assert Run Power Enable for Board 0 and Board 1 (if present)
    setGPIOOutput(board0RunPowerEnable->second,
                  board0RunPowerEnable->second->polarity);
    if (boardPresence.board1Present)
    {
        auto board1RunPowerEnable = powerSignalMap.find("Board1RunPowerEnable");
        if (board1RunPowerEnable == powerSignalMap.end())
        {
            throw std::runtime_error(
                "Board1RunPowerEnable signal not found in powerSignalMap");
        }
        setGPIOOutput(board1RunPowerEnable->second,
                      board1RunPowerEnable->second->polarity);
    }
}

// Helper function: Transition to HPM Power Good assert wait state
void NVL144PowerControl::transitionToHPMPowerGoodAssertState()
{
    pdbMainPowerOkWatchdogTimer.cancel();

    lg2::info(
        "NVL144 PDB Main Power OK Asserted. Conducting HPM Board Power Sequencing. Asserting HPM Board Pre System Reset, E1S Power Enable, de-asserting BMC SDD Reset, and asserting Run Power Enable Lines. Starting HPM Power Good Watchdog Timer. Transitioning to PowerState::waitForHPMPowerGoodAssert.");

    assertHPMBoardPowerSequence();
    startTimer(TimerMap["HPMPowerGoodWatchdogMs"], hpmPowerGoodWatchdogTimer,
               Event::hpmPowerGoodWatchdogTimerExpired);
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
            lg2::error(
                "PDB Main Power OK watchdog timer expired. PDB Main Power On Sequence Failed. Host Power On sequence failed. Conducting Cleanup Sequence: Setting GPIO states to match Host State OFF. Setting Host Power State to Off.");

            action = PowerAction::NONE;
            setGPIOsForHostStateOff();
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
        if (action == PowerAction::FORCE_OFF)
        {
            lg2::info(
                "NVL144 PDB Main Power OK De-Asserted. Host Forceful Shutdown Sequence Completed Successfully. Transitioning to PowerState::off.");
            action = PowerAction::NONE;
            setGPIOsForHostStateOff();
            setPowerState(PowerState::off);
        }
        else if (action == PowerAction::GRACE_OFF)
        {
            lg2::info(
                "NVL144 PDB Main Power OK De-Asserted. Host Graceful Shutdown Sequence Completed Successfully. Transitioning to PowerState::off.");
            action = PowerAction::NONE;
            setGPIOsForHostStateOff();
            setPowerState(PowerState::off);
        }
        else if (action == PowerAction::POWER_CYCLE)
        {
            lg2::info(
                "NVL144 PDB Main Power OK De-Asserted. Forceful Power Cycle Host Forceful Shutdown complete. Starting power cycle delay timer. Transitioning to PowerState::waitForPowerCycleDelay.");
            // Keep action = POWER_CYCLE (don't clear it)
            setGPIOsForHostStateOff();
            startTimer(TimerMap["PowerCycleDelayMs"], powerCycleDelayTimer,
                       Event::powerCycleDelayTimerExpired);
            setPowerState(PowerState::waitForPowerCycleDelay);
        }
        else
        {
            // Unknown action - default to off
            lg2::warning("Shutdown complete with unexpected action. Transitioning to off.");
            action = PowerAction::NONE;
            setGPIOsForHostStateOff();
            setPowerState(PowerState::off);
        }
    }
    else
    {
        // Log failure - abort any ongoing action
        lg2::error(
            "PDB Main Power OK watchdog timer expired. PDB Main Power Off Sequence Failed. Host Power Off sequence failed. Conducting Cleanup Sequence: Setting GPIO states to match Host State OFF. Setting Host Power State to Off.");
        action = PowerAction::NONE;
        setGPIOsForHostStateOff();
        setPowerState(PowerState::off);
    }
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
    if (board0RunPowerEnable == powerSignalMap.end())
    {
        throw std::runtime_error(
            "Board0RunPowerEnable signal not found in powerSignalMap");
    }

    auto e1sPowerEnable = powerSignalMap.find("E1SPowerEnable");
    if (e1sPowerEnable == powerSignalMap.end())
    {
        throw std::runtime_error(
            "E1SPowerEnable signal not found in powerSignalMap");
    }

    auto usbPowerEnable = powerSignalMap.find("USBPowerEnable");
    if (usbPowerEnable == powerSignalMap.end())
    {
        throw std::runtime_error(
            "USBPowerEnable signal not found in powerSignalMap");
    }

    auto bmcSSDReset = powerSignalMap.find("BMCSSDReset");
    if (bmcSSDReset == powerSignalMap.end())
    {
        throw std::runtime_error(
            "BMCSSDReset signal not found in powerSignalMap");
    }

    // De-assert Board 0 Run Power Enable
    setGPIOOutput(board0RunPowerEnable->second,
                  !board0RunPowerEnable->second->polarity);

    // De-assert Board 1 Run Power Enable if present
    if (boardPresence.board1Present)
    {
        auto board1RunPowerEnable = powerSignalMap.find("Board1RunPowerEnable");
        if (board1RunPowerEnable == powerSignalMap.end())
        {
            throw std::runtime_error(
                "Board1RunPowerEnable signal not found in powerSignalMap");
        }
        setGPIOOutput(board1RunPowerEnable->second,
                      !board1RunPowerEnable->second->polarity);
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

    lg2::info(
        "CPU Reset Indicator Asserted. CPUs are in reset. De-asserting Run Power Enable, E1S Power Enable, USB Power Enable, and asserting BMC SSD Reset lines. Starting HPM Power Good Watchdog Timer. Transitioning to PowerState::waitForHPMPowerGoodDeAssert.");

    deassertHPMPowerAndPeripherals();
    startTimer(TimerMap["HPMPowerGoodWatchdogMs"], hpmPowerGoodWatchdogTimer,
               Event::hpmPowerGoodWatchdogTimerExpired);
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
            lg2::error(
                "CPU Reset Watchdog Timer Expired. CPUs are not in reset. Host Shutdown sequence failed {reccomend checking CPLD  status}. Conducting Cleanup Sequence: Setting GPIO states to match Host State OFF. Setting Host Power State to Off.");

            action = PowerAction::NONE;
            setGPIOsForHostStateOff();
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
    if (board0PreSystemReset == powerSignalMap.end())
    {
        throw std::runtime_error(
            "Board0PreSystemReset signal not found in powerSignalMap");
    }

    auto nvl144pdbMainPowerEnable =
        powerSignalMap.find("NVL144PDBMainPowerEnable");
    if (nvl144pdbMainPowerEnable == powerSignalMap.end())
    {
        throw std::runtime_error(
            "NVL144PDBMainPowerEnable signal not found in powerSignalMap");
    }

    // De-assert Board 0 Pre System Reset
    setGPIOOutput(board0PreSystemReset->second,
                  !board0PreSystemReset->second->polarity);

    // De-assert Board 1 Pre System Reset if present
    if (boardPresence.board1Present)
    {
        auto board1PreSystemReset = powerSignalMap.find("Board1PreSystemReset");
        if (board1PreSystemReset == powerSignalMap.end())
        {
            throw std::runtime_error(
                "Board1PreSystemReset signal not found in powerSignalMap");
        }
        setGPIOOutput(board1PreSystemReset->second,
                      !board1PreSystemReset->second->polarity);
    }

    // De-assert NVL144 PDB Main Power Enable
    setGPIOOutput(nvl144pdbMainPowerEnable->second,
                  !nvl144pdbMainPowerEnable->second->polarity);
}

// Helper function: Transition to PDB Main Power Off wait state
void NVL144PowerControl::transitionToPDBMainPowerOffState()
{
    hpmPowerGoodWatchdogTimer.cancel();

    lg2::info(
        "HPM Board 0 Run Power Good de-asserted. De-asserting Pre System Reset lines. De-asserting NVL144 PDB Main Power Enable, Starting PDB Main Power OK Watchdog Timer. Transitioning to PowerState::waitForPDBMainPowerOff.");

    deassertPreSystemResetsAndPDBMainPower();
    startTimer(TimerMap["NVL144PdbMainPowerOkWatchdogMs"],
               pdbMainPowerOkWatchdogTimer,
               Event::pdbMainPowerOkWatchdogTimerExpired);
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
            // TODO: determine if this is the correct fault handling for No Run
            // Power Good De-assertion during shutdown sequences
            lg2::error(
                "HPM Power Good Watchdog Timer Expired. Host Forceful Shutdown sequence failed! Conducting Cleanup Sequence: Setting GPIO states to match Host State ON. Setting Host Power State to On.");

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
            lg2::error(
                "Required NVL144 PDB signal '{SIGNAL}' not found in config",
                "SIGNAL", signalName);
            throw std::runtime_error(
                "NVL144: Required PDB signal missing from config: " +
                signalName);
        }
    }

    // Call VRPowerControl to validate common VR signals
    VRPowerControl::validateRequiredSignals();

    lg2::info("NVL144 signal validation complete - all required signals present");
}

void NVL144PowerControl::validateTimerConfigs()
{
    // Validate NVL144-specific PDB timer
    for (const auto& timerName : platformRequiredTimeoutValues)
    {
        if (TimerMap.find(timerName) == TimerMap.end())
        {
            lg2::error(
                "Required NVL144 timer config '{TIMER}' not found in config",
                "TIMER", timerName);
            throw std::runtime_error(
                "NVL144PowerControl: Required timer config missing: " +
                timerName);
        }
    }

    // Call VRPowerControl to validate common VR timers
    VRPowerControl::validateTimerConfigs();

    lg2::info("NVL144 timer configuration validation complete - all required timers present");
}

void NVL144PowerControl::setDefaultValues()
{
    // Set NVL144 PDB-specific default values for output signals
    lg2::info("Setting NVL144 default values for output signals");

    // Find and validate all NVL144 PDB-specific signals first
    auto nvl144PdbMainPowerEnable =
        powerSignalMap.find("NVL144PDBMainPowerEnable");
    if (nvl144PdbMainPowerEnable == powerSignalMap.end())
    {
        throw std::runtime_error(
            "NVL144PDBMainPowerEnable signal not found in powerSignalMap");
    }

    auto e1sPowerEnable = powerSignalMap.find("E1SPowerEnable");
    if (e1sPowerEnable == powerSignalMap.end())
    {
        throw std::runtime_error(
            "E1SPowerEnable signal not found in powerSignalMap");
    }

    auto bmcSsdReset = powerSignalMap.find("BMCSSDReset");
    if (bmcSsdReset == powerSignalMap.end())
    {
        throw std::runtime_error(
            "BMCSSDReset signal not found in powerSignalMap");
    }

    // All NVL144 signals validated, now set the default states

    // NVL144 PDB Main Power Enable
    // - ON: Asserted (PDB should be powered)
    // - OFF: DeAsserted (PDB should be unpowered)
    nvl144PdbMainPowerEnable->second->defaultStateHostStateOn =
        DefaultState::Asserted;
    nvl144PdbMainPowerEnable->second->defaultStateHostStateOff =
        DefaultState::DeAsserted;

    // E1S Power Enable
    // - ON: Asserted (E1S should be powered)
    // - OFF: DeAsserted (E1S should be unpowered)
    e1sPowerEnable->second->defaultStateHostStateOn =
        DefaultState::Asserted;
    e1sPowerEnable->second->defaultStateHostStateOff =
        DefaultState::DeAsserted;

    // BMC SSD Reset
    // - ON: DeAsserted (BMC SSD should be out of reset)
    // - OFF: Asserted (BMC SSD should be in reset)
    bmcSsdReset->second->defaultStateHostStateOn =
        DefaultState::DeAsserted;
    bmcSsdReset->second->defaultStateHostStateOff =
        DefaultState::Asserted;

    // Call parent to set common VR/HPM defaults
    VRPowerControl::setDefaultValues();

    lg2::info("NVL144 default values set successfully");
}

} // namespace power_control
