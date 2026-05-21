// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "config.h"

#include "vr_power_control.hpp"

#include <phosphor-logging/lg2.hpp>

#include <filesystem>

namespace power_control
{
// Type aliases for convenience
using Event = PowerControl::Event;

// Constructor: Assigns handlers and registers events for common VR/HPM GPIOs
VRPowerControl::VRPowerControl(
    boost::asio::io_context& ioContext,
    std::shared_ptr<sdbusplus::asio::connection> conn,
    const std::string& configFilePath, const std::string& node,
    PersistentState& appState) :
    PowerControl(ioContext, conn, node, appState, configFilePath),
    hpmPowerGoodWatchdogTimer(ioContext), cpuResetWatchdogTimer(ioContext),
    cpuShutdownOkWatchdogTimer(ioContext),
    cpuBootDoneDeAssertWatchdogTimer(ioContext),
    powerCycleDelayTimer(ioContext), warmRebootDelayTimer(ioContext),
    pdbMainPowerOkWatchdogTimer(ioContext),
    hostInitiatedRebootPulseTimer(ioContext),
    hostRebootShutdownOkTimer(ioContext)
{
    // powerSignalMap is now populated by PowerControl::loadConfigValues()
    // Assign handlers and register events for common VR/HPM signals
    detectBoardPresence();

    // Add VR-specific required signals with handlers
    // Board 0 signals (always required for VR platforms)
    addRequiredSignal("Board0RunPowerEnable", 0, GPIODirection::OUT);
    addRequiredSignal("Board0RunPowerPG", 0, GPIODirection::IN,
                      [this](bool state) {
                          this->board0RunPowerPGHandler(state);
                      });
    addRequiredSignal("Board0PreSystemReset", 0, GPIODirection::OUT);
    addRequiredSignal("Board0CpuShutdownForce", 0, GPIODirection::OUT);
    addRequiredSignal("Board0CpuShutdownRequest", 0, GPIODirection::OUT);
    addRequiredSignal("Board0CpuShutdownOk", 0, GPIODirection::IN,
                      [this](bool state) {
                          this->board0CpuShutdownOkHandler(state);
                      });
    addRequiredSignal("CpuResetIndicator", 0, GPIODirection::IN,
                      [this](bool state) {
                          this->cpuResetIndicatorHandler(state);
                      });
    addRequiredSignal("USBPowerEnable", 0, GPIODirection::OUT);

    // Add Board 1 handler if Board 1 is present
    if (boardPresence.board1Present)
    {
        addRequiredSignal("Board1CpuShutdownOk", 1, GPIODirection::IN,
                          [this](bool state) {
                              this->board1CpuShutdownOkHandler(state);
                          });
    }

    // Add PDB signals shared by all VR platforms
    addRequiredSignal("PDBMainPowerOk", 0, GPIODirection::IN,
                      [this](bool state) {
                          this->pdbMainPowerOkHandler(state);
                      });
    // Note: PDBMainPowerEnable is NOT registered here — each platform registers
    // its own PDB enable signal(s) in its constructor (e.g. NVL72 registers
    // PDBMainPowerEnable; C2 has no such signal and uses PDBPSUPowerOn
    // instead).

    // call validateRequiredSignals() in the platform-specific class constructor
    // validateRequiredSignals();
}

void VRPowerControl::detectBoardPresence()
{
    // Check presence and update context using paths from build configuration
    boardPresence.parsecPdbPresent = checkIOXPresence(GB300_PDB_IOX_PATH);
    boardPresence.c2PdbPresent = checkIOXPresence(C2_PDB_IOX_PATH);
    boardPresence.nvl72PdbPresent = checkIOXPresence(NVL72_PDB_IOX_PATH);
    boardPresence.board0Present = checkIOXPresence(BOARD0_IOX_PATH);
    boardPresence.board1Present = checkIOXPresence(BOARD1_IOX_PATH);

    // Log detected board presence
    lg2::info("Board presence detection:");
    lg2::info("  GB300 PDB ({PATH}): {PRESENT}", "PATH",
              std::string(GB300_PDB_IOX_PATH), "PRESENT",
              boardPresence.parsecPdbPresent);
    lg2::info("  C2 PDB ({PATH}): {PRESENT}", "PATH",
              std::string(C2_PDB_IOX_PATH), "PRESENT",
              boardPresence.c2PdbPresent);
    lg2::info("  NVL72 PDB ({PATH}): {PRESENT}", "PATH",
              std::string(NVL72_PDB_IOX_PATH), "PRESENT",
              boardPresence.nvl72PdbPresent);
    lg2::info("  Board 0 ({PATH}): {PRESENT}", "PATH",
              std::string(BOARD0_IOX_PATH), "PRESENT",
              boardPresence.board0Present);
    lg2::info("  Board 1 ({PATH}): {PRESENT}", "PATH",
              std::string(BOARD1_IOX_PATH), "PRESENT",
              boardPresence.board1Present);
}

// Board presence detection functions
bool VRPowerControl::checkIOXPresence(const std::string& ioxPath)
{
    return std::filesystem::exists(ioxPath);
}

// =============================================================================
// GPIO EVENT HANDLERS (Member functions)
// =============================================================================

void VRPowerControl::board0RunPowerPGHandler(bool state)
{
    auto configPtr = getSignal("Board0RunPowerPG");
    if (!configPtr)
    {
        return;
    }

    auto& config = *configPtr;

    // Update D-Bus property
    setBoard0RunPowerPGState(state);
    lg2::info("Board0RunPowerPG GPIO event: value={VALUE}", "VALUE",
              static_cast<int>(state));

    Event powerControlEvent = (state == config.polarity)
                                  ? Event::board0RunPowerPGAssert
                                  : Event::board0RunPowerPGDeAssert;

    // Check for power faults and handle if detected
    if (checkAndHandleRunPowerFault(powerControlEvent))
    {
        return; // Fault was handled, exit early
    }

    // Normal event processing for expected state transitions
    this->sendPowerControlEvent(powerControlEvent);
}

void VRPowerControl::board0CpuShutdownOkHandler(bool state)
{
    auto configPtr = getSignal("Board0CpuShutdownOk");
    if (!configPtr)
    {
        return;
    }

    auto& config = *configPtr;

    // Update D-Bus property
    setBoard0CpuShutdownOkState(state);
    lg2::info("Board0CpuShutdownOk GPIO event: value={VALUE}", "VALUE",
              static_cast<int>(state));

    Event powerControlEvent = (state == config.polarity)
                                  ? Event::board0CpuShutdownOkAssert
                                  : Event::board0CpuShutdownOkDeAssert;
    this->sendPowerControlEvent(powerControlEvent);
}

void VRPowerControl::board1CpuShutdownOkHandler(bool state)
{
    if (!boardPresence.board1Present)
    {
        // Defensive guard: ignore Board 1 events on 1P systems.
        return;
    }

    auto configPtr = getSignal("Board1CpuShutdownOk");
    if (!configPtr)
    {
        return;
    }

    auto& config = *configPtr;

    // Update D-Bus property
    setBoard1CpuShutdownOkState(state);
    lg2::info("Board1CpuShutdownOk GPIO event: value={VALUE}", "VALUE",
              static_cast<int>(state));

    Event powerControlEvent = (state == config.polarity)
                                  ? Event::board1CpuShutdownOkAssert
                                  : Event::board1CpuShutdownOkDeAssert;
    this->sendPowerControlEvent(powerControlEvent);
}

void VRPowerControl::cpuResetIndicatorHandler(bool state)
{
    auto configPtr = getSignal("CpuResetIndicator");
    if (!configPtr)
    {
        return;
    }

    auto& config = *configPtr;

    // Update D-Bus property
    setCpuResetIndicatorState(state);
    lg2::info("CpuResetIndicator GPIO event: value={VALUE}", "VALUE",
              static_cast<int>(state));

    Event powerControlEvent = (state == config.polarity)
                                  ? Event::cpuResetIndicatorAssert
                                  : Event::cpuResetIndicatorDeAssert;
    this->sendPowerControlEvent(powerControlEvent);
}

void VRPowerControl::pdbMainPowerOkHandler(bool state)
{
    lg2::info("PDBMainPowerOk GPIO event: value={VALUE}", "VALUE",
              static_cast<int>(state));

    auto configPtr = getSignal("PDBMainPowerOk");
    if (!configPtr)
    {
        return;
    }

    Event powerControlEvent = (state == configPtr->polarity)
                                  ? Event::pdbMainPowerOkAssert
                                  : Event::pdbMainPowerOkDeAssert;

    this->sendPowerControlEvent(powerControlEvent);
}

std::function<void(Event)> VRPowerControl::getPowerStateHandler()
{
    // Map VR-specific PowerState values to their handler functions
    switch (powerState)
    {
        // VR-specific states
        case PowerState::waitForPDBMainPowerOk:
            return [this](Event e) { this->handleWaitForPDBMainPowerOk(e); };

        case PowerState::waitForPDBMainPowerOff:
            return [this](Event e) { this->handleWaitForPDBMainPowerOff(e); };

        case PowerState::waitForHPMPowerGoodAssert:
            return
                [this](Event e) { this->handleWaitForHPMPowerGoodAssert(e); };

        case PowerState::waitForHPMPowerGoodDeAssert:
            return
                [this](Event e) { this->handleWaitForHPMPowerGoodDeAssert(e); };

        case PowerState::waitForCPUResetAssert:
            return [this](Event e) { this->handleWaitForCPUResetAssert(e); };

        case PowerState::waitForCPUResetDeAssert:
            return [this](Event e) { this->handleWaitForCPUResetDeAssert(e); };

        case PowerState::waitForCPUShutdownOk:
            return [this](Event e) { this->handleWaitForCPUShutdownOk(e); };

        case PowerState::waitForCPUBootDoneDeAssert:
            return
                [this](Event e) { this->handleWaitForCPUBootDoneDeAssert(e); };

        case PowerState::waitForPowerCycleDelay:
            return [this](Event e) { this->handleWaitForPowerCycleDelay(e); };

        case PowerState::waitForRebootDelay:
            return [this](Event e) { this->handleWaitForRebootDelay(e); };

        case PowerState::waitForHostRebootShutdownOk:
            return
                [this](Event e) { this->handleWaitForHostRebootShutdownOk(e); };

        // Delegate upstream states to base class
        default:
            return PowerControl::getPowerStateHandler();
    }
}

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================

void VRPowerControl::transitionToOffStateWithRunPowerCheck()
{
    // Check current state of Board0RunPowerPG
    auto board0RunPowerPG = getSignal("Board0RunPowerPG");
    if (!board0RunPowerPG || !board0RunPowerPG->gpioLine)
    {
        lg2::error(
            "CRITICAL: Board0RunPowerPG not available - transitioning directly to off");
        setGPIOsForHostStateOff();
        setPowerState(PowerState::off);
        return;
    }

    bool runPowerPGAsserted =
        board0RunPowerPG->gpioLine.get_value() == board0RunPowerPG->polarity;

    if (runPowerPGAsserted)
    {
        // Run Power is still asserted - need to wait for de-assertion
        lg2::info(
            "Board0RunPowerPG is currently asserted. Transitioning to waitForHPMPowerGoodDeAssert to wait for de-assertion.");
        setPowerState(PowerState::waitForHPMPowerGoodDeAssert);
        setGPIOsForHostStateOff();
        startTimer("HPMPowerGoodWatchdogMs", hpmPowerGoodWatchdogTimer,
                   Event::hpmPowerGoodWatchdogTimerExpired);
    }
    else
    {
        // Run Power is already de-asserted - go directly to off
        lg2::info(
            "Board0RunPowerPG is already de-asserted. Transitioning directly to PowerState::off.");
        setGPIOsForHostStateOff();
        setPowerState(PowerState::off);
    }
}

// board0RunPowerPGHandler Helper Function
bool VRPowerControl::checkAndHandleRunPowerFault(Event powerControlEvent)
{
    // Power fault detection: Check for unexpected de-assertion
    if (powerControlEvent == Event::board0RunPowerPGDeAssert)
    {
        if (powerState != PowerState::waitForHPMPowerGoodDeAssert)
        {
            // POWER FAULT: Run Power Good de-asserted unexpectedly
            lg2::error(
                "POWER FAULT DETECTED: Board0RunPowerPG de-asserted unexpectedly while in power state {STATE}. "
                "Setting GPIO states to match Host State OFF. Transitioning to Host State OFF.",
                "STATE", getPowerStateName());

            // Transition to off, checking if we need to wait for de-assertion
            action = PowerAction::NONE;
            logResourceEvent(
                "ResourceErrorsDetected",
                {"Host0",
                 "Board0 Run Power Good de-asserted unexpectedly while in power state {STATE}.",
                 "STATE", getPowerStateName()},
                "xyz.openbmc_project.Logging.Entry.Level.Error");
            transitionToOffStateWithRunPowerCheck();

            return true;
        }
        // else: Expected de-assertion in waitForHPMPowerGoodDeAssert state
    }
    else if (powerControlEvent == Event::board0RunPowerPGAssert)
    {
        if (powerState != PowerState::waitForHPMPowerGoodAssert)
        {
            // Unexpected assertion (not critical, but worth logging)
            lg2::info(
                "Board0RunPowerPG asserted unexpectedly while in power state {STATE}. Continuing with normal event processing.",
                "STATE", getPowerStateName());
        }
        // else: Expected assertion in waitForHPMPowerGoodAssert state
    }

    // Return false to indicate normal processing should continue
    return false;
}

void VRPowerControl::handlePowerStateOn(Event event)
{
    switch (event)
    {
        case Event::cpuBootDoneDeAssert:
            // CPU_BOOT_DONE de-asserted before SHDN_OK: possible host-initiated
            // reboot. Transition first so a close-timed SHDN_OK is handled by
            // the new state.
            initiateWaitForHostRebootShutdownOk();
            break;

        case Event::board0CpuShutdownOkAssert:
        case Event::board1CpuShutdownOkAssert:
            // Host-initiated shutdown: CPU has asserted SHDN_OK
            handleHostInitiatedShutdown();
            break;

        case Event::powerOffRequest:
        case Event::gracefulPowerOffRequest:
            handleShutdownRequest(event);
            break;

        case Event::powerCycleRequest:
            lg2::info(
                "Forceful Power Cycle Request received. Initiating forceful shutdown");
            action = PowerAction::POWER_CYCLE;
            handleShutdownRequest(Event::powerOffRequest);
            break;

        case Event::gracefulPowerCycleRequest:
            lg2::info(
                "Graceful Power Cycle Request received. Initiating graceful shutdown");
            action = PowerAction::GRACEFUL_POWER_CYCLE;
            handleShutdownRequest(Event::gracefulPowerCycleRequest);
            break;

        case Event::gracefulResetRequest:
            lg2::info(
                "Graceful warm reboot requested. Initiating graceful shutdown request then warm reset sequence");
            action = PowerAction::GRACEFUL_WARM_REBOOT;
            handleShutdownRequest(Event::gracefulResetRequest);
            break;

        case Event::resetRequest:
            // Safe stating PHYs: (SHDN_FORCE & SHDN_OK) then toggle Pre System
            // Reset signals
            action = PowerAction::FORCE_WARM_REBOOT;
            handleShutdownRequest(Event::powerOffRequest);
            break;

        case Event::powerButtonPressed:
            break;

        default:
            lg2::info("No action taken.");
            break;
    }
}

void VRPowerControl::handlePowerStateOff(Event event)
{
    switch (event)
    {
        case Event::powerOnRequest:
            handlePowerOnRequest();
            break;

        case Event::powerCycleRequest:
        case Event::gracefulPowerCycleRequest:
            handlePowerCycleWhenOff(event);
            break;

        case Event::powerButtonPressed:
            break;

        case Event::gracefulResetRequest:
            lg2::info(
                "Graceful warm reboot requested while host is off; no action. "
                "Host must be powered on and booted before issuing graceful warm reboot request.");
            break;

        case Event::resetRequest:
            lg2::info(
                "Reset request received while host is off; no action. Host must be powered on.");
            break;

        default:
            lg2::info("No action taken.");
            break;
    }
}

void VRPowerControl::handleWaitForPDBMainPowerOk(Event event)
{
    switch (event)
    {
        case Event::pdbMainPowerOkAssert:
            transitionToHPMPowerGoodAssertState();
            break;

        case Event::pdbMainPowerOkWatchdogTimerExpired:
            lg2::error(
                "PDB Main Power OK watchdog timer expired. PDB Main Power On Sequence Failed. "
                "Host Power On sequence failed. Conducting Cleanup Sequence: Setting GPIO states "
                "to match Host State OFF. Checking Board0RunPowerPG state and transitioning appropriately.");

            action = PowerAction::NONE;
            logResourceEvent(
                "ResourceErrorsDetected",
                {"Host0", "PDB Main Power OK watchdog expired (power on)"},
                "xyz.openbmc_project.Logging.Entry.Level.Error");
            transitionToOffStateWithRunPowerCheck();
            break;

        default:
            lg2::info("No action taken.");
            break;
    }
}

void VRPowerControl::handleWaitForPDBMainPowerOff(Event event)
{
    switch (event)
    {
        case Event::pdbMainPowerOkDeAssert:
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

// =============================================================================
// HELPER FUNCTIONS (moved from platform, shared by all VR platforms)
// =============================================================================

void VRPowerControl::handleShutdownRequest(Event event)
{
    bool isForceful = (event == Event::powerOffRequest);
    std::string shutdownType = isForceful ? "Forceful" : "Graceful";

    if (!isForceful)
    {
        // Validate CPU Boot Done state for graceful operations
        int bootDoneState = getCPUBootDoneState();

        if (bootDoneState < 0)
        {
            lg2::error(
                "CPU Boot Done signal value not yet initialized by Phosphor GPIO Monitor! "
                "Host Graceful Operations cannot proceed.");
            logResourceEvent(
                "ResourceErrorsDetected",
                {"Host0",
                 "CPU Boot Done signal value not yet initialized by Phosphor GPIO Monitor! "
                 "Host Graceful Operations cannot proceed."},
                "xyz.openbmc_project.Logging.Entry.Level.Error");
            return;
        }
        else if (bootDoneState == 0)
        {
            lg2::error(
                "CPU Boot Done is DE-ASSERTED. Host Graceful Operations cannot proceed.");
            logResourceEvent(
                "ResourceErrorsDetected",
                {"Host0",
                 "CPU Boot Done is DE-ASSERTED. Host Graceful Operations cannot proceed."},
                "xyz.openbmc_project.Logging.Entry.Level.Error");
            return;
        }
        else
        {
            lg2::info(
                "CPU Boot Done is ASSERTED. Proceeding with Host Graceful Shutdown operation");
        }
    }

    // Preserve power cycle / force warm reboot context; only set action for
    // direct shutdown requests
    if (action != PowerAction::POWER_CYCLE &&
        action != PowerAction::GRACEFUL_POWER_CYCLE &&
        action != PowerAction::FORCE_WARM_REBOOT &&
        action != PowerAction::GRACEFUL_WARM_REBOOT)
    {
        action = isForceful ? PowerAction::FORCE_OFF : PowerAction::GRACE_OFF;
    }

    if (action == PowerAction::FORCE_WARM_REBOOT)
    {
        lg2::info(
            "Commencing force warm reboot sequence: asserting SHDN FORCE to safe state PHYs, "
            "then toggling Pre System Reset signals.");
    }
    else
    {
        lg2::info("Commencing {SHUTDOWN_TYPE} sequence.", "SHUTDOWN_TYPE",
                  shutdownType);
    }

    if (isSystemPowerOff())
    {
        if (action == PowerAction::POWER_CYCLE)
        {
            lg2::info(
                "Power already off during forceful power cycle. Setting GPIOs for host state OFF, "
                "starting power cycle delay timer, and transitioning to PowerState::waitForPowerCycleDelay");
            transitionToPowerCycleDelay();
        }
        else if (action == PowerAction::GRACEFUL_POWER_CYCLE)
        {
            lg2::info(
                "Power already off during graceful power cycle. Setting GPIOs for host state OFF, "
                "starting power cycle delay timer, and transitioning to PowerState::waitForPowerCycleDelay");
            transitionToPowerCycleDelay();
        }
        else if (action == PowerAction::GRACEFUL_WARM_REBOOT)
        {
            lg2::info(
                "Graceful warm reboot requested while host appears powered off. "
                "Cannot proceed with warm reboot. Transitioning to PowerState::Off");
            transitionToOffState();
        }
        else
        {
            lg2::info(
                "PDB Main Power and HPM Run Power is already disabled. Setting GPIOs for host state OFF "
                "and transitioning to PowerState::Off");
            transitionToOffState();
        }
    }
    else
    {
        initiateCPUShutdown(isForceful);
    }
}

void VRPowerControl::initiateCPUShutdown(bool isForceful)
{
    const char* shutdownSignalName =
        isForceful ? "Board0CpuShutdownForce" : "Board0CpuShutdownRequest";
    const char* shutdownOkTimerName =
        isForceful ? "ForcefulCpuShutdownOkWatchdogMs"
                   : "GracefulCpuShutdownOkWatchdogMs";
    const char* shutdownAction =
        isForceful ? "Shutdown Force" : "Shutdown Request";

    auto shutdownSignal = getSignal(shutdownSignalName);
    if (!shutdownSignal)
    {
        return;
    }

    // When asserting force, de-assert graceful request lines so both are not
    // active (e.g. upgrade from graceful SHDN_OK wait to forceful shutdown).
    if (isForceful)
    {
        auto board0Req = getSignal("Board0CpuShutdownRequest");
        if (board0Req)
        {
            setGPIOOutput(board0Req, !board0Req->polarity);
        }
    }

    lg2::info(
        "Asserting Board 0 CPU {SHUTDOWN_ACTION}. Starting CPU Shutdown OK Watchdog Timer. "
        "Transitioning to PowerState::waitForCPUShutdownOk",
        "SHUTDOWN_ACTION", shutdownAction);

    setGPIOOutput(shutdownSignal, shutdownSignal->polarity);

    if (boardPresence.board1Present)
    {
        const char* board1SignalName =
            isForceful ? "Board1CpuShutdownForce" : "Board1CpuShutdownRequest";

        auto board1ShutdownSignal = getSignal(board1SignalName);
        if (!board1ShutdownSignal)
        {
            return;
        }

        lg2::info("De-asserting Board 1 CPU {SHUTDOWN_ACTION}",
                  "SHUTDOWN_ACTION", shutdownAction);

        setGPIOOutput(board1ShutdownSignal, !board1ShutdownSignal->polarity);
    }

    startTimer(shutdownOkTimerName, cpuShutdownOkWatchdogTimer,
               Event::cpuShutdownOkWatchdogTimerExpired);
    setPowerState(PowerState::waitForCPUShutdownOk);
}

void VRPowerControl::handlePowerCycleWhenOff(Event event)
{
    bool isForceful = (event == Event::powerCycleRequest);
    const char* cycleType = isForceful ? "Forceful" : "Graceful";
    PowerAction cycleAction = isForceful ? PowerAction::POWER_CYCLE
                                         : PowerAction::GRACEFUL_POWER_CYCLE;
    Event shutdownEvent =
        isForceful ? Event::powerOffRequest : Event::gracefulPowerOffRequest;

    lg2::info("{CYCLE_TYPE} Power Cycle Request received while in off state",
              "CYCLE_TYPE", cycleType);

    auto board0RunPowerPG = getSignal("Board0RunPowerPG");
    if (!board0RunPowerPG || !board0RunPowerPG->gpioLine)
    {
        lg2::error(
            "CRITICAL: Board0RunPowerPG not available - cannot power cycle");
        return;
    }

    if (board0RunPowerPG->gpioLine.get_value() == !board0RunPowerPG->polarity)
    {
        lg2::info(
            "Verified Board 0 Run Power PG is de-asserted. Initiating Host Power On sequence");
        action = cycleAction;
        handlePowerOnRequest();
    }
    else
    {
        lg2::warning(
            "{CYCLE_TYPE} Power cycle requested but Board 0 Run Power PG is not de-asserted. "
            "Initiating Host {SHUTDOWN_TYPE} Shutdown first",
            "CYCLE_TYPE", cycleType, "SHUTDOWN_TYPE", cycleType);
        action = cycleAction;
        setPowerState(PowerState::on);
        handleShutdownRequest(shutdownEvent);
    }
}

void VRPowerControl::transitionToOffState()
{
    action = PowerAction::NONE;
    setGPIOsForHostStateOff();
    setPowerState(PowerState::off);
}

void VRPowerControl::transitionToPowerCycleDelay()
{
    // Keep action (POWER_CYCLE or GRACEFUL_POWER_CYCLE) - don't clear it
    setGPIOsForHostStateOff();
    startTimer("PowerCycleDelayMs", powerCycleDelayTimer,
               Event::powerCycleDelayTimerExpired);
    setPowerState(PowerState::waitForPowerCycleDelay);
}

void VRPowerControl::assertHPMBoardPowerSequence()
{
    auto board0PreSystemReset = getSignal("Board0PreSystemReset");
    if (!board0PreSystemReset)
    {
        return;
    }

    auto usbPowerEnable = getSignal("USBPowerEnable");
    if (!usbPowerEnable)
    {
        return;
    }

    auto board0RunPowerEnable = getSignal("Board0RunPowerEnable");
    if (!board0RunPowerEnable)
    {
        return;
    }

    // Assert Pre System Reset for Board 0
    setGPIOOutput(board0PreSystemReset, board0PreSystemReset->polarity);

    if (boardPresence.board1Present)
    {
        auto board1PreSystemReset = getSignal("Board1PreSystemReset");
        if (!board1PreSystemReset)
        {
            return;
        }
        setGPIOOutput(board1PreSystemReset, board1PreSystemReset->polarity);
    }

    setGPIOOutput(usbPowerEnable, usbPowerEnable->polarity);
    setGPIOOutput(board0RunPowerEnable, board0RunPowerEnable->polarity);

    if (boardPresence.board1Present)
    {
        auto board1RunPowerEnable = getSignal("Board1RunPowerEnable");
        if (!board1RunPowerEnable)
        {
            return;
        }
        setGPIOOutput(board1RunPowerEnable, board1RunPowerEnable->polarity);
    }
}

void VRPowerControl::transitionToHPMPowerGoodAssertState()
{
    cancelTimer("PDB Main Power OK Watchdog Timer",
                pdbMainPowerOkWatchdogTimer);

    lg2::info(
        "PDB Main Power OK Asserted. Conducting HPM Board Power Sequencing. "
        "Starting HPM Power Good Watchdog Timer. Transitioning to PowerState::waitForHPMPowerGoodAssert.");

    assertHPMBoardPowerSequence();
    startTimer("HPMPowerGoodWatchdogMs", hpmPowerGoodWatchdogTimer,
               Event::hpmPowerGoodWatchdogTimerExpired);
    setPowerState(PowerState::waitForHPMPowerGoodAssert);
}

void VRPowerControl::deassertHPMPowerAndPeripherals()
{
    auto board0RunPowerEnable = getSignal("Board0RunPowerEnable");
    if (!board0RunPowerEnable)
    {
        return;
    }

    auto usbPowerEnable = getSignal("USBPowerEnable");
    if (!usbPowerEnable)
    {
        return;
    }

    setGPIOOutput(board0RunPowerEnable, !board0RunPowerEnable->polarity);

    if (boardPresence.board1Present)
    {
        auto board1RunPowerEnable = getSignal("Board1RunPowerEnable");
        if (!board1RunPowerEnable)
        {
            return;
        }
        setGPIOOutput(board1RunPowerEnable, !board1RunPowerEnable->polarity);
    }

    setGPIOOutput(usbPowerEnable, !usbPowerEnable->polarity);
}

void VRPowerControl::transitionToHPMPowerGoodDeAssertState()
{
    cancelTimer("CPU Reset Watchdog Timer", cpuResetWatchdogTimer);

    lg2::info(
        "CPU Reset Indicator Asserted. CPUs are in reset. De-asserting Run Power Enable and USB Power Enable. "
        "Starting HPM Power Good Watchdog Timer. Transitioning to PowerState::waitForHPMPowerGoodDeAssert.");

    deassertHPMPowerAndPeripherals();
    startTimer("HPMPowerGoodWatchdogMs", hpmPowerGoodWatchdogTimer,
               Event::hpmPowerGoodWatchdogTimerExpired);
    setPowerState(PowerState::waitForHPMPowerGoodDeAssert);
}

void VRPowerControl::completeShutdownAndTransitionToOff(bool success)
{
    cancelTimer("PDB Main Power OK Watchdog Timer",
                pdbMainPowerOkWatchdogTimer);

    if (!success)
    {
        lg2::error(
            "PDB Main Power OK watchdog timer expired. PDB Main Power Off Sequence Failed. "
            "Host Power Off sequence failed. Conducting Cleanup Sequence: Setting GPIO states "
            "to match Host State OFF. Setting Host Power State to Off.");
        action = PowerAction::NONE;
        logResourceEvent(
            "ResourceErrorsDetected",
            {"Host0", "PDB Main Power OK watchdog expired (power off)"},
            "xyz.openbmc_project.Logging.Entry.Level.Error");
        setPowerState(PowerState::off);
        setGPIOsForHostStateOff();
        return;
    }

    applyShutdownAction();
}

void VRPowerControl::applyShutdownAction()
{
    switch (action)
    {
        case PowerAction::FORCE_OFF:
            lg2::info("Host Forceful Shutdown Sequence Completed Successfully. "
                      "Transitioning to PowerState::off.");
            transitionToOffState();
            break;

        case PowerAction::GRACE_OFF:
            lg2::info("Host Graceful Shutdown Sequence Completed Successfully. "
                      "Transitioning to PowerState::off.");
            transitionToOffState();
            break;

        case PowerAction::HOST_INITIATED_SHUTDOWN:
            lg2::info(
                "Host-Initiated Shutdown Sequence Completed Successfully. "
                "Transitioning to PowerState::off.");
            transitionToOffState();
            break;

        case PowerAction::POWER_CYCLE:
            lg2::info(
                "Forceful Power Cycle Shutdown complete. "
                "Starting power cycle delay timer. Transitioning to PowerState::waitForPowerCycleDelay.");
            transitionToPowerCycleDelay();
            break;

        case PowerAction::GRACEFUL_POWER_CYCLE:
            lg2::info(
                "Graceful Power Cycle Shutdown complete. "
                "Starting power cycle delay timer. Transitioning to PowerState::waitForPowerCycleDelay.");
            transitionToPowerCycleDelay();
            break;

        default:
            lg2::warning(
                "PDB powered down with unknown action. Setting GPIO states to match Host State OFF. "
                "Transitioning to PowerState::off.");
            action = PowerAction::NONE;
            logResourceEvent("ResourceErrorsDetected",
                             {"Host0", "PDB powered down with unknown action"},
                             "xyz.openbmc_project.Logging.Entry.Level.Warning");
            setPowerState(PowerState::off);
            setGPIOsForHostStateOff();
            break;
    }
}

// Helper function: Initiate a force warm reboot sequence
void VRPowerControl::initiateForceWarmReboot()
{
    cancelTimer("CPU Shutdown OK Watchdog Timer", cpuShutdownOkWatchdogTimer);

    action = PowerAction::FORCE_WARM_REBOOT;

    lg2::info("De-asserting Shutdown Force signals");
    auto board0CpuShutdownForce = getSignal("Board0CpuShutdownForce");
    if (!board0CpuShutdownForce)
    {
        return;
    }
    setGPIOOutput(board0CpuShutdownForce, !board0CpuShutdownForce->polarity);

    if (boardPresence.board1Present)
    {
        auto board1CpuShutdownForce = getSignal("Board1CpuShutdownForce");
        if (!board1CpuShutdownForce)
        {
            return;
        }
        setGPIOOutput(board1CpuShutdownForce,
                      !board1CpuShutdownForce->polarity);
    }

    lg2::info("Asserting Pre System Reset signals");

    // Assert Pre System Reset for Board 0
    auto board0PreSystemReset = getSignal("Board0PreSystemReset");
    if (!board0PreSystemReset)
    {
        return;
    }
    setGPIOOutput(board0PreSystemReset, board0PreSystemReset->polarity);

    // Assert Pre System Reset for Board 1 if present
    if (boardPresence.board1Present)
    {
        auto board1PreSystemReset = getSignal("Board1PreSystemReset");
        if (!board1PreSystemReset)
        {
            return;
        }
        setGPIOOutput(board1PreSystemReset, board1PreSystemReset->polarity);
    }

    // Start CPU Reset Assert watchdog and wait for CPU_RESET_L to assert
    startTimer("CpuResetWatchdogMs", cpuResetWatchdogTimer,
               Event::cpuResetWatchdogTimerExpired);
    setPowerState(PowerState::waitForCPUResetAssert);
}

// ============================================================================
// HELPER FUNCTIONS for handleWaitForHPMPowerGoodAssert
// ============================================================================

void VRPowerControl::deassertPreSystemResets()
{
    auto board0PreSystemReset = getSignal("Board0PreSystemReset");
    if (!board0PreSystemReset)
    {
        return;
    }

    // De-assert Board 0 Pre System Reset
    setGPIOOutput(board0PreSystemReset, !board0PreSystemReset->polarity);

    // De-assert Board 1 Pre System Reset if present
    if (boardPresence.board1Present)
    {
        auto board1PreSystemReset = getSignal("Board1PreSystemReset");
        if (!board1PreSystemReset)
        {
            return;
        }

        setGPIOOutput(board1PreSystemReset, !board1PreSystemReset->polarity);
    }
}

void VRPowerControl::abortWarmRebootCpuResetWatchdogFault(
    std::string_view faultDetail)
{
    cancelTimer("CPU Reset Watchdog Timer", cpuResetWatchdogTimer);

    lg2::error(
        "CRITICAL WARM REBOOT FAULT: {DETAIL}. De-asserting Pre System Reset only; "
        "HPM run power is NOT removed. Returning host/chassis to On.",
        "DETAIL", faultDetail);

    deassertPreSystemResets();

    logResourceEvent("ResourceErrorsDetected",
                     {std::string("Host0"), std::string(faultDetail)},
                     "xyz.openbmc_project.Logging.Entry.Level.Error");

    action = PowerAction::NONE;
    setPowerState(PowerState::on);
}

// Helper function: Transition to CPU Reset Assert wait state
void VRPowerControl::transitionToCPUResetDeAssertState()
{
    cancelTimer("HPM Power Good Watchdog Timer", hpmPowerGoodWatchdogTimer);

    lg2::info(
        "HPM Board 0 Run Power Good Asserted. De-asserting Pre System Resets. Starting CPU Reset Watchdog Timer. Transitioning to PowerState::waitForCPUResetDeAssert.");

    deassertPreSystemResets();
    startTimer("CpuResetWatchdogMs", cpuResetWatchdogTimer,
               Event::cpuResetWatchdogTimerExpired);
    setPowerState(PowerState::waitForCPUResetDeAssert);
}

// ============================================================================
// handleWaitForHPMPowerGoodAssert state handler
// ============================================================================

void VRPowerControl::handleWaitForHPMPowerGoodAssert(Event event)
{
    // TODO: Move powerStateWaitForHPMPowerGoodAssert() implementation here
    switch (event)
    {
        case Event::board0RunPowerPGAssert:
            transitionToCPUResetDeAssertState(); // Transition to CPU Reset
                                                 // De-assert state
            break;

        case Event::hpmPowerGoodWatchdogTimerExpired:
            lg2::error(
                "HPM Power Good Watchdog Timer Expired. Host Power On sequence failed. Conducting Cleanup Sequence: Setting GPIO states to match Host State OFF. Checking Board0RunPowerPG state and transitioning appropriately.");

            action = PowerAction::NONE;
            logResourceEvent(
                "ResourceErrorsDetected",
                {"Host0", "HPM Power Good Watchdog expired (power on)"},
                "xyz.openbmc_project.Logging.Entry.Level.Error");
            transitionToOffStateWithRunPowerCheck();
            break;

        default:
            lg2::info("No action taken for event: {EVENT}", "EVENT",
                      getEventName(event));
            break;
    }
}

void VRPowerControl::handleHPMPowerGoodWatchdogExpiredDuringShutdown()
{
    lg2::error(
        "HPM Power Good Watchdog Timer Expired. Host Forceful Shutdown sequence failed! "
        "Conducting Cleanup Sequence: Setting GPIO states to match Host State ON. "
        "Setting Host Power State to On.");

    action = PowerAction::NONE;
    logResourceEvent(
        "ResourceErrorsDetected",
        {"Host0", "HPM Power Good Watchdog expired (shutdown sequence)"},
        "xyz.openbmc_project.Logging.Entry.Level.Error");
    setGPIOsForHostStateOn();
    setPowerState(PowerState::on);
}

void VRPowerControl::handleWaitForHPMPowerGoodDeAssert(Event event)
{
    switch (event)
    {
        case Event::board0RunPowerPGDeAssert:
            initiatePDBPowerOff();
            break;

        case Event::hpmPowerGoodWatchdogTimerExpired:
            handleHPMPowerGoodWatchdogExpiredDuringShutdown();
            break;

        default:
            lg2::info("No action taken.");
            break;
    }
}

void VRPowerControl::handleCPUResetIndicatorAsserted()
{
    cancelTimer("CPU Reset Watchdog Timer", cpuResetWatchdogTimer);
    lg2::info("CPU Reset Indicator asserted - CPUs entered reset");

    if (action == PowerAction::FORCE_WARM_REBOOT ||
        action == PowerAction::GRACEFUL_WARM_REBOOT)
    {
        const char* warmRebootDelayKey =
            (action == PowerAction::GRACEFUL_WARM_REBOOT)
                ? "GracefulWarmRebootDelayMs"
                : "ForceWarmRebootDelayMs";
        auto it = TimerMap.find(warmRebootDelayKey);
        int delayMs = (it != TimerMap.end()) ? it->second : 0;
        lg2::info("Starting warm reboot delay ({KEY}) of {DELAY}ms", "KEY",
                  warmRebootDelayKey, "DELAY", delayMs);
        startTimer(warmRebootDelayKey, warmRebootDelayTimer,
                   Event::warmRebootDelayTimerExpired);
        setPowerState(PowerState::waitForRebootDelay);
    }
    else
    {
        // Shutdown flow: de-assert HPM power and wait for PG de-assert
        transitionToHPMPowerGoodDeAssertState();
    }
}

void VRPowerControl::handleCPUResetWatchdogExpired()
{
    if (action == PowerAction::FORCE_WARM_REBOOT ||
        action == PowerAction::GRACEFUL_WARM_REBOOT)
    {
        lg2::error(
            "Warm reboot fault: CPU_RESET_L did not assert within the timeout period. "
            "CPUs did not enter reset. Host Power State is still On. Conducting cleanup: "
            "Setting GPIO states to match Host State On.");
        abortWarmRebootCpuResetWatchdogFault(
            "Warm reboot fault: CPU_RESET_L did not assert within "
            "CpuResetWatchdogMs (CPUs did not enter reset). "
            "PRE_SYS_RST de-asserted; run power unchanged.");
        setGPIOsForHostStateOn();
    }
    else
    {
        lg2::error(
            "CPU Reset Watchdog expired. CPUs are not in reset. Host Shutdown sequence "
            "failed {recommend checking CPLD status}");
        logResourceEvent("ResourceErrorsDetected",
                         {"Host0", "CPU Reset Watchdog expired"},
                         "xyz.openbmc_project.Logging.Entry.Level.Error");
        lg2::error(
            "Conducting cleanup: Setting GPIO states to match Host State OFF. "
            "Checking Board0RunPowerPG state and transitioning appropriately.");
        action = PowerAction::NONE;
        transitionToOffStateWithRunPowerCheck();
    }
}

void VRPowerControl::handleWaitForCPUResetAssert(Event event)
{
    logEvent(__FUNCTION__, event);

    switch (event)
    {
        case Event::cpuResetIndicatorAssert:
            handleCPUResetIndicatorAsserted();
            break;

        case Event::cpuResetWatchdogTimerExpired:
            handleCPUResetWatchdogExpired();
            break;

        default:
            lg2::info("No action taken for event: {EVENT}", "EVENT",
                      getEventName(event));
            break;
    }
}

void VRPowerControl::handleWaitForCPUResetDeAssert(Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::cpuResetIndicatorDeAssert:
            cancelTimer("CPU Reset Watchdog Timer", cpuResetWatchdogTimer);

            // Check if this is warm reboot or normal power-on
            if (action == PowerAction::FORCE_WARM_REBOOT ||
                action == PowerAction::GRACEFUL_WARM_REBOOT)
            {
                // Warm reboot complete
                lg2::info(
                    "CPU Reset Indicator de-asserted. CPUs are out of reset. Warm reboot complete! Setting Host State to On/Running. Setting GPIOs to default state for host state ON.");
                setGPIOsForHostStateOn();
            }
            else
            {
                // Normal power-on - need to set GPIOs
                lg2::info(
                    "CPU Reset Indicator de-asserted. CPUs are out of reset. Setting Host Power State to On/Running");
            }
            action = PowerAction::NONE;
            setPowerState(PowerState::on);
            break;

        case Event::cpuResetWatchdogTimerExpired:
            if (action == PowerAction::FORCE_WARM_REBOOT ||
                action == PowerAction::GRACEFUL_WARM_REBOOT)
            {
                lg2::error(
                    "Warm reboot fault: CPU_RESET_L did not de-assert within the timeout period. CPUs may still be in reset. Host Power State is stil On. Conducting cleanup: Setting GPIO states to match Host State On.");
                abortWarmRebootCpuResetWatchdogFault(
                    "Warm reboot fault: CPU_RESET_L did not de-assert within "
                    "CpuResetWatchdogMs (CPUs may still be in reset). "
                    "PRE_SYS_RST de-asserted; run power domain unchanged.");
                setGPIOsForHostStateOff();
            }
            else
            {
                lg2::error(
                    "CPU Reset Watchdog expired. CPUs are not out of reset. Host Power On sequence failed. Conducting cleanup: Setting GPIO states to match Host State OFF. Checking Board0RunPowerPG state and transitioning appropriately.");
                action = PowerAction::NONE;
                transitionToOffStateWithRunPowerCheck();
                logResourceEvent(
                    "ResourceErrorsDetected",
                    {"Host0", "CPU Reset Watchdog expired"},
                    "xyz.openbmc_project.Logging.Entry.Level.Error");
            }
            break;

        default:
            lg2::info("No action taken for event: {EVENT}", "EVENT",
                      getEventName(event));
            break;
    }
}

// ============================================================================
// HELPER FUNCTIONS for handleWaitForCPUShutdownOk
// ============================================================================

// Helper function: Check if all required boards have asserted SHDN_OK
bool VRPowerControl::areAllRequiredBoardsShutdownOk()
{
    auto board0CpuShutdownOk = getSignal("Board0CpuShutdownOk");
    if (!board0CpuShutdownOk || !board0CpuShutdownOk->gpioLine)
    {
        lg2::error("CRITICAL: Board0CpuShutdownOk not available");
        return false;
    }

    bool board0ShutdownOkAsserted = board0CpuShutdownOk->gpioLine.get_value() ==
                                    board0CpuShutdownOk->polarity;

    if (!boardPresence.board1Present)
    {
        // 1P system - only need Board 0
        return board0ShutdownOkAsserted;
    }

    // 2P system - need both Board 0 and Board 1
    auto board1CpuShutdownOk = getSignal("Board1CpuShutdownOk");
    if (!board1CpuShutdownOk || !board1CpuShutdownOk->gpioLine)
    {
        lg2::error("CRITICAL: Board1CpuShutdownOk not available");
        return false;
    }

    bool board1ShutdownOkAsserted = board1CpuShutdownOk->gpioLine.get_value() ==
                                    board1CpuShutdownOk->polarity;

    return board0ShutdownOkAsserted && board1ShutdownOkAsserted;
}

// Helper function: Get count of boards that have asserted SHDN_OK (for logging)
int VRPowerControl::getShutdownOkAssertedCount()
{
    int count = 0;

    auto board0CpuShutdownOk = getSignal("Board0CpuShutdownOk");
    if (!board0CpuShutdownOk || !board0CpuShutdownOk->gpioLine)
    {
        lg2::error("CRITICAL: Board0CpuShutdownOk not available");
        return 0;
    }

    if (board0CpuShutdownOk->gpioLine.get_value() ==
        board0CpuShutdownOk->polarity)
    {
        count++;
    }

    if (boardPresence.board1Present)
    {
        auto board1CpuShutdownOk = getSignal("Board1CpuShutdownOk");
        if (!board1CpuShutdownOk || !board1CpuShutdownOk->gpioLine)
        {
            lg2::error("CRITICAL: Board1CpuShutdownOk not available");
            return count;
        }

        if (board1CpuShutdownOk->gpioLine.get_value() ==
            board1CpuShutdownOk->polarity)
        {
            count++;
        }
    }

    return count;
}

// Helper function: Handle host-initiated shutdown
void VRPowerControl::handleHostInitiatedShutdown()
{
    // Check CPU Boot Done state to validate this is a legitimate host shutdown
    int bootDoneState = getCPUBootDoneState();

    if (bootDoneState == -1 || bootDoneState == 0)
    {
        lg2::warning(
            "Host-initiated shutdown (CPU Shutdown OK assertion) received, but CPU Boot Done is not asserted (state={STATE}). Host-initiated shutdown cannot be performed.",
            "STATE", bootDoneState);
        return; // No-op, stay in current power state
    }

    // CPU Boot Done is asserted - start observation period to distinguish
    // between reboot and shutdown
    lg2::info(
        "SHDN_OK asserted with CPU Boot Done asserted. Starting CPU Boot Done observation period to distinguish host initiated reboot from host initiated shutdown. Transitioning to PowerState::waitForCPUBootDoneDeAssert.");

    // Start observation timer - if CPU_BOOT_DONE de-asserts, it's a reboot
    // If timer expires, it's a valid shutdown
    startTimer("CpuBootDoneDeAssertDelayMs", cpuBootDoneDeAssertWatchdogTimer,
               Event::cpuBootDoneDeAssertWatchdogTimerExpired);
    setPowerState(PowerState::waitForCPUBootDoneDeAssert);
}

// Helper function: Assert Pre System Reset lines for all present boards
void VRPowerControl::assertBoardPreSystemResets()
{
    auto board0PreSystemReset = getSignal("Board0PreSystemReset");
    if (!board0PreSystemReset)
    {
        return;
    }

    setGPIOOutput(board0PreSystemReset, board0PreSystemReset->polarity);

    if (boardPresence.board1Present)
    {
        auto board1PreSystemReset = getSignal("Board1PreSystemReset");
        if (!board1PreSystemReset)
        {
            return;
        }

        setGPIOOutput(board1PreSystemReset, board1PreSystemReset->polarity);
    }
}

// Helper function: Transition to CPU reset assert wait state (success path)
void VRPowerControl::transitionToCPUResetAssertState()
{
    cancelTimer("CPU Shutdown OK Watchdog Timer", cpuShutdownOkWatchdogTimer);

    // Log appropriate message based on board configuration
    if (boardPresence.board1Present)
    {
        lg2::info(
            "CPU Shutdown OK received from both boards. Asserting Pre System Reset lines. Transitioning to wait for CPU Reset assertion...");
    }
    else
    {
        lg2::info(
            "CPU Shutdown OK received from Board 0. Asserting Pre System Reset line. Transitioning to wait for CPU Reset assertion...");
    }

    assertBoardPreSystemResets();
    startTimer("CpuResetWatchdogMs", cpuResetWatchdogTimer,
               Event::cpuResetWatchdogTimerExpired);
    setPowerState(PowerState::waitForCPUResetAssert);

#ifdef CPU_RESET_EARLY_ASSERT_WAR
    // Hardware bug workaround: CPU_RESET_L may assert before we enter this
    // state. Check if CPU reset is already asserted immediately after state
    // transition.
    auto cpuResetIndicator = getSignal("CpuResetIndicator");
    if (cpuResetIndicator && cpuResetIndicator->gpioLine)
    {
        bool cpuResetAsserted = cpuResetIndicator->gpioLine.get_value() ==
                                cpuResetIndicator->polarity;

        if (cpuResetAsserted)
        {
            lg2::info(
                "WAR: CPU Reset Indicator already asserted upon entering waitForCPUResetAssert state. Manually triggering Event::cpuResetIndicatorAssert.");
            sendPowerControlEvent(Event::cpuResetIndicatorAssert);
        }
    }
#endif // CPU_RESET_EARLY_ASSERT_WAR
}

// Helper function: Abort graceful shutdown and return to powered-on state
void VRPowerControl::abortGracefulShutdown()
{
    lg2::error(
        "Graceful shutdown aborted - CPU(s) failed to assert SHDN_OK within timeout. Returning to powered-on state.");
    action = PowerAction::NONE;
    setPowerState(
        PowerState::on); // no transition to waitForHPMPowerGoodDeAssert,
                         // because Run Power PG is already asserted, and was
                         // never de-asserted
    setGPIOsForHostStateOn();
}

void VRPowerControl::abortGracefulWarmReboot()
{
    cancelTimer("CPU Shutdown OK Watchdog Timer", cpuShutdownOkWatchdogTimer);
    lg2::error(
        "Graceful warm reboot aborted - CPU(s) failed to assert SHDN_OK within timeout. No warm reset performed. Returning to powered-on state.");
    logResourceEvent(
        "ResourceErrorsDetected",
        {"Host0", "Graceful warm reboot aborted (SHDN_OK timeout)"},
        "xyz.openbmc_project.Logging.Entry.Level.Warning");
    action = PowerAction::NONE;
    setGPIOsForHostStateOn();
    setPowerState(PowerState::on);
}

// Helper function: Handle CPU Shutdown OK watchdog expiry during FORCE_OFF
void VRPowerControl::handleCPUShutdownOkWatchdogExpiry_ForceOff()
{
    // FORCE_OFF: Don't care about SHDN_OK state, just proceed
    lg2::info(
        "CPU Shutdown OK watchdog expired during FORCE_OFF. Proceeding with forced power down.");
    assertBoardPreSystemResets();
    startTimer("CpuResetWatchdogMs", cpuResetWatchdogTimer,
               Event::cpuResetWatchdogTimerExpired);
    setPowerState(PowerState::waitForCPUResetAssert);
}

void VRPowerControl::handleCPUShutdownOkWatchdogExpiry_ForceWarmReboot()
{
    lg2::info(
        "CPU Shutdown OK watchdog expired during Force Warm Reboot. Proceeding with Force Warm Reboot.");
    initiateForceWarmReboot();
}

// Helper function: Handle CPU Shutdown OK watchdog expiry during GRACE_OFF
void VRPowerControl::handleCPUShutdownOkWatchdogExpiry_GraceOff()
{
    // GRACE_OFF: Check how many boards asserted SHDN_OK
    int assertedCount = getShutdownOkAssertedCount();

    if (!boardPresence.board1Present)
    {
        // 1P system
        if (assertedCount == 0)
        {
            // Board 0 did not assert SHDN_OK - abort graceful shutdown
            lg2::error(
                "CPU Shutdown OK watchdog expired during Host Graceful Shutdown sequence. Board 0 CPU failed to assert SHDN_OK.");
            logResourceEvent(
                "ResourceErrorsDetected",
                {"Host0",
                 "CPU Shutdown OK watchdog expired (Board 0 did not assert SHDN_OK)"},
                "xyz.openbmc_project.Logging.Entry.Level.Error");
            abortGracefulShutdown();
        }
        else
        {
            // Board 0 asserted SHDN_OK - proceed (shouldn't normally reach here
            // as we'd transition earlier)
            lg2::info(
                "Board 0 CPU Shutdown OK confirmed. Proceeding with graceful power down.");
            transitionToCPUResetAssertState();
        }
    }
    else
    {
        // 2P system
        if (assertedCount == 0)
        {
            // Neither board asserted SHDN_OK - abort graceful shutdown
            lg2::error(
                "CPU Shutdown OK watchdog expired during GRACE_OFF. Neither CPU asserted SHDN_OK.");
            logResourceEvent(
                "ResourceErrorsDetected",
                {"Host0",
                 "CPU Shutdown OK watchdog expired (neither CPU asserted SHDN_OK)"},
                "xyz.openbmc_project.Logging.Entry.Level.Error");
            abortGracefulShutdown();
        }
        else if (assertedCount == 1)
        {
            // Only one board asserted SHDN_OK - system in bad state, proceed
            // anyway
            bool board0Asserted = false;
            auto board0CpuShutdownOk = getSignal("Board0CpuShutdownOk");
            if (!board0CpuShutdownOk || !board0CpuShutdownOk->gpioLine)
            {
                lg2::error("CRITICAL: Board0CpuShutdownOk not available");
            }
            else
            {
                board0Asserted = board0CpuShutdownOk->gpioLine.get_value() ==
                                 board0CpuShutdownOk->polarity;
            }

            if (board0Asserted)
            {
                lg2::warning(
                    "CPU Shutdown OK watchdog expired during Host Graceful Shutdown sequence. Only Board 0 asserted SHDN_OK in 2P configuration. System in bad state - proceeding with host shutdown. Asserting Pre System Reset lines and transitioning to PowerState::waitForCPUResetAssert.");
                logResourceEvent(
                    "ResourceErrorsDetected",
                    {"Host0",
                     "CPU Shutdown OK watchdog expired (only Board 0 asserted SHDN_OK in 2P configuration). Continuing with host shutdown."},
                    "xyz.openbmc_project.Logging.Entry.Level.Warning");
            }
            else
            {
                lg2::warning(
                    "CPU Shutdown OK watchdog expired during Host Graceful Shutdown sequence. Only Board 1 asserted SHDN_OK in 2P configuration. System in bad state - proceeding with host shutdown. Asserting Pre System Reset lines and transitioning to PowerState::waitForCPUResetAssert.");
                logResourceEvent(
                    "ResourceErrorsDetected",
                    {"Host0",
                     "CPU Shutdown OK watchdog expired (only Board 1 asserted SHDN_OK in 2P configuration). Continuing with host shutdown."},
                    "xyz.openbmc_project.Logging.Entry.Level.Warning");
            }

            assertBoardPreSystemResets();
            startTimer("CpuResetWatchdogMs", cpuResetWatchdogTimer,
                       Event::cpuResetWatchdogTimerExpired);
            setPowerState(PowerState::waitForCPUResetAssert);
        }
        else
        {
            // Both boards asserted SHDN_OK - proceed (shouldn't normally reach
            // here as we'd transition earlier)
            lg2::info(
                "Both CPUs confirmed Shutdown OK. Proceeding with graceful power down.");
            transitionToCPUResetAssertState();
        }
    }
}

// ============================================================================
// handleWaitForCPUShutdownOk state handler
// ============================================================================

void VRPowerControl::handleForceOffDuringGracefulCpuShutdownOkWait()
{
    lg2::info(
        "Forceful shutdown during graceful wait for CPU Shutdown OK; upgrading to forceful shutdown sequence");

    action = PowerAction::FORCE_OFF;
    handleShutdownRequest(Event::powerOffRequest);
}

void VRPowerControl::handleForceWarmRebootDuringGracefulCpuShutdownOkWait()
{
    lg2::info(
        "Force warm reboot during graceful warm reboot wait for CPU Shutdown OK; upgrading to force warm reboot sequence");

    cancelTimer("CPU Shutdown OK Watchdog Timer", cpuShutdownOkWatchdogTimer);

    auto board0Req = getSignal("Board0CpuShutdownRequest");
    if (board0Req)
    {
        setGPIOOutput(board0Req, !board0Req->polarity);
    }

    initiateForceWarmReboot();
}

void VRPowerControl::handleWaitForCPUShutdownOk(Event event)
{
    switch (event)
    {
        case Event::powerOffRequest:
            if (action == PowerAction::GRACE_OFF ||
                action == PowerAction::GRACEFUL_POWER_CYCLE ||
                action == PowerAction::GRACEFUL_WARM_REBOOT)
            {
                handleForceOffDuringGracefulCpuShutdownOkWait();
            }
            else
            {
                lg2::info("No action taken for event: {EVENT}", "EVENT",
                          getEventName(event));
            }
            break;

        case Event::resetRequest:
            if (action == PowerAction::GRACEFUL_WARM_REBOOT)
            {
                handleForceWarmRebootDuringGracefulCpuShutdownOkWait();
            }
            else
            {
                lg2::info("No action taken for event: {EVENT}", "EVENT",
                          getEventName(event));
            }
            break;

        case Event::board0CpuShutdownOkAssert:
        case Event::board1CpuShutdownOkAssert:
            // Check if all required boards have now asserted SHDN_OK
            if (areAllRequiredBoardsShutdownOk())
            {
                // All required boards have asserted - proceed with reset
                if (action == PowerAction::FORCE_WARM_REBOOT)
                {
                    initiateForceWarmReboot();
                }
                else
                {
                    transitionToCPUResetAssertState();
                }
            }
            else
            {
                // Still waiting for other board(s) in 2P configuration
                if (event == Event::board0CpuShutdownOkAssert)
                {
                    lg2::info(
                        "Board 0 CPU Shutdown OK asserted. Waiting for Board 1...");
                }
                else
                {
                    lg2::info(
                        "Board 1 CPU Shutdown OK asserted. Waiting for Board 0...");
                }
            }
            break;

        case Event::cpuShutdownOkWatchdogTimerExpired:
            // Behavior depends on power action (FORCE_OFF vs GRACE_OFF vs
            // POWER_CYCLE/GRACEFUL_POWER_CYCLE)
            if (action == PowerAction::FORCE_OFF ||
                action == PowerAction::POWER_CYCLE)
            {
                // Both FORCE_OFF and POWER_CYCLE use forceful shutdown
                handleCPUShutdownOkWatchdogExpiry_ForceOff();
            }
            else if (action == PowerAction::FORCE_WARM_REBOOT)
            {
                handleCPUShutdownOkWatchdogExpiry_ForceWarmReboot();
            }
            else if (action == PowerAction::GRACE_OFF ||
                     action == PowerAction::GRACEFUL_POWER_CYCLE)
            {
                // Both GRACE_OFF and GRACEFUL_POWER_CYCLE use graceful shutdown
                handleCPUShutdownOkWatchdogExpiry_GraceOff();
            }
            else if (action == PowerAction::GRACEFUL_WARM_REBOOT)
            {
                abortGracefulWarmReboot();
            }
            else
            {
                // Unknown action - log and do nothing
                lg2::warning(
                    "CPU Shutdown OK watchdog expired with unexpected power action. No action taken.");
                logResourceEvent(
                    "ResourceErrorsDetected",
                    {"Host0",
                     "CPU Shutdown OK watchdog expired (unexpected power action). No action taken."},
                    "xyz.openbmc_project.Logging.Entry.Level.Warning");
            }
            break;

        default:
            lg2::info("No action taken for event: {EVENT}", "EVENT",
                      getEventName(event));
            break;
    }
}

// ============================================================================
// handleWaitForCPUBootDoneDeAssert state handler
// ============================================================================

void VRPowerControl::handleHostInitiatedReboot()
{
    lg2::info(
        "Host Initiated Reboot detected. Pulsing HostState to Off for 1 second while keeping ChassisState On.");

    // Set action before transitioning so getChassisState() returns On
    // even while powerState is off (see VRPowerControl::getChassisState)
    action = PowerAction::HOST_INITIATED_REBOOT;
    setPowerState(PowerState::off);

    // After 1 second, restore power state to on
    hostInitiatedRebootPulseTimer.expires_after(std::chrono::seconds(1));
    hostInitiatedRebootPulseTimer.async_wait([this](
                                                 const boost::system::error_code
                                                     ec) {
        if (ec)
        {
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error(
                    "Host initiated reboot pulse timer failed: {ERROR_MSG}",
                    "ERROR_MSG", ec.message());
                logResourceEvent(
                    "ResourceErrorsDetected",
                    {"Host0", "Host initiated reboot pulse timer failed"},
                    "xyz.openbmc_project.Logging.Entry.Level.Warning");
            }
            return;
        }
        lg2::info(
            "Host initiated reboot pulse complete. Returning to PowerState::on.");
        action = PowerAction::NONE;
        setPowerState(PowerState::on);
    });
}

void VRPowerControl::initiateHostInitiatedShutdown()
{
    lg2::info(
        "CPU Boot Done remained asserted during observation period. Valid host-initiated shutdown confirmed. Asserting Pre System Reset lines. Starting CPU Reset Watchdog Timer. Transitioning to PowerState::waitForCPUResetAssert.");

    action = PowerAction::HOST_INITIATED_SHUTDOWN;
    assertBoardPreSystemResets();
    startTimer("CpuResetWatchdogMs", cpuResetWatchdogTimer,
               Event::cpuResetWatchdogTimerExpired);
    setPowerState(PowerState::waitForCPUResetAssert);
}

void VRPowerControl::handleWaitForCPUBootDoneDeAssert(Event event)
{
    switch (event)
    {
        case Event::cpuBootDoneDeAssert:
            cancelTimer("CPU Boot Done De-Assert Watchdog Timer",
                        cpuBootDoneDeAssertWatchdogTimer);
            lg2::info(
                "CPU Boot Done de-asserted after SHDN_OK. Confirmed host-initiated reboot. Initiating host-initiated reboot.");
            handleHostInitiatedReboot();
            break;

        case Event::cpuBootDoneDeAssertWatchdogTimerExpired:
            // Timer expired, CPU_BOOT_DONE stayed asserted - valid shutdown!
            initiateHostInitiatedShutdown();
            break;

        default:
            lg2::info("No action taken for event: {EVENT}", "EVENT",
                      getEventName(event));
            break;
    }
}

void VRPowerControl::initiateWaitForHostRebootShutdownOk()
{
    setPowerState(PowerState::waitForHostRebootShutdownOk);
    startTimer("HostRebootShutdownOkDelayMs", hostRebootShutdownOkTimer,
               Event::hostRebootShutdownOkTimerExpired);
    lg2::info(
        "CPU Boot Done de-asserted before SHDN_OK. Waiting for SHDN_OK assertion to confirm host-initiated reboot. Transitioning to PowerState::waitForHostRebootShutdownOk.");
}

void VRPowerControl::logPartialShutdownOkWarning()
{
    bool board0Asserted = false;
    bool board1Asserted = false;

    auto board0CpuShutdownOk = getSignal("Board0CpuShutdownOk");
    if (board0CpuShutdownOk && board0CpuShutdownOk->gpioLine)
    {
        board0Asserted = board0CpuShutdownOk->gpioLine.get_value() ==
                         board0CpuShutdownOk->polarity;
    }

    auto board1CpuShutdownOk = getSignal("Board1CpuShutdownOk");
    if (board1CpuShutdownOk && board1CpuShutdownOk->gpioLine)
    {
        board1Asserted = board1CpuShutdownOk->gpioLine.get_value() ==
                         board1CpuShutdownOk->polarity;
    }

    lg2::warning(
        "HostRebootShutdownOkDelayMs timeout: only 1 of 2 boards asserted SHDN_OK. Board 0: {B0}, Board 1: {B1}. Proceeding with host-initiated reboot.",
        "B0", board0Asserted ? "asserted" : "not asserted", "B1",
        board1Asserted ? "asserted" : "not asserted");
}

void VRPowerControl::handleWaitForHostRebootShutdownOk(Event event)
{
    switch (event)
    {
        case Event::board0CpuShutdownOkAssert:
        case Event::board1CpuShutdownOkAssert:
            if (areAllRequiredBoardsShutdownOk())
            {
                cancelTimer("Host Reboot Shutdown OK Timer",
                            hostRebootShutdownOkTimer);
                lg2::info(
                    "All required boards have asserted SHDN_OK. Initiating host-initiated reboot.");
                handleHostInitiatedReboot();
            }
            else
            {
                // 2P system: one board has asserted, still waiting for the
                // other
                if (event == Event::board0CpuShutdownOkAssert)
                {
                    lg2::info(
                        "Board 0 CPU Shutdown OK asserted. Waiting for Board 1 CPU Shutdown OK.");
                }
                else
                {
                    lg2::info(
                        "Board 1 CPU Shutdown OK asserted. Waiting for Board 0 CPU Shutdown OK.");
                }
            }
            break;

        case Event::hostRebootShutdownOkTimerExpired:
        {
            int assertedCount = getShutdownOkAssertedCount();
            if (assertedCount == 0)
            {
                lg2::info(
                    "No SHDN_OK received within HostRebootShutdownOkDelayMs window. Not a host-initiated reboot. Returning to PowerState::on.");
                setPowerState(PowerState::on);
            }
            else
            {
                // assertedCount == 1: only warn about partial SHDN_OK on 2P
                // systems; on 1P this is the expected full assertion count
                if (boardPresence.board1Present)
                {
                    logPartialShutdownOkWarning();
                }
                handleHostInitiatedReboot();
            }
            break;
        }

        default:
            lg2::info("No action taken for event: {EVENT}", "EVENT",
                      getEventName(event));
            break;
    }
}

void VRPowerControl::handleWaitForPowerCycleDelay(Event event)
{
    switch (event)
    {
        case Event::powerCycleDelayTimerExpired:
            lg2::info(
                "Power cycle delay complete. Initiating power on sequence");
            cancelTimer("Power Cycle Delay Timer", powerCycleDelayTimer);
            setPowerState(PowerState::off);
            // Keep action = POWER_CYCLE - will be cleared when we reach On
            // state Delegate to base/platform handlePowerStateOff to trigger
            // power on
            sendPowerControlEvent(Event::powerOnRequest);
            break;

        default:
            // Reject all other events during power cycle delay
            lg2::warning(
                "Event {EVENT} rejected - power cycle delay in progress",
                "EVENT", getEventName(event));
            break;
    }
}

void VRPowerControl::handleWaitForRebootDelay(Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::warmRebootDelayTimerExpired:
            if (action == PowerAction::GRACEFUL_WARM_REBOOT)
            {
                lg2::info(
                    "Graceful warm reboot delay complete - de-asserting Pre System Reset signals");
            }
            else if (action == PowerAction::FORCE_WARM_REBOOT)
            {
                lg2::info(
                    "Force warm reboot delay complete - de-asserting Pre System Reset signals");
            }
            else
            {
                lg2::info(
                    "Warm reboot delay complete - de-asserting Pre System Reset signals");
            }
            cancelTimer("Warm Reboot Delay Timer", warmRebootDelayTimer);

            // De-assert Pre System Reset signals (Board 0 and Board 1 if
            // present)
            deassertPreSystemResets();

            // Start CPU Reset De-Assert watchdog
            startTimer("CpuResetWatchdogMs", cpuResetWatchdogTimer,
                       Event::cpuResetWatchdogTimerExpired);

            setPowerState(PowerState::waitForCPUResetDeAssert);
            break;

        default:
            // Reject all other events during warm reboot delay
            lg2::warning(
                "Event {EVENT} rejected - warm reboot delay in progress",
                "EVENT", getEventName(event));
            break;
    }
}

std::string_view VRPowerControl::getHostState() const
{
    // VR-specific implementation - maps VR PowerState extensions to D-Bus host
    // state
    switch (powerState)
    {
        case PowerState::waitForPDBMainPowerOk:
        case PowerState::waitForHPMPowerGoodAssert:
        case PowerState::waitForCPUResetDeAssert:
            return "xyz.openbmc_project.State.Host.HostState.TransitioningToRunning";
            break;
        case PowerState::waitForCPUResetAssert:
        case PowerState::waitForCPUShutdownOk:
            return "xyz.openbmc_project.State.Host.HostState.TransitioningToOff";
            break;
        case PowerState::waitForCPUBootDoneDeAssert:
        case PowerState::waitForHostRebootShutdownOk:
            // During observation period, system is still ON
            // We haven't started shutdown yet - just determining if it's reboot
            // or shutdown
            return "xyz.openbmc_project.State.Host.HostState.Running";
            break;
        case PowerState::waitForPowerCycleDelay:
        case PowerState::waitForRebootDelay:
            // During power cycle delay or warm reboot delay, host is Off
            return "xyz.openbmc_project.State.Host.HostState.Off";
            break;
        case PowerState::waitForHPMPowerGoodDeAssert:
            return "xyz.openbmc_project.State.Host.HostState.Off";
            break;
        case PowerState::waitForPDBMainPowerOff:
            if (action == PowerAction::POWER_ON)
            {
                return "xyz.openbmc_project.State.Host.HostState.TransitioningToRunning";
            }
            else if (action == PowerAction::FORCE_OFF ||
                     action == PowerAction::GRACE_OFF ||
                     action == PowerAction::POWER_CYCLE ||
                     action == PowerAction::GRACEFUL_POWER_CYCLE ||
                     action == PowerAction::HOST_INITIATED_SHUTDOWN)
            {
                return "xyz.openbmc_project.State.Host.HostState.Off";
            }
            else
            {
                lg2::error(
                    "getHostState: waitForPDBMainPowerOff with unexpected action, defaulting to Off");
                return "xyz.openbmc_project.State.Host.HostState.Off";
            }
        default:
            break;
    }

    // Fall through to base class for upstream states
    return PowerControl::getHostState();
}

std::string_view VRPowerControl::getChassisState() const
{
    // VR-specific implementation - maps VR PowerState extensions to D-Bus
    // chassis state
    switch (powerState)
    {
        case PowerState::waitForPDBMainPowerOk:
        case PowerState::waitForHPMPowerGoodAssert:
            return "xyz.openbmc_project.State.Chassis.PowerState.TransitioningToOn";
            break;
        case PowerState::waitForHPMPowerGoodDeAssert:
        case PowerState::waitForCPUShutdownOk:
            return "xyz.openbmc_project.State.Chassis.PowerState.TransitioningToOff";
            break;
        case PowerState::waitForCPUBootDoneDeAssert:
        case PowerState::waitForHostRebootShutdownOk:
            // During observation period, chassis is still ON
            // We haven't started shutdown yet - just determining if it's reboot
            // or shutdown
            return "xyz.openbmc_project.State.Chassis.PowerState.On";
            break;
        case PowerState::off:
            // During a host-initiated reboot, the chassis never lost power so
            // CurrentPowerState must stay On while we pulse CurrentHostState
            // Off for 1 second to signal boot code collection services
            if (action == PowerAction::HOST_INITIATED_REBOOT)
            {
                return "xyz.openbmc_project.State.Chassis.PowerState.On";
            }
            break;
        case PowerState::waitForCPUResetAssert:
            // For warm reboot, chassis stays On (no power cycle)
            // For shutdown, chassis is transitioning to off
            if (action == PowerAction::FORCE_WARM_REBOOT ||
                action == PowerAction::GRACEFUL_WARM_REBOOT)
            {
                return "xyz.openbmc_project.State.Chassis.PowerState.On";
            }
            return "xyz.openbmc_project.State.Chassis.PowerState.TransitioningToOff";
            break;
        case PowerState::waitForPowerCycleDelay:
            return "xyz.openbmc_project.State.Chassis.PowerState.Off";
            break;
        case PowerState::waitForRebootDelay:
        case PowerState::waitForCPUResetDeAssert:
            // During warm reboot delay and CPU reset de-assert, chassis stays
            // On
            return "xyz.openbmc_project.State.Chassis.PowerState.On";
            break;
        case PowerState::waitForPDBMainPowerOff:
            if (action == PowerAction::POWER_ON)
            {
                return "xyz.openbmc_project.State.Chassis.PowerState.TransitioningToOn";
            }
            else if (action == PowerAction::FORCE_OFF ||
                     action == PowerAction::GRACE_OFF ||
                     action == PowerAction::POWER_CYCLE ||
                     action == PowerAction::GRACEFUL_POWER_CYCLE ||
                     action == PowerAction::HOST_INITIATED_SHUTDOWN)
            {
                return "xyz.openbmc_project.State.Chassis.PowerState.TransitioningToOff";
            }
            else
            {
                lg2::error(
                    "getChassisState: waitForPDBMainPowerOff with unexpected action, defaulting to Off");
                return "xyz.openbmc_project.State.Chassis.PowerState.Off";
            }
        default:
            break;
    }

    // Fall through to base class for upstream states
    return PowerControl::getChassisState();
}

std::string VRPowerControl::getPowerStateName() const
{
    // VR-specific state name mappings
    // TODO: Confirm  VR-specific logic

    switch (powerState)
    {
        case PowerState::waitForPDBMainPowerOk:
            return "Wait for PDB Main Power OK";
            break;
        case PowerState::waitForPDBMainPowerOff:
            return "Wait for PDB Main Power Off";
            break;
        case PowerState::waitForHPMPowerGoodAssert:
            return "Wait for HPM Power Good Assert";
            break;
        case PowerState::waitForHPMPowerGoodDeAssert:
            return "Wait for HPM Power Good De-Assert";
            break;
        case PowerState::waitForCPUResetAssert:
            return "Wait for CPU Reset Assert";
            break;
        case PowerState::waitForCPUResetDeAssert:
            return "Wait for CPU Reset De-Assert";
            break;
        case PowerState::waitForCPUShutdownOk:
            return "Wait for CPU Shutdown OK";
            break;
        case PowerState::waitForCPUBootDoneDeAssert:
            return "Wait for CPU Boot Done De-Assert";
            break;
        case PowerState::waitForHostRebootShutdownOk:
            return "Wait For Host Reboot Shutdown OK";
            break;
        case PowerState::waitForPowerCycleDelay:
            return "Wait for Power Cycle Delay";
            break;
        case PowerState::waitForRebootDelay:
            return "Wait for Reboot Delay";
            break;
        default:
            // Fall through to base class for upstream states
            break;
    }

    // Call base class for upstream states
    return PowerControl::getPowerStateName();
}

// validateRequiredSignals() is inherited from PowerControl base class.
// VR signals are added via addRequiredSignal() in constructor.

void VRPowerControl::validateTimerConfigs()
{
    // VR validates VR/HPM common timers (does NOT call base class)
    for (const auto& timerName : vrRequiredTimeoutValues)
    {
        if (TimerMap.find(timerName) == TimerMap.end())
        {
            lg2::error("Required VR timer config '{TIMER}' not found in config",
                       "TIMER", timerName);
            throw std::runtime_error(
                "VRPowerControl: Required timer config missing: " + timerName);
        }
    }

    lg2::info(
        "VR timer configuration validation complete - all required timers present");
}

void VRPowerControl::setDefaultValues()
{
    lg2::info("Initializing default values for VR output signals");

    // Find and validate all required signals first
    auto board0RunPowerEnable = getSignal("Board0RunPowerEnable");
    if (!board0RunPowerEnable)
    {
        return;
    }

    auto board0PreSystemReset = getSignal("Board0PreSystemReset");
    if (!board0PreSystemReset)
    {
        return;
    }

    auto board0CpuShutdownForce = getSignal("Board0CpuShutdownForce");
    if (!board0CpuShutdownForce)
    {
        return;
    }

    auto board0CpuShutdownRequest = getSignal("Board0CpuShutdownRequest");
    if (!board0CpuShutdownRequest)
    {
        return;
    }

    auto usbPowerEnable = getSignal("USBPowerEnable");
    if (!usbPowerEnable)
    {
        return;
    }

    // Board 1 signals (if present)
    std::shared_ptr<ConfigData> board1RunPowerEnable;
    std::shared_ptr<ConfigData> board1PreSystemReset;

    if (boardPresence.board1Present)
    {
        board1RunPowerEnable = getSignal("Board1RunPowerEnable");
        if (!board1RunPowerEnable)
        {
            return;
        }

        board1PreSystemReset = getSignal("Board1PreSystemReset");
        if (!board1PreSystemReset)
        {
            return;
        }
    }

    // All signals validated, now set the default states
    board0RunPowerEnable->defaultStateHostStateOn = DefaultState::Asserted;
    board0RunPowerEnable->defaultStateHostStateOff = DefaultState::DeAsserted;

    board0PreSystemReset->defaultStateHostStateOn = DefaultState::DeAsserted;
    board0PreSystemReset->defaultStateHostStateOff = DefaultState::Asserted;

    board0CpuShutdownForce->defaultStateHostStateOn = DefaultState::DeAsserted;
    board0CpuShutdownForce->defaultStateHostStateOff = DefaultState::DeAsserted;

    board0CpuShutdownRequest->defaultStateHostStateOn =
        DefaultState::DeAsserted;
    board0CpuShutdownRequest->defaultStateHostStateOff =
        DefaultState::DeAsserted;

    usbPowerEnable->defaultStateHostStateOn = DefaultState::Asserted;
    usbPowerEnable->defaultStateHostStateOff = DefaultState::DeAsserted;

    if (boardPresence.board1Present)
    {
        board1RunPowerEnable->defaultStateHostStateOn = DefaultState::Asserted;
        board1RunPowerEnable->defaultStateHostStateOff =
            DefaultState::DeAsserted;

        board1PreSystemReset->defaultStateHostStateOn =
            DefaultState::DeAsserted;
        board1PreSystemReset->defaultStateHostStateOff = DefaultState::Asserted;
    }

    lg2::info("VR default values set successfully");
}

} // namespace power_control
