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
    powerCycleDelayTimer(ioContext), warmRebootDelayTimer(ioContext)
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

    // call validateRequiredSignals() in the platform-specific class constructor
    // validateRequiredSignals();
}

void VRPowerControl::detectBoardPresence()
{
    // Check presence and update context using paths from build configuration
    boardPresence.parsecPdbPresent = checkIOXPresence(GB300_PDB_IOX_PATH);
    boardPresence.c2PdbPresent = checkIOXPresence(C2_PDB_IOX_PATH);
    boardPresence.nvl144PdbPresent = checkIOXPresence(NVL144_PDB_IOX_PATH);
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
    lg2::info("  NVL144 PDB ({PATH}): {PRESENT}", "PATH",
              std::string(NVL144_PDB_IOX_PATH), "PRESENT",
              boardPresence.nvl144PdbPresent);
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

// =============================================================================
// GPIO EVENT HANDLERS (Member functions)
// =============================================================================

void VRPowerControl::board0RunPowerPGHandler(bool state)
{
    auto it = powerSignalMap.find("Board0RunPowerPG");
    if (it == powerSignalMap.end())
    {
        lg2::error("Board0RunPowerPG signal not found in powerSignalMap");
        return;
    }

    auto& config = *it->second;

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
    auto it = powerSignalMap.find("Board0CpuShutdownOk");
    if (it == powerSignalMap.end())
    {
        lg2::error("Board0CpuShutdownOk signal not found in powerSignalMap");
        return;
    }

    auto& config = *it->second;

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

    auto it = powerSignalMap.find("Board1CpuShutdownOk");
    if (it == powerSignalMap.end())
    {
        lg2::error("Board1CpuShutdownOk signal not found in powerSignalMap");
        return;
    }

    auto& config = *it->second;

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
    auto it = powerSignalMap.find("CpuResetIndicator");
    if (it == powerSignalMap.end())
    {
        lg2::error("CpuResetIndicator signal not found in powerSignalMap");
        return;
    }

    auto& config = *it->second;

    // Update D-Bus property
    setCpuResetIndicatorState(state);
    lg2::info("CpuResetIndicator GPIO event: value={VALUE}", "VALUE",
              static_cast<int>(state));

    Event powerControlEvent = (state == config.polarity)
                                  ? Event::cpuResetIndicatorAssert
                                  : Event::cpuResetIndicatorDeAssert;
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

        // Delegate upstream states to base class
        default:
            return PowerControl::getPowerStateHandler();
    }
}

void VRPowerControl::handlePowerStateOn(Event event)
{
    (void)event;
    // TODO: Move NVL144-specific powerStateOn() implementation here
}

void VRPowerControl::handlePowerStateOff(Event event)
{
    (void)event;
    // TODO: Move NVL144-specific powerStateOff() implementation here
}

void VRPowerControl::handleWaitForPDBMainPowerOk(Event event)
{
    (void)event;
    // TODO: Move NVL144-specific powerStateWaitForPDBMainPowerOk()
    // implementation here
}

void VRPowerControl::handleWaitForPDBMainPowerOff(Event event)
{
    (void)event;
    // TODO: Move powerStateWaitForPDBMainPowerOff() implementation here
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
    setGPIOOutput(board0CpuShutdownForce,
                  !board0CpuShutdownForce->polarity);

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
    auto board0PreSystemReset = powerSignalMap.find("Board0PreSystemReset");
    if (board0PreSystemReset == powerSignalMap.end())
    {
        lg2::error("Board0PreSystemReset signal not found in powerSignalMap");
        return;
    }
    setGPIOOutput(board0PreSystemReset->second,
                  board0PreSystemReset->second->polarity);

    // Assert Pre System Reset for Board 1 if present
    if (boardPresence.board1Present)
    {
        auto board1PreSystemReset = powerSignalMap.find("Board1PreSystemReset");
        if (board1PreSystemReset == powerSignalMap.end())
        {
            lg2::error(
                "Board1PreSystemReset signal not found in powerSignalMap");
            return;
        }
        setGPIOOutput(board1PreSystemReset->second,
                      board1PreSystemReset->second->polarity);
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
    auto board0PreSystemReset = powerSignalMap.find("Board0PreSystemReset");
    if (board0PreSystemReset == powerSignalMap.end())
    {
        lg2::error("Board0PreSystemReset signal not found in powerSignalMap");
        return;
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
            lg2::error(
                "Board1PreSystemReset signal not found in powerSignalMap");
            return;
        }

        setGPIOOutput(board1PreSystemReset->second,
                      !board1PreSystemReset->second->polarity);
    }
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
            transitionToOffStateWithRunPowerCheck();
            break;

        default:
            lg2::info("No action taken for event: {EVENT}", "EVENT",
                      getEventName(event));
            break;
    }
}

void VRPowerControl::handleWaitForHPMPowerGoodDeAssert(Event event)
{
    (void)event;
    // TODO: Move powerStateWaitForHPMPowerGoodDeAssert() implementation here
}

void VRPowerControl::handleWaitForCPUResetAssert(Event event)
{
    (void)event;
    // TODO: Move powerStateWaitForCPUResetAssert() implementation here
}

void VRPowerControl::handleWaitForCPUResetDeAssert(Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::cpuResetIndicatorDeAssert:
            cancelTimer("CPU Reset Watchdog Timer", cpuResetWatchdogTimer);

            // Check if this is warm reboot or normal power-on
            if (action == PowerAction::FORCE_WARM_REBOOT)
            {
                // Warm reboot complete - GPIOs already in correct state
                lg2::info(
                    "CPU Reset Indicator de-asserted. CPUs are out of reset. Warm reboot complete! Setting Host State to On/Running.");
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
            if (action == PowerAction::FORCE_WARM_REBOOT)
            {
                lg2::error(
                    "CPU Reset Watchdog expired during warm reboot. CPUs did not come out of reset. Conducting cleanup: Setting GPIO states to match Host State OFF. Checking Board0RunPowerPG state and transitioning appropriately.");
            }
            else
            {
                lg2::error(
                    "CPU Reset Watchdog expired. CPUs are not out of reset. Host Power On sequence failed. Conducting cleanup: Setting GPIO states to match Host State OFF. Checking Board0RunPowerPG state and transitioning appropriately.");
            }

            action = PowerAction::NONE;
            transitionToOffStateWithRunPowerCheck();
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
    auto board0PreSystemReset = powerSignalMap.find("Board0PreSystemReset");
    if (board0PreSystemReset == powerSignalMap.end())
    {
        lg2::error("Board0PreSystemReset signal not found in powerSignalMap");
        return;
    }

    setGPIOOutput(board0PreSystemReset->second,
                  board0PreSystemReset->second->polarity);

    if (boardPresence.board1Present)
    {
        auto board1PreSystemReset = powerSignalMap.find("Board1PreSystemReset");
        if (board1PreSystemReset == powerSignalMap.end())
        {
            lg2::error(
                "Board1PreSystemReset signal not found in powerSignalMap");
            return;
        }

        setGPIOOutput(board1PreSystemReset->second,
                      board1PreSystemReset->second->polarity);
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
            }
            else
            {
                lg2::warning(
                    "CPU Shutdown OK watchdog expired during Host Graceful Shutdown sequence. Only Board 1 asserted SHDN_OK in 2P configuration. System in bad state - proceeding with host shutdown. Asserting Pre System Reset lines and transitioning to PowerState::waitForCPUResetAssert.");
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
    lg2::warning(
        "Force power-off during graceful CPU Shutdown OK wait is not handled on this platform");
}

void VRPowerControl::handleWaitForCPUShutdownOk(Event event)
{
    switch (event)
    {
        case Event::powerOffRequest:
            if (action == PowerAction::GRACE_OFF ||
                action == PowerAction::GRACEFUL_POWER_CYCLE)
            {
                handleForceOffDuringGracefulCpuShutdownOkWait();
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
            else
            {
                // Unknown action - log and do nothing
                lg2::warning(
                    "CPU Shutdown OK watchdog expired with unexpected power action. No action taken.");
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

void VRPowerControl::handleWaitForCPUBootDoneDeAssert(Event event)
{
    switch (event)
    {
        case Event::cpuBootDoneDeAssert:
            // CPU_BOOT_DONE de-asserted after SHDN_OK - this is a reboot!
            cancelTimer("CPU Boot Done De-Assert Watchdog Timer",
                        cpuBootDoneDeAssertWatchdogTimer);

            lg2::info(
                "CPU Boot Done de-asserted after SHDN_OK assertion. Host Initiated Reboot operation detected. Ignoring SHDN_OK and returning to PowerState::on.");

            action = PowerAction::NONE;
            setPowerState(PowerState::on);
            break;

        case Event::cpuBootDoneDeAssertWatchdogTimerExpired:
            // Timer expired, CPU_BOOT_DONE stayed asserted - valid shutdown!
            lg2::info(
                "CPU Boot Done remained asserted during observation period. Valid host-initiated shutdown confirmed. Asserting Pre System Reset lines. Starting CPU Reset Watchdog Timer. Transitioning to PowerState::waitForCPUResetAssert.");

            // Now proceed with actual shutdown
            action = PowerAction::HOST_INITIATED_SHUTDOWN;
            assertBoardPreSystemResets();
            startTimer("CpuResetWatchdogMs", cpuResetWatchdogTimer,
                       Event::cpuResetWatchdogTimerExpired);
            setPowerState(PowerState::waitForCPUResetAssert);
            break;

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
                "EVENT", static_cast<int>(event));
            break;
    }
}

void VRPowerControl::handleWaitForRebootDelay(Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::warmRebootDelayTimerExpired:
            lg2::info(
                "Warm reboot delay complete - de-asserting Pre System Reset signals");
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
                "EVENT", static_cast<int>(event));
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
            break;
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
            // During observation period, chassis is still ON
            // We haven't started shutdown yet - just determining if it's reboot
            // or shutdown
            return "xyz.openbmc_project.State.Chassis.PowerState.On";
            break;
        case PowerState::waitForCPUResetAssert:
            // For warm reboot, chassis stays On (no power cycle)
            // For shutdown, chassis is transitioning to off
            if (action == PowerAction::FORCE_WARM_REBOOT)
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
            break;
        default:
            break;
    }

    // Fall through to base class for upstream states
    return PowerControl::getChassisState();
}

std::string VRPowerControl::getPowerStateName()
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
    auto board0RunPowerEnable = powerSignalMap.find("Board0RunPowerEnable");
    if (board0RunPowerEnable == powerSignalMap.end())
    {
        lg2::error("Board0RunPowerEnable signal not found in powerSignalMap");
        return;
    }

    auto board0PreSystemReset = powerSignalMap.find("Board0PreSystemReset");
    if (board0PreSystemReset == powerSignalMap.end())
    {
        lg2::error("Board0PreSystemReset signal not found in powerSignalMap");
        return;
    }

    auto board0CpuShutdownForce = powerSignalMap.find("Board0CpuShutdownForce");
    if (board0CpuShutdownForce == powerSignalMap.end())
    {
        lg2::error("Board0CpuShutdownForce signal not found in powerSignalMap");
        return;
    }

    auto board0CpuShutdownRequest =
        powerSignalMap.find("Board0CpuShutdownRequest");
    if (board0CpuShutdownRequest == powerSignalMap.end())
    {
        lg2::error(
            "Board0CpuShutdownRequest signal not found in powerSignalMap");
        return;
    }

    auto usbPowerEnable = powerSignalMap.find("USBPowerEnable");
    if (usbPowerEnable == powerSignalMap.end())
    {
        lg2::error("USBPowerEnable signal not found in powerSignalMap");
        return;
    }

    // Board 1 signals (if present)
    std::map<std::string, std::shared_ptr<ConfigData>>::iterator
        board1RunPowerEnable;
    std::map<std::string, std::shared_ptr<ConfigData>>::iterator
        board1PreSystemReset;

    if (boardPresence.board1Present)
    {
        board1RunPowerEnable = powerSignalMap.find("Board1RunPowerEnable");
        if (board1RunPowerEnable == powerSignalMap.end())
        {
            lg2::error(
                "Board1RunPowerEnable signal not found in powerSignalMap");
            return;
        }

        board1PreSystemReset = powerSignalMap.find("Board1PreSystemReset");
        if (board1PreSystemReset == powerSignalMap.end())
        {
            lg2::error(
                "Board1PreSystemReset signal not found in powerSignalMap");
            return;
        }
    }

    // All signals validated, now set the default states
    board0RunPowerEnable->second->defaultStateHostStateOn =
        DefaultState::Asserted;
    board0RunPowerEnable->second->defaultStateHostStateOff =
        DefaultState::DeAsserted;

    board0PreSystemReset->second->defaultStateHostStateOn =
        DefaultState::DeAsserted;
    board0PreSystemReset->second->defaultStateHostStateOff =
        DefaultState::Asserted;

    board0CpuShutdownForce->second->defaultStateHostStateOn =
        DefaultState::DeAsserted;
    board0CpuShutdownForce->second->defaultStateHostStateOff =
        DefaultState::DeAsserted;

    board0CpuShutdownRequest->second->defaultStateHostStateOn =
        DefaultState::DeAsserted;
    board0CpuShutdownRequest->second->defaultStateHostStateOff =
        DefaultState::DeAsserted;

    usbPowerEnable->second->defaultStateHostStateOn = DefaultState::Asserted;
    usbPowerEnable->second->defaultStateHostStateOff = DefaultState::DeAsserted;

    if (boardPresence.board1Present)
    {
        board1RunPowerEnable->second->defaultStateHostStateOn =
            DefaultState::Asserted;
        board1RunPowerEnable->second->defaultStateHostStateOff =
            DefaultState::DeAsserted;

        board1PreSystemReset->second->defaultStateHostStateOn =
            DefaultState::DeAsserted;
        board1PreSystemReset->second->defaultStateHostStateOff =
            DefaultState::Asserted;
    }

    lg2::info("VR default values set successfully");
}

} // namespace power_control
