/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#include "config.h"

#include "vr_power_control.hpp"

#include <phosphor-logging/lg2.hpp>

#include <filesystem>

// External references to global variables from power_control.cpp
namespace power_control
{
extern PowerState powerState;
}

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
    hpmPowerGoodWatchdogTimer(ioContext),
    cpuResetWatchdogTimer(ioContext),
    cpuShutdownOkWatchdogTimer(ioContext),
    powerCycleDelayTimer(ioContext),
    warmRebootDelayTimer(ioContext)
{
    // powerSignalMap is now populated by PowerControl::loadConfigValues()
    // Assign handlers and register events for common VR/HPM signals
    detectBoardPresence();

    // Add VR-specific required signals with handlers
    // Board 0 signals (always required for VR platforms)
    addRequiredSignal("Board0RunPowerEnable", 0, GPIODirection::OUT);
    addRequiredSignal("Board0RunPowerPG", 0, GPIODirection::IN, [this](bool state) {
        this->board0RunPowerPGHandler(state);
    });
    addRequiredSignal("Board0PreSystemReset", 0, GPIODirection::OUT);
    addRequiredSignal("Board0CpuShutdownForce", 0, GPIODirection::OUT);
    addRequiredSignal("Board0CpuShutdownRequest", 0, GPIODirection::OUT);
    addRequiredSignal("Board0CpuShutdownOk", 0, GPIODirection::IN, [this](bool state) {
        this->board0CpuShutdownOkHandler(state);
    });
    addRequiredSignal("CpuResetIndicator", 0, GPIODirection::IN, [this](bool state) {
        this->cpuResetIndicatorHandler(state);
    });
    addRequiredSignal("USBPowerEnable", 0, GPIODirection::OUT);

    // Add Board 1 handler if Board 1 is present
    if (boardPresence.board1Present)
    {
        addRequiredSignal("Board1CpuShutdownOk", 1, GPIODirection::IN, [this](bool state) {
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
            
            // Set GPIOs for host state OFF
            setGPIOsForHostStateOff();
            
            // Force transition to off state
            setPowerState(PowerState::off);
            
            // Return true to indicate fault was handled
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
        throw std::runtime_error(
            "Board0RunPowerPG signal not found in powerSignalMap");
    }

    auto& config = *it->second;
    
    // Update D-Bus property
    setBoard0RunPowerPGState(state);
    lg2::info("Board0RunPowerPG GPIO event: value={VALUE}",
              "VALUE", state);
    
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

void VRPowerControl::board1RunPowerPGHandler(bool state)
{
    auto it = powerSignalMap.find("Board1RunPowerPG");
    if (it == powerSignalMap.end())
    {
        throw std::runtime_error(
            "Board1RunPowerPG signal not found in powerSignalMap");
    }

    auto& config = *it->second;
    Event powerControlEvent = (state == config.polarity)
                                  ? Event::board1RunPowerPGAssert
                                  : Event::board1RunPowerPGDeAssert;
    this->sendPowerControlEvent(powerControlEvent);
}

void VRPowerControl::board0CpuShutdownOkHandler(bool state)
{
    auto it = powerSignalMap.find("Board0CpuShutdownOk");
    if (it == powerSignalMap.end())
    {
        throw std::runtime_error(
            "Board0CpuShutdownOk signal not found in powerSignalMap");
    }

    auto& config = *it->second;
    
    // Update D-Bus property
    setBoard0CpuShutdownOkState(state);
    lg2::info("Board0CpuShutdownOk GPIO event: value={VALUE}",
              "VALUE", state);
    
    Event powerControlEvent = (state == config.polarity)
                                  ? Event::board0CpuShutdownOkAssert
                                  : Event::board0CpuShutdownOkDeAssert;
    this->sendPowerControlEvent(powerControlEvent);
}

void VRPowerControl::board1CpuShutdownOkHandler(bool state)
{
    auto it = powerSignalMap.find("Board1CpuShutdownOk");
    if (it == powerSignalMap.end())
    {
        throw std::runtime_error(
            "Board1CpuShutdownOk signal not found in powerSignalMap");
    }

    auto& config = *it->second;
    
    // Update D-Bus property
    setBoard1CpuShutdownOkState(state);
    lg2::info("Board1CpuShutdownOk GPIO event: value={VALUE}",
              "VALUE", state);
    
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
        throw std::runtime_error(
            "CpuResetIndicator signal not found in powerSignalMap");
    }

    auto& config = *it->second;
    
    // Update D-Bus property
    setCpuResetIndicatorState(state);
    lg2::info("CpuResetIndicator GPIO event: value={VALUE}",
              "VALUE", state);
    
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
    lg2::info("Force Warm Reboot request received - asserting Pre System Reset signals");
    action = PowerAction::FORCE_WARM_REBOOT;

    // Assert Pre System Reset for Board 0
    auto board0PreSystemReset = powerSignalMap.find("Board0PreSystemReset");
    if (board0PreSystemReset == powerSignalMap.end())
    {
        throw std::runtime_error(
            "Board0PreSystemReset signal not found in powerSignalMap");
    }
    setGPIOOutput(board0PreSystemReset->second,
                  board0PreSystemReset->second->polarity);

    // Assert Pre System Reset for Board 1 if present
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
        throw std::runtime_error(
            "Board0PreSystemReset signal not found in powerSignalMap");
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
}

// Helper function: Transition to CPU Reset Assert wait state
void VRPowerControl::transitionToCPUResetDeAssertState()
{
    hpmPowerGoodWatchdogTimer.cancel();

    lg2::info(
        "HPM Board 0 Run Power Good Asserted. De-asserting Pre System Resets. Starting CPU Reset Watchdog Timer. Transitioning to PowerState::waitForCPUResetDeAssert.");

    deassertPreSystemResets();
    startTimer(TimerMap["CpuResetWatchdogMs"], cpuResetWatchdogTimer,
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
                "HPM Power Good Watchdog Timer Expired. Host Power On sequence failed. Conducting Cleanup Sequence: Setting GPIO states to match Host State OFF. Setting Host Power State to Off.");

            setGPIOsForHostStateOff(); // TODO: fill function implementation
            action = PowerAction::NONE;
            setPowerState(PowerState::off);
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
            cpuResetWatchdogTimer.cancel();
            
            // Check if this is warm reboot or normal power-on
            if (action == PowerAction::FORCE_WARM_REBOOT)
            {
                // Warm reboot complete - GPIOs already in correct state
                lg2::info("CPU Reset Indicator de-asserted. CPUs are out of reset. Warm reboot complete! Setting Host State to On/Running.");
            }
            else
            {
                // Normal power-on - need to set GPIOs
                lg2::info("CPU Reset Indicator de-asserted. CPUs are out of reset. Setting Host Power State to On/Running");
            }
            setGPIOsForHostStateOn();
            action = PowerAction::NONE;
            setPowerState(PowerState::on);
            break;
            
        case Event::cpuResetWatchdogTimerExpired:
            if (action == PowerAction::FORCE_WARM_REBOOT)
            {
                lg2::error("CPU Reset Watchdog expired during warm reboot. CPUs did not come out of reset. Conducting cleanup: Setting GPIO states to match Host State OFF. Setting Host Power State to Off.");
            }
            else
            {
                lg2::error("CPU Reset Watchdog expired. CPUs are not out of reset. Host Power On sequence failed. Conducting cleanup: Setting GPIO states to match Host State OFF. Setting Host Power State to Off.");
            }
            
            setGPIOsForHostStateOff();
            action = PowerAction::NONE;
            setPowerState(PowerState::off);
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
    auto board0CpuShutdownOk = powerSignalMap.find("Board0CpuShutdownOk");
    if (board0CpuShutdownOk == powerSignalMap.end())
    {
        lg2::error("CRITICAL: Board0CpuShutdownOk signal not found in powerSignalMap");
        return false;
    }
    if (!board0CpuShutdownOk->second->gpioLine)
    {
        lg2::error("CRITICAL: Board0CpuShutdownOk GPIO line not initialized");
        return false;
    }

    bool board0ShutdownOkAsserted =
        board0CpuShutdownOk->second->gpioLine.get_value() ==
        board0CpuShutdownOk->second->polarity;

    if (!boardPresence.board1Present)
    {
        // 1P system - only need Board 0
        return board0ShutdownOkAsserted;
    }

    // 2P system - need both Board 0 and Board 1
    auto board1CpuShutdownOk = powerSignalMap.find("Board1CpuShutdownOk");
    if (board1CpuShutdownOk == powerSignalMap.end())
    {
        lg2::error("CRITICAL: Board1CpuShutdownOk signal not found in powerSignalMap");
        return false;
    }
    if (!board1CpuShutdownOk->second->gpioLine)
    {
        lg2::error("CRITICAL: Board1CpuShutdownOk GPIO line not initialized");
        return false;
    }

    bool board1ShutdownOkAsserted =
        board1CpuShutdownOk->second->gpioLine.get_value() ==
        board1CpuShutdownOk->second->polarity;

    return board0ShutdownOkAsserted && board1ShutdownOkAsserted;
}

// Helper function: Get count of boards that have asserted SHDN_OK (for logging)
int VRPowerControl::getShutdownOkAssertedCount()
{
    int count = 0;

    auto board0CpuShutdownOk = powerSignalMap.find("Board0CpuShutdownOk");
    if (board0CpuShutdownOk == powerSignalMap.end())
    {
        lg2::error("CRITICAL: Board0CpuShutdownOk signal not found in powerSignalMap");
        return 0;
    }
    if (!board0CpuShutdownOk->second->gpioLine)
    {
        lg2::error("CRITICAL: Board0CpuShutdownOk GPIO line not initialized");
        return 0;
    }

    if (board0CpuShutdownOk->second->gpioLine.get_value() ==
        board0CpuShutdownOk->second->polarity)
    {
        count++;
    }

    if (boardPresence.board1Present)
    {
        auto board1CpuShutdownOk = powerSignalMap.find("Board1CpuShutdownOk");
        if (board1CpuShutdownOk == powerSignalMap.end())
        {
            lg2::error("CRITICAL: Board1CpuShutdownOk signal not found in powerSignalMap");
            return count;
        }
        if (!board1CpuShutdownOk->second->gpioLine)
        {
            lg2::error("CRITICAL: Board1CpuShutdownOk GPIO line not initialized");
            return count;
        }

        if (board1CpuShutdownOk->second->gpioLine.get_value() ==
            board1CpuShutdownOk->second->polarity)
        {
            count++;
        }
    }

    return count;
}

// Helper function: Assert Pre System Reset lines for all present boards
void VRPowerControl::assertBoardPreSystemResets()
{
    auto board0PreSystemReset = powerSignalMap.find("Board0PreSystemReset");
    if (board0PreSystemReset == powerSignalMap.end())
    {
        throw std::runtime_error(
            "Board0PreSystemReset signal not found in powerSignalMap");
    }

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
}

// Helper function: Transition to CPU reset assert wait state (success path)
void VRPowerControl::transitionToCPUResetAssertState()
{
    cpuShutdownOkWatchdogTimer.cancel();

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
    startTimer(TimerMap["CpuResetWatchdogMs"], cpuResetWatchdogTimer,
               Event::cpuResetWatchdogTimerExpired);
    setPowerState(PowerState::waitForCPUResetAssert);
}

// Helper function: Abort graceful shutdown and return to powered-on state
void VRPowerControl::abortGracefulShutdown()
{
    lg2::error(
        "Graceful shutdown aborted - CPU(s) failed to assert SHDN_OK within timeout. Returning to powered-on state.");
    setGPIOsForHostStateOn();
    action = PowerAction::NONE;
    setPowerState(PowerState::on);
}

// Helper function: Handle CPU Shutdown OK watchdog expiry during FORCE_OFF
void VRPowerControl::handleCPUShutdownOkWatchdogExpiry_ForceOff()
{
    // FORCE_OFF: Don't care about SHDN_OK state, just proceed
    lg2::info(
        "CPU Shutdown OK watchdog expired during FORCE_OFF. Proceeding with forced power down.");
    assertBoardPreSystemResets();
    startTimer(TimerMap["CpuResetWatchdogMs"], cpuResetWatchdogTimer,
               Event::cpuResetWatchdogTimerExpired);
    setPowerState(PowerState::waitForCPUResetAssert);
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
            auto board0CpuShutdownOk =
                powerSignalMap.find("Board0CpuShutdownOk");
            if (board0CpuShutdownOk == powerSignalMap.end())
            {
                lg2::error("CRITICAL: Board0CpuShutdownOk signal not found in powerSignalMap");
            }
            else if (!board0CpuShutdownOk->second->gpioLine)
            {
                lg2::error("CRITICAL: Board0CpuShutdownOk GPIO line not initialized");
            }
            else
            {
                board0Asserted =
                    board0CpuShutdownOk->second->gpioLine.get_value() ==
                    board0CpuShutdownOk->second->polarity;
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
            startTimer(TimerMap["CpuResetWatchdogMs"], cpuResetWatchdogTimer,
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

void VRPowerControl::handleWaitForCPUShutdownOk(Event event)
{
    switch (event)
    {
        case Event::board0CpuShutdownOkAssert:
        case Event::board1CpuShutdownOkAssert:
            // Check if all required boards have now asserted SHDN_OK
            if (areAllRequiredBoardsShutdownOk())
            {
                // All required boards have asserted - proceed with reset
                transitionToCPUResetAssertState();
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
            // Behavior depends on power action (FORCE_OFF vs GRACE_OFF vs POWER_CYCLE/GRACEFUL_POWER_CYCLE)
            if (action == PowerAction::FORCE_OFF || 
                action == PowerAction::POWER_CYCLE)
            {
                // Both FORCE_OFF and POWER_CYCLE use forceful shutdown
                handleCPUShutdownOkWatchdogExpiry_ForceOff();
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

void VRPowerControl::handleWaitForPowerCycleDelay(Event event)
{
    switch (event)
    {
        case Event::powerCycleDelayTimerExpired:
            lg2::info("Power cycle delay complete. Initiating power on sequence");
            powerCycleDelayTimer.cancel();
            setPowerState(PowerState::off);
            // Keep action = POWER_CYCLE - will be cleared when we reach On state
            // Delegate to base/platform handlePowerStateOff to trigger power on
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
            lg2::info("Warm reboot delay complete - de-asserting Pre System Reset signals");
            warmRebootDelayTimer.cancel();
            
            // De-assert Pre System Reset signals (Board 0 and Board 1 if present)
            deassertPreSystemResets();
            
            // Start CPU Reset De-Assert watchdog
            startTimer("CpuResetWatchdogMs", cpuResetWatchdogTimer,
                      Event::cpuResetWatchdogTimerExpired);
            
            setPowerState(PowerState::waitForCPUResetDeAssert);
            break;

        default:
            // Reject all other events during warm reboot delay
            lg2::warning("Event {EVENT} rejected - warm reboot delay in progress",
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
            // During warm reboot delay and CPU reset de-assert, chassis stays On
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

    lg2::info("VR timer configuration validation complete - all required timers present");
}

void VRPowerControl::setDefaultValues()
{
    lg2::info("Initializing default values for VR output signals");

    // Find and validate all required signals first
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

    auto board0CpuShutdownForce =
        powerSignalMap.find("Board0CpuShutdownForce");
    if (board0CpuShutdownForce == powerSignalMap.end())
    {
        throw std::runtime_error(
            "Board0CpuShutdownForce signal not found in powerSignalMap");
    }

    auto board0CpuShutdownRequest =
        powerSignalMap.find("Board0CpuShutdownRequest");
    if (board0CpuShutdownRequest == powerSignalMap.end())
    {
        throw std::runtime_error(
            "Board0CpuShutdownRequest signal not found in powerSignalMap");
    }

    auto usbPowerEnable = powerSignalMap.find("USBPowerEnable");
    if (usbPowerEnable == powerSignalMap.end())
    {
        throw std::runtime_error(
            "USBPowerEnable signal not found in powerSignalMap");
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
            throw std::runtime_error(
                "Board1RunPowerEnable signal not found in powerSignalMap");
        }

        board1PreSystemReset = powerSignalMap.find("Board1PreSystemReset");
        if (board1PreSystemReset == powerSignalMap.end())
        {
            throw std::runtime_error(
                "Board1PreSystemReset signal not found in powerSignalMap");
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
        DefaultState::DeAsserted;

    board0CpuShutdownForce->second->defaultStateHostStateOn =
        DefaultState::DeAsserted;
    board0CpuShutdownForce->second->defaultStateHostStateOff =
        DefaultState::DeAsserted;

    board0CpuShutdownRequest->second->defaultStateHostStateOn =
        DefaultState::DeAsserted;
    board0CpuShutdownRequest->second->defaultStateHostStateOff =
        DefaultState::DeAsserted;

    usbPowerEnable->second->defaultStateHostStateOn = DefaultState::Asserted;
    usbPowerEnable->second->defaultStateHostStateOff =
        DefaultState::DeAsserted;

    if (boardPresence.board1Present)
    {
        board1RunPowerEnable->second->defaultStateHostStateOn =
            DefaultState::Asserted;
        board1RunPowerEnable->second->defaultStateHostStateOff =
            DefaultState::DeAsserted;

        board1PreSystemReset->second->defaultStateHostStateOn =
            DefaultState::DeAsserted;
        board1PreSystemReset->second->defaultStateHostStateOff =
            DefaultState::DeAsserted;
    }

    lg2::info("VR default values set successfully");
}

} // namespace power_control
