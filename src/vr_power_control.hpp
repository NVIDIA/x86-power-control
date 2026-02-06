/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#pragma once

#include "power_control_base.hpp"

#include <functional>

namespace power_control
{

/**
 * @brief Placeholder struct for board presence detection
 *
 * TODO: Replace with actual implementation from coworker
 * This struct will be populated by detectBoardPresence() method
 */
struct BoardPresence
{
    bool board0Present;    // HPM Board 0 IOX
    bool board1Present;    // HPM Board 1 IOX
    bool nvl144PdbPresent; // NVL144 PDB IOX
    bool c2PdbPresent;     // C2 PDB IOX
    bool parsecPdbPresent; // Parsec/GB300 PDB IOX
};

/**
 * @brief VR Power Control class - VR-specific extensions (NVL144 default)
 *
 * This class extends PowerControl with VR-specific functionality:
 * - PDB power sequencing
 * - HPM power sequencing
 * - CPU Reset monitoring
 * - CPU Shutdown OK monitoring
 * - 1P vs 2P support
 *
 * This class handles VR PowerState enums:
 * - waitForPDBMainPowerOk
 * - waitForPDBMainPowerOff
 * - waitForHPMPowerGoodAssert
 * - waitForHPMPowerGoodDeAssert
 * - waitForCPUResetAssert
 * - waitForCPUResetDeAssert
 * - waitForCPUShutdownOk
 * - ...
 *
 * Default implementations use NVL144 behavior. Platform-specific classes
 * (C2PowerControl, ParsecPowerControl, E5010PowerControl) override as needed.
 */
class VRPowerControl : public PowerControl
{
  public:
    VRPowerControl(boost::asio::io_context& ioContext,
                   std::shared_ptr<sdbusplus::asio::connection> conn,
                   const std::string& configFilePath, const std::string& node,
                   PersistentState& appState);
    virtual ~VRPowerControl() = default;

    /**
     * @brief Get the handler function for a given power state (VR extension)
     *
     * This overrides the base class to add handlers for VR-specific states.
     * Falls back to base class for upstream states.
     *
     * @param state The power state to get a handler for
     * @return Function that handles events in the given state, or nullptr if
     * unknown
     */
    std::function<void(Event)> getPowerStateHandler() override;

    /**
     * @brief Get the host state dbus property value for a given power state (VR
     * override)
     *
     * Converts VR-specific PowerState enums to D-Bus host state property string
     * value. Falls back to base class for upstream states.
     *
     * @return D-Bus host state string
     */
    std::string_view getHostState() const override;

    /**
     * @brief Get the chassis state dbus property value for current power state
     * (VR override)
     *
     * Converts VR-specific PowerState enums to D-Bus chassis state property
     * string value. Falls back to base class for upstream states.
     *
     * @return D-Bus chassis state string
     */
    std::string_view getChassisState() const override;

    /**
     * @brief Get a human-readable name for a power state (VR override)
     *
     * Converts VR-specific PowerState enums to strings for logging purposes.
     * Falls back to base class for upstream states.
     *
     * @param state The power state
     * @return Human-readable state name
     */
    std::string getPowerStateName() override;

  protected:
    /**
     * @brief Validate that all required signals for detected hardware are
     * present in config
     *
     * This method checks that ConfigData objects for all required signals exist
     * in powerSignalMap based on which boards are detected as present (uses
     * BoardPresence).
     *
     * VR implementation checks:
     * - Board 0 signals (always required): Board0RunPowerPG,
     * Board0CpuShutdownOk, etc.
     * - Board 1 signals (conditional): Only if boardPresence.board1Present is
     * true
     *
     * Platform classes override to add checks for platform-specific PDB
     * signals, then call VRPowerControl::validateRequiredSignals() for common
     * VR signals.
     *
     * @throws std::runtime_error if any required signal is missing from config
     */

    /**
     * @brief Validate that all required VR/HPM timer configurations are present
     * in TimerMap
     *
     * VRPowerControl validates VR/HPM common timers and does NOT call base
     * class. Platform classes should call this after validating their own
     * platform-specific timers.
     *
     * Validates:
     * - ForcefulCpuShutdownOkWatchdogMs
     * - GracefulCpuShutdownOkWatchdogMs
     * - CpuResetWatchdogMs
     * - HPMPowerGoodWatchdogMs
     *
     * @throws std::runtime_error if any required timer config is missing
     */
    void validateTimerConfigs() override;

    /**
     * @brief Set default values for VR output signals
     *
     * This virtual method configures the defaultStateHostStateOn and
     * defaultStateHostStateOff properties for common VR/HPM output signals
     * in the powerSignalMap.
     *
     * Platform classes override to set platform-specific PDB default values
     * first, then call this parent method to set common VR/HPM defaults.
     */
    virtual void setDefaultValues();

    /**
     * @brief Check if all required boards have asserted CPU Shutdown OK
     *
     * Reads GPIO values from powerSignalMap and checks if they match polarity.
     * For 1P: Only checks Board 0
     * For 2P: Checks both Board 0 and Board 1
     *
     * @return true if all required boards have asserted SHDN_OK, false
     * otherwise
     */
    bool areAllRequiredBoardsShutdownOk();

    /**
     * @brief Get count of boards that have asserted CPU Shutdown OK
     *
     * Used for detailed logging during graceful shutdown timeout handling.
     *
     * @return Number of boards (0, 1, or 2) that have asserted SHDN_OK
     */
    int getShutdownOkAssertedCount();

    /**
     * @brief Assert Pre System Reset lines for all present boards
     *
     * Sets Board0PreSystemReset to polarity (assert)
     * Sets Board1PreSystemReset to polarity if board1Present (assert)
     */
    void assertBoardPreSystemResets();

    /**
     * @brief Transition to CPU reset assert wait state (success path)
     *
     * Cancels CPU Shutdown OK watchdog timer, logs success, asserts Pre System
     * Reset lines, and transitions to PowerState::waitForCPUResetAssert.
     */
    void transitionToCPUResetAssertState();

    /**
     * @brief Abort graceful shutdown and return to powered-on state
     *
     * Called when CPU(s) fail to assert SHDN_OK during GRACE_OFF.
     * Sets GPIOs back to host state ON and transitions to PowerState::on.
     */
    void abortGracefulShutdown();

    /**
     * @brief Handle CPU Shutdown OK watchdog expiry during FORCE_OFF
     *
     * For forced power off, we don't care about SHDN_OK state - just proceed
     * with asserting Pre System Reset and transitioning to
     * waitForCPUResetAssert.
     */
    void handleCPUShutdownOkWatchdogExpiry_ForceOff();

    /**
     * @brief Handle CPU Shutdown OK watchdog expiry during GRACE_OFF
     *
     * For graceful power off, check how many boards asserted SHDN_OK:
     * - 1P: Abort if Board 0 didn't assert
     * - 2P: Abort if neither asserted, warn and proceed if only one asserted
     */
    void handleCPUShutdownOkWatchdogExpiry_GraceOff();

    /**
     * @brief De-assert Pre System Reset lines during HPM power-on sequence
     *
     * De-asserts Board 0 Pre System Reset and Board 1 Pre System Reset (if
     * present) when HPM Board 0 Run Power Good asserts during power-on.
     */
    void deassertPreSystemResets();

    /**
     * @brief Transition to CPU Reset De-assert wait state
     *
     * Cancels HPM power good watchdog, logs transition, de-asserts Pre System
     * Resets, starts CPU reset watchdog, and transitions to
     * waitForCPUResetDeAssert.
     */
    void transitionToCPUResetDeAssertState();

    /**
     * @brief Handle host-initiated shutdown
     *
     * Called when SHDN_OK is asserted while CPU_BOOT_DONE is asserted.
     * Starts observation period to distinguish between reboot (CPU_BOOT_DONE
     * de-asserts shortly after) and shutdown (CPU_BOOT_DONE stays asserted).
     */
    virtual void handleHostInitiatedShutdown();

    /**
     * @brief Board presence information
     *
     * TODO: Replace with actual implementation from coworker
     * This will be populated by detectBoardPresence() method
     */
    BoardPresence boardPresence;

    /**
     * @brief List of required VR/HPM common timer configurations
     */
    const std::vector<std::string> vrRequiredTimeoutValues = {
        "ForcefulCpuShutdownOkWatchdogMs",
        "GracefulCpuShutdownOkWatchdogMs",
        "CpuResetWatchdogMs",
        "HPMPowerGoodWatchdogMs",
        "PowerCycleDelayMs",
        "ForceWarmRebootDelayMs",
        "CpuBootDoneDeAssertDelayMs",
        "PowerOffSaveMs"};

  protected:
    // VR-SPECIFIC TIMERS
    // These timers are used by VR-specific power sequencing (not in upstream)

    /**
     * @brief Timer for HPM board power good assertion/de-assertion in HPM power
     * sequencing
     */
    boost::asio::steady_timer hpmPowerGoodWatchdogTimer;

    /**
     * @brief Timer for CPU reset assertion on power-on
     */
    boost::asio::steady_timer cpuResetWatchdogTimer;

    /**
     * @brief Timer for CPU shutdown OK assertion
     */
    boost::asio::steady_timer cpuShutdownOkWatchdogTimer;

    /**
     * @brief Timer for CPU Boot Done de-assertion during host-initiated
     * shutdown
     *
     * Used to distinguish between reboot (CPU_BOOT_DONE de-asserts) and
     * shutdown (CPU_BOOT_DONE stays asserted) when SHDN_OK is asserted.
     */
    boost::asio::steady_timer cpuBootDoneDeAssertWatchdogTimer;

    /**
     * @brief Timer for power cycle delay between shutdown and power on
     */
    boost::asio::steady_timer powerCycleDelayTimer;

    /**
     * @brief Timer for warm reboot delay (generic timer, can be reused for
     * graceful warm reboot)
     */
    boost::asio::steady_timer warmRebootDelayTimer;

  protected:
    // GPIO EVENT HANDLERS (Member functions)
    // These handlers check polarity and send appropriate events via
    // sendPowerControlEvent()

    /**
     * @brief Handler for Board 0 Run Power Good GPIO events
     *
     * Checks polarity and sends appropriate Event to sendPowerControlEvent()
     *
     * @param state The GPIO state (true = asserted, false = de-asserted)
     */
    void board0RunPowerPGHandler(bool state);

    /**
     * @brief Handler for detecting board presence
     *
     * Checks presence and updates context using paths from build configuration
     */
    void detectBoardPresence();

    /**
     * @brief Check if IOX presence is present
     *
     * Checks if the IOX presence is present
     */
    bool checkIOXPresence(const std::string& ioxPath);

    /**
     * @brief Handler for Board 1 Run Power Good GPIO events
     *
     * @param state The GPIO state (true = asserted, false = de-asserted)
     */
    void board1RunPowerPGHandler(bool state);

    /**
     * @brief Handler for Board 0 CPU Shutdown OK GPIO events
     *
     * @param state The GPIO state (true = asserted, false = de-asserted)
     */
    void board0CpuShutdownOkHandler(bool state);

    /**
     * @brief Handler for Board 1 CPU Shutdown OK GPIO events
     *
     * @param state The GPIO state (true = asserted, false = de-asserted)
     */
    void board1CpuShutdownOkHandler(bool state);

    /**
     * @brief Handler for CPU Reset Indicator GPIO events
     *
     * @param state The GPIO state (true = asserted, false = de-asserted)
     */
    void cpuResetIndicatorHandler(bool state);

  protected:
    // =============================================================================
    // OVERRIDDEN UPSTREAM STATE HANDLERS (with VR extensions)
    // =============================================================================

    /**
     * @brief Handler for PowerState::on (VR Override - NVL144 default)
     *
     * PREVIOUS IMPLEMENTATION (NVL144 behavior in powerStateOn):
     * Extends base class behavior to add:
     *
     * case Event::powerOffRequest:
     * case Event::gracefulPowerOffRequest:
     * case Event::powerCycleRequest:
     * case Event::powerButtonPressed:
     * case Event::forcePowerOffRequest:
     * case Event::gracefulPowerCycleRequest:
     *
     * NOTE: Platform classes may override to add platform-specific on-state
     * monitoring
     */
    void handlePowerStateOn(Event event) override;

    /**
     * @brief Handler for PowerState::off (VR Override - NVL144 default)
     *
     * PREVIOUS IMPLEMENTATION (NVL144 behavior in powerStateOff):
     * Extends base class behavior to define support for these events:
     *
     * case Event::powerOffRequest:
     * case Event::gracefulPowerOffRequest:
     * case Event::powerCycleRequest:
     * case Event::board0RunPowerPGDeAssert:
     * case Event::board0CpuShutdownOkDeAssert:
     * case Event::board1CpuShutdownOkDeAssert:
     * case Event::resetRequest:
     * case Event::powerButtonPressed:
     * case Event::forcePowerOffRequest:
     * case Event::gracefulPowerCycleRequest:
     *
     * NOTE: Platform classes override to customize PDB power-on behavior
     */
    void handlePowerStateOff(Event event) override;

    // =============================================================================
    // VR-SPECIFIC STATE HANDLERS (PDB Sequencing)
    // =============================================================================

    /**
     * @brief Handler for PowerState::waitForPDBMainPowerOk
     *
     * PREVIOUS IMPLEMENTATION (NVL144 default in
     * powerStateWaitForPDBMainPowerOk):
     *
     * NOTE: NVL144 overrides for NVL144-specific PDB behavior
     * NOTE: C2 overrides to enable 12V rails after PDB powers up
     * NOTE: Parsec (GB300) overrides for GB300-specific PDB behavior
     * NOTE: E5010 defines skip PDB sequencing and go directly to HPM sequencing
     */
    virtual void handleWaitForPDBMainPowerOk(Event event);

    /**
     * @brief Handler for PowerState::waitForPDBMainPowerOff
     *
     * PREVIOUS IMPLEMENTATION (in powerStateWaitForPDBMainPowerOff):
     *
     * Waits for PDB Main Power OK to de-assert after:
     * - De-asserting PDB Main Power Enable during power off sequence
     *
     * case Event::nvl144pdbMainPowerOkDeAssert:
     * case Event::pdbMainPowerOkWatchdogTimerExpired:
     *
     * NOTE: C2 overrides to handle C2 PDB PSU Power OK De-Assert
     * NOTE: Parsec overrides to handle GB300 PDB Main Power OK De-Assert
     * NOTE: E5010 defines skip PDB sequencing and go directly to HPM sequencing
     */
    virtual void handleWaitForPDBMainPowerOff(Event event);

    // =============================================================================
    // VR-SPECIFIC STATE HANDLERS (HPM Board Sequencing)
    // =============================================================================

    /**
     * @brief Handler for PowerState::waitForHPMPowerGoodAssert
     *
     * PREVIOUS IMPLEMENTATION (in powerStateWaitForHPMPowerGoodAssert):
     *
     * Waits for HPM board(s) Run Power Good to assert after:
     * - Asserting Pre System Reset for Board 0/1
     * - Asserting Run Power Enable
     *
     * case Event::board0RunPowerPGAssert:
     * case Event::hpmPowerGoodWatchdogTimerExpired:
     */
    virtual void handleWaitForHPMPowerGoodAssert(Event event);

    /**
     * @brief Handler for PowerState::waitForHPMPowerGoodDeAssert
     *
     * PREVIOUS IMPLEMENTATION (in powerStateWaitForHPMPowerGoodDeAssert):
     * - Asserting Pre System Reset for Board 0/1
     * - De-asserting Run Power Enable for Board 0/1
     *
     * Waits for HPM board(s) Run Power Good to de-assert during power off:
     *
     * case Event::board0RunPowerPGDeAssert:
     * case Event::hpmPowerGoodWatchdogTimerExpired:
     */
    virtual void handleWaitForHPMPowerGoodDeAssert(Event event);

    // =============================================================================
    // VR-SPECIFIC STATE HANDLERS (CPU Reset Monitoring)
    // =============================================================================

    /**
     * @brief Handler for PowerState::waitForCPUResetAssert
     *
     * PREVIOUS IMPLEMENTATION (in powerStateWaitForCPUResetAssert):
     *
     * Waits for CPU Reset Indicator to assert after:
     * - Asserting Run Pre System Reset for Board 0/1 during force off & grace
     * off
     *
     * case Event::cpuResetIndicatorAssert:
     * case Event::cpuResetWatchdogTimerExpired:
     */
    virtual void handleWaitForCPUResetAssert(Event event);

    /**
     * @brief Handler for PowerState::waitForCPUResetDeAssert
     *
     * PREVIOUS IMPLEMENTATION (in powerStateWaitForCPUResetDeAssert):
     *
     * Waits for CPU Reset Indicator to de-assert after:
     * - De-asserting Pre System Reset for Board 0/1 during power on
     *
     * case Event::cpuResetIndicatorDeAssert:
     * case Event::cpuResetWatchdogTimerExpired:
     */
    virtual void handleWaitForCPUResetDeAssert(Event event);

    // =============================================================================
    // VR-SPECIFIC STATE HANDLERS (Graceful Shutdown Monitoring)
    // =============================================================================

    /**
     * @brief Handler for PowerState::waitForCPUShutdownOk
     *
     * PREVIOUS IMPLEMENTATION (in powerStateWaitForCPUShutdownOk):
     *
     * Waits for CPU Shutdown OK to assert after:
     * - Asserting Shutdown Request (graceful) or Shutdown Force (forceful)
     * - Non-failure in forceful shutdown sequence, Failure in graceful shutdown
     * sequence
     *
     * case Event::board0CpuShutdownOkAssert:
     * case Event::board1CpuShutdownOkAssert:
     * case Event::cpuShutdownOkWatchdogTimerExpired:
     */
    virtual void handleWaitForCPUShutdownOk(Event event);

    /**
     * @brief Handler for PowerState::waitForCPUBootDoneDeAssert
     *
     * Waits for CPU_BOOT_DONE to de-assert (reboot) or timer to expire
     * (shutdown). This state is used to distinguish between host-initiated
     * reboot and shutdown when SHDN_OK is asserted.
     *
     * case Event::cpuBootDoneDeAssert: Reboot detected - return to On
     * case Event::cpuBootDoneDeAssertWatchdogTimerExpired: Shutdown confirmed
     *
     * @param event The event to process
     */
    virtual void handleWaitForCPUBootDoneDeAssert(Event event);

    /**
     * @brief Handle events in waitForPowerCycleDelay state
     *
     * Waits for the power cycle delay timer to expire before initiating
     * power on sequence. Rejects all other events during the delay.
     *
     * @param event The event to process
     */
    virtual void handleWaitForPowerCycleDelay(Event event);

    /**
     * @brief Handler for waitForRebootDelay state (warm reboot delay)
     * @param event Event that triggered this handler
     */
    virtual void handleWaitForRebootDelay(Event event);

    // GPIO EVENT HANDLER HELPER FUNCTIONS

    /**
     * @brief Transition to OFF state with Board0RunPowerPG check
     *
     * This helper function intelligently transitions to the appropriate state
     * based on the current state of Board0RunPowerPG:
     * - If Board0RunPowerPG is asserted: Transitions to
     * waitForHPMPowerGoodDeAssert and starts watchdog timer to wait for
     * de-assertion
     * - If Board0RunPowerPG is already de-asserted: Transitions directly to
     *   PowerState::off
     *
     * This prevents the bug where we wait for a de-assert event that will never
     * come (when power never came up in the first place).
     */
    void transitionToOffStateWithRunPowerCheck();

    /**
     * @brief Initiate a force warm reboot sequence
     *
     * This is a common helper function that can be used by any VR platform.
     * It performs the following:
     * - Sets action to FORCE_WARM_REBOOT
     * - Asserts Board0PreSystemReset
     * - Asserts Board1PreSystemReset (if Board1 is present)
     * - Starts CPU Reset watchdog timer
     * - Transitions to waitForCPUResetAssert state
     */
    void initiateForceWarmReboot();

    /**
     * @brief Check for and handle run power faults
     *
     * Detects when Board0RunPowerPG de-asserts or asserts unexpectedly (i.e.,
     * outside of a controlled power action). When a fault is detected:
     * - Logs an error message
     * - Sets GPIOs to OFF state
     * - Forces transition to PowerState::off
     *
     * @param powerControlEvent The event to check (assert or de-assert)
     * @return true if a fault was detected and handled (caller should return)
     * @return false if normal processing should continue
     */
    bool checkAndHandleRunPowerFault(Event powerControlEvent);
};

} // namespace power_control
