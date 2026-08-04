// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#pragma once

#include "power_control_base.hpp"

#include <functional>
#include <string_view>

namespace power_control
{

/**
 * @brief Board presence detection results
 *
 * Populated by detectBoardPresence() from IOX paths declared in the
 * platform's JSON resources section. Used by VR base and platform
 * ctors to gate Board1 signal registration.
 */
struct BoardPresence
{
    bool board0Present = false; // HPM Board 0 IOX present at runtime
    bool board1Present = false; // HPM Board 1 IOX present at runtime
};

/**
 * @brief VR Power Control class - VR-specific extensions (NVL72 default)
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
 * Default implementations use NVL72 behavior. Platform-specific classes
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
    std::string getPowerStateName() const override;

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
     * @brief Abort graceful warm reboot during SHDN_OK wait and return to on
     *
     * Called when the graceful CPU Shutdown OK watchdog expires during
     * GRACEFUL_WARM_REBOOT (no warm reset performed).
     */
    void abortGracefulWarmReboot();

    /**
     * @brief Warm reboot CPU reset watchdog fault: no run-power teardown
     *
     * For FORCE_WARM_REBOOT / GRACEFUL_WARM_REBOOT when CpuResetWatchdogMs
     * expires in waitForCPUResetAssert or waitForCPUResetDeAssert: cancel the
     * watchdog timer, de-assert PRE_SYS_RST (Board 0/1), log loudly to journal
     * and ResourceErrorsDetected, set action NONE and setPowerState::on. No HPM
     * run-power teardown.
     *
     * @param faultDetail Human-readable fault text for event log / journal
     */
    void abortWarmRebootCpuResetWatchdogFault(std::string_view faultDetail);

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
     * @brief Handle CPU Shutdown OK watchdog expiry during FORCE_WARM_REBOOT
     *
     * Proceeds with Pre System Reset assert and waitForCPUResetAssert even if
     * SHDN_OK did not arrive in Safe Stating time.
     */
    void handleCPUShutdownOkWatchdogExpiry_ForceWarmReboot();

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
     * @brief Handle host-initiated reboot
     *
     * Called when CPU_BOOT_DONE de-asserts shortly after SHDN_OK, confirming
     * a reboot rather than a shutdown. Pulses CurrentHostState to Off for 1
     * second (via PowerState::off + HOST_INITIATED_REBOOT) so boot code
     * collection services detect the reboot, while keeping CurrentPowerState
     * On throughout.
     */
    virtual void handleHostInitiatedReboot();

    /**
     * @brief Initiate waiting for SHDN_OK after CPU_BOOT_DONE de-asserts first
     *
     * Called from handlePowerStateOn() when CPU_BOOT_DONE de-asserts before
     * SHDN_OK is seen. Transitions to waitForHostRebootShutdownOk state and
     * starts the HostRebootShutdownOkDelayMs watchdog timer.
     */
    virtual void initiateWaitForHostRebootShutdownOk();

    /**
     * @brief Log a warning when only one of two boards asserts SHDN_OK
     *
     * Called during waitForHostRebootShutdownOk timeout handling when exactly
     * one board has asserted SHDN_OK on a 2P system. Reads GPIO state for both
     * boards and emits a warning journal entry identifying which board asserted
     * and which did not.
     */
    virtual void logPartialShutdownOkWarning();

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
        "GracefulWarmRebootDelayMs",
        "CpuBootDoneDeAssertDelayMs",
        "HostRebootShutdownOkDelayMs",
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

    /**
     * @brief Timer for PDB main power OK assertion/de-assertion in PDB
     * power sequencing
     */
    boost::asio::steady_timer pdbMainPowerOkWatchdogTimer;

    /**
     * @brief Timer for the 1-second HostState Off pulse during a host-initiated
     * reboot. Keeps CurrentHostState Off briefly so boot code collection
     * services can detect the reboot, while CurrentPowerState stays On.
     */
    boost::asio::steady_timer hostInitiatedRebootPulseTimer;

    /**
     * @brief Timer for waiting for SHDN_OK assertion after CPU_BOOT_DONE
     * de-asserts first during a host-initiated reboot.
     *
     * When CPU_BOOT_DONE de-asserts before SHDN_OK asserts, we enter
     * waitForHostRebootShutdownOk and use this timer to wait for the
     * expected SHDN_OK GPIO assertion(s).
     */
    boost::asio::steady_timer hostRebootShutdownOkTimer;

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
     * @brief Handler for PowerState::on (VR Override - NVL72 default)
     *
     * PREVIOUS IMPLEMENTATION (NVL72 behavior in powerStateOn):
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
     * @brief Handler for PowerState::off (VR Override - NVL72 default)
     *
     * PREVIOUS IMPLEMENTATION (NVL72 behavior in powerStateOff):
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
     * PREVIOUS IMPLEMENTATION (NVL72 default in
     * powerStateWaitForPDBMainPowerOk):
     *
     * NOTE: NVL72 overrides for NVL72-specific PDB behavior
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
     * case Event::pdbMainPowerOkDeAssert:
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
     * @brief Handle force power-off while waiting for CPU Shutdown OK during a
     * graceful shutdown, graceful power cycle, or graceful warm reboot
     * (GRACE_OFF, GRACEFUL_POWER_CYCLE, or GRACEFUL_WARM_REBOOT).
     *
     * Invoked when Event::powerOffRequest is received in
     * waitForCPUShutdownOk so the sequence can be upgraded to forceful
     * shutdown without waiting for the graceful watchdog. Platform
     * implementations typically call the same path as handlePowerStateOn for
     * force-off
     */
    virtual void handleForceOffDuringGracefulCpuShutdownOkWait();

    /**
     * @brief Upgrade graceful warm reboot to force warm reboot during wait for
     * CPU Shutdown OK
     *
     * Invoked when Event::resetRequest is received while
     * action == GRACEFUL_WARM_REBOOT in waitForCPUShutdownOk.
     */
    virtual void handleForceWarmRebootDuringGracefulCpuShutdownOkWait();

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
     * @brief Handle events in waitForHostRebootShutdownOk state
     *
     * Entered when CPU_BOOT_DONE de-asserts before SHDN_OK during a
     * host-initiated reboot. Waits for SHDN_OK GPIO assertion(s):
     * - All required boards assert SHDN_OK before timeout: call
     *   handleHostInitiatedReboot() immediately
     * - Timeout with 1 SHDN_OK (2P only): log a warning and call
     *   handleHostInitiatedReboot()
     * - Timeout with 0 SHDN_OK: transition back to PowerState::on
     *
     * @param event The event to process
     */
    virtual void handleWaitForHostRebootShutdownOk(Event event);

    /**
     * @brief Handler for PowerState::waitForCpuRecovery
     *
     * FSM is parked here while Board0PreSystemReset is held low for USB-RCM
     * strap programming. Ignores GPIO chatter from the CPU entering reset
     * (SHDN_OK, CpuResetIndicator). Exits and restores the reset GPIO if
     * power actually drops (PDBMainPowerOkDeAssert, Board0RunPowerPGDeAssert)
     * or if an explicit power-off / aux-cycle is requested over D-Bus.
     */
    virtual void handleWaitForCpuRecovery(Event event);

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

    // =============================================================================
    // PLATFORM-SPECIFIC VIRTUAL METHODS (pure virtual or overridable defaults)
    // =============================================================================

    /**
     * @brief Handle power on request from PowerState::off
     *
     * Platform-specific: asserts platform PDB enable signals, starts the
     * appropriate watchdog timer, and transitions to the first PDB power-on
     * wait state.
     *
     * Pure virtual — every platform must provide an implementation.
     */
    virtual void handlePowerOnRequest() = 0;

    /**
     * @brief Check if system power is already fully off
     *
     * Platform-specific: checks whichever power-indicator signals the
     * platform uses (e.g. Board0RunPowerPG + PDBMainPowerOk for NVL72).
     *
     * Pure virtual — every platform must provide an implementation.
     *
     * @return true if all platform power indicators are de-asserted
     */
    virtual bool isSystemPowerOff() = 0;

    /**
     * @brief Assert HPM board power sequence during power-on
     *
     * VR default: asserts Board0/1 Pre System Reset, calls
     * assertPlatformPeripherals(), then asserts Board0/1 Run Power Enable.
     *
     * Platform classes override only when inter-step ordering needs to
     * change (e.g. NVL72 inlines a 10 ms GPU_OVERT WAR sleep between
     * Board0 and Board1 Run Power Enable).
     */
    virtual void assertHPMBoardPowerSequence();

    /**
     * @brief De-assert HPM power and peripheral signals during shutdown
     *
     * VR default: de-asserts Board0/1 Run Power Enable, then calls
     * deassertPlatformPeripherals().
     *
     * Platform classes typically override deassertPlatformPeripherals()
     * rather than this method.
     */
    virtual void deassertHPMPowerAndPeripherals();

    /**
     * @brief Assert platform-specific peripheral signals during power-on
     *
     * Called from assertHPMBoardPowerSequence() between Pre System Reset
     * assertion and Run Power Enable assertion. Default is a no-op.
     *
     * Platform classes override to assert peripherals like USB Power
     * Enable, E1S Power Enable, SSD Power Disable de-assert, BMC SSD Reset
     * de-assert, and any inter-step delays specific to those peripherals.
     */
    virtual void assertPlatformPeripherals() {}

    /**
     * @brief De-assert platform-specific peripheral signals during shutdown
     *
     * Called from deassertHPMPowerAndPeripherals() after Run Power Enable
     * de-assertion. Default is a no-op.
     *
     * Platform classes override to de-assert peripherals on shutdown.
     * Note: this need not symmetrically mirror assertPlatformPeripherals();
     * some signals may be intentionally left in their power-on state.
     */
    virtual void deassertPlatformPeripherals() {}

    /**
     * @brief Initiate the PDB power-off sequence after HPM boards have
     * powered down
     *
     * Called when Board0RunPowerPG de-asserts. Platform-specific: de-asserts
     * the platform PDB enable signal(s), starts the PDB watchdog, and
     * transitions to waitForPDBMainPowerOff (or bypasses intermediate wait
     * states if the PDB ok signals are already de-asserted).
     *
     * Pure virtual — every platform must provide an implementation.
     */
    virtual void initiatePDBPowerOff() = 0;

    /**
     * @brief Complete shutdown and transition to the appropriate next state
     *
     * Called from handleWaitForPDBMainPowerOff with success=true on
     * pdbMainPowerOkDeAssert and success=false on watchdog expiry.
     *
     * VR default (NVL72 behaviour): dispatches on action to either
     * transitionToOffState() or transitionToPowerCycleDelay().
     *
     * Platform classes override when their PDB teardown has additional steps
     * after PDB Main Power OK de-asserts (e.g. C2 de-asserts PSU enable and
     * waits for PSU power off before finishing).
     *
     * @param success true if PDB ok de-asserted normally, false on watchdog
     */
    virtual void completeShutdownAndTransitionToOff(bool success);

    // =============================================================================
    // HELPER FUNCTIONS (moved from platform, shared by all VR platforms)
    // =============================================================================

    /**
     * @brief Handle shutdown request (forceful or graceful) from
     * PowerState::on
     *
     * Validates graceful operations (CPU Boot Done check), sets the action,
     * calls the virtual isSystemPowerOff() to decide whether to initiate CPU
     * shutdown or transition directly to off/power-cycle-delay.
     *
     * @param event powerOffRequest (forceful) or gracefulPowerOffRequest /
     *              gracefulPowerCycleRequest / gracefulResetRequest (graceful)
     */
    void handleShutdownRequest(Event event);

    /**
     * @brief Initiate CPU shutdown sequence
     *
     * Asserts the appropriate Board 0 shutdown signal (Force or Request),
     * de-asserts the Board 1 counterpart if present, starts the CPU Shutdown
     * OK watchdog, and transitions to waitForCPUShutdownOk.
     *
     * @param isForceful true → use SHDN_FORCE + forceful watchdog;
     *                   false → use SHDN_REQ + graceful watchdog
     */
    void initiateCPUShutdown(bool isForceful);

    /**
     * @brief Aux power cycle shutdown primitives (see PowerControl)
     *
     * Kick the VR graceful/forceful shutdown FSM, which is atomic and settles
     * at PowerState::on or ::off. The base aux orchestration drives these and
     * reacts to the settled state via the setPowerState() completion hook.
     */
    void initiateGracefulShutdown() override;
    void initiateForcefulShutdown() override;

    /**
     * @brief Handle power cycle request when in off state
     *
     * Reads Board0RunPowerPG to verify power is off: if de-asserted, calls
     * handlePowerOnRequest(); if still asserted, transitions to on and calls
     * handleShutdownRequest() to power down first.
     *
     * @param event powerCycleRequest (forceful) or gracefulPowerCycleRequest
     */
    void handlePowerCycleWhenOff(Event event);

    /**
     * @brief Dispatch to the appropriate final state after all PDB power
     * sequencing is complete
     *
     * Platform-agnostic action dispatch: looks at the current action and
     * calls transitionToOffState() or transitionToPowerCycleDelay().
     * Called by completeShutdownAndTransitionToOff (VR default) and directly
     * by platform state handlers when the final PDB signal de-asserts
     * (e.g. C2 calls this from handleWaitForPDBPSUPowerOff).
     */
    void applyShutdownAction();

    /**
     * @brief Transition to off state after successful shutdown
     *
     * Clears action, sets GPIOs to off state, transitions to
     * PowerState::off.
     */
    void transitionToOffState();

    /**
     * @brief Transition to power cycle delay state
     *
     * Preserves action, sets GPIOs to off state, starts the power cycle
     * delay timer, and transitions to waitForPowerCycleDelay.
     */
    void transitionToPowerCycleDelay();

    /**
     * @brief Transition to HPM Power Good assert wait state
     *
     * Cancels the PDB Main Power OK watchdog, calls the virtual
     * assertHPMBoardPowerSequence(), starts the HPM Power Good watchdog,
     * and transitions to waitForHPMPowerGoodAssert.
     */
    void transitionToHPMPowerGoodAssertState();

    /**
     * @brief Transition to HPM Power Good de-assert wait state
     *
     * Cancels the CPU reset watchdog, calls the virtual
     * deassertHPMPowerAndPeripherals(), starts the HPM Power Good watchdog,
     * and transitions to waitForHPMPowerGoodDeAssert.
     */
    void transitionToHPMPowerGoodDeAssertState();

    /**
     * @brief Handle CPU Reset Indicator assertion in waitForCPUResetAssert
     *
     * VR default: if warm reboot action, starts warm reboot delay timer and
     * transitions to waitForRebootDelay; otherwise calls
     * transitionToHPMPowerGoodDeAssertState() for the shutdown path.
     *
     * Platform classes override if they need different behaviour when CPUs
     * enter reset (e.g. different reboot delay logic or additional GPIO ops).
     */
    virtual void handleCPUResetIndicatorAsserted();

    /**
     * @brief Handle CPU Reset Watchdog expiry in waitForCPUResetAssert
     *
     * VR default: if warm reboot action, calls
     * abortWarmRebootCpuResetWatchdogFault() and restores host-ON GPIOs;
     * otherwise logs shutdown-sequence fault and calls
     * transitionToOffStateWithRunPowerCheck().
     *
     * Platform classes override if they need different fault recovery.
     */
    virtual void handleCPUResetWatchdogExpired();

    /**
     * @brief Handle HPM Power Good watchdog expiry during shutdown sequence
     *
     * VR default: logs fault, resets action to NONE, calls
     * setGPIOsForHostStateOn() and setPowerState(on).
     *
     * Platform classes override if they need additional cleanup when the
     * HPM power good watchdog fires during a shutdown.
     */
    virtual void handleHPMPowerGoodWatchdogExpiredDuringShutdown();

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
     * - Cancels the CPU Shutdown OK watchdog (if running)
     * - Sets action to FORCE_WARM_REBOOT
     * - De-asserts Board0CpuShutdownForce (SHDN_FORCE)
     * - De-asserts Board1CpuShutdownForce if Board1 is present
     * - Asserts Board0PreSystemReset
     * - Asserts Board1PreSystemReset (if Board1 is present)
     * - Starts CPU Reset watchdog timer
     * - Transitions to waitForCPUResetAssert state
     */
    void initiateForceWarmReboot();

    /**
     * @brief Initiate host-initiated shutdown sequence
     *
     * Called when the CPU Boot Done de-assert watchdog expires, confirming
     * CPU_BOOT_DONE stayed asserted — a valid host-initiated shutdown.
     * Asserts Pre System Reset lines, starts the CPU Reset watchdog, and
     * transitions to waitForCPUResetAssert.
     */
    void initiateHostInitiatedShutdown();

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
