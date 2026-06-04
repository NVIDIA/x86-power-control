// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#pragma once

#include "../../vr_power_control.hpp"

namespace power_control
{

/**
 * @brief C2 Power Control class
 *
 * This class represents the C2 platform-specific power control.
 * C2 is a 1P (single-board) VR platform with a unique PDB that differs
 * from NVL72:
 *
 * Power-on PDB sequence:
 *   1. Assert PDBPSUPowerOn → waitForPDBPSUPowerOk
 *   2. pdbPSUPowerOkAssert → assert 12V rails (GPU1, GPU2, HPM-AIC)
 *      → waitForPDBMainPowerOk
 *   3. pdbMainPowerOkAssert → HPM board sequencing → waitForHPMPowerGoodAssert
 *
 * Power-off PDB sequence (after HPM boards power down):
 *   1. De-assert 12V rails (HPM-AIC, GPU1, GPU2) → waitForPDBMainPowerOff
 *   2. pdbMainPowerOkDeAssert → de-assert PDBPSUPowerOn → waitForPDBPSUPowerOff
 *   3. pdbPSUPowerOkDeAssert → applyShutdownAction() → off or power cycle
 *
 * Differences from NVL72:
 *   - 1P only (no Board 1 signals)
 *   - No E1SPowerEnable, BMCSSDReset, SSDPowerDisable
 *   - PSU enable/ok signals before main PDB power
 *   - Three 12V rail enables (GPU1, GPU2, HPM-AIC)
 *   - isSystemPowerOff checks Board0RunPowerPG + PDBMainPowerOk + PDBPSUPowerOk
 */
class C2PowerControl : public VRPowerControl
{
  public:
    C2PowerControl(boost::asio::io_context& ioContext,
                   std::shared_ptr<sdbusplus::asio::connection> conn,
                   const std::string& configFilePath, const std::string& node,
                   PersistentState& appState);

    ~C2PowerControl() override = default;

    /**
     * @brief Get the handler function for a given power state (C2 extension)
     *
     * Adds routing for C2-specific states (waitForPDBPSUPowerOk,
     * waitForPDBPSUPowerOff) and delegates all other states to
     * VRPowerControl::getPowerStateHandler().
     */
    std::function<void(Event)> getPowerStateHandler() override;

    /**
     * @brief Get the host state D-Bus property value (C2 override)
     *
     * Adds host-state mappings for C2-specific power states and falls
     * back to VRPowerControl for all other states.
     */
    std::string_view getHostState() const override;

    /**
     * @brief Get the chassis state D-Bus property value (C2 override)
     *
     * Adds chassis-state mappings for C2-specific power states and falls
     * back to VRPowerControl for all other states.
     */
    std::string_view getChassisState() const override;

    /**
     * @brief Get a human-readable name for a power state (C2 override)
     *
     * Adds name mappings for C2-specific power states and falls back to
     * VRPowerControl for all other states.
     */
    std::string getPowerStateName() const override;

  protected:
    // =========================================================================
    // Pure-virtual overrides required by VRPowerControl
    // =========================================================================

    /**
     * @brief Check if system power is already fully off (C2 override)
     *
     * Returns true only when Board0RunPowerPG, PDBMainPowerOk, AND
     * PDBPSUPowerOk are all de-asserted.
     */
    bool isSystemPowerOff() override;

    /**
     * @brief Handle power on request from PowerState::off (C2 override)
     *
     * C2 power-on sequence: asserts PDBPSUPowerOn, starts PSU power ok
     * watchdog, and transitions to waitForPDBPSUPowerOk.
     *
     * TODO: Implement full C2 PDB power-on sequence.
     */
    void handlePowerOnRequest() override;

    /**
     * @brief Initiate PDB power-off after HPM boards have powered down
     * (C2 override)
     *
     * Always de-asserts the three 12V rail enables first (HPM-AIC, GPU1,
     * GPU2), then cascades down the power-ok signals with early returns:
     *   - PDBMainPowerOk still asserted → waitForPDBMainPowerOff + watchdog
     *   - PDBMainPowerOk de-asserted, PDBPSUPowerOk still asserted →
     *     de-assert PDBPSUPowerOn + waitForPDBPSUPowerOff + PSU watchdog
     *   - Both de-asserted → de-assert PDBPSUPowerOn +
     *     completeShutdownAndTransitionToOff(true)
     */
    void initiatePDBPowerOff() override;

    // =========================================================================
    // Virtual overrides — extend or replace VR defaults
    // =========================================================================

    /**
     * @brief Reject power-on / power-cycle while standby power is lost
     *
     * Returns false (with an explanatory reason) when stbyPowerLost is set,
     * since the IOXs that host the power-sequencing GPIOs are unpowered and
     * no power-on attempt could succeed.
     */
    bool canAcceptPowerOnRequest(std::string& reason) override;

    /**
     * @brief Suppress GPIO events from IOXs whose standby rail is down
     *
     * Returns true for every signal except StbyPwrOk itself while
     * stbyPowerLost is set. The StbyPwrOk signal must always be delivered
     * so we can observe recovery (or further state changes) on the witness.
     */
    bool shouldIgnoreEvent(const std::string& signalName) override;

    /**
     * @brief Complete shutdown after PDB Main Power OK de-asserts (C2 override)
     *
     * Cancels pdbMainPowerOkWatchdogTimer. On success: de-asserts
     * PDBPSUPowerOn, starts pdbPSUPowerOkWatchdogTimer, transitions to
     * waitForPDBPSUPowerOff. On failure (watchdog expired): logs error, sets
     * GPIOs to off state, transitions to PowerState::off.
     */
    void completeShutdownAndTransitionToOff(bool success) override;

    /**
     * @brief Set default values for C2 output signals (C2 override)
     *
     * Sets C2-specific PDB signal defaults (PSU enable, 12V rails) and the
     * USB Power Enable defaults, then calls VRPowerControl::setDefaultValues()
     * for common VR defaults.
     */
    void setDefaultValues() override;

    /**
     * @brief Assert C2 platform peripherals (C2 override)
     *
     * Asserts USB Power Enable. Called from
     * VRPowerControl::assertHPMBoardPowerSequence() between Pre System Reset
     * assertion and Run Power Enable assertion.
     */
    void assertPlatformPeripherals() override;

    /**
     * @brief De-assert C2 platform peripherals (C2 override)
     *
     * De-asserts USB Power Enable. Called from
     * VRPowerControl::deassertHPMPowerAndPeripherals() after Run Power Enable
     * de-assertion.
     */
    void deassertPlatformPeripherals() override;

    /**
     * @brief Validate required timer configurations for C2 (C2 override)
     *
     * Validates C2-specific timers (PdbPSUPowerOkWatchdogMs), then calls
     * VRPowerControl::validateTimerConfigs() for common VR timers.
     */
    void validateTimerConfigs() override;

    // =========================================================================
    // C2-specific state handlers
    // =========================================================================

    /**
     * @brief Handler for PowerState::waitForPDBPSUPowerOk
     *
     * Waits for PDBPSUPowerOk to assert after PDBPSUPowerOn is asserted.
     *
     * case Event::pdbPSUPowerOkAssert:  → assert 12V rails, start PDB
     *     main power OK watchdog, transition to waitForPDBMainPowerOk
     * case Event::pdbPSUPowerOkWatchdogTimerExpired: → fault, go to off
     *
     * TODO: Implement.
     */
    void handleWaitForPDBPSUPowerOk(Event event);

    /**
     * @brief Handler for PowerState::waitForPDBPSUPowerOff
     *
     * Waits for PDBPSUPowerOk to de-assert after PDBPSUPowerOn is
     * de-asserted.
     *
     * case Event::pdbPSUPowerOkDeAssert: → cancel PSU watchdog +
     *     applyShutdownAction() → off or power cycle
     * case Event::pdbPSUPowerOkWatchdogTimerExpired: → fault, go to off
     */
    void handleWaitForPDBPSUPowerOff(Event event);

  private:
    // =========================================================================
    // C2-specific GPIO handler
    // =========================================================================

    /**
     * @brief Handler for PDB Main Power OK GPIO events (C2)
     *
     * Dispatches the assert/de-assert event. An unexpected de-assertion
     * (outside waitForPDBMainPowerOff) is treated as a power fault and handled
     * by checkAndHandlePdbMainPowerOkFault().
     *
     * @param state The GPIO state (true = asserted, false = de-asserted)
     */
    void pdbMainPowerOkHandler(bool state);

    /**
     * @brief Detect and handle an unexpected PDB Main Power OK de-assertion
     *
     * Mirrors checkAndHandleRunPowerFault for PDBMainPowerOk: if PDBMainPowerOk
     * de-asserts while NOT in waitForPDBMainPowerOff (the only state where a
     * de-assertion is expected), logs a POWER FAULT, emits a
     * ResourceErrorsDetected event log, clears the action, and transitions to
     * off via transitionToOffStateWithRunPowerCheck().
     *
     * @param powerControlEvent The assert/de-assert event derived from the GPIO
     * @return true if a fault was detected and handled (caller should return)
     * @return false if normal event processing should continue
     */
    bool checkAndHandlePdbMainPowerOkFault(Event powerControlEvent);

    /**
     * @brief Handler for PDBPSUPowerOk GPIO events
     *
     * Sends Event::pdbPSUPowerOkAssert or Event::pdbPSUPowerOkDeAssert
     * to the state machine.
     *
     * @param state The GPIO state (true = asserted, false = de-asserted)
     */
    void pdbPSUPowerOkHandler(bool state);

    /**
     * @brief Handler for StbyPwrOk GPIO events
     *
     * StbyPwrOk witnesses the 12V HPM standby power domain that feeds the
     * IOXs hosting the power-sequencing GPIOs. The signal itself is on a
     * BMC-domain IOX that stays powered as long as the BMC has power.
     *
     * - On de-assert: calls markStandbyLost(). The event may arrive after
     *   shouldIgnoreEvent has already detected the loss via the witness
     *   read, in which case markStandbyLost is idempotent and this is a
     *   no-op.
     * - On re-assert: logs recovery. Does NOT clear stbyPowerLost.
     */
    void stbyPwrOkHandler(bool state);

    /**
     * @brief Mark the standby power domain as lost (idempotent)
     *
     * See NVL72PowerControl::markStandbyLost — same contract.
     */
    void markStandbyLost();

    // =========================================================================
    // C2-specific state transition helpers
    // =========================================================================

    /**
     * @brief Assert all three 12V rail enables and transition to
     * waitForPDBMainPowerOk
     *
     * Called when PDBPSUPowerOk asserts. Asserts the three 12V enables in
     * order: PDB12V_GPU1Enable, then PDB12V_GPU2_Enable, then
     * PDB12V_HPM-AICEnable. Starts the PdbMainPowerOkWatchdogMs timer, then
     * transitions to PowerState::waitForPDBMainPowerOk.
     */
    void assert12VRailsAndWaitForPDBMainPowerOk();

    /**
     * @brief Handle PDB PSU Power OK watchdog expiry
     *
     * Called when the PSU power OK watchdog fires during
     * PowerState::waitForPDBPSUPowerOk. Logs the fault, emits an event
     * log entry, sets GPIOs to host-off state, and transitions to
     * PowerState::off.
     */
    void handlePDBPSUPowerOkWatchdogExpired();

    // =========================================================================
    // C2-specific timers and config
    // =========================================================================

    /**
     * @brief Timer for PDB PSU power OK assertion/de-assertion in C2 PDB
     * power sequencing
     */
    boost::asio::steady_timer pdbPSUPowerOkWatchdogTimer;

    /**
     * @brief C2-specific required timer configuration keys
     */
    const std::vector<std::string> c2RequiredTimeoutValues = {
        "PdbPSUPowerOkWatchdogMs",
        "PdbMainPowerOkWatchdogMs",
    };

    /**
     * @brief Power indicator signals used to determine initial hardware
     * power state
     *
     * The host is considered ON only if ALL THREE signals are asserted:
     * Board0RunPowerPG, PDBMainPowerOk, and PDBPSUPowerOk.
     */
    const std::vector<std::string> powerIndicators = {
        "Board0RunPowerPG", "PDBMainPowerOk", "PDBPSUPowerOk"};

    /**
     * @brief Tracks whether the 12V HPM standby power domain has been lost
     *
     * Set true when StbyPwrOk de-asserts, indicating the IOXs that host
     * the power-sequencing GPIOs have lost their feed and any read/write
     * to them will fail. While set:
     *   - canAcceptPowerOnRequest() rejects power-on / power-cycle requests
     *   - shouldIgnoreEvent() suppresses dispatch of all GPIO events except
     *     StbyPwrOk itself
     *
     * In this commit the flag is set on de-assert and left set on re-assert
     * (recovery requires a BMC reboot or AC cycle). A later commit will
     * add automatic recovery.
     */
    bool stbyPowerLost = false;
};

} // namespace power_control
