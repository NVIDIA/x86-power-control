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
 *   1. De-assert 12V rails → waitForPDBMainPowerOff
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
     * Sets C2-specific PDB signal defaults (PSU enable, 12V rails), then
     * calls VRPowerControl::setDefaultValues() for common VR defaults.
     */
    void setDefaultValues() override;

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
     * @brief Handler for PDBPSUPowerOk GPIO events
     *
     * Sends Event::pdbPSUPowerOkAssert or Event::pdbPSUPowerOkDeAssert
     * to the state machine.
     *
     * @param state The GPIO state (true = asserted, false = de-asserted)
     */
    void pdbPSUPowerOkHandler(bool state);

    // =========================================================================
    // C2-specific state transition helpers
    // =========================================================================

    /**
     * @brief Assert all three 12V rail enables and transition to
     * waitForPDBMainPowerOk
     *
     * Called when PDBPSUPowerOk asserts. Asserts PDB12V_HPM-AICEnable,
     * PDB12V_GPU1Enable, and PDB12V_GPU2_Enable, starts the
     * PdbMainPowerOkWatchdogMs timer, then transitions to
     * PowerState::waitForPDBMainPowerOk.
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
};

} // namespace power_control
