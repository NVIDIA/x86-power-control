// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#pragma once

#include "../../vr_power_control.hpp"

namespace power_control
{

/**
 * @brief NVL72 Power Control class
 *
 * This class represents the NVL72 platform-specific power control.
 * VRPowerControl provides default implementations for all shared VR
 * behaviour. NVL72 overrides only where its hardware deviates:
 *
 * - PDB power-on: asserts PDBMainPowerEnable, waits for PDBMainPowerOk
 * - HPM board sequencing: additionally toggles E1SPowerEnable,
 *   BMCSSDReset, SSDPowerDisable with NVL72-specific ordering/delays
 * - isSystemPowerOff: checks Board0RunPowerPG AND PDBMainPowerOk
 * - PDB power-off: inspects PDBMainPowerOk before de-asserting
 *   PDBMainPowerEnable (NVL72-specific)
 * - pdbMainPowerOkHandler: applies HSC alert-mask WAR on each assert
 * - Supports Board 0 and optionally Board 1
 */
class NVL72PowerControl : public VRPowerControl
{
  public:
    NVL72PowerControl(boost::asio::io_context& ioContext,
                      std::shared_ptr<sdbusplus::asio::connection> conn,
                      const std::string& configFilePath,
                      const std::string& node, PersistentState& appState);

    ~NVL72PowerControl() override = default;

  protected:
    // =========================================================================
    // Pure-virtual overrides required by VRPowerControl
    // =========================================================================

    /**
     * @brief Check if system power is already fully off (NVL72 override)
     *
     * Returns true only when BOTH Board0RunPowerPG AND PDBMainPowerOk are
     * de-asserted.
     */
    bool isSystemPowerOff() override;

    /**
     * @brief Handle power on request from PowerState::off (NVL72 override)
     *
     * Asserts PDBMainPowerEnable, starts PdbMainPowerOkWatchdogMs timer,
     * and transitions to waitForPDBMainPowerOk.
     */
    void handlePowerOnRequest() override;

    /**
     * @brief Initiate PDB power-off after HPM boards have powered down
     * (NVL72 override)
     *
     * Checks PDBMainPowerOk state; if still asserted de-asserts
     * PDBMainPowerEnable and transitions to waitForPDBMainPowerOff,
     * otherwise bypasses the wait state.
     */
    void initiatePDBPowerOff() override;

    // =========================================================================
    // Virtual overrides — extend or replace VR defaults
    // =========================================================================

    /**
     * @brief Handler for PDB Main Power OK GPIO events (NVL72 override)
     *
     * Applies the NVL72 HSC alert-mask WAR on each PDBMainPowerOk assert,
     * then delegates to VRPowerControl::pdbMainPowerOkHandler().
     */
    void pdbMainPowerOkHandler(bool state) override;

    /**
     * @brief Assert HPM board power sequence during power-on (NVL72 override)
     *
     * NVL72-specific sequence: Board0/1 Pre System Reset, SSD Power Disable
     * de-assert, BMC SSD Reset de-assert, 1 ms delay, USB + E1S Power Enable,
     * Board0 Run Power Enable, 10 ms GPU WAR delay, Board1 Run Power Enable.
     */
    void assertHPMBoardPowerSequence() override;

    /**
     * @brief De-assert HPM power and peripherals during shutdown
     * (NVL72 override)
     *
     * Calls VRPowerControl::deassertHPMPowerAndPeripherals() for the common
     * signals (Board0/1 RunPowerEnable, USB), then additionally de-asserts
     * E1S Power Enable.
     */
    void deassertHPMPowerAndPeripherals() override;

    /**
     * @brief Set default values for NVL72 output signals (NVL72 override)
     *
     * Sets NVL72-specific PDB signal defaults, then calls
     * VRPowerControl::setDefaultValues() for common VR defaults.
     */
    void setDefaultValues() override;

    /**
     * @brief Validate that all required timer configurations for NVL72 are
     * present in TimerMap (NVL72 override)
     *
     * NVL72 has no platform-specific timers beyond what VRPowerControl
     * validates (PdbMainPowerOkWatchdogMs is now in vrRequiredTimeoutValues).
     * Delegates directly to VRPowerControl::validateTimerConfigs().
     */
    void validateTimerConfigs() override;

  private:
    // =========================================================================
    // NVL72-specific helpers
    // =========================================================================

    /**
     * @brief Add Board 1 GPIO state D-Bus properties
     *
     * Registers the Board1CpuShutdownOk property on the GPIO state interface.
     * Called only when boardPresence.board1Present is true.
     */
    void addBoard1GpioStateProperties();

    /**
     * @brief De-assert PDB Main Power Enable during shutdown
     *
     * De-asserts PDBMainPowerEnable when the HPM boards have powered down.
     */
    void deassertPreSystemResetsAndPDBMainPower();

    /**
     * @brief Transition to PDB Main Power Off wait state (unconditional)
     *
     * De-asserts PDBMainPowerEnable, starts PdbMainPowerOkWatchdogMs, and
     * transitions to waitForPDBMainPowerOff without checking current ok state.
     */
    void transitionToPDBMainPowerOffState();

    /**
     * @brief Mask HSC alerts and clear faults on shared PDB interrupt line
     *
     * HSCs share an interrupt with a PDB IOX; uncleared faults can hold the
     * line low and block IOX interrupts. Called on each PDBMainPowerOk assert
     * because masking may reset with power events.
     */
    void maskHscAlertsAndClearFaults();

    /**
     * @brief Timer used to delay PDB HSC MFR_ID retry attempts
     *
     * Keeps retry delays non-blocking so the power-control event loop can keep
     * processing GPIO, D-Bus, and timer events.
     */
    boost::asio::steady_timer hscMfrIdRetryTimer;

    /**
     * @brief Perform one PDB HSC MFR_ID vendor-detection attempt
     *
     * Runs the existing mask/clear sequence when vendor detection succeeds, or
     * schedules another retry when attempts remain.
     *
     * @param attempt Current zero-based retry attempt number
     */
    void maskHscAlertsAndClearFaultsAttempt(int attempt);

    /**
     * @brief Schedule the next PDB HSC MFR_ID vendor-detection attempt
     *
     * Uses hscMfrIdRetryTimer to retry later without blocking this thread.
     *
     * @param nextAttempt Next zero-based retry attempt number
     */
    void scheduleHscMfrIdRetry(int nextAttempt);

    /**
     * @brief Power indicator signals used to determine initial hardware power
     * state
     *
     * The host is considered ON only if BOTH Board0RunPowerPG AND
     * PDBMainPowerOk are asserted.
     */
    const std::vector<std::string> powerIndicators = {"Board0RunPowerPG",
                                                      "PDBMainPowerOk"};
};

} // namespace power_control
