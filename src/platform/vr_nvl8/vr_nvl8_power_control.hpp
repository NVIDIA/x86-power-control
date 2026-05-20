// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#pragma once

#include "../../vr_power_control.hpp"

namespace power_control
{

/**
 * @brief VR NVL8 (Fractal Vera) Power Control class
 *
 * VR NVL8 is a single-CPU VR platform with NO PDB — rail control was moved
 * to the HPM CPLD + HPM-MCU, so the BMC's job collapses to HPM
 * sequencing only: assert RUN_POWER_EN, wait for RUN_POWER_PG, drive
 * PRE_SYS_RST and the SHDN_REQ / SHDN_OK / SHDN_FORCE handshake, and
 * observe CPU_RESET_L. There is no PDBMainPowerOk, no PDBMainPowerEnable,
 * no PSU enable, no 12V rails, no USB Power Enable, no E1S, no SSD, no
 * Board 1.
 *
 * Inherits directly from VRPowerControl. The base HPM sequencing is
 * exactly what VR NVL8 needs, so most virtual methods are NOT overridden.
 * Overrides:
 *   - handlePowerOnRequest: skip PDB stage, jump straight to HPM Power
 *     Good assert wait.
 *   - isSystemPowerOff: only checks Board0RunPowerPG.
 *   - initiatePDBPowerOff: no PDB to tear down, just dispatch the
 *     shutdown action.
 *   - getPowerStateHandler / getHostState / getChassisState /
 *     getPowerStateName: defensive no-op / error-log mappings for the
 *     waitForPDBMainPowerOk / waitForPDBMainPowerOff power states which
 *     VR NVL8 should never enter.
 */
class VRNVL8PowerControl : public VRPowerControl
{
  public:
    VRNVL8PowerControl(boost::asio::io_context& ioContext,
                       std::shared_ptr<sdbusplus::asio::connection> conn,
                       const std::string& configFilePath,
                       const std::string& node, PersistentState& appState);

    ~VRNVL8PowerControl() override = default;

    /**
     * @brief Get the handler function for a given power state (VR NVL8
     * override)
     *
     * Defensive override: for waitForPDBMainPowerOk and
     * waitForPDBMainPowerOff (states VR NVL8 should never enter), logs an
     * error and returns an empty std::function so events in that state
     * are silently dropped instead of dispatching into VR base's PDB
     * handlers that reference an unstarted PDB watchdog timer.
     */
    std::function<void(Event)> getPowerStateHandler() override;

    /**
     * @brief Get the host state D-Bus property value (VR NVL8 override)
     *
     * Defensive override for the PDB wait states — log error and return
     * HostState::Off as a safe placeholder.
     */
    std::string_view getHostState() const override;

    /**
     * @brief Get the chassis state D-Bus property value (VR NVL8 override)
     *
     * Defensive override for the PDB wait states — log error and return
     * ChassisState::Off as a safe placeholder.
     */
    std::string_view getChassisState() const override;

    /**
     * @brief Get a human-readable name for a power state (VR NVL8 override)
     */
    std::string getPowerStateName() const override;

  protected:
    // =========================================================================
    // Pure-virtual overrides required by VRPowerControl
    // =========================================================================

    /**
     * @brief Check if system power is fully off (VR NVL8 override)
     *
     * Returns true iff Board0RunPowerPG is de-asserted. VR NVL8 has no PDB,
     * so no other power indicators apply.
     */
    bool isSystemPowerOff() override;

    /**
     * @brief Handle power on request from PowerState::off (VR NVL8 override)
     *
     * HPM-only flow: if Board0RunPowerPG is already asserted, treat as
     * already-on. Otherwise transition straight to HPM Power Good assert
     * wait — no PDB stage.
     */
    void handlePowerOnRequest() override;

    /**
     * @brief Complete shutdown after HPM boards have powered down
     * (VR NVL8 override)
     *
     * VR base calls this from handleWaitForHPMPowerGoodDeAssert when
     * Board0RunPowerPG de-asserts. VR NVL8 has no PDB to wait for, so we
     * just cancel the HPM watchdog and dispatch the shutdown action.
     */
    void initiatePDBPowerOff() override;

  private:
    /**
     * @brief Power indicator signals used to determine initial hardware
     * power state
     *
     * The host is considered ON iff Board0RunPowerPG is asserted.
     */
    const std::vector<std::string> powerIndicators = {"Board0RunPowerPG"};
};

} // namespace power_control
