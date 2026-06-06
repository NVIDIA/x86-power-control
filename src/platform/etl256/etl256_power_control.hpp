// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#pragma once

#include "../../vr_power_control.hpp"

namespace power_control
{

/**
 * @brief ETL256 Vera Power Control class
 *
 * ETL256 is a single-board VR platform with no PDB and no BMC-controlled
 * standby-power sequence. The BMC controls only the HPM run-power path:
 * assert PRE_SYS_RST_L, enable USB power, assert RUN_PWR_EN, wait for
 * MODULE_PWR_GOOD, then release PRE_SYS_RST_L and observe CPU reset.
 *
 * Inherits directly from VRPowerControl. The base HPM sequencing is close to
 * what ETL256 needs, so this class overrides only the ETL256-specific pieces:
 *   - handlePowerOnRequest: skip PDB stage and jump straight to HPM run power.
 *   - isSystemPowerOff: only checks Board0RunPowerPG / MODULE_PWR_GOOD.
 *   - initiatePDBPowerOff: no PDB to tear down, just dispatch shutdown action.
 *   - setDefaultValues: keep ETL256 off-state GPIO cleanup policy explicit.
 *   - getPowerStateHandler / getHostState / getChassisState /
 *     getPowerStateName: defensive mappings for PDB states ETL256 should
 *     never enter.
 */
class ETL256PowerControl : public VRPowerControl
{
  public:
    ETL256PowerControl(boost::asio::io_context& ioContext,
                       std::shared_ptr<sdbusplus::asio::connection> conn,
                       const std::string& configFilePath,
                       const std::string& node, PersistentState& appState);

    ~ETL256PowerControl() override = default;

    /**
     * @brief Get the handler function for the current power state.
     *
     * Defensive override: ETL256 should never enter waitForPDBMainPowerOk or
     * waitForPDBMainPowerOff because there is no PDB. If that happens, log an
     * error and return an empty handler instead of dispatching to inherited PDB
     * logic.
     */
    std::function<void(Event)> getPowerStateHandler() override;

    /**
     * @brief Get the host state D-Bus property value.
     *
     * Defensive override for unexpected PDB wait states. Normal states fall
     * back to VRPowerControl.
     */
    std::string_view getHostState() const override;

    /**
     * @brief Get the chassis state D-Bus property value.
     *
     * Defensive override for unexpected PDB wait states. Normal states fall
     * back to VRPowerControl.
     */
    std::string_view getChassisState() const override;

    /**
     * @brief Get a human-readable name for the current power state.
     */
    std::string getPowerStateName() const override;

  protected:
    // =========================================================================
    // Pure-virtual overrides required by VRPowerControl
    // =========================================================================

    /**
     * @brief Check if ETL256 host run power is fully off.
     *
     * ETL256 has no PDB power-good signal, so the host is considered off when
     * Board0RunPowerPG, mapped to B0_M0_MODULE_PWR_GOOD-I, is deasserted.
     */
    bool isSystemPowerOff() override;

    /**
     * @brief Handle a power-on request from PowerState::off.
     *
     * ETL256 skips PDB sequencing. If Board0RunPowerPG is already asserted,
     * treat the host as already on. Otherwise transition directly into the
     * inherited HPM power-good assert wait path.
     */
    void handlePowerOnRequest() override;

    /**
     * @brief Complete shutdown after HPM run power has dropped.
     *
     * VRPowerControl calls this after Board0RunPowerPG deasserts during
     * shutdown. ETL256 has no PDB teardown, so this only cancels the HPM
     * watchdog and dispatches the final shutdown action.
     */
    void initiatePDBPowerOff() override;

    // =========================================================================
    // ETL256-specific overrides
    // =========================================================================

    /**
     * @brief Assert ETL256 platform peripherals during power-on.
     *
     * Called by the inherited HPM power-on sequence after PRE_SYS_RST_L is
     * asserted and before RUN_PWR_EN is asserted. ETL256 uses this to enable
     * USB power.
     */
    void assertPlatformPeripherals() override;

    /**
     * @brief Deassert ETL256 platform peripherals during shutdown.
     *
     * Called after RUN_PWR_EN is deasserted. Current bring-up policy follows
     * the ETL256 JSON defaults and deasserts USB power when host run power is
     * off.
     */
    void deassertPlatformPeripherals() override;

    /**
     * @brief Set ETL256 output GPIO defaults for host on/off states.
     *
     * Sets ETL256 USB power defaults, applies the common VR defaults, then
     * adjusts ETL256-specific cleanup policy. In particular, ETL256 should
     * leave PreSystemReset deasserted after shutdown cleanup rather than
     * holding it asserted forever.
     */
    void setDefaultValues() override;

  private:
    /**
     * @brief Power indicator signals used to determine initial hardware state.
     *
     * The host is considered ON iff Board0RunPowerPG is asserted.
     */
    const std::vector<std::string> powerIndicators = {"Board0RunPowerPG"};
};

} // namespace power_control
