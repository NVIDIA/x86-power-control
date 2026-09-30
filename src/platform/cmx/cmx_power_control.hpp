// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#pragma once

#include "../../vr_power_control.hpp"

namespace power_control
{

/**
 * @brief CMX Power Control class
 *
 * CMX is a single-CPU VR platform with NO BMC-driven PDB stage: the HPM CPLD
 * sequences the 54V HSCs and the run rails, so the BMC only drives HPM
 * sequencing (RUN_PWR_EN, MODULE_PWR_GOOD, PRE_SYS_RST, the SHDN_REQ /
 * SHDN_OK / SHDN_FORCE handshake) and observes CPU reset. There is no Board 1.
 *
 * Inherits directly from VRPowerControl. Overrides:
 *   - handlePowerOnRequest: skip PDB stage, jump straight to HPM Power
 *     Good assert wait.
 *   - isSystemPowerOff: only checks Board0RunPowerPG.
 *   - initiatePDBPowerOff: no PDB to tear down, just dispatch the
 *     shutdown action.
 *   - canAcceptPowerOnRequest: rejects power-on while CpldReady is
 *     de-asserted or after HPM standby power (StbyPwrOk) was lost.
 *   - shouldIgnoreEvent: suppresses IOX events while standby is lost.
 *   - getPowerStateHandler / getHostState / getChassisState /
 *     getPowerStateName: defensive mappings for the waitForPDBMainPowerOk /
 *     waitForPDBMainPowerOff power states which CMX should never enter.
 */
class CMXPowerControl : public VRPowerControl
{
  public:
    CMXPowerControl(boost::asio::io_context& ioContext,
                    std::shared_ptr<sdbusplus::asio::connection> conn,
                    const std::string& configFilePath, const std::string& node,
                    PersistentState& appState);

    ~CMXPowerControl() override = default;

    std::function<void(Event)> getPowerStateHandler() override;
    std::string_view getHostState() const override;
    std::string_view getChassisState() const override;
    std::string getPowerStateName() const override;

  protected:
    bool isSystemPowerOff() override;
    void handlePowerOnRequest() override;
    void initiatePDBPowerOff() override;
    bool canAcceptPowerOnRequest(std::string& reason) override;
    bool shouldIgnoreEvent(const std::string& signalName) override;

  private:
    void cpldReadyHandler(bool state);
    void stbyPwrOkHandler(bool state);
    void markStandbyLost();

    /**
     * @brief Power indicator signals used to determine initial hardware
     * power state
     *
     * The host is considered ON iff Board0RunPowerPG is asserted.
     */
    const std::vector<std::string> powerIndicators = {"Board0RunPowerPG"};

    // Set when StbyPwrOk de-asserts; blocks power-on and suppresses IOX
    // events while the HPM standby power domain is down. Cleared only by AC
    // cycle or BMC reboot.
    bool stbyPowerLost = false;
};

} // namespace power_control
