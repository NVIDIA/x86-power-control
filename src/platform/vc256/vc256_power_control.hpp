// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#pragma once

#include "../../vr_power_control.hpp"

namespace power_control
{

/**
 * @brief VC-256 Power Control class
 *
 * Single-board VR platform with no PDB and HPM-only run-power sequencing.
 * Inherits all shared VR/HPM sequencing from VRPowerControl. Overrides
 * only the VC-256-specific pieces:
 *   - handlePowerOnRequest: skip PDB stage, go straight to HPM run power.
 *   - isSystemPowerOff: only checks Board0RunPowerPG / MODULE_PWR_GOOD.
 *   - initiatePDBPowerOff: no PDB to tear down, dispatch shutdown action.
 *   - canAcceptPowerOnRequest: rejects power-on when standby power is lost.
 *   - shouldIgnoreEvent: suppresses IOX events while standby is lost.
 *   - getPowerStateHandler / getHostState / getChassisState /
 *     getPowerStateName: defensive mappings for PDB states this platform
 *     should never enter.
 */
class VC256PowerControl : public VRPowerControl
{
  public:
    VC256PowerControl(boost::asio::io_context& ioContext,
                      std::shared_ptr<sdbusplus::asio::connection> conn,
                      const std::string& configFilePath,
                      const std::string& node, PersistentState& appState);

    ~VC256PowerControl() override = default;

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
    void stbyPwrOkHandler(bool state);
    void markStandbyLost();

    // Sample StbyPwrOk directly and latch loss if it reads de-asserted. The
    // GPIO callback only fires on edges, so a rail that is already down when
    // this service starts produces no event to observe.
    void refreshStandbyLostFromHardware();

    const std::vector<std::string> powerIndicators = {"Board0RunPowerPG"};

    // Set when StbyPwrOk de-asserts; blocks power-on and suppresses IOX
    // events while the standby power domain that feeds the sequencing IOX
    // is down. Cleared only by AC cycle or BMC reboot.
    bool stbyPowerLost = false;
};

} // namespace power_control
