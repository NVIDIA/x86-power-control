// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#pragma once

#include "../../vr_power_control.hpp"

namespace power_control
{

/**
 * @brief VC-256 Power Control class
 *
 * Single-board VR platform with a PDB-equivalent HSC (Hot Swap Controller)
 * stage ahead of HPM run-power sequencing, mirroring NVL72's PDB Main Power
 * mechanism with the HSC signals in place of NVL72's PDB PSU signals.
 * Inherits all shared VR/HPM sequencing (including the generic
 * waitForPDBMainPowerOk / waitForPDBMainPowerOff states) from
 * VRPowerControl. Overrides only the VC-256-specific pieces:
 *   - handlePowerOnRequest / initiatePDBPowerOff: assert/de-assert
 *     PDBMainPowerEnable (P54V_HSC_EN-O) and wait for PDBMainPowerOk
 *     (P54V_HSC_PG-I), same mechanism as NVL72's PDB Main Power stage.
 *   - isSystemPowerOff: checks Board0RunPowerPG AND PDBMainPowerOk.
 *   - pdbMainPowerOkHandler / checkAndHandlePdbMainPowerOkFault: dispatch
 *     PDBMainPowerOk edge events and treat an unexpected de-assert outside
 *     waitForPDBMainPowerOff as a power fault, same as NVL72 (without
 *     NVL72's HSC alert-mask WAR, which is hardware-specific to its HSC
 *     vendor).
 *   - canAcceptPowerOnRequest: rejects power-on when standby power is lost.
 *   - shouldIgnoreEvent: suppresses IOX events while standby is lost.
 *   - assertPlatformPeripherals / deassertPlatformPeripherals: drive
 *     HostReadyPowerEnable around Board0 Run Power Enable, same mechanism
 *     as USBPowerEnable on C2/NVL72.
 *   - setDefaultValues: adds HostReadyPowerEnable host-on/off defaults on
 *     top of the shared VR/HPM defaults.
 */
class VC256PowerControl : public VRPowerControl
{
  public:
    VC256PowerControl(boost::asio::io_context& ioContext,
                      std::shared_ptr<sdbusplus::asio::connection> conn,
                      const std::string& configFilePath,
                      const std::string& node, PersistentState& appState);

    ~VC256PowerControl() override = default;

  protected:
    bool isSystemPowerOff() override;
    void handlePowerOnRequest() override;
    void initiatePDBPowerOff() override;
    bool canAcceptPowerOnRequest(std::string& reason) override;
    bool shouldIgnoreEvent(const std::string& signalName) override;
    void assertPlatformPeripherals() override;
    void deassertPlatformPeripherals() override;
    void setDefaultValues() override;
    void validateTimerConfigs() override;

  private:
    void stbyPwrOkHandler(bool state);
    void markStandbyLost();

    // Sample StbyPwrOk directly and latch loss if it reads de-asserted. The
    // GPIO callback only fires on edges, so a rail that is already down when
    // this service starts produces no event to observe.
    void refreshStandbyLostFromHardware();

    // PDBMainPowerOk (P54V_HSC_PG-I) edge handler. Dispatches
    // pdbMainPowerOkAssert/DeAssert events, same as NVL72.
    void pdbMainPowerOkHandler(bool state);

    // Treats an unexpected PDBMainPowerOk de-assert outside
    // waitForPDBMainPowerOff as a power fault and forces off. Returns true
    // if a fault was handled (caller should not dispatch the event further).
    bool checkAndHandlePdbMainPowerOkFault(Event powerControlEvent);

    // De-assert PDBMainPowerEnable (P54V_HSC_EN-O).
    void deassertPDBMainPower();

    /**
     * @brief Power indicators used to determine host on/off from hardware
     *
     * Host is ON only if BOTH Board0RunPowerPG AND PDBMainPowerOk are
     * asserted.
     */
    const std::vector<std::string> powerIndicators = {"Board0RunPowerPG",
                                                      "PDBMainPowerOk"};

    /**
     * @brief VC-256-specific required timer configuration keys
     */
    const std::vector<std::string> vc256RequiredTimeoutValues = {
        "PdbMainPowerOkWatchdogMs",
    };

    // Set when StbyPwrOk de-asserts; blocks power-on and suppresses IOX
    // events while the standby power domain that feeds the sequencing IOX
    // is down. Cleared only by AC cycle or BMC reboot.
    bool stbyPowerLost = false;
};

} // namespace power_control
