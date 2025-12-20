/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#pragma once

#include "../../vr_power_control.hpp"

namespace power_control
{

/**
 * @brief NVL144 Power Control class
 * 
 * This class represents the NVL144 platform-specific power control.
 * Since VRPowerControl's default implementations already use NVL144 behavior,
 * this class typically does NOT need to override anything unless there are
 * NVL144-specific deviations from the VR defaults.
 * 
 * Platform characteristics:
 * - Has NVL144 PDB (Power Distribution Board)
 * - Uses E1S Power Enable
 * - Uses BMC SSD Reset
 * - Supports Board 0 and optionally Board 1
 * - After PDB powers up, asserts E1S and BMC SSD enables before HPM sequencing
 */
class NVL144PowerControl : public VRPowerControl
{
public:
    NVL144PowerControl(boost::asio::io_context& ioContext,
                       std::shared_ptr<sdbusplus::asio::connection> conn,
                       const std::string& node);
    virtual ~NVL144PowerControl() = default;

    /**
     * @brief Get the handler function for a given power state
     * 
     * NVL144 does not add new states, so this delegates to VRPowerControl.
     * 
     * @param state The power state to get a handler for
     * @return Function that handles events in the given state
     */
    std::function<void(Event)> getPowerStateHandler(PowerState state) override;

protected:
    /**
     * @brief Validate that all required signals for NVL144 platform are present in config
     * 
     * Checks for NVL144-specific PDB signals, then calls VRPowerControl::validateRequiredSignals()
     * to check common VR signals based on board presence.
     * 
     * @throws std::runtime_error if any required signal is missing from config
     */
    void validateRequiredSignals() override;

protected:
    // NVL144 uses the default VR implementations (which are NVL144 behavior)
    // Override only if NVL144 needs platform-specific variations
    
    /**
     * @brief Handler for PowerState::on (NVL144 Override - optional)
     * 
     * Currently uses VRPowerControl default implementation.
     * Override if NVL144 needs platform-specific on-state monitoring.
     */
    void handlePowerStateOn(Event event) override;

    /**
     * @brief Handler for PowerState::off (NVL144 Override - optional)
     * 
     * Currently uses VRPowerControl default implementation.
     * Override if NVL144 needs platform-specific off-state monitoring.
     */
    void handlePowerStateOff(Event event) override;

    /**
     * @brief Set all control GPIOs to match the host state "on" (NVL144 override)
     * 
     * Sets NVL144 PDB control GPIOs, then calls VRPowerControl::setGPIOsForHostStateOn()
     * to set VR control GPIOs.
     */
    void setGPIOsForHostStateOn() override;

    /**
     * @brief Set all control GPIOs to match the host state "off" (NVL144 override)
     * 
     * Sets NVL144 PDB control GPIOs, then calls VRPowerControl::setGPIOsForHostStateOff()
     * to set VR control GPIOs.
     */
    void setGPIOsForHostStateOff() override;


private:
    /**
     * @brief Required NVL144 PDB signals (always required for NVL144 platform)
     */
    const std::vector<std::string> requiredSignals = {
        "NVL144PDBMainPowerOk",
        "NVL144PDBMainPowerEnable",
        "E1SPowerEnable",
        "BMCSSDReset",
        // TODO: Add E1S, BMC SSD signals when implemented
    };

private:
    // NVL144-SPECIFIC GPIO HANDLERS (Member functions)
    
    /**
     * @brief Handler for NVL144 PDB Main Power OK GPIO events
     * 
     * - If state == true: Send Event::nvl144pdbMainPowerOkAssert
     * - If state == false: Send Event::nvl144pdbMainPowerOkDeAssert
     * 
     * @param state The GPIO state (true = asserted, false = de-asserted)
     */
    void nvl144pdbMainPowerOkHandler(bool state);
    
};

} // namespace power_control

