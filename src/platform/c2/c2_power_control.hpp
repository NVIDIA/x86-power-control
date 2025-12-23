/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#pragma once

#include "../../vr_power_control.hpp"

namespace power_control
{

/**
 * @brief C2 Power Control class
 * 
 * This class represents the C2 platform-specific power control.
 * C2 has unique PDB behavior compared to NVL144:
 * - Uses C2 PDB with PSU Power On/OK signals (not Main Power Enable/OK)
 * - After PDB PSU Power OK asserts, must enable 12V rails:
 *   - 12V HPM Enable
 *   - 12V GPU1 Enable
 *   - 12V GPU2 Enable
 * - Then proceeds to HPM sequencing (Pre System Reset + Run Power Enable)
 * - Does NOT use E1S Power Enable or BMC SSD Reset
 * 
 * Platform variants:
 * - C2 Non-Intelligent PDB (no MCU) - fully controlled by BMC
 * - C2 Intelligent PDB (with MCU) - partially managed by PDB MCU
 */
class C2PowerControl : public VRPowerControl
{
public:

    C2PowerControl(boost::asio::io_context& ioContext,
                   std::shared_ptr<sdbusplus::asio::connection> conn,
                   const std::string& configFilePath,
                   const std::string& node,
                   PersistentState& appState);
    virtual ~C2PowerControl() = default;

    /**
     * @brief Get the handler function for a given power state
     * 
     * C2 modifies the handleWaitForHPMPowerGoodDeAssert and handlePowerStateOff handlers to 
     * handle the C2 PDB 12V rails.
     * 
     * @param state The power state to get a handler for
     * @return Function that handles events in the given state
     */
    std::function<void(Event)> getPowerStateHandler() override;

protected:
    /**
     * @brief Validate that all required signals for C2 platform are present in config
     * 
     * Checks for C2-specific PDB signals, then calls VRPowerControl::validateRequiredSignals()
     * to check common VR signals based on board presence.
     * 
     * @throws std::runtime_error if any required signal is missing from config
     */
    void validateRequiredSignals() override;

private:
    // C2-SPECIFIC GPIO HANDLERS (Member functions)
    
    /**
     * @brief Handler for C2 PDB PSU Power OK GPIO events
     * 
     * - If state == true: Send Event::c2pdbPSUPowerOkAssert
     * - If state == false: Send Event::c2pdbPSUPowerOkDeAssert
     * 
     * @param state The GPIO state (true = asserted, false = de-asserted)
     */
    void c2pdbPSUPowerOkHandler(bool state);

protected:
    /**
     * @brief Required C2 PDB signals
     */
    const std::vector<std::string> requiredSignals = {
        "C2PDBPSUPowerOk",
        "C2PDB12VHPMEnable",
        "C2PDB12VGPU1Enable",
        "C2PDB12VGPU2Enable"
    };

protected:

    // OVERRIDDEN VR STATE HANDLERS (C2-specific behavior)
    
    /**
     * @brief Handler for PowerState::on (C2 Override)
     * 
     * PREVIOUS IMPLEMENTATION (C2-specific from powerStateOn):
     * 
     * Overrides VRPowerControl::handlePowerStateOn() to handle C2-specific monitoring:
     *
     */
    void handlePowerStateOn(Event event) override;

    /**
     * @brief Handler for PowerState::off (C2 Override)
     * 
     * PREVIOUS IMPLEMENTATION (C2-specific from powerStateOff in power_control.cpp):
     * 
     * Overrides VRPowerControl::handlePowerStateOff() to customize C2 PDB power-on:
     */
    void handlePowerStateOff(Event event) override;

    /**
     * @brief Handler for PowerState::waitForPDBMainPowerOk (C2 Override)
     * 
     * PREVIOUS IMPLEMENTATION (C2-specific from powerStateWaitForPDBMainPowerOk):
     * 
     * Overrides VRPowerControl::handleWaitForPDBMainPowerOk() to handle C2 PDB:
     * 
     * NOTE: This is the KEY DIFFERENCE from NVL144 - C2 must enable 12V rails
     *       after PDB powers up, whereas NVL144 only needs to enable E1S/BMC SSD
     */
    void handleWaitForPDBMainPowerOk(Event event) override;

    /**
     * @brief Handler for PowerState::waitForHPMPowerGoodDeAssert (C2 Override)
     * 
     * PREVIOUS IMPLEMENTATION (C2-specific from powerStateWaitForHPMPowerGoodDeAssert):
     * 
     * Overrides VRPowerControl::handleWaitForHPMPowerGoodDeAssert() to handle de-assertion of 12V C2 PDB rails:
     * 
     */
    void handleWaitForHPMPowerGoodDeAssert(Event event) override;

    /**
     * @brief Set all control GPIOs to match the host state "on" (C2 override)
     * 
     * Sets C2 PDB control GPIOs, then calls VRPowerControl::setGPIOsForHostStateOn()
     * to set VR control GPIOs.
     */
    void setGPIOsForHostStateOn() override;

    /**
     * @brief Set all control GPIOs to match the host state "off" (C2 override)
     * 
     * Sets C2 PDB control GPIOs, then calls VRPowerControl::setGPIOsForHostStateOff()
     * to set VR control GPIOs.
     */
    void setGPIOsForHostStateOff() override;
};

} // namespace power_control

