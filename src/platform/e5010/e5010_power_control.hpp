/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#pragma once

#include "../../vr_power_control.hpp"

namespace power_control
{

/**
 * @brief E5010 Power Control class
 * 
 * This class represents the E5010 platform-specific power control.
 * E5010 is unique because it does NOT have a PDB (Power Distribution Board):
 * - No PDB power sequencing required
 * - Goes directly to HPM board power sequencing
 * - Should NEVER transition to waitForPDBMainPowerOk or waitForPDBMainPowerOff states
 */
class E5010PowerControl : public VRPowerControl
{
public:
    E5010PowerControl(boost::asio::io_context& ioContext,
                      std::shared_ptr<sdbusplus::asio::connection> conn,
                      const std::string& node);
    virtual ~E5010PowerControl() = default;

    /**
     * @brief Get the handler function for a given power state
     * 
     * TODO: Add E5010-specific states here, that are overriden by E5010PowerControl
     * 
     * @param state The power state to get a handler for
     * @return Function that handles events in the given state
     */
    std::function<void(Event)> getPowerStateHandler(PowerState state) override;

protected:
    /**
     * @brief Validate that all required signals for E5010 platform are present in config
     * 
     * E5010 has no PDB, so only calls VRPowerControl::validateRequiredSignals()
     * to check common VR signals based on board presence.
     * 
     * @throws std::runtime_error if any required signal is missing from config
     */
    void validateRequiredSignals() override;

protected:
    /**
     * @brief Handler for PowerState::on (E5010 Override)
     * 
     * PREVIOUS IMPLEMENTATION (E5010-specific from powerStateOn):
     * 
     * Overrides VRPowerControl::handlePowerStateOn()

     * E5010-specific: Simplified monitoring (no PDB faults to track)
     */
    void handlePowerStateOn(Event event) override;

    /**
     * @brief Handler for PowerState::off (E5010 Override)
     * 
     * PREVIOUS IMPLEMENTATION (E5010-specific from powerStateOff in power_control.cpp):
     * 
     * Overrides VRPowerControl::handlePowerStateOff() no PDB sequencing
     * 
     * 
     * NOTE: Completely skips PDB power-on sequence
     * NOTE: Goes directly from off → waitForHPMPowerGoodAssert
     */
    void handlePowerStateOff(Event event) override;

    // E5010 does NOT override handleWaitForPDBMainPowerOk because it should
    // never enter that state. If it does, that's an error condition.
    
    /**
     * @brief Handler for PowerState::waitForHPMPowerGoodDeAssert (E5010 Override)
     * 
     * PREVIOUS IMPLEMENTATION (E5010-specific):
     * 
     * Overrides VRPowerControl::waitForHPMPowerGoodDeAssert() to skip PDB power-off sequence and go directly to off:
     * 
     * NOTE: Skips waitForPDBMainPowerOff state entirely
     */
    void handleWaitForHPMPowerGoodDeAssert(Event event) override;

    /**
     * @brief Set all control GPIOs to match the host state "on" (E5010 override)
     * 
     * E5010 has no PDB, so just calls VRPowerControl::setGPIOsForHostStateOn()
     * to set VR control GPIOs.
     */
    void setGPIOsForHostStateOn() override;

    /**
     * @brief Set all control GPIOs to match the host state "off" (E5010 override)
     * 
     * E5010 has no PDB, so just calls VRPowerControl::setGPIOsForHostStateOff()
     * to set VR control GPIOs.
     */
    void setGPIOsForHostStateOff() override;
};

} // namespace power_control

