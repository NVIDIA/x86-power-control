/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#pragma once

#include "../../vr_power_control.hpp"

namespace power_control
{

/**
 * @brief Parsec Power Control class
 * 
 * This class represents the Parsec platform-specific power control.
 * Parsec uses GB300 PDB which has similar sequencing to NVL144:
 * - Uses GB300 PDB Main Power Enable/OK signals
 * - After GB300 PDB Main Power OK asserts, proceeds directly to HPM sequencing
 * - Does NOT use E1S Power Enable or BMC SSD Reset (unlike NVL144)
 * - Does NOT use 12V rail enables (unlike C2)
 * 
 * Key difference from NVL144:
 * - GB300 PDB Main Power OK is monitored via input event (gpio_keys_gb300)
 *   instead of direct GPIO line, so event handling differs slightly
 * - Simpler HPM sequencing (no E1S/BMC SSD toggles)
 */
class ParsecPowerControl : public VRPowerControl
{
public:
    ParsecPowerControl() = default;
    virtual ~ParsecPowerControl() = default;

    /**
     * @brief Get the handler function for a given power state
     * 
     * Parsec does not define new PowerState values, so this delegates to VRPowerControl.
     * 
     * @param state The power state to get a handler for
     * @return Function that handles events in the given state
     */
    std::function<void(Event)> getPowerStateHandler(PowerState state) override;

protected:
    /**
     * @brief Handler for PowerState::on (Parsec Override)
     * 
     * PREVIOUS IMPLEMENTATION (Parsec/GB300-specific from powerStateOn):
     * 
     * Overrides VRPowerControl::handlePowerStateOn() to handle Parsec-specific monitoring:
     * 
     * Parsec-specific monitoring:
     * - Event::gb300pdbMainPowerOkDeAssert → unexpected GB300 PDB power loss
     */
    void handlePowerStateOn(Event event) override;

    /**
     * @brief Handler for PowerState::off (Parsec Override)
     * 
     * PREVIOUS IMPLEMENTATION (Parsec/GB300-specific from powerStateOff):
     * 
     * Overrides VRPowerControl::handlePowerStateOff() to customize GB300 PDB power-on:
     * 
     * 
     * NOTE: Simpler than NVL144 - no E1S or BMC SSD toggling (if GB300 PDB is already powered on)
     */
    void handlePowerStateOff(Event event) override;

    /**
     * @brief Handler for PowerState::waitForPDBMainPowerOk (Parsec Override)
     * 
     * PREVIOUS IMPLEMENTATION (Parsec/GB300-specific from powerStateWaitForPDBMainPowerOk):
     * 
     * Overrides VRPowerControl::handleWaitForPDBMainPowerOk() to handle GB300 PDB:
     * 
     * NOTE: no E1S/BMC SSD toggling (compared to NVL144)
     */
    void handleWaitForPDBMainPowerOk(Event event) override;
};

} // namespace power_control

