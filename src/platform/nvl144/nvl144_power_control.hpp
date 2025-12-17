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
    NVL144PowerControl(boost::asio::io_context& ioContext);
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

    /**
     * @brief Initialize NVL144-specific GPIO events (Phase 2 of initialization)
     * 
     * This method is called after loadConfigValues() and detectBoardPresence().
     * It first calls VRPowerControl::initializeGPIO() to register common VR GPIOs,
     * then registers NVL144-specific GPIOs:
     * - NVL144 PDB Main Power OK
     * - E1S Power Enable
     * - BMC SSD Reset
     */
    void initializeGPIO() override;

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

private:
    // =============================================================================
    // NVL144-SPECIFIC ConfigData OBJECTS
    // =============================================================================
    
    ConfigData nvl144pdbMainPowerOkConfig;
    ConfigData nvl144pdbMainPowerEnableConfig;
    ConfigData e1sPowerEnableConfig;
    ConfigData bmcSSDResetConfig;
    
    // =============================================================================
    // NVL144-SPECIFIC GPIO LINES
    // =============================================================================
    
    gpiod::line nvl144pdbMainPowerOkLine;
    gpiod::line nvl144pdbMainPowerEnableLine;
    gpiod::line e1sPowerEnableLine;
    gpiod::line bmcSSDResetLine;
    
    // =============================================================================
    // NVL144-SPECIFIC EVENT DESCRIPTORS
    // =============================================================================
    
    boost::asio::posix::stream_descriptor nvl144pdbMainPowerOkEvent;
    
    // =============================================================================
    // NVL144-SPECIFIC GPIO HANDLERS (Member functions)
    // =============================================================================
    
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

