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
    NVL144PowerControl(boost::asio::io_context& ioContext, const std::string& configFilePath, std::string node = "0");

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
     * @brief Handler for PowerState::on (NVL144 Override)
     * Override if NVL144 needs platform-specific on-state monitoring.
     */
    void handlePowerStateOn(Event event) override;

    /**
     * @brief Handler for PowerState::off (NVL144 Override)
     * Override if NVL144 needs platform-specific off-state monitoring.
     */
    void handlePowerStateOff(Event event) override;

    /**
     * @brief Handler for PowerState::waitForPDBMainPowerOk (NVL144 Override )
     * 
     * Override if NVL144 needs platform-specific waitForPDBMainPowerOk monitoring.
     */
     void handleWaitForPDBMainPowerOk(Event event) override;

    /**
     * @brief Handler for PowerState::waitForPDBMainPowerOff (NVL144 Override)
     * 
     * Override if NVL144 needs platform-specific waitForPDBMainPowerOff monitoring.
     */
    void handleWaitForPDBMainPowerOff(Event event) override;

    /**
     * @brief Handler for PowerState::waitForCPUResetAssert (NVL144 Override)
     * 
     * Override if NVL144 needs platform-specific waitForCPUResetAssert monitoring.
     */
    void handleWaitForCPUResetAssert (Event event) override;

    /**
     * @brief Handler for PowerState::waitForHPMPowerGoodDeAssert (NVL144 Override)
     * 
     * Override if NVL144 needs platform-specific waitForHPMPowerGoodDeAssert monitoring.
     */
    void handleWaitForHPMPowerGoodDeAssert(Event event) override;

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

    /**
     * @brief Handle shutdown request (forceful or graceful) from PowerState::on
     * 
     * Determines whether to assert CPU Shutdown Force or CPU Shutdown Request based
     * on event type, checks if power is already off, and initiates shutdown sequence.
     */
    void handleShutdownRequest(Event event);

    /**
     * @brief Handle power on request from PowerState::off
     * 
     * Checks if power is already on, and initiates power-on sequence by asserting
     * NVL144 PDB Main Power Enable if necessary.
     */
    void handlePowerOnRequest();

    /**
     * @brief Assert HPM board power sequence during power-on
     * 
     * Asserts Board 0/1 Pre System Reset, E1S/USB Power Enable, de-asserts BMC SSD Reset,
     * and asserts Board 0/1 Run Power Enable.
     */
    void assertHPMBoardPowerSequence();

    /**
     * @brief Transition to HPM Power Good assert wait state
     * 
     * Cancels PDB power watchdog, logs transition, asserts HPM board power sequence,
     * starts HPM power good watchdog, and transitions to waitForHPMPowerGoodAssert.
     */
    void transitionToHPMPowerGoodAssertState();

    /**
     * @brief Complete shutdown and transition to off state
     * 
     * Cancels PDB power watchdog, logs success/failure based on event, sets GPIOs
     * for host state off, and transitions to PowerState::off.
     * 
     * @param success True if shutdown completed successfully, false if watchdog expired
     */
    void completeShutdownAndTransitionToOff(bool success);

    /**
     * @brief De-assert HPM power and peripheral power during shutdown
     * 
     * De-asserts Board 0/1 Run Power Enable, E1S Power Enable, USB Power Enable
     * and asserts BMC SSD Reset when CPUs are in reset during shutdown sequence.
     */
    void deassertHPMPowerAndPeripherals();

    /**
     * @brief Transition to HPM Power Good de-assert wait state
     * 
     * Cancels CPU reset watchdog, logs transition, de-asserts HPM power/peripherals,
     * starts HPM power good watchdog, and transitions to waitForHPMPowerGoodDeAssert.
     */
    void transitionToHPMPowerGoodDeAssertState();

    /**
     * @brief De-assert Pre System Resets and PDB Main Power during shutdown
     * 
     * De-asserts Board 0/1 Pre System Reset and NVL144 PDB Main Power Enable
     * when HPM power good de-asserts during shutdown sequence.
     */
    void deassertPreSystemResetsAndPDBMainPower();

    /**
     * @brief Transition to PDB Main Power Off wait state
     * 
     * Cancels HPM power good watchdog, logs transition, de-asserts Pre System Resets
     * and PDB main power, starts PDB power watchdog, and transitions to waitForPDBMainPowerOff.
     */
    void transitionToPDBMainPowerOffState();

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

