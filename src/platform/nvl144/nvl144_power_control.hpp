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
                       const std::string& configFilePath,
                       const std::string& node, PersistentState& appState);

    virtual ~NVL144PowerControl() = default;

    /**
     * @brief Get the handler function for a given power state
     *
     * NVL144 does not add new states, so this delegates to VRPowerControl.
     *
     * @param state The power state to get a handler for
     * @return Function that handles events in the given state
     */
    std::function<void(Event)> getPowerStateHandler() override;

  protected:
    /**
     * @brief Validate that all required signals for NVL144 platform are present
     * in config
     *
     * Checks for NVL144-specific PDB signals, then calls
     * VRPowerControl::validateRequiredSignals() to check common VR signals
     * based on board presence.
     *
     * @throws std::runtime_error if any required signal is missing from config
     */

    /**
     * @brief Validate that all required timer configurations for NVL144
     * platform are present in TimerMap
     *
     * Checks for NVL144-specific PDB timer, then calls
     * VRPowerControl::validateTimerConfigs() to check common VR timers.
     *
     * @throws std::runtime_error if any required timer config is missing
     */
    void validateTimerConfigs() override;

    /**
     * @brief Add Board 1 GPIO state properties
     *
     * Adds Board 1 GPIO state properties to the GPIO state interface. 
     * Board 1 SHDN OK is only present if Board 1 is present.
     *
     * @return void
    */
    void addBoard1GpioStateProperties();
     
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
     * Override if NVL144 needs platform-specific waitForPDBMainPowerOk
     * monitoring.
     */
    void handleWaitForPDBMainPowerOk(Event event) override;

    /**
     * @brief Handler for PowerState::waitForPDBMainPowerOff (NVL144 Override)
     *
     * Override if NVL144 needs platform-specific waitForPDBMainPowerOff
     * monitoring.
     */
    void handleWaitForPDBMainPowerOff(Event event) override;

    /**
     * @brief Handler for PowerState::waitForCPUResetAssert (NVL144 Override)
     *
     * Override if NVL144 needs platform-specific waitForCPUResetAssert
     * monitoring.
     */
    void handleWaitForCPUResetAssert(Event event) override;

    /**
     * @brief Handler for PowerState::waitForHPMPowerGoodDeAssert (NVL144
     * Override)
     *
     * Override if NVL144 needs platform-specific waitForHPMPowerGoodDeAssert
     * monitoring.
     */
    void handleWaitForHPMPowerGoodDeAssert(Event event) override;

    /**
     * @brief Set default values for NVL144 output signals (NVL144 override)
     *
     * Sets NVL144 PDB-specific default values for output signals, then calls
     * VRPowerControl::setDefaultValues() to set common VR defaults.
     */
    void setDefaultValues() override;

    /**
     * @brief Handle shutdown request (forceful or graceful) from PowerState::on
     *
     * Determines whether to assert CPU Shutdown Force or CPU Shutdown Request
     * based on event type, checks if power is already off, and initiates
     * shutdown sequence.
     */
    void handleShutdownRequest(Event event);

    /**
     * @brief Check if system power is already off
     *
     * @return true if both Board0RunPowerPG and NVL144PDBMainPowerOk are de-asserted
     */
    bool isSystemPowerOff();

    /**
     * @brief Initiate CPU shutdown sequence
     *
     * @param shutdownSignalName Name of the shutdown signal to assert
     * @param shutdownOkTimeout Timeout for CPU Shutdown OK watchdog
     * @param shutdownAction Description of shutdown action for logging
     *
     * Asserts the specified shutdown signal, starts the watchdog timer,
     * and transitions to waitForCPUShutdownOk state.
     */
    void initiateCPUShutdown(const char* shutdownSignalName,
                             int shutdownOkTimeout,
                             const char* shutdownAction);

    /**
     * @brief Handle power on request from PowerState::off
     *
     * Checks if power is already on, and initiates power-on sequence by
     * asserting NVL144 PDB Main Power Enable if necessary.
     */
    void handlePowerOnRequest();

    /**
     * @brief Handle power cycle request when in off state
     *
     * @param event The power cycle event (powerCycleRequest or gracefulPowerCycleRequest)
     *
     * Verifies power is actually off by checking Board0RunPowerPG, then
     * initiates power on sequence. If power is not fully off, initiates
     * appropriate shutdown (forceful or graceful) first based on event type.
     */
    void handlePowerCycleWhenOff(Event event);

    /**
     * @brief Assert HPM board power sequence during power-on
     *
     * Asserts Board 0/1 Pre System Reset, E1S/USB Power Enable, de-asserts BMC
     * SSD Reset, and asserts Board 0/1 Run Power Enable.
     */
    void assertHPMBoardPowerSequence();

    /**
     * @brief Transition to HPM Power Good assert wait state
     *
     * Cancels PDB power watchdog, logs transition, asserts HPM board power
     * sequence, starts HPM power good watchdog, and transitions to
     * waitForHPMPowerGoodAssert.
     */
    void transitionToHPMPowerGoodAssertState();

    /**
     * @brief Complete shutdown and transition to off state
     *
     * Cancels PDB power watchdog, logs success/failure based on event, sets
     * GPIOs for host state off, and transitions to PowerState::off.
     *
     * @param success True if shutdown completed successfully, false if watchdog
     * expired
     */
    void completeShutdownAndTransitionToOff(bool success);

    /**
     * @brief Transition to off state after successful shutdown
     *
     * Clears action, sets GPIOs to off state, transitions to off.
     */
    void transitionToOffState();

    /**
     * @brief Transition to power cycle delay state
     *
     * Preserves action, sets GPIOs to off state, starts delay timer,
     * transitions to waitForPowerCycleDelay.
     */
    void transitionToPowerCycleDelay();

    /**
     * @brief De-assert HPM power and peripheral power during shutdown
     *
     * De-asserts Board 0/1 Run Power Enable, E1S Power Enable, USB Power Enable
     * and asserts BMC SSD Reset when CPUs are in reset during shutdown
     * sequence.
     */
    void deassertHPMPowerAndPeripherals();

    /**
     * @brief Transition to HPM Power Good de-assert wait state
     *
     * Cancels CPU reset watchdog, logs transition, de-asserts HPM
     * power/peripherals, starts HPM power good watchdog, and transitions to
     * waitForHPMPowerGoodDeAssert.
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
     * Cancels HPM power good watchdog, logs transition, de-asserts Pre System
     * Resets and PDB main power, starts PDB power watchdog, and transitions to
     * waitForPDBMainPowerOff.
     */
    void transitionToPDBMainPowerOffState();

  private:
    /**
     * @brief Required NVL144 PDB signals (always required for NVL144 platform)
     */
    /**
     * @brief List of required NVL144 platform-specific timer configurations
     */
    const std::vector<std::string> platformRequiredTimeoutValues = {
        "NVL144PdbMainPowerOkWatchdogMs",
    };

    /**
     * @brief Timer for NVL144 PDB main power OK assertion/de-assertion in PDB
     * power sequencing
     */
    boost::asio::steady_timer pdbMainPowerOkWatchdogTimer;

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
