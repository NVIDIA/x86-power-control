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
    ParsecPowerControl(boost::asio::io_context& ioContext,
                       std::shared_ptr<sdbusplus::asio::connection> conn,
                       const std::string& configFilePath,
                       const std::string& node, PersistentState& appState);
    virtual ~ParsecPowerControl() = default;

    /**
     * @brief Get the handler function for a given power state
     *
     * Parsec does not define new PowerState values, so this delegates to
     * VRPowerControl.
     *
     * @param state The power state to get a handler for
     * @return Function that handles events in the given state
     */
    std::function<void(Event)> getPowerStateHandler() override;

  protected:
    /**
     * @brief Validate that all required signals for Parsec platform are present
     * in config
     *
     * Checks for Parsec-specific GB300 PDB signals, then calls
     * VRPowerControl::validateRequiredSignals() to check common VR signals
     * based on board presence.
     *
     * @throws std::runtime_error if any required signal is missing from config
     */
    void validateRequiredSignals() override;

    /**
     * @brief Validate that all required timer configurations for Parsec
     * platform are present in TimerMap
     *
     * TODO: Implement Parsec-specific timer validation when timers are added
     * Checks for Parsec-specific PDB timer, then calls
     * VRPowerControl::validateTimerConfigs() to check common VR timers.
     *
     * @throws std::runtime_error if any required timer config is missing
     */
    void validateTimerConfigs() override;

  private:
    /**
     * @brief Handler for GB300 PDB Main Power OK GPIO events
     *
     * - If state == true: Send Event::gb300pdbMainPowerOkAssert
     * - If state == false: Send Event::gb300pdbMainPowerOkDeAssert
     *
     * @param state The GPIO state (true = asserted, false = de-asserted)
     */
    void gb300pdbMainPowerOkHandler(bool state);

    /**
     * @brief Cached state for GB300 PDB Main Power OK signal
     *
     * Since gpio_keys_polled has exclusive control of the GPIO, we can't
     * request the GPIO state directly. Instead, we track the state based on
     * input events.
     */
    int gb300pdbMainPowerOkState = 0;

  protected:
    /**
     * @brief Required Parsec GB300 PDB signals
     */
    const std::vector<std::string> requiredSignals = {
        "GB300PDBMainPowerOk", "GB300PDBMainPowerEnable"};

    /**
     * @brief List of required Parsec platform-specific timer configurations
     * TODO: Add ParsecPdbMainPowerOkWatchdogMs when Parsec sequencing is
     * implemented
     */
    const std::vector<std::string> platformRequiredTimeoutValues = {
        // Empty for now - add "ParsecPdbMainPowerOkWatchdogMs" when needed
    };

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
    /**
     * @brief Handler for PowerState::on (Parsec Override)
     *
     * PREVIOUS IMPLEMENTATION (Parsec/GB300-specific from powerStateOn):
     *
     * Overrides VRPowerControl::handlePowerStateOn() to handle Parsec-specific
     * monitoring:
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
     * Overrides VRPowerControl::handlePowerStateOff() to customize GB300 PDB
     * power-on:
     *
     *
     * NOTE: Simpler than NVL144 - no E1S or BMC SSD toggling (if GB300 PDB is
     * already powered on)
     */
    void handlePowerStateOff(Event event) override;

    /**
     * @brief Handler for PowerState::waitForPDBMainPowerOk (Parsec Override)
     *
     * PREVIOUS IMPLEMENTATION (Parsec/GB300-specific from
     * powerStateWaitForPDBMainPowerOk):
     *
     * Overrides VRPowerControl::handleWaitForPDBMainPowerOk() to handle GB300
     * PDB:
     *
     * NOTE: no E1S/BMC SSD toggling (compared to NVL144)
     */
    void handleWaitForPDBMainPowerOk(Event event) override;

    /**
     * @brief Set default values for Parsec output signals (Parsec override)
     *
     * Sets Parsec GB300 PDB-specific default values for output signals, then
     * calls VRPowerControl::setDefaultValues() to set common VR defaults.
     */
    void setDefaultValues() override;
};

} // namespace power_control
