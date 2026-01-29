/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#include "parsec_power_control.hpp"

namespace power_control
{

// Constructor: Adds Parsec-specific ConfigData entries to powerSignalMap
ParsecPowerControl::ParsecPowerControl(
    boost::asio::io_context& ioContext,
    std::shared_ptr<sdbusplus::asio::connection> conn,
    const std::string& configFilePath, const std::string& node,
    PersistentState& appState) :
    VRPowerControl(ioContext, conn, configFilePath, node,
                   appState) // Call parent constructor (registers VR GPIOs)
{
    // powerSignalMap is now populated by base class
    // PowerControl::loadConfigValues() VR handlers already added to
    // gpioHandlerMap by VRPowerControl constructor Now add Parsec-specific
    // handlers to the map

    addRequiredSignal("GB300PDBMainPowerOk", 0, GPIODirection::IN,
                      [this](bool state) {
                          this->gb300pdbMainPowerOkHandler(state);
                      });
    addRequiredSignal("GB300PDBMainPowerEnable", 0, GPIODirection::OUT);

    if (boardPresence.board1Present)
    {
        addRequiredSignal("Board1RunPowerEnable", 1, GPIODirection::OUT);
        addRequiredSignal("Board1PreSystemReset", 1, GPIODirection::OUT);
        addRequiredSignal("Board1CpuShutdownOk", 1, GPIODirection::IN);
        addBoard1GpioStateProperties();
    }

    // call validateRequiredSignals() to validate all required signals
    validateRequiredSignals();

    // call validateTimerConfigs() to validate all required timers
    validateTimerConfigs();

    // call setDefaultValues() to set the default values for output signals
    setDefaultValues();

    // Initialize ALL host0 interfaces at once - this makes the path visible to
    // ObjectMapper After this, mapper wait /xyz/openbmc_project/state/host0
    // will return and ALL interfaces (Host, Boot.Progress, OS, Gpio) will be
    // ready
    initializeHostStateInterface();

    // Register all GPIO handlers (from base, VR, and Parsec)
    registerGPIOHandlers();
}

// PARSEC-SPECIFIC GPIO EVENT HANDLERS

void ParsecPowerControl::gb300pdbMainPowerOkHandler(bool state)
{
    auto it = powerSignalMap.find("GB300PDBMainPowerOk");
    if (it == powerSignalMap.end())
    {
        lg2::error("GB300PDBMainPowerOk signal not found in powerSignalMap");
        return;
    }

    auto& config = *it->second;
    Event powerControlEvent = (state == config.polarity)
                                  ? Event::gb300pdbMainPowerOkAssert
                                  : Event::gb300pdbMainPowerOkDeAssert;
    this->sendPowerControlEvent(powerControlEvent, powerState);
}

std::function<void(Event)> ParsecPowerControl::getPowerStateHandler()
{
    // Parsec does not define new PowerState values, so delegate everything
    // to VRPowerControl which handles all VR and upstream states
    switch (powerState)
    {
        // TODO: Add Parsec-specific states here, that are overridden by
        // ParsecPowerControl

        // Delegate all states to parent VRPowerControl
        default:
            return VRPowerControl::getPowerStateHandler();
    }
}

void ParsecPowerControl::addBoard1GpioStateProperties()
{
    gpioStateIface->register_property_r(
        "Board1CpuShutdownOk", int{-1},
        sdbusplus::vtable::property_::emits_change,
        [this](const auto&) { return board1CpuShutdownOkState; });

    // Add Board1 GPIO property setter for D-Bus
    gpioPropertySetters["Board1CpuShutdownOk"] = [](PowerControl* pc, int val) {
        pc->setBoard1CpuShutdownOkState(val);
    };
}

void ParsecPowerControl::handlePowerStateOn(Event event)
{
    // TODO: Move Parsec-specific powerStateOn() implementation here
}

void ParsecPowerControl::handlePowerStateOff(Event event)
{
    // TODO: Move Parsec-specific powerStateOff() implementation here
}

void ParsecPowerControl::handleWaitForPDBMainPowerOk(Event event)
{
    // TODO: Move Parsec-specific powerStateWaitForPDBMainPowerOk()
    // implementation here
}

void ParsecPowerControl::validateRequiredSignals()
{
    // Validate Parsec GB300 PDB signals (always required for Parsec platform)
    for (const auto& signalName : requiredSignals)
    {
        if (powerSignalMap.find(signalName) == powerSignalMap.end())
        {
            lg2::error(
                "Required Parsec GB300 PDB signal '{SIGNAL}' not found in config",
                "SIGNAL", signalName);
            throw std::runtime_error(
                "Parsec: Required GB300 PDB signal missing from config: " +
                signalName);
        }
    }

    // Configure GB300PDBMainPowerOk for input event monitoring
    // (gpio_keys_polled driver)
    auto gb300pdbMainPowerOkIt = powerSignalMap.find("GB300PDBMainPowerOk");
    if (gb300pdbMainPowerOkIt != powerSignalMap.end())
    {
        auto& configData = gb300pdbMainPowerOkIt->second;

        // Enable input event monitoring
        configData->useInputEvents = true;

        // Configure input event parameters
        InputEventConfig inputConfig;
        inputConfig.deviceName = "gpio_keys_gb300";
        inputConfig.signalName = "GB300_PDB_MAIN_PWR_OK_Mon";
        inputConfig.keyCode = 0x102; // BTN_2 key code
        inputConfig.stateTracker = &gb300pdbMainPowerOkState;

        configData->inputEventConfig = inputConfig;
    }

    // Call VRPowerControl to validate common VR signals
    VRPowerControl::validateRequiredSignals();

    lg2::info("Parsec signal validation complete");
}

void ParsecPowerControl::validateTimerConfigs()
{
    // TODO: Validate Parsec-specific PDB timer when implemented
    // Example: "ParsecPdbMainPowerOkWatchdogMs"
    for (const auto& timerName : platformRequiredTimeoutValues)
    {
        if (TimerMap.find(timerName) == TimerMap.end())
        {
            lg2::error(
                "Required Parsec timer config '{TIMER}' not found in config",
                "TIMER", timerName);
            throw std::runtime_error(
                "ParsecPowerControl: Required timer config missing: " +
                timerName);
        }
    }

    // Call VRPowerControl to validate common VR timers
    VRPowerControl::validateTimerConfigs();

    lg2::info("Parsec timer configuration validation complete");
}

void ParsecPowerControl::setDefaultValues()
{
    // Set Parsec GB300 PDB-specific default values for output signals
    lg2::info("Setting Parsec default values for output signals");

    // Find and validate all Parsec GB300 PDB-specific signals first
    auto gb300PdbMainPowerEnable =
        powerSignalMap.find("GB300PDBMainPowerEnable");
    if (gb300PdbMainPowerEnable == powerSignalMap.end())
    {
        lg2::error(
            "GB300PDBMainPowerEnable signal not found in powerSignalMap");
        return;
    }

    // All Parsec signals validated, now set the default states

    // GB300 PDB Main Power Enable
    // - ON: Asserted (GB300 PDB should be powered)
    // - OFF: DeAsserted (GB300 PDB should be unpowered)
    gb300PdbMainPowerEnable->second->defaultStateHostStateOn =
        DefaultState::Asserted;
    gb300PdbMainPowerEnable->second->defaultStateHostStateOff =
        DefaultState::DeAsserted;

    // Call parent to set common VR/HPM defaults
    VRPowerControl::setDefaultValues();

    lg2::info("Parsec default values set successfully");
}

} // namespace power_control
