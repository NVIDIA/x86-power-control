/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#include "c2_power_control.hpp"

namespace power_control
{

// Constructor: Assigns handlers and registers events for C2-specific GPIOs
C2PowerControl::C2PowerControl(
    boost::asio::io_context& ioContext,
    std::shared_ptr<sdbusplus::asio::connection> conn,
    const std::string& configFilePath, const std::string& node,
    PersistentState& appState) :
    VRPowerControl(ioContext, conn, configFilePath, node,
                   appState) // Call parent constructor (registers VR GPIOs)
{
    // powerSignalMap is now populated by base class
    // PowerControl::loadConfigValues() VR handlers already added to
    // gpioHandlerMap by VRPowerControl constructor Now add C2-specific handlers
    // to the map

    // call validateRequiredSignals() to validate all required signals
    validateRequiredSignals();

    // call validateTimerConfigs() to validate all required timers
    validateTimerConfigs();

    // call setDefaultValues() to set the default values for output signals
    setDefaultValues();

    // Add C2-specific GPIO handler to the map
    gpioHandlerMap["C2PDBPSUPowerOk"] = [this](bool state) {
        this->c2pdbPSUPowerOkHandler(state);
    };

    // Register all GPIO handlers (from base, VR, and C2)
    registerGPIOHandlers();

    // Note: C2PDB_Type no requestGPIOEvents call needed
}

// C2-SPECIFIC GPIO EVENT HANDLERS

void C2PowerControl::c2pdbPSUPowerOkHandler(bool state)
{
    auto it = powerSignalMap.find("C2PDBPSUPowerOk");
    if (it == powerSignalMap.end())
    {
        throw std::runtime_error(
            "C2PDBPSUPowerOk signal not found in powerSignalMap");
    }

    auto& config = *it->second;
    Event powerControlEvent = (state == config.polarity)
                                  ? Event::c2pdbPSUPowerOkAssert
                                  : Event::c2pdbPSUPowerOkDeAssert;
    this->sendPowerControlEvent(powerControlEvent, powerState);
}

std::function<void(Event)> C2PowerControl::getPowerStateHandler()
{
    switch (powerState)
    {
        // C2 modifies the handleWaitForHPMPowerGoodDeAssert and
        // handlePowerStateOff handlers to handle the C2 PDB 12V rails.
        case PowerState::waitForHPMPowerGoodDeAssert:
            return
                [this](Event e) { this->handleWaitForHPMPowerGoodDeAssert(e); };
        case PowerState::off:
            return [this](Event e) { this->handlePowerStateOff(e); };

        // TODO: Add any other C2-specific states here...

        // Delegate all states not overridden by C2 to parent VRPowerControl
        default:
            return VRPowerControl::getPowerStateHandler();
    }
}

void C2PowerControl::handlePowerStateOn(Event event)
{
    // TODO: Move C2-specific powerStateOn() implementation here
}

void C2PowerControl::handlePowerStateOff(Event event)
{
    // TODO: Move C2-specific powerStateOff() implementation here
}

void C2PowerControl::handleWaitForPDBMainPowerOk(Event event)
{
    // TODO: Move C2-specific powerStateWaitForPDBMainPowerOk() implementation
    // here
}

void C2PowerControl::handleWaitForHPMPowerGoodDeAssert(Event event)
{
    // TODO: Move C2-specific powerStateWaitForHPMPowerGoodDeAssert()
    // implementation here
}

void C2PowerControl::validateRequiredSignals()
{
    // Validate C2 PDB signals (always required for C2 platform)
    for (const auto& signalName : requiredSignals)
    {
        if (powerSignalMap.find(signalName) == powerSignalMap.end())
        {
            lg2::error("Required C2 PDB signal '{SIGNAL}' not found in config",
                       "SIGNAL", signalName);
            throw std::runtime_error(
                "C2: Required PDB signal missing from config: " + signalName);
        }
    }

    // Call VRPowerControl to validate common VR signals
    VRPowerControl::validateRequiredSignals();

    lg2::info("C2 signal validation complete");
}

void C2PowerControl::validateTimerConfigs()
{
    // TODO: Validate C2-specific PDB timer when implemented
    // Example: "C2PdbPSUPowerOkWatchdogMs"
    for (const auto& timerName : platformRequiredTimeoutValues)
    {
        if (TimerMap.find(timerName) == TimerMap.end())
        {
            lg2::error("Required C2 timer config '{TIMER}' not found in config",
                       "TIMER", timerName);
            throw std::runtime_error(
                "C2PowerControl: Required timer config missing: " + timerName);
        }
    }

    // Call VRPowerControl to validate common VR timers
    VRPowerControl::validateTimerConfigs();

    lg2::info("C2 timer configuration validation complete");
}

void C2PowerControl::setDefaultValues()
{
    // Set C2 PDB-specific default values for output signals
    lg2::info("Setting C2 default values for output signals");

    // Find and validate all C2 PDB-specific signals first
    auto c2PdbPsuPowerEnable = powerSignalMap.find("C2PDBPSUPowerEnable");
    if (c2PdbPsuPowerEnable == powerSignalMap.end())
    {
        throw std::runtime_error(
            "C2PDBPSUPowerEnable signal not found in powerSignalMap");
    }

    auto c2Pdb12vHpmEnable = powerSignalMap.find("C2PDB12VHPMEnable");
    if (c2Pdb12vHpmEnable == powerSignalMap.end())
    {
        throw std::runtime_error(
            "C2PDB12VHPMEnable signal not found in powerSignalMap");
    }

    auto c2Pdb12vGpu1Enable = powerSignalMap.find("C2PDB12VGPU1Enable");
    if (c2Pdb12vGpu1Enable == powerSignalMap.end())
    {
        throw std::runtime_error(
            "C2PDB12VGPU1Enable signal not found in powerSignalMap");
    }

    auto c2Pdb12vGpu2Enable = powerSignalMap.find("C2PDB12VGPU2Enable");
    if (c2Pdb12vGpu2Enable == powerSignalMap.end())
    {
        throw std::runtime_error(
            "C2PDB12VGPU2Enable signal not found in powerSignalMap");
    }

    // All C2 signals validated, now set the default states

    // C2 PDB PSU Power Enable
    // - ON: Asserted (C2 PDB PSU output should be enabled)
    // - OFF: DeAsserted (C2 PDB PSU should not be enabled)
    c2PdbPsuPowerEnable->second->defaultStateHostStateOn =
        DefaultState::Asserted;
    c2PdbPsuPowerEnable->second->defaultStateHostStateOff =
        DefaultState::DeAsserted;

    // C2 PDB 12V HPM Enable
    // - ON: Asserted (12V HPM rail should be powered)
    // - OFF: DeAsserted (12V HPM rail should be unpowered)
    c2Pdb12vHpmEnable->second->defaultStateHostStateOn = DefaultState::Asserted;
    c2Pdb12vHpmEnable->second->defaultStateHostStateOff =
        DefaultState::DeAsserted;

    // C2 PDB 12V GPU1 Enable
    // - ON: Asserted (12V GPU1 rail should be powered)
    // - OFF: DeAsserted (12V GPU1 rail should be unpowered)
    c2Pdb12vGpu1Enable->second->defaultStateHostStateOn =
        DefaultState::Asserted;
    c2Pdb12vGpu1Enable->second->defaultStateHostStateOff =
        DefaultState::DeAsserted;

    // C2 PDB 12V GPU2 Enable
    // - ON: Asserted (12V GPU2 rail should be powered)
    // - OFF: DeAsserted (12V GPU2 rail should be unpowered)
    c2Pdb12vGpu2Enable->second->defaultStateHostStateOn =
        DefaultState::Asserted;
    c2Pdb12vGpu2Enable->second->defaultStateHostStateOff =
        DefaultState::DeAsserted;

    // Call parent to set common VR/HPM defaults
    VRPowerControl::setDefaultValues();

    lg2::info("C2 default values set successfully");
}

} // namespace power_control
