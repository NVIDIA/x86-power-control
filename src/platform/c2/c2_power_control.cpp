/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#include "c2_power_control.hpp"

// External references to global variables from power_control.cpp
namespace power_control
{
extern PowerState powerState;
}

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
    // Lookup config for polarity (guaranteed to exist since handler was
    // registered)
    auto& config = *powerSignalMap["C2PDBPSUPowerOk"];

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

void C2PowerControl::setGPIOsForHostStateOn()
{
    // TODO: Set C2 PDB control GPIOs for host state "on"
    // - Assert C2PDB12VHPMEnable
    // - Assert C2PDB12VGPU1Enable
    // - Assert C2PDB12VGPU2Enable

    lg2::info("Setting C2 GPIOs for host state ON");

    // Call parent to set VR GPIOs
    VRPowerControl::setGPIOsForHostStateOn();
}

void C2PowerControl::setGPIOsForHostStateOff()
{
    // TODO: Set C2 PDB control GPIOs for host state "off"
    // - De-assert C2PDB12VHPMEnable
    // - De-assert C2PDB12VGPU1Enable
    // - De-assert C2PDB12VGPU2Enable

    lg2::info("Setting C2 GPIOs for host state OFF");

    // Call parent to set VR GPIOs
    VRPowerControl::setGPIOsForHostStateOff();
}

} // namespace power_control
