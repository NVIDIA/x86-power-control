/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#include "vr_power_control.hpp"

// External references to global variables from power_control.cpp
namespace power_control
{
    extern PowerState powerState;
    extern Context powerContext;
}

namespace power_control
{

// Constructor: Assigns handlers and registers events for common VR/HPM GPIOs
VRPowerControl::VRPowerControl(boost::asio::io_context& ioContext,
                               std::shared_ptr<sdbusplus::asio::connection> conn,
                               const std::string& node)
    : PowerControl(ioContext, conn, node),  // Call base constructor (populates powerSignalMap from JSON)
      pdbMainPowerOkWatchdogTimer(ioContext),
      hpmPowerGoodWatchdogTimer(ioContext),
      cpuResetWatchdogTimer(ioContext),
      cpuShutdownOkWatchdogTimer(ioContext)
{
    // powerSignalMap is now populated by PowerControl::loadConfigValues()
    // Assign handlers and register events for common VR/HPM signals

    // call validateRequiredSignals() to validate all required signals
    validateRequiredSignals();

    auto it = powerSignalMap.find("Board0RunPowerPG");
    it->second->gpioHandler = [this](bool state) { this->board0RunPowerPGHandler(state); };

    if(!requestGPIOEvents(*it->second))
    {
        lg2::error("Failed to register GPIO events for Board 0 Run Power Good");
        throw std::runtime_error("VR: Failed to register Board 0 Run Power Good GPIO events");
    }

    it = powerSignalMap.find("Board0CpuShutdownOk");
    it->second->gpioHandler = [this](bool state) { this->board0CpuShutdownOkHandler(state); };

    if(!requestGPIOEvents(*it->second))
    {
        lg2::error("Failed to register GPIO events for Board 0 CPU Shutdown OK");
        throw std::runtime_error("VR: Failed to register Board 0 CPU Shutdown OK GPIO events");
    }

    it = powerSignalMap.find("CpuResetIndicator");
    it->second->gpioHandler = [this](bool state) { this->cpuResetIndicatorHandler(state); };

    if(!requestGPIOEvents(*it->second))
    {
        lg2::error("Failed to register GPIO events for CpuResetIndicator");
        throw std::runtime_error("VR: Failed to register CPU Reset Indicator GPIO events");
    }

    if (auto it = powerSignalMap.find("Board1RunPowerPG"); it != powerSignalMap.end())
    {
        it->second->gpioHandler = [this](bool state) { this->board1RunPowerPGHandler(state); };

        if(!requestGPIOEvents(*it->second))
        {
            lg2::error("Failed to register GPIO events for Board 1 Run Power Good");
            throw std::runtime_error("VR: Failed to register Board 1 Run Power Good GPIO events");
        }
    }

    if (auto it = powerSignalMap.find("Board1CpuShutdownOk"); it != powerSignalMap.end())
    {
        it->second->gpioHandler = [this](bool state) { this->board1CpuShutdownOkHandler(state); };

        if(!requestGPIOEvents(*it->second))
        {
            lg2::error("Failed to register GPIO events for Board 1 CPU Shutdown OK");
            throw std::runtime_error("VR: Failed to register Board 1 CPU Shutdown OK GPIO events");
        }
    }

    
    // DO NOT NEED TO MONITOR BOARD 1 RUN POWER GOOD - ONLY MONITOR BOARD 0 RUN POWER GOOD
    // // Board 1 Run Power Good (optional - depends on presence)
    // if (auto it = powerSignalMap.find("Board1RunPowerPG"); it != powerSignalMap.end())
    // {
    //     if (powerContext.presence.board1)
    //     {
    //         it->second->gpioHandler = [this](bool state) { this->board1RunPowerPGHandler(state); };
    //         if (!requestGPIOEvents(*it->second))
    //         {
    //             lg2::error("Failed to register GPIO events for Board1RunPowerPG");
    //             throw std::runtime_error("VR: Failed to register Board 1 Run Power Good GPIO events");
    //         }
    //     }
    // }
}

// =============================================================================
// GPIO EVENT HANDLERS (Member functions)
// =============================================================================

void VRPowerControl::board0RunPowerPGHandler(bool state)
{
    // Lookup config for polarity (guaranteed to exist since handler was registered)
    auto& config = *powerSignalMap["Board0RunPowerPG"];
    
    Event powerControlEvent = (state == config.polarity)
                                  ? Event::board0RunPowerPGAssert
                                  : Event::board0RunPowerPGDeAssert;
    this->sendPowerControlEvent(powerControlEvent, powerState);
}

void VRPowerControl::board1RunPowerPGHandler(bool state)
{
    // Lookup config for polarity (guaranteed to exist since handler was registered)
    auto& config = *powerSignalMap["Board1RunPowerPG"];
    
    Event powerControlEvent = (state == config.polarity)
                                  ? Event::board1RunPowerPGAssert
                                  : Event::board1RunPowerPGDeAssert;
    this->sendPowerControlEvent(powerControlEvent, powerState);
}

void VRPowerControl::board0CpuShutdownOkHandler(bool state)
{
    // Lookup config for polarity (guaranteed to exist since handler was registered)
    auto& config = *powerSignalMap["Board0CpuShutdownOk"];
    
    Event powerControlEvent = (state == config.polarity)
                                  ? Event::board0CpuShutdownOkAssert
                                  : Event::board0CpuShutdownOkDeAssert;
    this->sendPowerControlEvent(powerControlEvent, powerState);
}

void VRPowerControl::board1CpuShutdownOkHandler(bool state)
{
    // Lookup config for polarity (guaranteed to exist since handler was registered)
    auto& config = *powerSignalMap["Board1CpuShutdownOk"];
    
    Event powerControlEvent = (state == config.polarity)
                                  ? Event::board1CpuShutdownOkAssert
                                  : Event::board1CpuShutdownOkDeAssert;
    this->sendPowerControlEvent(powerControlEvent, powerState);
}

void VRPowerControl::cpuResetIndicatorHandler(bool state)
{
    // Lookup config for polarity (guaranteed to exist since handler was registered)
    auto& config = *powerSignalMap["CpuResetIndicator"];
    
    Event powerControlEvent = (state == config.polarity)
                                  ? Event::cpuResetIndicatorAssert
                                  : Event::cpuResetIndicatorDeAssert;
    this->sendPowerControlEvent(powerControlEvent, powerState);
}

std::function<void(Event)> VRPowerControl::getPowerStateHandler(PowerState state)
{
    // Map VR-specific PowerState values to their handler functions
    switch (state)
    {
        // VR-specific states
        case PowerState::waitForPDBMainPowerOk:
            return [this](Event e) { this->handleWaitForPDBMainPowerOk(e); };
        
        case PowerState::waitForPDBMainPowerOff:
            return [this](Event e) { this->handleWaitForPDBMainPowerOff(e); };
        
        case PowerState::waitForHPMPowerGoodAssert:
            return [this](Event e) { this->handleWaitForHPMPowerGoodAssert(e); };
        
        case PowerState::waitForHPMPowerGoodDeAssert:
            return [this](Event e) { this->handleWaitForHPMPowerGoodDeAssert(e); };
        
        case PowerState::waitForCPUResetAssert:
            return [this](Event e) { this->handleWaitForCPUResetAssert(e); };
        
        case PowerState::waitForCPUResetDeAssert:
            return [this](Event e) { this->handleWaitForCPUResetDeAssert(e); };
        
        case PowerState::waitForCPUShutdownOk:
            return [this](Event e) { this->handleWaitForCPUShutdownOk(e); };
        
        // Delegate upstream states to base class
        default:
            return PowerControl::getPowerStateHandler(state);
    }
}

void VRPowerControl::handlePowerStateOn(Event event)
{
    // TODO: Move NVL144-specific powerStateOn() implementation here

}

void VRPowerControl::handlePowerStateOff(Event event)
{
    // TODO: Move NVL144-specific powerStateOff() implementation here

}

void VRPowerControl::handleWaitForPDBMainPowerOk(Event event)
{
    // TODO: Move NVL144-specific powerStateWaitForPDBMainPowerOk() implementation here

}

void VRPowerControl::handleWaitForPDBMainPowerOff(Event event)
{
    // TODO: Move powerStateWaitForPDBMainPowerOff() implementation here

}

void VRPowerControl::handleWaitForHPMPowerGoodAssert(Event event)
{
    // TODO: Move powerStateWaitForHPMPowerGoodAssert() implementation here

}

void VRPowerControl::handleWaitForHPMPowerGoodDeAssert(Event event)
{
    // TODO: Move powerStateWaitForHPMPowerGoodDeAssert() implementation here

}

void VRPowerControl::handleWaitForCPUResetAssert(Event event)
{
    // TODO: Move powerStateWaitForCPUResetAssert() implementation here
 
}

void VRPowerControl::handleWaitForCPUResetDeAssert(Event event)
{
    // TODO: Move powerStateWaitForCPUResetDeAssert() implementation here
}

void VRPowerControl::handleWaitForCPUShutdownOk(Event event)
{
    // TODO: Move powerStateWaitForCPUShutdownOk() implementation here

}

std::string_view VRPowerControl::getHostState(const PowerState state)
{
    // VR-specific implementation - maps VR PowerState extensions to D-Bus host state
    switch (state)
    {
        case PowerState::waitForPDBMainPowerOk:
        case PowerState::waitForHPMPowerGoodAssert:
        case PowerState::waitForCPUResetDeAssert:
            return "xyz.openbmc_project.State.Host.HostState.TransitioningToRunning";
            break;
        case PowerState::waitForCPUResetAssert:
        case PowerState::waitForCPUShutdownOk:
            return "xyz.openbmc_project.State.Host.HostState.TransitioningToOff";
            break;
        case PowerState::waitForHPMPowerGoodDeAssert:
            return "xyz.openbmc_project.State.Host.HostState.Off";
            break;
        case PowerState::waitForPDBMainPowerOff:
            if (powerContext.action == PowerAction::POWER_ON)
            {
                return "xyz.openbmc_project.State.Host.HostState.TransitioningToRunning";
            }
            else if (powerContext.action == PowerAction::FORCE_OFF || 
                     powerContext.action == PowerAction::GRACE_OFF ||
                     powerContext.action == PowerAction::HOST_INITIATED_SHUTDOWN)
            {
                return "xyz.openbmc_project.State.Host.HostState.Off";
            }
            break;
        default:
            break;
    }
    
    // Fall through to base class for upstream states
    return PowerControl::getHostState(state);
}

std::string_view VRPowerControl::getChassisState(const PowerState state)
{
    // VR-specific implementation - maps VR PowerState extensions to D-Bus chassis state
    switch (state)
    {
        case PowerState::waitForPDBMainPowerOk:
        case PowerState::waitForHPMPowerGoodAssert:
            return "xyz.openbmc_project.State.Chassis.PowerState.TransitioningToOn";
            break;
        case PowerState::waitForHPMPowerGoodDeAssert:
        case PowerState::waitForCPUResetAssert:
        case PowerState::waitForCPUShutdownOk:
            return "xyz.openbmc_project.State.Chassis.PowerState.TransitioningToOff";
            break;
        case PowerState::waitForCPUResetDeAssert:
            return "xyz.openbmc_project.State.Chassis.PowerState.On";
            break;
        case PowerState::waitForPDBMainPowerOff:
            if (powerContext.action == PowerAction::POWER_ON)
            {
                return "xyz.openbmc_project.State.Chassis.PowerState.TransitioningToOn";
            }
            else if (powerContext.action == PowerAction::FORCE_OFF || 
                     powerContext.action == PowerAction::GRACE_OFF ||
                     powerContext.action == PowerAction::HOST_INITIATED_SHUTDOWN)
            {
                return "xyz.openbmc_project.State.Chassis.PowerState.TransitioningToOff";
            }
            break;
        default:
            break;
    }
    
    // Fall through to base class for upstream states
    return PowerControl::getChassisState(state);
}

std::string VRPowerControl::getPowerStateName(const PowerState state)
{
    // VR-specific state name mappings
    // TODO: Confirm  VR-specific logic
    
    switch (state)
    {
        case PowerState::waitForPDBMainPowerOk:
            return "Wait for PDB Main Power OK";
            break;
        case PowerState::waitForPDBMainPowerOff:
            return "Wait for PDB Main Power Off";
            break;
        case PowerState::waitForHPMPowerGoodAssert:
            return "Wait for HPM Power Good Assert";
            break;
        case PowerState::waitForHPMPowerGoodDeAssert:
            return "Wait for HPM Power Good De-Assert";
            break;
        case PowerState::waitForCPUResetAssert:
            return "Wait for CPU Reset Assert";
            break;
        case PowerState::waitForCPUResetDeAssert:
            return "Wait for CPU Reset De-Assert";
            break;
        case PowerState::waitForCPUShutdownOk:
            return "Wait for CPU Shutdown OK";
            break;
        default:
            // Fall through to base class for upstream states
            break;
    }
    
    // Call base class for upstream states
    return PowerControl::getPowerStateName(state);
}

void VRPowerControl::validateRequiredSignals()
{
    // Validate Board 0 signals (always required)
    for (const auto& signalName : requiredBoard0Signals)
    {
        if (powerSignalMap.find(signalName) == powerSignalMap.end())
        {
            lg2::error("Required Board 0 signal '{SIGNAL}' not found in config", 
                      "SIGNAL", signalName);
            throw std::runtime_error("VR: Required Board 0 signal missing from config: " + signalName);
        }
    }
    
    // Conditionally validate Board 1 signals (only if Board 1 is present)
    if (boardPresence.board1Present)
    {
        for (const auto& signalName : requiredBoard1Signals)
        {
            if (powerSignalMap.find(signalName) == powerSignalMap.end())
            {
                lg2::error("Required Board 1 signal '{SIGNAL}' not found in config", 
                          "SIGNAL", signalName);
                throw std::runtime_error("VR: Required Board 1 signal missing from config: " + signalName);
            }
        }
    }
    
    lg2::info("VR signal validation complete - all required signals present");
}

void VRPowerControl::setGPIOsForHostStateOn()
{
    // TODO: Set VR control GPIOs for host state "on"
    // - Assert Board0RunPowerEnable
    // - De-assert Board0PreSystemReset
    // - Assert Board0CpuShutdownForce (or Request?)
    // If Board 1 present:
    //   - Assert Board1RunPowerEnable
    //   - De-assert Board1PreSystemReset
    
    lg2::info("Setting VR GPIOs for host state ON");
}

void VRPowerControl::setGPIOsForHostStateOff()
{
    // TODO: Set VR control GPIOs for host state "off"
    // - De-assert Board0RunPowerEnable
    // - Assert Board0PreSystemReset
    // - De-assert Board0CpuShutdownForce (or Request?)
    // If Board 1 present:
    //   - De-assert Board1RunPowerEnable
    //   - Assert Board1PreSystemReset
    
    lg2::info("Setting VR GPIOs for host state OFF");
}

} // namespace power_control

