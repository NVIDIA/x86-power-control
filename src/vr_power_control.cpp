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
VRPowerControl::VRPowerControl(boost::asio::io_context& ioContext)
    : PowerControl(ioContext)  // Call base constructor (populates powerSignalMap from JSON)
{
    // powerSignalMap is now populated by PowerControl::loadConfigValues()
    // Assign handlers and register events for common VR/HPM signals
    
    // Board 0 Run Power Good
    if (auto it = powerSignalMap.find("Board0RunPowerPG"); it != powerSignalMap.end())
    {
        it->second->gpioHandler = [this](bool state) { this->board0RunPowerPGHandler(state); };
        if (!requestGPIOEvents(*it->second))
        {
            lg2::error("Failed to register GPIO events for Board 0 Run Power Good");
            throw std::runtime_error("VR: Failed to register Board 0 Run Power Good GPIO events");
        }
    }
    else
    {
        lg2::error("Board0RunPowerPG not found in config");
        throw std::runtime_error("VR: Required signal Board0RunPowerPG missing from config");
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
    
    // Board 0 CPU Shutdown OK
    if (auto it = powerSignalMap.find("Board0CpuShutdownOk"); it != powerSignalMap.end())
    {
        it->second->gpioHandler = [this](bool state) { this->board0CpuShutdownOkHandler(state); };
        if (!requestGPIOEvents(*it->second))
        {
            lg2::error("Failed to register GPIO events for Board0CpuShutdownOk");
            throw std::runtime_error("VR: Failed to register Board 0 CPU Shutdown OK GPIO events");
        }
    }
    
    // Board 1 CPU Shutdown OK (optional)
    if (auto it = powerSignalMap.find("Board1CpuShutdownOk"); it != powerSignalMap.end())
    {
        if (powerContext.presence.board1)
        {
            it->second->gpioHandler = [this](bool state) { this->board1CpuShutdownOkHandler(state); };
            if (!requestGPIOEvents(*it->second))
            {
                lg2::error("Failed to register GPIO events for Board1CpuShutdownOk");
                throw std::runtime_error("VR: Failed to register Board 1 CPU Shutdown OK GPIO events");
            }
        }
    }
    
    // CPU Reset Indicator
    if (auto it = powerSignalMap.find("CpuResetIndicator"); it != powerSignalMap.end())
    {
        it->second->gpioHandler = [this](bool state) { this->cpuResetIndicatorHandler(state); };
        if (!requestGPIOEvents(*it->second))
        {
            lg2::error("Failed to register GPIO events for CpuResetIndicator");
            throw std::runtime_error("VR: Failed to register CPU Reset Indicator GPIO events");
        }
    }
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

} // namespace power_control

