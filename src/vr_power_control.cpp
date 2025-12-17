/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#include "vr_power_control.hpp"

// External references to global variables/functions from power_control.cpp
namespace power_control
{
    extern std::map<std::string, ConfigData*> powerSignalMap;
    extern PowerState powerState;
    extern Context powerContext;
    extern ConfigType;
    
    extern bool requestGPIOEvents(
        const std::string& lineName,
        std::function<void(bool)> handler,
        gpiod::line& gpioLine,
        boost::asio::posix::stream_descriptor& gpioEventDescriptor
    );
}

namespace power_control
{

// Constructor: Adds common VR/HPM ConfigData entries to powerSignalMap
VRPowerControl::VRPowerControl(boost::asio::io_context& ioContext)
    : PowerControl(ioContext),                  // Call base constructor
      board0RunPowerPGEvent(ioContext),         // Initialize event descriptors with io
      board1RunPowerPGEvent(ioContext),
      board0CpuShutdownOkEvent(ioContext),
      board1CpuShutdownOkEvent(ioContext),
      cpuResetIndicatorEvent(ioContext)
{
    // Add common VR/HPM signals to powerSignalMap
    powerSignalMap["Board0RunPowerPG"] = &board0RunPowerPGConfig;
    powerSignalMap["Board0RunPowerEnable"] = &board0RunPowerEnableConfig;
    powerSignalMap["Board0PreSystemReset"] = &board0PreSystemResetConfig;
    powerSignalMap["Board0CpuShutdownForce"] = &board0CpuShutdownForceConfig;
    powerSignalMap["Board0CpuShutdownRequest"] = &board0CpuShutdownRequestConfig;
    powerSignalMap["Board0CpuShutdownOk"] = &board0CpuShutdownOkConfig;
    
    powerSignalMap["Board1RunPowerPG"] = &board1RunPowerPGConfig;
    powerSignalMap["Board1RunPowerEnable"] = &board1RunPowerEnableConfig;
    powerSignalMap["Board1PreSystemReset"] = &board1PreSystemResetConfig;
    powerSignalMap["Board1CpuShutdownForce"] = &board1CpuShutdownForceConfig;
    powerSignalMap["Board1CpuShutdownOk"] = &board1CpuShutdownOkConfig;
    
    powerSignalMap["CpuResetIndicator"] = &cpuResetIndicatorConfig;
    powerSignalMap["USBPowerEnable"] = &usbPowerEnableConfig;
}

// Initialize common VR/HPM GPIO events (PHASE 2: After loadConfigValues())
void VRPowerControl::initializeGPIO()
{
    // Register common VR/HPM GPIO event handlers
    
    // Board 0 Run Power Good
    if (board0RunPowerPGConfig.type == ConfigType::GPIO)
    {
        if (!requestGPIOEvents(board0RunPowerPGConfig.lineName,
                              [this](bool state) { this->board0RunPowerPGHandler(state); },
                              board0RunPowerPGLine,
                              board0RunPowerPGEvent))
        {
            throw std::runtime_error("VR: Failed to register Board 0 Run Power Good GPIO events");
        }
    }
    
    // Board 1 Run Power Good
    if (powerContext.presence.board1 && board1RunPowerPGConfig.type == ConfigType::GPIO)
    {
        if (!requestGPIOEvents(board1RunPowerPGConfig.lineName,
                              [this](bool state) { this->board1RunPowerPGHandler(state); },
                              board1RunPowerPGLine,
                              board1RunPowerPGEvent))
        {
            throw std::runtime_error("VR: Failed to register Board 1 Run Power Good GPIO events");
        }
    }
    
    // Board 0 CPU Shutdown OK
    if (board0CpuShutdownOkConfig.type == ConfigType::GPIO)
    {
        if (!requestGPIOEvents(board0CpuShutdownOkConfig.lineName,
                              [this](bool state) { this->board0CpuShutdownOkHandler(state); },
                              board0CpuShutdownOkLine,
                              board0CpuShutdownOkEvent))
        {
            throw std::runtime_error("VR: Failed to register Board 0 CPU Shutdown OK GPIO events");
        }
    }
    
    // Board 1 CPU Shutdown OK
    if (powerContext.presence.board1 && board1CpuShutdownOkConfig.type == ConfigType::GPIO)
    {
        if (!requestGPIOEvents(board1CpuShutdownOkConfig.lineName,
                              [this](bool state) { this->board1CpuShutdownOkHandler(state); },
                              board1CpuShutdownOkLine,
                              board1CpuShutdownOkEvent))
        {
            throw std::runtime_error("VR: Failed to register Board 1 CPU Shutdown OK GPIO events");
        }
    }
    
    // CPU Reset Indicator
    if (cpuResetIndicatorConfig.type == ConfigType::GPIO)
    {
        if (!requestGPIOEvents(cpuResetIndicatorConfig.lineName,
                              [this](bool state) { this->cpuResetIndicatorHandler(state); },
                              cpuResetIndicatorLine,
                              cpuResetIndicatorEvent))
        {
            throw std::runtime_error("VR: Failed to register CPU Reset Indicator GPIO events");
        }
    }
}

// =============================================================================
// GPIO EVENT HANDLERS (Member functions)
// =============================================================================

void VRPowerControl::board0RunPowerPGHandler(bool state)
{
    Event powerControlEvent = (state == board0RunPowerPGConfig.polarity)
                                  ? Event::board0RunPowerPGAssert
                                  : Event::board0RunPowerPGDeAssert;
    this->sendPowerControlEvent(powerControlEvent, powerState);
}

void VRPowerControl::board1RunPowerPGHandler(bool state)
{
    Event powerControlEvent = (state == board1RunPowerPGConfig.polarity)
                                  ? Event::board1RunPowerPGAssert
                                  : Event::board1RunPowerPGDeAssert;
    this->sendPowerControlEvent(powerControlEvent, powerState);
}

void VRPowerControl::board0CpuShutdownOkHandler(bool state)
{
    Event powerControlEvent = (state == board0CpuShutdownOkConfig.polarity)
                                  ? Event::board0CpuShutdownOkAssert
                                  : Event::board0CpuShutdownOkDeAssert;
    this->sendPowerControlEvent(powerControlEvent, powerState);
}

void VRPowerControl::board1CpuShutdownOkHandler(bool state)
{
    Event powerControlEvent = (state == board1CpuShutdownOkConfig.polarity)
                                  ? Event::board1CpuShutdownOkAssert
                                  : Event::board1CpuShutdownOkDeAssert;
    this->sendPowerControlEvent(powerControlEvent, powerState);
}

void VRPowerControl::cpuResetIndicatorHandler(bool state)
{
    Event powerControlEvent = (state == cpuResetIndicatorConfig.polarity)
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

} // namespace power_control

