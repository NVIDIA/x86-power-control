#include "power_control_base.hpp"

namespace power_control
{

std::function<void(Event)> PowerControl::getPowerStateHandler(PowerState state)
{
    // Map upstream PowerState values to their handler functions
    switch (state)
    {
        case PowerState::on:
            return [this](Event e) { this->handlePowerStateOn(e); };
        
        case PowerState::waitForPSPowerOK:
            return [this](Event e) { this->handleWaitForPSPowerOK(e); };
        
        case PowerState::waitForSIOPowerGood:
            return [this](Event e) { this->handleWaitForSIOPowerGood(e); };
        
        case PowerState::off:
            return [this](Event e) { this->handlePowerStateOff(e); };
        
        case PowerState::transitionToOff:
            return [this](Event e) { this->handleTransitionToOff(e); };
        
        case PowerState::gracefulTransitionToOff:
            return [this](Event e) { this->handleGracefulTransitionToOff(e); };
        
        case PowerState::cycleOff:
            return [this](Event e) { this->handleCycleOff(e); };
        
        case PowerState::transitionToCycleOff:
            return [this](Event e) { this->handleTransitionToCycleOff(e); };
        
        case PowerState::gracefulTransitionToCycleOff:
            return [this](Event e) { this->handleGracefulTransitionToCycleOff(e); };
        
        case PowerState::checkForWarmReset:
            return [this](Event e) { this->handleCheckForWarmReset(e); };
        
        // Unknown state - not an upstream state
        // check for nul pointer return
        default:
            return nullptr;
    }
}

void PowerControl::sendPowerControlEvent(Event event, PowerState currentState)
{
    // Use the virtual getPowerStateHandler to get the correct handler
    std::function<void(Event)> handler = this->getPowerStateHandler(currentState);
    
    if (handler == nullptr)
    {
        lg2::error("Failed to find handler for power state: {STATE}", "STATE",
                   static_cast<int>(currentState));
        return;
    }
    
    // Execute the handler (will use virtual dispatch)
    handler(event);
}

void PowerControl::handlePowerStateOn(Event event)
{
    // TODO: Move upstream powerStateOn() implementation here
    // 
    // PREVIOUS IMPLEMENTATION (from powerStateOn in power_control.cpp):
    // - logEvent(__FUNCTION__, event);
    // - switch (event):
    //     case Event::psPowerOKDeAssert:
    //         - setPowerState(PowerState::off);
    //         - beep(beepPowerFail);
    //     case Event::sioS5Assert:
    //         - setPowerState(PowerState::transitionToOff);
    //         - addRestartCause(RestartCause::softReset);
    //     case Event::pltRstAssert / Event::postCompleteDeAssert:
    //         - setPowerState(PowerState::checkForWarmReset);
    //         - addRestartCause(RestartCause::softReset);
    //     case Event::powerButtonPressed:
    //         - graceful power off sequence
    //     case Event::gracefulPowerOffRequest:
    //         - setPowerState(PowerState::gracefulTransitionToOff);
    //     case Event::powerCycleRequest:
    //         - setPowerState(PowerState::gracefulTransitionToCycleOff);
    //     case Event::resetRequest:
    //         - reset sequence
    //     case Event::powerOffRequest:
    //         - setPowerState(PowerState::transitionToOff);
}

void PowerControl::handlePowerStateOff(Event event)
{
    // TODO: Move upstream powerStateOff() implementation here
    //
    // PREVIOUS IMPLEMENTATION (from powerStateOff in power_control.cpp - UPSTREAM ONLY):
    // - logEvent(__FUNCTION__, event);
    // - switch (event):
    //     case Event::psPowerOKAssert:
    //         - if (sioEnabled) → setPowerState(PowerState::waitForSIOPowerGood)
    //         - else → setPowerState(PowerState::on)
    //     case Event::sioS5DeAssert:
    //         - psPowerOKWatchdogTimerStart();
    //         - setPowerState(PowerState::waitForPSPowerOK);
    //     case Event::sioPowerGoodAssert:
    //         - setPowerState(PowerState::on);
    //     case Event::powerButtonPressed:
    //         - psPowerOKWatchdogTimerStart();
    //         - setPowerState(PowerState::waitForPSPowerOK);
}

void PowerControl::handleWaitForPSPowerOK(Event event)
{
    // TODO: Move upstream powerStateWaitForPSPowerOK() implementation here
    //
    // PREVIOUS IMPLEMENTATION:
    // - logEvent(__FUNCTION__, event);
    // - switch (event):
    //     case Event::psPowerOKAssert:
    //         - psPowerOKWatchdogTimer.cancel();
    //         - if (sioEnabled) → sioPowerGoodWatchdogTimerStart(), setPowerState(PowerState::waitForSIOPowerGood)
    //         - else → setPowerState(PowerState::on)
    //     case Event::psPowerOKWatchdogTimerExpired:
    //         - setPowerState(PowerState::off);
    //         - lg2::error("PS_PWROK signal did not assert within timeout");
}

void PowerControl::handleWaitForSIOPowerGood(Event event)
{
    // TODO: Move upstream powerStateWaitForSIOPowerGood() implementation here
    //
    // PREVIOUS IMPLEMENTATION:
    // - logEvent(__FUNCTION__, event);
    // - switch (event):
    //     case Event::sioPowerGoodAssert:
    //         - sioPowerGoodWatchdogTimer.cancel();
    //         - setPowerState(PowerState::on);
    //     case Event::sioPowerGoodWatchdogTimerExpired:
    //         - setPowerState(PowerState::off);
    //         - lg2::error("SIO Power Good signal did not assert within timeout");
}

void PowerControl::handleTransitionToOff(Event event)
{
    // TODO: Move upstream powerStateTransitionToOff() implementation here
    //
    // PREVIOUS IMPLEMENTATION:
    // - Forcefully de-assert all power control signals
    // - Transition to off state
}

void PowerControl::handleGracefulTransitionToOff(Event event)
{
    // TODO: Move upstream powerStateGracefulTransitionToOff() implementation here
    //
    // PREVIOUS IMPLEMENTATION:
    // - Assert graceful shutdown request
    // - Start graceful power off timer
    // - If host shuts down gracefully (sioS5Assert) → transition to off
    // - If timer expires → forceful shutdown → transition to off
}

void PowerControl::handleCycleOff(Event event)
{
    // TODO: Move upstream powerStateCycleOff() implementation here
    //
    // PREVIOUS IMPLEMENTATION:
    // - Wait in powered off state for power cycle timer
    // - When timer expires → transition to power on (waitForPSPowerOK)
}

void PowerControl::handleTransitionToCycleOff(Event event)
{
    // TODO: Move upstream powerStateTransitionToCycleOff() implementation here
    //
    // PREVIOUS IMPLEMENTATION:
    // - Forcefully de-assert all power control signals
    // - Start power cycle timer
    // - Transition to cycleOff state
}

void PowerControl::handleGracefulTransitionToCycleOff(Event event)
{
    // TODO: Move upstream powerStateGracefulTransitionToCycleOff() implementation here
    //
    // PREVIOUS IMPLEMENTATION:
    // - Assert graceful shutdown request
    // - Start graceful power off timer
    // - If host shuts down gracefully → transition to cycleOff
    // - If timer expires → forceful shutdown → transition to cycleOff
}

void PowerControl::handleCheckForWarmReset(Event event)
{
    // TODO: Move upstream powerStateCheckForWarmReset() implementation here
    //
    // PREVIOUS IMPLEMENTATION:
    // - Monitor PLT_RST or POST Complete signals
    // - If PLT_RST de-asserts (warm reset) → transition back to on
    // - If power off detected → transition to transitionToOff
}

} // namespace power_control

