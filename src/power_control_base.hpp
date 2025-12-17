#pragma once

#include <functional>

// Forward declarations (these enums are defined in power_control.cpp)
namespace power_control
{
    enum class PowerState;
    enum class Event;
}

namespace power_control
{

/**
 * @brief Base Power Control class - Upstream functionality
 * 
 * This class contains state handlers for upstream (openbmc x86-power-control)
 * functionality. It handles:
 * - Power supply and SIO power good monitoring
 * - Graceful and forceful power off sequences
 * - Power cycling
 * - Warm reset detection
 * 
 * This class only knows about/uses upstream PowerState enums (on, off, waitForPSPowerOK, etc.)
 */
class PowerControl
{
public:
    PowerControl() = default;
    virtual ~PowerControl() = default;

    /**
     * @brief Get the handler function for a given power state
     * 
     * This virtual function maps PowerState enum values to their corresponding
     * handler functions. Derived classes can override this to add handlers for
     * additional states.
     * 
     * @param state The power state to get a handler for
     * @return Function that handles events in the given state, or nullptr if unknown
     */
    virtual std::function<void(Event)> getPowerStateHandler(PowerState state);

    /**
     * @brief Send an event to the appropriate power state handler
     * 
     * This method looks up the handler for the current power state and dispatches
     * the event to it. If no handler is found, an error is logged.
     * 
     * @param event The event to dispatch
     * @param currentState The current power state (used to look up the handler)
     */
    void sendPowerControlEvent(Event event, PowerState currentState);

protected:
    // =============================================================================
    // UPSTREAM STATE HANDLERS
    // These handle upstream power states and should match upstream behavior
    // =============================================================================

    /**
     * @brief Handler for PowerState::on
     * 
     * PREVIOUS IMPLEMENTATION (Upstream):
     * - Handled DC power loss (psPowerOKDeAssert) → transition to off
     * - Handled S5 assertion (sioS5Assert) → transition to graceful power off
     * - Handled PLT_RST assertion or POST Complete de-assertion → check for warm reset
     * - Handled power button press → graceful power off
     * - Handled graceful power off request → graceful power off
     * - Handled power cycle request → graceful power cycle
     * - Handled forceful power off request → forceful power off
     * - Handled reset request → reset the host
     */
    virtual void handlePowerStateOn(Event event);

    /**
     * @brief Handler for PowerState::off
     * 
     * PREVIOUS IMPLEMENTATION (Upstream):
     * - Handled PS Power OK assertion → transition to waitForSIOPowerGood or on
     * - Handled SIO S5 de-assertion → start PS power OK watchdog, transition to waitForPSPowerOK
     * - Handled SIO Power Good assertion → transition to on
     * - Handled power button press → start PS power OK watchdog, transition to waitForPSPowerOK
     */
    virtual void handlePowerStateOff(Event event);

    /**
     * @brief Handler for PowerState::waitForPSPowerOK
     * 
     * PREVIOUS IMPLEMENTATION (Upstream):
     * - Waited for PS Power OK assertion
     * - Handled watchdog timeout (power supply failed to come up)
     * - Transitioned to waitForSIOPowerGood or on depending on SIO configuration
     */
    virtual void handleWaitForPSPowerOK(Event event);

    /**
     * @brief Handler for PowerState::waitForSIOPowerGood
     * 
     * PREVIOUS IMPLEMENTATION (Upstream):
     * - Waited for SIO Power Good assertion
     * - Handled watchdog timeout (SIO failed to come up)
     * - Transitioned to on when SIO Power Good asserted
     */
    virtual void handleWaitForSIOPowerGood(Event event);

    /**
     * @brief Handler for PowerState::transitionToOff
     * 
     * PREVIOUS IMPLEMENTATION (Upstream):
     * - Forceful power off sequence
     * - De-asserted power control signals
     * - Transitioned to off state
     */
    virtual void handleTransitionToOff(Event event);

    /**
     * @brief Handler for PowerState::gracefulTransitionToOff
     * 
     * PREVIOUS IMPLEMENTATION (Upstream):
     * - Graceful shutdown sequence
     * - Asserted graceful shutdown request signals
     * - Started graceful shutdown timer
     * - Waited for host to shut down gracefully
     * - If timer expired, forced shutdown
     */
    virtual void handleGracefulTransitionToOff(Event event);

    /**
     * @brief Handler for PowerState::cycleOff
     * 
     * PREVIOUS IMPLEMENTATION (Upstream):
     * - Power cycle off state
     * - Waited for power cycle timer to expire
     * - Transitioned back to power on
     */
    virtual void handleCycleOff(Event event);

    /**
     * @brief Handler for PowerState::transitionToCycleOff
     * 
     * PREVIOUS IMPLEMENTATION (Upstream):
     * - Transition to power cycle off
     * - De-asserted power control signals
     * - Started power cycle timer
     * - Transitioned to cycleOff state
     */
    virtual void handleTransitionToCycleOff(Event event);

    /**
     * @brief Handler for PowerState::gracefulTransitionToCycleOff
     * 
     * PREVIOUS IMPLEMENTATION (Upstream):
     * - Graceful transition to power cycle
     * - Asserted graceful shutdown request signals
     * - Started graceful shutdown timer
     * - If gracefully shut down, transitioned to cycleOff
     * - If timer expired, forced shutdown and transitioned to cycleOff
     */
    virtual void handleGracefulTransitionToCycleOff(Event event);

    /**
     * @brief Handler for PowerState::checkForWarmReset
     * 
     * PREVIOUS IMPLEMENTATION (Upstream):
     * - Checked if the reset was a warm reset (host initiated) or cold reset
     * - Monitored PLT_RST or POST Complete signals
     * - If warm reset detected, allowed it to proceed
     * - If cold reset (power off), transitioned to off
     */
    virtual void handleCheckForWarmReset(Event event);
};

} // namespace power_control

