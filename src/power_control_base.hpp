#pragma once

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <regex>
#include <string>
#include <string_view>
#include <gpiod.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <sdbusplus/asio/object_server.hpp>

// Forward declarations (these enums are defined in power_control.cpp)
namespace power_control
{
    enum class PowerState;
    enum class Event;
    enum class ConfigType;
}

namespace power_control
{

// TODO: Add the Upsteram ConfigType enum to the class

/**
 * @brief Upstream Configuration data for a single GPIO or D-Bus signal
 * 
 * This structure consolidates all resources needed for a signal:
 * - Metadata (name, line name, polarity, type)
 * - GPIO line handle
 * - Event descriptor for async monitoring (populated if an input GPIO and requested via requestGPIOEvents)
 * - Handler function for GPIO events (initially null, if an input GPIO  added after loadConfigValues)
 */
struct ConfigData
{
    std::string name;
    std::string lineName;
    std::string dbusName;
    std::string path;
    std::string interface;
    std::optional<std::regex> matchRegex;
    bool polarity;
    ConfigType type;
    gpiod::line gpioLine;                               // GPIO line handle
    boost::asio::posix::stream_descriptor eventDescriptor; // Event descriptor for async monitoring
    std::function<void(bool)> gpioHandler;              // Handler function for GPIO events (initially null, populated after loadConfigValues)
    
    // Constructor to initialize event descriptor with io_context
    ConfigData(boost::asio::io_context& io) 
        : eventDescriptor(io), gpioHandler(nullptr) {}
};

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
    PowerControl(boost::asio::io_context& ioContext, 
                 std::shared_ptr<sdbusplus::asio::connection> conn,
                 const std::string& node);
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

    /**
     * @brief Set the power state and update D-Bus interfaces
     * 
     * This method updates the internal power state and writes to 
     * the dbus host and chassis interfaces. It uses virtual dispatch to call the
     * appropriate getHostState() and getChassisState() implementations.
     * 
     * @param state The new power state
     */
    void setPowerState(const PowerState state);

    /**
     * @brief Get the host state string for a given power state (virtual)
     * 
     * Converts a PowerState enum to the corresponding D-Bus host state string.
     * Derived classes can override this to provide platform-specific mappings.
     * 
     * @param state The power state
     * @return D-Bus host state string
     */
    virtual std::string_view getHostState(const PowerState state);

    /**
     * @brief Get the chassis state string for a given power state (virtual)
     * 
     * Converts a PowerState enum to the corresponding D-Bus chassis state string.
     * Derived classes can override this to provide platform-specific mappings.
     * 
     * @param state The power state
     * @return D-Bus chassis state string
     */
    virtual std::string_view getChassisState(const PowerState state);

    /**
     * @brief Get a human-readable name for a power state (virtual)
     * 
     * Converts a PowerState enum to a string for logging purposes.
     * 
     * @param state The power state
     * @return Human-readable state name
     */
    virtual std::string getPowerStateName(const PowerState state);

    /**
     * @brief Log a power state transition
     * 
     * Logs an informational message when the power state changes.
     * 
     * @param state The new power state
     */
    void logStateTransition(const PowerState state);

    /**
     * @brief Get current time in milliseconds
     * 
     * Returns the current system time in milliseconds since epoch.
     * Used for timestamping power state transitions.
     * 
     * @return Current time in milliseconds
     */
    uint64_t getCurrentTimeMs();

protected:
    /**
     * @brief Power signal map - maps signal names to ConfigData
     * 
     * This map is dynamically populated from the JSON config file during construction.
     * Each entry contains all resource information for a single signal.
     */
    std::map<std::string, std::shared_ptr<ConfigData>> powerSignalMap;
    
    /**
     * @brief Reference to the io_context for async operations
     */
    boost::asio::io_context& ioContext;
    
    /**
     * @brief D-Bus connection
     */
    std::shared_ptr<sdbusplus::asio::connection> dbusConn;
    
    /**
     * @brief Node identifier
     */
    std::string nodeId;
    
    /**
     * @brief Static D-Bus interface for host state
     * 
     */
    static std::shared_ptr<sdbusplus::asio::dbus_interface> hostIface;
    
    /**
     * @brief Static D-Bus interface for chassis state
     * 
     */
    static std::shared_ptr<sdbusplus::asio::dbus_interface> chassisIface;
    
    /**
     * @brief Load configuration values from JSON config file
     * 
     * This method reads the JSON config file and dynamically creates ConfigData
     * entries in powerSignalMap for each signal defined in the file.
     * 
     * Called by the base class constructor.
     * 
     * @param io The io_context for initializing stream descriptors
     */
    void loadConfigValues(boost::asio::io_context& io);
    
    /**
     * @brief Initialize D-Bus interfaces
     * 
     * Creates and registers the host, chassis D-Bus interfaces, and other upstream D-Bus interfaces.
     * 
     * @param conn The D-Bus connection
     * @param node The node identifier (e.g., "0" for host0)
     */
    void initializeDBusInterfaces(std::shared_ptr<sdbusplus::asio::connection> conn,
                                   const std::string& node);
    
    /**
     * @brief Request GPIO events for a signal
     * 
     * Registers async event monitoring for a GPIO signal. The handler function
     * stored in ConfigData will be called when events occur.
     * 
     * @param config The ConfigData containing GPIO line, event descriptor, and handler
     * @return true if successful, false otherwise
     * @throws std::runtime_error if registration fails
     */
    bool requestGPIOEvents(ConfigData& config);
    
    /**
     * @brief Wait for GPIO event on a signal
     * 
     * Sets up async waiting for the next GPIO event. This is called recursively
     * by requestGPIOEvents to continuously monitor events.
     * 
     * @param config The ConfigData containing GPIO line, event descriptor, and handler
     */
    void waitForGPIOEvent(ConfigData& config);

    // TODO: Add the Upstream Event Descriptors to the class
    // TODO: Add the Upstream ConfigData to the class
    // TODO: Add the Upstream GPIO Lines to the class
    // TODO: Add the Upstream GPIO Event Handlers

protected:

    // TODO: Add the Upstream Event Descriptors to the class
    // TODO: Add the Upstream ConfigData to the class
    // TODO: Add the Upstream GPIO Lines to the class
    // TODO: Add the Upstream GPIO Event Handlers

    // UPSTREAM STATE HANDLERS
    // These handle upstream power states and should match upstream behavior

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

