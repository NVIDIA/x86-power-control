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
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <sdbusplus/asio/object_server.hpp>

// Forward declarations (these enums are defined in power_control.cpp)
namespace power_control
{
    enum class PowerState;
    enum class ConfigType;
    class PersistentState;
}

namespace power_control
{

// TODO: Add the Upsteram ConfigType enum to the class

/**
 * @brief Restart Cause enumeration
 * 
 * Tracks the reason for a host restart/reboot.
 * Used to set the RestartCause D-Bus property.
 */
enum class RestartCause
{
    command,
    resetButton,
    powerButton,
    watchdog,
    powerPolicyOn,
    powerPolicyRestore,
    softReset,
};

/**
 * @brief Global set of restart causes for the current restart
 * 
 * Multiple causes can be added during a restart sequence.
 * The highest priority cause is selected and reported.
 */
extern boost::container::flat_set<RestartCause> causeSet;

/**
 * @brief Convert RestartCause enum to D-Bus string
 * 
 * @param cause The restart cause to convert
 * @return D-Bus RestartCause property string
 */
std::string getRestartCause(RestartCause cause);

/**
 * @brief Add a restart cause to the set
 * 
 * @param cause The restart cause to add
 */
void addRestartCause(const RestartCause cause);

/**
 * @brief Clear the restart cause set for next restart
 */
void clearRestartCause();

/**
 * @brief Set the RestartCause D-Bus property
 * 
 * @param cause The restart cause string to set
 */
void setRestartCauseProperty(const std::string& cause);

/**
 * @brief Determine and set the restart cause from causeSet
 * 
 * Evaluates the set of causes and selects the highest priority
 * cause to report via the D-Bus RestartCause property.
 */
void setRestartCause();

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
                 const std::string& node,
                 PersistentState& appState);
    virtual ~PowerControl() = default;

    /**
     * @brief Power control events
     * 
     * Events that trigger power state transitions.
     */
    enum class Event
    {
        psPowerOKAssert,
        psPowerOKDeAssert,
        sioPowerGoodAssert,
        sioPowerGoodDeAssert,
        sioS5Assert,
        sioS5DeAssert,
        pltRstAssert,
        pltRstDeAssert,
        postCompleteAssert,
        postCompleteDeAssert,
        powerButtonPressed,
        resetButtonPressed,
        powerCycleTimerExpired,
        psPowerOKWatchdogTimerExpired,
        pdbMainPowerOkWatchdogTimerExpired,
        hpmPowerGoodWatchdogTimerExpired,
        cpuResetWatchdogTimerExpired,
        cpuShutdownOkWatchdogTimerExpired,
        sioPowerGoodWatchdogTimerExpired,
        gracefulPowerOffTimerExpired,
        powerOnRequest,
        powerOffRequest,
        powerCycleRequest,
        resetRequest,
        gracefulPowerOffRequest,
        gracefulPowerCycleRequest,
        warmResetDetected,
        nvl144pdbMainPowerOkAssert,
        nvl144pdbMainPowerOkDeAssert,
        gb300pdbMainPowerOkAssert,
        gb300pdbMainPowerOkDeAssert,
        c2pdbPSUPowerOkAssert,
        c2pdbPSUPowerOkDeAssert,
        board0RunPowerPGAssert,
        board0RunPowerPGDeAssert,
        board1RunPowerPGAssert,
        board1RunPowerPGDeAssert,
        cpuResetIndicatorAssert,
        cpuResetIndicatorDeAssert,
        board0CpuShutdownOkAssert,
        board0CpuShutdownOkDeAssert,
        board1CpuShutdownOkAssert,
        board1CpuShutdownOkDeAssert,
    };

    /**
     * @brief Get human-readable name for an Event
     * 
     * Converts an Event enum value to a descriptive string for logging.
     * 
     * @param event The event to get the name for
     * @return Human-readable event name
     */
    static std::string getEventName(Event event);

    /**
     * @brief Log an event received by a state handler
     * 
     * Logs an informational message showing which state handler received which event.
     * 
     * @param stateHandler Name of the state handler function
     * @param event The event that was received
     */
    static void logEvent(std::string_view stateHandler, Event event);

    std::string hostDbusName = "xyz.openbmc_project.State.Host";
    std::string chassisDbusName = "xyz.openbmc_project.State.Chassis";
    std::string osDbusName = "xyz.openbmc_project.State.OperatingSystem";
    std::string buttonDbusName = "xyz.openbmc_project.Chassis.Buttons";
    std::string nmiDbusName = "xyz.openbmc_project.Control.Host.NMI";
    std::string rstCauseDbusName =
        "xyz.openbmc_project.Control.Host.RestartCause";

    enum class PowerAction {
        NONE,
        POWER_ON,
        FORCE_OFF,
        GRACE_OFF,
        POWER_CYCLE,
        SYSTEM_RESET,
        HOST_INITIATED_SHUTDOWN,
    };

    // This map contains all timer values that are to be read from json config
    boost::container::flat_map<std::string, int> TimerMap = {
    {"PowerPulseMs", 200},
    {"ForceOffPulseMs", 15000},
    {"ResetPulseMs", 500},
    {"PowerCycleMs", 5000},
    {"SioPowerGoodWatchdogMs", 1000},
    {"CpuResetWatchdogMs", 10000},
    {"CpuShutdownOkWatchdogMs", 10000},
    {"GracefulPowerOffS", (5 * 60)},
    {"WarmResetCheckMs", 500},
    {"PowerOffSaveMs", 7000},
    {"SlotPowerCycleMs", 200},
    {"DbusGetPropertyRetry", 1000}};

    enum class DbusConfigType
    {
        name = 1,
        path,
        interface,
        property
    };

    // Mandatory config parameters for dbus inputs
    boost::container::flat_map<DbusConfigType, std::string> dbusParams = {
        {DbusConfigType::name, "DbusName"},
        {DbusConfigType::path, "Path"},
        {DbusConfigType::interface, "Interface"},
        {DbusConfigType::property, "Property"}};

    enum class ConfigType
    {
        GPIO = 1,
        DBUS
    };

    PowerAction action = PowerAction::NONE;
    std::string target_state = "HostOff";


    std::shared_ptr<sdbusplus::asio::dbus_interface> hostIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> bootProgressIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> chassisIface;
    #ifdef CHASSIS_SYSTEM_RESET
    std::shared_ptr<sdbusplus::asio::dbus_interface> chassisSysIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> chassisSlotIface;
    #endif
    std::shared_ptr<sdbusplus::asio::dbus_interface> powerButtonIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> resetButtonIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> nmiButtonIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> osIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> idButtonIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> nmiOutIface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> restartCauseIface;

    gpiod::line powerButtonMask;
    gpiod::line resetButtonMask;
    bool nmiButtonMasked;
    #if IGNORE_SOFT_RESETS_DURING_POST
    bool ignoreNextSoftReset;
    #endif

    // Changed from default true to false
    bool nmiEnabled;
    bool nmiWhenPoweredOff;
    bool sioEnabled;

    /**
     * @brief Get the handler function for the current power state
     * 
     * This virtual function maps PowerState enum values to their corresponding
     * handler functions. Derived classes can override this to add handlers for
     * additional states.
     * 
     * @return Function that handles events in the current state, or nullptr if unknown
     */
    virtual std::function<void(Event)> getPowerStateHandler();

    /**
     * @brief Send an event to the appropriate power state handler
     * 
     * This method looks up the handler for the current power state and dispatches
     * the event to it. If no handler is found, an error is logged.
     * 
     * @param event The event to dispatch
     */
    void sendPowerControlEvent(Event event);

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
     * @brief Save the power state to persistent storage after a delay
     * 
     * Schedules saving the current power state to persistent storage.
     * The save is delayed to avoid excessive writes during rapid state changes.
     */
    void savePowerState(const PowerState state);

    /**
     * @brief Get the host state string for a given power state (virtual)
     * 
     * Converts a PowerState enum to the corresponding D-Bus host state string.
     * Derived classes can override this to provide platform-specific mappings.
     * 
     * @param state The power state
     * @return D-Bus host state string
     */
    virtual std::string_view getHostState() const;

    /**
     * @brief Get the chassis state string for current power state (virtual)
     * 
     * Converts the internal powerState to the corresponding D-Bus chassis state string.
     * Derived classes can override this to provide platform-specific mappings.
     * 
     * @return D-Bus chassis state string
     */
    virtual std::string_view getChassisState() const;

    /**
     * @brief Get the current power state
     * 
     * @return Current PowerState enum value
     */
    PowerState getPowerState() const { return powerState; }

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

    /**
     * @brief Set a GPIO output to a specified value
     * 
     * Finds a GPIO line by name, requests it as an output, and sets its value.
     * If the line is already requested, just sets the value.
     * Uses the ConfigData object from powerSignalMap which contains the line name
     * and GPIO line handle.
     * 
     * @param config Shared pointer to ConfigData containing GPIO information
     * @param value The value to set (0 or 1)
     * @return true if successful, false otherwise
     */
    bool setGPIOOutput(std::shared_ptr<ConfigData> config, const int value);

    /**
     * @brief Start a timer with timeout from TimerMap
     * 
     * Looks up the timeout value from TimerMap and starts the timer.
     * Handles all standard error checking and logging.
     * Calls sendPowerControlEvent with the specified event when timer expires.
     * 
     * @param timerName Name of the timer in TimerMap to lookup timeout
     * @param timer Reference to the timer to start
     * @param eventOnExpiry Event to send when timer expires successfully
     */
    void startTimer(const std::string& timerName,
                    boost::asio::steady_timer& timer,
                    Event eventOnExpiry);

    /**
     * @brief Start a timer with direct timeout value
     * 
     * Starts the timer with the provided timeout value.
     * Handles all standard error checking and logging.
     * Calls sendPowerControlEvent with the specified event when timer expires.
     * 
     * @param timeoutMs Timeout in milliseconds
     * @param timer Reference to the timer to start
     * @param eventOnExpiry Event to send when timer expires successfully
     */
    void startTimer(int timeoutMs,
                    boost::asio::steady_timer& timer,
                    Event eventOnExpiry);

    /**
     * @brief Validate that all required signals are present in config
     * 
     * This virtual method checks that ConfigData objects for all required signals
     * exist in powerSignalMap. The base PowerControl implementation is a placeholder.
     * 
     * Derived classes override to check for platform-specific required signals.
     * 
     * @throws std::runtime_error if any required signal is missing from config
     */
    virtual void validateRequiredSignals();

protected:
    /**
     * @brief Power signal map - maps signal names to ConfigData
     * 
     * This map is dynamically populated from the JSON config file during construction.
     * Each entry contains all resource information for a single signal.
     */
    std::map<std::string, std::shared_ptr<ConfigData>> powerSignalMap;
    std::string configFilePath;
    std::shared_ptr<sdbusplus::asio::connection> conn;
    std::string node;
    PersistentState& appState;
    
    /**
     * @brief Current power state
     */
    PowerState powerState;

    /**
     * @brief Map of GPIO signal names to their handler functions
     * 
     * This map is built up by each class in the hierarchy (PowerControl, VRPowerControl, 
     * and platform-specific classes) in their constructors. The registerGPIOHandlers() 
     * method then assigns these handlers to the corresponding ConfigData objects in the 
     * powerSignalMap and calls requestGPIOEvents() for each.
     */
    std::map<std::string, std::function<void(bool)>> gpioHandlerMap;

    /**
     * @brief Register all GPIO handlers and request GPIO events
     * 
     * This method should be called in the most derived class's constructor after all
     * classes have added their handlers to gpioHandlerMap. It iterates through the map,
     * assigns the handler functions to the corresponding ConfigData objects in the
     * powerSignalMap, and calls requestGPIOEvents() for each signal.
     * 
     * @throws std::runtime_error if a signal in gpioHandlerMap is not found in powerSignalMap
     *                            or if requestGPIOEvents() fails for any signal
     */
    void registerGPIOHandlers();
    
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
     * @brief Application name for GPIO requests
     */
    std::string appName;
    
    // UPSTREAM TIMERS
    // These timers are used by the upstream (OpenBMC x86-power-control) functionality
    
    /**
     * @brief Timer for holding GPIOs asserted
     */
    boost::asio::steady_timer gpioAssertTimer;
    
    /**
     * @brief Timer between off and on during a power cycle
     */
    boost::asio::steady_timer powerCycleTimer;
    
    /**
     * @brief Timer for OS gracefully powering off
     */
    boost::asio::steady_timer gracefulPowerOffTimer;
    
    /**
     * @brief Timer for warm reset check
     */
    boost::asio::steady_timer warmResetCheckTimer;
    
    /**
     * @brief Timer for power supply power OK assertion on power-on
     */
    boost::asio::steady_timer psPowerOKWatchdogTimer;
    
    /**
     * @brief Timer for SIO power good assertion on power-on
     */
    boost::asio::steady_timer sioPowerGoodWatchdogTimer;
    
    /**
     * @brief Timer for power-off state save for power loss tracking
     */
    boost::asio::steady_timer powerStateSaveTimer;
    
    /**
     * @brief POH (Power On Hours) timer
     */
    boost::asio::steady_timer pohCounterTimer;
    
    /**
     * @brief Timer for when to allow restart cause updates
     */
    boost::asio::steady_timer restartCauseTimer;
    
    /**
     * @brief Timer for slot power cycle
     */
    boost::asio::steady_timer slotPowerCycleTimer;
    
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
     * @brief Initialize Host D-Bus interface
     * 
     * Creates and registers the host state D-Bus interface for host power transitions.
     */
    void initializeHostInterface();
    
    /**
     * @brief Initialize Chassis D-Bus interface
     * 
     * Creates and registers the chassis state D-Bus interface for chassis power transitions.
     */
    void initializeChassisInterface();
    
    /**
     * @brief Initialize Chassis System D-Bus interface (CHASSIS_SYSTEM_RESET)
     * 
     * Creates and registers the chassis system interface for system-level power control.
     * Only available when CHASSIS_SYSTEM_RESET is defined.
     */
    void initializeChassisSystemInterface();
    
    /**
     * @brief Initialize Boot Progress D-Bus interface
     * 
     * Creates and registers the Boot.Progress interface for tracking boot stages.
     * This allows external entities (IPMI, PLDM, etc.) to update boot progress.
     */
    void initializeBootProgressInterface();
    
    /**
     * @brief Initialize Button D-Bus interfaces
     * 
     * Creates and registers D-Bus interfaces for all button controls:
     * - Power button interface
     * - Reset button interface
     * - NMI button interface
     * - ID button interface
     * - NMI out interface
     */
    void initializeButtonInterfaces();
    
    /**
     * @brief Initialize OS State D-Bus interface
     * 
     * Creates and registers the Operating System Status interface.
     */
    void initializeOSInterface();
    
    /**
     * @brief Initialize Restart Cause D-Bus interface
     * 
     * Creates and registers the Restart Cause interface for tracking
     * why the host was restarted.
     */
    void initializeRestartCauseInterface();
    
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

protected:

    /**
     * @brief Handler for PS Power OK GPIO signal
     * 
     * Called when the power supply power OK signal changes state.
     * Sends psPowerOKAssert or psPowerOKDeAssert event based on polarity.
     * 
     * Signal key in powerSignalMap: "PowerOk"
     * 
     * @param state The current state of the GPIO line
     */
    virtual void psPowerOKHandler(bool state);

    /**
     * @brief Handler for SIO Power Good GPIO signal
     * 
     * Called when the SIO power good signal changes state.
     * Sends sioPowerGoodAssert or sioPowerGoodDeAssert event based on polarity.
     * 
     * Signal key in powerSignalMap: "SioPowerGood"
     * 
     * @param state The current state of the GPIO line
     */
    virtual void sioPowerGoodHandler(bool state);

    /**
     * @brief Handler for SIO S5 GPIO signal
     * 
     * Called when the SIO S5 signal changes state.
     * Sends sioS5Assert or sioS5DeAssert event based on polarity.
     * 
     * Signal key in powerSignalMap: "SIOS5"
     * 
     * @param state The current state of the GPIO line
     */
    virtual void sioS5Handler(bool state);

    /**
     * @brief Handler for Power Button GPIO signal
     * 
     * Called when the power button signal changes state.
     * Updates D-Bus ButtonPressed property and sends powerButtonPressed event.
     * Adds RestartCause::powerButton when button is pressed.
     * Respects powerButtonMask for masking button presses.
     * 
     * Signal key in powerSignalMap: "PowerButton"
     * 
     * @param state The current state of the GPIO line
     */
    virtual void powerButtonHandler(bool state);

    /**
     * @brief Handler for Reset Button GPIO signal
     * 
     * Called when the reset button signal changes state.
     * Updates D-Bus ButtonPressed property and sends resetButtonPressed event.
     * Adds RestartCause::resetButton when button is pressed.
     * Respects resetButtonMask for masking button presses.
     * 
     * Signal key in powerSignalMap: "ResetButton"
     * 
     * @param state The current state of the GPIO line
     */
    virtual void resetButtonHandler(bool state);

    /**
     * @brief Log a power button press event
     * 
     * Sends a Redfish event log entry for power button press.
     */
    void powerButtonPressLog();

    /**
     * @brief Log a reset button press event
     * 
     * Sends a Redfish event log entry for reset button press.
     */
    void resetButtonPressLog();

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

