// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#pragma once

#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <gpiod.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <regex>
#include <string>
#include <string_view>

// Forward declarations
namespace power_control
{
class PersistentState;
} // namespace power_control

namespace power_control
{

// Power state enumeration - defines all possible power states
enum class PowerState
{
    on,
    off,
    waitForPowerOK,
    waitForSIOPowerGood,
    transitionToOff,
    gracefulTransitionToOff,
    cycleOff,
    transitionToCycleOff,
    gracefulTransitionToCycleOff,
    checkForWarmReset,
    // VR-specific states
    waitForPDBMainPowerOk,
    waitForPDBMainPowerOff,
    waitForHPMPowerGoodAssert,
    waitForHPMPowerGoodDeAssert,
    waitForCPUResetAssert,
    waitForCPUResetDeAssert,
    waitForCPUShutdownOk,
    waitForCPUBootDoneDeAssert,
    waitForPowerCycleDelay,
    waitForRebootDelay,
};

// Configuration type enumeration
enum class ConfigType
{
    GPIO = 1,
    DBUS
};

// In tca spec, 0 indicates output and 1 indicates input
enum class GPIODirection
{
    OUT = 0,
    IN
};

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
 * @brief Operating system state stages
 *
 * Represents the current state of the operating system as it boots and runs.
 */
enum class OperatingSystemStateStage
{
    Inactive,
    Standby,
};

/**
 * @brief Default state enumeration for output GPIO signals
 *
 * Defines the default state for output GPIO signals in different host states.
 * NA is used for input signals or signals without a default state.
 */
enum class DefaultState
{
    NA,         // Not applicable (input signals or no default)
    Asserted,   // Signal should be asserted
    DeAsserted, // Signal should be de-asserted
};

/**
 * @brief Convert RestartCause enum to D-Bus string
 *
 * @param cause The restart cause to convert
 * @return D-Bus RestartCause property string
 */
std::string getRestartCause(RestartCause cause);

/**
 * @brief Input event configuration for gpio_keys_polled driver signals
 *
 * This structure contains the parameters needed for monitoring Linux input
 * events via the gpio_keys_polled driver, as an alternative to direct GPIO
 * monitoring.
 */
struct InputEventConfig
{
    std::string deviceName; // Input device name (e.g., "gpio_keys_gb300")
    std::string signalName; // Signal name for logging (e.g.,
                            // "GB300_PDB_MAIN_PWR_OK_Mon")
    uint16_t keyCode;  // Key code to filter events by (e.g., 0x102 for BTN_2)
    int* stateTracker; // Pointer to track the cached state value

    InputEventConfig() : keyCode(0), stateTracker(nullptr) {}
};

/**
 * @brief Upstream Configuration data for a single GPIO or D-Bus signal
 *
 * This structure consolidates all resources needed for a signal:
 * - Metadata (name, line name, polarity, type)
 * - GPIO line handle
 * - Event descriptor for async monitoring (populated if an input GPIO and
 * requested via requestGPIOEvents)
 * - Handler function for GPIO events (initially null, if an input GPIO  added
 * after loadConfigValues)
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
    gpiod::line gpioLine;    // GPIO line handle
    GPIODirection direction; // GPIO direction (input or output)
    boost::asio::posix::stream_descriptor
        eventDescriptor;     // Event descriptor for async monitoring
    std::function<void(bool)>
        gpioHandler; // Handler function for GPIO events (initially null,
                     // populated after loadConfigValues)

    // Input event monitoring (for gpio_keys_polled driver)
    bool useInputEvents;  // Use input event monitoring instead of direct GPIO
    std::optional<InputEventConfig>
        inputEventConfig; // Configuration for input event monitoring

    // Default states for output signals in different host states
    DefaultState defaultStateHostStateOn;  // Default state when host is on
    DefaultState defaultStateHostStateOff; // Default state when host is off

    // Constructor to initialize event descriptor with io_context
    ConfigData(boost::asio::io_context& io) :
        eventDescriptor(io), gpioHandler(nullptr), useInputEvents(false),
        defaultStateHostStateOn(DefaultState::NA),
        defaultStateHostStateOff(DefaultState::NA)
    {}
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
 * This class only knows about/uses upstream PowerState enums (on, off,
 * waitForPSPowerOK, etc.)
 */
class PowerControl
{
  public:
    PowerControl(boost::asio::io_context& ioContext,
                 std::shared_ptr<sdbusplus::asio::connection> conn,
                 const std::string& node, PersistentState& appState,
                 const std::string& configFilePath = "");
    virtual ~PowerControl() = default;

    /**
     * @brief Power control events
     *
     * Events that trigger power state transitions.
     */
    enum class Event
    {
        powerOKAssert,
        powerOKDeAssert,
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
        powerOKWatchdogTimerExpired,
        pdbMainPowerOkWatchdogTimerExpired,
        hpmPowerGoodWatchdogTimerExpired,
        cpuResetWatchdogTimerExpired,
        cpuShutdownOkWatchdogTimerExpired,
        cpuBootDoneDeAssertWatchdogTimerExpired,
        sioPowerGoodWatchdogTimerExpired,
        gracefulPowerOffTimerExpired,
        powerOnRequest,
        powerOffRequest,
        powerCycleRequest,
        resetRequest,
        gracefulPowerOffRequest,
        gracefulPowerCycleRequest,
        warmResetDetected,
        powerCycleDelayTimerExpired,
        warmRebootDelayTimerExpired,
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
        cpuBootDoneAssert,
        cpuBootDoneDeAssert,
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
     * Logs an informational message showing which state handler received which
     * event.
     *
     * @param stateHandler Name of the state handler function
     * @param event The event that was received
     */
    static void logEvent(std::string_view stateHandler, Event event);

    /**
     * @brief Request all D-Bus bus names for this service
     *
     * This should be called after all initialization is complete,
     * so clients don't discover the service before it's ready.
     */
    void requestBusNames();

    std::string hostDbusName = "xyz.openbmc_project.State.Host";
    std::string chassisDbusName = "xyz.openbmc_project.State.Chassis";
    std::string osDbusName = "xyz.openbmc_project.State.OperatingSystem";
    std::string buttonDbusName = "xyz.openbmc_project.Chassis.Buttons";
    std::string nmiDbusName = "xyz.openbmc_project.Control.Host.NMI";

    enum class PowerAction
    {
        NONE,
        POWER_ON,
        FORCE_OFF,
        GRACE_OFF,
        POWER_CYCLE,
        GRACEFUL_POWER_CYCLE,
        SYSTEM_RESET,
        HOST_INITIATED_SHUTDOWN,
        FORCE_WARM_REBOOT,
    };

    // This map contains all timer values that are to be read from json config
    boost::container::flat_map<std::string, int> TimerMap;

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
    std::shared_ptr<sdbusplus::asio::dbus_interface> gpioStateIface;

    /**
     * @brief Map of GPIO signal names to their D-Bus property setter functions
     *
     * Used to initialize D-Bus GPIO properties with actual hardware values.
     * Base class initializes with common Board0 signals. Derived classes can
     * add platform-specific signals (e.g., Board1 signals).
     */
    std::unordered_map<std::string, std::function<void(PowerControl*, int)>>
        gpioPropertySetters;

    gpiod::line powerButtonMask;
    gpiod::line resetButtonMask;
    bool nmiButtonMasked = false;
#if IGNORE_SOFT_RESETS_DURING_POST
    bool ignoreNextSoftReset = false;
#endif

    // Changed from default true to false
    bool nmiEnabled = true;
    bool nmiWhenPoweredOff = true;
    bool sioEnabled = true;

    /**
     * @brief Current operating system state
     */
    OperatingSystemStateStage operatingSystemState =
        OperatingSystemStateStage::Inactive;

    /**
     * @brief GPIO state tracking for D-Bus properties
     *
     * These track the current state of common GPIO signals owned and will be
     * exposed via D-Bus: -1 = Uninitialized (no event received yet) 0 =
     * De-asserted 1 = Asserted
     */
    int cpuBootDone{-1};
    int cpuResetIndicatorState{-1};
    int board0RunPowerPGState{-1};
    int board0CpuShutdownOkState{-1};
    int board1CpuShutdownOkState{-1};

    /**
     * @brief Setter methods for VR GPIO states
     *
     * These update both the member variable and the D-Bus property.
     * Called from GPIO event handlers in derived classes.
     */
    void setCpuResetIndicatorState(int state);
    void setBoard0RunPowerPGState(int state);
    void setBoard0CpuShutdownOkState(int state);
    void setBoard1CpuShutdownOkState(int state);

    /**
     * @brief Get the handler function for the current power state
     *
     * This virtual function maps PowerState enum values to their corresponding
     * handler functions. Derived classes can override this to add handlers for
     * additional states.
     *
     * @return Function that handles events in the current state, or nullptr if
     * unknown
     */
    virtual std::function<void(Event)> getPowerStateHandler();

    /**
     * @brief Send an event to the appropriate power state handler
     *
     * This method looks up the handler for the current power state and
     * dispatches the event to it. If no handler is found, an error is logged.
     *
     * @param event The event to dispatch
     */
    void sendPowerControlEvent(Event event);

    /**
     * @brief Set the power state and update D-Bus interfaces
     *
     * This method updates the internal power state and writes to
     * the dbus host and chassis interfaces. It uses virtual dispatch to call
     * the appropriate getHostState() and getChassisState() implementations.
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
     * Converts the internal powerState to the corresponding D-Bus chassis state
     * string. Derived classes can override this to provide platform-specific
     * mappings.
     *
     * @return D-Bus chassis state string
     */
    virtual std::string_view getChassisState() const;

    /**
     * @brief Get the current power state
     *
     * @return Current PowerState enum value
     */
    PowerState getPowerState() const
    {
        return powerState;
    }

    /**
     * @brief Get a human-readable name for the current power state (virtual)
     *
     * Converts the internal powerState enum to a string for logging purposes.
     *
     * @return Human-readable state name
     */
    virtual std::string getPowerStateName();

    /**
     * @brief Log a power state transition
     *
     * Logs an informational message when the power state changes.
     */
    void logStateTransition();

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
     * @brief Get a signal from powerSignalMap by name
     *
     * Looks up a signal in powerSignalMap and returns its ConfigData.
     * Logs a CRITICAL error if the signal is not found.
     *
     * @param signalName Name of the signal to find
     * @return Shared pointer to ConfigData, or nullptr if not found
     */
    std::shared_ptr<ConfigData> getSignal(const std::string& signalName);

    /**
     * @brief Set a GPIO output to a specified value
     *
     * Finds a GPIO line by name, requests it as an output, and sets its value.
     * If the line is already requested, just sets the value.
     * Uses the ConfigData object from powerSignalMap which contains the line
     * name and GPIO line handle.
     *
     * @param config Shared pointer to ConfigData containing GPIO information
     * @param value The value to set (0 or 1)
     * @return true if successful, false otherwise
     */
    bool setGPIOOutput(std::shared_ptr<ConfigData> config, const int value);

    /**
     * @brief Set a masked GPIO output for a specified duration
     *
     * Sets a masked GPIO line (powerButtonMask or resetButtonMask) to a value
     * for a specified duration, then automatically releases it back to the
     * opposite value. Uses the ConfigData to access the GPIO line and name.
     *
     * @param config Shared pointer to ConfigData containing the masked GPIO
     * line
     * @param value The value to set
     * @param durationMs Duration in milliseconds to hold the value
     * @return 0 on success, -1 on failure
     */
    int setMaskedGPIOOutputForMs(std::shared_ptr<ConfigData> config,
                                 const int value, const int durationMs);

    /**
     * @brief Set a GPIO output for a specified duration
     *
     * Sets a GPIO to a value for a specified duration, then automatically
     * releases it back to the opposite value. Respects button masking for
     * PowerOut and ResetOut.
     *
     * @param config Shared pointer to ConfigData containing GPIO information
     * @param value The value to set
     * @param durationMs Duration in milliseconds to hold the value
     * @return 0 on success, -1 on failure
     */
    int setGPIOOutputForMs(std::shared_ptr<ConfigData> config, const int value,
                           const int durationMs);

    /**
     * @brief Assert a GPIO according to its polarity for a specified duration
     *
     * Helper function that asserts a GPIO to its configured active polarity
     * for the specified duration.
     *
     * @param config Shared pointer to ConfigData containing GPIO information
     * and polarity
     * @param durationMs Duration in milliseconds to hold the assertion
     * @return 0 on success, -1 on failure
     */
    int assertGPIOForMs(std::shared_ptr<ConfigData> config,
                        const int durationMs);

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
                    boost::asio::steady_timer& timer, Event eventOnExpiry);

    /**
     * @brief Cancel a timer with logging
     *
     * Cancels the specified timer and logs the cancellation.
     * Logs twice: once when cancel is initiated, and once when the
     * async_wait callback fires with operation_aborted.
     *
     * @param timerName Human-readable name of the timer for logging
     * @param timer Reference to the timer to cancel
     */
    void cancelTimer(const std::string& timerName,
                     boost::asio::steady_timer& timer);

    /**
     * @brief Validate that all required signals are present in config
     *
     * This virtual method checks that ConfigData objects for all required
     * signals exist in powerSignalMap. The base PowerControl implementation is
     * a placeholder.
     *
     * Derived classes override to check for platform-specific required signals.
     *
     * @throws std::runtime_error if any required signal is missing from config
     */
    virtual void validateRequiredSignals();

    /**
     * @brief Validate that all required timer configurations are present in
     * TimerMap
     *
     * This method checks that timeout values for all required timers exist in
     * TimerMap after loadConfigValues() has populated it from the JSON config.
     *
     * Derived classes override to check for platform-specific timeout values.
     * Base class implementation validates upstream/base timers.
     *
     * @throws std::runtime_error if any required timer config is missing
     */
    virtual void validateTimerConfigs() = 0;

    /**
     * @brief Monitor NMI source property changes via D-Bus
     *
     * Sets up a D-Bus match to listen for NMI source property changes
     * and triggers nmiReset() when NMI is enabled.
     */
    void nmiSourcePropertyMonitor();

    /**
     * @brief Start the POH (Power On Hours) counter timer
     *
     * Starts a 1-hour timer that increments the POH counter when the host is
     * running. The timer recursively restarts itself to continuously track
     * power-on hours.
     */
    void pohCounterTimerStart();

    /**
     * @brief Beep priority for power failure
     */
    static constexpr uint8_t beepPowerFail = 8;

    /**
     * @brief Send a beep code via D-Bus
     *
     * Sends a beep command with the specified priority to the BeepCode service.
     *
     * @param beepPriority Priority level for the beep (0-255)
     */
    virtual void beep(const uint8_t& beepPriority);

    /**
     * @brief List of required base/upstream timer configurations
     */
    const std::vector<std::string> baseRequiredTimers = {
        "PowerPulseMs",        "ForceOffPulseMs",        "ResetPulseMs",
        "PowerCycleMs",        "SioPowerGoodWatchdogMs", "PowerOKWatchdogMs",
        "GracefulPowerOffS",   "WarmResetCheckMs",       "PowerOffSaveMs",
        "DbusGetPropertyRetry"};

    /**
     * @brief Monitor current host state changes
     *
     * Sets up a D-Bus match to monitor host state property changes.
     * When host transitions to Running, starts POH timer and clears restart
     * cause. When host transitions to Off, cancels POH timer, sets OS state to
     * Inactive, sets restart cause, and logs DC power off event.
     */
    void currentHostStateMonitor();

    /**
     * @brief Set the restart cause D-Bus property
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
     * @brief Add a restart cause to the set
     *
     * @param cause The restart cause to add
     */
    void addRestartCause(const RestartCause cause);

    /**
     * @brief Clear the restart cause set for next restart
     */
    void clearRestartCause();

#ifdef USE_ACBOOT
    /**
     * @brief Reset the ACBoot property based on restart cause
     *
     * If the restart was caused by a command or soft reset, resets
     * the ACBoot property to False.
     */
    void resetACBootProperty();
#endif

  protected:
    /**
     * @brief Get the CPU Boot Done state
     *
     * @return int CPU Boot Done state:
     *         -1 = Uninitialized (not set by GPIO monitor yet)
     *          0 = De-asserted (CPU boot not complete)
     *          1 = Asserted (CPU boot complete)
     */
    int getCPUBootDoneState() const
    {
        return cpuBootDone;
    }

    /**
     * @brief Power signal map - maps signal names to ConfigData
     *
     * This map is dynamically populated from the JSON config file during
     * construction. Each entry contains all resource information for a single
     * signal.
     */
    std::map<std::string, std::shared_ptr<ConfigData>> powerSignalMap;

    /**
     * @brief Reference to the io_context for async operations
     */
    boost::asio::io_context& ioContext;

    std::shared_ptr<sdbusplus::asio::connection> conn;

    /**
     * @brief Object server for managing D-Bus objects
     *
     * Must persist for the lifetime of the daemon to keep ObjectManager
     * and all D-Bus interfaces alive.
     */
    sdbusplus::asio::object_server objServer;

    /**
     * @brief Node identifier
     */
    std::string nodeId;

    PersistentState& appState;

    /**
     * @brief Current power state
     */
    PowerState powerState;

    /**
     * @brief Set of restart causes for the current restart
     *
     * Multiple causes can be added during a restart sequence.
     * The highest priority cause is selected and reported.
     */
    boost::container::flat_set<RestartCause> causeSet;

    /**
     * @brief Required Board 0 signals
     * Add all REQUIRED signals to board0 by default,
     * as board 0 is always present.
     */
    std::vector<std::string> requiredBoard0Signals;

    /**
     * @brief Required Board 1 signals
     * These could potentially be empty, if board 1 is not present.
     */
    std::vector<std::string> requiredBoard1Signals;

    /**
     * @brief Add a required signal to the appropriate board's signal list
     *
     * Checks if the signal is present in powerSignalMap and optionally assigns
     * a handler to its ConfigData. If a handler is provided and the signal
     * exists in powerSignalMap, the handler is assigned directly to the
     * ConfigData's gpioHandler field.
     *
     * @param signalName The name of the GPIO signal to add
     * @param boardIndex The board index (0 or 1)
     * @param handler Optional handler function to assign to the signal's
     *                ConfigData
     */
    void addRequiredSignal(const std::string& signalName, int boardIndex,
                           GPIODirection direction,
                           std::function<void(bool)> handler = nullptr);

    /**
     * @brief Map of GPIO signal names to their handler functions
     *
     * This map is built up by each class in the hierarchy (PowerControl,
     * VRPowerControl, and platform-specific classes) in their constructors. The
     * registerGPIOHandler() method then assigns these handlers to the
     * corresponding ConfigData objects in the powerSignalMap and calls
     * requestGPIOEvents() for each.
     */
    void registerGPIOHandler(const std::string& signalName,
                             GPIODirection direction,
                             std::function<void(bool)> handler);

    /**
     * @brief D-Bus connection
     */
    std::shared_ptr<sdbusplus::asio::connection> dbusConn;

    /**
     * @brief Application name for GPIO requests
     */
    std::string appName;

    /**
     * @brief Configuration file path
     */
    std::string configFilePath;

    // UPSTREAM TIMERS
    // These timers are used by the upstream (OpenBMC x86-power-control)
    // functionality

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
     * @brief Timer for power OK watchdog on power-on
     */
    boost::asio::steady_timer powerOKWatchdogTimer;

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
     * @brief Map of retry timers for D-Bus property reads
     *
     * Maps signal name (string) to retry timer for rescheduling failed D-Bus
     * reads
     */
    boost::container::flat_map<std::string, boost::asio::steady_timer>
        dBusRetryTimers;

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
    void loadConfigValues();

    /**
     * @brief Initialize ObjectManager D-Bus interface
     *
     * Creates the org.freedesktop.DBus.ObjectManager interface on the parent
     * path (/xyz/openbmc_project/state). This MUST be called BEFORE claiming
     * service names to ensure the Mapper daemon can track all child objects.
     *
     * Uses sdbusplus::asio::object_server::add_manager() which automatically
     * provides GetManagedObjects() method and emits InterfacesAdded/Removed
     * signals when child interfaces are initialized, allowing the Mapper to
     * update its cache in real-time.
     */
    void initializeObjectManager();

    /**
     * @brief Initialize Host D-Bus interface
     *
     * Creates and registers the host state D-Bus interface for host power
     * transitions.
     */
    void registerHostInterface();

    /**
     * @brief Initialize Chassis D-Bus interface
     *
     * Creates and registers the chassis state D-Bus interface for chassis power
     * transitions.
     */
    void initializeChassisInterface();

    /**
     * @brief Initialize Chassis System D-Bus interface (CHASSIS_SYSTEM_RESET)
     *
     * Creates and registers the chassis system interface for system-level power
     * control. Only available when CHASSIS_SYSTEM_RESET is defined.
     */
    void initializeChassisSystemInterface();

    /**
     * @brief Initialize Boot Progress D-Bus interface
     *
     * Creates and registers the Boot.Progress interface for tracking boot
     * stages. This allows external entities (IPMI, PLDM, etc.) to update boot
     * progress.
     */
    void initializeBootProgressInterface();

    /**
     * @brief Set boot progress property
     *
     * @param bootProgressStage The boot progress stage string
     */
    void setBootProgress(const std::string& bootProgressStage);

    /**
     * @brief Set boot progress OEM property
     *
     * @param oemProgress The OEM-specific boot progress string
     */
    void setBootProgressOem(const std::string& oemProgress);

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
     * @brief Register GPIO State D-Bus interface
     *
     * Creates xyz.openbmc_project.State.Gpio interface and registers
     * common GPIO methods/properties (CPU Boot Done). Does NOT call
     * initialize() - derived classes can add additional GPIO properties
     * before calling initializeHostStateInterface().
     */
    void registerGpioStateInterface();

    /**
     * @brief Initialize ALL D-Bus interfaces on host0 path (call from derived
     * class)
     *
     * Initializes all interfaces on /xyz/openbmc_project/state/host0:
     * - xyz.openbmc_project.State.Gpio
     * - xyz.openbmc_project.State.Boot.Progress
     * - xyz.openbmc_project.State.OperatingSystem.Status
     * - xyz.openbmc_project.State.Host
     *
     * This should be called from the most-derived class constructor AFTER
     * all interface properties have been registered (including
     * platform-specific GPIO properties like Board1CpuShutdownOk).
     *
     * By initializing all interfaces at once, "mapper wait
     * /xyz/openbmc_project/state/host0" will only return when ALL interfaces
     * are ready.
     */
    void initializeHostStateInterface();

    /**
     * @brief Request GPIO events for a signal
     *
     * Registers async event monitoring for a GPIO signal. The handler function
     * stored in ConfigData will be called when events occur.
     *
     * @param config The ConfigData containing GPIO line, event descriptor, and
     * handler
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
     * @param config The ConfigData containing GPIO line, event descriptor, and
     * handler
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
    virtual void powerOKHandler(bool state);

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
     * @brief Handler for ID Button GPIO signal
     *
     * Called when the ID button signal changes state.
     * Updates D-Bus ButtonPressed property.
     *
     * Signal key in powerSignalMap: "IdButton"
     *
     * @param state The current state of the GPIO line
     */
    virtual void idButtonHandler(bool state);

    /**
     * @brief Handler for PLT_RST GPIO signal
     *
     * Called when the platform reset signal changes state.
     * Sends pltRstAssert or pltRstDeAssert event.
     *
     * @param state The current state of the GPIO line (true = de-asserted)
     */
    virtual void pltRstHandler(bool state);

    /**
     * @brief Handler for SIO_ONCONTROL GPIO signal
     *
     * Called when the SIO on control signal changes state.
     * Logs the state change for debugging purposes.
     *
     * @param state The current state of the GPIO line
     */
    virtual void sioOnControlHandler(bool state);

    /**
     * @brief Handler for Host Misc D-Bus property changes
     *
     * Handles ESpiPlatformReset property changes from the Host.Misc interface.
     * Calls pltRstHandler when the ESpiPlatformReset property changes.
     *
     * @param msg The D-Bus message containing the property changes
     */
    void hostMiscHandler(sdbusplus::message_t& msg);

    /**
     * @brief Extract a property value from a D-Bus PropertiesChanged message
     *
     * Template function to extract a typed value from a D-Bus properties
     * changed signal.
     *
     * @tparam T The expected type of the property value
     * @param msg The D-Bus message to read from
     * @param name The property name to look for
     * @return The property value if found and matches name, std::nullopt
     * otherwise
     */
    template <typename T>
    std::optional<T> getMessageValue(sdbusplus::message_t& msg,
                                     const std::string& name);

    /**
     * @brief Get GPIO state from a D-Bus message
     *
     * Extracts GPIO state from a D-Bus PropertiesChanged message.
     * Supports both boolean properties and regex matching on string properties.
     *
     * @param msg The D-Bus message to read from
     * @param config The ConfigData with property details and optional regex
     * @param value Output parameter for the GPIO state
     * @return true if state was successfully extracted, false otherwise
     */
    bool getDbusMsgGPIOState(sdbusplus::message_t& msg,
                             const ConfigData& config, bool& value);

    /**
     * @brief Create a D-Bus match for GPIO-like property changes
     *
     * Creates a sdbusplus match object that monitors D-Bus property changes
     * and calls the provided callback with the GPIO state.
     *
     * @param cfg The ConfigData with D-Bus service, path, interface details
     * @param onMatch Callback function to invoke with the GPIO state
     * @return A D-Bus match object
     */
    sdbusplus::bus::match_t dbusGPIOMatcher(const ConfigData& cfg,
                                            std::function<void(bool)> onMatch);

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

    /**
     * @brief Log a system power good failure
     *
     * Sends a Redfish event log entry for system power good failure (VR
     * failure). Uses SioPowerGoodWatchdogMs from TimerMap.
     */
    void systemPowerGoodFailedLog();

    /**
     * @brief Log a power supply power OK failure
     *
     * Sends a Redfish event log entry for power supply power good failure.
     * Uses PsPowerOKWatchdogMs from TimerMap.
     */
    void psPowerOKFailedLog();

    /**
     * @brief Log an NMI button press event
     *
     * Sends a Redfish event log entry for NMI button press.
     */
    void nmiButtonPressLog();

    /**
     * @brief Log an NMI diagnostic interrupt event
     *
     * Sends a Redfish event log entry for NMI diagnostic interrupt.
     */
    void nmiDiagIntLog();

    /**
     * @brief Set the NMI Enable property via D-Bus
     *
     * @param value The value to set (true = enabled, false = disabled)
     */
    void nmiSetEnableProperty(bool value);

    /**
     * @brief Perform NMI reset operation
     *
     * Pulses the NMI output GPIO and logs the diagnostic interrupt event.
     */
    void nmiReset();

    /**
     * @brief Set the NMI source via D-Bus
     *
     * Sets the BMC source for NMI to FrontPanelButton and enables NMI.
     */
    void setNmiSource();

    // INPUT EVENT HANDLING (for gpio_keys_polled driver)

    /**
     * @brief Find the correct input event device by name
     *
     * Searches through /dev/input/eventX devices to find the one matching the
     * given name.
     *
     * @param deviceName The name of the input device to find
     * @return The path to the event device (e.g., "/dev/input/event0"), or
     * empty string if not found
     */
    std::string findInputEventDevice(const std::string& deviceName);

    /**
     * @brief Monitor Linux input events
     *
     * Asynchronously waits for and processes Linux input events from a stream
     * descriptor. This is used for handling GPIO events via the
     * gpio_keys_polled driver.
     *
     * @param name The name of the signal for logging purposes
     * @param eventHandler The callback function to handle the event (receives
     * bool state)
     * @param keyCode The key code to filter events by
     * @param event The stream descriptor for the input device
     * @param stateTracker Optional pointer to track the current state
     */
    void waitForInputEvent(
        const std::string& name, const std::function<void(bool)>& eventHandler,
        uint16_t keyCode, boost::asio::posix::stream_descriptor& event,
        int* stateTracker = nullptr);

    /**
     * @brief Request monitoring of Linux input events
     *
     * Sets up monitoring for Linux input events from a gpio_keys_polled driver
     * device. Opens the event device and starts asynchronous event monitoring.
     *
     * @param deviceName The name of the input device to monitor
     * @param signalName The name of the signal for logging purposes
     * @param keyCode The key code to filter events by
     * @param handler The callback function to handle events (receives bool
     * state)
     * @param eventDescriptor The stream descriptor to use for the input device
     * @param stateTracker Optional pointer to track the current state
     * @return true if setup was successful, false otherwise
     */
    bool requestInputEvents(
        const std::string& deviceName, const std::string& signalName,
        uint16_t keyCode, const std::function<void(bool)>& handler,
        boost::asio::posix::stream_descriptor& eventDescriptor,
        int* stateTracker = nullptr);

    // UPSTREAM STATE HANDLERS
    // These handle upstream power states and should match upstream behavior

    /**
     * @brief Handler for PowerState::on
     *
     * PREVIOUS IMPLEMENTATION (Upstream):
     * - Handled DC power loss (psPowerOKDeAssert) → transition to off
     * - Handled S5 assertion (sioS5Assert) → transition to graceful power off
     * - Handled PLT_RST assertion or POST Complete de-assertion → check for
     * warm reset
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
     * - Handled SIO S5 de-assertion → start PS power OK watchdog, transition to
     * waitForPSPowerOK
     * - Handled SIO Power Good assertion → transition to on
     * - Handled power button press → start PS power OK watchdog, transition to
     * waitForPSPowerOK
     */
    virtual void handlePowerStateOff(Event event);

    /**
     * @brief Handler for PowerState::waitForPowerOK
     *
     * PREVIOUS IMPLEMENTATION (Upstream):
     * - Waited for Power OK assertion
     * - Handled watchdog timeout (power supply failed to come up)
     * - Transitioned to waitForSIOPowerGood or on depending on SIO
     * configuration
     */
    virtual void handleWaitForPowerOK(Event event);

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

    // POWER CONTROL OPERATIONS
    // referenced by upstream power state handlers

    /**
     * @brief Power on the system
     *
     * Asserts the PowerOut GPIO signal for the duration specified by
     * PowerPulseMs in TimerMap to initiate a power-on sequence.
     * Used by state handlers to turn on the system.
     */
    virtual void powerOn();

    /**
     * @brief Gracefully power off the system
     *
     * Asserts the PowerOut GPIO signal for the duration specified by
     * PowerPulseMs in TimerMap to initiate a graceful shutdown sequence.
     * This simulates a short power button press.
     */
    virtual void gracefulPowerOff();

    /**
     * @brief Force power off the system
     *
     * Asserts the PowerOut GPIO signal for the duration specified by
     * ForceOffPulseMs in TimerMap to force an immediate shutdown.
     * This simulates a long power button press (hard power off).
     * Sets up a timer to detect if the force-off fails.
     */
    virtual void forcePowerOff();

    /**
     * @brief Reset the system
     *
     * Asserts the ResetOut GPIO signal for the duration specified by
     * ResetPulseMs in TimerMap to perform a system reset.
     */
    virtual void reset();

    // OPERATING SYSTEM STATE MANAGEMENT

    /**
     * @brief Get the D-Bus string representation of an OS state
     *
     * @param stage The OS state stage
     * @return D-Bus interface string for the given stage
     */
    std::string_view getOperatingSystemStateStage(
        OperatingSystemStateStage stage) const;

    /**
     * @brief Set the operating system state
     *
     * Updates the OS state, D-Bus property, and resets ignoreNextSoftReset flag
     * when transitioning to Standby (POST complete).
     *
     * @param stage The new OS state stage
     */
    void setOperatingSystemState(OperatingSystemStateStage stage);

    /**
     * @brief Set all control GPIOs to their default state for host "on"
     *
     * Iterates through powerSignalMap and sets all output signals with
     * non-NA defaultStateHostStateOn to their configured state.
     * The default values must be configured via setDefaultValues() first.
     */
    void setGPIOsForHostStateOn();

    /**
     * @brief Set all control GPIOs to their default state for host "off"
     *
     * Iterates through powerSignalMap and sets all output signals with
     * non-NA defaultStateHostStateOff to their configured state.
     * The default values must be configured via setDefaultValues() first.
     */
    void setGPIOsForHostStateOff();

    // D-BUS PROPERTY MANAGEMENT
    // Functions for reading D-Bus properties and handling GPIO signals from
    // D-Bus

    /**
     * @brief Get D-Bus property value for a GPIO signal
     *
     * Reads a boolean D-Bus property as specified in the ConfigData.
     * On failure, schedules a retry via reschedulePropertyRead().
     *
     * @param configData Shared pointer to signal configuration with D-Bus
     * details
     * @return Property value (>0 if true, 0 if false, -1 on error)
     */
    int getProperty(std::shared_ptr<ConfigData> configData);

    /**
     * @brief Reschedule a failed D-Bus property read
     *
     * Creates/reuses a retry timer to attempt reading the D-Bus property again
     * after DbusGetPropertyRetry milliseconds. On success, calls
     * setInitialValue().
     *
     * @param configData Shared pointer to signal configuration to retry
     */
    void reschedulePropertyRead(std::shared_ptr<ConfigData> configData);

    /**
     * @brief Set initial value for a signal based on its name
     *
     * Updates D-Bus interface properties or internal state based on the signal
     * name:
     * - PowerButton/ResetButton/NMIButton/IdButton: Updates ButtonPressed
     * property
     * - PostComplete: Updates operating system state
     * - PowerOk: Sets power state and updates host state
     *
     * @param configData Shared pointer to signal configuration
     * @param initialValue The initial boolean value read from hardware/D-Bus
     */
    void setInitialValue(std::shared_ptr<ConfigData> configData,
                         bool initialValue);

    /**
     * @brief Initialize power state based on hardware GPIO indicators
     *
     * Reads the specified GPIO signals and sets the initial power state
     * based on their current values. Sets output GPIOs to match the detected
     * state. Should be called after registerGPIOHandlers() but before power
     * restore runs.
     *
     * This allows platforms to determine the actual hardware power state on
     * boot rather than assuming a default state. Useful for warm boots, BMC
     * reboots during host operation, or recovering from external power
     * changes.
     *
     * @param powerIndicatorSignals List of GPIO signal names to check for
     * power state
     * @param requireAllAsserted If true, ALL signals must be asserted for ON
     * state If false, ANY signal asserted means ON state (default: true)
     */
    void initializePowerStateFromHardware(
        const std::vector<std::string>& powerIndicatorSignals,
        bool requireAllAsserted = true);
};

} // namespace power_control
