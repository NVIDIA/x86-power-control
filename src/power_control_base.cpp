// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "power_control_base.hpp"

#include "power_restore.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <systemd/sd-journal.h>
#include <unistd.h>

#include <nlohmann/json.hpp>
#include <phosphor-logging/lg2.hpp>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <format>
#include <fstream>
#include <vector>

namespace power_control
{
// Type alias for convenience - Event is inside PowerControl class
using Event = PowerControl::Event;

// TODO: define virtual method
std::string PowerControl::getEventName(Event event)
{
    switch (event)
    {
        case Event::powerOKAssert:
            return "power supply power OK assert";
        case Event::powerOKDeAssert:
            return "power supply power OK de-assert";
        case Event::sioPowerGoodAssert:
            return "SIO power good assert";
        case Event::sioPowerGoodDeAssert:
            return "SIO power good de-assert";
        case Event::sioS5Assert:
            return "SIO S5 assert";
        case Event::sioS5DeAssert:
            return "SIO S5 de-assert";
        case Event::pltRstAssert:
            return "PLT_RST assert";
        case Event::pltRstDeAssert:
            return "PLT_RST de-assert";
        case Event::postCompleteAssert:
            return "POST Complete assert";
        case Event::postCompleteDeAssert:
            return "POST Complete de-assert";
        case Event::powerButtonPressed:
            return "power button pressed";
        case Event::resetButtonPressed:
            return "reset button pressed";
        case Event::powerCycleTimerExpired:
            return "power cycle timer expired";
        case Event::powerOKWatchdogTimerExpired:
            return "power supply power OK watchdog timer expired";
        case Event::pdbMainPowerOkWatchdogTimerExpired:
            return "PDB main power OK watchdog timer expired";
        case Event::hpmPowerGoodWatchdogTimerExpired:
            return "HPM power good watchdog timer expired";
        case Event::cpuResetWatchdogTimerExpired:
            return "CPU reset watchdog timer expired";
        case Event::cpuShutdownOkWatchdogTimerExpired:
            return "CPU shutdown OK watchdog timer expired";
        case Event::cpuBootDoneDeAssertWatchdogTimerExpired:
            return "CPU Boot Done de-assert watchdog timer expired";
        case Event::sioPowerGoodWatchdogTimerExpired:
            return "SIO power good watchdog timer expired";
        case Event::gracefulPowerOffTimerExpired:
            return "graceful power-off timer expired";
        case Event::powerOnRequest:
            return "power-on request";
        case Event::powerOffRequest:
            return "power-off request";
        case Event::powerCycleRequest:
            return "power-cycle request";
        case Event::resetRequest:
            return "reset request";
        case Event::gracefulResetRequest:
            return "graceful reset request";
        case Event::gracefulPowerOffRequest:
            return "graceful power-off request";
        case Event::gracefulPowerCycleRequest:
            return "graceful power-cycle request";
        case Event::warmResetDetected:
            return "warm reset detected";
        case Event::powerCycleDelayTimerExpired:
            return "power cycle delay timer expired";
        case Event::warmRebootDelayTimerExpired:
            return "warm reboot delay timer expired";
        case Event::pdbMainPowerOkAssert:
            return "PDB main power OK assert";
        case Event::pdbMainPowerOkDeAssert:
            return "PDB main power OK de-assert";
        case Event::gb300pdbMainPowerOkAssert:
            return "GB300 PDB main power OK assert";
        case Event::gb300pdbMainPowerOkDeAssert:
            return "GB300 PDB main power OK de-assert";
        case Event::c2pdbPSUPowerOkAssert:
            return "C2 PDB main power OK assert";
        case Event::c2pdbPSUPowerOkDeAssert:
            return "C2 PDB main power OK de-assert";
        case Event::board0RunPowerPGAssert:
            return "Board 0 run power PG assert";
        case Event::board0RunPowerPGDeAssert:
            return "Board 0 run power PG de-assert";
        case Event::board1RunPowerPGAssert:
            return "Board 1 run power PG assert";
        case Event::board1RunPowerPGDeAssert:
            return "Board 1 run power PG de-assert";
        case Event::cpuResetIndicatorAssert:
            return "CPU reset indicator assert";
        case Event::cpuResetIndicatorDeAssert:
            return "CPU reset indicator de-assert";
        case Event::board0CpuShutdownOkAssert:
            return "Board 0 CPU shutdown OK assert";
        case Event::board0CpuShutdownOkDeAssert:
            return "Board 0 CPU shutdown OK de-assert";
        case Event::board1CpuShutdownOkAssert:
            return "Board 1 CPU shutdown OK assert";
        case Event::board1CpuShutdownOkDeAssert:
            return "Board 1 CPU shutdown OK de-assert";
        case Event::cpuBootDoneAssert:
            return "CPU Boot Done assert";
        case Event::cpuBootDoneDeAssert:
            return "CPU Boot Done de-assert";
        default:
            return "unknown event: " + std::to_string(static_cast<int>(event));
    }
}

void PowerControl::logResourceEvent(
    const std::string& eventName,
    std::initializer_list<std::string> messageArgs, std::string_view severity)
{
    if (messageArgs.size() == 0)
    {
        lg2::error("logResourceEvent: messageArgs is empty");
        return;
    }

    auto method = conn->new_method_call(
        "xyz.openbmc_project.Logging", "/xyz/openbmc_project/logging",
        "xyz.openbmc_project.Logging.Create", "Create");

    std::map<std::string, std::string> additionalData;
    additionalData["REDFISH_MESSAGE_ID"] =
        std::format("ResourceEvent.1.0.{}", eventName);

    std::string argsStr;
    auto it = messageArgs.begin();
    argsStr += *it;
    for (++it; it != messageArgs.end(); ++it)
    {
        argsStr += ',';
        argsStr += *it;
    }
    additionalData["REDFISH_MESSAGE_ARGS"] = argsStr;

    method.append(eventName, severity, additionalData);
    conn->call(method);
}

void PowerControl::logEvent(std::string_view stateHandler, Event event)
{
    lg2::info("{STATE_HANDLER}: {EVENT} event received", "STATE_HANDLER",
              stateHandler, "EVENT", getEventName(event));
}

int PowerControl::i2cWrite(int file, uint16_t address,
                           const std::vector<uint8_t>& data)
{
    if (data.empty())
    {
        return -1;
    }

    struct i2c_msg msg{};
    struct i2c_rdwr_ioctl_data rdwr{};

    msg.addr = address;
    msg.flags = 0;
    msg.len = static_cast<__u16>(data.size());
    msg.buf = const_cast<uint8_t*>(data.data());

    rdwr.msgs = &msg;
    rdwr.nmsgs = 1;

    int ret = ioctl(file, I2C_RDWR, &rdwr);
    return (ret == 1) ? 0 : -1;
}

PowerControl::PowerControl(boost::asio::io_context& ioContext,
                           std::shared_ptr<sdbusplus::asio::connection> conn,
                           const std::string& node, PersistentState& appState,
                           const std::string& configFilePath) :
    ioContext(ioContext), conn(conn), objServer(conn), nodeId(node),
    appState(appState), appName("power-control"),
    configFilePath(
        configFilePath.empty()
            ? "/usr/share/x86-power-control/power-config-host" + node + ".json"
            : configFilePath),
    // Timers
    gpioAssertTimer(ioContext), powerCycleTimer(ioContext),
    gracefulPowerOffTimer(ioContext), warmResetCheckTimer(ioContext),
    powerOKWatchdogTimer(ioContext), sioPowerGoodWatchdogTimer(ioContext),
    powerStateSaveTimer(ioContext), pohCounterTimer(ioContext),
    restartCauseTimer(ioContext), slotPowerCycleTimer(ioContext)
{
    // Load configuration from JSON file and populate powerSignalMap
    loadConfigValues();

    // Register base class GPIO handlers
    // These handlers are available in all platforms and can be overridden by
    // derived classes

    // We do not currently use the below GPIOs, so let's not enforce them in the
    // base class GpioName: Handler function PowerOk: powerOKHandler
    // SioPowerGood: sioPowerGoodHandler
    // SIOS5: sioS5Handler
    // PowerButton: powerButtonHandler
    // ResetButton: resetButtonHandler

    // Initialize ObjectManager BEFORE claiming service names
    // Ensuring the Mapper daemon can track all child objects via
    // InterfacesAdded signals
    initializeObjectManager();

    // DON'T claim bus names yet! Must wait until all interfaces are
    // initialized. Names will be claimed by derived class after all
    // initialization is complete.

    // Initialize D-Bus interfaces
    registerHostInterface();
    initializeChassisInterface();
    initializeBootProgressInterface();
#ifdef CHASSIS_SYSTEM_RESET
    initializeChassisSystemInterface();
#endif
    initializeButtonInterfaces();
    initializeOSInterface();
    registerGpioStateInterface();

    // Initialize GPIO property setters map with common Board0 signals
    // Derived classes can add additional signals (e.g., Board1 signals)
    gpioPropertySetters = {
        {"CpuResetIndicator",
         [](PowerControl* pc, int val) { pc->setCpuResetIndicatorState(val); }},
        {"Board0RunPowerPG",
         [](PowerControl* pc, int val) { pc->setBoard0RunPowerPGState(val); }},
        {"Board0CpuShutdownOk", [](PowerControl* pc, int val) {
             pc->setBoard0CpuShutdownOkState(val);
         }}};

    // Note: initializeHostStateInterface() is called by derived classes
    // (e.g., NVL72PowerControl) after they register any additional GPIO
    // properties. It initializes ALL host0 interfaces at once.
}

std::function<void(Event)> PowerControl::getPowerStateHandler()
{
    // Map upstream PowerState values to their handler functions
    switch (powerState)
    {
        case PowerState::on:
            return [this](Event e) { this->handlePowerStateOn(e); };

        case PowerState::waitForPowerOK:
            return [this](Event e) { this->handleWaitForPowerOK(e); };

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
            return [this](Event e) {
                this->handleGracefulTransitionToCycleOff(e);
            };

        case PowerState::checkForWarmReset:
            return [this](Event e) { this->handleCheckForWarmReset(e); };

        // Unknown state - not an upstream state
        // check for nul pointer return
        default:
            return nullptr;
    }
}

void PowerControl::sendPowerControlEvent(Event event)
{
    // Use the virtual getPowerStateHandler to get the correct handler
    std::function<void(Event)> handler = this->getPowerStateHandler();

    if (handler == nullptr)
    {
        lg2::error("Failed to find handler for power state: {STATE}", "STATE",
                   static_cast<int>(powerState));
        return;
    }

    // Execute the handler (will use virtual dispatch)
    handler(event);
}

void PowerControl::loadConfigValues()
{
    // Dynamically build powerSignalMap from JSON config file

    // Determine config file path
    std::ifstream configFile(configFilePath.c_str());
    if (!configFile.is_open())
    {
        lg2::error("loadConfigValues: Cannot open config path \'{PATH}\'",
                   "PATH", configFilePath);
        throw std::runtime_error(
            "Failed to open config file: " + configFilePath);
    }

    auto jsonData = nlohmann::json::parse(configFile, nullptr, true, true);
    if (jsonData.is_discarded())
    {
        lg2::error("Power config readings JSON parser failure");
        throw std::runtime_error("JSON parser failure");
    }

    auto gpios = jsonData["gpio_configs"];

    // Dynamically create ConfigData entries from JSON
    for (nlohmann::json& gpioConfig : gpios)
    {
        if (!gpioConfig.contains("Name"))
        {
            lg2::error("The 'Name' field must be defined in Json file");
            throw std::runtime_error("Missing 'Name' field in JSON config");
        }

        std::string gpioName = gpioConfig["Name"];

        // Create a new ConfigData object (dynamically allocated)
        auto configPtr = std::make_shared<ConfigData>(ioContext);
        configPtr->name = gpioName;

        // Parse Type
        if (!gpioConfig.contains("Type"))
        {
            lg2::error("The \'Type\' field must be defined in Json file");
            throw std::runtime_error(
                "Missing 'Type' field for signal: " + gpioName);
        }

        std::string signalType = gpioConfig["Type"];
        if (signalType == "GPIO")
        {
            configPtr->type = ConfigType::GPIO;
        }
        else if (signalType == "DBUS")
        {
            configPtr->type = ConfigType::DBUS;
        }
        else
        {
            lg2::error("{TYPE} is not a recognized power-control signal type",
                       "TYPE", signalType);
            throw std::runtime_error("Invalid signal type: " + signalType);
        }

        // Parse GPIO-specific fields
        if (configPtr->type == ConfigType::GPIO)
        {
            if (gpioConfig.contains("LineName"))
            {
                configPtr->lineName = gpioConfig["LineName"];
            }
            else
            {
                lg2::error(
                    "The \'LineName\' field must be defined for GPIO configuration");
                throw std::runtime_error(
                    "Missing 'LineName' for GPIO: " + gpioName);
            }

            if (gpioConfig.contains("Polarity"))
            {
                std::string polarity = gpioConfig["Polarity"];
                if (polarity == "ActiveLow")
                {
                    configPtr->polarity = false;
                }
                else if (polarity == "ActiveHigh")
                {
                    configPtr->polarity = true;
                }
                else
                {
                    lg2::error(
                        "Polarity defined but not properly setup. Please only ActiveHigh or ActiveLow. Currently set to {POLARITY}",
                        "POLARITY", polarity);
                    throw std::runtime_error(
                        "Invalid polarity for: " + gpioName);
                }
            }
            else
            {
                lg2::error("Polarity field not found for {GPIO_NAME}",
                           "GPIO_NAME", configPtr->lineName);
                throw std::runtime_error(
                    "Missing 'Polarity' for GPIO: " + gpioName);
            }
        }
        else // DBUS type
        {
            // Parse D-Bus specific fields
            std::map<std::string, std::string> dbusParams = {
                {"DbusName", "DbusName"},
                {"Path", "Path"},
                {"Interface", "Interface"},
                {"Property", "Property"}};

            for (auto& [key, dbusParamName] : dbusParams)
            {
                if (!gpioConfig.contains(dbusParamName))
                {
                    lg2::error(
                        "The {DBUS_NAME} field must be defined for Dbus configuration ",
                        "DBUS_NAME", dbusParamName);
                    throw std::runtime_error(
                        "Missing D-Bus field: " + dbusParamName);
                }
            }

            configPtr->dbusName = gpioConfig["DbusName"];
            configPtr->path = gpioConfig["Path"];
            configPtr->interface = gpioConfig["Interface"];
            configPtr->lineName =
                gpioConfig["Property"]; // Property name stored in lineName

            // dbus-based inputs must be active-high
            configPtr->polarity = true;

            // MatchRegex is optional
            auto item = gpioConfig.find("MatchRegex");
            if (item != gpioConfig.end())
            {
                try
                {
                    configPtr->matchRegex = std::regex(*item);
                }
                catch (const std::regex_error& e)
                {
                    lg2::error("Invalid MatchRegex for {NAME}: {ERR}", "NAME",
                               gpioName, "ERR", e.what());
                    throw std::runtime_error(
                        "Invalid MatchRegex for: " + gpioName);
                }
            }
        }

        // Add to powerSignalMap
        powerSignalMap[gpioName] = configPtr;
    }

    lg2::info("Successfully loaded {COUNT} signal configurations from JSON",
              "COUNT", powerSignalMap.size());

    // Load timer values from JSON config
    if (jsonData.contains("timing_configs"))
    {
        auto timers = jsonData["timing_configs"];
        if (timers.is_object())
        {
            for (auto& [key, value] : timers.items())
            {
                if (value.is_number_integer())
                {
                    TimerMap[key] = value.get<int>();
                }
                else
                {
                    lg2::warning(
                        "Timer '{TIMER}' has non-integer value, skipping",
                        "TIMER", key);
                }
            }
            lg2::info(
                "Successfully loaded {COUNT} timer configurations from JSON",
                "COUNT", TimerMap.size());
        }
        else
        {
            lg2::warning(
                "'timing_configs' field in JSON is not an object, skipping timer loading");
        }
    }
    else
    {
        lg2::info(
            "No 'timing_configs' field found in JSON config, TimerMap will be empty");
    }
}

bool PowerControl::requestGPIOEvents(ConfigData& config)
{
    // Migrated from static function in power_control.cpp

    // Find the GPIO line
    config.gpioLine = gpiod::find_line(config.lineName);
    if (!config.gpioLine)
    {
        lg2::error("Failed to find the {GPIO_NAME} line", "GPIO_NAME",
                   config.lineName);
        return false;
    }

    try
    {
        config.gpioLine.request(
            {appName, gpiod::line_request::EVENT_BOTH_EDGES, {}});
    }
    catch (const std::exception& e)
    {
        lg2::error("Failed to request events for {GPIO_NAME}: {ERROR}",
                   "GPIO_NAME", config.lineName, "ERROR", e);
        return false;
    }

    int gpioLineFd = config.gpioLine.event_get_fd();
    if (gpioLineFd < 0)
    {
        lg2::error("Failed to get {GPIO_NAME} fd", "GPIO_NAME",
                   config.lineName);
        return false;
    }

    config.eventDescriptor.assign(gpioLineFd);

    // Initialize D-Bus GPIO property with current hardware value (if mapped)
    auto setterIt = gpioPropertySetters.find(config.name);
    if (setterIt != gpioPropertySetters.end())
    {
        int currentValue = config.gpioLine.get_value();
        setterIt->second(this, currentValue);
        lg2::info("Initialized D-Bus property '{SIGNAL}' to {VALUE}", "SIGNAL",
                  config.name, "VALUE", currentValue);
    }

    waitForGPIOEvent(config);
    return true;
}

void PowerControl::waitForGPIOEvent(ConfigData& config)
{
    // Migrated from static function in power_control.cpp

    config.eventDescriptor.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [this, &config](const boost::system::error_code ec) {
            if (ec)
            {
                lg2::error("{GPIO_NAME} fd handler error: {ERROR_MSG}",
                           "GPIO_NAME", config.name, "ERROR_MSG", ec.message());
                // TODO: throw here to force power-control service to exit/fail?
                return;
            }
            gpiod::line_event line_event = config.gpioLine.event_read();
            bool gpioState =
                (line_event.event_type == gpiod::line_event::RISING_EDGE);

            // Call the handler if it's been assigned
            if (config.gpioHandler)
            {
                config.gpioHandler(gpioState);
            }

            // Recursively wait for next event
            waitForGPIOEvent(config);
        });
}

void PowerControl::handlePowerStateOn(Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::powerOKDeAssert:
            setPowerState(PowerState::off);
            // DC power is unexpectedly lost, beep
            beep(beepPowerFail);
            break;
        case Event::sioS5Assert:
            setPowerState(PowerState::transitionToOff);
#if IGNORE_SOFT_RESETS_DURING_POST
            // Only recognize soft resets once host gets past POST COMPLETE
            if (operatingSystemState != OperatingSystemStateStage::Standby)
            {
                ignoreNextSoftReset = true;
            }
#endif
            addRestartCause(RestartCause::softReset);
            break;
#if USE_PLT_RST
        case Event::pltRstAssert:
#else
        case Event::postCompleteDeAssert:
#endif
            setPowerState(PowerState::checkForWarmReset);
#if IGNORE_SOFT_RESETS_DURING_POST
            // Only recognize soft resets once host gets past POST COMPLETE
            if (operatingSystemState != OperatingSystemStateStage::Standby)
            {
                ignoreNextSoftReset = true;
            }
#endif
            addRestartCause(RestartCause::softReset);
            startTimer("WarmResetCheckMs", warmResetCheckTimer,
                       Event::warmResetDetected);
            break;
        case Event::powerButtonPressed:
            setPowerState(PowerState::gracefulTransitionToOff);
            startTimer("GracefulPowerOffS", gracefulPowerOffTimer,
                       Event::gracefulPowerOffTimerExpired);
            break;
        case Event::powerOffRequest:
            setPowerState(PowerState::transitionToOff);
            forcePowerOff();
            break;
        case Event::gracefulPowerOffRequest:
            setPowerState(PowerState::gracefulTransitionToOff);
            startTimer("GracefulPowerOffS", gracefulPowerOffTimer,
                       Event::gracefulPowerOffTimerExpired);
            gracefulPowerOff();
            break;
        case Event::powerCycleRequest:
            setPowerState(PowerState::transitionToCycleOff);
            forcePowerOff();
            break;
        case Event::gracefulPowerCycleRequest:
            setPowerState(PowerState::gracefulTransitionToCycleOff);
            startTimer("GracefulPowerOffS", gracefulPowerOffTimer,
                       Event::gracefulPowerOffTimerExpired);
            gracefulPowerOff();
            break;
        case Event::resetButtonPressed:
            setPowerState(PowerState::checkForWarmReset);
            startTimer("WarmResetCheckMs", warmResetCheckTimer,
                       Event::warmResetDetected);
            break;
        case Event::resetRequest:
            reset();
            break;
        default:
            lg2::info("No action taken for event: {EVENT}", "EVENT",
                      getEventName(event));
            break;
    }
}

void PowerControl::handlePowerStateOff(Event event)
{
    (void)event;
    // TODO: Move upstream powerStateOff() implementation here
    //
    // PREVIOUS IMPLEMENTATION (from powerStateOff in power_control.cpp -
    // UPSTREAM ONLY):
    // - logEvent(__FUNCTION__, event);
    // - switch (event):
    //     case Event::psPowerOKAssert:
    //         - if (sioEnabled) →
    //         setPowerState(PowerState::waitForSIOPowerGood)
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

void PowerControl::handleWaitForPowerOK(Event event)
{
    (void)event;
    // TODO: Move upstream powerStateWaitForPSPowerOK() implementation here
    //
    // PREVIOUS IMPLEMENTATION:
    // - logEvent(__FUNCTION__, event);
    // - switch (event):
    //     case Event::psPowerOKAssert:
    //         - psPowerOKWatchdogTimer.cancel();
    //         - if (sioEnabled) → sioPowerGoodWatchdogTimerStart(),
    //         setPowerState(PowerState::waitForSIOPowerGood)
    //         - else → setPowerState(PowerState::on)
    //     case Event::psPowerOKWatchdogTimerExpired:
    //         - setPowerState(PowerState::off);
    //         - lg2::error("PS_PWROK signal did not assert within timeout");
}

void PowerControl::handleWaitForSIOPowerGood(Event event)
{
    (void)event;
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
    //         - lg2::error("SIO Power Good signal did not assert within
    //         timeout");
}

void PowerControl::handleTransitionToOff(Event event)
{
    (void)event;
    // TODO: Move upstream powerStateTransitionToOff() implementation here
    //
    // PREVIOUS IMPLEMENTATION:
    // - Forcefully de-assert all power control signals
    // - Transition to off state
}

void PowerControl::handleGracefulTransitionToOff(Event event)
{
    (void)event;
    // TODO: Move upstream powerStateGracefulTransitionToOff() implementation
    // here
    //
    // PREVIOUS IMPLEMENTATION:
    // - Assert graceful shutdown request
    // - Start graceful power off timer
    // - If host shuts down gracefully (sioS5Assert) → transition to off
    // - If timer expires → forceful shutdown → transition to off
}

void PowerControl::handleCycleOff(Event event)
{
    (void)event;
    // TODO: Move upstream powerStateCycleOff() implementation here
    //
    // PREVIOUS IMPLEMENTATION:
    // - Wait in powered off state for power cycle timer
    // - When timer expires → transition to power on (waitForPSPowerOK)
}

void PowerControl::handleTransitionToCycleOff(Event event)
{
    (void)event;
    // TODO: Move upstream powerStateTransitionToCycleOff() implementation here
    //
    // PREVIOUS IMPLEMENTATION:
    // - Forcefully de-assert all power control signals
    // - Start power cycle timer
    // - Transition to cycleOff state
}

void PowerControl::handleGracefulTransitionToCycleOff(Event event)
{
    (void)event;
    // TODO: Move upstream powerStateGracefulTransitionToCycleOff()
    // implementation here
    //
    // PREVIOUS IMPLEMENTATION:
    // - Assert graceful shutdown request
    // - Start graceful power off timer
    // - If host shuts down gracefully → transition to cycleOff
    // - If timer expires → forceful shutdown → transition to cycleOff
}

void PowerControl::handleCheckForWarmReset(Event event)
{
    (void)event;
    // TODO: Move upstream powerStateCheckForWarmReset() implementation here
    //
    // PREVIOUS IMPLEMENTATION:
    // - Monitor PLT_RST or POST Complete signals
    // - If PLT_RST de-asserts (warm reset) → transition back to on
    // - If power off detected → transition to transitionToOff
}

std::string_view PowerControl::getHostState() const
{
    // Upstream implementation - maps PowerState to D-Bus host state
    switch (powerState)
    {
        case PowerState::on:
        case PowerState::gracefulTransitionToOff:
        case PowerState::gracefulTransitionToCycleOff:
            return "xyz.openbmc_project.State.Host.HostState.Running";
            break;
        case PowerState::waitForPowerOK:
        case PowerState::waitForSIOPowerGood:
        case PowerState::off:
        case PowerState::transitionToOff:
        case PowerState::transitionToCycleOff:
        case PowerState::cycleOff:
        case PowerState::checkForWarmReset:
            return "xyz.openbmc_project.State.Host.HostState.Off";
            break;
        default:
            return "";
            break;
    }
}

std::string_view PowerControl::getChassisState() const
{
    // Upstream implementation - maps PowerState to D-Bus chassis state
    switch (powerState)
    {
        case PowerState::on:
        case PowerState::transitionToOff:
        case PowerState::gracefulTransitionToOff:
        case PowerState::transitionToCycleOff:
        case PowerState::gracefulTransitionToCycleOff:
        case PowerState::checkForWarmReset:
            return "xyz.openbmc_project.State.Chassis.PowerState.On";
            break;
        case PowerState::waitForPowerOK:
        case PowerState::waitForSIOPowerGood:
        case PowerState::off:
        case PowerState::cycleOff:
            return "xyz.openbmc_project.State.Chassis.PowerState.Off";
            break;
        default:
            return "";
            break;
    }
}

std::string PowerControl::getPowerStateName()
{
    // Upstream implementation - only knows about upstream power states
    switch (powerState)
    {
        case PowerState::on:
            return "On";
            break;
        case PowerState::waitForPowerOK:
            return "Wait for Power OK";
            break;
        case PowerState::waitForSIOPowerGood:
            return "Wait for SIO Power Good";
            break;
        case PowerState::off:
            return "Off";
            break;
        case PowerState::transitionToOff:
            return "Transition to Off";
            break;
        case PowerState::gracefulTransitionToOff:
            return "Graceful Transition to Off";
            break;
        case PowerState::cycleOff:
            return "Power Cycle Off";
            break;
        case PowerState::transitionToCycleOff:
            return "Transition to Power Cycle Off";
            break;
        case PowerState::gracefulTransitionToCycleOff:
            return "Graceful Transition to Power Cycle Off";
            break;
        case PowerState::checkForWarmReset:
            return "Check for Warm Reset";
            break;
        default:
            return "unknown state: " +
                   std::to_string(static_cast<int>(powerState));
            break;
    }
}

void PowerControl::logStateTransition()
{
    lg2::info("Host{HOST}: Moving to \"{STATE}\" state", "HOST", nodeId,
              "STATE", this->getPowerStateName());
}

void PowerControl::setBootProgress(const std::string& bootProgressStage)
{
    if (bootProgressIface)
    {
        bootProgressIface->set_property("BootProgress", bootProgressStage);

        // Update timestamp
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                             now.time_since_epoch())
                             .count();
        bootProgressIface->set_property("BootProgressLastUpdate",
                                        static_cast<uint64_t>(timestamp));

        lg2::info("Boot progress updated to: {PROGRESS}", "PROGRESS",
                  bootProgressStage);
    }
}

void PowerControl::setBootProgressOem(const std::string& oemProgress)
{
    if (bootProgressIface)
    {
        bootProgressIface->set_property("BootProgressOem", oemProgress);
        lg2::info("Boot progress OEM updated to: {OEM_PROGRESS}",
                  "OEM_PROGRESS", oemProgress);
    }
}

uint64_t PowerControl::getCurrentTimeMs()
{
    struct timespec time = {};

    if (clock_gettime(CLOCK_REALTIME, &time) < 0)
    {
        return 0;
    }
    uint64_t currentTimeMs = static_cast<uint64_t>(time.tv_sec) * 1000;
    currentTimeMs += static_cast<uint64_t>(time.tv_nsec) / 1000 / 1000;

    return currentTimeMs;
}

void PowerControl::setPowerState(const PowerState state)
{
    // Note: This function still references the external global powerState
    // variable which will need to be refactored in the future.

    // Update global power state
    powerState = state;
    logStateTransition();

    // Update D-Bus host state (uses virtual dispatch)
    hostIface->set_property("CurrentHostState",
                            std::string(this->getHostState()));

    // Update D-Bus chassis state (uses virtual dispatch)
    chassisIface->set_property("CurrentPowerState",
                               std::string(this->getChassisState()));
    chassisIface->set_property("LastStateChangeTime", getCurrentTimeMs());

    // Reset boot progress to Unspecified when host powers off
    if (state == PowerState::off)
    {
        setBootProgress(
            "xyz.openbmc_project.State.Boot.Progress.ProgressStages.Unspecified");
        logResourceEvent(
            "ResourcePoweredOff", {"Host0"},
            "xyz.openbmc_project.Logging.Entry.Level.Informational");
    }
    else if (state == PowerState::on)
    {
        logResourceEvent(
            "ResourcePoweredOn", {"Host0"},
            "xyz.openbmc_project.Logging.Entry.Level.Informational");
    }

    // Save the power state for the restore policy
    savePowerState(state);
}

void PowerControl::savePowerState(const PowerState state)
{
    auto it = TimerMap.find("PowerOffSaveMs");
    if (it == TimerMap.end())
    {
        lg2::error("Timer config 'PowerOffSaveMs' not found in TimerMap");
        return;
    }
    powerStateSaveTimer.expires_after(std::chrono::milliseconds(it->second));
    powerStateSaveTimer.async_wait([this,
                                    state](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error("Power-state save async_wait failed: {ERROR_MSG}",
                           "ERROR_MSG", ec.message());
            }
            return;
        }
        appState.set(PersistentState::Params::PowerState,
                     std::string{getChassisState()});
    });
}

void PowerControl::initializeObjectManager()
{
    // Create ObjectManager on the parent path /xyz/openbmc_project/state
    // MUST be done BEFORE claiming service names to ensure proper
    // Mapper synchronization
    //
    // When ObjectManager is present on a parent path, sdbusplus automatically
    // emits InterfacesAdded signals when child interfaces (like
    // /xyz/openbmc_project/state/host0) are initialized. The Mapper daemon
    // subscribes to these signals and updates its cache in real-time.
    //
    // Without ObjectManager, "mapper wait" will hang even though the
    // interface exists.

    // Use the member objServer (which persists for daemon lifetime)
    // to ensure ObjectManager stays alive
    objServer.add_manager("/xyz/openbmc_project/state");

    lg2::info("ObjectManager interface created on /xyz/openbmc_project/state");
}

void PowerControl::requestBusNames()
{
    lg2::info("Claiming D-Bus service names...");

    // Claim primary names
    if (nodeId == "0")
    {
        // For backwards compatibility, claim the primary names
        conn->request_name(hostDbusName.c_str());
        conn->request_name(chassisDbusName.c_str());
        conn->request_name(osDbusName.c_str());
        conn->request_name(nmiDbusName.c_str());
    }

    // Append the node ID to the dbus names & Request all the dbus names
    conn->request_name((hostDbusName + nodeId).c_str());
    conn->request_name((chassisDbusName + nodeId).c_str());
    conn->request_name((osDbusName + nodeId).c_str());
    conn->request_name((nmiDbusName + nodeId).c_str());

    // Only claim buttons name if we created button interfaces
    // (avoid conflict with separate buttons daemon)
    if (powerButtonIface || resetButtonIface || nmiButtonIface || idButtonIface)
    {
        lg2::info("Claiming Buttons service name (button interfaces present)");
        conn->request_name(buttonDbusName.c_str());
    }
    else
    {
        lg2::info("Skipping Buttons service name (no button interfaces)");
    }

    lg2::info("D-Bus service names claimed successfully");
}

void PowerControl::registerHostInterface()
{
    // Note: Button masking (powerButtonMask, resetButtonMask) and restart cause
    // tracking (addRestartCause) are not yet moved to the class, so those
    // checks are commented out.

    // Create Host Interface
    hostIface =
        objServer.add_interface("/xyz/openbmc_project/state/host" + nodeId,
                                "xyz.openbmc_project.State.Host");

    // Interface for IPMI/Redfish initiated host state transitions
    hostIface->register_property(
        "RequestedHostTransition",
        std::string("xyz.openbmc_project.State.Host.Transition.Off"),
        [this](const std::string& requested, std::string& resp) {
            // Note: Button masking and restart cause tracking not yet
            // implemented
            // TODO: Uncomment when powerButtonMask, resetButtonMask, and
            // addRestartCause are moved

            if (requested == "xyz.openbmc_project.State.Host.Transition.Off")
            {
                // TODO: Check power button mask when implemented
                if (!powerButtonMask)
                {
                    addRestartCause(RestartCause::command);
                    lg2::info("Host transition to Off requested");
                    // Defer event processing to avoid D-Bus reentrancy
                    boost::asio::post(ioContext, [this]() {
                        sendPowerControlEvent(Event::gracefulPowerOffRequest);
                    });
                }
                else
                {
                    lg2::warning("Power Button Masked.");
                    return 0;
                }
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Host.Transition.On")
            {
                // TODO: Check power button mask when implemented
                if (!powerButtonMask)
                {
                    lg2::info("Host transition to On requested");
                    addRestartCause(RestartCause::command);
                    // Defer event processing to avoid D-Bus reentrancy
                    boost::asio::post(ioContext, [this]() {
                        sendPowerControlEvent(Event::powerOnRequest);
                    });
                }
                else
                {
                    lg2::warning("Power Button Masked.");
                    return 0;
                }
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Host.Transition.Reboot")
            {
                // TODO: Check power button mask when implemented
                if (!powerButtonMask)
                {
                    lg2::info("Host transition to Reboot requested");
                    addRestartCause(RestartCause::command);
                    // Defer event processing to avoid D-Bus reentrancy
                    boost::asio::post(ioContext, [this]() {
                        sendPowerControlEvent(Event::powerCycleRequest);
                    });
                }
                else
                {
                    lg2::warning("Power Button Masked.");
                    return 0;
                }
            }
            else if (
                requested ==
                "xyz.openbmc_project.State.Host.Transition.GracefulWarmReboot")
            {
                // TODO: Check reset button mask when implemented
                if (!resetButtonMask)
                {
                    addRestartCause(RestartCause::command);
                    lg2::info(
                        "Host transition to GracefulWarmReboot requested");
                    // Defer event processing to avoid D-Bus reentrancy
                    boost::asio::post(ioContext, [this]() {
                        sendPowerControlEvent(Event::gracefulResetRequest);
                    });
                }
                else
                {
                    lg2::warning("Reset Button Masked.");
                    return 0;
                }
            }
            else if (
                requested ==
                "xyz.openbmc_project.State.Host.Transition.ForceWarmReboot")
            {
                // TODO: Check reset button mask when implemented
                if (!resetButtonMask)
                {
                    lg2::info("Host transition to ForceWarmReboot requested");
                    addRestartCause(RestartCause::command);
                    // Defer event processing to avoid D-Bus reentrancy
                    boost::asio::post(ioContext, [this]() {
                        sendPowerControlEvent(Event::resetRequest);
                    });
                }
                else
                {
                    lg2::warning("Reset Button Masked.");
                    return 0;
                }
            }
            else
            {
                lg2::error("Unrecognized host state transition request.");
                return 0;
            }
            resp = requested;
            return 1;
        });

    hostIface->register_property("CurrentHostState",
                                 std::string(getHostState()));

    // RestartCause and RequestedRestartCause on State.Host for parity with
    // PSM/legacy; path is /xyz/openbmc_project/state/host{nodeId}
    hostIface->register_property(
        "RestartCause",
        std::string("xyz.openbmc_project.State.Host.RestartCause.Unknown"));

    hostIface->register_property(
        "RequestedRestartCause",
        std::string("xyz.openbmc_project.State.Host.RestartCause.Unknown"),
        [this](const std::string& requested, std::string& resp) {
            if (requested ==
                "xyz.openbmc_project.State.Host.RestartCause.WatchdogTimer")
            {
                addRestartCause(RestartCause::watchdog);
                lg2::info("Restart cause watchdog requested");
            }
            else
            {
                lg2::error("Unrecognized RestartCause Request");
                return 0;
            }

            lg2::info("RestartCause requested: {RESTART_CAUSE}",
                      "RESTART_CAUSE", requested);
            resp = requested;
            return 1;
        });

    // NOTE: Do NOT call hostIface->initialize() here!
    // The Host interface should be initialized LAST (after all other interfaces
    // including Gpio) so that when the path /xyz/openbmc_project/state/host0
    // becomes visible, ALL interfaces are ready.
    // Call initializeHostStateInterface() from the most-derived class
    // constructor.

    lg2::info("Host interface registered (not yet initialized)");
}

void PowerControl::initializeChassisInterface()
{
    // Create Chassis Interface
    chassisIface =
        objServer.add_interface("/xyz/openbmc_project/state/chassis" + nodeId,
                                "xyz.openbmc_project.State.Chassis");

    chassisIface->register_property(
        "RequestedPowerTransition",
        std::string("xyz.openbmc_project.State.Chassis.Transition.Off"),
        [this](const std::string& requested, std::string& resp) {
            // Note: Button masking and restart cause tracking not yet
            // implemented
            // TODO: Uncomment when powerButtonMask and addRestartCause are
            // moved

            if (requested == "xyz.openbmc_project.State.Chassis.Transition.Off")
            {
                // TODO: Check power button mask when implemented
                if (!powerButtonMask)
                {
                    lg2::info("Chassis transition to Off requested");
                    addRestartCause(RestartCause::command);
                    // Defer event processing to avoid D-Bus reentrancy
                    boost::asio::post(ioContext, [this]() {
                        sendPowerControlEvent(Event::powerOffRequest);
                    });
                }
                else
                {
                    lg2::warning("Power Button Masked.");
                    return 0;
                }
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Chassis.Transition.On")
            {
                // TODO: Check power button mask when implemented
                if (!powerButtonMask)
                {
                    lg2::info("Chassis transition to On requested");
                    addRestartCause(RestartCause::command);
                    // Defer event processing to avoid D-Bus reentrancy
                    boost::asio::post(ioContext, [this]() {
                        sendPowerControlEvent(Event::powerOnRequest);
                    });
                }
                else
                {
                    lg2::warning("Power Button Masked.");
                    return 0;
                }
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Chassis.Transition.PowerCycle")
            {
                // TODO: Check power button mask when implemented
                if (!powerButtonMask)
                {
                    lg2::info("Chassis transition to PowerCycle requested");
                    addRestartCause(RestartCause::command);
                    // Defer event processing to avoid D-Bus reentrancy
                    boost::asio::post(ioContext, [this]() {
                        sendPowerControlEvent(Event::powerCycleRequest);
                    });
                }
                else
                {
                    lg2::warning("Power Button Masked.");
                    return 0;
                }
            }
            else
            {
                lg2::error("Unrecognized chassis state transition request.");
                return 0;
            }
            resp = requested;
            return 1;
        });

    chassisIface->register_property("CurrentPowerState",
                                    std::string(getChassisState()));
    chassisIface->register_property("LastStateChangeTime", getCurrentTimeMs());

    chassisIface->initialize();

    lg2::info("Created the chassis interface successfully");
}

#ifdef CHASSIS_SYSTEM_RESET
void PowerControl::initializeChassisSystemInterface()
{
    // Chassis System Interface
    chassisSysIface =
        objServer.add_interface("/xyz/openbmc_project/state/chassis_system0",
                                "xyz.openbmc_project.State.Chassis");

    chassisSysIface->register_property(
        "RequestedPowerTransition",
        std::string("xyz.openbmc_project.State.Chassis.Transition.On"),
        [this](const std::string& requested, std::string& resp) {
            if (requested ==
                "xyz.openbmc_project.State.Chassis.Transition.PowerCycle")
            {
                // TODO: systemReset() needs to be moved or made virtual
                // systemReset();
                addRestartCause(RestartCause::command);
                lg2::info("Chassis system PowerCycle requested");
            }
            else
            {
                lg2::error(
                    "Unrecognized chassis system state transition request.");
                return 0;
            }
            resp = requested;
            return 1;
        });
    chassisSysIface->register_property("CurrentPowerState",
                                       std::string(getChassisState()));
    chassisSysIface->register_property("LastStateChangeTime",
                                       getCurrentTimeMs());

    chassisSysIface->initialize();

    lg2::info("Created the chassis system interface successfully");
}
#endif

void PowerControl::initializeBootProgressInterface()
{
    // Boot Progress Interface
    // This interface allows external entities (IPMI, PLDM, etc.) to update boot
    // progress
    bootProgressIface =
        objServer.add_interface("/xyz/openbmc_project/state/host" + nodeId,
                                "xyz.openbmc_project.State.Boot.Progress");

    // BootProgress property - indicates the current boot stage
    bootProgressIface->register_property(
        "BootProgress",
        std::string(
            "xyz.openbmc_project.State.Boot.Progress.ProgressStages.Unspecified"),
        [this](const std::string& requested, std::string& resp) {
            lg2::info("BootProgress updated to: {BOOT_PROGRESS}",
                      "BOOT_PROGRESS", requested);
            resp = requested;

            // Update the timestamp when BootProgress changes
            auto now = std::chrono::system_clock::now();
            auto timestamp =
                std::chrono::duration_cast<std::chrono::microseconds>(
                    now.time_since_epoch())
                    .count();

            if (bootProgressIface)
            {
                bootProgressIface->set_property(
                    "BootProgressLastUpdate", static_cast<uint64_t>(timestamp));
            }

            return 1;
        });

    // BootProgressLastUpdate property - timestamp of last update (microseconds
    // since epoch)
    bootProgressIface->register_property("BootProgressLastUpdate",
                                         static_cast<uint64_t>(0));

    // BootProgressOem property - OEM-specific boot progress information
    bootProgressIface->register_property(
        "BootProgressOem", std::string(""),
        [](const std::string& requested, std::string& resp) {
            lg2::info("BootProgressOem updated to: {OEM_PROGRESS}",
                      "OEM_PROGRESS", requested);
            resp = requested;
            return 1;
        });

    // NOTE: Do NOT call initialize() here - deferred to
    // initializeHostStateInterface()
    lg2::info("Boot.Progress interface registered (not yet initialized)");
}

void PowerControl::initializeButtonInterfaces()
{
    // Buttons Service
    // Power Button Interface
    auto powerButtonConfig = powerSignalMap.find("PowerButton");
    if (powerButtonConfig != powerSignalMap.end() &&
        !powerButtonConfig->second->lineName.empty())
    {
        powerButtonIface = objServer.add_interface(
            "/xyz/openbmc_project/chassis/buttons/power",
            "xyz.openbmc_project.Chassis.Buttons");

        powerButtonIface->register_property(
            "ButtonMasked", false, [this](const bool requested, bool& current) {
                if (requested)
                {
                    if (powerButtonMask)
                    {
                        return 1;
                    }
                    auto powerOutConfig = powerSignalMap.find("PowerOut");
                    if (powerOutConfig != powerSignalMap.end())
                    {
                        if (!setGPIOOutput(powerOutConfig->second,
                                           !powerOutConfig->second->polarity))
                        {
                            lg2::error("Failed to request GPIO");
                            return 0;
                        }
                        powerButtonMask = powerOutConfig->second->gpioLine;
                    }
                    lg2::info("Power Button Masked.");
                }
                else
                {
                    if (!powerButtonMask)
                    {
                        return 1;
                    }
                    lg2::info("Power Button Un-masked");
                    powerButtonMask.reset();
                }
                // Update the mask setting
                current = requested;
                return 1;
            });

        // Check power button state - default to not pressed
        bool powerButtonPressed = false;
        if (powerButtonConfig->second->gpioLine)
        {
            powerButtonPressed =
                powerButtonConfig->second->gpioLine.get_value() == 0;
        }

        powerButtonIface->register_property("ButtonPressed",
                                            powerButtonPressed);
        powerButtonIface->initialize();

        lg2::info("Created the power button interface successfully");
    }

    // Reset Button Interface
    auto resetButtonConfig = powerSignalMap.find("ResetButton");
    if (resetButtonConfig != powerSignalMap.end() &&
        !resetButtonConfig->second->lineName.empty())
    {
        resetButtonIface = objServer.add_interface(
            "/xyz/openbmc_project/chassis/buttons/reset",
            "xyz.openbmc_project.Chassis.Buttons");

        resetButtonIface->register_property(
            "ButtonMasked", false, [this](const bool requested, bool& current) {
                if (requested)
                {
                    if (resetButtonMask)
                    {
                        return 1;
                    }
                    auto resetOutConfig = powerSignalMap.find("ResetOut");
                    if (resetOutConfig != powerSignalMap.end())
                    {
                        if (!setGPIOOutput(resetOutConfig->second,
                                           !resetOutConfig->second->polarity))
                        {
                            lg2::error("Failed to request GPIO");
                            return 0;
                        }
                        resetButtonMask = resetOutConfig->second->gpioLine;
                    }
                    lg2::info("Reset Button Masked.");
                }
                else
                {
                    if (!resetButtonMask)
                    {
                        return 1;
                    }
                    lg2::info("Reset Button Un-masked");
                    resetButtonMask.reset();
                }
                // Update the mask setting
                current = requested;
                return 1;
            });

        // Check reset button state - default to not pressed
        bool resetButtonPressed = false;
        if (resetButtonConfig->second->gpioLine)
        {
            resetButtonPressed =
                resetButtonConfig->second->gpioLine.get_value() == 0;
        }

        resetButtonIface->register_property("ButtonPressed",
                                            resetButtonPressed);
        resetButtonIface->initialize();

        lg2::info("Created the reset button interface successfully");
    }

    // NMI Button Interface
    auto nmiButtonConfig = powerSignalMap.find("NMIButton");
    if (nmiButtonConfig != powerSignalMap.end() &&
        nmiButtonConfig->second->gpioLine)
    {
        nmiButtonIface =
            objServer.add_interface("/xyz/openbmc_project/chassis/buttons/nmi",
                                    "xyz.openbmc_project.Chassis.Buttons");

        nmiButtonIface->register_property(
            "ButtonMasked", false, [this](const bool requested, bool& current) {
                if (nmiButtonMasked == requested)
                {
                    // NMI button mask is already set as requested, so no change
                    return 1;
                }
                if (requested)
                {
                    lg2::info("NMI Button Masked.");
                    nmiButtonMasked = true;
                }
                else
                {
                    lg2::info("NMI Button Un-masked.");
                    nmiButtonMasked = false;
                }
                // Update the mask setting
                current = nmiButtonMasked;
                return 1;
            });

        // Check NMI button state
        bool nmiButtonPressed = false;
        if (nmiButtonConfig->second->gpioLine)
        {
            nmiButtonPressed = nmiButtonConfig->second->gpioLine.get_value() ==
                               0;
        }

        nmiButtonIface->register_property("ButtonPressed", nmiButtonPressed);
        nmiButtonIface->initialize();

        lg2::info("Created the NMI button interface successfully");
    }

    // NMI Out Interface
    auto nmiOutConfig = powerSignalMap.find("NMIOut");
    if (nmiOutConfig != powerSignalMap.end() && nmiOutConfig->second->gpioLine)
    {
        nmiOutIface = objServer.add_interface(
            "/xyz/openbmc_project/control/host" + nodeId + "/nmi",
            "xyz.openbmc_project.Control.Host.NMI");

        nmiOutIface->register_method("NMI", [this]() { nmiReset(); });
        nmiOutIface->initialize();

        lg2::info("Created the NMI out interface successfully");
    }

    // ID Button Interface
    auto idButtonConfig = powerSignalMap.find("IdButton");
    if (idButtonConfig != powerSignalMap.end() &&
        idButtonConfig->second->gpioLine)
    {
        idButtonIface =
            objServer.add_interface("/xyz/openbmc_project/chassis/buttons/id",
                                    "xyz.openbmc_project.Chassis.Buttons");

        // Check ID button state
        bool idButtonPressed = false;
        if (idButtonConfig->second->gpioLine)
        {
            idButtonPressed = idButtonConfig->second->gpioLine.get_value() == 0;
        }

        idButtonIface->register_property("ButtonPressed", idButtonPressed);
        idButtonIface->initialize();

        lg2::info("Created the ID button interface successfully");
    }
}

void PowerControl::initializeOSInterface()
{
    // OS State Interface
    osIface = objServer.add_interface(
        "/xyz/openbmc_project/state/host" + nodeId,
        "xyz.openbmc_project.State.OperatingSystem.Status");

    // Default to Inactive state
    osIface->register_property(
        "OperatingSystemState",
        std::string(
            "xyz.openbmc_project.State.OperatingSystem.Status.OSStatus.Inactive"));

    // NOTE: Do NOT call initialize() here - deferred to
    // initializeHostStateInterface()
    lg2::info("OS state interface registered (not yet initialized)");
}

void PowerControl::registerGpioStateInterface()
{
    // GPIO State Interface
    gpioStateIface =
        objServer.add_interface("/xyz/openbmc_project/state/host" + nodeId,
                                "xyz.openbmc_project.State.Gpio");

    // Register method: SetCpuBootDone(i state)
    gpioStateIface->register_method("SetCpuBootDone", [this](const int& state) {
        // Validate input: only accept 0 or 1
        if (state != 0 && state != 1)
        {
            lg2::error("SetCpuBootDone rejected: Invalid state value {STATE}. "
                       "Only 0 (de-asserted) or 1 (asserted) are allowed.",
                       "STATE", state);
            return 0;
        }

        // Update member variable
        cpuBootDone = state;

        // Log state change
        const char* stateStr = (state == 1) ? "ASSERTED" : "DE-ASSERTED";
        lg2::info("CPU Boot Done state changed to: {STATE}", "STATE", stateStr);

        // Update property value
        gpioStateIface->set_property("CpuBootDone", state);

        // Send power control event based on state
        Event cpuBootDoneEvent = (state == 1) ? Event::cpuBootDoneAssert
                                              : Event::cpuBootDoneDeAssert;
        sendPowerControlEvent(cpuBootDoneEvent);
        return 1;
    });

    // Register property: CpuBootDone (read-only, int type, initialized to -1)
    gpioStateIface->register_property_r(
        "CpuBootDone", int{-1}, sdbusplus::vtable::property_::emits_change,
        [this](const auto&) { return cpuBootDone; });

    // Register VR GPIO state properties (read-only)
    gpioStateIface->register_property_r(
        "CpuResetIndicator", int{-1},
        sdbusplus::vtable::property_::emits_change,
        [this](const auto&) { return cpuResetIndicatorState; });

    gpioStateIface->register_property_r(
        "Board0RunPowerPG", int{-1}, sdbusplus::vtable::property_::emits_change,
        [this](const auto&) { return board0RunPowerPGState; });

    gpioStateIface->register_property_r(
        "Board0CpuShutdownOk", int{-1},
        sdbusplus::vtable::property_::emits_change,
        [this](const auto&) { return board0CpuShutdownOkState; });

    // Note: Does NOT call initialize() - derived classes may register
    // additional GPIO properties before calling initializeHostStateInterface()
    // which initializes ALL host0 interfaces at once.
}

void PowerControl::initializeHostStateInterface()
{
    // Initialize ALL interfaces on /xyz/openbmc_project/state/host0 at once
    // This ensures that when the path becomes visible to ObjectMapper,
    // ALL interfaces are ready. This allows "mapper wait /path" to work
    // reliably for dependent services.

    // Initialize in order: Gpio first (since it was registered last in base),
    // then the rest, with Host last (as it's the primary interface)
    if (gpioStateIface)
    {
        gpioStateIface->initialize();
        lg2::info("GPIO state interface initialized");
    }

    if (bootProgressIface)
    {
        bootProgressIface->initialize();
        lg2::info("Boot.Progress interface initialized");
    }

    if (osIface)
    {
        osIface->initialize();
        lg2::info("OS state interface initialized");
    }

    if (hostIface)
    {
        hostIface->initialize();
        lg2::info(
            "Host state interface initialized - all host0 interfaces now ready");
    }
}

// =============================================================================
// VR GPIO STATE SETTERS (Update D-Bus properties)
// =============================================================================

void PowerControl::setCpuResetIndicatorState(int state)
{
    cpuResetIndicatorState = state;
    if (gpioStateIface)
    {
        gpioStateIface->set_property("CpuResetIndicator", state);
    }
}

void PowerControl::setBoard0RunPowerPGState(int state)
{
    board0RunPowerPGState = state;
    if (gpioStateIface)
    {
        gpioStateIface->set_property("Board0RunPowerPG", state);
    }
}

void PowerControl::setBoard0CpuShutdownOkState(int state)
{
    board0CpuShutdownOkState = state;
    if (gpioStateIface)
    {
        gpioStateIface->set_property("Board0CpuShutdownOk", state);
    }
}

void PowerControl::setBoard1CpuShutdownOkState(int state)
{
    board1CpuShutdownOkState = state;
    if (gpioStateIface)
    {
        gpioStateIface->set_property("Board1CpuShutdownOk", state);
    }
}

std::shared_ptr<ConfigData> PowerControl::getSignal(
    const std::string& signalName)
{
    auto it = powerSignalMap.find(signalName);
    if (it == powerSignalMap.end())
    {
        lg2::error("CRITICAL: {SIGNAL} signal not found in powerSignalMap",
                   "SIGNAL", signalName);
        return nullptr;
    }
    return it->second;
}

bool PowerControl::setGPIOOutput(std::shared_ptr<ConfigData> config,
                                 const int value)
{
    if (!config)
    {
        lg2::error("setGPIOOutput called with null ConfigData pointer");
        return false;
    }

    // Find the GPIO line
    if (!config->gpioLine)
    {
        config->gpioLine = gpiod::find_line(config->lineName);
        if (!config->gpioLine)
        {
            lg2::error("Failed to find the {GPIO_NAME} line", "GPIO_NAME",
                       config->lineName);
            return false;
        }
    }

    // Request GPIO output to specified value
    if (!config->gpioLine.is_requested())
    {
        try
        {
            config->gpioLine.request(
                {appName, gpiod::line_request::DIRECTION_OUTPUT, {}}, value);
        }
        catch (const std::exception& e)
        {
            lg2::error("Failed to request {GPIO_NAME} output: {ERROR}",
                       "GPIO_NAME", config->lineName, "ERROR", e);
            return false;
        }
    }
    else
    {
        try
        {
            config->gpioLine.set_value(value);
        }
        catch (const std::exception& e)
        {
            lg2::error("Failed to set {GPIO_NAME} value: {ERROR}", "GPIO_NAME",
                       config->lineName, "ERROR", e);
            return false;
        }
    }

    lg2::info("{GPIO_NAME} set to {GPIO_VALUE}", "GPIO_NAME", config->lineName,
              "GPIO_VALUE", value);
    return true;
}

// GPIO Timing Functions

int PowerControl::setMaskedGPIOOutputForMs(
    std::shared_ptr<ConfigData> config, const int value, const int durationMs)
{
    if (!config)
    {
        lg2::error(
            "setMaskedGPIOOutputForMs called with null ConfigData pointer");
        return -1;
    }

    if (!config->gpioLine)
    {
        lg2::error(
            "setMaskedGPIOOutputForMs: GPIO line for {GPIO_NAME} is not initialized",
            "GPIO_NAME", config->lineName);
        return -1;
    }

    // Set the masked GPIO line to the specified value
    config->gpioLine.set_value(value);
    lg2::info("{GPIO_NAME} set to {GPIO_VALUE}", "GPIO_NAME", config->lineName,
              "GPIO_VALUE", value);

    gpioAssertTimer.expires_after(std::chrono::milliseconds(durationMs));
    gpioAssertTimer.async_wait([config, value](
                                   const boost::system::error_code ec) mutable {
        // Set the masked GPIO line back to the opposite value
        if (config && config->gpioLine)
        {
            config->gpioLine.set_value(!value);
            lg2::info("{GPIO_NAME} released", "GPIO_NAME", config->lineName);
        }
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error("{GPIO_NAME} async_wait failed: {ERROR_MSG}",
                           "GPIO_NAME", config->lineName, "ERROR_MSG",
                           ec.message());
            }
        }
    });
    return 0;
}

int PowerControl::setGPIOOutputForMs(std::shared_ptr<ConfigData> config,
                                     const int value, const int durationMs)
{
    if (!config)
    {
        lg2::error("setGPIOOutputForMs called with null ConfigData pointer");
        return -1;
    }

    // Check if the requested GPIO is masked
    // If PowerOut is being controlled and powerButtonMask is set, use masked
    // version
    auto powerOutIt = powerSignalMap.find("PowerOut");
    if (powerButtonMask && powerOutIt != powerSignalMap.end() &&
        config->lineName == powerOutIt->second->lineName)
    {
        return setMaskedGPIOOutputForMs(powerOutIt->second, value, durationMs);
    }

    // If ResetOut is being controlled and resetButtonMask is set, use masked
    // version
    auto resetOutIt = powerSignalMap.find("ResetOut");
    if (resetButtonMask && resetOutIt != powerSignalMap.end() &&
        config->lineName == resetOutIt->second->lineName)
    {
        return setMaskedGPIOOutputForMs(resetOutIt->second, value, durationMs);
    }

    // No mask set, so request and set the GPIO normally
    if (!setGPIOOutput(config, value))
    {
        return -1;
    }

    const std::string name = config->lineName;

    gpioAssertTimer.expires_after(std::chrono::milliseconds(durationMs));
    gpioAssertTimer.async_wait(
        [config, value, name](const boost::system::error_code ec) mutable {
            // Set the GPIO line back to the opposite value
            if (config && config->gpioLine)
            {
                config->gpioLine.set_value(!value);
                lg2::info("{GPIO_NAME} released", "GPIO_NAME", name);
            }
            if (ec)
            {
                // operation_aborted is expected if timer is canceled before
                // completion.
                if (ec != boost::asio::error::operation_aborted)
                {
                    lg2::error("{GPIO_NAME} async_wait failed: {ERROR_MSG}",
                               "GPIO_NAME", name, "ERROR_MSG", ec.message());
                }
            }
        });
    return 0;
}

int PowerControl::assertGPIOForMs(std::shared_ptr<ConfigData> config,
                                  const int durationMs)
{
    if (!config)
    {
        lg2::error("assertGPIOForMs called with null ConfigData pointer");
        return -1;
    }

    return setGPIOOutputForMs(config, config->polarity, durationMs);
}

void PowerControl::startTimer(const std::string& timerName,
                              boost::asio::steady_timer& timer,
                              Event eventOnExpiry)
{
    lg2::info("Starting timer {TIMER_NAME}", "TIMER_NAME", timerName);

    // Look up timeout from TimerMap
    auto it = TimerMap.find(timerName);
    if (it == TimerMap.end())
    {
        lg2::error("Timer '{TIMER}' not found in TimerMap", "TIMER", timerName);
        return;
    }

    int timeoutMs = it->second;

    lg2::info("{TIMER_NAME} timer started with {TIMEOUT_MS}ms timeout",
              "TIMER_NAME", timerName, "TIMEOUT_MS", timeoutMs);

    timer.expires_after(std::chrono::milliseconds(timeoutMs));
    timer.async_wait([this, eventOnExpiry,
                      timerName](const boost::system::error_code& ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error("{TIMER_NAME} timer async_wait failed: {ERROR_MSG}",
                           "TIMER_NAME", timerName, "ERROR_MSG", ec.message());
            }
            else
            {
                lg2::info("{TIMER_NAME} timer canceled", "TIMER_NAME",
                          timerName);
            }
            return;
        }

        lg2::info("{TIMER_NAME} timer expired", "TIMER_NAME", timerName);
        sendPowerControlEvent(eventOnExpiry);
    });
}

void PowerControl::cancelTimer(const std::string& timerName,
                               boost::asio::steady_timer& timer)
{
    lg2::info("Canceling {TIMER_NAME}", "TIMER_NAME", timerName);
    timer.cancel();
    // The timer name will also be logged when the async_wait callback fires
    // with operation_aborted: "{TIMER_NAME} canceled"
}

void PowerControl::addRequiredSignal(const std::string& signalName,
                                     int boardIndex, GPIODirection direction,
                                     std::function<void(bool)> handler)
{
    if (boardIndex == 0)
    {
        requiredBoard0Signals.push_back(signalName);
    }
    else if (boardIndex == 1)
    {
        requiredBoard1Signals.push_back(signalName);
    }
    else
    {
        lg2::error("Invalid board index {INDEX} for signal {SIGNAL}", "INDEX",
                   boardIndex, "SIGNAL", signalName);
        return;
    }

    if (handler)
    {
        registerGPIOHandler(signalName, direction, handler);
    }
}

void PowerControl::validateRequiredSignals()
{
    // TODO: Determine which configs are required by upstream PowerControl
    //
    // {"PowerOut", &powerOutConfig},
    // {"PowerOk", &powerOkConfig},
    // {"ResetOut", &resetOutConfig},
    // {"NMIOut", &nmiOutConfig},
    // {"SioPowerGood", &sioPwrGoodConfig},
    // {"SioOnControl", &sioOnControlConfig},
    // {"SIOS5", &sioS5Config},
    // {"PostComplete", &postCompleteConfig},
    // {"PowerButton", &powerButtonConfig},
    // {"ResetButton", &resetButtonConfig},
    // {"IdButton", &idButtonConfig},
    // {"NMIButton", &nmiButtonConfig},
    // {"SlotPower", &slotPowerConfig},
    // {"HpmStbyEn", &hpmStbyEnConfig}};
    lg2::info("Validating required signals");
    // Validate Board 0 signals (always required)
    for (const auto& signalName : requiredBoard0Signals)
    {
        lg2::info("Required Board 0 signal: '{SIGNAL}'", "SIGNAL", signalName);
        auto it = powerSignalMap.find(signalName);
        if (it == powerSignalMap.end())
        {
            lg2::error("Required Board 0 signal '{SIGNAL}' not found in config",
                       "SIGNAL", signalName);
            throw std::runtime_error(
                "Required Board 0 signal missing from config: " + signalName);
        }

        lg2::info("'{SIGNAL}' found in config", "SIGNAL", signalName);
        if (it->second->direction == GPIODirection::IN &&
            it->second->gpioHandler == nullptr)
        {
            lg2::error(
                "Required Board 0 signal '{SIGNAL}' is an input signal, but no handler function was provided",
                "SIGNAL", signalName);
        }
    }

    // Validate Board 1 signals (if any were added)
    for (const auto& signalName : requiredBoard1Signals)
    {
        lg2::info("Required Board 1 signal: '{SIGNAL}'", "SIGNAL", signalName);
        auto it = powerSignalMap.find(signalName);
        if (it == powerSignalMap.end())
        {
            lg2::error("Required Board 1 signal '{SIGNAL}' not found in config",
                       "SIGNAL", signalName);
            throw std::runtime_error(
                "Required Board 1 signal missing from config: " + signalName);
        }
        lg2::info("'{SIGNAL}' found in config", "SIGNAL", signalName);
        if (it->second->direction == GPIODirection::IN &&
            it->second->gpioHandler == nullptr)
        {
            lg2::error(
                "Required Board 1 signal '{SIGNAL}' is an input signal, but no handler function was provided",
                "SIGNAL", signalName);
        }
    }

    lg2::info("Signal validation complete - all required signals present");
}

void PowerControl::validateTimerConfigs()
{
    // Base class validates upstream timers
    for (const auto& timerName : baseRequiredTimers)
    {
        if (TimerMap.find(timerName) == TimerMap.end())
        {
            lg2::error("Required timer config '{TIMER}' not found in config",
                       "TIMER", timerName);
            throw std::runtime_error(
                "PowerControl: Required timer config missing: " + timerName);
        }
    }

    lg2::info("PowerControl timer configuration validation complete");
}

void PowerControl::registerGPIOHandler(const std::string& signalName,
                                       GPIODirection direction,
                                       std::function<void(bool)> handler)
{
    lg2::info("Registering GPIO handler for signal '{SIGNAL}'", "SIGNAL",
              signalName);

    // Find the signal in powerSignalMap
    auto it = powerSignalMap.find(signalName);
    if (it == powerSignalMap.end())
    {
        lg2::error(
            "GPIO signal '{SIGNAL}' not found in config, handler not registered",
            "SIGNAL", signalName);
        return;
    }

    // Set the direction of the GPIO
    it->second->direction = direction;

    // Assign the handler to the ConfigData object
    it->second->gpioHandler = handler;

    // Check if this signal uses input event monitoring (gpio_keys_polled
    // driver)
    if (it->second->useInputEvents)
    {
        // Validate input event configuration
        if (!it->second->inputEventConfig.has_value())
        {
            lg2::error(
                "Signal '{SIGNAL}' is configured for input events but inputEventConfig is not set",
                "SIGNAL", signalName);
            throw std::runtime_error(
                "Signal '" + signalName + "' missing inputEventConfig");
        }

        auto& inputConfig = it->second->inputEventConfig.value();

        // Request input events for this signal
        if (!requestInputEvents(inputConfig.deviceName, inputConfig.signalName,
                                inputConfig.keyCode, handler,
                                it->second->eventDescriptor,
                                inputConfig.stateTracker))
        {
            lg2::error("Failed to register input events for '{SIGNAL}'",
                       "SIGNAL", signalName);
            throw std::runtime_error(
                "Failed to register input events for '" + signalName + "'");
        }

        lg2::info("Successfully registered input event handler for '{SIGNAL}'",
                  "SIGNAL", signalName);
    }
    else
    {
        // Request GPIO events for this signal (traditional method)
        if (!requestGPIOEvents(*it->second))
        {
            lg2::error("Failed to register GPIO events for '{SIGNAL}'",
                       "SIGNAL", signalName);
            throw std::runtime_error(
                "Failed to register GPIO events for '" + signalName + "'");
        }

        lg2::info("Successfully registered GPIO handler for '{SIGNAL}'",
                  "SIGNAL", signalName);
    }
}

// ===========================================================================
// GPIO Event Handlers
// ===========================================================================

void PowerControl::powerOKHandler(bool state)
{
    auto it = powerSignalMap.find("PowerOk");
    if (it == powerSignalMap.end())
    {
        lg2::error("PowerOk signal not found in powerSignalMap");
        return;
    }

    auto& config = *it->second;
    Event powerControlEvent = (state == config.polarity)
                                  ? Event::powerOKAssert
                                  : Event::powerOKDeAssert;
    sendPowerControlEvent(powerControlEvent);
}

void PowerControl::sioPowerGoodHandler(bool state)
{
    auto it = powerSignalMap.find("SioPowerGood");
    if (it == powerSignalMap.end())
    {
        lg2::error("SioPowerGood signal not found in powerSignalMap");
        return;
    }

    auto& config = *it->second;
    Event powerControlEvent = (state == config.polarity)
                                  ? Event::sioPowerGoodAssert
                                  : Event::sioPowerGoodDeAssert;
    sendPowerControlEvent(powerControlEvent);
}

void PowerControl::sioS5Handler(bool state)
{
    auto it = powerSignalMap.find("SIOS5");
    if (it == powerSignalMap.end())
    {
        lg2::error("SIOS5 signal not found in powerSignalMap");
        return;
    }

    auto& config = *it->second;
    Event powerControlEvent =
        (state == config.polarity) ? Event::sioS5Assert : Event::sioS5DeAssert;
    sendPowerControlEvent(powerControlEvent);
}

void PowerControl::powerButtonHandler(bool state)
{
    auto it = powerSignalMap.find("PowerButton");
    if (it == powerSignalMap.end())
    {
        lg2::error("PowerButton signal not found in powerSignalMap");
        return;
    }

    auto& config = *it->second;
    bool asserted = state == config.polarity;
    powerButtonIface->set_property("ButtonPressed", asserted);
    if (asserted)
    {
        powerButtonPressLog();
        if (!powerButtonMask)
        {
            sendPowerControlEvent(Event::powerButtonPressed);
            addRestartCause(RestartCause::powerButton);
        }
        else
        {
            lg2::info("power button press masked");
        }
    }
#if USE_BUTTON_PASSTHROUGH
    // Note: powerOutConfig is not in base class yet, this will need to be
    // handled by derived classes if they use button passthrough
    lg2::info("Button passthrough not implemented in base class");
#endif
}

void PowerControl::resetButtonHandler(bool state)
{
    // Lookup config for polarity (guaranteed to exist since handler was
    // registered)
    auto it = powerSignalMap.find("ResetButton");

    if (it == powerSignalMap.end())
    {
        lg2::error("ResetButton signal not found in powerSignalMap");
        return;
    }

    auto& config = *it->second;

    bool asserted = state == config.polarity;
    resetButtonIface->set_property("ButtonPressed", asserted);
    if (asserted)
    {
        resetButtonPressLog();
        if (!resetButtonMask)
        {
            sendPowerControlEvent(Event::resetButtonPressed);
            addRestartCause(RestartCause::resetButton);
        }
        else
        {
            lg2::info("reset button press masked");
        }
    }
#if USE_BUTTON_PASSTHROUGH
    // Note: resetOutConfig is not in base class yet, this will need to be
    // handled by derived classes if they use button passthrough
    lg2::info("Button passthrough not implemented in base class");
#endif
}

void PowerControl::idButtonHandler(bool state)
{
    // Lookup config for polarity (guaranteed to exist since handler was
    // registered)
    auto it = powerSignalMap.find("IdButton");
    if (it == powerSignalMap.end())
    {
        lg2::error("IdButton not found in powerSignalMap");
        return;
    }
    auto& config = *it->second;

    bool asserted = state == config.polarity;
    idButtonIface->set_property("ButtonPressed", asserted);
}

void PowerControl::pltRstHandler(bool pltRst)
{
    if (pltRst)
    {
        sendPowerControlEvent(Event::pltRstDeAssert);
    }
    else
    {
        sendPowerControlEvent(Event::pltRstAssert);
    }
}

void PowerControl::sioOnControlHandler(bool state)
{
    lg2::info("SIO_ONCONTROL value changed: {VALUE}", "VALUE",
              static_cast<int>(state));
}

void PowerControl::hostMiscHandler(sdbusplus::message_t& msg)
{
    std::string interfaceName;
    boost::container::flat_map<std::string, std::variant<bool>>
        propertiesChanged;
    try
    {
        msg.read(interfaceName, propertiesChanged);
    }
    catch (const std::exception& e)
    {
        lg2::error("Unable to read Host Misc status: {ERROR}", "ERROR", e);
        return;
    }
    if (propertiesChanged.empty())
    {
        lg2::error("ERROR: Empty Host.Misc PropertiesChanged signal received");
        return;
    }

    for (auto& [property, value] : propertiesChanged)
    {
        if (property == "ESpiPlatformReset")
        {
            bool* pltRst = std::get_if<bool>(&value);
            if (pltRst == nullptr)
            {
                lg2::error("{PROPERTY} property invalid", "PROPERTY", property);
                return;
            }
            pltRstHandler(*pltRst);
        }
    }
}

template <typename T>
std::optional<T> PowerControl::getMessageValue(sdbusplus::message_t& msg,
                                               const std::string& name)
{
    std::string event;
    std::string thresholdInterface;
    boost::container::flat_map<std::string, std::variant<T>> propertiesChanged;

    msg.read(thresholdInterface, propertiesChanged);
    if (propertiesChanged.empty())
    {
        return std::nullopt;
    }

    event = propertiesChanged.begin()->first;
    if (event.empty() || event != name)
    {
        return std::nullopt;
    }

    return std::get<T>(propertiesChanged.begin()->second);
}

// Explicit template instantiations
template std::optional<bool> PowerControl::getMessageValue<bool>(
    sdbusplus::message_t& msg, const std::string& name);
template std::optional<std::string> PowerControl::getMessageValue<std::string>(
    sdbusplus::message_t& msg, const std::string& name);

bool PowerControl::getDbusMsgGPIOState(sdbusplus::message_t& msg,
                                       const ConfigData& config, bool& value)
{
    try
    {
        if (config.matchRegex.has_value())
        {
            std::optional<std::string> s =
                getMessageValue<std::string>(msg, config.lineName);
            if (!s.has_value())
            {
                return false;
            }

            std::smatch m;
            value = std::regex_match(s.value(), m, config.matchRegex.value());
        }
        else
        {
            std::optional<bool> v = getMessageValue<bool>(msg, config.lineName);
            if (!v.has_value())
            {
                return false;
            }
            value = v.value();
        }
        return true;
    }
    catch (const std::exception& e)
    {
        lg2::error(
            "exception while reading dbus property \'{DBUS_NAME}\': {ERROR}",
            "DBUS_NAME", config.lineName, "ERROR", e);
        return false;
    }
}

sdbusplus::bus::match_t PowerControl::dbusGPIOMatcher(
    const ConfigData& cfg, std::function<void(bool)> onMatch)
{
    auto pulseEventMatcherCallback =
        [this, &cfg, onMatch](sdbusplus::message_t& msg) {
            bool value = false;
            if (!getDbusMsgGPIOState(msg, cfg, value))
            {
                return;
            }
            onMatch(value);
        };

    return sdbusplus::bus::match_t(
        static_cast<sdbusplus::bus_t&>(*conn),
        "type='signal',interface='org.freedesktop.DBus.Properties',member='"
        "PropertiesChanged',arg0='" +
            cfg.interface + "',path='" + cfg.path + "',sender='" +
            cfg.dbusName + "'",
        std::move(pulseEventMatcherCallback));
}

void PowerControl::powerButtonPressLog()
{
    sd_journal_send("MESSAGE=PowerControl: power button pressed", "PRIORITY=%i",
                    LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.PowerButtonPressed", NULL);
}

void PowerControl::resetButtonPressLog()
{
    sd_journal_send("MESSAGE=PowerControl: reset button pressed", "PRIORITY=%i",
                    LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.ResetButtonPressed", NULL);
}

void PowerControl::systemPowerGoodFailedLog()
{
    auto it = TimerMap.find("SioPowerGoodWatchdogMs");
    int timeoutValue = (it != TimerMap.end()) ? it->second : -1;
    sd_journal_send(
        "MESSAGE=PowerControl: system power good failed to assert (VR failure)",
        "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
        "OpenBMC.0.1.SystemPowerGoodFailed", "REDFISH_MESSAGE_ARGS=%d",
        timeoutValue, NULL);
}

void PowerControl::psPowerOKFailedLog()
{
    auto it = TimerMap.find("PsPowerOKWatchdogMs");
    int timeoutValue = (it != TimerMap.end()) ? it->second : -1;
    sd_journal_send(
        "MESSAGE=PowerControl: power supply power good failed to assert",
        "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
        "OpenBMC.0.1.PowerSupplyPowerGoodFailed", "REDFISH_MESSAGE_ARGS=%d",
        timeoutValue, NULL);
}

void PowerControl::nmiButtonPressLog()
{
    sd_journal_send("MESSAGE=PowerControl: NMI button pressed", "PRIORITY=%i",
                    LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.NMIButtonPressed", NULL);
}

void PowerControl::nmiDiagIntLog()
{
    sd_journal_send("MESSAGE=PowerControl: NMI Diagnostic Interrupt",
                    "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.NMIDiagnosticInterrupt", NULL);
}

void PowerControl::nmiSetEnableProperty(bool value)
{
    conn->async_method_call(
        [](boost::system::error_code ec) {
            if (ec)
            {
                lg2::error("failed to set NMI source");
            }
        },
        "xyz.openbmc_project.Settings",
        "/xyz/openbmc_project/Chassis/Control/NMISource",
        "org.freedesktop.DBus.Properties", "Set",
        "xyz.openbmc_project.Chassis.Control.NMISource", "Enabled",
        std::variant<bool>{value});
}

void PowerControl::nmiReset()
{
    const static constexpr int nmiOutPulseTimeMs = 200;

    auto nmiOutIt = powerSignalMap.find("NMIOut");
    if (nmiOutIt == powerSignalMap.end() || !nmiOutIt->second->gpioLine)
    {
        lg2::error(
            "NMIOut not found in powerSignalMap or GPIO line not initialized");
        return;
    }

    auto& nmiOutConfig = nmiOutIt->second;

    lg2::info("NMI out action");
    nmiOutConfig->gpioLine.set_value(nmiOutConfig->polarity);
    lg2::info("{GPIO_NAME} set to {GPIO_VALUE}", "GPIO_NAME",
              nmiOutConfig->lineName, "GPIO_VALUE", nmiOutConfig->polarity);

    gpioAssertTimer.expires_after(std::chrono::milliseconds(nmiOutPulseTimeMs));
    gpioAssertTimer.async_wait([this, nmiOutConfig](
                                   const boost::system::error_code ec) {
        // restore the NMI_OUT GPIO line back to the opposite value
        nmiOutConfig->gpioLine.set_value(!nmiOutConfig->polarity);
        lg2::info("{GPIO_NAME} released", "GPIO_NAME", nmiOutConfig->lineName);
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error("{GPIO_NAME} async_wait failed: {ERROR_MSG}",
                           "GPIO_NAME", nmiOutConfig->lineName, "ERROR_MSG",
                           ec.message());
            }
        }
    });
    // log to redfish
    nmiDiagIntLog();
    lg2::info("NMI out action completed");
    // reset Enable Property
    nmiSetEnableProperty(false);
}

void PowerControl::nmiSourcePropertyMonitor()
{
    // Check if NMIOut is configured before setting up monitor
    auto nmiOutIt = powerSignalMap.find("NMIOut");
    if (nmiOutIt == powerSignalMap.end())
    {
        lg2::info(
            "NMIOut not configured, skipping NMI source property monitor");
        return;
    }

    lg2::info("NMI Source Property Monitor");

    static std::unique_ptr<sdbusplus::bus::match_t> nmiSourceMatch =
        std::make_unique<sdbusplus::bus::match_t>(
            *conn,
            "type='signal',interface='org.freedesktop.DBus.Properties',"
            "member='PropertiesChanged',"
            "arg0namespace='xyz.openbmc_project.Chassis.Control.NMISource'",
            [this](sdbusplus::message_t& msg) {
                std::string interfaceName;
                boost::container::flat_map<std::string,
                                           std::variant<bool, std::string>>
                    propertiesChanged;
                std::string state;
                bool value = true;
                try
                {
                    msg.read(interfaceName, propertiesChanged);
                    if (propertiesChanged.begin()->first == "Enabled")
                    {
                        value =
                            std::get<bool>(propertiesChanged.begin()->second);
                        lg2::info(
                            "NMI Enabled propertiesChanged value: {VALUE}",
                            "VALUE", value);
                        nmiEnabled = value;
                        if (nmiEnabled)
                        {
                            nmiReset();
                        }
                    }
                }
                catch (const std::exception& e)
                {
                    lg2::error("Unable to read NMI source: {ERROR}", "ERROR",
                               e);
                    return;
                }
            });
}

void PowerControl::setNmiSource()
{
    conn->async_method_call(
        [](boost::system::error_code ec) {
            if (ec)
            {
                lg2::error("failed to set NMI source");
            }
        },
        "xyz.openbmc_project.Settings",
        "/xyz/openbmc_project/Chassis/Control/NMISource",
        "org.freedesktop.DBus.Properties", "Set",
        "xyz.openbmc_project.Chassis.Control.NMISource", "BMCSource",
        std::variant<std::string>{
            "xyz.openbmc_project.Chassis.Control.NMISource.BMCSourceSignal.FrontPanelButton"});
    // set Enable Property
    nmiSetEnableProperty(true);
}

void PowerControl::pohCounterTimerStart()
{
    lg2::info("POH timer started");
    // Set the time-out as 1 hour, to align with POH command in ipmid
    pohCounterTimer.expires_after(std::chrono::hours(1));
    pohCounterTimer.async_wait([this](const boost::system::error_code& ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error("POH timer async_wait failed: {ERROR_MSG}",
                           "ERROR_MSG", ec.message());
            }
            lg2::info("POH timer canceled");
            return;
        }

        if (getHostState() !=
            "xyz.openbmc_project.State.Host.HostState.Running")
        {
            return;
        }

        conn->async_method_call(
            [this](boost::system::error_code ec,
                   const std::variant<uint32_t>& pohCounterProperty) {
                if (ec)
                {
                    lg2::error("error getting poh counter");
                    return;
                }
                const uint32_t* pohCounter =
                    std::get_if<uint32_t>(&pohCounterProperty);
                if (pohCounter == nullptr)
                {
                    lg2::error("unable to read poh counter");
                    return;
                }

                conn->async_method_call(
                    [](boost::system::error_code ec) {
                        if (ec)
                        {
                            lg2::error("failed to set poh counter");
                        }
                    },
                    "xyz.openbmc_project.Settings",
                    "/xyz/openbmc_project/state/chassis0",
                    "org.freedesktop.DBus.Properties", "Set",
                    "xyz.openbmc_project.State.PowerOnHours", "POHCounter",
                    std::variant<uint32_t>(*pohCounter + 1));
            },
            "xyz.openbmc_project.Settings",
            "/xyz/openbmc_project/state/chassis0",
            "org.freedesktop.DBus.Properties", "Get",
            "xyz.openbmc_project.State.PowerOnHours", "POHCounter");

        pohCounterTimerStart();
    });
}

void PowerControl::beep(const uint8_t& beepPriority)
{
    lg2::info("Beep with priority: {BEEP_PRIORITY}", "BEEP_PRIORITY",
              beepPriority);
    conn->async_method_call(
        [](boost::system::error_code ec) {
            if (ec)
            {
                lg2::error(
                    "beep returned error with async_method_call (ec = {ERROR_MSG})",
                    "ERROR_MSG", ec.message());
                return;
            }
        },
        "xyz.openbmc_project.BeepCode", "/xyz/openbmc_project/BeepCode",
        "xyz.openbmc_project.BeepCode", "Beep", uint8_t(beepPriority));
}

void PowerControl::currentHostStateMonitor()
{
    if (getHostState() == "xyz.openbmc_project.State.Host.HostState.Running")
    {
        pohCounterTimerStart();
        // Clear the restart cause set for the next restart
        clearRestartCause();
    }
    else
    {
        cancelTimer("POH Counter Timer", pohCounterTimer);
        // Set the restart cause set for this restart
        setRestartCause();
    }

    static auto match = sdbusplus::bus::match_t(
        *conn,
        "type='signal',member='PropertiesChanged', "
        "interface='org.freedesktop.DBus.Properties', "
        "arg0='xyz.openbmc_project.State.Host'",
        [this](sdbusplus::message_t& message) {
            std::string intfName;
            std::map<std::string, std::variant<std::string>> properties;

            try
            {
                message.read(intfName, properties);
            }
            catch (const std::exception& e)
            {
                lg2::error("Unable to read host state: {ERROR}", "ERROR", e);
                return;
            }
            if (properties.empty())
            {
                lg2::error("ERROR: Empty PropertiesChanged signal received");
                return;
            }

            // We only want to check for CurrentHostState
            if (properties.begin()->first != "CurrentHostState")
            {
                return;
            }
            std::string* currentHostState =
                std::get_if<std::string>(&(properties.begin()->second));
            if (currentHostState == nullptr)
            {
                lg2::error("{PROPERTY} property invalid", "PROPERTY",
                           properties.begin()->first);
                return;
            }

            if (*currentHostState ==
                "xyz.openbmc_project.State.Host.HostState.Running")
            {
                pohCounterTimerStart();
                // Clear the restart cause set for the next restart
                clearRestartCause();
                sd_journal_send("MESSAGE=Host system DC power is on",
                                "PRIORITY=%i", LOG_INFO,
                                "REDFISH_MESSAGE_ID=%s",
                                "OpenBMC.0.1.DCPowerOn", NULL);
            }
            else
            {
                cancelTimer("POH Counter Timer", pohCounterTimer);
                // POST_COMPLETE GPIO event is not working in some platforms
                // when power state is changed to OFF. This resulted in
                // 'OperatingSystemState' to stay at 'Standby', even though
                // system is OFF. Set 'OperatingSystemState' to 'Inactive'
                // if HostState is turned to OFF.
                setOperatingSystemState(OperatingSystemStateStage::Inactive);

                // Set the restart cause set for this restart
                setRestartCause();
#ifdef USE_ACBOOT
                this->resetACBootProperty();
#endif // USE_ACBOOT
                sd_journal_send("MESSAGE=Host system DC power is off",
                                "PRIORITY=%i", LOG_INFO,
                                "REDFISH_MESSAGE_ID=%s",
                                "OpenBMC.0.1.DCPowerOff", NULL);
            }
        });
}

// RestartCause Implementation (available in the power_control namespace)

std::string getRestartCause(RestartCause cause)
{
    switch (cause)
    {
        case RestartCause::command:
            return "xyz.openbmc_project.State.Host.RestartCause.RemoteCommand";
            break;
        case RestartCause::resetButton:
            return "xyz.openbmc_project.State.Host.RestartCause.ResetButton";
            break;
        case RestartCause::powerButton:
            return "xyz.openbmc_project.State.Host.RestartCause.PowerButton";
            break;
        case RestartCause::watchdog:
            return "xyz.openbmc_project.State.Host.RestartCause.WatchdogTimer";
            break;
        case RestartCause::powerPolicyOn:
            return "xyz.openbmc_project.State.Host.RestartCause.PowerPolicyAlwaysOn";
            break;
        case RestartCause::powerPolicyRestore:
            return "xyz.openbmc_project.State.Host.RestartCause.PowerPolicyPreviousState";
            break;
        case RestartCause::softReset:
            return "xyz.openbmc_project.State.Host.RestartCause.SoftReset";
            break;
        default:
            return "xyz.openbmc_project.State.Host.RestartCause.Unknown";
            break;
    }
}

void PowerControl::addRestartCause(const RestartCause cause)
{
    // Add this to the set of causes for this restart
    this->causeSet.insert(cause);
}

void PowerControl::clearRestartCause()
{
    // Clear the set for the next restart
    this->causeSet.clear();
}

void PowerControl::setRestartCauseProperty(const std::string& cause)
{
    lg2::info("RestartCause set to {RESTART_CAUSE}", "RESTART_CAUSE", cause);
    hostIface->set_property("RestartCause", cause);
}

void PowerControl::setRestartCause()
{
    // Determine the actual restart cause based on the set of causes
    std::string restartCause =
        "xyz.openbmc_project.State.Host.RestartCause.Unknown";
    if (this->causeSet.contains(RestartCause::watchdog))
    {
        restartCause = getRestartCause(RestartCause::watchdog);
    }
    else if (this->causeSet.contains(RestartCause::command))
    {
        restartCause = getRestartCause(RestartCause::command);
    }
    else if (this->causeSet.contains(RestartCause::resetButton))
    {
        restartCause = getRestartCause(RestartCause::resetButton);
    }
    else if (this->causeSet.contains(RestartCause::powerButton))
    {
        restartCause = getRestartCause(RestartCause::powerButton);
    }
    else if (this->causeSet.contains(RestartCause::powerPolicyOn))
    {
        restartCause = getRestartCause(RestartCause::powerPolicyOn);
    }
    else if (this->causeSet.contains(RestartCause::powerPolicyRestore))
    {
        restartCause = getRestartCause(RestartCause::powerPolicyRestore);
    }
    else if (this->causeSet.contains(RestartCause::softReset))
    {
#if IGNORE_SOFT_RESETS_DURING_POST
        if (PowerControl::ignoreNextSoftReset)
        {
            PowerControl::ignoreNextSoftReset = false;
            return;
        }
#endif
        restartCause = getRestartCause(RestartCause::softReset);
    }

    setRestartCauseProperty(restartCause);
}

#ifdef USE_ACBOOT
void PowerControl::resetACBootProperty()
{
    if ((this->causeSet.contains(RestartCause::command)) ||
        (this->causeSet.contains(RestartCause::softReset)))
    {
        conn->async_method_call(
            [](boost::system::error_code ec) {
                if (ec)
                {
                    lg2::error("failed to reset ACBoot property");
                }
            },
            "xyz.openbmc_project.Settings",
            "/xyz/openbmc_project/control/host0/ac_boot",
            "org.freedesktop.DBus.Properties", "Set",
            "xyz.openbmc_project.Common.ACBoot", "ACBoot",
            std::variant<std::string>{"False"});
    }
}
#endif // USE_ACBOOT

// INPUT EVENT HANDLING (for gpio_keys_polled driver)

std::string PowerControl::findInputEventDevice(const std::string& deviceName)
{
    // Search through /dev/input/eventX devices to find the one matching our
    // name
    for (int i = 0; i < 32; i++)
    {
        std::string eventPath = "/dev/input/event" + std::to_string(i);
        std::string namePath =
            "/sys/class/input/event" + std::to_string(i) + "/device/name";

        std::ifstream nameFile(namePath);
        if (nameFile.is_open())
        {
            std::string name;
            std::getline(nameFile, name);
            if (name == deviceName)
            {
                lg2::info("Found input device {DEVICE_NAME} at {EVENT_PATH}",
                          "DEVICE_NAME", deviceName, "EVENT_PATH", eventPath);
                return eventPath;
            }
        }
    }
    return "";
}

void PowerControl::waitForInputEvent(
    const std::string& name, const std::function<void(bool)>& eventHandler,
    uint16_t keyCode, boost::asio::posix::stream_descriptor& event,
    int* stateTracker)
{
    event.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [this, name, eventHandler, keyCode, &event,
         stateTracker](const boost::system::error_code ec) {
            if (ec)
            {
                lg2::error("{INPUT_NAME} fd handler error: {ERROR_MSG}",
                           "INPUT_NAME", name, "ERROR_MSG", ec.message());
                return;
            }

            struct input_event inputEvent;
            ssize_t bytesRead =
                read(event.native_handle(), &inputEvent, sizeof(inputEvent));

            if (bytesRead != sizeof(inputEvent))
            {
                if (bytesRead < 0)
                {
                    lg2::error("{INPUT_NAME} read error: {ERROR}", "INPUT_NAME",
                               name, "ERROR", strerror(errno));
                }
                else
                {
                    lg2::error(
                        "{INPUT_NAME} read error: incomplete event (got {BYTES} bytes)",
                        "INPUT_NAME", name, "BYTES", bytesRead);
                }
                waitForInputEvent(name, eventHandler, keyCode, event,
                                  stateTracker);
                return;
            }

            // We only care about EV_KEY events with our specific key code
            if (inputEvent.type == EV_KEY && inputEvent.code == keyCode)
            {
                lg2::info(
                    "{INPUT_NAME} event: code={KEY_CODE:#x} value={VALUE}",
                    "INPUT_NAME", name, "KEY_CODE", inputEvent.code, "VALUE",
                    inputEvent.value);

                // Update state tracker if provided
                if (stateTracker != nullptr)
                {
                    *stateTracker = inputEvent.value;
                }

                // Value 1 = pressed (high), 0 = released (low)
                eventHandler(inputEvent.value == 1);
            }

            // Continue waiting for next event
            waitForInputEvent(name, eventHandler, keyCode, event, stateTracker);
        });
}

bool PowerControl::requestInputEvents(
    const std::string& deviceName, const std::string& signalName,
    uint16_t keyCode, const std::function<void(bool)>& handler,
    boost::asio::posix::stream_descriptor& eventDescriptor, int* stateTracker)
{
    // Find the input device
    std::string eventPath = findInputEventDevice(deviceName);
    if (eventPath.empty())
    {
        lg2::error("Failed to find input device {DEVICE_NAME}", "DEVICE_NAME",
                   deviceName);
        return false;
    }

    // Open the event device
    int fd = open(eventPath.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0)
    {
        lg2::error("Failed to open {EVENT_PATH}: {ERROR}", "EVENT_PATH",
                   eventPath, "ERROR", strerror(errno));
        return false;
    }

    // Assign to the stream descriptor
    eventDescriptor.assign(fd);

    // Start waiting for events
    waitForInputEvent(signalName, handler, keyCode, eventDescriptor,
                      stateTracker);

    lg2::info(
        "Successfully set up input event monitoring for {SIGNAL_NAME} on {EVENT_PATH} with key code {KEY_CODE:#x}",
        "SIGNAL_NAME", signalName, "EVENT_PATH", eventPath, "KEY_CODE",
        keyCode);

    return true;
}

// Power Control Operations
// referenced by upstream power state handlers

void PowerControl::powerOn()
{
    auto powerOutIt = powerSignalMap.find("PowerOut");
    if (powerOutIt != powerSignalMap.end())
    {
        auto it = TimerMap.find("PowerPulseMs");
        if (it == TimerMap.end())
        {
            lg2::error("Timer config 'PowerPulseMs' not found in TimerMap");
            return;
        }
        assertGPIOForMs(powerOutIt->second, it->second);
    }
    else
    {
        lg2::error("PowerOut not found in powerSignalMap");
    }
}

void PowerControl::gracefulPowerOff()
{
    auto powerOutIt = powerSignalMap.find("PowerOut");
    if (powerOutIt != powerSignalMap.end())
    {
        auto it = TimerMap.find("PowerPulseMs");
        if (it == TimerMap.end())
        {
            lg2::error("Timer config 'PowerPulseMs' not found in TimerMap");
            return;
        }
        assertGPIOForMs(powerOutIt->second, it->second);
    }
    else
    {
        lg2::error("PowerOut not found in powerSignalMap");
    }
}

void PowerControl::forcePowerOff()
{
    auto powerOutIt = powerSignalMap.find("PowerOut");
    if (powerOutIt == powerSignalMap.end())
    {
        lg2::error("PowerOut not found in powerSignalMap");
        return;
    }

    auto it = TimerMap.find("ForceOffPulseMs");
    if (it == TimerMap.end())
    {
        lg2::error("Timer config 'ForceOffPulseMs' not found in TimerMap");
        return;
    }
    if (assertGPIOForMs(powerOutIt->second, it->second) < 0)
    {
        return;
    }

    // If the force off timer expires, then the power-button override failed
    gpioAssertTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error("Force power off async_wait failed: {ERROR_MSG}",
                           "ERROR_MSG", ec.message());
            }
            return;
        }

        lg2::error("Power-button override failed. Not sure what to do now.");
    });
}

void PowerControl::reset()
{
    auto resetOutIt = powerSignalMap.find("ResetOut");
    if (resetOutIt != powerSignalMap.end())
    {
        auto it = TimerMap.find("ResetPulseMs");
        if (it == TimerMap.end())
        {
            lg2::error("Timer config 'ResetPulseMs' not found in TimerMap");
            return;
        }
        assertGPIOForMs(resetOutIt->second, it->second);
    }
    else
    {
        lg2::error("ResetOut not found in powerSignalMap");
    }
}

// Operating System State Management

std::string_view PowerControl::getOperatingSystemStateStage(
    OperatingSystemStateStage stage) const
{
    switch (stage)
    {
        case OperatingSystemStateStage::Inactive:
            return "xyz.openbmc_project.State.OperatingSystem.Status.OSStatus.Inactive";
        case OperatingSystemStateStage::Standby:
            return "xyz.openbmc_project.State.OperatingSystem.Status.OSStatus.Standby";
        default:
            return "xyz.openbmc_project.State.OperatingSystem.Status.OSStatus.Inactive";
    }
}

void PowerControl::setOperatingSystemState(OperatingSystemStateStage stage)
{
    if (!osIface)
    {
        return;
    }

    operatingSystemState = stage;
#if IGNORE_SOFT_RESETS_DURING_POST
    // If POST complete has asserted set ignoreNextSoftReset to false to avoid
    // masking soft resets after POST
    if (operatingSystemState == OperatingSystemStateStage::Standby)
    {
        ignoreNextSoftReset = false;
    }
#endif
    osIface->set_property("OperatingSystemState",
                          std::string(getOperatingSystemStateStage(stage)));

    lg2::info("Moving os state to {STATE} stage", "STATE",
              getOperatingSystemStateStage(stage));
}

// D-Bus Property Management

int PowerControl::getProperty(std::shared_ptr<ConfigData> configData)
{
    std::variant<bool> resp;

    try
    {
        auto method = conn->new_method_call(
            configData->dbusName.c_str(), configData->path.c_str(),
            "org.freedesktop.DBus.Properties", "Get");
        method.append(configData->interface.c_str(),
                      configData->lineName.c_str());

        auto reply = conn->call(method);
        if (reply.is_method_error())
        {
            lg2::error(
                "Error reading {PROPERTY} D-Bus property on interface {INTERFACE} and path {PATH}",
                "PROPERTY", configData->lineName, "INTERFACE",
                configData->interface, "PATH", configData->path);
            return -1;
        }

        reply.read(resp);
    }
    catch (const sdbusplus::exception_t& e)
    {
        lg2::error("Exception while reading {PROPERTY}: {WHAT}", "PROPERTY",
                   configData->lineName, "WHAT", e.what());
        reschedulePropertyRead(configData);
        return -1;
    }

    auto respValue = std::get_if<bool>(&resp);
    if (!respValue)
    {
        lg2::error("Error: {PROPERTY} D-Bus property is not the expected type",
                   "PROPERTY", configData->lineName);
        return -1;
    }
    return (*respValue);
}

void PowerControl::reschedulePropertyRead(
    std::shared_ptr<ConfigData> configData)
{
    auto item = dBusRetryTimers.find(configData->name);

    if (item == dBusRetryTimers.end())
    {
        auto newItem = dBusRetryTimers.emplace(
            configData->name, boost::asio::steady_timer(ioContext));

        if (!newItem.second)
        {
            lg2::error("Failed to add new timer for {NAME}", "NAME",
                       configData->name);
            return;
        }

        item = newItem.first;
    }

    auto& timer = item->second;
    auto it = TimerMap.find("DbusGetPropertyRetry");
    if (it == TimerMap.end())
    {
        lg2::error("Timer config 'DbusGetPropertyRetry' not found in TimerMap");
        return;
    }
    timer.expires_after(std::chrono::milliseconds(it->second));
    timer.async_wait([this, configData](const boost::system::error_code ec) {
        if (ec)
        {
            lg2::error("Retry timer for {NAME} failed: {MSG}", "NAME",
                       configData->name, "MSG", ec.message());
            dBusRetryTimers.erase(configData->name);
            return;
        }

        int property = getProperty(configData);

        if (property >= 0)
        {
            setInitialValue(configData, (property > 0));
            dBusRetryTimers.erase(configData->name);
        }
    });
}

void PowerControl::setInitialValue(std::shared_ptr<ConfigData> configData,
                                   bool initialValue)
{
    if (configData->name == "PowerOk")
    {
        // Set power state based on PowerOk signal
        powerState = (initialValue ? PowerState::on : PowerState::off);
        hostIface->set_property("CurrentHostState",
                                std::string(getHostState()));
        lg2::info("PowerOk initial value: {VALUE}, power state set to {STATE}",
                  "VALUE", initialValue, "STATE", getHostState());
    }
    else if (configData->name == "PowerButton")
    {
        powerButtonIface->set_property("ButtonPressed", !initialValue);
    }
    else if (configData->name == "ResetButton")
    {
        resetButtonIface->set_property("ButtonPressed", !initialValue);
    }
    else if (configData->name == "NMIButton")
    {
        nmiButtonIface->set_property("ButtonPressed", !initialValue);
    }
    else if (configData->name == "IdButton")
    {
        idButtonIface->set_property("ButtonPressed", !initialValue);
    }
    else if (configData->name == "PostComplete")
    {
        // Look up PostComplete config to check polarity
        auto postCompleteIt = powerSignalMap.find("PostComplete");
        if (postCompleteIt != powerSignalMap.end())
        {
            OperatingSystemStateStage osState =
                (initialValue == postCompleteIt->second->polarity
                     ? OperatingSystemStateStage::Standby
                     : OperatingSystemStateStage::Inactive);
            setOperatingSystemState(osState);
        }
        // If PostComplete not in powerSignalMap, just skip silently
    }
    else
    {
        lg2::info("Unknown signal name {NAME} for setInitialValue", "NAME",
                  configData->name);
    }
}

void PowerControl::initializePowerStateFromHardware(
    const std::vector<std::string>& powerIndicatorSignals,
    bool requireAllAsserted)
{
    if (powerIndicatorSignals.empty())
    {
        lg2::error(
            "initializePowerStateFromHardware called with empty signal list");
        return;
    }

    std::vector<bool> signalStates;

    // Read all power indicator signals
    for (const auto& signalName : powerIndicatorSignals)
    {
        auto it = powerSignalMap.find(signalName);
        if (it == powerSignalMap.end())
        {
            lg2::error(
                "Power indicator signal '{SIGNAL}' not found in powerSignalMap",
                "SIGNAL", signalName);
            throw std::runtime_error(
                "Required power indicator not found: " + signalName);
        }

        // Verify GPIO line is available
        if (!it->second->gpioLine)
        {
            lg2::error(
                "Power indicator signal '{SIGNAL}' GPIO line not initialized",
                "SIGNAL", signalName);
            throw std::runtime_error(
                "Power indicator GPIO not initialized: " + signalName);
        }

        // Read current GPIO value
        int gpioValue = it->second->gpioLine.get_value();
        // Consider polarity: signal is asserted when GPIO value matches
        // polarity
        bool isAsserted = (gpioValue == it->second->polarity);
        signalStates.push_back(isAsserted);

        lg2::info("Power indicator '{SIGNAL}' is {STATE}", "SIGNAL", signalName,
                  "STATE", (isAsserted ? "asserted" : "de-asserted"));
    }

    // Determine power state based on signal states
    bool hostIsOn;
    if (requireAllAsserted)
    {
        // ALL signals must be asserted for host to be considered ON
        hostIsOn = std::all_of(signalStates.begin(), signalStates.end(),
                               [](bool state) { return state; });
    }
    else
    {
        // ANY signal asserted means host is ON
        hostIsOn = std::any_of(signalStates.begin(), signalStates.end(),
                               [](bool state) { return state; });
    }

    // Set power state based on hardware
    if (hostIsOn)
    {
        lg2::info(
            "Hardware indicates host is ON, initializing to PowerState::on");
        powerState = PowerState::on;
        setGPIOsForHostStateOn();
    }
    else
    {
        lg2::info(
            "Hardware indicates host is OFF, initializing to PowerState::off");
        powerState = PowerState::off;
        setGPIOsForHostStateOff();
    }

    // Update D-Bus interfaces to reflect actual hardware state
    hostIface->set_property("CurrentHostState", std::string(getHostState()));
    chassisIface->set_property("CurrentPowerState",
                               std::string(getChassisState()));
    chassisIface->set_property("LastStateChangeTime", getCurrentTimeMs());
}

void PowerControl::setGPIOsForHostStateOn()
{
    lg2::info("Setting GPIOs to default state for host state ON");

    // Iterate through powerSignalMap and set signals with non-NA defaults
    for (auto& [signalName, configData] : powerSignalMap)
    {
        if (configData->defaultStateHostStateOn != DefaultState::NA)
        {
            // Determine the GPIO value based on the default state and polarity
            int value;
            if (configData->defaultStateHostStateOn == DefaultState::Asserted)
            {
                value = configData->polarity;
            }
            else // DefaultState::DeAsserted
            {
                value = !configData->polarity;
            }

            lg2::debug(
                "Setting {SIGNAL} to {VALUE} for default state for host state ON",
                "SIGNAL", signalName, "VALUE", value);
            setGPIOOutput(configData, value);
        }
    }
}

void PowerControl::setGPIOsForHostStateOff()
{
    lg2::info("Setting GPIOs to default state for host state OFF");

    // Iterate through powerSignalMap and set signals with non-NA defaults
    for (auto& [signalName, configData] : powerSignalMap)
    {
        if (configData->defaultStateHostStateOff != DefaultState::NA)
        {
            // Determine the GPIO value based on the default state and polarity
            int value;
            if (configData->defaultStateHostStateOff == DefaultState::Asserted)
            {
                value = configData->polarity;
            }
            else // DefaultState::DeAsserted
            {
                value = !configData->polarity;
            }

            lg2::debug(
                "Setting {SIGNAL} to {VALUE} for default state for host state OFF",
                "SIGNAL", signalName, "VALUE", value);
            setGPIOOutput(configData, value);
        }
    }
}

} // namespace power_control
