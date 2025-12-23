#include "power_control_base.hpp"
#include <phosphor-logging/lg2.hpp>
#include <systemd/sd-journal.h>
#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <ctime>
#include <linux/input.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>


namespace power_control
{
// TODO: define virtual method
std::string PowerControl::getEventName(Event event)
{
    switch (event)
    {
        case Event::psPowerOKAssert:
            return "power supply power OK assert";
        case Event::psPowerOKDeAssert:
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
        case Event::psPowerOKWatchdogTimerExpired:
            return "power supply power OK watchdog timer expired";
        case Event::pdbMainPowerOkWatchdogTimerExpired:
            return "PDB main power OK watchdog timer expired";
        case Event::hpmPowerGoodWatchdogTimerExpired:
            return "HPM power good watchdog timer expired";
        case Event::cpuResetWatchdogTimerExpired:
            return "CPU reset watchdog timer expired";
        case Event::cpuShutdownOkWatchdogTimerExpired:
            return "CPU shutdown OK watchdog timer expired";
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
        case Event::gracefulPowerOffRequest:
            return "graceful power-off request";
        case Event::gracefulPowerCycleRequest:
            return "graceful power-cycle request";
        case Event::warmResetDetected:
            return "warm reset detected";
        case Event::nvl144pdbMainPowerOkAssert:
            return "NVL144 PDB main power OK assert";
        case Event::nvl144pdbMainPowerOkDeAssert:
            return "NVL144 PDB main power OK de-assert";
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
        default:
            return "unknown event: " + std::to_string(static_cast<int>(event));
    }
}

void PowerControl::logEvent(std::string_view stateHandler, Event event)
{
    lg2::info("{STATE_HANDLER}: {EVENT} event received", "STATE_HANDLER",
              stateHandler, "EVENT", getEventName(event));
}

PowerControl::PowerControl(boost::asio::io_context& ioContext,
                           std::shared_ptr<sdbusplus::asio::connection> conn,
                           const std::string& node)
    : ioContext(ioContext), conn(conn), nodeId(node), appName("power-control"),
      gpioAssertTimer(ioContext),
      powerCycleTimer(ioContext),
      gracefulPowerOffTimer(ioContext),
      warmResetCheckTimer(ioContext),
      psPowerOKWatchdogTimer(ioContext),
      sioPowerGoodWatchdogTimer(ioContext),
      powerStateSaveTimer(ioContext),
      pohCounterTimer(ioContext),
      restartCauseTimer(ioContext),
      slotPowerCycleTimer(ioContext)
{
    // Load configuration from JSON file and populate powerSignalMap
    loadConfigValues(ioContext);

    // Register base class GPIO handlers
    // These handlers are available in all platforms and can be overridden by derived classes
    gpioHandlerMap["PowerOk"] = [this](bool state) { this->psPowerOKHandler(state); };
    gpioHandlerMap["SioPowerGood"] = [this](bool state) { this->sioPowerGoodHandler(state); };
    gpioHandlerMap["SIOS5"] = [this](bool state) { this->sioS5Handler(state); };
    gpioHandlerMap["PowerButton"] = [this](bool state) { this->powerButtonHandler(state); };
    gpioHandlerMap["ResetButton"] = [this](bool state) { this->resetButtonHandler(state); };

    hostDbusName += node;
    chassisDbusName += node;
    osDbusName += node;
    buttonDbusName += node;
    nmiDbusName += node;
    rstCauseDbusName += node;

    // Request all the dbus names
    conn->request_name(hostDbusName.c_str());
    conn->request_name(chassisDbusName.c_str());
    conn->request_name(osDbusName.c_str());
    conn->request_name(buttonDbusName.c_str());
    conn->request_name(nmiDbusName.c_str());
    conn->request_name(rstCauseDbusName.c_str());

    // Initialize D-Bus interfaces
    initializeHostInterface();
    initializeChassisInterface();
    initializeBootProgressInterface();
#ifdef CHASSIS_SYSTEM_RESET
    initializeChassisSystemInterface();
#endif
    initializeButtonInterfaces();
    initializeOSInterface();
    initializeRestartCauseInterface();
}

std::function<void(Event)> PowerControl::getPowerStateHandler()
{
    // Map upstream PowerState values to their handler functions
    switch (powerState)
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

void PowerControl::loadConfigValues(boost::asio::io_context& io)
{
    // Dynamically build powerSignalMap from JSON config file
    
    // Determine config file path
    const std::string configFilePath =
        "/usr/share/x86-power-control/power-config-host" + node + ".json";
    
    std::ifstream configFile(configFilePath.c_str());
    if (!configFile.is_open())
    {
        lg2::error("loadConfigValues: Cannot open config path \'{PATH}\'",
                   "PATH", configFilePath);
        throw std::runtime_error("Failed to open config file: " + configFilePath);
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
        auto configPtr = std::make_shared<ConfigData>(io);
        configPtr->name = gpioName;
        
        // Parse Type
        if (!gpioConfig.contains("Type"))
        {
            lg2::error("The \'Type\' field must be defined in Json file");
            throw std::runtime_error("Missing 'Type' field for signal: " + gpioName);
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
                throw std::runtime_error("Missing 'LineName' for GPIO: " + gpioName);
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
                    throw std::runtime_error("Invalid polarity for: " + gpioName);
                }
            }
            else
            {
                lg2::error("Polarity field not found for {GPIO_NAME}",
                           "GPIO_NAME", configPtr->lineName);
                throw std::runtime_error("Missing 'Polarity' for GPIO: " + gpioName);
            }
        }
        else  // DBUS type
        {
            // Parse D-Bus specific fields
            std::map<std::string, std::string> dbusParams = {
                {"DbusName", "DbusName"},
                {"Path", "Path"},
                {"Interface", "Interface"},
                {"Property", "Property"}
            };
            
            for (auto& [key, dbusParamName] : dbusParams)
            {
                if (!gpioConfig.contains(dbusParamName))
                {
                    lg2::error(
                        "The {DBUS_NAME} field must be defined for Dbus configuration ",
                        "DBUS_NAME", dbusParamName);
                    throw std::runtime_error("Missing D-Bus field: " + dbusParamName);
                }
            }
            
            configPtr->dbusName = gpioConfig["DbusName"];
            configPtr->path = gpioConfig["Path"];
            configPtr->interface = gpioConfig["Interface"];
            configPtr->lineName = gpioConfig["Property"];  // Property name stored in lineName
            
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
                    throw std::runtime_error("Invalid MatchRegex for: " + gpioName);
                }
            }
        }
        
        // Add to powerSignalMap
        powerSignalMap[gpioName] = configPtr;
    }
    
    lg2::info("Successfully loaded {COUNT} signal configurations from JSON",
              "COUNT", powerSignalMap.size());
    
    // Load timer values from JSON config
    if (jsonData.contains("timers"))
    {
        auto timers = jsonData["timers"];
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
                    lg2::warning("Timer '{TIMER}' has non-integer value, skipping", 
                                 "TIMER", key);
                }
            }
            lg2::info("Successfully loaded {COUNT} timer configurations from JSON",
                      "COUNT", TimerMap.size());
        }
        else
        {
            lg2::warning("'timers' field in JSON is not an object, skipping timer loading");
        }
    }
    else
    {
        lg2::info("No 'timers' field found in JSON config, TimerMap will be empty");
    }
}

bool PowerControl::requestGPIOEvents(ConfigData& config)
{
    // Migrated from static function in power_control.cpp
    
    // Find the GPIO line
    config.gpioLine = gpiod::find_line(config.lineName);
    if (!config.gpioLine)
    {
        lg2::error("Failed to find the {GPIO_NAME} line", "GPIO_NAME", config.lineName);
        return false;
    }

    try
    {
        config.gpioLine.request({appName, gpiod::line_request::EVENT_BOTH_EDGES, {}});
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
        lg2::error("Failed to get {GPIO_NAME} fd", "GPIO_NAME", config.lineName);
        return false;
    }

    config.eventDescriptor.assign(gpioLineFd);

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
            bool gpioState = (line_event.event_type == gpiod::line_event::RISING_EDGE);
            
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

std::string_view PowerControl::getHostState(const PowerState state)
{
    // Upstream implementation - maps PowerState to D-Bus host state
    switch (state)
    {
        case PowerState::on:
        case PowerState::gracefulTransitionToOff:
        case PowerState::gracefulTransitionToCycleOff:
            return "xyz.openbmc_project.State.Host.HostState.Running";
            break;
        case PowerState::waitForPSPowerOK:
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

std::string_view PowerControl::getChassisState(const PowerState state)
{
    // Upstream implementation - maps PowerState to D-Bus chassis state
    switch (state)
    {
        case PowerState::on:
        case PowerState::transitionToOff:
        case PowerState::gracefulTransitionToOff:
        case PowerState::transitionToCycleOff:
        case PowerState::gracefulTransitionToCycleOff:
        case PowerState::checkForWarmReset:
            return "xyz.openbmc_project.State.Chassis.PowerState.On";
            break;
        case PowerState::waitForPSPowerOK:
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

std::string PowerControl::getPowerStateName(PowerState state)
{
    // Upstream implementation - only knows about upstream power states
    switch (state)
    {
        case PowerState::on:
            return "On";
            break;
        case PowerState::waitForPSPowerOK:
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
            return "unknown state: " + std::to_string(static_cast<int>(state));
            break;
    }
}

void PowerControl::logStateTransition(const PowerState state)
{
    lg2::info("Host{HOST}: Moving to \"{STATE}\" state", "HOST", nodeId, "STATE",
              this->getPowerStateName(state));
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
    // Note: This function still references the external global powerState variable
    // which will need to be refactored in the future.
    
    // Update global power state
    powerState = state;
    logStateTransition(state);

    // Update D-Bus host state (uses virtual dispatch)
    hostIface->set_property("CurrentHostState",
                            std::string(this->getHostState(powerState)));

    // Update D-Bus chassis state (uses virtual dispatch)
    chassisIface->set_property("CurrentPowerState",
                               std::string(this->getChassisState(powerState)));
    chassisIface->set_property("LastStateChangeTime", getCurrentTimeMs());

    // Reset boot progress to Unspecified when host powers off
    // TODO: Commented out for now - boot progress interface not created yet
    // if (state == PowerState::off)
    // {
    //     setBootProgress("xyz.openbmc_project.State.Boot.Progress.ProgressStages.Unspecified");
    // }

    // Save the power state for the restore policy
    // TODO: Commented out for now - will be implemented when powerStateSaveTimer 
    // and appState are moved to the base PowerControl class
    // savePowerState(state);
}

void PowerControl::initializeHostInterface()
{
    // Note: Button masking (powerButtonMask, resetButtonMask) and restart cause tracking
    // (addRestartCause) are not yet moved to the class, so those checks are commented out.
    
    // Create Host Interface
    sdbusplus::asio::object_server hostServer =
        sdbusplus::asio::object_server(*conn);

    hostIface =
        hostServer.add_interface("/xyz/openbmc_project/state/host" + nodeId,
                                 "xyz.openbmc_project.State.Host");
    
    // Interface for IPMI/Redfish initiated host state transitions
    hostIface->register_property(
        "RequestedHostTransition",
        std::string("xyz.openbmc_project.State.Host.Transition.Off"),
        [this](const std::string& requested, std::string& resp) {
            // Note: Button masking and restart cause tracking not yet implemented
            // TODO: Uncomment when powerButtonMask, resetButtonMask, and addRestartCause are moved
            
            if (requested == "xyz.openbmc_project.State.Host.Transition.Off")
            {
                // TODO: Check power button mask when implemented
                // if (!powerButtonMask)
                // {
                    // Use member function sendPowerControlEvent
                    sendPowerControlEvent(Event::gracefulPowerOffRequest);
                    // addRestartCause(RestartCause::command);
                    lg2::info("Host transition to Off requested");
                // }
                // else
                // {
                //     lg2::info("Power Button Masked.");
                //     throw std::invalid_argument("Transition Request Masked");
                //     return 0;
                // }

                // sendPowerControlEvent(Event::gracefulPowerOffRequest);
                // addRestartCause(RestartCause::command);
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Host.Transition.On")
            {
                // TODO: Check power button mask when implemented
                // if (!powerButtonMask)
                // {
                    sendPowerControlEvent(Event::powerOnRequest);
                    // addRestartCause(RestartCause::command);
                    lg2::info("Host transition to On requested");
                // }
                // else
                // {
                //     lg2::info("Power Button Masked.");
                //     throw std::invalid_argument("Transition Request Masked");
                //     return 0;
                // }
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Host.Transition.Reboot")
            {
                // TODO: Check power button mask when implemented
                // if (!powerButtonMask)
                // {
                    sendPowerControlEvent(Event::powerCycleRequest);
                    // addRestartCause(RestartCause::command);
                    lg2::info("Host transition to Reboot requested");
                // }
                // else
                // {
                //     lg2::info("Power Button Masked.");
                //     throw std::invalid_argument("Transition Request Masked");
                //     return 0;
                // }
            }
            else if (
                requested ==
                "xyz.openbmc_project.State.Host.Transition.GracefulWarmReboot")
            {
                // TODO: Check reset button mask when implemented
                // if (!resetButtonMask)
                // {
                    sendPowerControlEvent(Event::gracefulPowerCycleRequest);
                    // addRestartCause(RestartCause::command);
                    lg2::info("Host transition to GracefulWarmReboot requested");
                // }
                // else
                // {
                //     lg2::info("Reset Button Masked.");
                //     throw std::invalid_argument("Transition Request Masked");
                //     return 0;
                // }
            }
            else if (
                requested ==
                "xyz.openbmc_project.State.Host.Transition.ForceWarmReboot")
            {
                // TODO: Check reset button mask when implemented
                // if (!resetButtonMask)
                // {
                    sendPowerControlEvent(Event::resetRequest);
                    // addRestartCause(RestartCause::command);
                    lg2::info("Host transition to ForceWarmReboot requested");
                // }
                // else
                // {
                //     lg2::info("Reset Button Masked.");
                //     throw std::invalid_argument("Transition Request Masked");
                //     return 0;
                // }
            }
            else
            {
                lg2::error("Unrecognized host state transition request.");
                throw std::invalid_argument("Unrecognized Transition Request");
                return 0;
            }
            resp = requested;
            return 1;
        });
    
    hostIface->register_property("CurrentHostState",
                                 std::string(getHostState(powerState)));

    hostIface->initialize();

    lg2::info("Created the host interface successfully");
}

void PowerControl::initializeChassisInterface()
{
    // Create Chassis Interface
    sdbusplus::asio::object_server chassisServer =
        sdbusplus::asio::object_server(*conn);

    chassisIface =
        chassisServer.add_interface("/xyz/openbmc_project/state/chassis" + nodeId,
                                    "xyz.openbmc_project.State.Chassis");

    chassisIface->register_property(
        "RequestedPowerTransition",
        std::string("xyz.openbmc_project.State.Chassis.Transition.Off"),
        [this](const std::string& requested, std::string& resp) {
            // Note: Button masking and restart cause tracking not yet implemented
            // TODO: Uncomment when powerButtonMask and addRestartCause are moved
            
            if (requested == "xyz.openbmc_project.State.Chassis.Transition.Off")
            {
                // TODO: Check power button mask when implemented
                // if (!powerButtonMask)
                // {
                    sendPowerControlEvent(Event::powerOffRequest);
                    // addRestartCause(RestartCause::command);
                    lg2::info("Chassis transition to Off requested");
                // }
                // else
                // {
                //     lg2::info("Power Button Masked.");
                //     throw std::invalid_argument("Transition Request Masked");
                //     return 0;
                // }
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Chassis.Transition.On")
            {
                // TODO: Check power button mask when implemented
                // if (!powerButtonMask)
                // {
                    sendPowerControlEvent(Event::powerOnRequest);
                    // addRestartCause(RestartCause::command);
                    lg2::info("Chassis transition to On requested");
                // }
                // else
                // {
                //     lg2::info("Power Button Masked.");
                //     throw std::invalid_argument("Transition Request Masked");
                //     return 0;
                // }
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Chassis.Transition.PowerCycle")
            {
                // TODO: Check power button mask when implemented
                // if (!powerButtonMask)
                // {
                    sendPowerControlEvent(Event::powerCycleRequest);
                    // addRestartCause(RestartCause::command);
                    lg2::info("Chassis transition to PowerCycle requested");
                // }
                // else
                // {
                //     lg2::info("Power Button Masked.");
                //     throw std::invalid_argument("Transition Request Masked");
                //     return 0;
                // }
            }
            else
            {
                lg2::error("Unrecognized chassis state transition request.");
                throw std::invalid_argument("Unrecognized Transition Request");
                return 0;
            }
            resp = requested;
            return 1;
        });
    
    chassisIface->register_property("CurrentPowerState",
                                    std::string(getChassisState(powerState)));
    chassisIface->register_property("LastStateChangeTime", getCurrentTimeMs());

    chassisIface->initialize();

    lg2::info("Created the chassis interface successfully");
}

#ifdef CHASSIS_SYSTEM_RESET
void PowerControl::initializeChassisSystemInterface()
{
    // Chassis System Interface
    sdbusplus::asio::object_server chassisSysServer =
        sdbusplus::asio::object_server(*conn);

    chassisSysIface = chassisSysServer.add_interface(
        "/xyz/openbmc_project/state/chassis_system0",
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
                // addRestartCause(RestartCause::command);
                lg2::info("Chassis system PowerCycle requested");
            }
            else
            {
                lg2::error("Unrecognized chassis system state transition request.");
                throw std::invalid_argument("Unrecognized Transition Request");
                return 0;
            }
            resp = requested;
            return 1;
        });
    chassisSysIface->register_property(
        "CurrentPowerState", std::string(getChassisState(powerState)));
    chassisSysIface->register_property("LastStateChangeTime", getCurrentTimeMs());

    chassisSysIface->initialize();

    lg2::info("Created the chassis system interface successfully");
}
#endif

void PowerControl::initializeBootProgressInterface()
{
    // Boot Progress Interface
    // This interface allows external entities (IPMI, PLDM, etc.) to update boot progress
    sdbusplus::asio::object_server hostServer =
        sdbusplus::asio::object_server(*conn);
    
    bootProgressIface = 
        hostServer.add_interface("/xyz/openbmc_project/state/host" + nodeId,
                                 "xyz.openbmc_project.State.Boot.Progress");
    
    // BootProgress property - indicates the current boot stage
    bootProgressIface->register_property(
        "BootProgress",
        std::string("xyz.openbmc_project.State.Boot.Progress.ProgressStages.Unspecified"),
        [this](const std::string& requested, std::string& resp) {
            lg2::info("BootProgress updated to: {BOOT_PROGRESS}", "BOOT_PROGRESS", requested);
            resp = requested;
            
            // Update the timestamp when BootProgress changes
            auto now = std::chrono::system_clock::now();
            auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                now.time_since_epoch()).count();
            
            if (bootProgressIface)
            {
                bootProgressIface->set_property("BootProgressLastUpdate", 
                                               static_cast<uint64_t>(timestamp));
            }
            
            return 1;
        });
    
    // BootProgressLastUpdate property - timestamp of last update (microseconds since epoch)
    bootProgressIface->register_property(
        "BootProgressLastUpdate",
        static_cast<uint64_t>(0));
    
    // BootProgressOem property - OEM-specific boot progress information
    bootProgressIface->register_property(
        "BootProgressOem",
        std::string(""),
        [](const std::string& requested, std::string& resp) {
            lg2::info("BootProgressOem updated to: {OEM_PROGRESS}", "OEM_PROGRESS", requested);
            resp = requested;
            return 1;
        });
    
    bootProgressIface->initialize();
    
    lg2::info("Created the Boot.Progress interface successfully");
}

void PowerControl::initializeButtonInterfaces()
{
    // Buttons Service
    sdbusplus::asio::object_server buttonsServer =
        sdbusplus::asio::object_server(*conn);

    // Power Button Interface
    auto powerButtonConfig = powerSignalMap.find("PowerButton");
    if (powerButtonConfig != powerSignalMap.end() && 
        !powerButtonConfig->second->lineName.empty())
    {
        powerButtonIface = buttonsServer.add_interface(
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
                            throw std::runtime_error("Failed to request GPIO");
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
            powerButtonPressed = powerButtonConfig->second->gpioLine.get_value() == 0;
        }

        powerButtonIface->register_property("ButtonPressed", powerButtonPressed);
        powerButtonIface->initialize();
        
        lg2::info("Created the power button interface successfully");
    }

    // Reset Button Interface
    auto resetButtonConfig = powerSignalMap.find("ResetButton");
    if (resetButtonConfig != powerSignalMap.end() && 
        !resetButtonConfig->second->lineName.empty())
    {
        resetButtonIface = buttonsServer.add_interface(
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
                            throw std::runtime_error("Failed to request GPIO");
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
            resetButtonPressed = resetButtonConfig->second->gpioLine.get_value() == 0;
        }

        resetButtonIface->register_property("ButtonPressed", resetButtonPressed);
        resetButtonIface->initialize();
        
        lg2::info("Created the reset button interface successfully");
    }

    // NMI Button Interface
    auto nmiButtonConfig = powerSignalMap.find("NMIButton");
    if (nmiButtonConfig != powerSignalMap.end() && 
        nmiButtonConfig->second->gpioLine)
    {
        nmiButtonIface = buttonsServer.add_interface(
            "/xyz/openbmc_project/chassis/buttons/nmi",
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
            nmiButtonPressed = nmiButtonConfig->second->gpioLine.get_value() == 0;
        }

        nmiButtonIface->register_property("ButtonPressed", nmiButtonPressed);
        nmiButtonIface->initialize();
        
        lg2::info("Created the NMI button interface successfully");
    }

    // NMI Out Interface
    auto nmiOutConfig = powerSignalMap.find("NMIOut");
    if (nmiOutConfig != powerSignalMap.end() && 
        nmiOutConfig->second->gpioLine)
    {
        sdbusplus::asio::object_server nmiOutServer =
            sdbusplus::asio::object_server(*conn);

        nmiOutIface = nmiOutServer.add_interface(
            "/xyz/openbmc_project/control/host" + nodeId + "/nmi",
            "xyz.openbmc_project.Control.Host.NMI");
        
        // Note: nmiReset method would need to be implemented in the class
        // For now, register a placeholder that logs a message
        nmiOutIface->register_method("NMI", [this]() {
            lg2::info("NMI method called - not yet implemented in base class");
        });
        nmiOutIface->initialize();
        
        lg2::info("Created the NMI out interface successfully");
    }

    // ID Button Interface
    auto idButtonConfig = powerSignalMap.find("IdButton");
    if (idButtonConfig != powerSignalMap.end() && 
        idButtonConfig->second->gpioLine)
    {
        idButtonIface = buttonsServer.add_interface(
            "/xyz/openbmc_project/chassis/buttons/id",
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
    // OS State Service
    sdbusplus::asio::object_server osServer =
        sdbusplus::asio::object_server(*conn);

    // OS State Interface
    osIface = osServer.add_interface(
        "/xyz/openbmc_project/state/host" + nodeId,
        "xyz.openbmc_project.State.OperatingSystem.Status");

    // Default to Inactive state
    osIface->register_property(
        "OperatingSystemState",
        std::string("xyz.openbmc_project.State.OperatingSystem.Status.OSStatus.Inactive"));

    osIface->initialize();

    lg2::info("Created the OS state interface successfully");
}

void PowerControl::initializeRestartCauseInterface()
{
    // Restart Cause Service
    sdbusplus::asio::object_server restartCauseServer =
        sdbusplus::asio::object_server(*conn);

    // Restart Cause Interface
    restartCauseIface = restartCauseServer.add_interface(
        "/xyz/openbmc_project/control/host" + nodeId + "/restart_cause",
        "xyz.openbmc_project.Control.Host.RestartCause");

    restartCauseIface->register_property(
        "RestartCause",
        std::string("xyz.openbmc_project.State.Host.RestartCause.Unknown"));

    restartCauseIface->register_property(
        "RequestedRestartCause",
        std::string("xyz.openbmc_project.State.Host.RestartCause.Unknown"),
        [this](const std::string& requested, std::string& resp) {
            if (requested ==
                "xyz.openbmc_project.State.Host.RestartCause.WatchdogTimer")
            {
                // TODO: addRestartCause(RestartCause::watchdog);
                lg2::info("Restart cause watchdog requested");
            }
            else
            {
                throw std::invalid_argument(
                    "Unrecognized RestartCause Request");
                return 0;
            }

            lg2::info("RestartCause requested: {RESTART_CAUSE}",
                      "RESTART_CAUSE", requested);
            resp = requested;
            return 1;
        });

    restartCauseIface->initialize();

    lg2::info("Created the restart cause interface successfully");
}

bool PowerControl::setGPIOOutput(std::shared_ptr<ConfigData> config, const int value)
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
            lg2::error("Failed to find the {GPIO_NAME} line", "GPIO_NAME", config->lineName);
            return false;
        }
    }
    
    // Request GPIO output to specified value
    if (!config->gpioLine.is_requested())
    {
        try
        {
            config->gpioLine.request({appName, gpiod::line_request::DIRECTION_OUTPUT, {}},
                            value);
        }
        catch (const std::exception& e)
        {
            lg2::error("Failed to request {GPIO_NAME} output: {ERROR}", "GPIO_NAME",
                    config->lineName, "ERROR", e);
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
            lg2::error("Failed to set {GPIO_NAME} value: {ERROR}",
                       "GPIO_NAME", config->lineName, "ERROR", e);
            return false;
        }
    }

    lg2::info("{GPIO_NAME} set to {GPIO_VALUE}", "GPIO_NAME", config->lineName,
              "GPIO_VALUE", value);
    return true;
}

void PowerControl::startTimer(const std::string& timerName,
                               boost::asio::steady_timer& timer,
                               Event eventOnExpiry)
{
    // Look up timeout from TimerMap
    auto it = TimerMap.find(timerName);
    if (it == TimerMap.end())
    {
        lg2::error("Timer '{TIMER}' not found in TimerMap", "TIMER", timerName);
        throw std::runtime_error("Timer not found in TimerMap: " + timerName);
    }
    
    int timeoutMs = it->second;
    
    // Use the overloaded version with direct timeout
    startTimer(timeoutMs, timer, eventOnExpiry);
}

void PowerControl::startTimer(int timeoutMs,
                               boost::asio::steady_timer& timer,
                               Event eventOnExpiry)
{
    lg2::info("Timer started with {TIMEOUT_MS}ms timeout", "TIMEOUT_MS", timeoutMs);
    
    timer.expires_after(std::chrono::milliseconds(timeoutMs));
    timer.async_wait([this, eventOnExpiry](const boost::system::error_code& ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before completion
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error("Timer async_wait failed: {ERROR_MSG}",
                          "ERROR_MSG", ec.message());
            }
            lg2::info("Timer canceled");
            return;
        }
        
        lg2::info("Timer expired");
        sendPowerControlEvent(eventOnExpiry);
    });
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
}

void PowerControl::registerGPIOHandlers()
{
    lg2::info("Registering GPIO handlers from gpioHandlerMap");
    
    for (const auto& [signalName, handler] : gpioHandlerMap)
    {
        // Find the signal in powerSignalMap
        auto it = powerSignalMap.find(signalName);
        if (it == powerSignalMap.end())
        {
            lg2::error("GPIO signal '{SIGNAL}' not found in powerSignalMap", 
                      "SIGNAL", signalName);
            throw std::runtime_error("GPIO signal '" + signalName + 
                                    "' not found in powerSignalMap");
        }
        
        // Assign the handler to the ConfigData object
        it->second->gpioHandler = handler;
        
        // Check if this signal uses input event monitoring (gpio_keys_polled driver)
        if (it->second->useInputEvents)
        {
            // Validate input event configuration
            if (!it->second->inputEventConfig.has_value())
            {
                lg2::error("Signal '{SIGNAL}' is configured for input events but inputEventConfig is not set",
                          "SIGNAL", signalName);
                throw std::runtime_error("Signal '" + signalName + 
                                        "' missing inputEventConfig");
            }
            
            auto& inputConfig = it->second->inputEventConfig.value();
            
            // Request input events for this signal
            if (!requestInputEvents(inputConfig.deviceName,
                                   inputConfig.signalName,
                                   inputConfig.keyCode,
                                   handler,
                                   it->second->eventDescriptor,
                                   inputConfig.stateTracker))
            {
                lg2::error("Failed to register input events for '{SIGNAL}'", 
                          "SIGNAL", signalName);
                throw std::runtime_error("Failed to register input events for '" + 
                                        signalName + "'");
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
                throw std::runtime_error("Failed to register GPIO events for '" + 
                                        signalName + "'");
            }
            
            lg2::info("Successfully registered GPIO handler for '{SIGNAL}'", 
                     "SIGNAL", signalName);
        }
    }
    
    lg2::info("All GPIO handlers registered successfully");
}

// ===========================================================================
// GPIO Event Handlers
// ===========================================================================

void PowerControl::psPowerOKHandler(bool state)
{
    // Lookup config for polarity (guaranteed to exist since handler was registered)
    auto& config = *powerSignalMap["PowerOk"];
    
    Event powerControlEvent = (state == config.polarity)
                                  ? Event::psPowerOKAssert
                                  : Event::psPowerOKDeAssert;
    sendPowerControlEvent(powerControlEvent, powerState);
}

void PowerControl::sioPowerGoodHandler(bool state)
{
    // Lookup config for polarity (guaranteed to exist since handler was registered)
    auto& config = *powerSignalMap["SioPowerGood"];
    
    Event powerControlEvent = (state == config.polarity)
                                  ? Event::sioPowerGoodAssert
                                  : Event::sioPowerGoodDeAssert;
    sendPowerControlEvent(powerControlEvent, powerState);
}

void PowerControl::sioS5Handler(bool state)
{
    // Lookup config for polarity (guaranteed to exist since handler was registered)
    auto& config = *powerSignalMap["SIOS5"];
    
    Event powerControlEvent = (state == config.polarity)
                                  ? Event::sioS5Assert
                                  : Event::sioS5DeAssert;
    sendPowerControlEvent(powerControlEvent, powerState);
}

void PowerControl::powerButtonHandler(bool state)
{
    // Lookup config for polarity (guaranteed to exist since handler was registered)
    auto& config = *powerSignalMap["PowerButton"];
    
    bool asserted = state == config.polarity;
    powerButtonIface->set_property("ButtonPressed", asserted);
    if (asserted)
    {
        powerButtonPressLog();
        if (!powerButtonMask)
        {
            sendPowerControlEvent(Event::powerButtonPressed, powerState);
            addRestartCause(RestartCause::powerButton);
        }
        else
        {
            lg2::info("power button press masked");
        }
    }
#if USE_BUTTON_PASSTHROUGH
    // Note: powerOutConfig is not in base class yet, this will need to be handled
    // by derived classes if they use button passthrough
    lg2::info("Button passthrough not implemented in base class");
#endif
}

void PowerControl::resetButtonHandler(bool state)
{
    // Lookup config for polarity (guaranteed to exist since handler was registered)
    auto& config = *powerSignalMap["ResetButton"];
    
    bool asserted = state == config.polarity;
    resetButtonIface->set_property("ButtonPressed", asserted);
    if (asserted)
    {
        resetButtonPressLog();
        if (!resetButtonMask)
        {
            sendPowerControlEvent(Event::resetButtonPressed, powerState);
            addRestartCause(RestartCause::resetButton);
        }
        else
        {
            lg2::info("reset button press masked");
        }
    }
#if USE_BUTTON_PASSTHROUGH
    // Note: resetOutConfig is not in base class yet, this will need to be handled
    // by derived classes if they use button passthrough
    lg2::info("Button passthrough not implemented in base class");
#endif
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

// RestartCause Implementation (available in the power_control namespace)

// Initialize the global causeSet
boost::container::flat_set<RestartCause> causeSet;

std::string getRestartCause(RestartCause cause)
{
    switch (cause)
    {
        case RestartCause::command:
            return "xyz.openbmc_project.State.Host.RestartCause.IpmiCommand";
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

void addRestartCause(const RestartCause cause)
{
    // Add this to the set of causes for this restart
    causeSet.insert(cause);
}

void clearRestartCause()
{
    // Clear the set for the next restart
    causeSet.clear();
}

void setRestartCauseProperty(const std::string& cause)
{
    lg2::info("RestartCause set to {RESTART_CAUSE}", "RESTART_CAUSE", cause);
    PowerControl::restartCauseIface->set_property("RestartCause", cause);
}

void setRestartCause()
{
    // Determine the actual restart cause based on the set of causes
    std::string restartCause =
        "xyz.openbmc_project.State.Host.RestartCause.Unknown";
    if (causeSet.contains(RestartCause::watchdog))
    {
        restartCause = getRestartCause(RestartCause::watchdog);
    }
    else if (causeSet.contains(RestartCause::command))
    {
        restartCause = getRestartCause(RestartCause::command);
    }
    else if (causeSet.contains(RestartCause::resetButton))
    {
        restartCause = getRestartCause(RestartCause::resetButton);
    }
    else if (causeSet.contains(RestartCause::powerButton))
    {
        restartCause = getRestartCause(RestartCause::powerButton);
    }
    else if (causeSet.contains(RestartCause::powerPolicyOn))
    {
        restartCause = getRestartCause(RestartCause::powerPolicyOn);
    }
    else if (causeSet.contains(RestartCause::powerPolicyRestore))
    {
        restartCause = getRestartCause(RestartCause::powerPolicyRestore);
    }
    else if (causeSet.contains(RestartCause::softReset))
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

// INPUT EVENT HANDLING (for gpio_keys_polled driver)

std::string PowerControl::findInputEventDevice(const std::string& deviceName)
{
    // Search through /dev/input/eventX devices to find the one matching our name
    for (int i = 0; i < 32; i++)
    {
        std::string eventPath = "/dev/input/event" + std::to_string(i);
        std::string namePath = "/sys/class/input/event" + std::to_string(i) + "/device/name";
        
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
        [this, name, eventHandler, keyCode, &event, stateTracker](const boost::system::error_code ec) {
            if (ec)
            {
                lg2::error("{INPUT_NAME} fd handler error: {ERROR_MSG}",
                           "INPUT_NAME", name, "ERROR_MSG", ec.message());
                return;
            }
            
            struct input_event inputEvent;
            ssize_t bytesRead = read(event.native_handle(), &inputEvent, sizeof(inputEvent));
            
            if (bytesRead != sizeof(inputEvent))
            {
                if (bytesRead < 0)
                {
                    lg2::error("{INPUT_NAME} read error: {ERROR}",
                               "INPUT_NAME", name, "ERROR", strerror(errno));
                }
                else
                {
                    lg2::error("{INPUT_NAME} read error: incomplete event (got {BYTES} bytes)",
                               "INPUT_NAME", name, "BYTES", bytesRead);
                }
                waitForInputEvent(name, eventHandler, keyCode, event, stateTracker);
                return;
            }
            
            // We only care about EV_KEY events with our specific key code
            if (inputEvent.type == EV_KEY && inputEvent.code == keyCode)
            {
                lg2::info("{INPUT_NAME} event: code={KEY_CODE:#x} value={VALUE}",
                         "INPUT_NAME", name, "KEY_CODE", inputEvent.code, 
                         "VALUE", inputEvent.value);
                
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
    boost::asio::posix::stream_descriptor& eventDescriptor,
    int* stateTracker)
{
    // Find the input device
    std::string eventPath = findInputEventDevice(deviceName);
    if (eventPath.empty())
    {
        lg2::error("Failed to find input device {DEVICE_NAME}",
                   "DEVICE_NAME", deviceName);
        return false;
    }
    
    // Open the event device
    int fd = open(eventPath.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0)
    {
        lg2::error("Failed to open {EVENT_PATH}: {ERROR}",
                   "EVENT_PATH", eventPath, "ERROR", strerror(errno));
        return false;
    }
    
    // Assign to the stream descriptor
    eventDescriptor.assign(fd);
    
    // Start waiting for events
    waitForInputEvent(signalName, handler, keyCode, eventDescriptor, stateTracker);
    
    lg2::info("Successfully set up input event monitoring for {SIGNAL_NAME} on {EVENT_PATH} with key code {KEY_CODE:#x}",
             "SIGNAL_NAME", signalName, "EVENT_PATH", eventPath, "KEY_CODE", keyCode);
    
    return true;
}

} // namespace power_control

