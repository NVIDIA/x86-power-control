#include "power_control_base.hpp"
#include <phosphor-logging/lg2.hpp>
#include <fstream>
#include <nlohmann/json.hpp>
#include <ctime>

namespace power_control
{
    extern PowerState powerState;
}


namespace power_control
{
// Initialize static members
std::shared_ptr<sdbusplus::asio::dbus_interface> PowerControl::hostIface = nullptr;
std::shared_ptr<sdbusplus::asio::dbus_interface> PowerControl::chassisIface = nullptr;

PowerControl::PowerControl(boost::asio::io_context& ioContext, const std::string& configFilePath, std::string node = "0")
    : ioContext(ioContext), conn(std::make_shared<sdbusplus::asio::connection>(ioContext)), nodeId(node), appName("power-control"),
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
    loadConfigValues(ioContext, configFilePath);
    
    // Load configuration from JSON file and populate powerSignalMap

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
    initializeDBusInterfaces(conn, nodeId);
}

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

void PowerControl::initializeDBusInterfaces(std::shared_ptr<sdbusplus::asio::connection> conn,
                                             const std::string& node)
{
    // Note: This function still references the external global powerState variable
    // Button masking (powerButtonMask, resetButtonMask) and restart cause tracking
    // (addRestartCause) are not yet moved to the class, so those checks are commented out.
    
    // Create Host Interface
    sdbusplus::asio::object_server hostServer =
        sdbusplus::asio::object_server(*conn);

    hostIface =
        hostServer.add_interface("/xyz/openbmc_project/state/host" + node,
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
                    sendPowerControlEvent(Event::gracefulPowerOffRequest, powerState);
                    // addRestartCause(RestartCause::command);
                    lg2::info("Host transition to Off requested");
                // }
                // else
                // {
                //     lg2::info("Power Button Masked.");
                //     throw std::invalid_argument("Transition Request Masked");
                //     return 0;
                // }

                // sendPowerControlEvent(Event::gracefulPowerOffRequest, powerState);
                // addRestartCause(RestartCause::command);
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Host.Transition.On")
            {
                // TODO: Check power button mask when implemented
                // if (!powerButtonMask)
                // {
                    sendPowerControlEvent(Event::powerOnRequest, powerState);
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
                    sendPowerControlEvent(Event::powerCycleRequest, powerState);
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
                    sendPowerControlEvent(Event::gracefulPowerCycleRequest, powerState);
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
                    sendPowerControlEvent(Event::resetRequest, powerState);
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

    // Create Chassis Interface
    sdbusplus::asio::object_server chassisServer =
        sdbusplus::asio::object_server(*conn);

    chassisIface =
        chassisServer.add_interface("/xyz/openbmc_project/state/chassis" + node,
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
                    sendPowerControlEvent(Event::powerOffRequest, powerState);
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
                    sendPowerControlEvent(Event::powerOnRequest, powerState);
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
                    sendPowerControlEvent(Event::powerCycleRequest, powerState);
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
        sendPowerControlEvent(eventOnExpiry, powerState);
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

} // namespace power_control

