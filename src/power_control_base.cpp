#include "power_control_base.hpp"
#include <phosphor-logging/lg2.hpp>
#include <fstream>
#include <nlohmann/json.hpp>

namespace power_control
{

PowerControl::PowerControl(boost::asio::io_context& ioContext)
    : ioContext(ioContext)
{
    // Load configuration from JSON file and populate powerSignalMap
    loadConfigValues(ioContext);
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
    // Note: 'node' global variable from power_control.cpp - for now hardcode to "0"
    // TODO: Pass node as parameter or make it accessible
    std::string node = "0";  // Default to host 0
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
        config.gpioLine.request({"x86-power-control", gpiod::line_request::EVENT_BOTH_EDGES, {}});
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

} // namespace power_control

