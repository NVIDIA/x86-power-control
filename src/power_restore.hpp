/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#pragma once

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <nlohmann/json.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <filesystem>
#include <list>
#include <string_view>

namespace power_control
{

/**
 * @brief Persistent State Manager
 *
 * This manager supposed to store runtime parameters that supposed to be
 * persistent over BMC reboot. It provides simple Get/Set interface and handle
 * default values, hardcoded in getDefault() method.
 * @note: currently only string parameters supported
 */
using dbusPropertiesList =
    boost::container::flat_map<std::string,
                               std::variant<std::string, uint64_t>>;
/**
 * @brief The class contains functions to invoke power restore policy.
 *
 * This class only exists to unite all PowerRestore-related code. It supposed
 * to run only once on application startup.
 */
class PowerRestoreController
{
  public:
    PowerRestoreController(boost::asio::io_context& io) :
        policyInvoked(false), powerRestoreDelay(-1), powerRestoreTimer(io),
        timerFired(false)
    {}
    /**
     * @brief Power Restore entry point.
     *
     * Call this to start Power Restore algorithm.
     */
     void run()
     {
         std::string powerRestorePolicyObject =
             "/xyz/openbmc_project/control/host" + node + "/power_restore_policy";
         powerRestorePolicyLog();
         // this list only needs to be created once
         if (matches.empty())
         {
             matches.emplace_back(
                 *conn,
                 match_rules::interfacesAdded() +
                     match_rules::argNpath(0, powerRestorePolicyObject) +
                     match_rules::sender(setingsService),
                 powerRestoreConfigHandler, this);
     #ifdef USE_ACBOOT
             matches.emplace_back(*conn,
                                  match_rules::interfacesAdded() +
                                      match_rules::argNpath(0, powerACBootObject) +
                                      match_rules::sender(setingsService),
                                  powerRestoreConfigHandler, this);
             matches.emplace_back(*conn,
                                  match_rules::propertiesChanged(powerACBootObject,
                                                                 powerACBootIface) +
                                      match_rules::sender(setingsService),
                                  powerRestoreConfigHandler, this);
     #endif // USE_ACBOOT
         }
     
         // Check if it's already on DBus
         conn->async_method_call(
             [this](boost::system::error_code ec,
                    const dbusPropertiesList properties) {
                 if (ec)
                 {
                     return;
                 }
                 setProperties(properties);
             },
             setingsService, powerRestorePolicyObject,
             "org.freedesktop.DBus.Properties", "GetAll", powerRestorePolicyIface);
     
     #ifdef USE_ACBOOT
         // Check if it's already on DBus
         conn->async_method_call(
             [this](boost::system::error_code ec,
                    const dbusPropertiesList properties) {
                 if (ec)
                 {
                     return;
                 }
                 setProperties(properties);
             },
             setingsService, powerACBootObject, "org.freedesktop.DBus.Properties",
             "GetAll", powerACBootIface);
     #endif
     }
    /**
     * @brief Initialize configuration parameters.
     *
     * Parse list of properties, received from dbus, to set Power Restore
     * algorithm configuration.
     * @param props - map of property names and values
     */
     void setProperties(const dbusPropertiesList& props)
     {
         for (auto& [property, propValue] : props)
         {
             if (property == "PowerRestorePolicy")
             {
                 const std::string* value = std::get_if<std::string>(&propValue);
                 if (value == nullptr)
                 {
                     lg2::error("Unable to read Power Restore Policy");
                     continue;
                 }
                 powerRestorePolicy = *value;
             }
             else if (property == "PowerRestoreDelay")
             {
                 const uint64_t* value = std::get_if<uint64_t>(&propValue);
                 if (value == nullptr)
                 {
                     lg2::error("Unable to read Power Restore Delay");
                     continue;
                 }
                 powerRestoreDelay = *value / 1000000; // usec to sec
             }
     #ifdef USE_ACBOOT
             else if (property == "ACBoot")
             {
                 const std::string* value = std::get_if<std::string>(&propValue);
                 if (value == nullptr)
                 {
                     lg2::error("Unable to read AC Boot status");
                     continue;
                 }
                 acBoot = *value;
             }
     #endif // USE_ACBOOT
         }
         invokeIfReady();
     }

  private:
    bool policyInvoked;
    std::string powerRestorePolicy;
    int powerRestoreDelay;
    std::list<sdbusplus::bus::match_t> matches;
    boost::asio::steady_timer powerRestoreTimer;
    bool timerFired;
#ifdef USE_ACBOOT
    std::string acBoot;
#endif // USE_ACBOOT

    /**
     * @brief Check if all required algorithms parameters are set
     *
     * Call this after set any of Power Restore algorithm parameters. Once all
     * parameters are set this will run invoke() function.
     */
     void invokeIfReady()
     {
         if ((powerRestorePolicy.empty()) || (powerRestoreDelay < 0))
         {
             return;
         }
     #ifdef USE_ACBOOT
         if (acBoot.empty() || acBoot == "Unknown")
         {
             return;
         }
     #endif
     
         matches.clear();
         if (!timerFired)
         {
             // Calculate the delay from now to meet the requested delay
             // Subtract the approximate uboot time
             static constexpr const int ubootSeconds = 20;
             int delay = powerRestoreDelay - ubootSeconds;
             // Subtract the time since boot
             struct sysinfo info = {};
             if (sysinfo(&info) == 0)
             {
                 delay -= info.uptime;
             }
     
             if (delay > 0)
             {
                 powerRestoreTimer.expires_after(std::chrono::seconds(delay));
                 lg2::info("Power Restore delay of {DELAY} seconds started", "DELAY",
                           delay);
                 powerRestoreTimer.async_wait([this](const boost::system::error_code
                                                         ec) {
                     if (ec)
                     {
                         // operation_aborted is expected if timer is canceled before
                         // completion.
                         if (ec == boost::asio::error::operation_aborted)
                         {
                             return;
                         }
                         lg2::error(
                             "power restore policy async_wait failed: {ERROR_MSG}",
                             "ERROR_MSG", ec.message());
                     }
                     else
                     {
                         lg2::info("Power Restore delay timer expired");
                     }
                     invoke();
                 });
                 timerFired = true;
             }
             else
             {
                 invoke();
             }
         }
     }
    /**
     * @brief Actually perform power restore actions.
     *
     * Take Power Restore actions according to Policy and other parameters.
     */
     void invoke()
     {
         // we want to run Power Restore only once
         if (policyInvoked)
         {
             return;
         }
         policyInvoked = true;
     
         lg2::info("Invoking Power Restore Policy {POLICY}", "POLICY",
                   powerRestorePolicy);
         if (powerRestorePolicy ==
             "xyz.openbmc_project.Control.Power.RestorePolicy.Policy.AlwaysOn")
         {
             sendPowerControlEvent(Event::powerOnRequest);
             setRestartCauseProperty(getRestartCause(RestartCause::powerPolicyOn));
         }
         else if (powerRestorePolicy ==
                  "xyz.openbmc_project.Control.Power.RestorePolicy.Policy.Restore")
         {
             if (wasPowerDropped())
             {
                 lg2::info("Power was dropped, restoring Host On state");
                 sendPowerControlEvent(Event::powerOnRequest);
                 setRestartCauseProperty(
                     getRestartCause(RestartCause::powerPolicyRestore));
             }
             else
             {
                 lg2::info("No power drop, restoring Host Off state");
             }
         }
         // We're done with the previous power state for the restore policy, so store
         // the current state
         savePowerState(powerState);
     }
    /**
     * @brief Check if power was dropped.
     *
     * Read last saved power state to determine if host power was enabled before
     * last BMC reboot.
     */
     bool wasPowerDropped()
     {
         std::string state = appState.get(PersistentState::Params::PowerState);
         return state == "xyz.openbmc_project.State.Chassis.PowerState.On";
     }
};

class PersistentState
{
  public:
    /**
     * List of all supported parameters
     */
    enum class Params
    {
        PowerState,
    };

    /**
     * @brief Persistent storage initialization
     *
     * Class constructor automatically load last state from JSON file
     */
    PersistentState();
    /**
     * @brief Persistent storage cleanup
     *
     * Class destructor automatically save state to JSON file
     */
    ~PersistentState();
    /**
     * @brief Get parameter value from the storage
     *
     * Get the parameter from cached storage. Default value returned, if
     * parameter was not set before.
     * @param parameter - parameter to get
     * @return parameter value
     */
    const std::string get(Params parameter);
    /**
     * @brief Store parameter value
     *
     * Set the parameter value in cached storage and dump it to disk.
     * @param parameter - parameter to set
     * @param value - parameter value to assign
     */
    void set(Params parameter, const std::string& value);

  private:
    nlohmann::json stateData;
    const std::filesystem::path powerControlDir = "/var/lib/power-control";
    const std::string_view stateFile = "state.json";
    const int indentationSize = 2;

    /**
     * @brief Covert parameter ID to name
     *
     * Get the name corresponding to the given parameter.
     * String name only used by the manager internal to generate human-readable
     * JSON.
     * @param parameter - parameter to convert
     * @return parameter name
     */
    const std::string getName(const Params parameter);
    /**
     * @brief Get default parameter value
     *
     * Get the default value, associated with given parameter.
     * @param parameter - parameter to get
     * @return parameter default value
     */
    const std::string getDefault(const Params parameter);
    /**
     * @brief Save cache to file on disk
     */
    void saveState();
};

} // namespace power_control
