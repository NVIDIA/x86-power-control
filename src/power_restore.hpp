// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#pragma once

#include "power_control_base.hpp"

#include <sys/sysinfo.h>
#include <systemd/sd-journal.h>

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <nlohmann/json.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <filesystem>
#include <fstream>
#include <list>
#include <string_view>

namespace match_rules = sdbusplus::bus::match::rules;

#ifdef USE_ACBOOT
// Defined in header so all TUs (including power_control_base.cpp) see them;
// inline avoids multiple-definition errors and works with LTO.
inline constexpr const char* powerACBootObject =
    "/xyz/openbmc_project/control/host0/ac_boot";
inline constexpr const char* powerACBootIface =
    "xyz.openbmc_project.Common.ACBoot";
#endif

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
 * @brief Persistent State Manager
 *
 * This manager stores runtime parameters that are supposed to be
 * persistent over BMC reboots. It provides a simple Get/Set interface and
 * handles default values, hardcoded in getDefault() method.
 * @note: currently only string parameters supported
 */
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
    PersistentState()
    {
        // create the power control directory if it doesn't exist
        std::error_code ec;
        if (!(std::filesystem::create_directories(powerControlDir, ec)))
        {
            if (ec.value() != 0)
            {
                lg2::error("failed to create {DIR_NAME}: {ERROR_MSG}",
                           "DIR_NAME", powerControlDir.string(), "ERROR_MSG",
                           ec.message());
                throw std::runtime_error("Failed to create state directory");
            }
        }

        // read saved state, it's ok, if the file doesn't exists
        std::ifstream appStateStream(powerControlDir / stateFile);
        if (!appStateStream.is_open())
        {
            lg2::info("Cannot open state file \'{PATH}\'", "PATH",
                      std::string(powerControlDir / stateFile));
            stateData = nlohmann::json({});
            return;
        }
        try
        {
            appStateStream >> stateData;
            if (stateData.is_discarded())
            {
                lg2::info("Cannot parse state file \'{PATH}\'", "PATH",
                          std::string(powerControlDir / stateFile));
                stateData = nlohmann::json({});
                return;
            }
        }
        catch (const std::exception& ex)
        {
            lg2::info("Cannot read state file \'{PATH}\'", "PATH",
                      std::string(powerControlDir / stateFile));
            stateData = nlohmann::json({});
            return;
        }
    }

    /**
     * @brief Persistent storage cleanup
     *
     * Class destructor automatically save state to JSON file
     */
    ~PersistentState()
    {
        saveState();
    }

    PersistentState(const PersistentState&) = delete;
    PersistentState& operator=(const PersistentState&) = delete;
    PersistentState(PersistentState&&) = delete;
    PersistentState& operator=(PersistentState&&) = delete;

    /**
     * @brief Get parameter value from the storage
     *
     * Get the parameter from cached storage. Default value returned, if
     * parameter was not set before.
     * @param parameter - parameter to get
     * @return parameter value
     */
    const std::string get(Params parameter)
    {
        auto val = stateData.find(getName(parameter));
        if (val != stateData.end())
        {
            return val->get<std::string>();
        }
        return getDefault(parameter);
    }

    /**
     * @brief Store parameter value
     *
     * Set the parameter value in cached storage and dump it to disk.
     * @param parameter - parameter to set
     * @param value - parameter value to assign
     */
    void set(Params parameter, const std::string& value)
    {
        stateData[getName(parameter)] = value;
        saveState();
    }

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
    const std::string getName(const Params parameter)
    {
        switch (parameter)
        {
            case Params::PowerState:
                return "PowerState";
        }
        return "";
    }

    /**
     * @brief Get default parameter value
     *
     * Get the default value, associated with given parameter.
     * @param parameter - parameter to get
     * @return parameter default value
     */
    const std::string getDefault(const Params parameter)
    {
        switch (parameter)
        {
            // For now we only store PowerState
            case Params::PowerState:
                return "";
        }
        return "";
    }

    /**
     * @brief Save cache to file on disk
     */
    void saveState()
    {
        std::ofstream appStateStream(powerControlDir / stateFile,
                                     std::ios::trunc);
        if (!appStateStream.is_open())
        {
            lg2::error("Cannot write state file \'{PATH}\'", "PATH",
                       std::string(powerControlDir / stateFile));
            return;
        }
        appStateStream << stateData.dump(indentationSize);
    }
};

// Forward declaration
class PowerControl;

/**
 * @brief The class contains functions to invoke power restore policy.
 *
 * This class only exists to unite all PowerRestore-related code. It supposed
 * to run only once on application startup.
 */
class PowerRestoreController
{
  public:
    PowerRestoreController(boost::asio::io_context& io,
                           std::shared_ptr<sdbusplus::asio::connection> conn,
                           const std::string& node,
                           PowerControl& powerControlRef,
                           PersistentState& appStateRef) :
        policyInvoked(false), powerRestoreDelay(-1), powerRestoreTimer(io),
        timerFired(false), conn(conn), node(node),
        powerControl(powerControlRef), appState(appStateRef)
    {}
    /**
     * @brief Power Restore entry point.
     *
     * Call this to start Power Restore algorithm.
     */
    void run()
    {
        std::string powerRestorePolicyObject =
            "/xyz/openbmc_project/control/host" + node +
            "/power_restore_policy";
        powerRestorePolicyLog();
        // this list only needs to be created once
        if (matches.empty())
        {
            matches.emplace_back(
                *conn,
                match_rules::interfacesAdded() +
                    match_rules::argNpath(0, powerRestorePolicyObject) +
                    match_rules::sender(settingsService),
                powerRestoreConfigHandler, this);
#ifdef USE_ACBOOT
            matches.emplace_back(
                *conn,
                match_rules::interfacesAdded() +
                    match_rules::argNpath(0, powerACBootObject) +
                    match_rules::sender(settingsService),
                powerRestoreConfigHandler, this);
            matches.emplace_back(*conn,
                                 match_rules::propertiesChanged(
                                     powerACBootObject, powerACBootIface) +
                                     match_rules::sender(settingsService),
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
            settingsService, powerRestorePolicyObject,
            "org.freedesktop.DBus.Properties", "GetAll",
            powerRestorePolicyIface);

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
            settingsService, powerACBootObject,
            "org.freedesktop.DBus.Properties", "GetAll", powerACBootIface);
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
    // D-Bus service and interface constants
    static constexpr const char* settingsService =
        "xyz.openbmc_project.Settings";
    static constexpr const char* powerRestorePolicyIface =
        "xyz.openbmc_project.Control.Power.RestorePolicy";

    bool policyInvoked;
    std::string powerRestorePolicy;
    int powerRestoreDelay;
    std::list<sdbusplus::bus::match_t> matches;
    boost::asio::steady_timer powerRestoreTimer;
    bool timerFired;
#ifdef USE_ACBOOT
    std::string acBoot;
#endif // USE_ACBOOT
    std::shared_ptr<sdbusplus::asio::connection> conn;
    std::string node;
    PowerControl& powerControl;
    PersistentState& appState;

    /**
     * @brief Log that power restore policy was applied
     *
     * Sends a Redfish event log entry for power restore policy application.
     */
    void powerRestorePolicyLog()
    {
        sd_journal_send("MESSAGE=PowerControl: power restore policy applied",
                        "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                        "OpenBMC.0.1.PowerRestorePolicyApplied", NULL);
    }

    /**
     * @brief D-Bus match callback for power restore configuration changes
     *
     * Handles InterfacesAdded and PropertiesChanged signals for power restore
     * policy configuration.
     *
     * @param m The sd_bus message
     * @param context Pointer to PowerRestoreController instance
     * @param error SD-Bus error (unused)
     * @return 1 on success
     */
    static int powerRestoreConfigHandler(sd_bus_message* m, void* context,
                                         sd_bus_error*)
    {
        if (context == nullptr || m == nullptr)
        {
            throw std::runtime_error("Invalid match");
        }
        sdbusplus::message_t message(m);
        PowerRestoreController* powerRestore =
            static_cast<PowerRestoreController*>(context);

        if (std::string(message.get_member()) == "InterfacesAdded")
        {
            sdbusplus::message::object_path path;
            boost::container::flat_map<std::string, dbusPropertiesList> data;

            message.read(path, data);

            for (auto& [iface, properties] : data)
            {
                if ((iface == powerRestorePolicyIface)
#ifdef USE_ACBOOT
                    || (iface == powerACBootIface)
#endif // USE_ACBOOT
                )
                {
                    powerRestore->setProperties(properties);
                }
            }
        }
        else if (std::string(message.get_member()) == "PropertiesChanged")
        {
            std::string interfaceName;
            dbusPropertiesList propertiesChanged;

            message.read(interfaceName, propertiesChanged);

            powerRestore->setProperties(propertiesChanged);
        }
        return 1;
    }

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
                lg2::info("Power Restore delay of {DELAY} seconds started",
                          "DELAY", delay);
                powerRestoreTimer.async_wait([this](
                                                 const boost::system::error_code
                                                     ec) {
                    if (ec)
                    {
                        // operation_aborted is expected if timer is canceled
                        // before completion.
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
            powerControl.sendPowerControlEvent(
                PowerControl::Event::powerOnRequest);
            powerControl.setRestartCauseProperty(
                getRestartCause(RestartCause::powerPolicyOn));
        }
        else if (
            powerRestorePolicy ==
            "xyz.openbmc_project.Control.Power.RestorePolicy.Policy.Restore")
        {
            int powerDropped = wasPowerDropped();
            if (powerDropped < 0)
            {
                lg2::error(
                    "Failed to read power state from persistent storage, do nothing");
            }
            else if (powerDropped > 0)
            {
                lg2::info("Restoring Host On state");
                powerControl.sendPowerControlEvent(
                    PowerControl::Event::powerOnRequest);
                powerControl.setRestartCauseProperty(
                    getRestartCause(RestartCause::powerPolicyRestore));
            }
            else
            {
                lg2::info("Restoring Host Off state");
                powerControl.sendPowerControlEvent(
                    PowerControl::Event::powerOffRequest);
                powerControl.setRestartCauseProperty(
                    getRestartCause(RestartCause::powerPolicyRestore));
            }
        }
        // We're done with the previous power state for the restore policy, so
        // store the current state
        savePowerState();
    }

    /**
     * @brief Save the current power state for restore policy
     *
     * Stores the current chassis power state to persistent storage.
     */
    void savePowerState()
    {
        std::string chassisState = std::string(powerControl.getChassisState());
        appState.set(PersistentState::Params::PowerState, chassisState);
    }
    /**
     * @brief Check if power was dropped.
     *
     * Read last saved power state to determine if host power was enabled before
     * last BMC reboot.
     */
    int wasPowerDropped()
    {
        std::string state = appState.get(PersistentState::Params::PowerState);
        lg2::info("Power state from persistent storage: {STATE}", "STATE",
                  state);
        if (state.empty())
        {
            return -1;
        }
        return state == "xyz.openbmc_project.State.Chassis.PowerState.On";
    }
};

} // namespace power_control
