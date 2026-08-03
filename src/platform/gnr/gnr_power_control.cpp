/*
 * SPDX-License-Identifier: Apache-2.0
 * GNR platform: minimal power control (PowerOk, PowerOut, G3Soft sequence).
 */

#include "gnr_power_control.hpp"
#include "../../mctp_send.hpp"

#include <nlohmann/json.hpp>
#include <phosphor-logging/lg2.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <fstream>
#include <stdexcept>

namespace power_control
{
using Event = PowerControl::Event;

std::optional<uint8_t> GNRPowerControl::loadMctpEid()
{
    std::ifstream configFile(configFilePath.c_str());
    if (!configFile.is_open())
    {
        lg2::error("GNR: cannot open config path '{PATH}' to read mctp_eid",
                   "PATH", configFilePath);
        throw std::runtime_error(
            "Failed to open config file: " + configFilePath);
    }

    auto jsonData = nlohmann::json::parse(configFile, nullptr, true, true);
    auto it = jsonData.find("mctp_eid");
    if (it == jsonData.end())
    {
        return std::nullopt;
    }

    if (!it->is_number_integer())
    {
        lg2::error("GNR: 'mctp_eid' must be an integer");
        throw std::runtime_error("Invalid 'mctp_eid' in config file");
    }

    auto eid = it->get<int>();
    if (eid < mctpMinEid || eid > mctpMaxEid)
    {
        lg2::error("GNR: 'mctp_eid' {EID} outside assignable range {MIN}-{MAX}",
                   "EID", eid, "MIN", mctpMinEid, "MAX", mctpMaxEid);
        throw std::runtime_error("Invalid 'mctp_eid' in config file");
    }

    lg2::info("GNR: using MCTP destination eid {EID}", "EID", eid);
    return static_cast<uint8_t>(eid);
}

void GNRPowerControl::sendVdm(std::span<const uint8_t> packet,
                              const std::string& what)
{
    if (!mctpEid)
    {
        lg2::error("GNR: no mctp_eid configured, not sending {WHAT} VDM",
                   "WHAT", what);
        return;
    }

    lg2::info("GNR: sending {WHAT} VDM to eid {EID} ({LEN} bytes)", "WHAT",
              what, "EID", *mctpEid, "LEN", packet.size());
    mctpSendAsync(ioContext, *mctpEid, 0x7f, packet,
                  [what](MctpResult result) {
        if (!result)
            lg2::error("GNR: {WHAT} VDM failed: {ERR}", "WHAT", what, "ERR",
                       result.error().message());
        else
            lg2::info("GNR: {WHAT} VDM acknowledged ({LEN} bytes)", "WHAT",
                      what, "LEN", result->size());
    });
}

std::chrono::milliseconds GNRPowerControl::getTimeoutWithDefault(
    const std::string& key, std::chrono::milliseconds default_t)
{
    auto it = TimerMap.find(key);
    if (it != TimerMap.end())
    {
        return std::chrono::milliseconds(it->second);
    }
    return default_t;
}

GNRPowerControl::GNRPowerControl(
    boost::asio::io_context& ioContext,
    std::shared_ptr<sdbusplus::asio::connection> conn,
    const std::string& configFilePath, const std::string& node,
    PersistentState& appState) :
    PowerControl(ioContext, conn, node, appState, configFilePath),
    gnrPowerOnTimer(ioContext)
{
    using namespace std::chrono_literals;
    // Only require PowerOk and PowerOut; G3Soft GPIOs are optional in JSON
    addRequiredSignal("PowerOk", 0, GPIODirection::IN,
                      [this](bool state) { powerOKHandler(state); });
    addRequiredSignal("PowerOut", 0, GPIODirection::OUT);

    // PostComplete is optional — not all GNR boards have this GPIO.
    // Registration failure must not take down power control (PowerOk/PowerOut).
    if (powerSignalMap.count("PostComplete"))
    {
        addRequiredSignal(
            "PostComplete", 0, GPIODirection::IN,
            [this](bool state) {
                auto it = powerSignalMap.find("PostComplete");
                if (it == powerSignalMap.end())
                    return;
                bool asserted = (state == it->second->polarity);
                setOperatingSystemState(
                    asserted ? OperatingSystemStateStage::Standby
                             : OperatingSystemStateStage::Inactive);
                // Deliberately not sending postComplete*Assert
                // events: the base handlePowerStateOn() treats a
                // POST Complete de-assert as a platform reset and
                // moves to checkForWarmReset, which is an empty
                // stub and therefore a dead-end state. On GNR this
                // signal only reports OS state and gates the erot
                // boot-complete notification.
                if (asserted)
                {
                    static constexpr std::array<uint8_t, 11> bootCompletePacket = {
                        0x00, 0x00, 0x16, 0x47,
                        0x80, 0x01, 0x02, 0x02, 0x03, 0x00, 0x00};
                    sendVdm(bootCompletePacket, "boot-complete");
                }
            },
            /*optional=*/true);
    }

    validateRequiredSignals();
    validateTimerConfigs();
    setDefaultValues();

    // The G3Soft sequence brackets its GPIO steps with erot notifications, so
    // a board wired for it must also declare where those VDMs go.
    mctpEid = loadMctpEid();
    if (hasG3SoftSignals() && !mctpEid)
    {
        lg2::error(
            "GNR: G3Soft GPIOs are configured but 'mctp_eid' is missing from {PATH}",
            "PATH", configFilePath);
        throw std::runtime_error(
            "Missing 'mctp_eid' required by the G3Soft sequence");
    }

    // Determine power state from hardware before exposing interfaces to D-Bus,
    // so the initial published values are correct.
    initializePowerStateFromHardware({"PowerOk"}, true);
    initializeHostStateInterface();
    g3SoftAp0Timeout = getTimeoutWithDefault("G3SoftAp0TimeoutMs", 15s);
    g3SoftPowerButtonDelay =
        getTimeoutWithDefault("G3SoftPowerButtonDelayMs", 10s);
    pexResetPulse = getTimeoutWithDefault("PexResetPulseMs", 10ms);
}

void GNRPowerControl::validateTimerConfigs() {}

void GNRPowerControl::setDefaultValues()
{
    lg2::info("Initializing default values for GNR output signals");

    // look at moving default states for power on to base class
    auto powerOut = powerSignalMap.find("PowerOut");
    if (powerOut != powerSignalMap.end())
    {
        powerOut->second->defaultStateHostStateOn = DefaultState::DeAsserted;
        powerOut->second->defaultStateHostStateOff = DefaultState::DeAsserted;
    }
}

void GNRPowerControl::handlePowerStateOn(Event event)
{
    if (event != Event::gracefulResetRequest)
    {
        PowerControl::handlePowerStateOn(event);

        if (event == Event::powerOKDeAssert)
        {
            // A spurious power loss takes the same path as a commanded
            // power off.
            lg2::info("GNR: unexpected power loss, running power-down flow");
            completePowerDown();
        }
        return;
    }

    lg2::info(
        "Graceful warm reboot requested; starting graceful power cycle");
    setPowerState(PowerState::gracefulTransitionToCycleOff);
    startTimer("GracefulPowerOffS", gracefulPowerOffTimer,
               Event::gracefulPowerOffTimerExpired);
    gracefulPowerOff();
}

void GNRPowerControl::handlePowerStateOff(Event event)
{
    switch (event)
    {
        case Event::powerOKAssert:
            // System has powered on (PowerOk asserted); update state so
            // D-Bus/Redfish power status reflects On.
            setPowerState(PowerState::on);
            break;
        case Event::powerOnRequest:
            powerOn();
            break;
        case Event::powerCycleRequest:
        case Event::gracefulPowerCycleRequest:
            powerOn();
            break;
        case Event::powerButtonPressed:
            break;
        case Event::gracefulResetRequest:
            lg2::info(
                "Graceful warm reboot requested while host is off; no action");
            break;
        case Event::resetRequest:
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

void GNRPowerControl::handleTransitionToOff(Event event)
{
    switch (event)
    {
        case Event::powerOKDeAssert:
            setPowerState(PowerState::off);
            completePowerDown();
            break;
        default:
            lg2::info("GNR handleTransitionToOff: no action for event");
            break;
    }
}

void GNRPowerControl::handleGracefulTransitionToOff(Event event)
{
    switch (event)
    {
        case Event::powerOKDeAssert:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::off);
            completePowerDown();
            break;
        case Event::gracefulPowerOffTimerExpired:
            setPowerState(PowerState::transitionToOff);
            forcePowerOff();
            break;
        default:
            lg2::info("GNR handleGracefulTransitionToOff: no action for event");
            break;
    }
}

void GNRPowerControl::handleTransitionToCycleOff(Event event)
{
    switch (event)
    {
        case Event::powerOKDeAssert:
            setPowerState(PowerState::cycleOff);
            startTimer("PowerCycleMs", powerCycleTimer,
                       Event::powerCycleTimerExpired);
            break;
        default:
            lg2::info("GNR handleTransitionToCycleOff: no action for event");
            break;
    }
}

void GNRPowerControl::handleGracefulTransitionToCycleOff(Event event)
{
    switch (event)
    {
        case Event::powerOKDeAssert:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::cycleOff);
            startTimer("PowerCycleMs", powerCycleTimer,
                       Event::powerCycleTimerExpired);
            break;
        case Event::gracefulPowerOffTimerExpired:
            setPowerState(PowerState::transitionToCycleOff);
            forcePowerOff();
            break;
        default:
            lg2::info(
                "GNR handleGracefulTransitionToCycleOff: no action for event");
            break;
    }
}

void GNRPowerControl::handleCycleOff(Event event)
{
    switch (event)
    {
        case Event::powerCycleTimerExpired:
            powerOn();
            break;
        case Event::powerOKAssert:
            setPowerState(PowerState::on);
            powerCycleTimer.cancel();
            break;
        default:
            lg2::info("GNR handleCycleOff: no action for event");
            break;
    }
}

bool GNRPowerControl::isSystemPowerOff()
{
    auto powerOk = getSignal("PowerOk");
    if (!powerOk)
    {
        lg2::info(
            "GNR isSystemPowerOff(): PowerOk not configured, assuming off");
        return true;
    }
    int val = readGPIOInputValue(powerOk);
    if (val < 0)
    {
        lg2::warning(
            "GNR isSystemPowerOff(): failed to read PowerOk, assuming off");
        return true;
    }
    // PowerOk asserted (1) = system has power; deasserted (0) = system off
    return (val == 0);
}

void GNRPowerControl::sendPowerOffVdm()
{
    static constexpr std::array<uint8_t, 9> powerOffNotification = {
        0x00, 0x00, 0x16, 0x47, 0x80, 0x01, 0x0a, 0x02, 0x02};
    sendVdm(powerOffNotification, "power-off");
}

void GNRPowerControl::completePowerDown()
{
    // Notify the erot that we are powering down, then drive the system into G3.
    sendPowerOffVdm();

    auto g3SoftEn = getSignal("G3SoftEn");
    if (g3SoftEn)
    {
        lg2::info("GNR power-off: asserting G3SoftEn");
        if (!setGPIOOutput(g3SoftEn, g3SoftEn->polarity))
        {
            lg2::error("GNR power-off: failed to assert G3SoftEn");
        }
    }

    auto pexResetN = getSignal("PexResetN");
    if (pexResetN)
    {
        lg2::info("GNR power-off: asserting PexResetN");
        if (!setGPIOOutput(pexResetN, pexResetN->polarity))
        {
            lg2::error("GNR power-off: failed to assert PexResetN");
        }
    }
}

void GNRPowerControl::startPowerButtonDelay()
{
    lg2::info("GNR power-on: step 4 - waiting {MS}ms before power button",
              "MS", g3SoftPowerButtonDelay.count());
    gnrPowerOnPhase = GNRPowerOnPhase::WaitingPowerButtonDelay;
    gnrPowerOnTimer.expires_after(g3SoftPowerButtonDelay);
    gnrPowerOnTimer.async_wait(
        std::bind_front(&GNRPowerControl::onGNRPowerOnTimer, this));
}

void GNRPowerControl::powerOn()
{
    lg2::info("GNR powerOn() entered");

    if (!shouldRunG3SoftSequence())
    {
        lg2::info(
            "GNR powerOn(): skipping G3Soft sequence, pulsing power button");
        PowerControl::powerOn();
        return;
    }

    startGNRPowerOnSequence();
}

void GNRPowerControl::initiateGracefulShutdown()
{
    // Match handlePowerStateOn(gracefulPowerOffRequest): brief PowerOut pulse
    // and wait for PowerOk to drop (or GracefulPowerOffS timeout -> forceful).
    setPowerState(PowerState::gracefulTransitionToOff);
    startTimer("GracefulPowerOffS", gracefulPowerOffTimer,
               Event::gracefulPowerOffTimerExpired);
    gracefulPowerOff();
}

void GNRPowerControl::initiateForcefulShutdown()
{
    // Match handlePowerStateOn(powerOffRequest): long PowerOut hold.
    setPowerState(PowerState::transitionToOff);
    forcePowerOff();
}

bool GNRPowerControl::hasG3SoftSignals()
{
    return getSignal("G3SoftEn") && getSignal("Ap0ResetN") &&
           getSignal("PexResetN");
}

bool GNRPowerControl::shouldRunG3SoftSequence()
{
    if (!hasG3SoftSignals())
    {
        lg2::info("GNR G3Soft: not configured (missing G3SoftEn/Ap0ResetN/"
                  "PexResetN), skipping sequence");
        return false;
    }

    auto powerOk = getSignal("PowerOk");
    if (powerOk)
    {
        int pwrOkVal = readGPIOInputValue(powerOk);
        if (pwrOkVal == 1)
        {
            lg2::info("PowerOk already asserted, skipping G3Soft sequence");
            return false;
        }
    }
    return true;
}

void GNRPowerControl::startGNRPowerOnSequence()
{
    auto g3SoftEn = getSignal("G3SoftEn");
    auto ap0ResetN = getSignal("Ap0ResetN");
    auto pexResetN = getSignal("PexResetN");
    if (!g3SoftEn || !ap0ResetN || !pexResetN)
    {
        lg2::error("GNR G3Soft: required signals missing");
        return;
    }

    lg2::info("GNR power-on: step 1 - de-asserting G3SoftEn");
    if (!setGPIOOutput(g3SoftEn, !g3SoftEn->polarity))
    {
        lg2::error("GNR power-on: failed to de-assert G3SoftEn");
        return;
    }

    lg2::info("GNR power-on: step 2 - polling Ap0ResetN until de-asserted");
    gnrPowerOnPhase = GNRPowerOnPhase::WaitingAp0ResetN;
    gnrPowerOnStartTime = std::chrono::steady_clock::now();
    gnrPowerOnTimer.expires_after(std::min(
        std::chrono::milliseconds(ap0PollIntervalMs), g3SoftAp0Timeout));
    gnrPowerOnTimer.async_wait(
        std::bind_front(&GNRPowerControl::onGNRPowerOnTimer, this));
}

void GNRPowerControl::onGNRPowerOnTimer(const boost::system::error_code& ec)
{
    if (ec == boost::asio::error::operation_aborted)
    {
        // The timer was re-armed or cancelled by another path (e.g. the
        // Ap0ResetN interrupt starting the power button delay). That path owns
        // the phase, so leave it alone.
        return;
    }
    if (ec)
    {
        if (ec != boost::asio::error::operation_aborted)
        {
            lg2::error("GNR power-on timer error: {ERROR}", "ERROR",
                       ec.message());
        }
        gnrPowerOnPhase = GNRPowerOnPhase::Idle;
        return;
    }

    switch (gnrPowerOnPhase)
    {
        case GNRPowerOnPhase::WaitingAp0ResetN:
        {
            auto ap0ResetN = getSignal("Ap0ResetN");

            auto elapsed = std::chrono::steady_clock::now() -
                           gnrPowerOnStartTime;
            int val = readGPIOInputValue(ap0ResetN);
            if (val == 1)
            {
                lg2::info(
                    "GNR G3Soft: step 2 - Ap0ResetN reached HIGH after {MS}ms",
                    "MS",
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        elapsed)
                        .count());
                auto pexResetN = getSignal("PexResetN");
                lg2::info(
                    "GNR G3Soft: step 3 - pulsing PexResetN LOW for {MS}ms",
                    "MS", pexResetPulse.count());
                if (!setGPIOOutput(pexResetN, 0))
                {
                    lg2::error("GNR G3Soft: failed to set PexResetN LOW");
                    gnrPowerOnPhase = GNRPowerOnPhase::Idle;
                    return;
                }
                gnrPowerOnPhase = GNRPowerOnPhase::WaitingPexResetPulse;
                gnrPowerOnTimer.expires_after(pexResetPulse);
            }
            else if (elapsed >= g3SoftAp0Timeout)
            {
                lg2::error("GNR G3Soft: Ap0ResetN did not assert within {MS}ms",
                           "MS", g3SoftAp0Timeout.count());
                gnrPowerOnPhase = GNRPowerOnPhase::Idle;
                return;
            }
            else
            {
                const auto remaining =
                    g3SoftAp0Timeout -
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        elapsed);
                gnrPowerOnTimer.expires_after(std::min(
                    std::chrono::milliseconds(ap0PollIntervalMs), remaining));
            }
            break;
        }
        case GNRPowerOnPhase::WaitingPexResetPulse:
        {
            auto pexResetN = getSignal("PexResetN");
            if (!setGPIOOutput(pexResetN, 1))
            {
                lg2::error("GNR G3Soft: failed to set PexResetN HIGH");
                gnrPowerOnPhase = GNRPowerOnPhase::Idle;
                return;
            }

            lg2::info("GNR G3Soft: PexResetN pulsed done ({MS}ms)", "MS",
                      pexResetPulse.count());

            startPowerButtonDelay();
            return;
        }
        case GNRPowerOnPhase::WaitingPowerButtonDelay:
            lg2::info("GNR power-on: step 5 - pulsing power button");
            gnrPowerOnPhase = GNRPowerOnPhase::Idle;
            PowerControl::powerOn();
            return;
        default:
            gnrPowerOnPhase = GNRPowerOnPhase::Idle;
            return;
    }

    gnrPowerOnTimer.async_wait(
        std::bind_front(&GNRPowerControl::onGNRPowerOnTimer, this));
}

int GNRPowerControl::readGPIOInputValue(std::shared_ptr<ConfigData> config)
{
    if (!config || config->lineName.empty())
    {
        return -1;
    }
    if (config->gpioLine && config->gpioLine.is_requested())
    {
        return config->gpioLine.get_value();
    }
    gpiod::line line = gpiod::find_line(config->lineName);
    if (!line)
    {
        lg2::error("readGPIOInputValue: Failed to find line {GPIO_NAME}",
                   "GPIO_NAME", config->lineName);
        return -1;
    }
    try
    {
        line.request({appName, gpiod::line_request::DIRECTION_INPUT, {}});
        int value = line.get_value();
        line.release();
        return value;
    }
    catch (const std::exception& e)
    {
        lg2::error("readGPIOInputValue {GPIO_NAME}: {ERROR}", "GPIO_NAME",
                   config->lineName, "ERROR", e);
        return -1;
    }
}

} // namespace power_control
