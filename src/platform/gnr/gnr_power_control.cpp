/*
 * SPDX-License-Identifier: Apache-2.0
 * GNR platform: minimal power control (PowerOk, PowerOut, G3Soft sequence).
 */

#include "gnr_power_control.hpp"

#include <phosphor-logging/lg2.hpp>

#include <chrono>

namespace power_control
{
using Event = PowerControl::Event;

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

    validateRequiredSignals();
    validateTimerConfigs();
    setDefaultValues();

    initializeHostStateInterface();
    initializePowerStateFromHardware({"PowerOk"}, true);
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

bool GNRPowerControl::shouldRunG3SoftSequence()
{
    auto g3SoftEn = getSignal("G3SoftEn");
    auto ap0ResetN = getSignal("Ap0ResetN");
    auto pexResetN = getSignal("PexResetN");
    if (!g3SoftEn || !ap0ResetN || !pexResetN)
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

    lg2::info("GNR G3Soft: step 1 - driving G3SoftEn LOW");
    if (!setGPIOOutput(g3SoftEn, 0))
    {
        lg2::error("GNR G3Soft: failed to set G3SoftEn LOW");
        return;
    }

    gnrPowerOnPhase = GNRPowerOnPhase::WaitingAp0ResetN;
    gnrPowerOnStartTime = std::chrono::steady_clock::now();
    gnrPowerOnTimer.expires_after(g3SoftAp0Timeout);
    gnrPowerOnTimer.async_wait(
        std::bind_front(&GNRPowerControl::onGNRPowerOnTimer, this));
}

void GNRPowerControl::onGNRPowerOnTimer(const boost::system::error_code& ec)
{
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
                gnrPowerOnTimer.expires_after(g3SoftPowerButtonDelay);
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
                gnrPowerOnTimer.expires_after(g3SoftAp0Timeout);
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

            lg2::info(
                "GNR powerOn(): waiting {MS}ms after PEX reset before power button",
                "MS", g3SoftPowerButtonDelay.count());
            gnrPowerOnPhase = GNRPowerOnPhase::WaitingPowerButtonDelay;
            gnrPowerOnTimer.expires_after(g3SoftPowerButtonDelay);
            break;
        }
        case GNRPowerOnPhase::WaitingPowerButtonDelay:
            lg2::info("GNR powerOn(): pulsing power button (PowerOut)");
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
