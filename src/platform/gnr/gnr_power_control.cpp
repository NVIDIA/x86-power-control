/*
 * SPDX-License-Identifier: Apache-2.0
 * GNR platform: minimal power control (PowerOk, PowerOut, G3Soft sequence).
 */

#include "gnr_power_control.hpp"
#include "../../mctp_send.hpp"

#include <endian.h>

#include <nlohmann/json.hpp>
#include <phosphor-logging/lg2.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <type_traits>

namespace power_control
{
using Event = PowerControl::Event;

/** NVIDIA vendor-defined MCTP request header (architecture doc 3.1.1). */
struct __attribute__((__packed__)) GlacierVdmHeader
{
    uint8_t iana[4] = {0x00, 0x00, 0x16, 0x47}; // NVIDIA IANA 0x1647
    uint8_t request = 0x80;                     // Rq=1, D=0, instance ID 0
    uint8_t msgType = 0x01;                     // Glacier message family
    uint8_t commandCode;
    uint8_t msgVersion;
};

/** Boot Complete v2 (Glacier FW design doc 4.3.1.3). */
struct __attribute__((__packed__)) BootCompleteV2
{
    GlacierVdmHeader header{.commandCode = 0x02, .msgVersion = 0x02};
    // SLOT_REPORTING_VALID (bit 2) clear, so BOOT_SLOT (bits 1:0) is 2b'11:
    // the BMC does not report which slot the host booted from.
    uint8_t bootSlot = 0x03;
    uint8_t rsvd[2] = {0x00, 0x00};
};

/**
 * Add External Timestamp (Glacier FW design doc 4.3.1.18).
 *
 * Lets the erot log parser convert its boot-relative timestamps to wall clock.
 * The erot rejects any message version other than 1, and throttles callers to
 * 10 requests per 10 minutes.
 */
struct __attribute__((__packed__)) AddExternalTimestamp
{
    GlacierVdmHeader header{.commandCode = 0x13, .msgVersion = 0x01};
    uint64_t epochMicroseconds; // big endian
};

/** Restart (state change) Notification v2 (architecture doc 7.3). */
struct __attribute__((__packed__)) RestartNotificationV2
{
    GlacierVdmHeader header{.commandCode = 0x0a, .msgVersion = 0x02};
    uint8_t apState; // 0: restarting, 1: sleep (S3/S5), 2: G3Soft (DC cycle)
};
constexpr uint8_t apStateG3Soft = 0x02;

/** Copy a VDM struct into the caller's buffer for mctpSendAsync(). */
template <typename T>
void packVdm(const T& msg, std::array<uint8_t, sizeof(T)>& bytes)
{
    static_assert(std::is_trivially_copyable_v<T>,
                  "VDM message must be trivially copyable");
    static_assert(sizeof(bytes) == sizeof(T),
                  "VDM buffer must be exactly the size of the message");
    std::memcpy(bytes.data(), &msg, sizeof(T));
}

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
                    std::array<uint8_t, sizeof(BootCompleteV2)> packet;
                    packVdm(BootCompleteV2{}, packet);
                    sendVdm(packet, "boot-complete");
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

    // Anchor the erot log to wall clock once, as soon as we know where to send
    // it. The send is queued on ioContext and completes after run() starts.
    if (mctpEid)
    {
        sendTimestampVdm();
    }
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
            // A cycle enters G3Soft just like a power off does, so the erot
            // gets the same NVRAM/flash handling before the AP comes back and
            // powerOn()'s G3Soft exit has a matching entry.
            completePowerDown();
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
            completePowerDown();
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

void GNRPowerControl::sendTimestampVdm()
{
    uint64_t nowMicroseconds = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());

    AddExternalTimestamp timestamp{.epochMicroseconds =
                                       htobe64(nowMicroseconds)};
    std::array<uint8_t, sizeof(timestamp)> packet;
    packVdm(timestamp, packet);
    sendVdm(packet, "add-external-timestamp");
}

void GNRPowerControl::sendPowerOffVdm()
{
    // Tell the erot the AP is entering G3Soft so it asserts AP_RESET#, copies
    // NVRAM and revalidates flash before releasing the AP again.
    std::array<uint8_t, sizeof(RestartNotificationV2)> packet;
    packVdm(RestartNotificationV2{.apState = apStateG3Soft}, packet);
    sendVdm(packet, "power-off");
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
