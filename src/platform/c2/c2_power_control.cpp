// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "c2_power_control.hpp"

#include <phosphor-logging/lg2.hpp>

#include <filesystem>
#include <fstream>

namespace power_control
{
// Type aliases for convenience
using Event = PowerControl::Event;

namespace
{
constexpr auto cpuBootDoneMarker = "/run/bmc-state/CPU_BOOT_DONE-I";
constexpr auto cpuBootDoneService = "cpu-boot-done.service";
constexpr auto cpuBootUndoneService = "cpu-boot-undone.service";
} // namespace

// Constructor
C2PowerControl::C2PowerControl(
    boost::asio::io_context& ioContext,
    std::shared_ptr<sdbusplus::asio::connection> conn,
    const std::string& configFilePath, const std::string& node,
    PersistentState& appState) :
    VRPowerControl(ioContext, conn, configFilePath, node, appState),
    pdbPSUPowerOkWatchdogTimer(ioContext)
{
    // Required resources (IOX paths). Declare before signals because the
    // required GPIO signals live on these IOXs — if the IOX path is missing
    // from config, no signal on it can be valid.
    addRequiredResource("Board0IoxPath", ResourceType::IOXPath);
    addRequiredResource("PdbIoxPath", ResourceType::IOXPath);

    // VRPowerControl constructor already registers:
    //   Board0 VR signals (RunPowerEnable, RunPowerPG, PreSystemReset,
    //   CpuShutdownForce/Request/Ok, CpuResetIndicator).
    // C2 is 1P — no Board 1 signals needed. C2 has no PDBMainPowerEnable;
    // the PDB PSU enable signal is PDBPSUPowerOn (registered below).
    // USBPowerEnable is C2-specific, registered here.
    // Add C2-specific PDB signals here.

    addRequiredSignal("PDBMainPowerOk", 0, GPIODirection::IN,
                      [this](bool state) {
                          this->pdbMainPowerOkHandler(state);
                      });
    addRequiredSignal("StbyPwrOk", 0, GPIODirection::IN,
                      [this](bool state) { this->stbyPwrOkHandler(state); });
    addRequiredSignal("CpuBootDone", 0, GPIODirection::IN,
                      [this](bool state) { this->cpuBootDoneHandler(state); });
    addRequiredSignal("PDBPSUPowerOn", 0, GPIODirection::OUT);
    addRequiredSignal("PDBPSUPowerOk", 0, GPIODirection::IN,
                      [this](bool state) {
                          this->pdbPSUPowerOkHandler(state);
                      });
    addRequiredSignal("PDB12V_HPM-AICEnable", 0, GPIODirection::OUT);
    addRequiredSignal("PDB12V_GPU1Enable", 0, GPIODirection::OUT);
    addRequiredSignal("PDB12V_GPU2_Enable", 0, GPIODirection::OUT);
    addRequiredSignal("USBPowerEnable", 0, GPIODirection::OUT);

    // Validate required resources first (IOX paths) then signals on them.
    PowerControl::validateRequiredResources();
    PowerControl::validateRequiredSignals();

    // Validate all required timers
    validateTimerConfigs();

    // Set default values for output signals
    setDefaultValues();

    // Determine power state from hardware before exposing interfaces to D-Bus,
    // so the initial published values are correct.
    // Host is ON only if ALL THREE indicators are asserted.
    initializePowerStateFromHardware(powerIndicators, true);

    // Match phosphor-gpio-monitor's former ExecuteAtStart behavior without
    // injecting an artificial edge into the power-control state machine.
    auto cpuBootDoneSignal = getSignal("CpuBootDone");
    if (cpuBootDoneSignal && cpuBootDoneSignal->gpioLine)
    {
        updateCpuBootDoneFromGpio(cpuBootDoneSignal->gpioLine.get_value(),
                                  false);
    }

    // Initialize all host0 interfaces — makes the path visible to ObjectMapper.
    // Called after initializePowerStateFromHardware so the correct state is
    // published immediately on InterfacesAdded.
    initializeHostStateInterface();
}

void C2PowerControl::cpuBootDoneHandler(bool state)
{
    updateCpuBootDoneFromGpio(state, true);
}

void C2PowerControl::updateCpuBootDoneFromGpio(bool state,
                                               bool notifyStateMachine)
{
    auto config = getSignal("CpuBootDone");
    if (!config)
    {
        return;
    }

    const bool asserted = (state == config->polarity);
    lg2::info(
        "CpuBootDone GPIO event: raw value={VALUE}, logical state={STATE}",
        "VALUE", static_cast<int>(state), "STATE",
        asserted ? "ASSERTED" : "DE-ASSERTED");

    // Notify the FSM before starting non-critical systemd actions.
    if (!updateCpuBootDoneState(asserted ? 1 : 0, notifyStateMachine))
    {
        return;
    }

    setOperatingSystemState(asserted ? OperatingSystemStateStage::Standby
                                     : OperatingSystemStateStage::Inactive);
    updateCpuBootDoneMarker(asserted);

    startSystemdUnit(asserted ? cpuBootDoneService : cpuBootUndoneService);
}

void C2PowerControl::updateCpuBootDoneMarker(bool asserted)
{
    const std::filesystem::path marker(cpuBootDoneMarker);
    std::error_code ec;

    if (asserted)
    {
        std::filesystem::create_directories(marker.parent_path(), ec);
        if (ec)
        {
            lg2::error("Failed to create CPU Boot Done state directory: {ERR}",
                       "ERR", ec.message());
            return;
        }

        std::ofstream markerFile(marker);
        if (!markerFile)
        {
            lg2::error("Failed to create CPU Boot Done marker {PATH}", "PATH",
                       marker.string());
        }
        return;
    }

    std::filesystem::remove(marker, ec);
    if (ec)
    {
        lg2::error("Failed to remove CPU Boot Done marker {PATH}: {ERR}",
                   "PATH", marker.string(), "ERR", ec.message());
    }
}

// ============================================================================
// C2-specific state transition helpers
// ============================================================================

void C2PowerControl::assert12VRailsAndWaitForPDBMainPowerOk()
{
    auto pdb12vHPMAICEnable = getSignal("PDB12V_HPM-AICEnable");
    if (!pdb12vHPMAICEnable)
    {
        return;
    }

    auto pdb12vGPU1Enable = getSignal("PDB12V_GPU1Enable");
    if (!pdb12vGPU1Enable)
    {
        return;
    }

    auto pdb12vGPU2Enable = getSignal("PDB12V_GPU2_Enable");
    if (!pdb12vGPU2Enable)
    {
        return;
    }

    lg2::info(
        "PDB PSU Power OK asserted. Asserting 12V rails in order: GPU1, GPU2, HPM-AIC. "
        "Starting PDB Main Power OK Watchdog Timer. "
        "Transitioning to PowerState::waitForPDBMainPowerOk.");
    setGPIOOutput(pdb12vGPU1Enable, pdb12vGPU1Enable->polarity);
    setGPIOOutput(pdb12vGPU2Enable, pdb12vGPU2Enable->polarity);
    setGPIOOutput(pdb12vHPMAICEnable, pdb12vHPMAICEnable->polarity);
    startTimer("PdbMainPowerOkWatchdogMs", pdbMainPowerOkWatchdogTimer,
               Event::pdbMainPowerOkWatchdogTimerExpired);
    setPowerState(PowerState::waitForPDBMainPowerOk);
}

void C2PowerControl::handlePDBPSUPowerOkWatchdogExpired()
{
    lg2::error(
        "PDB PSU Power OK watchdog timer expired. PDB PSU Power On Sequence Failed. "
        "Host Power On sequence failed. Conducting Cleanup Sequence: Setting GPIO states "
        "to match Host State OFF.");
    action = PowerAction::NONE;
    logResourceEvent(
        "ResourceErrorsDetected",
        {"Host0", "PDB PSU Power OK Watchdog Timer Expired (power on)"},
        "xyz.openbmc_project.Logging.Entry.Level.Error");
    setGPIOsForHostStateOff();
    setPowerState(PowerState::off);
}

// ============================================================================
// C2-specific GPIO handler
// ============================================================================

void C2PowerControl::pdbMainPowerOkHandler(bool state)
{
    lg2::info("PDBMainPowerOk GPIO event: value={VALUE}", "VALUE",
              static_cast<int>(state));

    auto configPtr = getSignal("PDBMainPowerOk");
    if (!configPtr)
    {
        return;
    }

    Event powerControlEvent = (state == configPtr->polarity)
                                  ? Event::pdbMainPowerOkAssert
                                  : Event::pdbMainPowerOkDeAssert;

    if (powerControlEvent == Event::pdbMainPowerOkAssert)
    {
        pdbMainPowerOkDeassertedDuringRecovery = false;
    }
    else if (powerState == PowerState::waitForCpuRecovery)
    {
        pdbMainPowerOkDeassertedDuringRecovery = true;
    }

    // Check for power faults and handle if detected
    if (checkAndHandlePdbMainPowerOkFault(powerControlEvent))
    {
        return; // Fault was handled, exit early
    }

    this->sendPowerControlEvent(powerControlEvent);
}

// pdbMainPowerOkHandler Helper Function
bool C2PowerControl::checkAndHandlePdbMainPowerOkFault(Event powerControlEvent)
{
    // Power fault detection: Check for unexpected de-assertion
    if (powerControlEvent == Event::pdbMainPowerOkDeAssert)
    {
        if (powerState != PowerState::waitForPDBMainPowerOff &&
            powerState != PowerState::waitForCpuRecovery)
        {
            // POWER FAULT: PDB Main Power OK de-asserted unexpectedly
            lg2::error(
                "POWER FAULT DETECTED: PDBMainPowerOk de-asserted unexpectedly while in power state {STATE}. "
                "Setting GPIO states to match Host State OFF. Transitioning to Host State OFF.",
                "STATE", getPowerStateName());

            // Transition to off, checking if we need to wait for de-assertion
            action = PowerAction::NONE;
            logResourceEvent(
                "ResourceErrorsDetected",
                {"Host0",
                 "PDB Main Power OK de-asserted unexpectedly while in power state {STATE}.",
                 "STATE", getPowerStateName()},
                "xyz.openbmc_project.Logging.Entry.Level.Error");
            transitionToOffStateWithRunPowerCheck();

            return true;
        }
        // else: Expected de-assertion in waitForPDBMainPowerOff state, or a
        // recovery-session fault — let it fall through to
        // sendPowerControlEvent() so VRPowerControl::handleWaitForCpuRecovery()
        // can release Board0PreSystemReset before driving the same cleanup.
    }

    // Return false to indicate normal processing should continue
    return false;
}

void C2PowerControl::pdbPSUPowerOkHandler(bool state)
{
    lg2::info("PDBPSUPowerOk GPIO event: value={VALUE}", "VALUE",
              static_cast<int>(state));

    auto pdbPSUPowerOk = getSignal("PDBPSUPowerOk");
    if (!pdbPSUPowerOk || !pdbPSUPowerOk->gpioLine)
    {
        lg2::error("CRITICAL: PDBPSUPowerOk not available");
        return;
    }

    Event powerControlEvent = (state == pdbPSUPowerOk->polarity)
                                  ? Event::pdbPSUPowerOkAssert
                                  : Event::pdbPSUPowerOkDeAssert;

    this->sendPowerControlEvent(powerControlEvent);
}

void C2PowerControl::stbyPwrOkHandler(bool state)
{
    auto configPtr = getSignal("StbyPwrOk");
    if (!configPtr)
    {
        lg2::error("CRITICAL: StbyPwrOk signal not available");
        return;
    }

    const bool asserted = (state == configPtr->polarity);

    if (!asserted)
    {
        markStandbyLost();
        return;
    }

    // Re-assert. Recovery (re-arming GPIO lines and clearing stbyPowerLost)
    // is intentionally not handled in this commit — operator intervention
    // (AC cycle or BMC reboot) is required.
    lg2::info(
        "12V HPM standby power domain restored - AC cycle recommended to recover.");
    logResourceEvent(
        "ResourceEvent",
        {"Host0",
         "12V HPM standby power domain restored - AC cycle recommended to recover."},
        "xyz.openbmc_project.Logging.Entry.Level.Informational");
}

bool C2PowerControl::canAcceptPowerOnRequest(std::string& reason)
{
    if (stbyPowerLost)
    {
        reason =
            "System in degraded state due to prior standby power loss - AC cycle required to recover power sequencing";
        return false;
    }
    return true;
}

bool C2PowerControl::shouldIgnoreEvent(const std::string& signalName)
{
    // Always deliver StbyPwrOk so we can observe state changes on the
    // witness itself.
    if (signalName == "StbyPwrOk")
    {
        return false;
    }

    // If standby is already known lost, suppress.
    if (stbyPowerLost)
    {
        return true;
    }

    // Standby flag isn't set yet, but a noise event from a dying IOX may
    // surface BEFORE the StbyPwrOk de-assert event due to kernel per-chip
    // event delivery ordering. Look at the witness directly so the flag
    // is set eagerly, before this event's handler runs on a dead line.
    auto stby = getSignal("StbyPwrOk");
    if (stby && stby->gpioLine)
    {
        int value = 0;
        try
        {
            value = stby->gpioLine.get_value();
        }
        catch (const std::exception&)
        {
            return false;
        }

        if (value != stby->polarity)
        {
            markStandbyLost();
            return true;
        }
    }

    return false;
}

void C2PowerControl::markStandbyLost()
{
    if (stbyPowerLost)
    {
        return;
    }
    stbyPowerLost = true;

    lg2::error(
        "12V HPM standby power domain lost - power sequencing hardware unavailable. AC cycle recommended to recover.");
    logResourceEvent(
        "ResourceErrorsDetected",
        {"Host0",
         "12V HPM standby power domain lost - power sequencing hardware unavailable. AC cycle recommended to recover."},
        "xyz.openbmc_project.Logging.Entry.Level.Error");

    setPowerState(PowerState::off);
}

// ============================================================================
// getPowerStateHandler / state name / host+chassis state
// ============================================================================

std::function<void(Event)> C2PowerControl::getPowerStateHandler()
{
    switch (powerState)
    {
        case PowerState::waitForPDBPSUPowerOk:
            return [this](Event e) { this->handleWaitForPDBPSUPowerOk(e); };

        case PowerState::waitForPDBPSUPowerOff:
            return [this](Event e) { this->handleWaitForPDBPSUPowerOff(e); };

        default:
            return VRPowerControl::getPowerStateHandler();
    }
}

std::string_view C2PowerControl::getHostState() const
{
    switch (powerState)
    {
        case PowerState::waitForPDBPSUPowerOk:
            return "xyz.openbmc_project.State.Host.HostState.TransitioningToRunning";

        case PowerState::waitForPDBPSUPowerOff:
            return "xyz.openbmc_project.State.Host.HostState.TransitioningToOff";

        default:
            break;
    }
    return VRPowerControl::getHostState();
}

std::string_view C2PowerControl::getChassisState() const
{
    switch (powerState)
    {
        case PowerState::waitForPDBPSUPowerOk:
            return "xyz.openbmc_project.State.Chassis.PowerState.TransitioningToOn";

        case PowerState::waitForPDBPSUPowerOff:
            return "xyz.openbmc_project.State.Chassis.PowerState.TransitioningToOff";

        default:
            break;
    }
    return VRPowerControl::getChassisState();
}

std::string C2PowerControl::getPowerStateName() const
{
    switch (powerState)
    {
        case PowerState::waitForPDBPSUPowerOk:
            return "Wait for PDB PSU Power OK";

        case PowerState::waitForPDBPSUPowerOff:
            return "Wait for PDB PSU Power Off";

        default:
            break;
    }
    return VRPowerControl::getPowerStateName();
}

// ============================================================================
// Pure-virtual overrides
// ============================================================================

bool C2PowerControl::isSystemPowerOff()
{
    auto board0RunPowerPG = getSignal("Board0RunPowerPG");
    if (!board0RunPowerPG || !board0RunPowerPG->gpioLine)
    {
        lg2::error("CRITICAL: Board0RunPowerPG not available");
        return false;
    }

    auto pdbMainPowerOk = getSignal("PDBMainPowerOk");
    if (!pdbMainPowerOk || !pdbMainPowerOk->gpioLine)
    {
        lg2::error("CRITICAL: PDBMainPowerOk not available");
        return false;
    }

    auto pdbPSUPowerOk = getSignal("PDBPSUPowerOk");
    if (!pdbPSUPowerOk || !pdbPSUPowerOk->gpioLine)
    {
        lg2::error("CRITICAL: PDBPSUPowerOk not available");
        return false;
    }

    return (
        board0RunPowerPG->gpioLine.get_value() == !board0RunPowerPG->polarity &&
        pdbMainPowerOk->gpioLine.get_value() == !pdbMainPowerOk->polarity &&
        pdbPSUPowerOk->gpioLine.get_value() == !pdbPSUPowerOk->polarity);
}

void C2PowerControl::handlePowerOnRequest()
{
    lg2::info(
        "Power On Request received. Setting GPIOs to default state for host state Off and "
        "Commencing C2 Host Main Power On sequence.");

    auto board0RunPowerPG = getSignal("Board0RunPowerPG");
    if (!board0RunPowerPG || !board0RunPowerPG->gpioLine)
    {
        lg2::error(
            "CRITICAL: Board0RunPowerPG not available - cannot power on");
        return;
    }

    auto pdbMainPowerOk = getSignal("PDBMainPowerOk");
    if (!pdbMainPowerOk || !pdbMainPowerOk->gpioLine)
    {
        lg2::error("CRITICAL: PDBMainPowerOk not available - cannot power on");
        return;
    }

    auto pdbPSUPowerOk = getSignal("PDBPSUPowerOk");
    if (!pdbPSUPowerOk || !pdbPSUPowerOk->gpioLine)
    {
        lg2::error("CRITICAL: PDBPSUPowerOk not available - cannot power on");
        return;
    }

    auto pdbPSUPowerOn = getSignal("PDBPSUPowerOn");
    if (!pdbPSUPowerOn || !pdbPSUPowerOn->gpioLine)
    {
        lg2::error("CRITICAL: PDBPSUPowerOn not available - cannot power on");
        return;
    }

    // Check if power is already on (all 3 power indicators asserted)
    if (board0RunPowerPG->gpioLine.get_value() == board0RunPowerPG->polarity &&
        pdbMainPowerOk->gpioLine.get_value() == pdbMainPowerOk->polarity &&
        pdbPSUPowerOk->gpioLine.get_value() == pdbPSUPowerOk->polarity)
    {
        lg2::info(
            "PDB PSU Power, PDB Main Power, and HPM Run Power are already enabled. "
            "Setting GPIOs for host state ON and transitioning to PowerState::on.");
        setGPIOsForHostStateOn();
        action = PowerAction::NONE;
        setPowerState(PowerState::on);
    }
    else
    {
        setGPIOsForHostStateOff();
        lg2::info(
            "Asserting PDBPSUPowerOn. Starting PDB PSU Power OK Watchdog Timer. "
            "Transitioning to PowerState::waitForPDBPSUPowerOk.");
        action = PowerAction::POWER_ON;
        setGPIOOutput(pdbPSUPowerOn, pdbPSUPowerOn->polarity);
        startTimer("PdbPSUPowerOkWatchdogMs", pdbPSUPowerOkWatchdogTimer,
                   Event::pdbPSUPowerOkWatchdogTimerExpired);
        setPowerState(PowerState::waitForPDBPSUPowerOk);
    }
}

void C2PowerControl::initiatePDBPowerOff()
{
    cancelTimer("HPM Power Good Watchdog Timer", hpmPowerGoodWatchdogTimer);

    // Get the 12V rail enable signals
    auto pdb12vHPMAICEnable = getSignal("PDB12V_HPM-AICEnable");
    if (!pdb12vHPMAICEnable)
    {
        return;
    }

    auto pdb12vGPU1Enable = getSignal("PDB12V_GPU1Enable");
    if (!pdb12vGPU1Enable)
    {
        return;
    }

    auto pdb12vGPU2Enable = getSignal("PDB12V_GPU2_Enable");
    if (!pdb12vGPU2Enable)
    {
        return;
    }

    // De-assert the 12V rail enables. On C2, PDBMainPowerOk is downstream of
    // these rails, so removing them drops PDBMainPowerOk.
    lg2::info(
        "HPM Board 0 Run Power Good de-asserted. De-asserting 12V rails in order: HPM-AIC, GPU1, GPU2.");
    setGPIOOutput(pdb12vHPMAICEnable, !pdb12vHPMAICEnable->polarity);
    setGPIOOutput(pdb12vGPU1Enable, !pdb12vGPU1Enable->polarity);
    setGPIOOutput(pdb12vGPU2Enable, !pdb12vGPU2Enable->polarity);

    if (pdbMainPowerOkDeassertedDuringRecovery)
    {
        pdbMainPowerOkDeassertedDuringRecovery = false;
        lg2::info(
            "PDBMainPowerOk already de-asserted during CPU recovery; bypassing the duplicate wait and continuing PSU teardown.");
        completeShutdownAndTransitionToOff(true);
        return;
    }

    // Wait for the PDBMainPowerOk de-assertion GPIO event rather than reading
    // its level here. De-asserting the 12V rails above drops PDBMainPowerOk,
    // but its edge event is delivered asynchronously after this handler
    // returns. If we read the level and bypassed waitForPDBMainPowerOff, that
    // pending de-assert event would arrive in a later state (e.g.
    // waitForPDBPSUPowerOff) and be misread as an unexpected power fault by
    // checkAndHandlePdbMainPowerOkFault(). Transitioning here guarantees the
    // event lands in waitForPDBMainPowerOff, the one state where it is
    // expected. PSU teardown continues from
    // completeShutdownAndTransitionToOff() once PDBMainPowerOk de-asserts (or
    // the watchdog expires).
    lg2::info("Starting PDB Main Power OK Watchdog Timer. Transitioning to "
              "PowerState::waitForPDBMainPowerOff to wait for de-assertion.");
    setPowerState(PowerState::waitForPDBMainPowerOff);
    startTimer("PdbMainPowerOkWatchdogMs", pdbMainPowerOkWatchdogTimer,
               Event::pdbMainPowerOkWatchdogTimerExpired);
}

// ============================================================================
// Virtual overrides extending VR defaults
// ============================================================================

void C2PowerControl::completeShutdownAndTransitionToOff(bool success)
{
    cancelTimer("PDB Main Power OK Watchdog Timer",
                pdbMainPowerOkWatchdogTimer);

    if (!success)
    {
        lg2::error(
            "PDB Main Power OK watchdog timer expired. PDB Main Power OK did not de-assert. "
            "Continuing PSU teardown sequence despite fault.");
        logResourceEvent(
            "ResourceErrorsDetected",
            {"Host0", "PDB Main Power OK watchdog expired (power off)"},
            "xyz.openbmc_project.Logging.Entry.Level.Error");
        // Fall through to PSU teardown — do not return. The HPM/host is already
        // off; best-effort power down the PSU side and log if that also fails.
    }

    auto pdbPSUPowerOn = getSignal("PDBPSUPowerOn");
    if (!pdbPSUPowerOn || !pdbPSUPowerOn->gpioLine)
    {
        lg2::error(
            "CRITICAL: PDBPSUPowerOn not available - cannot complete PSU teardown");
        action = PowerAction::NONE;
        setGPIOsForHostStateOff();
        setPowerState(PowerState::off);
        return;
    }

    lg2::info(
        "De-asserting PDBPSUPowerOn. Starting PDB PSU Power OK Watchdog Timer. "
        "Transitioning to PowerState::waitForPDBPSUPowerOff.");
    setGPIOOutput(pdbPSUPowerOn, !pdbPSUPowerOn->polarity);
    startTimer("PdbPSUPowerOkWatchdogMs", pdbPSUPowerOkWatchdogTimer,
               Event::pdbPSUPowerOkWatchdogTimerExpired);
    setPowerState(PowerState::waitForPDBPSUPowerOff);
}

void C2PowerControl::setDefaultValues()
{
    lg2::info(
        "Defining C2 platform GPIOs asserted and de-asserted states based on host state ON and OFF");

    auto pdbPSUPowerOn = getSignal("PDBPSUPowerOn");
    if (!pdbPSUPowerOn)
    {
        return;
    }

    auto pdb12vHPMAICEnable = getSignal("PDB12V_HPM-AICEnable");
    if (!pdb12vHPMAICEnable)
    {
        return;
    }

    auto pdb12vGPU1Enable = getSignal("PDB12V_GPU1Enable");
    if (!pdb12vGPU1Enable)
    {
        return;
    }

    auto pdb12vGPU2Enable = getSignal("PDB12V_GPU2_Enable");
    if (!pdb12vGPU2Enable)
    {
        return;
    }

    auto usbPowerEnable = getSignal("USBPowerEnable");
    if (!usbPowerEnable)
    {
        return;
    }

    // PDB PSU Power On (active_low enable): ON=Asserted, OFF=DeAsserted
    pdbPSUPowerOn->defaultStateHostStateOn = DefaultState::Asserted;
    pdbPSUPowerOn->defaultStateHostStateOff = DefaultState::DeAsserted;

    // 12V Rail enables: ON=Asserted, OFF=DeAsserted
    pdb12vHPMAICEnable->defaultStateHostStateOn = DefaultState::Asserted;
    pdb12vHPMAICEnable->defaultStateHostStateOff = DefaultState::DeAsserted;

    pdb12vGPU1Enable->defaultStateHostStateOn = DefaultState::Asserted;
    pdb12vGPU1Enable->defaultStateHostStateOff = DefaultState::DeAsserted;

    pdb12vGPU2Enable->defaultStateHostStateOn = DefaultState::Asserted;
    pdb12vGPU2Enable->defaultStateHostStateOff = DefaultState::DeAsserted;

    // USB Power Enable: ON=Asserted, OFF=DeAsserted
    usbPowerEnable->defaultStateHostStateOn = DefaultState::Asserted;
    usbPowerEnable->defaultStateHostStateOff = DefaultState::DeAsserted;

    // Call parent to set common VR/HPM defaults
    VRPowerControl::setDefaultValues();

    lg2::info("C2 GPIOs asserted and de-asserted states defined successfully");
}

void C2PowerControl::validateTimerConfigs()
{
    for (const auto& timerName : c2RequiredTimeoutValues)
    {
        if (TimerMap.find(timerName) == TimerMap.end())
        {
            lg2::error("Required C2 timer config '{TIMER}' not found in config",
                       "TIMER", timerName);
            throw std::runtime_error(
                "C2PowerControl: Required timer config missing: " + timerName);
        }
    }

    VRPowerControl::validateTimerConfigs();

    lg2::info(
        "C2 timer configuration validation complete - all required timers present");
}

// ============================================================================
// C2-specific state handlers
// ============================================================================

void C2PowerControl::handleWaitForPDBPSUPowerOk(Event event)
{
    switch (event)
    {
        case Event::pdbPSUPowerOkAssert:
            assert12VRailsAndWaitForPDBMainPowerOk();
            break;

        case Event::pdbPSUPowerOkWatchdogTimerExpired:
            // WAR: https://nvbugspro.nvidia.com/bug/6406745
            // Missed edge recovery: if PDBPSUPowerOk is already asserted,
            // continue the power-on sequence (assert 12V rails) instead of
            // failing.
            if (gpioAtExpectedLevelOnTimeout("PDBPSUPowerOk", true))
            {
                lg2::warning(
                    "Recovering from suspected missed edge: PDBPSUPowerOk is already asserted. "
                    "Continuing power-on sequence to assert 12V rails.");
                assert12VRailsAndWaitForPDBMainPowerOk();
            }
            else
            {
                handlePDBPSUPowerOkWatchdogExpired();
            }
            break;

        default:
            lg2::info("No action taken for event: {EVENT}", "EVENT",
                      getEventName(event));
            break;
    }
}

void C2PowerControl::handleWaitForPDBPSUPowerOff(Event event)
{
    switch (event)
    {
        case Event::pdbPSUPowerOkDeAssert:
            cancelTimer("PDB PSU Power OK Watchdog Timer",
                        pdbPSUPowerOkWatchdogTimer);
            lg2::info(
                "PDB PSU Power OK de-asserted. PDB Power Off Sequence complete.");
            applyShutdownAction();
            break;

        case Event::pdbPSUPowerOkWatchdogTimerExpired:
            // WAR: https://nvbugspro.nvidia.com/bug/6406745
            // Missed edge recovery: if PDBPSUPowerOk is already de-asserted,
            // treat it as a successful de-assertion and complete the shutdown
            // action instead of reporting a fault.
            if (gpioAtExpectedLevelOnTimeout("PDBPSUPowerOk", false))
            {
                lg2::warning(
                    "Recovering from suspected missed edge: PDBPSUPowerOk is already de-asserted. "
                    "PDB Power Off Sequence complete.");
                applyShutdownAction();
                break;
            }

            lg2::error(
                "PDB PSU Power OK watchdog timer expired. PDB PSU Power Off Sequence Failed. "
                "Conducting Cleanup Sequence: Setting GPIO states to match Host State OFF.");
            action = PowerAction::NONE;
            logResourceEvent(
                "ResourceErrorsDetected",
                {"Host0", "PDB PSU Power OK watchdog expired (power off)"},
                "xyz.openbmc_project.Logging.Entry.Level.Error");
            setGPIOsForHostStateOff();
            setPowerState(PowerState::off);
            break;

        default:
            lg2::info("No action taken for event: {EVENT}", "EVENT",
                      getEventName(event));
            break;
    }
}

void C2PowerControl::handleWaitForPDBMainPowerOk(Event event)
{
    // WAR: https://nvbugspro.nvidia.com/bug/6406745
    // C2-only missed-edge workaround: a dropped PDBMainPowerOk assert edge can
    // leave us waiting even though the line already asserted. If so, continue
    // the power-on sequence instead of failing. All other events (including a
    // genuine timeout with the line still de-asserted) defer to the base.
    if (event == Event::pdbMainPowerOkWatchdogTimerExpired &&
        gpioAtExpectedLevelOnTimeout("PDBMainPowerOk", true))
    {
        lg2::warning(
            "Recovering from suspected missed edge: PDBMainPowerOk is already asserted. "
            "Continuing power-on sequence to HPM board sequencing.");
        transitionToHPMPowerGoodAssertState();
        return;
    }

    VRPowerControl::handleWaitForPDBMainPowerOk(event);
}

void C2PowerControl::handleWaitForPDBMainPowerOff(Event event)
{
    // WAR: https://nvbugspro.nvidia.com/bug/6406745
    // C2-only missed-edge workaround: a dropped PDBMainPowerOk de-assert edge
    // can leave us waiting even though the line already de-asserted. If so,
    // complete the shutdown as a success instead of reporting a fault. All
    // other events defer to the base.
    if (event == Event::pdbMainPowerOkWatchdogTimerExpired &&
        gpioAtExpectedLevelOnTimeout("PDBMainPowerOk", false))
    {
        lg2::warning(
            "Recovering from suspected missed edge: PDBMainPowerOk is already de-asserted. "
            "Continuing power-off sequence.");
        completeShutdownAndTransitionToOff(true);
        return;
    }

    VRPowerControl::handleWaitForPDBMainPowerOff(event);
}

bool C2PowerControl::gpioAtExpectedLevelOnTimeout(const std::string& signalName,
                                                  bool expectAsserted)
{
    // WAR: https://nvbugspro.nvidia.com/bug/6406745
    auto configPtr = getSignal(signalName);
    if (!configPtr || !configPtr->gpioLine)
    {
        lg2::error(
            "GPIO LEVEL CHECK: {SIGNAL} not available - cannot read GPIO level on watchdog timeout.",
            "SIGNAL", signalName);
        return false;
    }

    int value = -1;
    try
    {
        value = configPtr->gpioLine.get_value();
    }
    catch (const std::exception& e)
    {
        lg2::error(
            "GPIO LEVEL CHECK: failed to read {SIGNAL} GPIO level on watchdog timeout: {ERROR}",
            "SIGNAL", signalName, "ERROR", e.what());
        return false;
    }

    const bool asserted = (value == configPtr->polarity);
    const char* actualStr = asserted ? "asserted" : "de-asserted";
    const char* expectedStr = expectAsserted ? "asserted" : "de-asserted";

    if (asserted == expectAsserted)
    {
        // Level is already where we were waiting for it to be, yet the edge
        // event never arrived - likely a dropped/missed GPIO edge event rather
        // than a genuine hardware sequencing failure. The caller can safely
        // continue the sequence as though the edge arrived.
        lg2::error(
            "GPIO LEVEL CHECK: {SIGNAL} watchdog expired but the line is already {ACTUAL} "
            "(expected {EXPECTED}) [raw value={VALUE}, polarity={POLARITY}]. The hardware transition "
            "appears to have occurred but its GPIO edge event was not delivered.",
            "SIGNAL", signalName, "ACTUAL", actualStr, "EXPECTED", expectedStr,
            "VALUE", value, "POLARITY", static_cast<int>(configPtr->polarity));
        return true;
    }

    lg2::error(
        "GPIO LEVEL CHECK: {SIGNAL} watchdog expired and the line is {ACTUAL} "
        "(expected {EXPECTED}) [raw value={VALUE}, polarity={POLARITY}]. Consistent with a genuine "
        "hardware sequencing failure (line never reached the expected level).",
        "SIGNAL", signalName, "ACTUAL", actualStr, "EXPECTED", expectedStr,
        "VALUE", value, "POLARITY", static_cast<int>(configPtr->polarity));
    return false;
}

// ============================================================================
// Platform peripheral hooks
// ============================================================================

void C2PowerControl::assertPlatformPeripherals()
{
    auto usbPowerEnable = getSignal("USBPowerEnable");
    if (!usbPowerEnable)
    {
        return;
    }
    setGPIOOutput(usbPowerEnable, usbPowerEnable->polarity);
}

void C2PowerControl::deassertPlatformPeripherals()
{
    auto usbPowerEnable = getSignal("USBPowerEnable");
    if (!usbPowerEnable)
    {
        return;
    }
    setGPIOOutput(usbPowerEnable, !usbPowerEnable->polarity);
}

} // namespace power_control
