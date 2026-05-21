// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "c2_power_control.hpp"

#include <phosphor-logging/lg2.hpp>

namespace power_control
{
// Type aliases for convenience
using Event = PowerControl::Event;

// Constructor
C2PowerControl::C2PowerControl(
    boost::asio::io_context& ioContext,
    std::shared_ptr<sdbusplus::asio::connection> conn,
    const std::string& configFilePath, const std::string& node,
    PersistentState& appState) :
    VRPowerControl(ioContext, conn, configFilePath, node, appState),
    pdbPSUPowerOkWatchdogTimer(ioContext)
{
    // VRPowerControl constructor already registers:
    //   PDBMainPowerOk (with pdbMainPowerOkHandler),
    //   Board0 VR signals (RunPowerEnable, RunPowerPG, PreSystemReset,
    //   CpuShutdownForce/Request/Ok, CpuResetIndicator, USBPowerEnable).
    // C2 is 1P — no Board 1 signals needed. C2 has no PDBMainPowerEnable;
    // the PDB PSU enable signal is PDBPSUPowerOn (registered below).
    // Add C2-specific PDB signals here.

    addRequiredSignal("PDBPSUPowerOn", 0, GPIODirection::OUT);
    addRequiredSignal("PDBPSUPowerOk", 0, GPIODirection::IN,
                      [this](bool state) {
                          this->pdbPSUPowerOkHandler(state);
                      });
    addRequiredSignal("PDB12V_HPM-AICEnable", 0, GPIODirection::OUT);
    addRequiredSignal("PDB12V_GPU1Enable", 0, GPIODirection::OUT);
    addRequiredSignal("PDB12V_GPU2_Enable", 0, GPIODirection::OUT);

    // Validate all required signals (VR + C2)
    PowerControl::validateRequiredSignals();

    // Validate all required timers
    validateTimerConfigs();

    // Set default values for output signals
    setDefaultValues();

    // Determine power state from hardware before exposing interfaces to D-Bus,
    // so the initial published values are correct.
    // Host is ON only if ALL THREE indicators are asserted.
    initializePowerStateFromHardware(powerIndicators, true);

    // Initialize all host0 interfaces — makes the path visible to ObjectMapper.
    // Called after initializePowerStateFromHardware so the correct state is
    // published immediately on InterfacesAdded.
    initializeHostStateInterface();
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
        "PDB PSU Power OK asserted. Asserting 12V rails (HPM-AIC, GPU1, GPU2). "
        "Starting PDB Main Power OK Watchdog Timer. "
        "Transitioning to PowerState::waitForPDBMainPowerOk.");
    setGPIOOutput(pdb12vHPMAICEnable, pdb12vHPMAICEnable->polarity);
    setGPIOOutput(pdb12vGPU1Enable, pdb12vGPU1Enable->polarity);
    setGPIOOutput(pdb12vGPU2Enable, pdb12vGPU2Enable->polarity);
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

    // Get all required signals upfront
    auto pdbMainPowerOk = getSignal("PDBMainPowerOk");
    if (!pdbMainPowerOk || !pdbMainPowerOk->gpioLine)
    {
        lg2::error("CRITICAL: PDBMainPowerOk not available");
        return;
    }

    auto pdbPSUPowerOk = getSignal("PDBPSUPowerOk");
    if (!pdbPSUPowerOk || !pdbPSUPowerOk->gpioLine)
    {
        lg2::error("CRITICAL: PDBPSUPowerOk not available");
        return;
    }

    auto pdbPSUPowerOn = getSignal("PDBPSUPowerOn");
    if (!pdbPSUPowerOn || !pdbPSUPowerOn->gpioLine)
    {
        lg2::error("CRITICAL: PDBPSUPowerOn not available");
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

    // Always de-assert the 12V rail enables first
    lg2::info(
        "HPM Board 0 Run Power Good de-asserted. De-asserting 12V rails (HPM-AIC, GPU1, GPU2).");
    setGPIOOutput(pdb12vHPMAICEnable, !pdb12vHPMAICEnable->polarity);
    setGPIOOutput(pdb12vGPU1Enable, !pdb12vGPU1Enable->polarity);
    setGPIOOutput(pdb12vGPU2Enable, !pdb12vGPU2Enable->polarity);

    // If PDBMainPowerOk is still asserted, wait for it to fall
    if (pdbMainPowerOk->gpioLine.get_value() == pdbMainPowerOk->polarity)
    {
        lg2::info(
            "PDBMainPowerOk is currently asserted. Starting PDB Main Power OK Watchdog Timer. "
            "Transitioning to PowerState::waitForPDBMainPowerOff to wait for de-assertion.");
        setPowerState(PowerState::waitForPDBMainPowerOff);
        startTimer("PdbMainPowerOkWatchdogMs", pdbMainPowerOkWatchdogTimer,
                   Event::pdbMainPowerOkWatchdogTimerExpired);
        return;
    }

    // PDBMainPowerOk already de-asserted. If PDBPSUPowerOk still asserted, wait
    // for it
    if (pdbPSUPowerOk->gpioLine.get_value() == pdbPSUPowerOk->polarity)
    {
        lg2::info(
            "PDBMainPowerOk already de-asserted. PDBPSUPowerOk is currently asserted. "
            "De-asserting PDBPSUPowerOn. Starting PDB PSU Power OK Watchdog Timer. "
            "Transitioning to PowerState::waitForPDBPSUPowerOff to wait for de-assertion.");
        setGPIOOutput(pdbPSUPowerOn, !pdbPSUPowerOn->polarity);
        setPowerState(PowerState::waitForPDBPSUPowerOff);
        startTimer("PdbPSUPowerOkWatchdogMs", pdbPSUPowerOkWatchdogTimer,
                   Event::pdbPSUPowerOkWatchdogTimerExpired);
        return;
    }

    // Both PDB Main Power OK and PDB PSU Power OK already de-asserted
    lg2::info("PDBMainPowerOk and PDBPSUPowerOk are both already de-asserted. "
              "De-asserting PDBPSUPowerOn. Bypassing wait states.");
    setGPIOOutput(pdbPSUPowerOn, !pdbPSUPowerOn->polarity);
    applyShutdownAction();
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
            handlePDBPSUPowerOkWatchdogExpired();
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

} // namespace power_control
