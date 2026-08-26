// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "vr_nvl8_power_control.hpp"

#include <phosphor-logging/lg2.hpp>

namespace power_control
{
// Type aliases for convenience
using Event = PowerControl::Event;

// Constructor
VRNVL8PowerControl::VRNVL8PowerControl(
    boost::asio::io_context& ioContext,
    std::shared_ptr<sdbusplus::asio::connection> conn,
    const std::string& configFilePath, const std::string& node,
    PersistentState& appState) :
    VRPowerControl(ioContext, conn, configFilePath, node, appState)
{
    // Required resources (IOX paths). VR NVL8 has a single Board 0 IOX and
    // no PDB IOX.
    addRequiredResource("Board0IoxPath", ResourceType::IOXPath);

    // VRPowerControl already registers Board0 VR signals:
    //   Board0RunPowerEnable, Board0RunPowerPG, Board0PreSystemReset,
    //   Board0CpuShutdownForce, Board0CpuShutdownRequest,
    //   Board0CpuShutdownOk, CpuResetIndicator.
    // VR NVL8 has no PDB (no PDBMainPowerOk / PDBMainPowerEnable / PSU enable
    // / 12V rails), no USB Power Enable, no E1S, no SSD reset, and no
    // Board 1 — no additional signals to register here.

    // Validate required resources first (IOX paths) then signals on them.
    PowerControl::validateRequiredResources();
    PowerControl::validateRequiredSignals();
    validateTimerConfigs();
    setDefaultValues();

    // Determine power state from hardware before exposing interfaces to D-Bus,
    // so the initial published values are correct. VR NVL8 is ON iff
    // Board0RunPowerPG is asserted (no PDB indicators).
    initializePowerStateFromHardware(powerIndicators, true);

    // Initialize all host0 interfaces — makes the path visible to ObjectMapper.
    // Called after initializePowerStateFromHardware so the correct state is
    // published immediately on InterfacesAdded.
    registerCpuBootDoneSetterMethod();
    initializeHostStateInterface();
}

// ============================================================================
// Pure-virtual overrides
// ============================================================================

bool VRNVL8PowerControl::isSystemPowerOff()
{
    auto board0RunPowerPG = getSignal("Board0RunPowerPG");
    if (!board0RunPowerPG || !board0RunPowerPG->gpioLine)
    {
        lg2::error("CRITICAL: Board0RunPowerPG not available");
        return false;
    }

    return board0RunPowerPG->gpioLine.get_value() ==
           !board0RunPowerPG->polarity;
}

void VRNVL8PowerControl::handlePowerOnRequest()
{
    lg2::info(
        "Power On Request received. VR NVL8 has no PDB — commencing HPM-only "
        "Power On sequence.");

    auto board0RunPowerPG = getSignal("Board0RunPowerPG");
    if (!board0RunPowerPG || !board0RunPowerPG->gpioLine)
    {
        lg2::error(
            "CRITICAL: Board0RunPowerPG not available - cannot power on");
        return;
    }

    // Check if power is already on
    if (board0RunPowerPG->gpioLine.get_value() == board0RunPowerPG->polarity)
    {
        lg2::info(
            "Board 0 Run Power Good is already asserted. Setting GPIOs for "
            "host state ON and transitioning to PowerState::on.");
        setGPIOsForHostStateOn();
        action = PowerAction::NONE;
        setPowerState(PowerState::on);
    }
    else
    {
        setGPIOsForHostStateOff();
        lg2::info("Commencing HPM Board Power Sequencing. Transitioning to "
                  "PowerState::waitForHPMPowerGoodAssert.");
        action = PowerAction::POWER_ON;
        transitionToHPMPowerGoodAssertState();
    }
}

void VRNVL8PowerControl::initiatePDBPowerOff()
{
    // VR NVL8 has no PDB to tear down. Dispatch the shutdown action
    // immediately.
    lg2::info(
        "HPM Board 0 Run Power Good de-asserted. VR NVL8 has no PDB — dispatching "
        "shutdown action.");
    cancelTimer("HPM Power Good Watchdog Timer", hpmPowerGoodWatchdogTimer);
    applyShutdownAction();
}

// ============================================================================
// Defensive overrides — PDB states should never be entered on VR NVL8
// ============================================================================

std::function<void(Event)> VRNVL8PowerControl::getPowerStateHandler()
{
    switch (powerState)
    {
        case PowerState::waitForPDBMainPowerOk:
        case PowerState::waitForPDBMainPowerOff:
            lg2::error(
                "VR NVL8: Unexpected PDB power state — VR NVL8 has no PDB. "
                "Events in this state will be dropped.");
            return {};

        default:
            return VRPowerControl::getPowerStateHandler();
    }
}

std::string_view VRNVL8PowerControl::getHostState() const
{
    switch (powerState)
    {
        case PowerState::waitForPDBMainPowerOk:
        case PowerState::waitForPDBMainPowerOff:
            lg2::error("VR NVL8: Unexpected PDB power state in getHostState, "
                       "returning Off");
            return "xyz.openbmc_project.State.Host.HostState.Off";

        default:
            break;
    }
    return VRPowerControl::getHostState();
}

std::string_view VRNVL8PowerControl::getChassisState() const
{
    switch (powerState)
    {
        case PowerState::waitForPDBMainPowerOk:
        case PowerState::waitForPDBMainPowerOff:
            lg2::error(
                "VR NVL8: Unexpected PDB power state in getChassisState, "
                "returning Off");
            return "xyz.openbmc_project.State.Chassis.PowerState.Off";

        default:
            break;
    }
    return VRPowerControl::getChassisState();
}

std::string VRNVL8PowerControl::getPowerStateName() const
{
    switch (powerState)
    {
        case PowerState::waitForPDBMainPowerOk:
        case PowerState::waitForPDBMainPowerOff:
            return "Unexpected PDB state (VR NVL8 has no PDB)";

        default:
            break;
    }
    return VRPowerControl::getPowerStateName();
}

} // namespace power_control
