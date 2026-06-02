// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "nvl72_power_control.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <phosphor-logging/lg2.hpp>

#include <chrono>
#include <format>
#include <string>
#include <thread>
#include <vector>

namespace power_control
{
// Type aliases for convenience
using Event = PowerControl::Event;

// Constructor: Assigns handlers and registers events for NVL72-specific GPIOs
NVL72PowerControl::NVL72PowerControl(
    boost::asio::io_context& ioContext,
    std::shared_ptr<sdbusplus::asio::connection> conn,
    const std::string& configFilePath, const std::string& node,
    PersistentState& appState) :
    VRPowerControl(
        ioContext, conn, configFilePath, node,
        appState) // Call parent constructor (registers VR + PDB GPIOs)
{
    // Required resources (IOX paths). Declare before signals because the
    // required GPIO signals live on these IOXs — if the IOX path is missing
    // from config, no signal on it can be valid.
    addRequiredResource("Board0IoxPath", ResourceType::IOXPath);
    addRequiredResource("Board1IoxPath", ResourceType::IOXPath);
    addRequiredResource("PdbIoxPath", ResourceType::IOXPath);

    // VRPowerControl constructor already registers:
    //   Board0 VR signals, and Board1CpuShutdownOk (if board1Present).
    // Add NVL72-specific signals here.
    addRequiredSignal("PDBMainPowerOk", 0, GPIODirection::IN,
                      [this](bool state) {
                          this->pdbMainPowerOkHandler(state);
                      });
    addRequiredSignal("PDBMainPowerEnable", 0, GPIODirection::OUT);
    addRequiredSignal("E1SPowerEnable", 0, GPIODirection::OUT);
    addRequiredSignal("BMCSSDReset", 0, GPIODirection::OUT);
    addRequiredSignal("SSDPowerDisable", 0, GPIODirection::OUT);
    addRequiredSignal("USBPowerEnable", 0, GPIODirection::OUT);

    if (boardPresence.board1Present)
    {
        addRequiredSignal("Board1RunPowerEnable", 1, GPIODirection::OUT);
        addRequiredSignal("Board1PreSystemReset", 1, GPIODirection::OUT);
        addRequiredSignal("Board1CpuShutdownForce", 1, GPIODirection::OUT);
        addRequiredSignal("Board1CpuShutdownRequest", 1, GPIODirection::OUT);
        addBoard1GpioStateProperties();
    }

    // Validate required resources first (IOX paths) then signals on them.

    PowerControl::validateRequiredResources();
    PowerControl::validateRequiredSignals();

    // Validate all required timers
    validateTimerConfigs();

    // Set default values for output signals
    setDefaultValues();

    // Determine power state from hardware before exposing interfaces to D-Bus,
    // so the initial published values are correct.
    // Host is ON only if BOTH Board0RunPowerPG AND PDBMainPowerOk are asserted.
    initializePowerStateFromHardware(powerIndicators, true);

    // Initialize all host0 interfaces — makes the path visible to ObjectMapper.
    // Called after initializePowerStateFromHardware so the correct state is
    // published immediately on InterfacesAdded.
    initializeHostStateInterface();
}

// ============================================================================
// Pure-virtual overrides
// ============================================================================

bool NVL72PowerControl::isSystemPowerOff()
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

    return (
        board0RunPowerPG->gpioLine.get_value() == !board0RunPowerPG->polarity &&
        pdbMainPowerOk->gpioLine.get_value() == !pdbMainPowerOk->polarity);
}

void NVL72PowerControl::handlePowerOnRequest()
{
    lg2::info(
        "Power On Request received. Setting GPIOs to default state for host state Off and Commencing Host Main Power On sequence.");

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

    auto pdbMainPowerEnable = getSignal("PDBMainPowerEnable");
    if (!pdbMainPowerEnable || !pdbMainPowerEnable->gpioLine)
    {
        lg2::error(
            "CRITICAL: PDBMainPowerEnable not available - cannot power on");
        return;
    }

    // Check if power is already on
    if (board0RunPowerPG->gpioLine.get_value() == board0RunPowerPG->polarity &&
        pdbMainPowerOk->gpioLine.get_value() == pdbMainPowerOk->polarity)
    {
        lg2::info(
            "PDB Main Power and HPM Run Power is already enabled. Setting GPIOs for host state ON and transitioning to PowerState::On");
        setGPIOsForHostStateOn();
        action = PowerAction::NONE;
        setPowerState(PowerState::on);
    }
    else
    {
        setGPIOsForHostStateOff();
        lg2::info(
            "Asserting PDB Main Power Enable. Starting PDB Main Power OK Watchdog Timer. Transitioning to PowerState::waitForPDBMainPowerOk");
        action = PowerAction::POWER_ON;
        setGPIOOutput(pdbMainPowerEnable, pdbMainPowerEnable->polarity);
        startTimer("PdbMainPowerOkWatchdogMs", pdbMainPowerOkWatchdogTimer,
                   Event::pdbMainPowerOkWatchdogTimerExpired);
        setPowerState(PowerState::waitForPDBMainPowerOk);
    }
}

void NVL72PowerControl::initiatePDBPowerOff()
{
    cancelTimer("HPM Power Good Watchdog Timer", hpmPowerGoodWatchdogTimer);

    auto pdbMainPowerOk = getSignal("PDBMainPowerOk");
    if (!pdbMainPowerOk || !pdbMainPowerOk->gpioLine)
    {
        lg2::error("CRITICAL: PDBMainPowerOk not available");
        // Fallback: assume worst case and transition to waitForPDBMainPowerOff
        lg2::info(
            "HPM Board 0 Run Power Good de-asserted. De-asserting PDB Main Power Enable. "
            "Starting PDB Main Power OK Watchdog Timer. Transitioning to PowerState::waitForPDBMainPowerOff.");
        setPowerState(PowerState::waitForPDBMainPowerOff);
        deassertPreSystemResetsAndPDBMainPower();
        startTimer("PdbMainPowerOkWatchdogMs", pdbMainPowerOkWatchdogTimer,
                   Event::pdbMainPowerOkWatchdogTimerExpired);
        return;
    }

    bool pdbMainPowerOkAsserted =
        pdbMainPowerOk->gpioLine.get_value() == pdbMainPowerOk->polarity;

    if (pdbMainPowerOkAsserted)
    {
        lg2::info(
            "HPM Board 0 Run Power Good de-asserted. PDBMainPowerOk is currently asserted. "
            "De-asserting PDB Main Power Enable. Starting PDB Main Power OK Watchdog Timer. "
            "Transitioning to PowerState::waitForPDBMainPowerOff to wait for de-assertion.");
        setPowerState(PowerState::waitForPDBMainPowerOff);
        deassertPreSystemResetsAndPDBMainPower();
        startTimer("PdbMainPowerOkWatchdogMs", pdbMainPowerOkWatchdogTimer,
                   Event::pdbMainPowerOkWatchdogTimerExpired);
    }
    else
    {
        // PDB Main Power OK is already de-asserted — bypass wait state
        lg2::info(
            "HPM Board 0 Run Power Good de-asserted. PDBMainPowerOk is already de-asserted. "
            "De-asserting PDB Main Power Enable. Bypassing PowerState::waitForPDBMainPowerOff...");
        deassertPreSystemResetsAndPDBMainPower();
        completeShutdownAndTransitionToOff(true);
    }
}

// ============================================================================
// Virtual overrides — extend VR defaults
// ============================================================================

void NVL72PowerControl::pdbMainPowerOkHandler(bool state)
{
    lg2::info("PDBMainPowerOk GPIO event: value={VALUE}", "VALUE",
              static_cast<int>(state));

    auto configPtr = getSignal("PDBMainPowerOk");
    if (!configPtr)
    {
        return;
    }

    // WAR: Mask HSC alerts and clear faults on each PDBMainPowerOk assert
    if (state == configPtr->polarity)
    {
        maskHscAlertsAndClearFaults();
    }

    Event powerControlEvent = (state == configPtr->polarity)
                                  ? Event::pdbMainPowerOkAssert
                                  : Event::pdbMainPowerOkDeAssert;

    // Check for power faults and handle if detected
    if (checkAndHandlePdbMainPowerOkFault(powerControlEvent))
    {
        return; // Fault was handled, exit early
    }

    this->sendPowerControlEvent(powerControlEvent);
}

// pdbMainPowerOkHandler Helper Function
bool NVL72PowerControl::checkAndHandlePdbMainPowerOkFault(
    Event powerControlEvent)
{
    // Power fault detection: Check for unexpected de-assertion
    if (powerControlEvent == Event::pdbMainPowerOkDeAssert)
    {
        if (powerState != PowerState::waitForPDBMainPowerOff)
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
        // else: Expected de-assertion in waitForPDBMainPowerOff state
    }

    // Return false to indicate normal processing should continue
    return false;
}

void NVL72PowerControl::assertHPMBoardPowerSequence()
{
    auto board0PreSystemReset = getSignal("Board0PreSystemReset");
    if (!board0PreSystemReset)
    {
        return;
    }

    auto board0RunPowerEnable = getSignal("Board0RunPowerEnable");
    if (!board0RunPowerEnable)
    {
        return;
    }

    // 1. PRE_SYS_RST_L for all present boards
    setGPIOOutput(board0PreSystemReset, board0PreSystemReset->polarity);

    if (boardPresence.board1Present)
    {
        auto board1PreSystemReset = getSignal("Board1PreSystemReset");
        if (!board1PreSystemReset)
        {
            return;
        }
        setGPIOOutput(board1PreSystemReset, board1PreSystemReset->polarity);
    }

    // 2. NVL72 peripherals (SSD/BMCSSDReset/1ms/USB/E1S)
    assertPlatformPeripherals();

    // 3. RUN_POWER_EN — Board0
    setGPIOOutput(board0RunPowerEnable, board0RunPowerEnable->polarity);

    // GPU_OVERT PWR FAULT WAR — non-POR; remove this override when fixed in HW
    lg2::info(
        "GPU_OVERT PWR FAULT WAR: Sleeping for 10 ms after asserting Board 0 Run Power Enable");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // 4. RUN_POWER_EN — Board1
    if (boardPresence.board1Present)
    {
        auto board1RunPowerEnable = getSignal("Board1RunPowerEnable");
        if (!board1RunPowerEnable)
        {
            return;
        }
        setGPIOOutput(board1RunPowerEnable, board1RunPowerEnable->polarity);
    }
}

void NVL72PowerControl::assertPlatformPeripherals()
{
    auto usbPowerEnable = getSignal("USBPowerEnable");
    if (!usbPowerEnable)
    {
        return;
    }

    auto e1sPowerEnable = getSignal("E1SPowerEnable");
    if (!e1sPowerEnable)
    {
        return;
    }

    auto bmcSSDReset = getSignal("BMCSSDReset");
    if (!bmcSSDReset)
    {
        return;
    }

    auto ssdPowerDisable = getSignal("SSDPowerDisable");
    if (!ssdPowerDisable)
    {
        return;
    }

    setGPIOOutput(ssdPowerDisable, !ssdPowerDisable->polarity);
    setGPIOOutput(bmcSSDReset, !bmcSSDReset->polarity);

    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    setGPIOOutput(usbPowerEnable, usbPowerEnable->polarity);
    setGPIOOutput(e1sPowerEnable, e1sPowerEnable->polarity);
}

void NVL72PowerControl::deassertPlatformPeripherals()
{
    auto usbPowerEnable = getSignal("USBPowerEnable");
    if (!usbPowerEnable)
    {
        return;
    }

    auto e1sPowerEnable = getSignal("E1SPowerEnable");
    if (!e1sPowerEnable)
    {
        return;
    }

    setGPIOOutput(usbPowerEnable, !usbPowerEnable->polarity);
    setGPIOOutput(e1sPowerEnable, !e1sPowerEnable->polarity);
}

void NVL72PowerControl::setDefaultValues()
{
    lg2::info(
        "Defining platform GPIOs asserted and de-asserted states based on host state ON and OFF");

    auto pdbMainPowerEnable = getSignal("PDBMainPowerEnable");
    if (!pdbMainPowerEnable)
    {
        return;
    }

    auto e1sPowerEnable = getSignal("E1SPowerEnable");
    if (!e1sPowerEnable)
    {
        return;
    }

    auto bmcSsdReset = getSignal("BMCSSDReset");
    if (!bmcSsdReset)
    {
        return;
    }

    auto ssdPowerDisable = getSignal("SSDPowerDisable");
    if (!ssdPowerDisable)
    {
        return;
    }

    auto usbPowerEnable = getSignal("USBPowerEnable");
    if (!usbPowerEnable)
    {
        return;
    }

    // PDB Main Power Enable: ON=Asserted, OFF=DeAsserted
    pdbMainPowerEnable->defaultStateHostStateOn = DefaultState::Asserted;
    pdbMainPowerEnable->defaultStateHostStateOff = DefaultState::DeAsserted;

    // E1S Power Enable: ON=Asserted, OFF=DeAsserted
    e1sPowerEnable->defaultStateHostStateOn = DefaultState::Asserted;
    e1sPowerEnable->defaultStateHostStateOff = DefaultState::DeAsserted;

    // BMC SSD Reset: ON=DeAsserted (out of reset), OFF=DeAsserted
    bmcSsdReset->defaultStateHostStateOn = DefaultState::DeAsserted;
    bmcSsdReset->defaultStateHostStateOff = DefaultState::DeAsserted;

    // SSD Power Disable: ON=DeAsserted (power enabled), OFF=DeAsserted
    ssdPowerDisable->defaultStateHostStateOn = DefaultState::DeAsserted;
    ssdPowerDisable->defaultStateHostStateOff = DefaultState::DeAsserted;

    // USB Power Enable: ON=Asserted, OFF=DeAsserted
    usbPowerEnable->defaultStateHostStateOn = DefaultState::Asserted;
    usbPowerEnable->defaultStateHostStateOff = DefaultState::DeAsserted;

    // Call parent to set common VR/HPM defaults
    VRPowerControl::setDefaultValues();

    lg2::info(
        "NVL72 GPIOs asserted and de-asserted states defined successfully");
}

void NVL72PowerControl::validateTimerConfigs()
{
    for (const auto& timerName : nvl72RequiredTimeoutValues)
    {
        if (TimerMap.find(timerName) == TimerMap.end())
        {
            lg2::error(
                "Required NVL72 timer config '{TIMER}' not found in config",
                "TIMER", timerName);
            throw std::runtime_error(
                "NVL72PowerControl: Required timer config missing: " +
                timerName);
        }
    }

    VRPowerControl::validateTimerConfigs();

    lg2::info(
        "NVL72 timer configuration validation complete - all required timers present");
}

// ============================================================================
// NVL72-specific private helpers
// ============================================================================

void NVL72PowerControl::addBoard1GpioStateProperties()
{
    gpioStateIface->register_property_r(
        "Board1CpuShutdownOk", int{-1},
        sdbusplus::vtable::property_::emits_change,
        [this](const auto&) { return board1CpuShutdownOkState; });

    gpioPropertySetters["Board1CpuShutdownOk"] = [](PowerControl* pc, int val) {
        pc->setBoard1CpuShutdownOkState(val);
    };
}

std::string NVL72PowerControl::mfrIdToHex(const std::vector<uint8_t>& mfrId)
{
    std::string out;
    bool first = true;
    for (auto byteVal : mfrId)
    {
        if (!first)
        {
            out += ' ';
        }
        first = false;
        out += std::format("0x{:02x}", static_cast<unsigned>(byteVal));
    }
    return out;
}

NVL72PowerControl::HscVendor NVL72PowerControl::getHscVendor(
    const std::vector<uint8_t>& mfrId)
{
    if (mfrId.size() < 4)
    {
        lg2::error("HSC WAR: MFR_ID is too short: {SIZE}", "SIZE",
                   mfrId.size());
        return HscVendor::unknown;
    }
    if (mfrId[0] == 0x03)
    {
        if (mfrId[1] == 0x54 && mfrId[2] == 0x49 && mfrId[3] == 0x00)
        {
            return HscVendor::ti;
        }
        if (mfrId[1] == 0x53 && mfrId[2] == 0x50 && mfrId[3] == 0x4d)
        {
            return HscVendor::mps;
        }
        if (mfrId[1] == 0x49 && mfrId[2] == 0x46 && mfrId[3] == 0x00)
        {
            return HscVendor::ifx;
        }
    }
    return HscVendor::unknown;
}

void NVL72PowerControl::maskHscAlertsAndClearFaults()
{
    constexpr int hscBus = 9;
    constexpr uint16_t hscDetectAddr = 0x10;
    constexpr uint8_t hscMfrIdRegister = 0x99;
    // NVBug 6058661: MP5926 MFR_ID block reads can transiently return invalid
    // data during early boot, so retry before deciding the PDB HSC vendor is
    // unknown.
    constexpr int hscMfrIdReadAttempts = 3;
    const std::vector<uint8_t> clearCmd = {0x03};
    HscVendor hscVendor = HscVendor::unknown;

    auto vendorToString = [](HscVendor vendor) -> const char* {
        switch (vendor)
        {
            case HscVendor::ti:
                return "TI";
            case HscVendor::mps:
                return "MPS";
            case HscVendor::ifx:
                return "IFX";
            default:
                return "unknown";
        }
    };

    const std::string i2cPath = "/dev/i2c-" + std::to_string(hscBus);
    int file = open(i2cPath.c_str(), O_RDWR | O_CLOEXEC);
    if (file < 0)
    {
        lg2::error("HSC WAR: failed to open I2C bus {PATH}", "PATH", i2cPath);
        return;
    }

    std::vector<uint8_t> mfrId(4);
    int attempt = 0;
    for (; attempt < hscMfrIdReadAttempts; ++attempt)
    {
        if (PowerControl::i2cRead(file, hscDetectAddr, hscMfrIdRegister,
                                  mfrId) < 0)
        {
            lg2::warning(
                "HSC WAR: failed to read MFR_ID on attempt {ATTEMPT}: bus {BUS}, addr {ADDR}",
                "ATTEMPT", (attempt + 1), "BUS", hscBus, "ADDR",
                static_cast<int>(hscDetectAddr));
            // Only sleep before retry, not on last attempt
            if (attempt < hscMfrIdReadAttempts - 1)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            continue;
        }

        hscVendor = getHscVendor(mfrId);

        if (hscVendor != HscVendor::unknown)
        {
            break;
        }

        lg2::warning(
            "HSC WAR: Read successful, but invalid MFR_ID {MFR_ID} on attempt {ATTEMPT}",
            "MFR_ID", mfrIdToHex(mfrId), "ATTEMPT", (attempt + 1));
        if (attempt < hscMfrIdReadAttempts - 1)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    if (attempt == hscMfrIdReadAttempts)
    {
        lg2::error(
            "HSC WAR: exhausted all {ATTEMPTS} retry attempts: bus {BUS}, addr {ADDR}",
            "BUS", hscBus, "ADDR", static_cast<int>(hscDetectAddr), "ATTEMPTS",
            hscMfrIdReadAttempts);
        close(file);
        return;
    }

    const std::string mfrIdHex = mfrIdToHex(mfrId);

    lg2::info(
        "HSC WAR: selected {VENDOR} PDB HSC vendor from MFR_ID {MFR_ID} on bus {BUS}, addr {ADDR}",
        "VENDOR", vendorToString(hscVendor), "MFR_ID", mfrIdHex, "BUS", hscBus,
        "ADDR", static_cast<int>(hscDetectAddr));

    if (hscVendor == HscVendor::ti || hscVendor == HscVendor::mps)
    {
        static const std::vector<uint16_t> hscAddrs = {0x10, 0x12, 0x14, 0x16};
        const std::vector<uint8_t> maskCmd = {0xD8, 0xFF, 0xFF};

        lg2::info(
            "HSC WAR: starting {VENDOR} PDB HSC fault mask and clear sequence on bus {BUS}",
            "VENDOR", vendorToString(hscVendor), "BUS", hscBus);

        for (uint16_t addr : hscAddrs)
        {
            if (PowerControl::i2cWrite(file, addr, maskCmd) < 0)
            {
                lg2::error("HSC mask alert failed: bus {BUS}, addr {ADDR}",
                           "BUS", hscBus, "ADDR", static_cast<int>(addr));
            }
            else
            {
                lg2::info("HSC mask alert success: bus {BUS}, addr {ADDR}",
                          "BUS", hscBus, "ADDR", static_cast<int>(addr));
            }

            if (PowerControl::i2cWrite(file, addr, clearCmd) < 0)
            {
                lg2::error("HSC clear fault failed: bus {BUS}, addr {ADDR}",
                           "BUS", hscBus, "ADDR", static_cast<int>(addr));
            }
            else
            {
                lg2::info("HSC clear fault success: bus {BUS}, addr {ADDR}",
                          "BUS", hscBus, "ADDR", static_cast<int>(addr));
            }
        }

        lg2::info(
            "HSC WAR: completed {VENDOR} PDB HSC fault mask and clear sequence on bus {BUS}",
            "VENDOR", vendorToString(hscVendor), "BUS", hscBus);
    }
    else if (hscVendor == HscVendor::ifx)
    {
        static const std::vector<uint16_t> hscAddrs = {0x10, 0x1c, 0x1d, 0x1e};
        constexpr uint8_t ifxMaskWarnsRegister = 0xE2;
        constexpr uint8_t ifxMaskFaultsRegister = 0xDF;
        constexpr uint8_t ifxGpoCfgRegister = 0xDB;
        constexpr uint8_t ifxSmbAlertDisableMask = 0xCF;
        const std::vector<uint8_t> maskWarnsCmd = {ifxMaskWarnsRegister, 0x00,
                                                   0x00};
        const std::vector<uint8_t> maskFaultsCmd = {ifxMaskFaultsRegister, 0x00,
                                                    0x00};

        lg2::info(
            "HSC WAR: starting IFX PDB HSC fault mask and clear sequence on bus {BUS}",
            "BUS", hscBus);

        for (uint16_t addr : hscAddrs)
        {
            if (PowerControl::i2cWrite(file, addr, maskWarnsCmd) < 0)
            {
                lg2::error("HSC MASK_WARNS failed: bus {BUS}, addr {ADDR}",
                           "BUS", hscBus, "ADDR", static_cast<int>(addr));
            }
            else
            {
                lg2::info("HSC MASK_WARNS success: bus {BUS}, addr {ADDR}",
                          "BUS", hscBus, "ADDR", static_cast<int>(addr));
            }

            if (PowerControl::i2cWrite(file, addr, maskFaultsCmd) < 0)
            {
                lg2::error("HSC MASK_FAULTS failed: bus {BUS}, addr {ADDR}",
                           "BUS", hscBus, "ADDR", static_cast<int>(addr));
            }
            else
            {
                lg2::info("HSC MASK_FAULTS success: bus {BUS}, addr {ADDR}",
                          "BUS", hscBus, "ADDR", static_cast<int>(addr));
            }

            std::vector<uint8_t> gpoCfg(2);
            if (PowerControl::i2cRead(file, addr, ifxGpoCfgRegister, gpoCfg) <
                0)
            {
                lg2::error("HSC GPO_CFG read failed: bus {BUS}, addr {ADDR}",
                           "BUS", hscBus, "ADDR", static_cast<int>(addr));
            }
            else
            {
                const std::vector<uint8_t> gpoCfgWrite = {
                    ifxGpoCfgRegister, gpoCfg[0],
                    static_cast<uint8_t>(gpoCfg[1] & ifxSmbAlertDisableMask)};
                if (PowerControl::i2cWrite(file, addr, gpoCfgWrite) < 0)
                {
                    lg2::error(
                        "HSC GPO_CFG SMBALERT disable failed: bus {BUS}, addr {ADDR}",
                        "BUS", hscBus, "ADDR", static_cast<int>(addr));
                }
                else
                {
                    lg2::info(
                        "HSC GPO_CFG SMBALERT disable success: bus {BUS}, addr {ADDR}, orig_hi {ORIG_HI}, new_hi {NEW_HI}",
                        "BUS", hscBus, "ADDR", static_cast<int>(addr),
                        "ORIG_HI", static_cast<int>(gpoCfg[1]), "NEW_HI",
                        static_cast<int>(gpoCfgWrite[2]));
                }
            }

            if (PowerControl::i2cWrite(file, addr, clearCmd) < 0)
            {
                lg2::error("HSC clear fault failed: bus {BUS}, addr {ADDR}",
                           "BUS", hscBus, "ADDR", static_cast<int>(addr));
            }
            else
            {
                lg2::info("HSC clear fault success: bus {BUS}, addr {ADDR}",
                          "BUS", hscBus, "ADDR", static_cast<int>(addr));
            }
        }

        lg2::info(
            "HSC WAR: completed IFX PDB HSC fault mask and clear sequence on bus {BUS}",
            "BUS", hscBus);
    }

    close(file);
}

void NVL72PowerControl::deassertPreSystemResetsAndPDBMainPower()
{
    auto pdbMainPowerEnable = getSignal("PDBMainPowerEnable");
    if (!pdbMainPowerEnable)
    {
        return;
    }

    // De-assert PDB Main Power Enable
    setGPIOOutput(pdbMainPowerEnable, !pdbMainPowerEnable->polarity);
}

void NVL72PowerControl::transitionToPDBMainPowerOffState()
{
    cancelTimer("HPM Power Good Watchdog Timer", hpmPowerGoodWatchdogTimer);

    lg2::info(
        "HPM Board 0 Run Power Good de-asserted. De-asserting PDB Main Power Enable. "
        "Starting PDB Main Power OK Watchdog Timer. Transitioning to PowerState::waitForPDBMainPowerOff.");

    setPowerState(PowerState::waitForPDBMainPowerOff);
    deassertPreSystemResetsAndPDBMainPower();
    startTimer("PdbMainPowerOkWatchdogMs", pdbMainPowerOkWatchdogTimer,
               Event::pdbMainPowerOkWatchdogTimerExpired);
}

} // namespace power_control
