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
    // Call parent constructor (registers VR + PDB GPIOs)
    VRPowerControl(ioContext, conn, configFilePath, node, appState),
    hscMfrIdRetryTimer(ioContext)
{
    // VRPowerControl constructor already registers:
    //   PDBMainPowerOk (with pdbMainPowerOkHandler),
    //   Board0 VR signals, and Board1CpuShutdownOk (if board1Present).
    // Add NVL72-specific signals here.
    addRequiredSignal("PDBMainPowerEnable", 0, GPIODirection::OUT);
    addRequiredSignal("E1SPowerEnable", 0, GPIODirection::OUT);
    addRequiredSignal("BMCSSDReset", 0, GPIODirection::OUT);
    addRequiredSignal("SSDPowerDisable", 0, GPIODirection::OUT);

    if (boardPresence.board1Present)
    {
        addRequiredSignal("Board1RunPowerEnable", 1, GPIODirection::OUT);
        addRequiredSignal("Board1PreSystemReset", 1, GPIODirection::OUT);
        addRequiredSignal("Board1CpuShutdownForce", 1, GPIODirection::OUT);
        addRequiredSignal("Board1CpuShutdownRequest", 1, GPIODirection::OUT);
        addBoard1GpioStateProperties();
    }

    // Validate all required signals (VR + NVL72)
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
    // WAR: Mask HSC alerts and clear faults on each PDBMainPowerOk assert
    auto configPtr = getSignal("PDBMainPowerOk");
    if (!configPtr)
    {
        return;
    }

    if (state == configPtr->polarity)
    {
        maskHscAlertsAndClearFaults();
    }

    // Delegate common event dispatch to VRPowerControl
    VRPowerControl::pdbMainPowerOkHandler(state);
}

void NVL72PowerControl::assertHPMBoardPowerSequence()
{
    auto board0RunPowerEnable = getSignal("Board0RunPowerEnable");
    if (!board0RunPowerEnable)
    {
        return;
    }

    auto board0PreSystemReset = getSignal("Board0PreSystemReset");
    if (!board0PreSystemReset)
    {
        return;
    }

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

    // Assert Pre System Reset for Board 0 and Board 1 (if present)
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

    // Assert peripheral power and de-assert BMC SSD Reset
    setGPIOOutput(ssdPowerDisable, !ssdPowerDisable->polarity);
    setGPIOOutput(bmcSSDReset, !bmcSSDReset->polarity);

    // sleep for 1 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    setGPIOOutput(usbPowerEnable, usbPowerEnable->polarity);
    setGPIOOutput(e1sPowerEnable, e1sPowerEnable->polarity);

    // Assert Run Power Enable for Board 0 and Board 1 (if present)
    setGPIOOutput(board0RunPowerEnable, board0RunPowerEnable->polarity);

    lg2::info(
        "GPU_OVERT PWR FAULT WAR: Sleeping for 10 ms after asserting Board 0 Run Power Enable");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

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

void NVL72PowerControl::deassertHPMPowerAndPeripherals()
{
    // De-assert the common VR signals first (Board0/1 RunPowerEnable + USB)
    VRPowerControl::deassertHPMPowerAndPeripherals();

    // NVL72-specific: also de-assert E1S Power Enable
    auto e1sPowerEnable = getSignal("E1SPowerEnable");
    if (!e1sPowerEnable)
    {
        return;
    }
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

    // Call parent to set common VR/HPM defaults
    VRPowerControl::setDefaultValues();

    lg2::info(
        "NVL72 GPIOs asserted and de-asserted states defined successfully");
}

void NVL72PowerControl::validateTimerConfigs()
{
    // PdbMainPowerOkWatchdogMs is now validated by VRPowerControl
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

void NVL72PowerControl::maskHscAlertsAndClearFaults()
{
    hscMfrIdRetryTimer.cancel();
    maskHscAlertsAndClearFaultsAttempt(1);
}

void NVL72PowerControl::scheduleHscMfrIdRetry(int nextAttempt)
{
    hscMfrIdRetryTimer.expires_after(std::chrono::milliseconds(100));
    hscMfrIdRetryTimer.async_wait(
        [this, nextAttempt](const boost::system::error_code& ec) {
            if (ec)
            {
                if (ec != boost::asio::error::operation_aborted)
                {
                    lg2::debug("HSC MFR_ID retry timer failed: {ERROR}",
                               "ERROR", ec.message());
                }
                return;
            }
            maskHscAlertsAndClearFaultsAttempt(nextAttempt);
        });
}

void NVL72PowerControl::maskHscAlertsAndClearFaultsAttempt(int attempt)
{
    constexpr int hscBus = 9;
    constexpr uint16_t hscDetectAddr = 0x10;
    constexpr uint8_t hscMfrIdRegister = 0x99;
    // NVBug 6058661: MP5926 MFR_ID block reads can transiently return invalid
    // data during early boot, so retry before deciding the PDB HSC vendor is
    // unknown.
    constexpr int hscMfrIdReadAttempts = 5;
    const std::vector<uint8_t> clearCmd = {0x03};
    enum class HscVendor
    {
        unknown,
        ti,
        mps,
        ifx,
    };
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
        // open() failure is fatal — bus is unaffected by MPS bug so no need to
        // retry
        lg2::error("HSC WAR: failed to open I2C bus {PATH}", "PATH", i2cPath);
        return;
    }

    std::vector<uint8_t> mfrId(4);

    const int readRet =
        PowerControl::i2cRead(file, hscDetectAddr, hscMfrIdRegister, mfrId);
    if (readRet < 0)
    {
        if (attempt < hscMfrIdReadAttempts)
        {
            close(file);
            scheduleHscMfrIdRetry(attempt + 1);
            return;
        }
        lg2::error(
            "HSC WAR: failed to read MFR_ID after {ATTEMPTS} attempts: bus {BUS}, addr {ADDR}",
            "BUS", hscBus, "ADDR", static_cast<int>(hscDetectAddr), "ATTEMPTS",
            hscMfrIdReadAttempts);
        close(file);
        return;
    }
    if (mfrId[0] == 0x03 && mfrId[1] == 0x54 && mfrId[2] == 0x49 &&
        mfrId[3] == 0x00)
    {
        hscVendor = HscVendor::ti;
    }
    else if (mfrId[0] == 0x03 && mfrId[1] == 0x53 && mfrId[2] == 0x50 &&
             mfrId[3] == 0x4d)
    {
        hscVendor = HscVendor::mps;
    }
    else if (mfrId[0] == 0x03 && mfrId[1] == 0x49 && mfrId[2] == 0x46 &&
             mfrId[3] == 0x00)
    {
        hscVendor = HscVendor::ifx;
    }

    const std::string mfrIdHex = std::format(
        "0x{:02x} 0x{:02x} 0x{:02x} 0x{:02x}", static_cast<unsigned>(mfrId[0]),
        static_cast<unsigned>(mfrId[1]), static_cast<unsigned>(mfrId[2]),
        static_cast<unsigned>(mfrId[3]));

    if (hscVendor == HscVendor::unknown)
    {
        if (attempt < hscMfrIdReadAttempts)
        {
            close(file);
            scheduleHscMfrIdRetry(attempt + 1);
            return;
        }

        lg2::error(
            "HSC WAR: unknown PDB HSC vendor from MFR_ID {MFR_ID} after retries on bus {BUS}, addr {ADDR}; skipping vendor-specific mask/clear",
            "MFR_ID", mfrIdHex, "BUS", hscBus, "ADDR",
            static_cast<int>(hscDetectAddr));
        close(file);
        return;
    }

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
