/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#pragma once

#include "power_control_base.hpp"
#include <functional>

namespace power_control
{

/**
 * @brief VR Power Control class - VR-specific extensions (NVL144 default)
 * 
 * This class extends PowerControl with VR-specific functionality:
 * - PDB power sequencing
 * - HPM power sequencing  
 * - CPU Reset monitoring
 * - CPU Shutdown OK monitoring
 * - 1P vs 2P support
 * 
 * This class handles VR PowerState enums:
 * - waitForPDBMainPowerOk
 * - waitForPDBMainPowerOff
 * - waitForHPMPowerGoodAssert
 * - waitForHPMPowerGoodDeAssert
 * - waitForCPUResetAssert
 * - waitForCPUResetDeAssert
 * - waitForCPUShutdownOk
 * - ...
 * 
 * Default implementations use NVL144 behavior. Platform-specific classes
 * (C2PowerControl, ParsecPowerControl, E5010PowerControl) override as needed.
 */
class VRPowerControl : public PowerControl
{
public:
    VRPowerControl(boost::asio::io_context& ioContext);
    virtual ~VRPowerControl() = default;

    /**
     * @brief Get the handler function for a given power state (VR extension)
     * 
     * This overrides the base class to add handlers for VR-specific states.
     * Falls back to base class for upstream states.
     * 
     * @param state The power state to get a handler for
     * @return Function that handles events in the given state, or nullptr if unknown
     */
    std::function<void(Event)> getPowerStateHandler(PowerState state) override;

protected:
    // GPIO EVENT HANDLERS (Member functions)
    // These handlers check polarity and send appropriate events via sendPowerControlEvent()
    
    /**
     * @brief Handler for Board 0 Run Power Good GPIO events
     * 
     * Checks polarity and sends appropriate Event to sendPowerControlEvent()
     * 
     * @param state The GPIO state (true = asserted, false = de-asserted)
     */
    void board0RunPowerPGHandler(bool state);
    
    /**
     * @brief Handler for Board 1 Run Power Good GPIO events
     * 
     * @param state The GPIO state (true = asserted, false = de-asserted)
     */
    void board1RunPowerPGHandler(bool state);
    
    /**
     * @brief Handler for Board 0 CPU Shutdown OK GPIO events
     * 
     * @param state The GPIO state (true = asserted, false = de-asserted)
     */
    void board0CpuShutdownOkHandler(bool state);
    
    /**
     * @brief Handler for Board 1 CPU Shutdown OK GPIO events
     * 
     * @param state The GPIO state (true = asserted, false = de-asserted)
     */
    void board1CpuShutdownOkHandler(bool state);
    
    /**
     * @brief Handler for CPU Reset Indicator GPIO events
     * 
     * @param state The GPIO state (true = asserted, false = de-asserted)
     */
    void cpuResetIndicatorHandler(bool state);

protected:
    // =============================================================================
    // OVERRIDDEN UPSTREAM STATE HANDLERS (with VR extensions)
    // =============================================================================

    /**
     * @brief Handler for PowerState::on (VR Override - NVL144 default)
     * 
     * PREVIOUS IMPLEMENTATION (NVL144 behavior in powerStateOn):
     * Extends base class behavior to add:
     * 
     * case Event::powerOffRequest:
     * case Event::gracefulPowerOffRequest:
     * case Event::powerCycleRequest:
     * case Event::powerButtonPressed:
     * case Event::forcePowerOffRequest:
     * case Event::gracefulPowerCycleRequest:
     * 
     * NOTE: Platform classes may override to add platform-specific on-state monitoring
     */
    void handlePowerStateOn(Event event) override;

    /**
     * @brief Handler for PowerState::off (VR Override - NVL144 default)
     * 
     * PREVIOUS IMPLEMENTATION (NVL144 behavior in powerStateOff):
     * Extends base class behavior to define support for these events:
     * 
     * case Event::powerOffRequest:
     * case Event::gracefulPowerOffRequest:
     * case Event::powerCycleRequest:
     * case Event::board0RunPowerPGDeAssert:
     * case Event::board0CpuShutdownOkDeAssert:
     * case Event::board1CpuShutdownOkDeAssert:
     * case Event::resetRequest:
     * case Event::powerButtonPressed:
     * case Event::forcePowerOffRequest:
     * case Event::gracefulPowerCycleRequest:
     *   
     * NOTE: Platform classes override to customize PDB power-on behavior
     */
    void handlePowerStateOff(Event event) override;

    // =============================================================================
    // VR-SPECIFIC STATE HANDLERS (PDB Sequencing)
    // =============================================================================

    /**
     * @brief Handler for PowerState::waitForPDBMainPowerOk
     * 
     * PREVIOUS IMPLEMENTATION (NVL144 default in powerStateWaitForPDBMainPowerOk):
     * 
     * case Event::nvl144pdbMainPowerOkAssert:
     * case Event::pdbMainPowerOkWatchdogTimerExpired:
     * 
     * NOTE: C2 overrides to enable 12V rails after PDB powers up
     * NOTE: Parsec (GB300) overrides for GB300-specific PDB behavior
     * NOTE: E5010 defines skip PDB sequencing and go directly to HPM sequencing
     */
    virtual void handleWaitForPDBMainPowerOk(Event event);

    /**
     * @brief Handler for PowerState::waitForPDBMainPowerOff
     * 
     * PREVIOUS IMPLEMENTATION (in powerStateWaitForPDBMainPowerOff):
     * 
     * Waits for PDB Main Power OK to de-assert after:
     * - De-asserting PDB Main Power Enable during power off sequence
     * 
     * case Event::nvl144pdbMainPowerOkDeAssert:
     * case Event::pdbMainPowerOkWatchdogTimerExpired:
     *
     * NOTE: C2 overrides to handle C2 PDB PSU Power OK De-Assert
     * NOTE: Parsec overrides to handle GB300 PDB Main Power OK De-Assert
     * NOTE: E5010 defines skip PDB sequencing and go directly to HPM sequencing
     */
    virtual void handleWaitForPDBMainPowerOff(Event event);

    // =============================================================================
    // VR-SPECIFIC STATE HANDLERS (HPM Board Sequencing)
    // =============================================================================

    /**
     * @brief Handler for PowerState::waitForHPMPowerGoodAssert
     * 
     * PREVIOUS IMPLEMENTATION (in powerStateWaitForHPMPowerGoodAssert):
     * 
     * Waits for HPM board(s) Run Power Good to assert after:
     * - Asserting Pre System Reset for Board 0/1
     * - Asserting Run Power Enable
     * 
     * case Event::board0RunPowerPGAssert:
     * case Event::hpmPowerGoodWatchdogTimerExpired:
     */
    virtual void handleWaitForHPMPowerGoodAssert(Event event);

    /**
     * @brief Handler for PowerState::waitForHPMPowerGoodDeAssert
     * 
     * PREVIOUS IMPLEMENTATION (in powerStateWaitForHPMPowerGoodDeAssert):
     * - Asserting Pre System Reset for Board 0/1
     * - De-asserting Run Power Enable for Board 0/1
     * 
     * Waits for HPM board(s) Run Power Good to de-assert during power off:
     * 
     * case Event::board0RunPowerPGDeAssert:
     * case Event::hpmPowerGoodWatchdogTimerExpired:
     */
    virtual void handleWaitForHPMPowerGoodDeAssert(Event event);

    // =============================================================================
    // VR-SPECIFIC STATE HANDLERS (CPU Reset Monitoring)
    // =============================================================================

    /**
     * @brief Handler for PowerState::waitForCPUResetAssert
     * 
     * PREVIOUS IMPLEMENTATION (in powerStateWaitForCPUResetAssert):
     * 
     * Waits for CPU Reset Indicator to assert after:
     * - Asserting Run Pre System Reset for Board 0/1 during force off & grace off
     * 
     * case Event::cpuResetIndicatorAssert:
     * case Event::cpuResetWatchdogTimerExpired:
     */
    virtual void handleWaitForCPUResetAssert(Event event);

    /**
     * @brief Handler for PowerState::waitForCPUResetDeAssert
     * 
     * PREVIOUS IMPLEMENTATION (in powerStateWaitForCPUResetDeAssert):
     * 
     * Waits for CPU Reset Indicator to de-assert after:
     * - De-asserting Pre System Reset for Board 0/1 during power on
     * 
     * case Event::cpuResetIndicatorDeAssert:
     * case Event::cpuResetWatchdogTimerExpired:
     */
    virtual void handleWaitForCPUResetDeAssert(Event event);

    // =============================================================================
    // VR-SPECIFIC STATE HANDLERS (Graceful Shutdown Monitoring)
    // =============================================================================

    /**
     * @brief Handler for PowerState::waitForCPUShutdownOk
     * 
     * PREVIOUS IMPLEMENTATION (in powerStateWaitForCPUShutdownOk):
     * 
     * Waits for CPU Shutdown OK to assert after:
     * - Asserting Shutdown Request (graceful) or Shutdown Force (forceful)
     * - Non-failure in forceful shutdown sequence, Failure in graceful shutdown sequence
     * 
     * case Event::board0CpuShutdownOkAssert:
     * case Event::board1CpuShutdownOkAssert:
     * case Event::cpuShutdownOkWatchdogTimerExpired:
     */
    virtual void handleWaitForCPUShutdownOk(Event event);
};

} // namespace power_control

