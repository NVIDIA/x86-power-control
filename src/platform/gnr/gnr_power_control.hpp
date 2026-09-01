/*
 * SPDX-License-Identifier: Apache-2.0
 * GNR platform: minimal power control (PowerOk, PowerOut, G3Soft sequence).
 */

#pragma once

#include "../../power_control_base.hpp"

#include <boost/asio/steady_timer.hpp>

#include <chrono>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

namespace power_control
{

/** Phases for the async G3Soft power-on sequence (no blocking sleeps). */
enum class GNRPowerOnPhase
{
    Idle,
    WaitingAp0ResetN,       // polling Ap0ResetN until HIGH or timeout
    WaitingPexResetPulse,   // PexResetN LOW held for PexResetPulseMs
    WaitingPowerButtonDelay // delay after PEX reset before pressing power
                            // button
};

/**
 * @brief GNR Power Control class
 *
 * Minimal platform that only requires PowerOk and PowerOut (and optional
 * G3Soft GPIOs) from JSON. Used for GNR BMC where NVL144/VR signals are
 * not present. Power-on sequence uses timers and states instead of sleeps.
 */
class GNRPowerControl : public PowerControl
{
  public:
    GNRPowerControl(boost::asio::io_context& ioContext,
                    std::shared_ptr<sdbusplus::asio::connection> conn,
                    const std::string& configFilePath, const std::string& node,
                    PersistentState& appState);
    virtual ~GNRPowerControl() = default;

    void powerOn() override;

    /**
     * @brief Check if system power is off using the PowerOk signal
     *
     * Reads the PowerOk GPIO; when deasserted (0), the system is off.
     *
     * @return true if PowerOk is deasserted or read fails (assume off),
     *         false if PowerOk is asserted (system has power)
     */
    bool isSystemPowerOff();

    /**
     * @brief Start graceful host shutdown for aux/full power-cycle
     *
     * Pulses PowerOut and waits for PowerOk to drop (same path as a
     * gracefulPowerOffRequest while On). Settles at On or Off.
     */
    void initiateGracefulShutdown() override;

    /**
     * @brief Start forceful host shutdown for aux/full power-cycle
     *
     * Holds PowerOut for ForceOffPulseMs (same path as a powerOffRequest
     * while On). Settles at On or Off.
     */
    void initiateForcefulShutdown() override;

  protected:
    void powerOKHandler(bool state) override;
    void validateTimerConfigs() override;
    void setDefaultValues();
    void handlePowerStateOn(Event event) override;
    void handlePowerStateOff(Event event) override;
    void handleTransitionToOff(Event event) override;
    void handleGracefulTransitionToOff(Event event) override;
    void handleTransitionToCycleOff(Event event) override;
    void handleGracefulTransitionToCycleOff(Event event) override;
    void handleCycleOff(Event event) override;

  private:
    void sendPowerOffVdm();
    /**
     * @brief Consume the one-shot ignore for the SBIOS reset
     *
     * SBIOS resets the host part way through its power sequence, so the first
     * PowerOk de-assert after the host comes up is not a real power off. When
     * the ignore is armed this clears it, starts the settle fallback and
     * returns true so the caller leaves the power state alone.
     *
     * @return true if this de-assert must be ignored
     */
    bool consumePowerOkDeAssertIgnore();
    /** Give the erot log parser a wall-clock anchor for its boot-relative
     * timestamps. */
    void sendTimestampVdm();
    /** Notify the erot, assert G3SoftEn and drive PexResetN low. */
    void completePowerDown();
    /** PexResetN released; settle for G3SoftPowerButtonDelayMs, then pulse. */
    void startPowerButtonDelay();
    int readGPIOInputValue(std::shared_ptr<ConfigData> config);
    /** Start the async G3Soft sequence (no blocking). */
    void startGNRPowerOnSequence();
    /** Timer callback: advance the power-on state machine. */
    void onGNRPowerOnTimer(const boost::system::error_code& ec);
    /** Returns true if G3Soft GPIOs are configured and PowerOk is deasserted.
     */
    bool shouldRunG3SoftSequence();
    /** Returns true if all three G3Soft GPIOs are present in the JSON config. */
    bool hasG3SoftSignals();
    /**
     * @brief Read the destination MCTP EID from the host config JSON
     *
     * Parses the optional top-level "mctp_eid" field of configFilePath.
     *
     * @return the configured EID, or nullopt when the field is absent
     * @throws std::runtime_error if the field is present but not an integer in
     *         the assignable EID range
     */
    std::optional<uint8_t> loadMctpEid();
    /** Send an erot notification VDM; no-op when no EID is configured. */
    void sendVdm(std::span<const uint8_t> packet, const std::string& what);

    static constexpr int ap0PollIntervalMs = 100;

    std::chrono::milliseconds getTimeoutWithDefault(
        const std::string& key, std::chrono::milliseconds default_t);

    GNRPowerOnPhase gnrPowerOnPhase{GNRPowerOnPhase::Idle};
    boost::asio::steady_timer gnrPowerOnTimer;
    /** Set while the host is up: the next PowerOk de-assert is the SBIOS
     * reset, not a power off. */
    bool ignoreNextPowerOkDeAssert{false};
    /** Backstop for an ignored de-assert that PowerOk never recovers from. */
    boost::asio::steady_timer sbiosResetSettleTimer;
    std::chrono::milliseconds sbiosResetSettleTimeout;
    std::chrono::steady_clock::time_point gnrPowerOnStartTime{};
    std::chrono::milliseconds g3SoftPowerButtonDelay;
    std::chrono::milliseconds pexResetPulse;
    std::chrono::milliseconds g3SoftAp0Timeout;
    /** Destination EID for erot VDMs; required when G3Soft is configured. */
    std::optional<uint8_t> mctpEid;
};

} // namespace power_control
