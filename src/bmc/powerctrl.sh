#!/bin/bash

# Power control script for x86-power-control
# Uses D-Bus to communicate with xyz.openbmc_project.State.Host and
# xyz.openbmc_project.State.Chassis services

# D-Bus service names and paths
HOST_SERVICE="xyz.openbmc_project.State.Host"
CHASSIS_SERVICE="xyz.openbmc_project.State.Chassis"
HOST_PATH="/xyz/openbmc_project/state/host0"
CHASSIS_PATH="/xyz/openbmc_project/state/chassis0"
HOST_IFACE="xyz.openbmc_project.State.Host"
CHASSIS_IFACE="xyz.openbmc_project.State.Chassis"

# Transition values
HOST_TRANSITION_ON="xyz.openbmc_project.State.Host.Transition.On"
HOST_TRANSITION_OFF="xyz.openbmc_project.State.Host.Transition.Off"
HOST_TRANSITION_REBOOT="xyz.openbmc_project.State.Host.Transition.Reboot"
HOST_TRANSITION_GRACEFUL_WARM_REBOOT="xyz.openbmc_project.State.Host.Transition.GracefulWarmReboot"
HOST_TRANSITION_FORCE_WARM_REBOOT="xyz.openbmc_project.State.Host.Transition.ForceWarmReboot"

CHASSIS_TRANSITION_ON="xyz.openbmc_project.State.Chassis.Transition.On"
CHASSIS_TRANSITION_OFF="xyz.openbmc_project.State.Chassis.Transition.Off"
CHASSIS_TRANSITION_POWER_CYCLE="xyz.openbmc_project.State.Chassis.Transition.PowerCycle"

# State values for comparison
HOST_STATE_RUNNING="xyz.openbmc_project.State.Host.HostState.Running"
HOST_STATE_OFF="xyz.openbmc_project.State.Host.HostState.Off"
CHASSIS_STATE_ON="xyz.openbmc_project.State.Chassis.PowerState.On"
CHASSIS_STATE_OFF="xyz.openbmc_project.State.Chassis.PowerState.Off"

#
# Power on the host
#
power_on()
{
    echo "Power On Host"
    echo "Use journalctl -uf -b xyz.openbmc_project.Chassis.Control.Power@0.service to monitor power on progress"
    busctl set-property "$HOST_SERVICE" "$HOST_PATH" "$HOST_IFACE" \
        RequestedHostTransition s "$HOST_TRANSITION_ON"
    if [ $? -eq 0 ]; then
        echo "Power on request sent successfully"
    else
        echo "Failed to send power on request"
        return 1
    fi
}

#
# Force power off (immediate, no graceful shutdown)
#
power_off()
{
    echo "Force Power Off Host"
    busctl set-property "$CHASSIS_SERVICE" "$CHASSIS_PATH" "$CHASSIS_IFACE" \
        RequestedPowerTransition s "$CHASSIS_TRANSITION_OFF"
    if [ $? -eq 0 ]; then
        echo "Force power off request sent successfully"
    else
        echo "Failed to send force power off request"
        return 1
    fi
}

#
# Force shutdown (same as power_off - immediate power removal)
#
do_shutdown_force()
{
    echo "Force Shutdown Host"
    busctl set-property "$CHASSIS_SERVICE" "$CHASSIS_PATH" "$CHASSIS_IFACE" \
        RequestedPowerTransition s "$CHASSIS_TRANSITION_OFF"
    if [ $? -eq 0 ]; then
        echo "Force shutdown request sent successfully"
    else
        echo "Failed to send force shutdown request"
        return 1
    fi
}

#
# Graceful shutdown request (asks host to shut down cleanly)
#
do_shutdown_request()
{
    echo "Graceful Shutdown Request"
    busctl set-property "$HOST_SERVICE" "$HOST_PATH" "$HOST_IFACE" \
        RequestedHostTransition s "$HOST_TRANSITION_OFF"
    if [ $? -eq 0 ]; then
        echo "Graceful shutdown request sent successfully"
    else
        echo "Failed to send graceful shutdown request"
        return 1
    fi
}

#
# Graceful power off (same as do_shutdown_request)
#
grace_off()
{
    echo "Graceful Power Off"
    do_shutdown_request
}

#
# Get current power status
#
power_status()
{
    echo "Power Status:"

    # Get host state
    local host_state
    host_state=$(busctl get-property "$HOST_SERVICE" "$HOST_PATH" "$HOST_IFACE" \
        CurrentHostState 2>/dev/null | awk '{print $2}' | tr -d '"')

    if [ -n "$host_state" ]; then
        echo "  Host State: $host_state"
        if [ "$host_state" == "$HOST_STATE_RUNNING" ]; then
            echo "  Host: Running"
        elif [ "$host_state" == "$HOST_STATE_OFF" ]; then
            echo "  Host: Off"
        else
            echo "  Host: Transitioning"
        fi
    else
        echo "  Host State: Unknown (service may not be running)"
    fi

    # Get chassis state
    local chassis_state
    chassis_state=$(busctl get-property "$CHASSIS_SERVICE" "$CHASSIS_PATH" "$CHASSIS_IFACE" \
        CurrentPowerState 2>/dev/null | awk '{print $2}' | tr -d '"')

    if [ -n "$chassis_state" ]; then
        echo "  Chassis State: $chassis_state"
        if [ "$chassis_state" == "$CHASSIS_STATE_ON" ]; then
            echo "  Chassis Power: On"
        elif [ "$chassis_state" == "$CHASSIS_STATE_OFF" ]; then
            echo "  Chassis Power: Off"
        else
            echo "  Chassis Power: Transitioning"
        fi
    else
        echo "  Chassis State: Unknown (service may not be running)"
    fi
}

#
# Reset/reboot the host (warm reboot)
#
reset()
{
    echo "Reset Host"
    busctl set-property "$HOST_SERVICE" "$HOST_PATH" "$HOST_IFACE" \
        RequestedHostTransition s "$HOST_TRANSITION_FORCE_WARM_REBOOT"
    if [ $? -eq 0 ]; then
        echo "Reset request sent successfully"
    else
        echo "Failed to send reset request"
        return 1
    fi
}

#
# Power cycle (power off then on)
#
power_cycle()
{
    echo "Power Cycle Host"
    busctl set-property "$HOST_SERVICE" "$HOST_PATH" "$HOST_IFACE" \
        RequestedPowerTransition s "$HOST_TRANSITION_REBOOT"
    if [ $? -eq 0 ]; then
        echo "Power cycle request sent successfully"
    else
        echo "Failed to send power cycle request"
        return 1
    fi
}

#
# Graceful warm reboot
#
graceful_warm_reboot()
{
    echo "Graceful Warm Reboot"
    busctl set-property "$HOST_SERVICE" "$HOST_PATH" "$HOST_IFACE" \
        RequestedHostTransition s "$HOST_TRANSITION_GRACEFUL_WARM_REBOOT"
    if [ $? -eq 0 ]; then
        echo "Graceful warm reboot request sent successfully"
    else
        echo "Failed to send graceful warm reboot request"
        return 1
    fi
}

#
# Force warm reboot
#
force_warm_reboot()
{
    echo "Force Warm Reboot"
    busctl set-property "$HOST_SERVICE" "$HOST_PATH" "$HOST_IFACE" \
        RequestedHostTransition s "$HOST_TRANSITION_FORCE_WARM_REBOOT"
    if [ $? -eq 0 ]; then
        echo "Force warm reboot request sent successfully"
    else
        echo "Failed to send force warm reboot request"
        return 1
    fi
}

#
# Usage/help
#
usage()
{
    echo "Usage: $0 <command>"
    echo ""
    echo "Commands:"
    echo "  power_on              - Power on the host"
    echo "  power_off             - Force power off (immediate)"
    echo "  do_shutdown_force     - Force shutdown (immediate)"
    echo "  do_shutdown_request   - Graceful shutdown request"
    echo "  grace_off             - Graceful power off"
    echo "  power_status          - Get current power status"
    echo "  reset                 - Reset/reboot the host"
    echo "  power_cycle           - Power cycle (off then on)"
    echo "  graceful_warm_reboot  - Graceful warm reboot"
    echo "  force_warm_reboot     - Force warm reboot"
    echo ""
}

# Main entry point - dispatch based on command
case "$1" in
    power_on)
        power_on
        ;;
    power_off)
        power_off
        ;;
    do_shutdown_force)
        do_shutdown_force
        ;;
    do_shutdown_request)
        do_shutdown_request
        ;;
    grace_off)
        grace_off
        ;;
    power_status)
        power_status
        ;;
    reset)
        reset
        ;;
    power_cycle)
        power_cycle
        ;;
    graceful_warm_reboot)
        graceful_warm_reboot
        ;;
    force_warm_reboot)
        force_warm_reboot
        ;;
    *)
        usage
        exit 1
        ;;
esac

