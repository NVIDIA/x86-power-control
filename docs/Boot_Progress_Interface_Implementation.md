# Boot.Progress Interface Implementation

## Overview
This document describes the implementation of the `xyz.openbmc_project.State.Boot.Progress` interface in x86-power-control.

## D-Bus Interface Details

### Service Name
- `xyz.openbmc_project.State.Host0` (for host 0)
- Can be accessed with or without the "0" suffix for backward compatibility

### Object Path
- `/xyz/openbmc_project/state/host0` (for host 0)

### Interface Name
- `xyz.openbmc_project.State.Boot.Progress`

## Properties

### 1. BootProgress (string)
**Description**: Indicates the current boot stage of the host.

**Type**: String (enum value)

**Valid Values**:
- `xyz.openbmc_project.State.Boot.Progress.ProgressStages.Unspecified` - System is powered off or not booting (default)
- `xyz.openbmc_project.State.Boot.Progress.ProgressStages.PrimaryProcInit` - Primary processor initialization
- `xyz.openbmc_project.State.Boot.Progress.ProgressStages.BusInit` - Bus initialization
- `xyz.openbmc_project.State.Boot.Progress.ProgressStages.MemoryInit` - Memory initialization
- `xyz.openbmc_project.State.Boot.Progress.ProgressStages.SecondaryProcInit` - Secondary processor initialization
- `xyz.openbmc_project.State.Boot.Progress.ProgressStages.PCIInit` - PCI resource configuration
- `xyz.openbmc_project.State.Boot.Progress.ProgressStages.StartingOS` - Starting operating system boot process
- `xyz.openbmc_project.State.Boot.Progress.ProgressStages.SystemSetup` - System setup/configuration
- `xyz.openbmc_project.State.Boot.Progress.ProgressStages.SystemInitComplete` - System initialization complete, OS loading
- `xyz.openbmc_project.State.Boot.Progress.ProgressStages.OSRunning` - Operating system is running
- `xyz.openbmc_project.State.Boot.Progress.ProgressStages.OEM` - OEM-specific boot progress

**Read/Write**: Read/Write

**Behavior**: 
- When this property is updated, the `BootProgressLastUpdate` timestamp is automatically updated
- The property is automatically reset to `Unspecified` when the host powers off

### 2. BootProgressLastUpdate (uint64_t)
**Description**: Timestamp of when the BootProgress property was last updated.

**Type**: uint64_t (microseconds since epoch)

**Read/Write**: Read-only (automatically updated when BootProgress changes)

**Default**: 0

### 3. BootProgressOem (string)
**Description**: OEM-specific boot progress information. Only meaningful when BootProgress is set to "OEM".

**Type**: String

**Read/Write**: Read/Write

**Default**: "" (empty string)

## Usage Examples

### Reading Boot Progress via busctl
```bash
# Read BootProgress property
busctl get-property xyz.openbmc_project.State.Host0 \
    /xyz/openbmc_project/state/host0 \
    xyz.openbmc_project.State.Boot.Progress \
    BootProgress

# Read BootProgressLastUpdate
busctl get-property xyz.openbmc_project.State.Host0 \
    /xyz/openbmc_project/state/host0 \
    xyz.openbmc_project.State.Boot.Progress \
    BootProgressLastUpdate

# Read BootProgressOem
busctl get-property xyz.openbmc_project.State.Host0 \
    /xyz/openbmc_project/state/host0 \
    xyz.openbmc_project.State.Boot.Progress \
    BootProgressOem
```

### Setting Boot Progress via busctl
```bash
# Set BootProgress to MemoryInit
busctl set-property xyz.openbmc_project.State.Host0 \
    /xyz/openbmc_project/state/host0 \
    xyz.openbmc_project.State.Boot.Progress \
    BootProgress s "xyz.openbmc_project.State.Boot.Progress.ProgressStages.MemoryInit"

# Set OEM-specific boot progress
busctl set-property xyz.openbmc_project.State.Host0 \
    /xyz/openbmc_project/state/host0 \
    xyz.openbmc_project.State.Boot.Progress \
    BootProgressOem s "CustomOEMBootStage1"
```

### Using in Python with sdbus
```python
import asyncio
from sdbus import DbusInterfaceCommonAsync, dbus_property_async

class BootProgressInterface(DbusInterfaceCommonAsync,
                            interface_name='xyz.openbmc_project.State.Boot.Progress'):
    @dbus_property_async('s', property_name='BootProgress')
    def boot_progress(self) -> str:
        raise NotImplementedError

async def main():
    # Connect to the interface
    boot_progress = BootProgressInterface.new_proxy(
        'xyz.openbmc_project.State.Host0',
        '/xyz/openbmc_project/state/host0'
    )
    
    # Read current boot progress
    progress = await boot_progress.boot_progress
    print(f"Current boot progress: {progress}")
    
    # Set boot progress
    await boot_progress.boot_progress = \
        "xyz.openbmc_project.State.Boot.Progress.ProgressStages.OSRunning"

asyncio.run(main())
```

## Integration with External Services

### IPMI Integration
IPMI daemons (like phosphor-host-ipmid) can monitor host firmware sensor readings and update the BootProgress property accordingly:

```cpp
// Example: Update boot progress from IPMI sensor reading
conn->async_method_call(
    [](boost::system::error_code ec) {
        if (ec) {
            std::cerr << "Failed to set boot progress\n";
        }
    },
    "xyz.openbmc_project.State.Host0",
    "/xyz/openbmc_project/state/host0",
    "org.freedesktop.DBus.Properties",
    "Set",
    "xyz.openbmc_project.State.Boot.Progress",
    "BootProgress",
    std::variant<std::string>(
        "xyz.openbmc_project.State.Boot.Progress.ProgressStages.MemoryInit"
    )
);
```

### PLDM Integration
PLDM daemons can update boot progress based on platform event messages from the host:

```cpp
// Example: Update boot progress from PLDM event
if (sensorEvent == PLDM_BIOS_POST_MEMORY_INIT) {
    setBootProgress(
        "xyz.openbmc_project.State.Boot.Progress.ProgressStages.MemoryInit"
    );
}
```

## Internal Helper Functions

The implementation provides internal helper functions for use within x86-power-control:

### setBootProgress(const std::string& bootProgressStage)
Sets the boot progress and automatically updates the timestamp.

**Example**:
```cpp
setBootProgress("xyz.openbmc_project.State.Boot.Progress.ProgressStages.SystemInitComplete");
```

### setBootProgressOem(const std::string& oemProgress)
Sets the OEM-specific boot progress string.

**Example**:
```cpp
setBootProgressOem("VendorSpecificStage1");
```

## Automatic State Management

### Power Off Behavior
When the host transitions to `PowerState::off`, the BootProgress property is automatically reset to:
```
xyz.openbmc_project.State.Boot.Progress.ProgressStages.Unspecified
```

This ensures that stale boot progress information from a previous boot cycle is not retained.

## Code Locations

### Key Files Modified
- `src/power_control.cpp`:
  - Line ~216: Added `bootProgressIface` declaration
  - Line ~433-459: Added helper functions `setBootProgress()` and `setBootProgressOem()`
  - Line ~1076-1080: Added automatic reset of boot progress on power off
  - Line ~5654-5699: Boot Progress interface initialization

## Testing

### Manual Testing
1. **Power on the host**:
   ```bash
   busctl set-property xyz.openbmc_project.State.Host0 \
       /xyz/openbmc_project/state/host0 \
       xyz.openbmc_project.State.Host \
       RequestedHostTransition s "xyz.openbmc_project.State.Host.Transition.On"
   ```

2. **Monitor boot progress**:
   ```bash
   busctl monitor xyz.openbmc_project.State.Host0 \
       /xyz/openbmc_project/state/host0 \
       xyz.openbmc_project.State.Boot.Progress
   ```

3. **Simulate boot stage updates** (from external service):
   ```bash
   # Memory Init
   busctl set-property xyz.openbmc_project.State.Host0 \
       /xyz/openbmc_project/state/host0 \
       xyz.openbmc_project.State.Boot.Progress \
       BootProgress s "xyz.openbmc_project.State.Boot.Progress.ProgressStages.MemoryInit"
   
   # OS Running
   busctl set-property xyz.openbmc_project.State.Host0 \
       /xyz/openbmc_project/state/host0 \
       xyz.openbmc_project.State.Boot.Progress \
       BootProgress s "xyz.openbmc_project.State.Boot.Progress.ProgressStages.OSRunning"
   ```

4. **Power off and verify reset**:
   ```bash
   busctl set-property xyz.openbmc_project.State.Host0 \
       /xyz/openbmc_project/state/host0 \
       xyz.openbmc_project.State.Host \
       RequestedHostTransition s "xyz.openbmc_project.State.Host.Transition.Off"
   
   # Verify it's reset to Unspecified
   busctl get-property xyz.openbmc_project.State.Host0 \
       /xyz/openbmc_project/state/host0 \
       xyz.openbmc_project.State.Boot.Progress \
       BootProgress
   ```

### Logging
Boot progress updates are logged with:
```
lg2::info("Boot progress updated to: {PROGRESS}", "PROGRESS", bootProgressStage);
```

Check journalctl for these messages:
```bash
journalctl -u xyz.openbmc_project.Chassis.Control.Power@0.service -f | grep -i "boot progress"
```

## References
- [phosphor-dbus-interfaces Boot.Progress](https://github.com/openbmc/phosphor-dbus-interfaces/tree/master/yaml/xyz/openbmc_project/State/Boot)
- [phosphor-state-manager Host State Manager](https://github.com/openbmc/phosphor-state-manager)


