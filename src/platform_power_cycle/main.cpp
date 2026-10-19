#include "platform_power_cycle.hpp"

#include <boost/asio/io_context.hpp>
#include <sdbusplus/asio/connection.hpp>

#include <cstdlib>
#include <exception>
#include <iostream>

int main()
{
    try
    {
        boost::asio::io_context io;
        sdbusplus::asio::connection bus(io);
        nvidia::platform_power_cycle::PlatformPowerCycle manager(
            bus, nvidia::platform_power_cycle::listUnitFiles(bus));

        // Claim the well-known name only after the complete API is published.
        // The systemd unit is Type=dbus, so acquiring the name advertises that
        // the service is ready for property reads and method calls.
        bus.request_name(nvidia::platform_power_cycle::busName.data());

        io.run();
    }
    catch (const std::exception& error)
    {
        std::cerr << "Failed to run platform power-cycle service: "
                  << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
