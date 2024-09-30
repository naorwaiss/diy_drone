// RtesUtils.h

#ifndef RTES_UTILS
#define RTES_UTILS

#include <Arduino.h>
#include <QNEthernet.h>
#include <imxrt.h>
namespace RtesUtils
{

    /// @brief Gets teensy MAC address.
    /// @param pMacAddress pointer to store the teensy mac address to.
    void getTeensyMAC(uint8_t *pMacAddress);

    /// @brief Makes sure there can be an ethernet connection.
    /// @return `true` if a connection can be established; otherwise, `false`.
    bool hasPhysicalEthernet();

    /// @brief Encodes the given character buffer as unsigned bytes.
    /// @param charBuffer Pointer to a `const char` buffer. The buffer is assumed to be valid and null-terminated.
    /// @return A pointer to the same buffer cast to `const uint8_t*`.
    /// @note Ensure that the passed `charBuffer` is null-terminated.
    const uint8_t *encodeToBytes(const char *charBuffer);

    /**
     * @brief Prints the current network configuration using `qindesign::network::Ethernet`.
     *
     * This function retrieves and prints the following network information:
     * - Local IP address
     * - Subnet mask
     * - Gateway IP address
     * - DNS server IP address
     *
     * The information is printed to the serial output for debugging or monitoring purposes.
     */
    void printNetworkInfo();

}

#endif // RTES_UTILS