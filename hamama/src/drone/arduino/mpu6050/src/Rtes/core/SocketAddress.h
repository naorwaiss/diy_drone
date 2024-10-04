// SocketAddress.h
#ifndef SOCKET_ADDRESS_H
#define SOCKET_ADDRESS_H

#include <IPAddress.h>

/// @brief Represents a socket address.
struct SocketAddress
{
    IPAddress ip;
    uint16_t port;

    /// @brief Constructs a `SocketAddress`.
    /// @param ip IP address of type `IPAddress`.
    /// @param port Port number of type `uint16_t`.
    SocketAddress(const IPAddress &ip, uint16_t port) : ip(ip), port(port) {}

    /// @brief Compares this `SocketAddress` with another for equality.
    /// @param other Another `SocketAddress` to compare against.
    /// @return `true` if both addresses are equal, `false` otherwise.
    bool operator==(const SocketAddress &other) const
    {
        return ip == other.ip && port == other.port;
    }

    /// @brief Compares this `SocketAddress` with another for inequality.
    /// @param other Another `SocketAddress` to compare against.
    /// @return `true` if both addresses are not equal, `false` otherwise.
    bool operator!=(const SocketAddress &other) const
    {
        return !(*this == other);
    }

    // Defined as an equality for the map to use this as a key
    bool operator<(const SocketAddress &other) const
    {
        if (ip != other.ip)
        {
            for (int i = 0; i < 4; i++)
            {
                if (ip[i] != other.ip[i])
                {
                    return ip[i] < other.ip[i];
                }
            }
        }
        return port < other.port;
    }
};

#endif // SOCKET_ADDRESS_H
