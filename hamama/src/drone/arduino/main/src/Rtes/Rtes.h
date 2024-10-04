// Rtes.h
#ifndef RTES_H
#define RTES_H

#include <vector>
#include <map>

#include <Arduino.h>
#include <QNEthernet.h>

#include "core/RtesUtils.h"
#include "core/SocketAddress.h"
#include "core/RtesSession.h"

#define RTES_RESERVED_FIRST_BYTE 0x01

#define RTES_UDP_TX_PACKET_MAX_SIZE 4096 // 4 KB
#define RTES_DEFAULT_MAX_SESSIONS 1      // Only 1 session is allowed at once

#define RTES_DEFAULT_PING_INTERVAL 500 // 0.1 seconds
#define RTES_PING_ABORT_TIMEOUT 2500   // After 2.5 seconds it abort and will try to re-ping.

#define RTES_SESSION_MAX_INACTIVITY_DURATION 5000 // After 5 seconds of not receiving from that session, disconnect.

using OnConnectionCallback = void (*)(RtesSession &session);

/// @brief RTES - Real Time Ethernet Socket
class Rtes
{

public:
    SocketAddress address;
    unsigned int pingInterval;

    byte macAddress[6];

    /// @brief
    /// @param address the socket address <`SocketAddress`>
    /// @param pingInterval ping interval in milliseconds <`unsigned int`>
    Rtes(SocketAddress address, unsigned int pingInterval = RTES_DEFAULT_PING_INTERVAL);

    /// @brief Initializing Ethernet with the constructed address.
    /// @param maxSessions The number of sessions allowed.
    void begin(unsigned short maxSessions = RTES_DEFAULT_MAX_SESSIONS);

    /// @brief Initializing Ethernet with the constructed address.
    /// @param allowedStaticAddresses The addresses allowed to have sessions with.
    void begin(const std::vector<SocketAddress> allowedStaticAddresses);

    /// @brief Processes RTES operations by retrieving and handling the most recent incoming packet,
    /// and performing lifecycle updates.
    ///
    /// This method first calls `getAndHandleIncomingPacket` to check for and handle any incoming packets.
    /// It then calls `updateLifecycle` every few seconds to manage lifecycle-related tasks and cleanups.
    ///! Add pinging
    void process();

    /// @brief Sends the given bytes to the given address  regardless of if it is connected with a session to it.
    /// @param dataBuffer char pointer of the data (make sure it has a null-terminator).
    /// @param address The address to send the bytes to.
    /// @param firstByte Optional first byte to send before the rest of the message.
    void sendTo(const char *dataBuffer, SocketAddress &address, uint8_t firstByte = 0x00);

    /// @brief Same as `.sendTo` just with raw bytes instead of converting the char pointer.
    /// @param bytes Vector of unsigned bytes to send.
    /// @param address The address to send the bytes to.
    /// @param firstByte Optional first byte to send before the rest of the message.
    void sendToRaw(std::vector<uint8_t> bytes, SocketAddress &address, uint8_t firstByte = 0x00);

    /// @brief On connection callback that returns the the `OnConnectionCallback` with the newly created session.
    /// @param onConnectionCallback
    void onConnection(const OnConnectionCallback &onConnectionCallback);

    /// @brief Sends a ping to the given address.
    /// @param address The address it pings to.
    /// @param pingSequence The ping sequence to identify the ping.
    void pingTo(SocketAddress &address, uint8_t pingSequence);

    ~Rtes();

private:
    bool rtesBegan = false;
    unsigned short maxSessions = 0;
    std::vector<SocketAddress> allowedStaticAddresses = {};
    std::map<SocketAddress, uint32_t> ackPendingAddresses = {};
    std::map<SocketAddress, RtesSession *> sessions = {};

    char packetBuffer[RTES_UDP_TX_PACKET_MAX_SIZE];

    OnConnectionCallback onConnectionCallback = nullptr;

    uint32_t lastLifecycleUpdate;

    /// @brief Initializing Ethernet.
    void beginEthernet();

    /// @brief Retrieves the most recent incoming packet if available, processes it by calling relevant handlers, and then clears the buffer.
    ///
    /// This method invokes handlers such as `handleUnknownPacket` and `handleSessionPacket` to process the packet.
    void getAndHandleIncomingPacket();

    /// @brief Handles a packet that is not registered.
    /// @param address Packet address.
    /// @param data Packet data.
    void handleUnknownPacket(SocketAddress &address, char *data);

    /// @brief Handles a packet that in a session.
    /// @param session Session Object.
    /// @param data Data of the session packet.
    void handleSessionPacket(RtesSession &session, char *data);

    /// @brief Sends a packet of bytes to an address.
    /// @param bytes Vector of unsigned bytes to send.
    /// @param address The address to send the packet to.
    void sendPacket(std::vector<uint8_t> bytes, SocketAddress &address);

    /// @brief Checks if a session can be created with the given address.
    ///
    /// This method verifies whether a session can be created by checking if the maximum number of sessions has been reached
    /// or if the address is in the list of allowed static IPs.
    ///
    /// @param address The socket address to check.
    /// @return True if a session can be created with this address, meaning either the maximum number of sessions has not
    ///         been reached or the address is an allowed static IP.
    bool canCreateSession(SocketAddress &address) const;

    /// @brief Updates the RTES lifecycle by cleaning up unnecessary memory from disconnected sessions and handling late ACK pending addresses.
    void updateLifecycle();
};

#endif // RTES_H
