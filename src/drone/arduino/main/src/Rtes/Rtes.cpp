#include "Rtes.h"

#include <assert.h>

// #define RTES_DEBUG_MODE

// #ifdef RTES_DEBUG_MODE
// #define RTES_DEBUG_PRINT(msg) Serial.println(msg);
// #else
// #define RTES_DEBUG_PRINT(msg)
// #endif

#define NO_PACKET_SIZE -1

#define RTES_CONNECTION_PACKET_SIZE 2
#define RTES_CONNECTION_TYPE 0x01

#define RTES_CONNECTION_HANDSHAKE_SYN 0x01
#define RTES_CONNECTION_HANDSHAKE_SYN_ACK 0x02
#define RTES_CONNECTION_HANDSHAKE_ACK 0x03

#define RTES_CONNECTION_HANDSHAKE_TIMEOUT 1500 // Waits a 1.5 seconds before aborting the syn-ack connection

#define RTES_CONNECTION_PING 0x10
#define RTES_CONNECTION_PONG 0x11
#define RTES_CONNECTION_TERMINATE 0xFF

#define RTES_LIFECYCLE_INTERVAL 20000

// using namespace qindesign::network;

qindesign::network::EthernetUDP Udp;

Rtes::Rtes(SocketAddress address, unsigned int pingInterval)
    : address(address), pingInterval(pingInterval)
{
    RtesUtils::getTeensyMAC(macAddress);
    lastLifecycleUpdate = millis();
}

void Rtes::begin(unsigned short maxSessions)
{
    assert(maxSessions > 0 && "RTES Max sessions must be a positive non zero value.");
    this->maxSessions = maxSessions;
    beginEthernet();
}

void Rtes::begin(const std::vector<SocketAddress> allowedStaticAddresses)
{
    assert(!allowedStaticAddresses.empty() && "RTES Must have at least one static address allowed.");
    this->allowedStaticAddresses = allowedStaticAddresses;
    beginEthernet();
}

void Rtes::process()
{
    // Get the most recent incoming packet if available and handle it
    getAndHandleIncomingPacket();

    // TODO: set this as a .process of every session
    uint32_t now1 = millis();
    for (auto sessionsIterator = sessions.begin(); sessionsIterator != sessions.end();)
    {
        if (now1 - sessionsIterator->second->lastSignOfLife > RTES_SESSION_MAX_INACTIVITY_DURATION)
        {
            Serial.println("Disconneced");
            // Remove the session
            sessionsIterator = sessions.erase(sessionsIterator);
        }
        else
        {
            sessionsIterator->second->handlePingSendRoutine();
            ++sessionsIterator;
        }
    }

    // Check if it is time to update the lifecycle, if it is set new time to `lastLifecycleUpdate`
    uint32_t now = millis();
    if (now - lastLifecycleUpdate > RTES_LIFECYCLE_INTERVAL)
    {
        updateLifecycle();
        lastLifecycleUpdate = now;
    }
}

void Rtes::sendTo(const char *dataBuffer, SocketAddress &address, uint8_t firstByte)
{
    if (dataBuffer == nullptr)
        return;

    std::vector<uint8_t> bytes;
    size_t dataLength = strlen(dataBuffer); // Get the length of the string
    bytes.reserve(dataLength);              // Reserve space in the vector

    // Copy data into the vector
    for (size_t i = 0; i < dataLength; ++i)
    {
        bytes.push_back(static_cast<uint8_t>(dataBuffer[i]));
    }

    sendToRaw(bytes, address, firstByte);
}

void Rtes::sendToRaw(std::vector<uint8_t> bytes, SocketAddress &address, uint8_t firstByte)
{
    assert(firstByte == 0 || firstByte > RTES_RESERVED_FIRST_BYTE && "RTES First byte must be above the reserved first byte.");
    if (firstByte != 0)
        bytes.insert(bytes.begin(), firstByte);
    sendPacket(bytes, address);
}

void Rtes::onConnection(const OnConnectionCallback &onConnectionCallback)
{
    this->onConnectionCallback = onConnectionCallback;
}

void Rtes::pingTo(SocketAddress &address, uint8_t pingSequence)
{
    // Send ping
    sendPacket({RTES_CONNECTION_TYPE, RTES_CONNECTION_PING, pingSequence}, address);
}

Rtes::~Rtes()
{
    allowedStaticAddresses.clear();
    ackPendingAddresses.clear();
}

////////// PRIVATE METHODS //////////

void Rtes::beginEthernet()
{
    assert(!rtesBegan && "RTES Already initialized.");

    // Checking if an ethernet connection can be made (hardware-wise).
    while (!RtesUtils::hasPhysicalEthernet())
    {
        Serial.println("No physical ethernet is attached.");
    }

    // Attempting to initialize Ethernet connection
    while (!qindesign::network::Ethernet.begin(macAddress, address.ip))
    {
        Serial.println("Failed to configure Ethernet using static IP. Retrying...");
    }

    Udp.begin(address.port);
    rtesBegan = true;

    Serial.printf("\r\nServer Started On Port: %d\r\n", address.port);
    RtesUtils::printNetworkInfo();
}

void Rtes::getAndHandleIncomingPacket()
{
    int packetSize = Udp.parsePacket();
    if (packetSize == NO_PACKET_SIZE)
        return;

    // Get packet address
    SocketAddress remoteAddress = SocketAddress(Udp.remoteIP(), Udp.remotePort());

    // Get packet data, read it to `packetBuffer`
    Udp.read(packetBuffer, packetSize);
    Udp.flush();

    // Check if address is a session one.
    auto sessionIterator = sessions.find(remoteAddress);

    // No such sesson exists.
    if (sessionIterator == sessions.end())
    {
        if (packetSize == RTES_CONNECTION_PACKET_SIZE) // The packet size to establish a connection.
        {
            handleUnknownPacket(remoteAddress, packetBuffer);
        }
    }
    // Getting the session of that socket address.
    else
    {
        handleSessionPacket(*sessionIterator->second, packetBuffer);
    }

    // Clear the buffer by setting all bytes to zero
    memset(packetBuffer, 0, sizeof(packetBuffer));
}

void Rtes::handleUnknownPacket(SocketAddress &address, char *data)
{
    // If cannot create a session do nothing
    if (!canCreateSession(address))
        return;

    // uint8_t con[2] = {data[0], data[1]};

    if (data[0] != RTES_CONNECTION_TYPE)
        return;

    uint8_t connectionProtocolType = data[1];

    if (connectionProtocolType == RTES_CONNECTION_HANDSHAKE_SYN) // Received SYN
    {
        // Send SYN_ACK
        sendPacket({RTES_CONNECTION_TYPE, RTES_CONNECTION_HANDSHAKE_SYN_ACK}, address);

        // Start time for when sent
        ackPendingAddresses.insert(std::pair(address, millis()));
        return;
    }

    if (connectionProtocolType == RTES_CONNECTION_HANDSHAKE_ACK) // Received ACK
    {
        // Check for a ACK pending in that address
        auto ackPendingIterator = ackPendingAddresses.find(address);
        if (ackPendingIterator == ackPendingAddresses.end()) // No ACK pending found in that address
            return;

        // Get the ACK timeout
        uint32_t ackTimeout = millis() - ackPendingIterator->second;

        // If the current time is before the timeout period, create a session between them,
        // remove from `ackPendingAddresses` and call `onConnectionCallback` if one was assigned
        // Otherwise, resend the SYN connection handshake.
        if (ackTimeout < RTES_CONNECTION_HANDSHAKE_TIMEOUT)
        {
            RtesSession *session = new RtesSession(address, *this);

            // Create a session between them.
            sessions.insert(std::pair(address, session));

            // Remove address from the ack pending addresses
            ackPendingAddresses.erase(address);

            // If there is an on connection callback assigned call it with the newly created session
            if (onConnectionCallback != nullptr)
                onConnectionCallback(*session);
        }
        else
        {
            // Resend SYN
            sendPacket({RTES_CONNECTION_TYPE, RTES_CONNECTION_HANDSHAKE_SYN_ACK}, address);
            // Restart the time
            ackPendingAddresses[address] = millis();
        }
    }
}

void Rtes::handleSessionPacket(RtesSession &session, char *data)
{
    // Set new last sign of life for every received packet from session
    session.lastSignOfLife = millis();

    uint8_t dataType = data[0];
    if (dataType == RTES_CONNECTION_TYPE)
    {
        //! Handle when sent SYN send ACK back.

        switch (data[1])
        {
        case RTES_CONNECTION_PONG: // Received pong
            session.receivedPong(data[2]);
            return;

        case RTES_CONNECTION_HANDSHAKE_SYN: // Trying to connected when already connected
            sendPacket({RTES_CONNECTION_TYPE, RTES_CONNECTION_HANDSHAKE_SYN_ACK}, session.address);
            return;
        
        case RTES_CONNECTION_HANDSHAKE_ACK:
            Serial.println("Reconnected");
            return;

        default:
            Serial.println("CONNECTION TYPE");
        }
    }
    else if (session.onReceiveRegisters[dataType] != nullptr)
    {
        session.onReceiveRegisters[dataType](data + 1); // Adding 1 to start after the first byte
    }
    else
    {
        session.callOnDefault(data);
    }
}

void Rtes::sendPacket(std::vector<uint8_t> bytes, SocketAddress &address)
{
    Udp.beginPacket(address.ip, address.port);
    Udp.write(bytes.data(), bytes.size());
    Udp.endPacket();
}

bool Rtes::canCreateSession(SocketAddress &address) const
{
    // Check if `maxSessions` is set
    if (maxSessions != 0)
        return sessions.size() < maxSessions;

    // If `maxSessions` is not set then check if address is in the allowed static IPs list
    for (const SocketAddress &addr : allowedStaticAddresses)
    {
        if (addr == address)
            return true; // Address is allowed
    }
    return false; // Address is not allowed
}

void Rtes::updateLifecycle()
{
    // HANDLE LIFECYCLE
    Serial.println("LIFE CYCLE");
}