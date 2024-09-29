// RtesSession.h
#ifndef RTES_SESSION
#define RTES_SESSION

#include <vector>

#include "SocketAddress.h"

#define RTES_SESSION_NUMBER_OF_RECEIVE_REGISTERS 255

#define SESSION_PING_AVG_MEASURE_SIZE 5

class Rtes; // Forward declaration of Rtes class

using OnReceiveCallback = void (*)(const char *data);
using OnPingChangeCallback = void (*)(const uint16_t ping);

class RtesSession
{
public:
    /// @brief Socket Address.
    SocketAddress address;

    /// @brief Array of 255 registers to callbacks for every first byte data type.
    OnReceiveCallback onReceiveRegisters[RTES_SESSION_NUMBER_OF_RECEIVE_REGISTERS] = {nullptr};
    
    /// @brief Ping with session, measured as an average of the last `SESSION_PING_AVG_MEASURE_SIZE` pings.
    uint16_t ping = 0;

    /// @brief Stores the timestamp for every receive from the session to check if disconnected.
    uint32_t lastSignOfLife = 0;

    /// @brief Initializes a session with the given address.
    /// @param address The session address reference.
    /// @param rtesSocket The RTES socket instance reference.
    RtesSession(SocketAddress &address, Rtes &rtesSocket);

    /// @brief Emits a message to the session address.
    /// @param dataBuffer char pointer of the data (make sure it has a null-terminator).
    void emit(const char *dataBuffer);

    /// @brief Emits a message to the session address.
    /// @param firstByte Optional first byte to send before the rest of the message.
    /// @param dataBuffer char pointer of the data (make sure it has a null-terminator).
    void emit(uint8_t firstByte, const char *dataBuffer);

    /// @brief Same as `.emit` just with raw bytes instead of converting the char pointer.
    /// @param bytes Vector of unsigned bytes to send.
    void emitRaw(std::vector<uint8_t> bytes);

    /// @brief Same as `.emit` just with raw bytes instead of converting the char pointer.
    /// @param firstByte Optional first byte to send before the rest of the message.
    /// @param bytes Vector of unsigned bytes to send.
    void emitRaw(uint8_t firstByte, std::vector<uint8_t> bytes);

    void on(uint8_t dataType, const OnReceiveCallback &onReceiveCallback);

    void onDefault(const OnReceiveCallback &onReceiveCallback);

    void callOnDefault(const char *data);

    void handlePingSendRoutine();

    void receivedPong(uint8_t pongSequence);

    void onPingChange(const OnPingChangeCallback &onPingChangeCallback);

    //! add received ping also when using as client

private:
    /// @brief Rtes socket reference.
    Rtes &rtesSocket;

    /// @brief Default receive register callback if not matched on the registers.
    OnReceiveCallback onDefaultReceiveRegister = nullptr;

    OnPingChangeCallback onPingChangeCallback = nullptr;

    /// @brief Last ping timestamp.
    uint32_t lastPing = 0;

    uint8_t pingSequence = 0;

    /// @brief The ping delay after every succesful ping, 0 when waiting for ping response (pong).
    uint32_t pingDelay = 0;

    /// @brief Average ping out of the successful pings.
    uint32_t pingAvg[SESSION_PING_AVG_MEASURE_SIZE] = {0};

    /// @brief Index to assign the ping time to on the average array.
    uint8_t currentPingAvgIndex = 0;

    /// @brief Setting the first session ping to set all the pingAvg values to it.
    bool firstPing = true;

    /// @brief Sends a ping to the session address.
    void sendPing();
};

#endif // RTES_SESSION
