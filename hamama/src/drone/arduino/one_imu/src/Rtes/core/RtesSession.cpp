#include "RtesSession.h"
#include "../Rtes.h"

#include <assert.h>

RtesSession::RtesSession(
    SocketAddress &address,
    Rtes &rtesSocket) : address(address),
                        rtesSocket(rtesSocket)
{

    lastSignOfLife = millis();

    sendPing();
}

void RtesSession::emit(const char *dataBuffer)
{
    rtesSocket.sendTo(dataBuffer, address, 0x00);
}

void RtesSession::emit(uint8_t firstByte, const char *dataBuffer)
{
    rtesSocket.sendTo(dataBuffer, address, firstByte);
}

void RtesSession::emitRaw(std::vector<uint8_t> bytes)
{
    rtesSocket.sendToRaw(bytes, address, 0x00);
}

void RtesSession::emitRaw(uint8_t firstByte, std::vector<uint8_t> bytes)
{
    rtesSocket.sendToRaw(bytes, address, firstByte);
}

void RtesSession::on(uint8_t dataType, const OnReceiveCallback &onReceiveCallback)
{
    assert(dataType > RTES_RESERVED_FIRST_BYTE && "RTES::SESSION can't listen on reserved first byte.");
    //! Add assert to gaurd if not going to reserved
    //! Also check for not setting twice the same one
    onReceiveRegisters[dataType] = onReceiveCallback;
}

void RtesSession::onDefault(const OnReceiveCallback &onReceiveCallback)
{
    onDefaultReceiveRegister = onReceiveCallback;
}

void RtesSession::callOnDefault(const char *data)
{
    if (onDefaultReceiveRegister != nullptr)
    {
        onDefaultReceiveRegister(data);
    }
}

void RtesSession::handlePingSendRoutine()
{
    uint32_t now = millis();
    if (pingDelay != 0)
    {
        if (now - pingDelay < RTES_DEFAULT_PING_INTERVAL)
            return;
        sendPing();
    }
    else if (now - lastPing > RTES_PING_ABORT_TIMEOUT)
    {
        Serial.println("PING ABORTED, re-pinging...");
        sendPing();
    }
}

void RtesSession::receivedPong(uint8_t pongSequence)
{
    if (pingSequence != pongSequence)
    {
        Serial.printf("NOT THE SAME PING SEQUANCE, %d, %d\r\n", pingSequence, pongSequence);
        return;
    }
    // Get the successful ping time
    uint32_t pingTime = millis() - lastPing;

    // Set a delay of the current timestamp
    pingDelay = millis();

    if (firstPing)
    {
        firstPing = false;
        for (int i = 0; i < SESSION_PING_AVG_MEASURE_SIZE; i++)
            pingAvg[i] = pingTime;
    }
    else
    {
        pingAvg[currentPingAvgIndex] = pingTime;
        currentPingAvgIndex = (currentPingAvgIndex + 1) % SESSION_PING_AVG_MEASURE_SIZE;
    }

    // Set `ping` to the new average ping

    int pingSum = 0;
    for (int i = 0; i < SESSION_PING_AVG_MEASURE_SIZE; i++)
        pingSum += pingAvg[i];

    int newPing = pingSum / SESSION_PING_AVG_MEASURE_SIZE;

    // If ping changed and onPingChangeCallback was set
    if (ping != newPing && onPingChangeCallback != nullptr)
        onPingChangeCallback(newPing);

    ping = newPing;
}

void RtesSession::onPingChange(const OnPingChangeCallback &onPingChangeCallback)
{
    this->onPingChangeCallback = onPingChangeCallback;
}

void RtesSession::sendPing()
{
    // Serial.println("Sending Ping...");

    // Sends ping to session address, pingSequence++ to get 0 to 255 wrapping with bit overflow
    rtesSocket.pingTo(address, ++pingSequence);

    // Time after sending the ping
    lastPing = millis();
    pingDelay = 0;
}
