#ifndef SIMPLE_CAN_PUB_SUB_H
#define SIMPLE_CAN_PUB_SUB_H

#include <stdint.h>

class SimpleCANPubSub {
public:
    struct CANFrame {
        uint8_t topic;
        uint8_t subtopic;
        uint32_t payload;
        uint16_t crc;
    };

    // Encode CANFrame into an 8-byte array
    static void encodeFrame(const CANFrame &frame, uint8_t *outFrame);

    // Decode an 8-byte array into a CANFrame
    static bool decodeFrame(const uint8_t *inFrame, CANFrame &frame);

    // Generate CRC for a frame
    static uint16_t calculateCRC(const CANFrame &frame);

    // Send a CAN frame
    static void sendFrame(uint8_t topic, uint8_t subtopic, uint32_t payload, void (*sendFunction)(const uint8_t *frame));

    // Receive a CAN frame
    static bool receiveFrame(const uint8_t *frame, void (*callback)(const CANFrame &frame));
};

#endif
