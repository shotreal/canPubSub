#include "SimpleCANPubSub.h"

// Encode CANFrame into an 8-byte array
void SimpleCANPubSub::encodeFrame(const CANFrame &frame, uint8_t *outFrame) {
    outFrame[0] = frame.topic;
    outFrame[1] = frame.subtopic;
    outFrame[2] = (frame.payload >> 24) & 0xFF;
    outFrame[3] = (frame.payload >> 16) & 0xFF;
    outFrame[4] = (frame.payload >> 8) & 0xFF;
    outFrame[5] = frame.payload & 0xFF;
    outFrame[6] = (frame.crc >> 8) & 0xFF;
    outFrame[7] = frame.crc & 0xFF;
}

// Decode an 8-byte array into a CANFrame
bool SimpleCANPubSub::decodeFrame(const uint8_t *inFrame, CANFrame &frame) {
    frame.topic = inFrame[0];
    frame.subtopic = inFrame[1];
    frame.payload = (inFrame[2] << 24) | (inFrame[3] << 16) | (inFrame[4] << 8) | inFrame[5];
    frame.crc = (inFrame[6] << 8) | inFrame[7];

    // Verify CRC
    return frame.crc == calculateCRC(frame);
}

// Generate CRC for a frame (simple XOR-based checksum)
uint16_t SimpleCANPubSub::calculateCRC(const CANFrame &frame) {
    uint16_t crc = 0xFFFF; // Initial value
    crc ^= frame.topic;
    crc ^= frame.subtopic;
    crc ^= (frame.payload >> 24) & 0xFF;
    crc ^= (frame.payload >> 16) & 0xFF;
    crc ^= (frame.payload >> 8) & 0xFF;
    crc ^= frame.payload & 0xFF;
    return crc;
}

// Send a CAN frame
void SimpleCANPubSub::sendFrame(uint8_t topic, uint8_t subtopic, uint32_t payload, void (*sendFunction)(const uint8_t *frame)) {
    CANFrame frame = {topic, subtopic, payload, 0};
    frame.crc = calculateCRC(frame);
    uint8_t encodedFrame[8];
    encodeFrame(frame, encodedFrame);
    sendFunction(encodedFrame);
}

// Receive a CAN frame
bool SimpleCANPubSub::receiveFrame(const uint8_t *frame, void (*callback)(const CANFrame &frame)) {
    CANFrame decodedFrame;
    if (decodeFrame(frame, decodedFrame)) {
        callback(decodedFrame);
        return true;
    }
    return false; // Invalid frame (CRC mismatch)
}
