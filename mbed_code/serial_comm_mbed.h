#ifndef _SERIAL_COMM_MBED_H_
#define _SERIAL_COMM_MBED_H_

#include "parameters.h"

#include "mbed.h"
#include "crc16.h"
#include "union_struct.h"
#include <cstdint>

#define BUF_SIZE 512

#define DLE 0x10
#define STX 0x02
#define ETX 0x03

class SerialCommunicatorMbed{
private:
    BufferedSerial buffered_serial_;

    bool flagStacking;
    bool flagDLEFound;

    uint32_t idx_stk_;
    unsigned char packet_stack_[BUF_SIZE];

// related to RX
private:
    uint32_t seq_recv_;
    unsigned char buf_recv_[BUF_SIZE];

    uint32_t len_packet_recv_;
    unsigned char packet_recv_[BUF_SIZE];

    bool flag_packet_ready_;

// related to TX
    uint32_t seq_send_;
    unsigned char buf_send_[BUF_SIZE];

    uint32_t len_packet_send_;
    unsigned char packet_send_[BUF_SIZE];

    bool flag_send_packet_ready_;

// for debug
private:
    uint32_t seq_recv_success_;
    uint32_t seq_recv_crc_error_;
    uint32_t seq_recv_overflow_;
    // uint32_t seq_

private:
    // DigitalOut signal_recv_;
    // DigitalOut signal_etx_;
    // DigitalOut signal_packet_ready_;
    // DigitalOut signal_send_;

public:
    SerialCommunicatorMbed(int baud_rate, PinName pin_tx, PinName pin_rx);
    
    void send_withChecksum(const unsigned char* data, int len);
    void send_withoutChecksum(const unsigned char* data, int len);

    bool tryToReadSerialBuffer();

    bool readable();
    bool writable();

    int getReceivedMessage(unsigned char* msg);

};

#endif