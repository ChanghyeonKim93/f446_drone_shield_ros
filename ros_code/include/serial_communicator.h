#ifndef _SERIAL_COMMUNICATOR_H_
#define _SERIAL_COMMUNICATOR_H_

#define BUF_SIZE 1024

// C++
#include <iostream>
#include <string>
#include <cstring>
#include <numeric>
#include <exception>

// Thread
#include <thread>
#include <chrono>
#include <future>

// Boost serial
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

#include "union_struct.h"
#include "crc16.h" // CRC16 checksum test
#include "timer.h"

#define DLE 0x10
#define STX 0x02
#define ETX 0x03

using namespace std::chrono_literals;

class SerialCommunicator {
public:
    SerialCommunicator(const std::string& portname, const int& baud_rate);
    ~SerialCommunicator();

    // Get packet
    bool isPacketReady();
    uint32_t getPacket(unsigned char* buf);

    // Send packet
    bool sendPacket(unsigned char* buf, uint32_t len);

    // Statistics
    void getRXStatistics(uint32_t& seq, uint32_t& seq_crcerr, uint32_t& seq_overflowerr, uint32_t& seq_exceptionerr);
    void getTXStatistics(uint32_t& seq);

private:
    void runThreadRX();
    void runThreadTX();

    void processRX(std::shared_future<void> terminate_signal);
    void processTX(std::shared_future<void> terminate_signal);

private:
    void setPortName(std::string portname);
    void setBaudRate(int baudrate);
    void checkSupportedBaudRate(int baud_rate);

// Port settings
private:
    void openSerialPort();
    void closeSerialPort();

private:
    void send_withChecksum(const unsigned char* data, int len);

private:
    unsigned short stringChecksumCRC16_CCITT(const unsigned char* s, int idx_start, int idx_end);


// Serial port related (boost::asio::serial )
private:
    std::string portname_;
    int         baud_rate_;

    boost::asio::serial_port*   serial_;
    boost::asio::io_service     io_service_;
    boost::asio::deadline_timer timeout_;
    
    uint32_t idx_stk_;
    unsigned char packet_stack_[BUF_SIZE];

// Related to RX
private:
    uint32_t seq_recv_;
    unsigned char buf_recv_[BUF_SIZE];

    uint32_t len_packet_recv_;
    unsigned char packet_recv_[BUF_SIZE];

    std::atomic<bool> flag_packet_ready_;

// Related to TX
private:
    uint32_t seq_send_;
    unsigned char buf_send_[BUF_SIZE];

    uint32_t len_packet_send_;
    unsigned char packet_send_[BUF_SIZE];

    std::atomic<bool> flag_send_packet_ready_;

// For debug
private:
    uint32_t seq_recv_crc_error_;
    uint32_t seq_recv_overflow_;
    uint32_t seq_recv_exception_;

// Variables to elegantly terminate TX & RX threads
private:
    std::shared_future<void> terminate_future_;
    std::promise<void>       terminate_promise_;

// TX & RX threads and their mutex
private:
    std::thread thread_tx_;
    std::thread thread_rx_;

    std::shared_ptr<std::mutex> mutex_tx_;
    std::shared_ptr<std::mutex> mutex_rx_;
};


#endif