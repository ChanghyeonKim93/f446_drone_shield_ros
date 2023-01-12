#include "serial_comm_mbed.h"
#include "BufferedSerial.h"
#include "crc16.h"

SerialCommunicatorMbed::SerialCommunicatorMbed(int baud_rate, PinName pin_tx, PinName pin_rx)
// : buffered_serial_(USBTX, USBRX),
// : buffered_serial_(PA_9, PA_10),
: buffered_serial_(pin_tx, pin_rx)
//  signal_recv_(PD_0), signal_packet_ready_(PD_1), signal_etx_(PD_2), signal_send_(PD_3)
{
    //Serial baud rate
    buffered_serial_.set_baud(baud_rate);
    // buffered_serial_.set_format(8, BufferedSerial::None, 1);
    // buffered_serial_.set_blocking(false);

    flagStacking = false;
    flagDLEFound = false;

    idx_stk_ = 0;

    seq_recv_ = 0;
    len_packet_recv_ = 0;

    flag_packet_ready_ = false;

    seq_send_ = 0;
    len_packet_send_ = 0;
    flag_send_packet_ready_ = false;

    seq_recv_success_   = 0;
    seq_recv_crc_error_ = 0;
    seq_recv_overflow_  = 0;
};


void SerialCommunicatorMbed::send_withChecksum(const unsigned char* data, int len){
    // signal_send_ = true;
    USHORT_UNION crc16_calc;
    crc16_calc.ushort_ = crc16_ccitt(data, 0, len-1);

    buf_send_[0] = DLE; buf_send_[1] = STX; // DLE, STX --> start of the packet
    uint32_t idx       = 2;
    for(uint32_t i = 0; i < len; ++i) {
        if(data[i] == DLE){
            buf_send_[idx++]  = DLE;
            buf_send_[idx++]  = DLE;
        }
        else buf_send_[idx++] = data[i];
    }
    
    // len = idx - 2;
    if(crc16_calc.bytes_[0] == DLE){
        buf_send_[idx++]  = DLE;
        buf_send_[idx++]  = DLE;
    }
    else buf_send_[idx++] = crc16_calc.bytes_[0];

    if(crc16_calc.bytes_[1] == DLE){
        buf_send_[idx++]  = DLE;
        buf_send_[idx++]  = DLE;
    }
    else buf_send_[idx++] = crc16_calc.bytes_[1];

    buf_send_[idx++] = DLE; buf_send_[idx++] = ETX; // DLE, ETX --> end of the packet.

    buffered_serial_.write((void*)buf_send_, idx);
    // signal_send_ = false;
};

void SerialCommunicatorMbed::send_withoutChecksum(const unsigned char* data, int len){
    buffered_serial_.write((void*)data, len);
};


bool SerialCommunicatorMbed::tryToReadSerialBuffer(){
    flag_packet_ready_ = false;
    if(buffered_serial_.readable()) { // data recved.
    
        uint32_t len_read = buffered_serial_.read(buf_recv_, BUF_SIZE);
        if(len_read > 0) { // there is data
        
            for(uint32_t i = 0; i < len_read; ++i){
                // signal_recv_ = true;

                unsigned char c = buf_recv_[i];

                if(flagStacking){ // 현재 Packet stack 중...
                    if(flagDLEFound){ // 1) DLE, DLE / 2) DLE, ETX
                        if(c == DLE){ // 1) DLE, DLE --> 실제데이터가 DLE
                            flagDLEFound = false;

                            packet_stack_[idx_stk_] = c;
                            ++idx_stk_;
                        }
                        else if(c == ETX){
                            // signal_etx_ = true;

                            flagStacking = false;
                            flagDLEFound = false;

                            USHORT_UNION crc16_calc;
                            crc16_calc.ushort_ = crc16_ccitt(packet_stack_, 0, idx_stk_ - 3);

                            USHORT_UNION crc16_recv;
                            crc16_recv.bytes_[0] = packet_stack_[idx_stk_-2];
                            crc16_recv.bytes_[1] = packet_stack_[idx_stk_-1];

                            if(crc16_calc.ushort_ == crc16_recv.ushort_){
                                // CRC test OK!
                                // signal_packet_ready_ = true;

                                ++seq_recv_;
                                // Packet end. copy the packet.
                                len_packet_recv_ = idx_stk_ - 2;
                                for(int j = 0; j < len_packet_recv_; ++j){
                                    packet_recv_[j] = packet_stack_[j];
                                }
                                flag_packet_ready_ = true;

                                // signal_packet_ready_ = false;
                            }
                            else{
                                // CRC error!
                                ++seq_recv_crc_error_;
                                len_packet_recv_   = 0;
                                flag_packet_ready_ = false;
                            }

                            idx_stk_ = 0;
                            // signal_etx_ = false;
                        }
                        else{
                            // exceptional case
                            flagStacking = false;
                            flagDLEFound = false;
                            idx_stk_ = 0;
                        }
                    }
                    else { // DLE not found
                        if(c == DLE){  // DLE found
                            flagDLEFound = true;
                        }
                        else{
                            packet_stack_[idx_stk_] = c;
                            ++idx_stk_;
                            if(idx_stk_ >= 48){
                                flagStacking = false;
                                flagDLEFound = false;
                                idx_stk_     = 0;
                                ++seq_recv_overflow_;
                            }
                        }
                    }
                }
                else{ // 아직 STX를 발견하지 못함. (Stack 하지않음)
                    if(flagDLEFound) { // 이전에 DLE나옴.
                        if(c == STX) {  // STX 찾음, 새로운 packet을 stack 시작함.

                            flagStacking = true;
                            idx_stk_ = 0;

                            flag_packet_ready_ = false; 
                        }
                        flagDLEFound = false;
                    }
                    else{ // DLE 아직 발견 못함.
                        if(c == DLE) {
                            flagDLEFound = true;
                        }
                    }
                }// end if(flagStacking) or not
                // signal_recv_ = false;
            } // end for
        } // end if(len_read>0)
    }
    return flag_packet_ready_;
};

int SerialCommunicatorMbed::getReceivedMessage(unsigned char* msg){
    for(int i = 0; i < len_packet_recv_; ++i) {
        msg[i] = packet_recv_[i];
    }
    flag_packet_ready_ = false;
    return len_packet_recv_;
};

bool SerialCommunicatorMbed::writable() { return buffered_serial_.writable(); };
bool SerialCommunicatorMbed::readable() { return buffered_serial_.readable(); };