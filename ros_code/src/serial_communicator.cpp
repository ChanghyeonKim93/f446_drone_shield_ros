#include "serial_communicator.h"

SerialCommunicator::SerialCommunicator(const std::string& portname, const int& baud_rate)
    : seq_recv_(0), seq_send_(0), 
    len_packet_recv_(0), len_packet_send_(0),
    seq_recv_crc_error_(0),  seq_recv_overflow_(0),
    flag_packet_ready_(false), flag_send_packet_ready_(false),
    io_service_(), timeout_(io_service_)
{    
    // initialize mutex
    mutex_rx_ = std::make_shared<std::mutex>();
    mutex_tx_ = std::make_shared<std::mutex>();

    // initialize the portname     
    this->setPortName(portname);

    // Check whether this baud rate is valid
    this->checkSupportedBaudRate(baud_rate);

    // Try to open serial port.
    this->openSerialPort();    
    
    // Run TX RX threads
    this->terminate_future_ = this->terminate_promise_.get_future();
    this->runThreadTX();
    this->runThreadRX();
    
    printf("SerialCommunicator - port {%s} is open.\n", portname_.c_str());
};

// deconstructor
SerialCommunicator::~SerialCommunicator() {
    // Terminate signal .
    std::cerr << "SerialCommunicator - terminate signal published...\n";
    this->terminate_promise_.set_value();

    // wait for TX & RX threads to terminate ...
    std::cerr << "                   - waiting 1 second to join TX / RX threads ...\n";
    std::this_thread::sleep_for(1s);

    if(this->thread_rx_.joinable()) this->thread_rx_.join();
    std::cerr << "                   -   RX thread joins successfully.\n";

    if(this->thread_tx_.joinable()) this->thread_tx_.join();
    std::cerr << "                   -   TX thread joins successfully.\n";

    // Close the serial port.
    this->closeSerialPort();
    std::cerr << "SerialCommunicator - terminated.\n";
};

bool SerialCommunicator::isPacketReady(){
    bool res = false;
    // mutex_rx_->lock();
    res = this->flag_packet_ready_;
    // std::cout << "ready in comm? : " << flag_packet_ready_ <<", " << res <<std::endl;
    // mutex_rx_->unlock();
    return this->flag_packet_ready_;
};

uint32_t SerialCommunicator::getPacket(unsigned char* buf){
    uint32_t len = 0;
    mutex_rx_->lock();
    if(len_packet_recv_ > 0){
        len = len_packet_recv_;
        for(uint32_t i = 0; i < len; ++i) buf[i] = packet_recv_[i];

        // Initialize the flag 
        len_packet_recv_ = 0;
        flag_packet_ready_ = false;
    }
    mutex_rx_->unlock();

    // return len > 0 when data ready.
    // else 0.
    return len;
};

bool SerialCommunicator::sendPacket(unsigned char* buf, uint32_t len){
    bool isOK = len > 0;

    // update message & length
    this->mutex_tx_->lock();

    len_packet_send_        = len; 
    for(uint32_t i = 0; i < len; ++i) 
        packet_send_[i] = buf[i];

    flag_send_packet_ready_ = true; // Flag up!
    this->mutex_tx_->unlock();
    return isOK;
};

void SerialCommunicator::getRXStatistics(uint32_t& seq, uint32_t& seq_crcerr, uint32_t& seq_overflowerr, uint32_t& seq_exceptionerr){
    seq              = seq_recv_;
    seq_crcerr       = seq_recv_crc_error_;
    seq_overflowerr  = seq_recv_overflow_;
    seq_exceptionerr = seq_recv_exception_;
};

void SerialCommunicator::getTXStatistics(uint32_t& seq){
    seq = seq_send_;
};

void SerialCommunicator::setPortName(std::string portname){ portname_ = portname; };
void SerialCommunicator::setBaudRate(int baudrate) {
    checkSupportedBaudRate(baudrate);

    boost::asio::serial_port_base::baud_rate baud_rate_option2(baudrate);
    serial_->set_option(baud_rate_option2);
    boost::asio::serial_port_base::baud_rate baud_rate_option3;
    serial_->get_option(baud_rate_option3);
    std::cout << "SerialCommunicator - baudrate is changed from 115200 (default) to " << baud_rate_option3.value() << std::endl;
};

void SerialCommunicator::checkSupportedBaudRate(int baud_rate){
    if(   baud_rate == 57600
       || baud_rate == 115200
       || baud_rate == 230400
       || baud_rate == 460800
       || baud_rate == 500000
       || baud_rate == 576000
       || baud_rate == 921600
       || baud_rate == 1000000
       || baud_rate == 1152000
       || baud_rate == 1500000
       || baud_rate == 2000000
       || baud_rate == 2500000
       || baud_rate == 3000000
       || baud_rate == 3500000
       || baud_rate == 4000000)
    {
        // OK
        baud_rate_ = baud_rate;
    }
    else std::runtime_error("SerialCommunicator - Unsupported Baudrate...");
};


void SerialCommunicator::openSerialPort(){
    // Generate serial port.
    std::cout << "SerialCommunicator - opening the serial port...\n";
    serial_ = new boost::asio::serial_port(io_service_);

    // Try to open the serial port
    try{
        serial_->open(portname_);
    }
    catch (boost::system::system_error& error){
        std::cout << "SerialCommunicator - port [" << portname_.c_str() << "] cannot be opened. Error message:" << error.what() << std::endl;
        throw std::runtime_error(error.what());
    }

    // If serial port cannot be opened, 
    if ( !serial_->is_open() ) {
      std::cout << "SerialCommunicator - [" << portname_ <<"] is not opened. terminate the node\n";
      throw std::runtime_error("");
    }

    // Set serial port spec.
    // No flow control, 8bit / no parity / stop bit 1
    boost::asio::serial_port_base::baud_rate    baud_rate_option(baud_rate_);
    boost::asio::serial_port_base::flow_control flow_control(boost::asio::serial_port_base::flow_control::none);
    boost::asio::serial_port_base::parity       parity(boost::asio::serial_port_base::parity::none);
    boost::asio::serial_port_base::stop_bits    stop_bits(boost::asio::serial_port_base::stop_bits::one);

    serial_->set_option(baud_rate_option);
    serial_->set_option(flow_control);
    serial_->set_option(parity);
    serial_->set_option(stop_bits);
};

void SerialCommunicator::closeSerialPort(){
    serial_->close();
    std::cerr << "SerialCommunicator - portname [" << portname_ << "] is closed...\n";
};

void SerialCommunicator::runThreadRX(){
    this->thread_rx_ = std::thread([&](){ processRX(this->terminate_future_); } );
};

void SerialCommunicator::runThreadTX(){
    this->thread_tx_ = std::thread([&](){ processTX(this->terminate_future_); } );
};

void SerialCommunicator::processRX(std::shared_future<void> terminate_signal){
    // Initialize
    bool flagStacking = false;
    bool flagDLEFound = false;

    std::this_thread::sleep_for(1s);
    idx_stk_ = 0;
    timer::tic();
    boost::system::error_code ec;
    while(true){
        // Try to read serial port
        int len_read = serial_->read_some(boost::asio::buffer(buf_recv_, BUF_SIZE), ec);
        if(ec == boost::system::errc::interrupted){
            // error code 4. Interrupted... 괜찮을걸? ㅋㅋ
            continue;
        }
        else if(ec == boost::system::errc::no_such_file_or_directory){
            std::cout << "WARNING    ! - serial port might be disconnected... error code : " << ec << std::endl; 
        }
        else if(ec){
            // Error code는 boost::system::errc 에서 찾으면 된다.
            std::cout << "WARNING    ! - serial_->read_some(): error code : " << ec << std::endl;
        }

        if( len_read > 0 ) { // There is data
            // std::cout << "get new : " << len_read << std::endl;
            for(uint32_t i = 0; i < len_read; ++i) {
                
                unsigned char c = buf_recv_[i];
                // std::cout << "stkidx : " << idx_stk_ <<", char:"<< (int)c << std::endl;

                if( flagStacking ) { // 현재 Packet stack 중...
                    if( flagDLEFound ) { // 1) DLE, DLE / 2) DLE, ETX
                        if( c == DLE ) { // 1) DLE, DLE --> 실제데이터가 DLE
                            flagDLEFound = false;

                            packet_stack_[idx_stk_] = c;
                            ++idx_stk_;
                        }
                        else if( c == ETX ){ // 2) DLE, ETX
                            flagStacking = false;
                            flagDLEFound = false;

                            // Check CRC16-CCITT
                            USHORT_UNION crc16_calc;
                            crc16_calc.ushort_ = this->stringChecksumCRC16_CCITT(packet_stack_, 0, idx_stk_ - 3);

                            USHORT_UNION crc16_recv;
                            crc16_recv.bytes_[0] = packet_stack_[idx_stk_-2];
                            crc16_recv.bytes_[1] = packet_stack_[idx_stk_-1];

                            // std::cout << " crc check: " << crc16_recv.ushort_ <<", " << crc16_calc.ushort_ << std::endl;
                            
                            if(crc16_calc.ushort_ == crc16_recv.ushort_){
                                //CRC test OK!
                                // std::cout <<" CRC OK!!!  success: " << seq_recv_ <<", crc err: " << seq_recv_crc_error_ <<", overflow err: " << seq_recv_overflow_ <<"\n";
                                ++seq_recv_;

                                // Packet END. Copy the packet.
                                // std::cout << "== " << timer::toc(0) << "ms, ETX found. seq: " << seq_recv_ << ", length: " << idx_stk_-2 << std::endl;
                                mutex_rx_->lock();
                                len_packet_recv_ = idx_stk_ - 2;
                                // std::cout << " recved contents: ";
                                for(int j = 0; j < len_packet_recv_; ++j){
                                    packet_recv_[j] = packet_stack_[j];
                                    // std::cout << (int)packet_recv_[j] << " ";
                                }
                                // std::cout << std::endl;

                                mutex_rx_->unlock();
                                flag_packet_ready_ = true;
                                // std::cout << " flag packet ready?: " << flag_packet_ready_ << std::endl;
                            }
                            else{
                                std::cout << "WARNING    ! - CRC ERROR !\n" << std::endl;
                                std::cout << "CRC ERR    ! seq: " << seq_recv_ <<", crc: " << seq_recv_crc_error_ <<", ofl: " << seq_recv_overflow_ <<", ect:" << seq_recv_exception_<<"\n";
                                for(int j = 0; j < idx_stk_; ++j){
                                    std::cout << (int)packet_stack_[j] << " ";
                                }
                                std::cout << std::endl;

                                ++seq_recv_crc_error_;
                                len_packet_recv_   = 0;
                                flag_packet_ready_ = false;
                            }

                            idx_stk_ = 0;                  
                        }
                        else{
                            // exceptional case
                            ++seq_recv_exception_;
                            flagStacking = false;
                            flagDLEFound = false;
                            idx_stk_ = 0;

                            std::cout << "WARNING    ! - While stacking, DLE is found, but there is no ETX.\n" << std::endl;
                            std::cout << "DLE,no ETX ! seq: " << seq_recv_ <<", crc: " << seq_recv_crc_error_ <<", ofl: " << seq_recv_overflow_ <<", ect:" << seq_recv_exception_<<"\n";

                        }
                    }
                    else { // 이전에 DLE가 발견되지 않았다.
                        if(c == DLE){ // DLE발견
                            flagDLEFound = true;
                        }
                        else { // 스택.
                            packet_stack_[idx_stk_] = c;
                            ++idx_stk_; 
                            if(idx_stk_ >= 64){ // wierd error...
                                for(int kk = 0 ; kk < idx_stk_; ++kk){
                                    std::cout << (int) packet_stack_[kk] << " ";
                                }
                                std::cout << "\n";
                                flagStacking = false;
                                flagDLEFound = false;
                                idx_stk_ = 0;
                                ++seq_recv_overflow_;
                                std::cout << "WARNING    ! - RX STACK OVER FLOW!\n" << std::endl;
                                std::cout << "OVERFLOW   ! seq: " << seq_recv_ <<", crc: " << seq_recv_crc_error_ <<", ofl: " << seq_recv_overflow_ <<", ect:" << seq_recv_exception_<<"\n";
                            }
                            // std::cout << (int)buf_recv_[i] << std::endl;
                        }
                    }

                }
                else { // flagStacking == false    아직 STX를 발견하지 못함. (Stack 하지않음)
                    if(flagDLEFound){ // 이전에 DLE나옴.
                        if(c == STX) { // STX 찾음, 새로운 packet을 stack 시작함.
                            // std::cout << "== " << " found STX!\n";

                            flagStacking       = true;
                            idx_stk_   = 0;

                            // flag_packet_ready_ = false;
                        }
                        flagDLEFound = false; // STX 이든 아니든...
                    }
                    else { // 
                        // std::cout << " not found STX. DLE found. \n";
                        if(c == DLE) {
                            flagDLEFound = true;
                        }
                    }
                }
            }
        }

        std::future_status terminate_status = terminate_signal.wait_for(std::chrono::microseconds(10));
        if (terminate_status == std::future_status::ready) {
                    

            break;
        }
    }
    std::cerr << "SerialCommunicator - RX thread receives termination signal.\n";
};

void SerialCommunicator::processTX(std::shared_future<void> terminate_signal){
    while(true){
        if(flag_send_packet_ready_){
            uint32_t len_tmp = len_packet_send_;

            if( mutex_tx_->try_lock()) {
                flag_send_packet_ready_ = false;

                send_withChecksum(packet_send_, len_packet_send_);
                len_packet_send_        = 0;
                ++seq_send_;

                mutex_tx_->unlock();
                // std::cout << "                                           TX Send:" << seq_send_ << ", len: " << len_tmp << std::endl;
            }
        }


        std::future_status terminate_status = terminate_signal.wait_for(std::chrono::microseconds(50));
        if (terminate_status == std::future_status::ready){

            // Zero send.
            USHORT_UNION pwm;
            pwm.ushort_ = 0;
            std::cerr << "pwm ushort : " << pwm.ushort_ << "\n";

            mutex_tx_->lock();
            for(int i = 0; i < 8; ++i){
                packet_send_[2*i]   = pwm.bytes_[0];
                packet_send_[2*i+1] = pwm.bytes_[1];
            }

            mutex_tx_->unlock();

            std::cout << " Send zero signal ... trial 1...\n";
            mutex_tx_->lock();
            send_withChecksum(packet_send_,16);
            mutex_tx_->unlock();

            std::cout << " Wait 1 second...\n";
            sleep(1);

            std::cout << " Send zero signal ... trial 2...\n";
            mutex_tx_->lock();
            send_withChecksum(packet_send_,16);
            mutex_tx_->unlock();

            std::cout << "Zero signal done.\n";
            break;
        } 
    }
    std::cerr << "SerialCommunicator - TX thread receives termination signal.\n";
};

void SerialCommunicator::send_withChecksum(const unsigned char* data, int len){
    USHORT_UNION crc16_calc;
    crc16_calc.ushort_ = crc16_ccitt(data, 0, len-1);

    uint32_t idx       = 2;
    buf_send_[0] = DLE; buf_send_[1] = STX; // DLE, STX --> start of the packet
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

    // std::cout << "send data: ";
    // for(int i = 0; i < idx; ++i){
    //     std::cout << (int)buf_send_[i] <<" ";
    // }
    // std::cout << "\n";
    serial_->write_some(boost::asio::buffer(buf_send_, idx));
    // boost::asio::write(*serial_, boost::asio::buffer(buf_send_, idx));
    // std::cout << "write length : " << len +6 << std::endl;
};

unsigned short SerialCommunicator::stringChecksumCRC16_CCITT(const unsigned char* s, int idx_start, int idx_end){
    return crc16_ccitt(s,idx_start, idx_end);
};
