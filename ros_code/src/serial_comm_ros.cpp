#include "serial_comm_ros.h"

SerialCommROS::SerialCommROS(ros::NodeHandle& nh)
: nh_(nh), portname_("/dev/ttyACM0"), 
baudrate_(115200), loop_frequency_(200)
{
    ROS_INFO_STREAM("SerialCommROS - starts.");
    ROS_INFO_STREAM("Default  port name  : " << portname_);
    ROS_INFO_STREAM("Default  baud rate  : " << baudrate_);
    ROS_INFO_STREAM("Default  node rate  : " << loop_frequency_ << " Hz\n");

    // Get ROS parameters
    this->getParameters();
    
    ROS_INFO_STREAM("User-set port name  : " << portname_);
    ROS_INFO_STREAM("User-set baud rate  : " << baudrate_);
    ROS_INFO_STREAM("User-set node rate  : " << loop_frequency_ << " Hz\n");

    // construct serial communicator
    serial_communicator_ = std::make_shared<SerialCommunicator>(portname_, baudrate_);
    
    // subscriber
    sub_msg_to_send_ = nh_.subscribe<std_msgs::UInt16MultiArray>(topicname_msg_to_send_, 1, &SerialCommROS::callbackToSend, this);

    // publisher
    pub_msg_recv_ = nh_.advertise<std_msgs::Int8MultiArray>(topicname_msg_recv_,1);

    // run
    this->run();
};

SerialCommROS::~SerialCommROS(){
    ROS_INFO_STREAM("SerialCommROS - terminate.");
};

void SerialCommROS::getParameters(){

    if(!ros::param::has("~serial_port")) 
        throw std::runtime_error("SerialCommROS - no 'serial_port' is set. terminate program.\n");
    if(!ros::param::has("~baud_rate")) 
        throw std::runtime_error("SerialCommROS - no 'baud_rate' is set. terminate program.\n");
    if(!ros::param::has("~topicname_pwm"))      
        throw std::runtime_error("SerialCommROS - no 'topicname_pwm' is set. terminate program.\n");
    if(!ros::param::has("~topicname_from_nucleo"))  
        throw std::runtime_error("SerialCommROS - no 'topicname_from_nucleo' is set. terminate program.\n");
    if(!ros::param::has("~frequency"))  
        throw std::runtime_error("SerialCommROS - no 'frequency' is set. terminate program.\n");
    
    ros::param::get("~serial_port",           portname_);
    ros::param::get("~baud_rate",             baudrate_);
    ros::param::get("~topicname_pwm",         topicname_msg_to_send_);
    ros::param::get("~topicname_from_nucleo", topicname_msg_recv_);
    ros::param::get("~frequency",             loop_frequency_);

    ROS_INFO_STREAM("topicname PWM        : " << topicname_msg_to_send_);
    ROS_INFO_STREAM("topicname from nucleo: " << topicname_msg_recv_);

};

void SerialCommROS::run(){
    ROS_INFO_STREAM("SerialCommROS - rosnode runs at {" << loop_frequency_ <<"} Hz\n");
    ros::Rate rate(loop_frequency_);
    // ros::Rate rate(30000);

    ros::Time time_prev = ros::Time::now();
    ros::Time time_curr;
    while(ros::ok()){
        bool is_ready = isPacketReady();
        if(is_ready) {
            uint32_t len = getMessage(buf_recv_);
            
            if(len > 0){
                // publish the Received message from the Nucleo board.
                for(int i = 0; i < len; ++i) msg_recv_.data.push_back(buf_recv_[i]);
                USHORT_UNION voltage_ushort;
                voltage_ushort.bytes_[0] = buf_recv_[0];
                voltage_ushort.bytes_[1] = buf_recv_[1];

                float voltage_float= ((float)voltage_ushort.ushort_/65535.0f * 3.3f);
                // ROS_INFO_STREAM("VOLTAGE : " << voltage_float << " V");
                pub_msg_recv_.publish(msg_recv_);
                msg_recv_.data.clear();
            }
            else{
                ROS_WARN_STREAM("In 'getMessage()', it returns len == 0.  An empty packet might be received.");
            }
            
        }
        
        time_curr = ros::Time::now();
        double dt = (time_curr-time_prev).toSec();
        if( dt > 0.9999){
            this->showSerialStatistics(dt);
            time_prev = time_curr;
        }
        ros::spinOnce();
        rate.sleep();
    }
};

void SerialCommROS::callbackToSend(const std_msgs::UInt16MultiArray::ConstPtr& msg){
    int len = this->fill16bitsTo8bits(msg,buf_send_);
    sendMessage(buf_send_, len);
};  

void SerialCommROS::showSerialStatistics(double dt){
    static uint32_t seq_rx_success_prev = 0;
    static uint32_t seq_tx_success_prev = 0;

    uint32_t seq_rx_success;
    uint32_t seq_rx_crcerr;
    uint32_t seq_rx_oflerr;
    uint32_t seq_rx_ecp;
    serial_communicator_->getRXStatistics(seq_rx_success, seq_rx_crcerr, seq_rx_oflerr, seq_rx_ecp);
    
    uint32_t seq_tx_success;
    serial_communicator_->getTXStatistics(seq_tx_success);

    // Calculate Rate
    double freq_rx = (double)(seq_rx_success - seq_rx_success_prev)/dt;
    double freq_tx = (double)(seq_tx_success - seq_tx_success_prev)/dt;

    // Show statistics
    ROS_INFO_STREAM("RX: " << freq_rx << " Hz / seq- good: " << seq_rx_success << " / err- crc:" << seq_rx_crcerr << ",ofl:" << seq_rx_oflerr << ",excpt:" << seq_rx_ecp);
    ROS_INFO_STREAM("TX: " << freq_tx << " Hz / seq- good: " << seq_tx_success);

    // Update the previous data
    seq_rx_success_prev = seq_rx_success;
    seq_tx_success_prev = seq_tx_success;
};

bool SerialCommROS::isPacketReady(){
    return serial_communicator_->isPacketReady();
};

uint32_t SerialCommROS::getMessage(unsigned char* data){
    uint32_t len = 0;
    len = serial_communicator_->getPacket(data);
    return len;
};

void SerialCommROS::sendMessage(unsigned char* data, int len){
    serial_communicator_->sendPacket(data, len);
};

int SerialCommROS::fill16bitsTo8bits(const std_msgs::UInt16MultiArray::ConstPtr& msg, unsigned char* buf_send){
    int len = msg->data.size();
    
    USHORT_UNION data_union;
    for(int i = 0; i < len; ++i) {
        data_union.ushort_ = msg->data[i];
        buf_send_[2*i]     = data_union.bytes_[0];
        buf_send_[2*i+1]   = data_union.bytes_[1];
    }

    return len*2;      
};
