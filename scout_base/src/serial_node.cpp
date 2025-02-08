#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "scout_msgs/Scout_carrier.h"

#define Packet_start 0xFF
#define Packet_end 0xFe
#define Packet_up 0x01
#define Packet_down 0x02
#define Packet_size 6

serial::Serial ser;

uint8_t calculateCRC8(std::vector<uint8_t> &data, int length) {
    uint8_t crc = 0;
    for (int i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void sendPacket(uint8_t mode, uint8_t data1, uint8_t data2) {
    std::vector<uint8_t> packet = {Packet_start, mode, data1, data2, Packet_end};
    uint8_t crc = calculateCRC8(packet, packet.size());
    packet.push_back(crc);

    // 패킷 전송
    ser.write(packet);

    ROS_INFO("Sent Packet: [%02X %02X %02X %02X %02X %02X]",
             packet[0], packet[1], packet[2], packet[3], packet[4], packet[5]);
}

void carrier_callback(const scout_msgs::Scout_carrier::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->mode);
    if(msg->mode == 1) sendPacket(Packet_up, 0x00, 0x00);
    else if(msg->mode == 2) sendPacket(Packet_down, 0x00, 0x00);

}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Subscriber carrier_sub = nh.subscribe("test", 1, carrier_callback);

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    // ros::spin();
    // return 0;
        ros::Rate loop_rate(5);
    while(ros::ok()){

        ros::spinOnce();

        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << result.data);
        }
        loop_rate.sleep();

    }
}