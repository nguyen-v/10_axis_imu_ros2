#include <sstream>
#include "imu/imu_cmd.h"
#include <serial/serial.h> 
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"

serial::Serial com; //Declare serial port object
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuMsg_pub;

//------------------------------------------------------------------------------
// Description: Serial port sends data to the device
// Input: buf[Len]=content to be sent
// Return: Returns the number of bytes sent
//------------------------------------------------------------------------------
int UART_Write(const U8 *buf, int Len)
{
    return com.write(buf, Len); //Send serial port data
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");//Set encoding to prevent Chinese garbled characters
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("imu_node");
    node->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
    std::string port_name;
    node->get_parameter<std::string>("port_name", port_name);//Get parameters
    imuMsg_pub = node->create_publisher<sensor_msgs::msg::Imu>("imu/data",20);

    try 
    {//Set the serial port properties and open the serial port
        com.setPort(port_name); 
        com.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        com.setTimeout(to); 
        com.open(); 
    }
    catch (serial::IOException& e) 
    { 
        RCLCPP_ERROR_STREAM(node->get_logger(), "Unable to open port-->" << port_name);
        return -1; 
    }
    if(!com.isOpen()) 
    {// Serial port opening failed
        return -1; 
    } 
    RCLCPP_INFO_STREAM(node->get_logger(), "Serial Port initialized");

    /**
     * Set device parameters
     * @param accStill    Inertial Navigation-Station Acceleration Threshold Unitdm/s?
     * @param stillToZero Inertial navigation -static zero return speed (unit cm/s) 0: No return to zero 255: Return to zero immediately
     * @param moveToZero  Inertial Navigation -Dynamic Zero Return Speed ​​(unit: cm/s) 0: No return to zero
     * @param isCompassOn Whether to use magnetic field fusion 0: Not used 1: Used
     * @param barometerFilter The filtering level of the barometer [value 0-3], the larger the value, the more stable it is but the worse the real-time performance.
     * @param reportHz The transmission frame rate of data actively reported [value 0-250HZ], 0 means 0.5HZ
     * @param gyroFilter    Gyroscope filter coefficient [value 0-2], the larger the value, the more stable it is but the worse the real-time performance.
     * @param accFilter     Accelerometer filter coefficient [value 0-4], the larger the value, the more stable it is but the worse the real-time performance.
     * @param compassFilter Magnetometer filter coefficient [value 0-9], the larger the value, the more stable it is but the worse the real-time performance.
     * @param Cmd_ReportTag Feature subscription tag
     */

    Cmd_12(5, 255, 0,  0, 2, 60, 1, 3, 5, 0x007f); // 1.Set parameters
    Cmd_03();//2.Wake up sensor
    Cmd_19();//3.Enable active data reporting
    Cmd_05();//4.Z axis angle reset to zero


    unsigned short  data_size;
    unsigned char   tmpdata[4096] ;
    
    rclcpp::Rate rate(100);//Frequency of message publishing
    while (rclcpp::ok())
    {//Process Imu data from the serial port
		//Number of serial port cache characters
        if(data_size = com.available())
        {//com.available(When the serial port does not have a cache, this function will wait until there is a cache before returning the number of characters.
            com.read(tmpdata, data_size);
            for(int i=0; i < data_size; i++)
            {
                Cmd_GetPkt(tmpdata[i]); // Transplantation: Fill in this function every time 1 byte of data is received. When a valid data packet is captured, it will call back and enter the Cmd_RxUnpack(U8 *buf, U8 DLen) function processing.
            }
        }
        //Process ROS information, such as subscribing to messages and calling callback functions
        rclcpp::spin_some(node->get_node_base_interface()); 
        rate.sleep();
    }

    return 0;
}
