// To publsih  data via USB to Pico.

#ifndef SERIAL_COMM_HPP_
#define SERIAL_COMM_HPP_


#include <iostream>
#include <boost/asio.hpp>
#include <thread>

class GlowLed
{
    public:
        GlowLed(): port("/dev/ttyACM0"), baud_rate(115200), serial(io, port){
            //port = "/dev/ttyACM0";
            //baud_rate = 500000; //9600;

            serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        } 

        void send_data(std::string data){        
                    boost::asio::write(serial, boost::asio::buffer(data));                              
        }

    private:
        std::string port;
        unsigned int baud_rate;
        boost::asio::io_service io;
        boost::asio::serial_port serial;
};

#endif 

