// To publsih data via WiFi to ESP8226.

#ifndef SERIAL_COMM_HPP_
#define SERIAL_COMM_HPP_


#include <iostream>
#include <boost/asio.hpp>
#include <thread>

using boost::asio::ip::tcp;
namespace asio = boost::asio;

class GlowLed
{
    public:
        GlowLed(): PORT(12345), ESP8266_IP("192.168.4.1"), resolver(io_context), socket(io_context){
            auto endpoints = resolver.resolve(ESP8266_IP, std::to_string(PORT));

            // Create a socket and connect to the ESP8266
            asio::connect(socket, endpoints);

        } 

        void send_data(std::string data){        
            asio::write(socket, asio::buffer(data));
            //boost::asio::write(serial, boost::asio::buffer(data));                              
        }

        std::string receive_data() {
            boost::asio::streambuf buffer;
            boost::asio::read_until(socket, buffer, '~'); // Read until the delimiter '~'

            std::istream input_stream(&buffer);
            std::string response;
            std::getline(input_stream, response, '~'); // Extract data without the delimiter

            //std::cout << "Data received from ESP8266: " << response << "\n";
            return response;
        }

    private:
        //std::string port;
        const int PORT; // Port number

        //unsigned int baud_rate;

        //boost::asio::io_service io;
        asio::io_context io_context;

        //boost::asio::serial_port serial;
        tcp::resolver resolver;
        tcp::socket socket;

        const std::string ESP8266_IP; // ESP8266 IP address
};

#endif 



// // To publsih data via WiFi to ESP8226.

// #ifndef SERIAL_COMM_HPP_
// #define SERIAL_COMM_HPP_


// #include <iostream>
// #include <boost/asio.hpp>
// #include <thread>

// using boost::asio::ip::tcp;
// namespace asio = boost::asio;

// class GlowLed
// {
//     public:
//         GlowLed(): PORT(12345), ESP8266_IP("192.168.4.1"), resolver(io_context), socket(io_context){
//             auto endpoints = resolver.resolve(ESP8266_IP, std::to_string(PORT));

//             // Create a socket and connect to the ESP8266
//             asio::connect(socket, endpoints);

//         } 

//         void send_data(std::string data){        
//             asio::write(socket, asio::buffer(data));
//             //boost::asio::write(serial, boost::asio::buffer(data));                              
//         }

//     private:
//         //std::string port;
//         const int PORT; // Port number

//         //unsigned int baud_rate;

//         //boost::asio::io_service io;
//         asio::io_context io_context;

//         //boost::asio::serial_port serial;
//         tcp::resolver resolver;
//         tcp::socket socket;

//         const std::string ESP8266_IP; // ESP8266 IP address
// };

// #endif 

