#pragma once


#include <atomic>
#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/serial_port_base.hpp>
#include <cstddef>
#include <string>
#include "common.hpp"
#include <yaml-cpp/yaml.h>
#include <deque>
namespace serial {
    class SerialDriver {
        public:
            struct Params {
                unsigned int boud_rate  = 115200 ;
                unsigned int char_size = 8 ;
                boost::asio::serial_port_base::parity::type parity = boost::asio::serial_port_base::parity::none ;
                boost::asio::serial_port_base::stop_bits::type stop_bit = boost::asio::serial_port_base::stop_bits::one ;
                boost::asio::serial_port_base::flow_control::type flow_contol = boost::asio::serial_port_base::flow_control::none ;
                std::string device_name ;
                size_t read_buf_size = 4096 ;

                void load (const YAML::Node& config) {
                    device_name = config["device_name"].as<std::string>();
                    boud_rate = config["boud_rate"].as<unsigned int>();
                    char_size = config["char_size"].as<unsigned int>();
                    read_buf_size = config["read_buf_size"].as<int>();
                }
            }Params_;

            SerialDriver (const YAML::Node& config):
                io_(),
                serial_(io_),
                read_buf_(Params_.read_buf_size),
                running_(false)
            {
                Params_.load(config);
            }


            ~SerialDriver () {
                stop() ;
            }


            void start () ;
            void stop () {
                if (!running_) return ;

                running_ = false ;
                serial_.cancel() ;
                serial_.close() ;
                io_.stop() ;
                if (io_thread_.joinable()) {
                    io_thread_.join() ;
                }
            }
        private:
            boost::asio::io_context io_;
            boost::asio::serial_port serial_ ;

            std::vector<uint8_t> read_buf_ ;
            std::deque<std::vector<uint8_t>> write_queue_ ;

            std::atomic<bool> running_ ;
            std::atomic<uint64_t> generation_ {0} ;

            std::thread io_thread_ ;


        } ;
    
} //namespace serial