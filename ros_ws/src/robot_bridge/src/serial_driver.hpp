#pragma once

#include <atomic>
#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/serial_port_base.hpp>
#include <cstddef>
#include <functional>
#include <string>
#include "common.hpp"
#include <yaml-cpp/yaml.h>
#include <deque>
#include <thread>

namespace serial {

class SerialDriver {
public:
    struct Params {
        unsigned int baud_rate    = 115200;
        unsigned int char_size    = 8;
        boost::asio::serial_port_base::parity::type       parity    = boost::asio::serial_port_base::parity::none;
        boost::asio::serial_port_base::stop_bits::type    stop_bits = boost::asio::serial_port_base::stop_bits::one;
        boost::asio::serial_port_base::flow_control::type flow_ctrl = boost::asio::serial_port_base::flow_control::none;
        std::string device_name;
        size_t read_buf_size = 4096;

        void load(const YAML::Node& config) {
            device_name   = config["device_name"].as<std::string>();
            baud_rate     = config["baud_rate"].as<unsigned int>();
            char_size     = config["char_size"].as<unsigned int>();
            read_buf_size = config["read_buf_size"].as<size_t>();
        }
    };

    using DataCallback  = std::function<void(const uint8_t*, std::size_t)>;
    using ErrorCallback = std::function<void(const std::string&)>;  // msg
    using InfoCallback  = std::function<void(const std::string&)>;

    explicit SerialDriver(const YAML::Node& config)
        : io_()
        , serial_(io_)
        , running_(false)
    {
        params_.load(config);
        read_buf_.resize(params_.read_buf_size);
    }

    ~SerialDriver() { stop(); }

    SerialDriver(const SerialDriver&)            = delete;
    SerialDriver& operator=(const SerialDriver&) = delete;

    void set_data_callback(DataCallback cb)   { data_cb_  = std::move(cb); }
    void set_error_callback(ErrorCallback cb) { error_cb_ = std::move(cb); }
    void set_info_callback(InfoCallback cb)   { info_cb_  = std::move(cb); }

    void start(const std::string& device_name = "") {
        if (running_) return;

        if (!device_name.empty()) {
            params_.device_name = device_name;
        }

        try {
            serial_.open(params_.device_name);
            serial_.set_option(boost::asio::serial_port_base::baud_rate(params_.baud_rate));
            serial_.set_option(boost::asio::serial_port_base::character_size(params_.char_size));
            serial_.set_option(boost::asio::serial_port_base::parity(params_.parity));
            serial_.set_option(boost::asio::serial_port_base::stop_bits(params_.stop_bits));
            serial_.set_option(boost::asio::serial_port_base::flow_control(params_.flow_ctrl));
        } catch (const std::exception& e) {
            log_error(std::string("Serial open error: ") + e.what());
            return;
        }

        running_ = true;
        io_.restart();
        do_read();

        io_thread_ = std::thread([this]() {
            try {
                io_.run();
            } catch (const std::exception& e) {
                log_error(std::string("Serial io_context error: ") + e.what());
            }
        });
    }

    void stop() {
        if (!running_.exchange(false)) return;

        close_serial();
        io_.stop();

        if (io_thread_.joinable()) {
            io_thread_.join();
        }
    }

    bool write(const std::vector<uint8_t>& data) {
        if (!running_) return false;
        boost::asio::post(io_, [this, data]() {
            bool write_in_progress = !write_queue_.empty();
            write_queue_.push_back(data);
            if (!write_in_progress) {
                do_write();
            }
        });
        return true;
    }

private:
    void do_read() {
        serial_.async_read_some(
            boost::asio::buffer(read_buf_),
            [this](boost::system::error_code ec, std::size_t length) {
                if (!ec) {
                    if (data_cb_) data_cb_(read_buf_.data(), length);
                    do_read();
                } else if (ec == boost::asio::error::operation_aborted) {
                    // normal shutdown
                } else {
                    log_error("Serial read error: " + ec.message());
                    stop();
                }
            });
    }

    void do_write() {
        boost::asio::async_write(
            serial_,
            boost::asio::buffer(write_queue_.front()),
            [this](boost::system::error_code ec, std::size_t /*length*/) {
                if (!ec) {
                    write_queue_.pop_front();
                    if (!write_queue_.empty()) {
                        do_write();
                    }
                } else if (ec == boost::asio::error::operation_aborted) {
                    write_queue_.clear();
                } else {
                    log_error("Serial write error: " + ec.message());
                    stop();
                }
            });
    }

    void close_serial() {
        if (serial_.is_open()) {
            boost::system::error_code ec;
            serial_.cancel(ec);
            serial_.close(ec);
            if (ec) {
                log_error("Error closing serial port: " + ec.message());
            }
        }
    }

    void log_error(const std::string& msg) {
        if (error_cb_) error_cb_(msg);
    }

private:
    Params params_;

    boost::asio::io_context   io_;
    boost::asio::serial_port  serial_;

    std::vector<uint8_t>             read_buf_;
    std::deque<std::vector<uint8_t>> write_queue_;

    std::atomic<bool> running_;
    std::thread       io_thread_;

    DataCallback  data_cb_;
    ErrorCallback error_cb_;
    InfoCallback  info_cb_;
};

} // namespace serial
