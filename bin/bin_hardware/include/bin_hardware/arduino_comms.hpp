#ifndef BIN_HARDWARE_ARDUINO_COMMS_HPP
#define BIN_HARDWARE_ARDUINO_COMMS_HPP

#include <sstream>
#include <string>
#include <algorithm>
#include <chrono>
#include <thread>
#include <libserial/SerialPort.h>
#include <iostream>

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
    switch (baud_rate)
    {
        case 1200:   return LibSerial::BaudRate::BAUD_1200;
        case 1800:   return LibSerial::BaudRate::BAUD_1800;
        case 2400:   return LibSerial::BaudRate::BAUD_2400;
        case 4800:   return LibSerial::BaudRate::BAUD_4800;
        case 9600:   return LibSerial::BaudRate::BAUD_9600;
        case 19200:  return LibSerial::BaudRate::BAUD_19200;
        case 38400:  return LibSerial::BaudRate::BAUD_38400;
        case 57600:  return LibSerial::BaudRate::BAUD_57600;
        case 115200: return LibSerial::BaudRate::BAUD_115200;
        case 230400: return LibSerial::BaudRate::BAUD_230400;
        default:
            std::cout << "Error! Baud rate " << baud_rate
                      << " not supported! Defaulting to 57600" << std::endl;
            return LibSerial::BaudRate::BAUD_57600;
    }
}

class ArduinoComms
{
public:
    ArduinoComms() = default;

    void connect(const std::string &serial_device,
                 int32_t baud_rate,
                 int32_t timeout_ms)
    {
        timeout_ms_ = timeout_ms;
        serial_conn_.Open(serial_device);
        serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
        // Uno/Nano resets on DTR; wait for optiboot to finish before sending commands
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        serial_conn_.FlushIOBuffers();
    }

    void disconnect()
    {
        serial_conn_.Close();
    }

    bool connected() const
    {
        return serial_conn_.IsOpen();
    }

    std::string send_msg(const std::string &msg_to_send, bool print_output = false)
    {
        serial_conn_.FlushIOBuffers();   // flush stale bytes before every transaction
        serial_conn_.Write(msg_to_send);

        std::string response;
        try
        {
            serial_conn_.ReadLine(response, '\n', timeout_ms_);
        }
        catch (const LibSerial::ReadTimeout &)
        {
            std::cerr << "ReadLine() timed out waiting for response to: "
                      << msg_to_send << std::endl;
        }

        if (print_output)
        {
            std::cout << "Sent: " << msg_to_send
                      << "  Recv: " << response << std::endl;
        }

        return response;
    }

    void send_empty_msg()
    {
        send_msg("\r");
    }

    // Reads 2 encoder values: left, right
    // Arduino sends: "left right\n"
    // Returns false if the reply timed out or was truncated; left/right are then left
    // untouched. Never write a partial parse through — the encoders are absolute, so a
    // fabricated zero reads as the wheels teleporting back to their startup position.
    bool read_encoder_values(int &left, int &right)
    {
        std::string response = send_msg("e\r");

        response.erase(std::remove(response.begin(), response.end(), '\r'), response.end());
        response.erase(std::remove(response.begin(), response.end(), '\n'), response.end());

        std::istringstream ss(response);
        int l = 0, r = 0;
        if (!(ss >> l >> r))
        {
            return false;
        }

        left  = l;
        right = r;
        return true;
    }

    // Sends 2 motor speed values: left, right
    void set_motor_values(int left, int right)
    {
        std::stringstream ss;
        ss << "m " << left << " " << right << "\r";
        send_msg(ss.str());
    }

    // Compiled lid firmware (servo_driver.h): 's 1' opens, any other value closes. Replies "OK".
    void set_lid(bool open)
    {
        send_msg(open ? "s 1\r" : "s 0\r");
    }

    struct DiagSnapshot {
        long  l_enc = 0,  r_enc = 0;
        float l_tgt_rpm = 0, r_tgt_rpm = 0;
        float l_rpm = 0,     r_rpm = 0;
        float l_err = 0,     r_err = 0;
        float l_int = 0,     r_int = 0;
        long  l_out = 0,     r_out = 0;
        float lin_vel = 0,   ang_vel = 0;
        bool  valid = false;
    };

    // Sends 'q', parses 14-field 'D ...' response into a DiagSnapshot.
    DiagSnapshot read_diagnostics()
    {
        DiagSnapshot s;
        std::string response = send_msg("q\r");
        response.erase(std::remove(response.begin(), response.end(), '\r'), response.end());
        response.erase(std::remove(response.begin(), response.end(), '\n'), response.end());

        if (response.size() < 2 || response[0] != 'D' || response[1] != ' ')
            return s;

        std::istringstream ss(response.substr(2));
        if (!(ss >> s.l_enc >> s.r_enc
                 >> s.l_tgt_rpm >> s.r_tgt_rpm
                 >> s.l_rpm     >> s.r_rpm
                 >> s.l_err     >> s.r_err
                 >> s.l_int     >> s.r_int
                 >> s.l_out     >> s.r_out
                 >> s.lin_vel   >> s.ang_vel))
            return s;

        s.valid = true;
        return s;
    }

    void set_pid_values(int lKp, int lKd, int lKi, int lKo,
                        int rKp, int rKd, int rKi, int rKo)
    {
        std::stringstream ss;
        ss << "u " << lKp << ":" << lKd << ":" << lKi << ":" << lKo
           << " "  << rKp << ":" << rKd << ":" << rKi << ":" << rKo << "\r";
        send_msg(ss.str());
    }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_{50};   // safe default; override via connect()
};

#endif  // BIN_HARDWARE_ARDUINO_COMMS_HPP
