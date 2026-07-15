import re

with open("src/bno055.cpp", "r") as f:
    text = f.read()

# 1. Add headers
text = text.replace("#include <sys/ioctl.h>", "#include <sys/ioctl.h>\n#include <termios.h>\n#include <poll.h>")

# 2. Add members to Impl
impl_search = "    uint8_t address_;\n    std::string i2c_device_;"
impl_replace = """    enum class ConnectionType { I2C, UART };
    ConnectionType conn_type_{ConnectionType::I2C};
    UARTConfig uart_config_;
    uint8_t address_;
    std::string i2c_device_;"""
text = text.replace(impl_search, impl_replace)

# 3. Add UART constructor to Impl
ctor_search = "    Impl(uint8_t address, std::string_view i2c_device)"
ctor_replace = """    Impl(const UARTConfig& uart_config) : uart_config_(uart_config) {
        conn_type_ = ConnectionType::UART;
    }

    Impl(uint8_t address, std::string_view i2c_device)"""
text = text.replace(ctor_search, ctor_replace)

# 4. Modify open_i2c to handle UART
open_i2c_search = """    bool open_i2c() {
        if (i2c_fd >= 0) {
            return true;
        }
#ifdef __linux__
        i2c_fd = open(i2c_device_.c_str(), O_RDWR);"""

open_i2c_replace = """    bool open_i2c() {
        if (i2c_fd >= 0) {
            return true;
        }
#ifdef __linux__
        if (conn_type_ == ConnectionType::UART) {
            i2c_fd = open(uart_config_.port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
            if (i2c_fd < 0) {
                log(LogLevel::Error, "Failed to open UART port: " + uart_config_.port);
                return false;
            }
            struct termios tty;
            if (tcgetattr(i2c_fd, &tty) != 0) {
                close(i2c_fd);
                i2c_fd = -1;
                return false;
            }
            speed_t speed = B115200;
            if (uart_config_.baudrate == 9600) speed = B9600;
            cfsetospeed(&tty, speed);
            cfsetispeed(&tty, speed);
            tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
            tty.c_iflag &= ~IGNBRK;
            tty.c_lflag = 0;
            tty.c_oflag = 0;
            tty.c_cc[VMIN]  = 0;
            tty.c_cc[VTIME] = static_cast<cc_t>(uart_config_.timeout * 10);
            tty.c_iflag &= ~(IXON | IXOFF | IXANY);
            tty.c_cflag |= (CLOCAL | CREAD);
            tty.c_cflag &= ~(PARENB | PARODD);
            tty.c_cflag &= ~CSTOPB;
            tty.c_cflag &= ~CRTSCTS;
            if (tcsetattr(i2c_fd, TCSANOW, &tty) != 0) {
                close(i2c_fd);
                i2c_fd = -1;
                return false;
            }
            return true;
        }

        i2c_fd = open(i2c_device_.c_str(), O_RDWR);"""
text = text.replace(open_i2c_search, open_i2c_replace)

# 5. Add UART read exact helper right before write8_raw
uart_helper = """
    bool uart_read_exact(uint8_t* buf, int len) {
        int read_bytes = 0;
        int timeout_ms = static_cast<int>(uart_config_.timeout * 1000);
        while (read_bytes < len) {
            struct pollfd pfd = { i2c_fd, POLLIN, 0 };
            int ret = poll(&pfd, 1, timeout_ms);
            if (ret > 0) {
                int n = ::read(i2c_fd, buf + read_bytes, len - read_bytes);
                if (n > 0) {
                    read_bytes += n;
                } else {
                    return false;
                }
            } else {
                return false;
            }
        }
        return true;
    }

    // Low-level raw methods"""
text = text.replace("    // Low-level raw methods", uart_helper)

# 6. Modify write8_raw
write8_search = """    bool write8_raw(uint8_t reg, uint8_t value) {
        if (i2c_fd < 0) return false;
#ifdef __linux__
        uint8_t buffer[2] = {reg, value};"""

write8_replace = """    bool write8_raw(uint8_t reg, uint8_t value) {
        if (i2c_fd < 0) return false;
#ifdef __linux__
        if (conn_type_ == ConnectionType::UART) {
            uint8_t buf[5] = {0xAA, 0x00, reg, 1, value};
            if (::write(i2c_fd, buf, 5) != 5) return false;
            uint8_t resp[2];
            if (!uart_read_exact(resp, 2)) return false;
            return (resp[0] == 0xEE && resp[1] == 0x01);
        }
        uint8_t buffer[2] = {reg, value};"""
text = text.replace(write8_search, write8_replace)

# 7. Modify writeLen_raw
writeLen_search = """    bool writeLen_raw(uint8_t reg, const uint8_t* buffer, uint8_t len) {
        if (i2c_fd < 0 || len > 31) return false;
#ifdef __linux__
        uint8_t write_buf[32];"""

writeLen_replace = """    bool writeLen_raw(uint8_t reg, const uint8_t* buffer, uint8_t len) {
        if (i2c_fd < 0 || len > 31) return false;
#ifdef __linux__
        if (conn_type_ == ConnectionType::UART) {
            std::vector<uint8_t> buf(4 + len);
            buf[0] = 0xAA; buf[1] = 0x00; buf[2] = reg; buf[3] = len;
            std::memcpy(buf.data() + 4, buffer, len);
            if (::write(i2c_fd, buf.data(), buf.size()) != (ssize_t)buf.size()) return false;
            uint8_t resp[2];
            if (!uart_read_exact(resp, 2)) return false;
            return (resp[0] == 0xEE && resp[1] == 0x01);
        }
        uint8_t write_buf[32];"""
text = text.replace(writeLen_search, writeLen_replace)

# 8. Modify read8_raw
read8_search = """    bool read8_raw(uint8_t reg, uint8_t& value) {
        if (i2c_fd < 0) return false;
#ifdef __linux__
        uint8_t reg_buf[1] = {reg};"""

read8_replace = """    bool read8_raw(uint8_t reg, uint8_t& value) {
        if (i2c_fd < 0) return false;
#ifdef __linux__
        if (conn_type_ == ConnectionType::UART) {
            uint8_t buf[4] = {0xAA, 0x01, reg, 1};
            if (::write(i2c_fd, buf, 4) != 4) return false;
            uint8_t resp[2];
            if (!uart_read_exact(resp, 2)) return false;
            if (resp[0] == 0xBB && resp[1] == 1) {
                return uart_read_exact(&value, 1);
            }
            return false;
        }
        uint8_t reg_buf[1] = {reg};"""
text = text.replace(read8_search, read8_replace)

# 9. Modify read8
read8_loop_search = """#ifdef __linux__
            uint8_t reg_buf[1] = {reg};
            if (::write(i2c_fd, reg_buf, 1) == 1) {
                if (::read(i2c_fd, &value, 1) == 1) {"""
read8_loop_replace = """#ifdef __linux__
            if (conn_type_ == ConnectionType::UART) {
                uint8_t buf[4] = {0xAA, 0x01, reg, 1};
                if (::write(i2c_fd, buf, 4) == 4) {
                    uint8_t resp[2];
                    if (uart_read_exact(resp, 2)) {
                        if (resp[0] == 0xBB && resp[1] == 1) {
                            if (uart_read_exact(&value, 1)) return true;
                        } else if (resp[0] == 0xEE && resp[1] == 0x07) {
                            // continue loop on error
                        }
                    }
                }
            } else {
                uint8_t reg_buf[1] = {reg};
                if (::write(i2c_fd, reg_buf, 1) == 1) {
                    if (::read(i2c_fd, &value, 1) == 1) {"""

text = text.replace(read8_loop_search, read8_loop_replace, 1)
text = text.replace("            } // end read8", "                }\n            }\n            } // end read8") # Manual fix if needed, wait I should use regex for closing brace.

# Let's do it smarter.
