import re

with open("src/bno055.cpp", "r") as f:
    content = f.read()

# Add termios
content = content.replace("#include <sys/ioctl.h>", "#include <sys/ioctl.h>\n#include <termios.h>\n#include <poll.h>")

# Add enum
impl_start = content.find("class BNO055::Impl {")
impl_public = content.find("public:", impl_start)

injection = """
    enum class ConnectionType { I2C, UART };
    ConnectionType conn_type_{ConnectionType::I2C};
    UARTConfig uart_config_;
"""
content = content[:impl_public + 7] + injection + content[impl_public + 7:]

# Add constructor
ctor1_pos = content.find("Impl(uint8_t address, std::string_view i2c_device)")
ctor2 = """
    Impl(const UARTConfig& uart_config) : uart_config_(uart_config) {
        conn_type_ = ConnectionType::UART;
    }
"""
content = content[:ctor1_pos] + ctor2 + content[ctor1_pos:]

# Replace open_i2c with open_device
content = content.replace("bool open_i2c() {", """
    bool open_device() {
        if (i2c_fd >= 0) return true;
#ifdef __linux__
        if (conn_type_ == ConnectionType::UART) {
            i2c_fd = open(uart_config_.port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
            if (i2c_fd < 0) {
                log(LogLevel::Error, "Failed to open UART port: " + uart_config_.port);
                return false;
            }
            struct termios tty;
            if (tcgetattr(i2c_fd, &tty) != 0) { close(i2c_fd); i2c_fd = -1; return false; }
            speed_t speed = B115200;
            switch(uart_config_.baudrate) {
                case 9600: speed = B9600; break;
                case 115200: speed = B115200; break;
                // Add more if needed, default to 115200
            }
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
            if (tcsetattr(i2c_fd, TCSANOW, &tty) != 0) { close(i2c_fd); i2c_fd = -1; return false; }
            return true;
        }
#endif
""")

content = content.replace("if (i2c_fd < 0 && !open_i2c())", "if (i2c_fd < 0 && !open_device())")

# Helper for UART read with timeout
uart_read_helper = """
    bool uart_read_exact(uint8_t* buf, int len) {
        int read_bytes = 0;
        int timeout_ms = static_cast<int>(uart_config_.timeout * 1000);
        while (read_bytes < len) {
            struct pollfd pfd = { i2c_fd, POLLIN, 0 };
            int ret = poll(&pfd, 1, timeout_ms);
            if (ret > 0) {
                int n = ::read(i2c_fd, buf + read_bytes, len - read_bytes);
                if (n > 0) read_bytes += n;
                else return false;
            } else {
                return false;
            }
        }
        return true;
    }
"""

content = content.replace("bool write8_raw(uint8_t reg, uint8_t value) {", uart_read_helper + """
    bool write8_raw(uint8_t reg, uint8_t value) {
        if (i2c_fd < 0) return false;
#ifdef __linux__
        if (conn_type_ == ConnectionType::UART) {
            uint8_t buf[5] = {0xAA, 0x00, reg, 1, value};
            if (::write(i2c_fd, buf, 5) != 5) return false;
            uint8_t resp[2];
            if (!uart_read_exact(resp, 2)) return false;
            return (resp[0] == 0xEE && resp[1] == 0x01);
        }
#endif
""")

content = content.replace("bool writeLen_raw(uint8_t reg, const uint8_t* buffer, uint8_t len) {", """
    bool writeLen_raw(uint8_t reg, const uint8_t* buffer, uint8_t len) {
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
#endif
""")

content = content.replace("bool read8_raw(uint8_t reg, uint8_t& value) {", """
    bool read8_raw(uint8_t reg, uint8_t& value) {
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
#endif
""")

# Fix readLen
content = content.replace("bool readLen(uint8_t reg, uint8_t* buffer, uint8_t len, int retries = 3) {", """
    bool readLen(uint8_t reg, uint8_t* buffer, uint8_t len, int retries = 3) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (int i = 0; i < retries; ++i) {
            if (i2c_fd < 0 && !open_device()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
#ifdef __linux__
            if (conn_type_ == ConnectionType::UART) {
                uint8_t buf[4] = {0xAA, 0x01, reg, len};
                if (::write(i2c_fd, buf, 4) == 4) {
                    uint8_t resp[2];
                    if (uart_read_exact(resp, 2)) {
                        if (resp[0] == 0xBB && resp[1] == len) {
                            if (uart_read_exact(buffer, len)) return true;
                        } else if (resp[0] == 0xEE && resp[1] == 0x07) {
                            // BUS_OVER_RUN_ERROR, wait and retry
                            std::this_thread::sleep_for(std::chrono::milliseconds(5));
                            continue;
                        }
                    }
                }
            } else {
                uint8_t reg_buf[1] = {reg};
                if (::write(i2c_fd, reg_buf, 1) == 1) {
                    if (::read(i2c_fd, buffer, len) == len) {
                        return true;
                    }
                }
            }
#else
            std::memset(buffer, 0, len);
            return true;
#endif
            diagnostics_.read_failures++;
            log(LogLevel::Warning, "readLen failed, retrying...");
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (reconnect()) {
            return false; // For simplicity just fail and let next loop retry
        }
        diagnostics_.read_failures++;
        log(LogLevel::Error, "readLen failed permanently");
        return false;
    }
""")

content = content.replace("BNO055::BNO055(uint8_t i2c_address, std::string_view i2c_device)", """
BNO055::BNO055(const UARTConfig& uart_config) : impl_(std::make_unique<Impl>(uart_config)) {}
BNO055::BNO055(uint8_t i2c_address, std::string_view i2c_device)
""")

with open("src/bno055.cpp", "w") as f:
    f.write(content)
