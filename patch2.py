import re

with open("src/bno055.cpp", "r") as f:
    text = f.read()

read8_loop_search = """#ifdef __linux__
            uint8_t reg_buf[1] = {reg};
            if (::write(i2c_fd, reg_buf, 1) == 1) {
                if (::read(i2c_fd, &value, 1) == 1) {
                    return true;
                }
            }"""

read8_loop_replace = """#ifdef __linux__
            if (conn_type_ == ConnectionType::UART) {
                uint8_t buf[4] = {0xAA, 0x01, reg, 1};
                if (::write(i2c_fd, buf, 4) == 4) {
                    uint8_t resp[2];
                    if (uart_read_exact(resp, 2)) {
                        if (resp[0] == 0xBB && resp[1] == 1) {
                            if (uart_read_exact(&value, 1)) return true;
                        } else if (resp[0] == 0xEE && resp[1] == 0x07) {
                            // continue loop, wait and retry
                        }
                    }
                }
            } else {
                uint8_t reg_buf[1] = {reg};
                if (::write(i2c_fd, reg_buf, 1) == 1) {
                    if (::read(i2c_fd, &value, 1) == 1) {
                        return true;
                    }
                }
            }"""
text = text.replace(read8_loop_search, read8_loop_replace)

readLen_loop_search = """#ifdef __linux__
            uint8_t reg_buf[1] = {reg};
            if (::write(i2c_fd, reg_buf, 1) == 1) {
                if (::read(i2c_fd, buffer, len) == len) {
                    return true;
                }
            }"""

readLen_loop_replace = """#ifdef __linux__
            if (conn_type_ == ConnectionType::UART) {
                uint8_t buf[4] = {0xAA, 0x01, reg, len};
                if (::write(i2c_fd, buf, 4) == 4) {
                    uint8_t resp[2];
                    if (uart_read_exact(resp, 2)) {
                        if (resp[0] == 0xBB && resp[1] == len) {
                            if (uart_read_exact(buffer, len)) return true;
                        } else if (resp[0] == 0xEE && resp[1] == 0x07) {
                            // continue loop, wait and retry
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
            }"""
text = text.replace(readLen_loop_search, readLen_loop_replace)

with open("src/bno055.cpp", "w") as f:
    f.write(text)

