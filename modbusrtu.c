#include <stdio.h>
#include <stdint.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

typedef enum {
    PARITY_NONE,
    PARITY_EVEN,
    PARITY_ODD
} parity_t;

typedef enum {
    STOPBITS_1,
    STOPBITS_2
} stopbits_t;

typedef struct serialPort {
    char *port_path;
    int baud_rate;
    int data_bits;
    parity_t parity;
    stopbits_t stop_bits;
    int file_descriptor;
} serialPort_t;

speed_t getBaud(int baud)
{
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        default: return B9600;
    }
}

int init_serial(serialPort_t *serialPort)
{
    serialPort->file_descriptor = open(serialPort->port_path, O_RDWR | O_NOCTTY);
    if (serialPort->file_descriptor < 0)
        return -1;

    struct termios options;
    if (tcgetattr(serialPort->file_descriptor, &options) != 0)
        return -1;

    // Set raw mode
    cfmakeraw(&options);

    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    options.c_cflag &= ~CSIZE;
    switch (serialPort->data_bits) {
        case 5: options.c_cflag |= CS5; break;
        case 6: options.c_cflag |= CS6; break;
        case 7: options.c_cflag |= CS7; break;
        default: options.c_cflag |= CS8; break;
    }

    switch (serialPort->parity) {
        case PARITY_NONE:
            options.c_cflag &= ~PARENB;
            break;
        case PARITY_EVEN:
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            break;
        case PARITY_ODD:
            options.c_cflag |= PARENB;
            options.c_cflag |= PARODD;
            break;
    }

    if (serialPort->stop_bits == STOPBITS_1)
        options.c_cflag &= ~CSTOPB;
    else
        options.c_cflag |= CSTOPB;

    speed_t br = getBaud(serialPort->baud_rate);
    cfsetispeed(&options, br);
    cfsetospeed(&options, br);

    // Enable receiver & ignore modem control
    options.c_cflag |= (CLOCAL | CREAD);

    // Disable hardware flow control
    options.c_cflag &= ~CRTSCTS;

    // Blocking read: wait for 1 byte
    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 0;

    // Flush before applying
    tcflush(serialPort->file_descriptor, TCIOFLUSH);

    if (tcsetattr(serialPort->file_descriptor, TCSANOW, &options) != 0)
        return -1;

    return 0;
}

uint16_t modbus_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];

        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }

    return crc;
}

int serial_write(serialPort_t *serialPort, uint8_t *frame, size_t length)
{
    size_t bytes_written = 0;

    while (bytes_written < length) {
        ssize_t n = write(
            serialPort->file_descriptor,
            frame + bytes_written,
            length - bytes_written
        );

        if (n < 0)
            return -1;

        bytes_written += n;
    }

    tcdrain(serialPort->file_descriptor);
    return bytes_written;
}


int serial_read(serialPort_t *serialPort, uint8_t *buffer, size_t length)
{
    int bytes_read = 0;

    while (bytes_read < length) {
        ssize_t n = read(serialPort->file_descriptor, buffer + bytes_read, length - bytes_read);
        if (n < 0) return -1;
        if (n == 0) break;
        bytes_read += n;
    }

    return bytes_read;
}

void close_serial(serialPort_t *serialPort)
{
    close(serialPort->file_descriptor);
}


int modbus_read_holding(serialPort_t *port,
                        uint8_t slave_id,
                        uint16_t start_addr,
                        uint16_t reg_count,
                        uint16_t *out_regs)
{
    uint8_t frame[8];
    frame[0] = slave_id;
    frame[1] = 0x03;                  
    frame[2] = start_addr >> 8;       
    frame[3] = start_addr & 0xFF;     
    frame[4] = reg_count >> 8;        
    frame[5] = reg_count & 0xFF;      

    uint16_t crc = modbus_crc16(frame, 6);
    frame[6] = crc & 0xFF;            
    frame[7] = crc >> 8;              

    serial_write(port, frame, 8);

    int expected = 5 + reg_count * 2;
    uint8_t response[256];

    int n = serial_read(port, response, expected);
    if (n != expected) return -1;

    uint16_t crc_resp = (response[n - 1] << 8) | response[n - 2];
    uint16_t crc_calc = modbus_crc16(response, n - 2);
    if (crc_resp != crc_calc) return -2;

    int byte_count = response[2];
    for (int i = 0; i < reg_count; i++) {
        out_regs[i] = (response[3 + 2*i] << 8) | response[4 + 2*i];
    }

    return 0;
}

int modbus_write_multiple(serialPort_t *port,
                          uint8_t slave_id,
                          uint16_t start_addr,
                          uint16_t reg_count,
                          uint16_t *values)
{
    uint8_t frame[260];

    frame[0] = slave_id;
    frame[1] = 0x10;
    frame[2] = start_addr >> 8;
    frame[3] = start_addr & 0xFF;
    frame[4] = reg_count >> 8;
    frame[5] = reg_count & 0xFF;
    frame[6] = reg_count * 2; 
    
    for (int i = 0; i < reg_count; i++) {
        frame[7 + 2*i]     = values[i] >> 8;
        frame[7 + 2*i + 1] = values[i] & 0xFF;
    }

    int frame_len = 7 + reg_count*2;

    uint16_t crc = modbus_crc16(frame, frame_len);
    frame[frame_len]     = crc & 0xFF;
    frame[frame_len + 1] = crc >> 8;

    serial_write(port, frame, frame_len + 2);

    uint8_t resp[8];
    int n = serial_read(port, resp, 8);
    if (n != 8) return -1;

    uint16_t crc_resp = (resp[7] << 8) | resp[6];
    uint16_t crc_calc = modbus_crc16(resp, 6);
    if (crc_resp != crc_calc) return -2;

    return 0;
}
