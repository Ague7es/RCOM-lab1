// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <stdio.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256

// Frame fields
#define FLAG  0x7E
#define A_TX  0x03 // frames sent by the Sender or answers from the Receiver
#define A_RX  0x01 // frames sent by the Receiver or answers from the Sender
#define C_SET 0x03
#define C_UA  0x07

// States of the supervision frame state machine
typedef enum
{
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP_ST
} State;

////////////////////////////////////////////////
// STATE MACHINE
////////////////////////////////////////////////
// Reads bytes from the serial port until a valid supervision frame
// [FLAG, a, c, a^c, FLAG] is received.
// Returns 0 on success, -1 on serial port error.
static int readSupervisionFrame(unsigned char a, unsigned char c)
{
    State state = START;
    unsigned char byte;

    while (state != STOP_ST)
    {
        // Read one byte from serial port.
        // NOTE: You must check how many bytes were actually read by reading the return value.
        int r = readByteSerialPort(&byte);
        if (r < 0)
            return -1; // serial port error
        if (r == 0)
            continue; // nothing read

        printf("Byte received: 0x%02X\n", byte);

        switch (state)
        {
        case START:
            if (byte == FLAG)
                state = FLAG_RCV;
            break;

        case FLAG_RCV:
            if (byte == a)
                state = A_RCV;
            else if (byte != FLAG)
                state = START; // repeated FLAG: stay in FLAG_RCV
            break;

        case A_RCV:
            if (byte == c)
                state = C_RCV;
            else if (byte == FLAG)
                state = FLAG_RCV;
            else
                state = START;
            break;

        case C_RCV:
            if (byte == (a ^ c))
                state = BCC_OK;
            else if (byte == FLAG)
                state = FLAG_RCV;
            else
                state = START;
            break;

        case BCC_OK:
            if (byte == FLAG)
                state = STOP_ST;
            else
                state = START;
            break;

        default:
            break;
        }
    }

    return 0;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llOpenTx(LinkLayer llParameters)
{
    // ----------------------------------------------------
    // This example code shows how to open the serial port and send a string.
    // TODO: Adapt and extend this code according to the specifications of the project.
    // ----------------------------------------------------

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    // Create string to send
    unsigned char set[5] = {FLAG, A_TX, C_SET, A_TX ^ C_SET, FLAG}; // BCC = 0x00

    // In non-canonical mode, '\n' does not end the writing.
    // Test this condition by placing a '\n' in the middle of the buffer.
    // The whole buffer must be sent even with the '\n'.

    int bytes = writeBytesSerialPort(set, 5);
    printf("%d bytes written to serial port\n", bytes);

    // Wait for the UA frame (answer from the Receiver)
    if (readSupervisionFrame(A_TX, C_UA) < 0)
    {
        printf("Error reading UA\n");
        return -1;
    }

    printf("UA received\n");

    // The serial port stays open: it is closed in llClose

    return 0;
}

int llOpenRx(LinkLayer llParameters)
{
    // ----------------------------------------------------
    // This example code shows how to open the serial port and receive a string.
    // TODO: Adapt and extend this code according to the specifications of the project.
    // ----------------------------------------------------

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    // Read from serial port until a valid SET frame is received.

    // NOTE: This while() cycle is a simple example showing how to read from the serial port.
    // It must be changed in order to respect the specifications of the protocol indicated in the Lab guide.
    // -> Replaced by the state machine in readSupervisionFrame().

    if (readSupervisionFrame(A_TX, C_SET) < 0)
    {
        printf("Error reading SET\n");
        return -1;
    }

    printf("SET received\n");

    unsigned char ua[5] = {FLAG, A_TX, C_UA, A_TX ^ C_UA, FLAG}; // BCC = 0x04

    int rx_send_bytes = writeBytesSerialPort(ua, 5);
    printf("%d bytes written to serial port\n", rx_send_bytes);

    // The serial port stays open: it is closed in llClose

    return 0;
}

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////
int llSend(const unsigned char *buf, int bufSize)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
int llReceive(unsigned char *packet)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llCloseTx()
{
    // TODO: Implement this function (DISC / DISC / UA exchange)

    // Wait until all bytes have been written to the serial port
    sleep(1);

    // Close serial port
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }

    printf("Serial port closed\n");

    return 0;
}

int llCloseRx()
{
    // TODO: Implement this function (DISC / DISC / UA exchange)

    // Wait until all bytes have been written to the serial port
    sleep(1);

    // Close serial port
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }

    printf("Serial port closed\n");

    return 0;
}