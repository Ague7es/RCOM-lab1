// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <signal.h>
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

// Return values of readSupervisionFrame
#define FRAME_OK       0
#define FRAME_ERROR   -1
#define FRAME_TIMEOUT -2

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
// ALARM
////////////////////////////////////////////////
// volatile: these variables are changed inside the signal handler
volatile int alarmEnabled = FALSE;
volatile int alarmCount = 0;

// User-defined function to handle alarms (handler function)
// This function will be called when the alarm is triggered
void alarmHandler(int signal)
{
    // Can be used to change a flag that increases the number of alarms
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d received\n", alarmCount);
}

////////////////////////////////////////////////
// STATE MACHINE
////////////////////////////////////////////////
// Reads bytes from the serial port until a valid supervision frame
// [FLAG, a, c, a^c, FLAG] is received.
// If useAlarm is TRUE, it gives up when the alarm is triggered.
// Returns FRAME_OK on success, FRAME_TIMEOUT if the alarm was triggered,
// FRAME_ERROR on serial port error.
static int readSupervisionFrame(unsigned char a, unsigned char c, int useAlarm)
{
    State state = START;
    unsigned char byte;

    while (state != STOP_ST)
    {
        // Read one byte from serial port.
        // NOTE: You must check how many bytes were actually read by reading the return value.
        int r = readByteSerialPort(&byte);

        // The alarm was triggered: stop reading so the frame can be retransmitted.
        // This is checked before r < 0 because the alarm can interrupt read() (EINTR).
        if (useAlarm && alarmEnabled == FALSE)
            return FRAME_TIMEOUT;

        if (r < 0)
            return FRAME_ERROR; // serial port error
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

    return FRAME_OK;
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

    // Install the function signal to be automatically invoked when the timer
    // expires, invoking in its turn the user function alarmHandler
    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        return -1;
    }

    printf("Alarm configured\n");

    // Create string to send
    unsigned char set[5] = {FLAG, A_TX, C_SET, A_TX ^ C_SET, FLAG}; // BCC = 0x00

    // In non-canonical mode, '\n' does not end the writing.
    // Test this condition by placing a '\n' in the middle of the buffer.
    // The whole buffer must be sent even with the '\n'.

    // 1 first transmission + up to nRetransmissions retransmissions
    alarmCount = 0;
    while (alarmCount <= llParameters.nRetransmissions)
    {
        int bytes = writeBytesSerialPort(set, 5);
        printf("SET sent (%d bytes), attempt #%d\n", bytes, alarmCount + 1);

        // Enable alarm in t seconds
        alarmEnabled = TRUE;
        alarm(llParameters.timeout);

        // Wait for the UA frame (answer from the Receiver)
        int res = readSupervisionFrame(A_TX, C_UA, TRUE);

        if (res == FRAME_OK)
        {
            // Disable pending alarms, if any
            alarm(0);
            printf("UA received\n");

            // The serial port stays open: it is closed in llClose
            return 0;
        }

        if (res == FRAME_ERROR)
        {
            alarm(0);
            printf("Error reading UA\n");
            return -1;
        }

        // res == FRAME_TIMEOUT: the loop retransmits the SET frame
        printf("Timeout, no UA received\n");
    }

    printf("Maximum number of retransmissions reached. Giving up.\n");
    return -1;
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
    // The receiver does not use the alarm: it waits until a SET arrives.

    if (readSupervisionFrame(A_TX, C_SET, FALSE) != FRAME_OK)
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