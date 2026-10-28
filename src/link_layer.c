// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256

// Frame fields
#define FLAG  0x7E
#define ESC   0x7D // escape octet used in byte stuffing
#define A_TX  0x03 // frames sent by the Sender or answers from the Receiver
#define A_RX  0x01 // frames sent by the Receiver or answers from the Sender
#define C_SET 0x03
#define C_UA  0x07
#define C_DISC 0x0B
#define C_I(n)   ((n) << 7)   // I frame number n:  0x00 / 0x80
#define C_RR(n)  (0xAA | (n)) // RR,  ready for n:  0xAA / 0xAB
#define C_REJ(n) (0x54 | (n)) // REJ, rejected n:   0x54 / 0x55

// Data field of an I frame (after destuffing): packet + BCC2
#define MAX_DATA_FIELD (MAX_PAYLOAD_SIZE + 1)
// Worst case I frame: every byte of the data field is stuffed (doubled)
#define MAX_I_FRAME (4 + 2 * MAX_DATA_FIELD + 1)

// Return values of readFrame / readSupervisionFrame
#define FRAME_OK       0
#define FRAME_ERROR   -1
#define FRAME_TIMEOUT -2

// States of the frame reception state machine
typedef enum
{
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    DATA_RCV, // reading the data field of an I frame
    ESC_RCV,  // previous byte of the data field was ESC
    STOP_ST
} State;

// Parameters saved in llOpen, used by the other functions
static LinkLayer connectionParameters;

// Transmitter: N(s) of the next I frame to send
static int ns = 0;
// Receiver: N(s) of the next I frame expected
static int nr = 0;
// Receiver: TRUE if the DISC of the transmitter was already received in llReceive
static int discReceived = FALSE;

// Protocol statistics, printed in llClose
static struct
{
    int iFramesSent;     // Tx: I frames written (including retransmissions)
    int retransmissions; // Tx: I frames written again (after timeout or REJ)
    int timeouts;        // Tx: timeouts waiting for RR/REJ
    int rejReceived;     // Tx: REJ frames received
    int iFramesAccepted; // Rx: new I frames passed to the application
    int duplicates;      // Rx: duplicate I frames received
    int rejSent;         // Rx: REJ frames sent (BCC2 errors)
} stats;

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

// Install the function signal to be automatically invoked when the timer
// expires, invoking in its turn the user function alarmHandler
static int installAlarm()
{
    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        return -1;
    }

    printf("Alarm configured\n");
    return 0;
}

////////////////////////////////////////////////
// STATE MACHINE
////////////////////////////////////////////////
// Reads bytes from the serial port until a valid frame with address a is received:
//   - supervision/unnumbered frame [FLAG, a, c, a^c, FLAG]
//   - I frame [FLAG, a, c, a^c, D1..DN, BCC2, FLAG] (only if data != NULL)
// The control field received is saved in *c. For I frames the destuffed data
// field (D1..DN followed by BCC2, not yet checked) is saved in data and its
// size in *dataSize. For supervision frames *dataSize is 0.
// Frames with a wrong header (BCC1) are ignored, as the protocol says.
// If useAlarm is TRUE, it gives up when the alarm is triggered.
// Returns FRAME_OK on success, FRAME_TIMEOUT if the alarm was triggered,
// FRAME_ERROR on serial port error.
static int readFrame(unsigned char a, unsigned char *c, unsigned char *data,
                     int *dataSize, int useAlarm)
{
    State state = START;
    unsigned char byte;
    int size = 0;

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
            if (byte == FLAG)
                state = FLAG_RCV;
            else
            {
                // Any control field is accepted here, the caller checks it
                *c = byte;
                state = C_RCV;
            }
            break;

        case C_RCV:
            if (byte == (a ^ *c))
                state = BCC_OK;
            else if (byte == FLAG)
                state = FLAG_RCV;
            else
                state = START; // header error: frame ignored
            break;

        case BCC_OK:
            if (byte == FLAG)
                state = STOP_ST; // supervision frame (no data field)
            else if (data == NULL)
                state = START; // not expecting I frames
            else
            {
                size = 0;
                state = DATA_RCV;
                // Process this byte as the first one of the data field
                if (byte == ESC)
                    state = ESC_RCV;
                else
                    data[size++] = byte;
            }
            break;

        case DATA_RCV:
            if (byte == FLAG)
                state = STOP_ST; // end of the I frame
            else if (byte == ESC)
                state = ESC_RCV;
            else if (size < MAX_DATA_FIELD)
                data[size++] = byte;
            else
                state = START; // frame too long: discard it
            break;

        case ESC_RCV:
            // Destuffing: 0x7D 0x5E -> 0x7E and 0x7D 0x5D -> 0x7D
            if (byte == FLAG)
                state = FLAG_RCV; // invalid escape sequence: new frame starts
            else if (size < MAX_DATA_FIELD)
            {
                data[size++] = byte ^ 0x20;
                state = DATA_RCV;
            }
            else
                state = START;
            break;

        default:
            break;
        }
    }

    if (dataSize != NULL)
        *dataSize = size;

    return FRAME_OK;
}

// Reads frames until a valid supervision frame [FLAG, a, c, a^c, FLAG]
// with the given control field is received. Other frames are ignored.
// Same return values as readFrame.
static int readSupervisionFrame(unsigned char a, unsigned char c, int useAlarm)
{
    unsigned char cRcv;
    int res;

    do
    {
        res = readFrame(a, &cRcv, NULL, NULL, useAlarm);
    } while (res == FRAME_OK && cRcv != c);

    return res;
}

// Sends the supervision frame [FLAG, a, c, a^c, FLAG].
// Returns the number of bytes written or -1 on error.
static int sendSupervisionFrame(unsigned char a, unsigned char c)
{
    unsigned char frame[5] = {FLAG, a, c, a ^ c, FLAG};
    return writeBytesSerialPort(frame, 5);
}

// Byte stuffing: writes byte in frame[pos], replacing FLAG and ESC
// by the escape sequences 0x7D 0x5E and 0x7D 0x5D.
// Returns the position after the written bytes.
static int stuffByte(unsigned char *frame, int pos, unsigned char byte)
{
    if (byte == FLAG || byte == ESC)
    {
        frame[pos++] = ESC;
        frame[pos++] = byte ^ 0x20;
    }
    else
        frame[pos++] = byte;

    return pos;
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
    connectionParameters = llParameters;
    ns = 0;
    memset(&stats, 0, sizeof(stats));

    if (installAlarm() < 0)
        return -1;

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
    closeSerialPort();
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
    connectionParameters = llParameters;
    nr = 0;
    discReceived = FALSE;
    memset(&stats, 0, sizeof(stats));

    // The receiver also needs the alarm: its DISC is protected by a timer
    if (installAlarm() < 0)
        return -1;

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
    if (buf == NULL || bufSize < 0 || bufSize > MAX_PAYLOAD_SIZE)
        return -1;

    // Build the I frame [FLAG, A, C, BCC1, D1..DN, BCC2, FLAG]
    // A, C and BCC1 never need stuffing (0x03, 0x00/0x80, 0x03/0x83)
    unsigned char frame[MAX_I_FRAME];
    int frameSize = 0;

    frame[frameSize++] = FLAG;
    frame[frameSize++] = A_TX;
    frame[frameSize++] = C_I(ns);
    frame[frameSize++] = A_TX ^ C_I(ns);

    // BCC2 is computed over the original bytes (before stuffing)
    unsigned char bcc2 = 0;
    for (int i = 0; i < bufSize; i++)
    {
        bcc2 ^= buf[i];
        frameSize = stuffByte(frame, frameSize, buf[i]);
    }
    frameSize = stuffByte(frame, frameSize, bcc2);

    frame[frameSize++] = FLAG;

    // Stop & Wait: 1 first transmission + up to nRetransmissions retransmissions
    // after a timeout. A REJ also causes a retransmission, but it does not count
    // for the limit: it proves the receiver is still answering.
    int timeouts = 0;
    int sentThisFrame = FALSE;
    while (timeouts <= connectionParameters.nRetransmissions)
    {
        if (writeBytesSerialPort(frame, frameSize) != frameSize)
        {
            printf("Error writing I frame\n");
            return -1;
        }
        printf("I(%d) sent (%d bytes), timeouts so far: %d\n", ns, frameSize, timeouts);
        stats.iFramesSent++;
        if (sentThisFrame)
            stats.retransmissions++;
        sentThisFrame = TRUE;

        // Enable alarm in t seconds
        alarmEnabled = TRUE;
        alarm(connectionParameters.timeout);

        // Wait for RR(ns+1) or REJ(ns). Any other frame (e.g. an old RR) is ignored.
        unsigned char c;
        int res;
        do
        {
            res = readFrame(A_TX, &c, NULL, NULL, TRUE);
        } while (res == FRAME_OK && c != C_RR(ns ^ 1) && c != C_REJ(ns));

        // Disable pending alarms, if any
        alarm(0);

        if (res == FRAME_ERROR)
        {
            printf("Error reading answer to I(%d)\n", ns);
            return -1;
        }

        if (res == FRAME_OK && c == C_RR(ns ^ 1))
        {
            printf("RR(%d) received\n", ns ^ 1);
            ns ^= 1;
            return bufSize;
        }

        // REJ or timeout: the loop retransmits the I frame
        if (res == FRAME_OK)
        {
            printf("REJ(%d) received\n", ns);
            stats.rejReceived++;
        }
        else
        {
            printf("Timeout, no answer to I(%d)\n", ns);
            timeouts++;
            stats.timeouts++;
        }
    }

    printf("Maximum number of retransmissions reached. Giving up.\n");
    return -1;
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
int llReceive(unsigned char *packet)
{
    unsigned char data[MAX_DATA_FIELD];

    while (TRUE)
    {
        unsigned char c;
        int dataSize;

        // The receiver does not use the alarm: it waits until a frame arrives
        if (readFrame(A_TX, &c, data, &dataSize, FALSE) != FRAME_OK)
        {
            printf("Error reading I frame\n");
            return -1;
        }

        // The UA sent in llOpenRx was lost and the transmitter sent SET again
        if (c == C_SET)
        {
            printf("SET received again, resending UA\n");
            sendSupervisionFrame(A_TX, C_UA);
            continue;
        }

        // The transmitter closed the connection (e.g. it gave up): llCloseRx answers it
        if (c == C_DISC)
        {
            printf("DISC received while waiting for an I frame\n");
            discReceived = TRUE;
            return -1;
        }

        if (c != C_I(0) && c != C_I(1))
            continue; // not an I frame: ignored

        int frameNs = (c == C_I(1));

        // Duplicate (our RR was lost): data discarded, confirmed with RR
        if (frameNs != nr)
        {
            printf("Duplicate I(%d) received, sending RR(%d)\n", frameNs, nr);
            sendSupervisionFrame(A_TX, C_RR(nr));
            stats.duplicates++;
            continue;
        }

        // Check BCC2 (last byte of the data field) against the data received
        unsigned char bcc2 = 0;
        for (int i = 0; i < dataSize - 1; i++)
            bcc2 ^= data[i];

        if (dataSize < 1 || bcc2 != data[dataSize - 1])
        {
            // New frame with an error in the data field: ask for retransmission
            printf("BCC2 error in I(%d), sending REJ(%d)\n", frameNs, nr);
            sendSupervisionFrame(A_TX, C_REJ(nr));
            stats.rejSent++;
            continue;
        }

        // New frame without errors: accepted and passed to the application
        int packetSize = dataSize - 1;
        memcpy(packet, data, packetSize);

        nr ^= 1;
        sendSupervisionFrame(A_TX, C_RR(nr));
        stats.iFramesAccepted++;
        printf("I(%d) received (%d bytes), sending RR(%d)\n", frameNs, packetSize, nr);

        return packetSize;
    }
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
// Sends the command frame (a, c) and waits for the answer (answerA, answerC),
// retransmitting the command after each timeout, up to nRetransmissions times.
// Returns 0 when the answer is received, -1 otherwise.
static int sendCommand(unsigned char a, unsigned char c,
                       unsigned char answerA, unsigned char answerC, const char *name)
{
    alarmCount = 0;
    while (alarmCount <= connectionParameters.nRetransmissions)
    {
        sendSupervisionFrame(a, c);
        printf("%s sent, attempt #%d\n", name, alarmCount + 1);

        // Enable alarm in t seconds
        alarmEnabled = TRUE;
        alarm(connectionParameters.timeout);

        int res = readSupervisionFrame(answerA, answerC, TRUE);

        // Disable pending alarms, if any
        alarm(0);

        if (res == FRAME_OK)
            return 0;
        if (res == FRAME_ERROR)
            return -1;

        printf("Timeout, no answer to %s\n", name);
    }

    printf("Maximum number of retransmissions reached. Giving up.\n");
    return -1;
}

static void printStatistics(int isTx)
{
    printf("\n---- Statistics ----\n");
    if (isTx)
    {
        printf("I frames sent:      %d\n", stats.iFramesSent);
        printf("Retransmissions:    %d\n", stats.retransmissions);
        printf("Timeouts:           %d\n", stats.timeouts);
        printf("REJ received:       %d\n", stats.rejReceived);
    }
    else
    {
        printf("I frames accepted:  %d\n", stats.iFramesAccepted);
        printf("Duplicates:         %d\n", stats.duplicates);
        printf("REJ sent:           %d\n", stats.rejSent);
    }
    printf("--------------------\n\n");
}

// Wait until all bytes have been written and close the serial port
static int closeConnection()
{
    sleep(1);

    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }

    printf("Serial port closed\n");
    return 0;
}

int llCloseTx()
{
    // Tx -> DISC (A = 0x03, command sent by the Transmitter)
    // Rx -> DISC (A = 0x01, command sent by the Receiver)
    int res = sendCommand(A_TX, C_DISC, A_RX, C_DISC, "DISC");

    if (res == 0)
    {
        printf("DISC received\n");

        // Tx -> UA (A = 0x01, answer sent by the Transmitter)
        sendSupervisionFrame(A_RX, C_UA);
        printf("UA sent\n");
    }

    printStatistics(TRUE);

    if (closeConnection() < 0)
        return -1;

    return res;
}

int llCloseRx()
{
    // Wait for the DISC of the transmitter (unless llReceive already got it).
    // Meanwhile, the last I frame may arrive again because our RR was lost:
    // it is a duplicate and must be confirmed with RR.
    unsigned char data[MAX_DATA_FIELD];
    while (!discReceived)
    {
        unsigned char c;
        int dataSize;

        if (readFrame(A_TX, &c, data, &dataSize, FALSE) != FRAME_OK)
        {
            printf("Error reading DISC\n");
            closeConnection();
            return -1;
        }

        if (c == C_DISC)
            discReceived = TRUE;
        else if (c == C_I(0) || c == C_I(1))
        {
            printf("Duplicate I(%d) received, sending RR(%d)\n", c == C_I(1), nr);
            sendSupervisionFrame(A_TX, C_RR(nr));
            stats.duplicates++;
        }
    }

    printf("DISC received\n");

    // Rx -> DISC (A = 0x01), protected by a timer; waits for the UA (A = 0x01)
    int res = sendCommand(A_RX, C_DISC, A_RX, C_UA, "DISC");

    if (res == 0)
        printf("UA received\n");

    printStatistics(FALSE);

    if (closeConnection() < 0)
        return -1;

    return res;
}