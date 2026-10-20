// RCOM 2026/2027
//
// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <string.h>

// Control field of the packets
#define C_START 1
#define C_DATA  2
#define C_END   3

// Parameter types (T) of the control packets
#define T_FILE_SIZE 0
#define T_FILE_NAME 1

// Data packet: [C, L2, L1, P1..PK] -> 3 bytes of header
#define DATA_HEADER_SIZE 3
#define MAX_FRAGMENT_SIZE (MAX_PAYLOAD_SIZE - DATA_HEADER_SIZE)

////////////////////////////////////////////////
// PACKETS
////////////////////////////////////////////////
// Builds a control packet [C, T1, L1, V1, T2, L2, V2]:
//   - T1 = file size (4 bytes, most significant byte first)
//   - T2 = file name
// Returns the packet size, or -1 if the file name does not fit.
static int buildControlPacket(unsigned char *packet, unsigned char c,
                              long fileSize, const char *fileName)
{
    int nameLength = strlen(fileName);
    if (nameLength > 255 || 1 + 2 + 4 + 2 + nameLength > MAX_PAYLOAD_SIZE)
        return -1;

    int size = 0;
    packet[size++] = c;

    packet[size++] = T_FILE_SIZE;
    packet[size++] = 4;
    packet[size++] = (fileSize >> 24) & 0xFF;
    packet[size++] = (fileSize >> 16) & 0xFF;
    packet[size++] = (fileSize >> 8) & 0xFF;
    packet[size++] = fileSize & 0xFF;

    packet[size++] = T_FILE_NAME;
    packet[size++] = nameLength;
    memcpy(packet + size, fileName, nameLength);
    size += nameLength;

    return size;
}

// Reads the file size from a control packet (START or END).
// Returns the file size, or -1 if the packet is malformed.
static long parseControlPacket(const unsigned char *packet, int packetSize)
{
    long fileSize = -1;

    // Go through the TLV parameters after the C field
    int i = 1;
    while (i + 2 <= packetSize)
    {
        unsigned char t = packet[i];
        unsigned char l = packet[i + 1];
        const unsigned char *v = packet + i + 2;

        if (i + 2 + l > packetSize)
            return -1; // parameter goes beyond the packet

        if (t == T_FILE_SIZE)
        {
            fileSize = 0;
            for (int j = 0; j < l; j++)
                fileSize = (fileSize << 8) | v[j];
        }
        else if (t == T_FILE_NAME)
            printf("File name: %.*s\n", l, v);

        i += 2 + l;
    }

    return fileSize;
}

////////////////////////////////////////////////
// TRANSMITTER
////////////////////////////////////////////////
static int sendFile(const char *filename)
{
    FILE *file = fopen(filename, "rb");
    if (file == NULL)
    {
        perror(filename);
        return -1;
    }

    // File size
    fseek(file, 0, SEEK_END);
    long fileSize = ftell(file);
    fseek(file, 0, SEEK_SET);

    unsigned char packet[MAX_PAYLOAD_SIZE];

    // START packet
    int packetSize = buildControlPacket(packet, C_START, fileSize, filename);
    if (packetSize < 0 || llSend(packet, packetSize) < 0)
    {
        printf("Error sending START packet\n");
        fclose(file);
        return -1;
    }
    printf("START sent (file size = %ld bytes)\n", fileSize);

    // DATA packets, each one with a fragment of the file
    long bytesSent = 0;
    int nRead;
    while ((nRead = fread(packet + DATA_HEADER_SIZE, 1, MAX_FRAGMENT_SIZE, file)) > 0)
    {
        packet[0] = C_DATA;
        packet[1] = (nRead >> 8) & 0xFF; // L2
        packet[2] = nRead & 0xFF;        // L1

        if (llSend(packet, DATA_HEADER_SIZE + nRead) < 0)
        {
            printf("Error sending DATA packet\n");
            fclose(file);
            return -1;
        }

        bytesSent += nRead;
        printf("DATA sent: %d bytes (%ld / %ld)\n", nRead, bytesSent, fileSize);
    }

    fclose(file);

    // END packet: repeats the information of the START packet
    packetSize = buildControlPacket(packet, C_END, fileSize, filename);
    if (llSend(packet, packetSize) < 0)
    {
        printf("Error sending END packet\n");
        return -1;
    }
    printf("END sent\n");

    return 0;
}

////////////////////////////////////////////////
// RECEIVER
////////////////////////////////////////////////
static int receiveFile(const char *filename)
{
    unsigned char packet[MAX_PAYLOAD_SIZE];

    // START packet
    int packetSize = llReceive(packet);
    if (packetSize < 1 || packet[0] != C_START)
    {
        printf("Error: expected START packet\n");
        return -1;
    }

    long fileSize = parseControlPacket(packet, packetSize);
    if (fileSize < 0)
    {
        printf("Error: malformed START packet\n");
        return -1;
    }
    printf("START received (file size = %ld bytes)\n", fileSize);

    FILE *file = fopen(filename, "wb");
    if (file == NULL)
    {
        perror(filename);
        return -1;
    }

    // DATA packets until the END packet
    long bytesReceived = 0;
    while (TRUE)
    {
        packetSize = llReceive(packet);
        if (packetSize < 1)
        {
            printf("Error receiving packet\n");
            fclose(file);
            return -1;
        }

        if (packet[0] == C_END)
            break;

        if (packet[0] != C_DATA || packetSize < DATA_HEADER_SIZE)
        {
            printf("Unknown packet ignored\n");
            continue;
        }

        // K = 256 * L2 + L1 must match the size of the data field received
        int k = 256 * packet[1] + packet[2];
        if (k != packetSize - DATA_HEADER_SIZE)
        {
            printf("Error: DATA packet with K = %d but %d data bytes\n",
                   k, packetSize - DATA_HEADER_SIZE);
            fclose(file);
            return -1;
        }

        fwrite(packet + DATA_HEADER_SIZE, 1, k, file);
        bytesReceived += k;
        printf("DATA received: %d bytes (%ld / %ld)\n", k, bytesReceived, fileSize);
    }

    fclose(file);
    printf("END received\n");

    // The END packet must repeat the information of the START packet
    if (parseControlPacket(packet, packetSize) != fileSize)
        printf("Warning: file size in END packet is different from START\n");

    if (bytesReceived != fileSize)
    {
        printf("Error: received %ld bytes, expected %ld\n", bytesReceived, fileSize);
        return -1;
    }

    return 0;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer llParameters = {
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout,
    };
    strcpy(llParameters.serialPort, serialPort);

    if (strcmp(role, "tx") == 0)
    {
        if (llOpenTx(llParameters) < 0)
        {
            printf("Error opening connection\n");
            return;
        }

        if (sendFile(filename) == 0)
            printf("File %s sent\n", filename);

        llCloseTx();
    }
    else if (strcmp(role, "rx") == 0)
    {
        if (llOpenRx(llParameters) < 0)
        {
            printf("Error opening connection\n");
            return;
        }

        if (receiveFile(filename) == 0)
            printf("File %s received\n", filename);

        llCloseRx();
    }
    else
    {
        printf("Invalid role: %s. Must be 'tx' or 'rx'.\n", role);
        return;
    }
}
