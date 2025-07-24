#include <ModbusMaster.h>
#include <SoftwareSerial.h>
#include <mcp_can.h>
#include <SPI.h>

// Define pins for RS485 communication
#define RX_PIN 2
#define TX_PIN 3
#define MAX485_DE 8
#define MAX485_RE_NEG 7

// Define CAN bus CS pin
#define CAN_CS_PIN 10

// Create SoftwareSerial object for RS485
SoftwareSerial mySerial(RX_PIN, TX_PIN);

// Create ModbusMaster object
ModbusMaster node;

// Create MCP_CAN object
MCP_CAN CAN(CAN_CS_PIN);


void preTransmission()
{

    digitalWrite(MAX485_RE_NEG, HIGH); // Enable RS485 Transmit mode
    digitalWrite(MAX485_DE, HIGH);
}

void postTransmission()
{
    digitalWrite(MAX485_RE_NEG, LOW); // Enable RS485 Receive mode
    digitalWrite(MAX485_DE, LOW);
}

void setup()
{

    pinMode(MAX485_RE_NEG, OUTPUT);
    pinMode(MAX485_DE, OUTPUT);

    digitalWrite(MAX485_RE_NEG, LOW);
    digitalWrite(MAX485_DE, LOW);
  
    Serial.begin(9600);
    mySerial.begin(9600); // Initialize SoftwareSerial for RS485

    // Initialize Modbus communication
    node.begin(1, mySerial); // Set Modbus ID to 1
    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);

    // Initialize CAN bus at 500kbps
    if (CAN.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK) {
        Serial.println("CAN bus initialized successfully");
    } else {
        Serial.println("CAN bus initialization failed");
        while (1); // Stop if CAN bus initialization fails
    }

    CAN.setMode(MCP_NORMAL); // Set CAN bus mode to normal operation

}

void sendCANMessage(float value, uint16_t id) {
    byte data[4];
    memcpy(data, &value, sizeof(float)); // Copy float value into the data array
    CAN.sendMsgBuf(id, 0, 4, data); // Send the CAN message with 4 bytes of data
}

float convertToFloat(uint16_t high, uint16_t low) {
    union {
        byte b[4];
        float fval;
    } data;

    data.b[0] = low & 0xFF;
    data.b[1] = (low >> 8) & 0xFF;
    data.b[2] = high & 0xFF;
    data.b[3] = (high >> 8) & 0xFF;

    return data.fval;
}

float convertCANDataToFloat(byte data[4]) {
    float value;
    memcpy(&value, data, sizeof(float)); // Copy 4 bytes from data array into float
    return value;
}

void loop()
{

    uint8_t result;
  
    // Read 16 registers starting from address 0x7530 (30000 in decimal)
    result = node.readInputRegisters(0x7530, 16);
  
    if (result == node.ku8MBSuccess)
    {
        // Read and send Velocity
        float velocity = convertToFloat(node.getResponseBuffer(0x00), node.getResponseBuffer(0x01));

        Serial.print("Velocity: ");
        Serial.println(velocity, 7);
        sendCANMessage(velocity, 0x100); // Send velocity over CAN with ID 0x100

        // Read and send Volume Flow
        float volumeFlow = convertToFloat(node.getResponseBuffer(0x02), node.getResponseBuffer(0x03));
        Serial.print("Volume Flow: ");
        Serial.println(volumeFlow, 7);
        sendCANMessage(volumeFlow, 0x101); // Send volume flow over CAN with ID 0x101

        // Read and send Mass Flow
        float massFlow = convertToFloat(node.getResponseBuffer(0x04), node.getResponseBuffer(0x05));
        Serial.print("Mass Flow: ");
        Serial.println(massFlow, 7);
        sendCANMessage(massFlow, 0x102); // Send mass flow over CAN with ID 0x102

        // Read and send Temperature
        float temperature = convertToFloat(node.getResponseBuffer(0x06), node.getResponseBuffer(0x07));
        Serial.print("Temperature: ");
        Serial.println(temperature, 7);
        sendCANMessage(temperature, 0x103); // Send temperature over CAN with ID 0x103

        // Read and send Density
        float density = convertToFloat(node.getResponseBuffer(0x08), node.getResponseBuffer(0x09));
        Serial.print("Density: ");
        Serial.println(density, 7);
        sendCANMessage(density, 0x104); // Send density over CAN with ID 0x104

        // Read and send Totaliser
        // Totaliser is at address 0x7D64 (starting register 0x7D64)
        result = node.readInputRegisters(0x7D64, 4); // Read 4 registers for Totaliser
        if (result == node.ku8MBSuccess)
        {
            float totaliser = convertToFloat(node.getResponseBuffer(0x00), node.getResponseBuffer(0x01)); // Adjusted indices for totaliser
            Serial.print("Totaliser: ");
            Serial.println(totaliser, 7);
            sendCANMessage(totaliser, 0x105); // Send totaliser over CAN with ID 0x105
        }
        else
        {
            Serial.print("Totaliser Modbus Error: ");
            Serial.println(result);
        }
    } 
    else 
    {
        Serial.print("Modbus Error: ");
        Serial.println(result);
    }

    // CAN bus'tan veri al ve float olarak dönüştür
    uint8_t len = 0;
    uint8_t buf[8];
    unsigned long id = 0;

    if (CAN_MSGAVAIL == CAN.checkReceive()) {
        if (CAN.readMsgBuf(&id, &len, buf) == CAN_OK) {
            if (len == 4) { // Eğer veri uzunluğu 4 byte ise (float veri)
                float receivedValue = convertCANDataToFloat(buf);
                Serial.print("Received Float Value: ");
                Serial.println(receivedValue, 7);
            } else {
                Serial.println("Received data length is not 4 bytes.");
            }
        }
    }

    Serial.println("\n ------------  \n ");
    delay(1000); // Delay before the next loop iteration
}
