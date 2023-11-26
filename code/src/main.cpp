#include <Arduino.h>
#include <SPI.h>
#include <avr/wdt.h>
#include "libs/interfaces/lora434InterfaceWrapper.hpp"
#include "libs/sx127x.hpp"
#include "libs/tinyMesh.hpp"


#define MOSI PIN_PA1
#define MISO PIN_PA2
#define SCK  PIN_PA3
#define CS   PIN_PA4
#define RST  PIN_PA7
#define DIO0 PIN_PB1

#define STRAP PIN_PB0

#define LED_R PIN_PB2
#define LED_B PIN_PB3

#define BTN1 PIN_PA6
#define BTN2 PIN_PA5
#define BTN3 STRAP


#define S_PORT 16 //our single service port

/*----(FUNCTION MACROS)----*/
#define IF_X_FALSE(x, msg, cmd) {if (x) {Serial.println(msg); cmd;}}
#define IF_X_TRUE(x, msg, cmd)  {if (x) {Serial.print(msg); Serial.print(x, 2); Serial.print(" ("); Serial.print(x); Serial.println(")"); cmd;}}
//reset the mcu
#define RESET() {Serial.println("reseting"); wdt_enable(WDTO_15MS); while (true);}

TinyMesh tm(TM_TYPE_NODE);
SX127X lora(CS, RST, DIO0);
lora434InterfaceWrapper lora_if(&lora);
packet_t packet;


uint16_t pattern_intr[2][2] = {{4900, 100}, {500, 100}};
//uint8_t pattern = 0;
uint8_t interval = 0;


//SX127X callbacks
uint8_t pinRead(uint8_t pin) {
    return digitalRead(pin);
}
void __delay(uint32_t ms) {
    delay(ms);
}
uint32_t __micros() {
    return micros();
}
void SPIBeginTransfer() {
    SPI.beginTransaction({14000000, MSBFIRST, SPI_MODE0});
}
void SPIEndTransfer() {
    SPI.endTransaction();
}
void SPITransfer(uint8_t addr, uint8_t *buffer, size_t length) {
    SPI.transfer(addr);
    SPI.transfer(buffer, length);
}


void failed() {
    while(true) {}
}

void printPacket() {
    Serial.print("Version:             ");
    Serial.println(packet.fields.version);
    Serial.print("Device type:         ");
    Serial.println(packet.fields.node_type);
    Serial.print("Message id:          ");
    Serial.println(tm.getMessageId(packet));
    Serial.print("Source address:      ");
    Serial.println(packet.fields.src_addr);
    Serial.print("Destination address: ");
    Serial.println(packet.fields.dst_addr);
    Serial.print("Port:                ");
    Serial.println(packet.fields.port);
    Serial.print("Message type:        ");
    Serial.println(packet.fields.msg_type);
    Serial.print("Data length:         ");
    Serial.println(packet.fields.data_len);
    Serial.print("Data: ");
    for (int i = 0; i < packet.fields.data_len; i++) {
        Serial.print(packet.fields.data[i]);
        Serial.print(" ");
    }
    Serial.println();
}


void handleIncomingAnswer() {
    if (packet.fields.msg_type == TM_MSG_OK) {
        Serial.println("OK Answer:");
        printPacket();
        return;
    }

    if (packet.fields.msg_type == TM_MSG_ERR) {
        Serial.println("ERR Answer:");
        printPacket();
        return;
    }

    if (packet.fields.msg_type == TM_MSG_CUSTOM) {
        Serial.println("Custom Answer:");
        printPacket();
        return;
    }
}

uint16_t sendPacketOnInterface() {
    uint16_t ret = 0;

    Serial.println("Sending packet:");
    printPacket();
    Serial.println("");

    uint32_t packet_id = tm.createPacketID(packet);
    ret = lora_if.transmitData(packet.raw, TM_HEADER_LENGTH + packet.fields.data_len);
    IF_X_TRUE(ret, "Failed to transmit data: ", return ret);

    IF_X_TRUE(tm.savePacketID(packet_id, millis()), "Failed to save packet: ", {});
    
    return ret;
}

void handleIncomingRequest() {
    uint16_t ret = 0;
    if (packet.fields.msg_type == TM_MSG_CUSTOM) {
        IF_X_TRUE(packet.fields.port != S_PORT, "Unknown service: ", return);

        //TODO handle our service
        Serial.println("Our service not implemented");

        uint8_t buf = TM_SERVICE_NOT_IMPLEMENTED;
        ret = tm.buildPacket(&packet, packet.fields.src_addr, TM_MSG_ERR, tm.getMessageId(packet) + 1, 0, &buf, 1);
        IF_X_TRUE(ret, "Failed to build packet: ", return);
        
        sendPacketOnInterface();
        return;
    }

    if (packet.fields.msg_type == TM_MSG_PING) {
        ret = tm.buildPacket(&packet, packet.fields.src_addr, TM_MSG_OK);
        IF_X_TRUE(ret, "Failed to build packet: ", return);

        sendPacketOnInterface();
        return;
    }

    if (packet.fields.msg_type == TM_MSG_RESET) {
        ret = tm.buildPacket(&packet, packet.fields.src_addr, TM_MSG_OK);
        IF_X_TRUE(ret, "Failed to build packet: ", RESET());
        lora_if.transmitData(packet.raw, TM_HEADER_LENGTH);
        RESET();
    }

    Serial.print("Unhandled request type:");
    Serial.println(packet.fields.msg_type);
    uint8_t buf = TM_SERVICE_NOT_IMPLEMENTED;
    ret = tm.buildPacket(&packet, packet.fields.src_addr, TM_MSG_ERR, 0, &buf, 1);
    IF_X_TRUE(ret, "Failed to build packet: ", return);

    sendPacketOnInterface();
}

//TODO save interfaces or forward = broadcast
void forwardPacket() {
    Serial.println("Forwarding packet");
    //uint16_t ret = tm.buildPacket(&packet, packet.fields.dst_addr, packet.fields.msg_type, packet.fields.port, packet.fields.data, packet.fields.data_len);
    //IF_X_TRUE(ret, "Failed to build packet: ", return);
    sendPacketOnInterface();
}


uint16_t getPacketOnInterface() {
    //get data from lora
    uint8_t len;
    uint16_t ret = 0;
    ret = lora_if.getData(packet.raw, &len);
    IF_X_TRUE(ret, "Failed to get data: ", return ret);
    
    //check if packet is valid
    ret = tm.checkHeader(packet);
    IF_X_TRUE(ret, "Incoming packet header bad: ", return ret);
    return ret;
}


/** @brief Synchronouse send and receive function. Exits once either answer is received or timeout occured enough times.
 * 
 * @param tries How many times to try to send and wait for answer
 * @param timeout Timeout (ms) when waiting for answer
 * @param destination Destionation to which to send the packet
 * @param message_type Message type
 * @param port Service port
 * @param buffer Buffer containing data to be sent
 * @param length Length of data
 * @return 
 */
uint16_t requestAwaitAnswer(uint8_t tries, uint32_t timeout, uint8_t destination, uint8_t message_type, uint8_t port, uint8_t *buffer, uint8_t length) {
    uint16_t ret = 0;

    for (int i = 0; i < tries; i++) {
        ret = tm.buildPacket(&packet, destination, message_type, port, buffer, length);
        IF_X_TRUE(ret, "Failed to build packet: ", continue);

        ret = sendPacketOnInterface();
        IF_X_TRUE(ret, "Failed to send a packet: ", continue);

        //start reception on lora
        ret = lora_if.startReception();
        IF_X_TRUE(ret, "Failed to start reception: ", continue);


        //wait for response
        auto timer = millis();
        while (millis() - timer < timeout) {
            //got some data
            if (digitalRead(DIO0)) {
                ret = getPacketOnInterface();
                IF_X_TRUE(ret, "Failed to get valid packet on interface: ", continue);

                ret = tm.checkPacket(packet);
                IF_X_TRUE(ret != TM_IN_ANSWER, "Incoming packet is not an answer: ", continue);
                Serial.println("Got answer:");
                printPacket();
                Serial.println("");
                return ret;
            }

            //clear saved packets in the meantime
            tm.clearSavedPackets(millis());
        }

        Serial.println("timeout");
    }

    return ret ? ret : 0xFF;
}


void setup() {
    //Run at 5MHz
    //_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PEN_bm);    //enable prescaler
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm);   //set prescaler to 4 -> 5MHz
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSC20M_gc);  //set oscilator to 20MHz

    //Serial.begin(9600);
    Serial.println("Start");
    delay(1000);
    SPI.begin();

    pinMode(LED_R, OUTPUT);
    pinMode(LED_B, OUTPUT);

    pinMode(BTN1, INPUT);
    pinMode(BTN2, INPUT);
    pinMode(BTN3, INPUT);

    //LORA init
    lora.registerMicros(__micros);
    lora.registerDelay(__delay);
    lora.registerPinMode(pinMode, INPUT, OUTPUT);
    lora.registerPinWrite(digitalWrite);
    lora.registerPinRead(pinRead);
    lora.registerSPIBeginTransfer(SPIBeginTransfer);
    lora.registerSPIEndTransfer(SPIEndTransfer);
    lora.registerSPITransfer(SPITransfer);
    lora.reset();
    Serial.println("after lora");
    uint16_t ret = lora.begin(868.0, 0x24, 8, LORA_BANDWIDTH_125kHz, LORA_SPREADING_FACTOR_9, LORA_CODING_RATE_4_7);
    IF_X_TRUE(ret, "lora begin failed: ", failed());

    //TM init
    tm.setSeed(46290);
    tm.setPort(S_PORT, TM_PORT_OUT | TM_PORT_DATA_NONE);
    tm.setAddress(23);

    //1. register
    ret = requestAwaitAnswer(3, TM_TIME_TO_STALE, tm.getGatewayAddress(), TM_MSG_REGISTER, 0, nullptr, 0);
    IF_X_TRUE(ret, "Failed to register: ", failed());
    Serial.println("Registered successfully");

    //save data from registration
    tm.clearSavedPackets(millis());
    //tm.setAddress(packet.fields.data[0]);
    tm.setGatewayAddress(packet.fields.src_addr);

    //2. tell gateway our ports
    uint8_t buf[2] = {S_PORT, TM_PORT_OUT | TM_PORT_DATA_NONE};
    ret = requestAwaitAnswer(3, TM_TIME_TO_STALE, tm.getGatewayAddress(), TM_MSG_PORT_ANOUNCEMENT, 0, buf, 2);
    IF_X_TRUE(ret, "Failed to send port anouncement: ", failed());
}

uint16_t getPacketOnInterface() {

}

bool pressed = false;
uint16_t ret = 0;
void loop() {
    if (digitalRead(BTN1) && !pressed) {
        pressed = true;
        digitalWrite(LED_B, HIGH);
        digitalWrite(LED_R, HIGH);

        ret = requestAwaitAnswer(1, TM_TIME_TO_STALE, 46, TM_MSG_CUSTOM, S_PORT, nullptr, 0);
        if (ret)
            digitalWrite(LED_B, LOW);
        else
            digitalWrite(LED_R, LOW);
        delay(500);
        digitalWrite(LED_R, LOW);
        digitalWrite(LED_B, LOW);
    }
    if (!digitalRead(BTN1) && pressed)
        pressed = false;

    if (digitalRead(DIO0)) {
        //TODO handle incoming data
        //get packet
        //handle incoming packet
            //handle answer
            //handle request
            //handle forward
    }

    tm.clearSavedPackets(millis());
}
