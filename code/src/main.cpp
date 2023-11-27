/** @file main.cpp
 * @author Lukáš Baštýř (l.bastyr@seznam.cz, 492875)
 * @brief Simple remote based around Attiny1624 and LoRa SX1278 using TinyMesh.
 * @version 0.1
 * @date 27-11-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */

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


#define BTN1_PORT 16
#define BTN2_PORT 32
#define BTN3_PORT 198


/*----(FUNCTION MACROS)----*/
#define IF_X_FALSE(x, msg, cmd) {if (x) {Serial.println(msg); cmd;}}
#define IF_X_TRUE(x, msg, cmd)  {if (x) {Serial.print(msg); Serial.print(x, 2); Serial.print(" ("); Serial.print(x); Serial.println(")"); cmd;}}
//reset the mcu
#define RESET() {Serial.println("reseting"); wdt_enable(WDTO_15MS); while (true);}


TinyMesh tm(TM_TYPE_NODE);
SX127X lora(CS, RST, DIO0);
lora434InterfaceWrapper lora_if(&lora);
packet_t packet;



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
    while(true) {
        digitalWrite(LED_B, HIGH);
        digitalWrite(LED_R, LOW);
        delay(200);
        digitalWrite(LED_B, LOW);
        digitalWrite(LED_R, HIGH);
        delay(200);
    }
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


/** @brief Send packet on our only interface and save it's ID
 * 
 * @return 0 on success, errors that occured during transmission
 */
uint16_t sendPacketOnInterface() {
    uint16_t ret = 0;

    //Serial.println("Sending packet:");
    //printPacket();
    //Serial.println("");

    uint32_t packet_id = tm.createPacketID(packet);
    ret = lora_if.transmitData(packet.raw, TM_HEADER_LENGTH + packet.fields.data_len);
    //IF_X_TRUE(ret, "Failed to transmit data: ", return ret);
    if(ret) return ret;

    //IF_X_TRUE(tm.savePacketID(packet_id, millis()), "Failed to save packet: ", {});
    tm.savePacketID(packet_id, millis());

    return ret;
}


/** @brief Get data from interface and try to build a pakcet
 * 
 * @return 0 on success, lora or tm errors on failure
 */
uint16_t getPacketOnInterface() {
    uint16_t ret = 0;

    //get data from lora
    uint8_t len;
    ret = lora_if.getData(packet.raw, &len);
    //IF_X_TRUE(ret, "Failed to get data: ", return ret);
    if (ret) return ret;

    //check if packet is valid
    ret = tm.checkHeader(packet);
    //IF_X_TRUE(ret, "Incoming packet header bad: ", return ret);

    return ret;
}


/** @brief Blocking send and receive function. Exits once either answer is received or timeout/error occured enough times.
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
        //IF_X_TRUE(ret, "Failed to build packet: ", continue);
        if(ret) continue;

        ret = sendPacketOnInterface();
        //IF_X_TRUE(ret, "Failed to send a packet: ", continue);
        if(ret) continue;

        //start reception on lora
        ret = lora_if.startReception();
        //IF_X_TRUE(ret, "Failed to start reception: ", continue);
        if(ret) continue;

        //wait for response
        auto timer = millis();
        while (millis() - timer < timeout) {
            //got some data
            if (digitalRead(DIO0)) {
                ret = getPacketOnInterface();
                //IF_X_TRUE(ret, "Failed to get valid packet on interface: ", continue);
                if (ret) continue;

                //printPacket();

                //check if incoming packet is an answer or something else (that we don't care about)
                ret = tm.checkPacket(packet);
                //IF_X_TRUE(ret, "Incoming packet is not an answer: ", continue);
                if (ret != TM_ERR_IN_DUPLICATE)
                    tm.savePacket(packet, millis());
                if (ret) continue;
                //Serial.println("Got answer:");
                //printPacket();
                //Serial.println("");

                //packet is an answer to our request
                return ret;
            }

            //clear saved packets in the meantime
            tm.clearSavedPackets(millis());
        }

        //Serial.println("timeout");
    }

    return ret ? ret : 0xFF;
}


void setup() {
    //Run at 5MHz
    //_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PEN_bm);    //enable prescaler
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm);   //set prescaler to 4 -> 5MHz
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSC20M_gc);  //set oscilator to 20MHz

    //configure pins
    pinMode(LED_R, OUTPUT);
    pinMode(LED_B, OUTPUT);
    pinMode(BTN1, INPUT);
    pinMode(BTN2, INPUT);
    pinMode(BTN3, INPUT);

    //SPI init
    SPI.begin();

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
    uint16_t ret = lora.begin(868.0, 0x24, 8, LORA_BANDWIDTH_125kHz, LORA_SPREADING_FACTOR_9, LORA_CODING_RATE_4_7);
    //IF_X_TRUE(ret, "lora begin failed: ", failed());
    if(ret) failed();

    //TinyMesh init
    tm.setSeed(46290);
    tm.addPort(BTN1_PORT);
    tm.addPort(BTN2_PORT);
    tm.addPort(BTN3_PORT);
    tm.setAddress(23);

    //1. register
    digitalWrite(LED_B, HIGH);
    ret = requestAwaitAnswer(3, TM_TIME_TO_STALE, tm.getGatewayAddress(), TM_MSG_REGISTER, 0, nullptr, 0);
    //IF_X_TRUE(ret, "Failed to register: ", failed());
    if (ret) failed();
    digitalWrite(LED_B, LOW);

    //save data from registration (our address is premade)
    tm.clearSavedPackets(millis());
    tm.setGatewayAddress(packet.fields.src_addr);

    //2. tell gateway our ports
    uint8_t buf[3] = {BTN1_PORT, BTN2_PORT, BTN3_PORT};
    digitalWrite(LED_R, HIGH);
    ret = requestAwaitAnswer(3, TM_TIME_TO_STALE, tm.getGatewayAddress(), TM_MSG_PORT_ANOUNCEMENT, 0, buf, 3);
    //IF_X_TRUE(ret, "Failed to send port anouncement: ", failed());
    if(ret) failed();
    digitalWrite(LED_R, LOW);
}


bool pressed = false;
uint16_t ret = 0;
void loop() {

    //our service request routine
    if ((digitalRead(BTN1) || digitalRead(BTN2) || digitalRead(BTN3)) && !pressed) {
        pressed = true;
        digitalWrite(LED_B, HIGH);
        digitalWrite(LED_R, HIGH);

        //send custom empty message on specified button port to address 46
        if (digitalRead(BTN1))
            ret = requestAwaitAnswer(1, TM_TIME_TO_STALE, 46, TM_MSG_CUSTOM, BTN1_PORT, nullptr, 0);
        else if (digitalRead(BTN2))
            ret = requestAwaitAnswer(1, TM_TIME_TO_STALE, 46, TM_MSG_CUSTOM, BTN2_PORT, nullptr, 0);
        else if (digitalRead(BTN3))
            ret = requestAwaitAnswer(1, TM_TIME_TO_STALE, 46, TM_MSG_CUSTOM, BTN3_PORT, nullptr, 0);
        
        //blue on success, red on error
        if (ret || packet.fields.msg_type != TM_MSG_OK)
            digitalWrite(LED_B, LOW);
        else
            digitalWrite(LED_R, LOW);

        delay(500);

        digitalWrite(LED_R, LOW);
        digitalWrite(LED_B, LOW);
    }

    //reset pressed state
    if (!digitalRead(BTN1) && !digitalRead(BTN2) && !digitalRead(BTN3) && pressed)
        pressed = false;

    tm.clearSavedPackets(millis());
}
