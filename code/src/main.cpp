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
#include "libs/interfaces/sx127xInterfaceWrapper.hpp"
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


TinyMesh tm(TM_NODE_TYPE_LP_NODE);
SX127X lora(CS, RST, DIO0);
sx127xInterfaceWrapper lora_if(&lora);
packet_t packet;


/*----(SX127X CALLBACKS)----*/
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


/*----(HELPERS & OTHER)----*/
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
    Serial.println(packet.fields.flags.fields.node_type);
    Serial.print("Message id:          ");
    Serial.println(tm.getMessageId(&packet));
    Serial.print("Source address:      ");
    Serial.println(packet.fields.source);
    Serial.print("Destination address: ");
    Serial.println(packet.fields.destination);
    Serial.print("Message type:        ");
    Serial.println(packet.fields.flags.fields.message_type);
    Serial.print("Data length:         ");
    Serial.println(packet.fields.data_length);
    Serial.print("Data: ");
    for (int i = 0; i < packet.fields.data_length; i++) {
        Serial.print(packet.fields.data[i]);
        Serial.print(" ");
    }
    Serial.println();
}


/** @brief Handler for incoming packets (not used)
 * 
 */
void handleIncomingAnswer() {
    uint8_t msg_type = packet.fields.flags.fields.message_type;
    if (msg_type == TM_MSG_OK) {
        Serial.println("OK Answer");
        return;
    }

    if (msg_type == TM_MSG_ERR) {
        Serial.println("ERR Answer");
        return;
    }

    if (msg_type == TM_MSG_CUSTOM) {
        Serial.println("Custom Answer");
        return;
    }
}

/** @brief Send packet on our only interface and save it's ID
 * 
 * @return 0 on success, errors that occured during transmission
 */
uint8_t sendPacketOnInterface() {
    uint8_t ret = 0;

    uint32_t packet_id = tm.createPacketID(&packet);
    ret = lora_if.transmitData(packet.raw, TM_HEADER_LENGTH + packet.fields.data_length);
    if (ret) return ret;
    tm.savePacketID(packet_id);

    return ret;
}

/** @brief Get data from interface and try to build a pakcet
 * 
 * @return 0 on success, lora or tm errors on failure
 */
uint8_t getPacketOnInterface() {
    uint8_t ret = 0;
    uint8_t len;
    ret = lora_if.getData(packet.raw, &len);
    if (ret) return ret;

    //check if packet is valid
    ret = tm.checkHeader(&packet);
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
uint8_t requestAwait(uint8_t retries, uint32_t timeout, uint8_t destination, uint8_t message_type, uint8_t *buffer, uint8_t length) {
    uint8_t ret = 0;

    for (int i = 0; i < retries; i++) {
        ret = tm.buildPacket(&packet, destination, tm.lcg(), message_type, buffer, length);
        if(ret) continue;

        ret = sendPacketOnInterface();
        if(ret) continue;

        //start reception on lora
        ret = lora_if.startReception();
        if(ret) continue;

        ret = 0xFF;

        //wait for response
        auto timer = millis();
        while (millis() - timer < timeout) {
            //got some data
            if (digitalRead(DIO0)) {
                ret = getPacketOnInterface();
                if (ret) continue;

                ret = tm.checkPacket(&packet);
                if (!(ret & TM_PACKET_RESPONSE)) continue;
                return 0;
            }
        }

        //Serial.println("timeout");
    }

    return ret;
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
    lora.registerMicros(micros);
    lora.registerDelay(delay);
    lora.registerPinMode(pinMode, INPUT, OUTPUT);
    lora.registerDigitalWrite(digitalWrite);
    lora.registerDigitalRead(digitalRead);
    lora.registerSPIBeginTransfer(SPIBeginTransfer);
    lora.registerSPIEndTransfer(SPIEndTransfer);
    lora.registerSPITransfer(SPITransfer);
    lora.reset();
    uint16_t ret = lora.begin(868.0, 0x24, 8, LORA_BANDWIDTH_125kHz, LORA_SPREADING_FACTOR_9, LORA_CODING_RATE_4_7);
    //IF_X_TRUE(ret, "lora begin failed: ", failed());
    if(ret) failed();

    //TinyMesh init
    tm.setSeed(46290);
    tm.setAddress(23);
    tm.registerMillis(millis);

    //1. register
    digitalWrite(LED_B, HIGH);
    ret = requestAwait(3, TM_CLEAR_TIME, tm.getGatewayAddress(), TM_MSG_REGISTER, nullptr, 0);
    if (ret) failed();
    digitalWrite(LED_B, LOW);

    //save data from registration (our address is premade)
    tm.clearSentQueue();
    tm.setGatewayAddress(packet.fields.source);

    digitalWrite(LED_R, LOW);
}


bool pressed = false;
uint16_t ret = 0;
void loop() {

    //wake up
    //our service request routine
    if ((digitalRead(BTN1) || digitalRead(BTN2) || digitalRead(BTN3)) && !pressed) {
        pressed = true;
        digitalWrite(LED_B, HIGH);
        digitalWrite(LED_R, HIGH);

        //send custom empty message on specified button port to address 46
        if (digitalRead(BTN1)) {
            uint8_t data = 'T';
            ret = requestAwait(1, TM_CLEAR_TIME, 46, TM_MSG_CUSTOM, &data, 1);
        }
        else if (digitalRead(BTN2))
            ret = requestAwait(1, TM_CLEAR_TIME, 46, TM_MSG_CUSTOM, nullptr, 0);
        else if (digitalRead(BTN3)) {
            uint8_t data[] = {'N', 'D', 'B', 3, 7, 'T', 21,35, 'H', 36, 83};
            ret = requestAwait(1, TM_CLEAR_TIME, tm.getGatewayAddress(), TM_MSG_CUSTOM, data, 11);
        }

        //blue on success, red on error
        if (ret || packet.fields.flags.fields.message_type != TM_MSG_OK)
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

    tm.clearSentQueue();

    //go back to sleep
}
