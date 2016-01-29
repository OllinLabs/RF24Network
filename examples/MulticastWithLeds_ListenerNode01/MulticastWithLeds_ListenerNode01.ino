/*
 *  Copyright 2015 OllinLabs, or its affiliates. All Rights Reserved.
 */

#include <SPI.h>
#include <RF24Network.h>
#include <RF24.h>

/**
 * Test the RX capacity of this node when set in receive mode.
 *
 * This class can demonstrate the Multicast capabilities by setting
 * three RFNodes, one to publish p2p message and broadcast message
 * a) Node00 = MulticastListerNode00.ino, will write to serial the messages
 *      that are intended for it AND the multicast messages received
 * b) Node01 = MulticastListerNode01.ino, will write to serial the messages
 *      that are intended for it AND the multicast messages received
 * c) Node02-5 = TxMulticastTest (See documentation for that class)
 *      basically it will transmit to both 00, and multicast.
 *
 * @author ricardo
 */

// Led Pin setup
const uint8_t GREEN_LED = 6;
const uint8_t BLUE_LED = 2;
const uint8_t RED_LED = 5;

const uint8_t CHIP_ENABLE = 9;
const uint8_t CHIP_SELECT = 10;

RF24 radio(CHIP_ENABLE, CHIP_SELECT);
RF24Network network(radio);

const uint16_t nodeAddress = 01;
const uint8_t channel = 90;

// Structure of our payload
struct payload_t {
    unsigned long ms;
    unsigned long counter;
};

void setupLeds(void) {
    pinMode(GREEN_LED, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);
    pinMode(RED_LED, OUTPUT);

    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(BLUE_LED, HIGH);
    digitalWrite(RED_LED, HIGH);

    delay(50);

    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(RED_LED, LOW);
}

void blinkLed(uint16_t from_node) {
    switch (from_node) {
        case 02:
            digitalWrite(GREEN_LED, HIGH);
            delay(500);
            digitalWrite(GREEN_LED, LOW);
            break;
        case 65535:
            digitalWrite(BLUE_LED, HIGH);
            delay(500);
            digitalWrite(BLUE_LED, LOW);
            break;
    }
}

void setup(void) {
    Serial.begin(57600);
    Serial.print("ListenerNode (");
    Serial.print(nodeAddress);
    Serial.println(")");

    setupLeds();

    SPI.begin();
    radio.begin();
    network.begin(channel, nodeAddress);
}

void loop(void) {
    network.update();

    while (network.available()) {
        RF24NetworkHeader header;
        payload_t payload;
        network.read(header,&payload, sizeof(payload));
        Serial.print("Received packet # ");
        Serial.print(payload.counter);
        Serial.print(" from ");
        Serial.print(header.from_node);
        Serial.print(" sentTo ");
        Serial.print(header.to_node);
        Serial.print(" at ");
        Serial.println(payload.ms);
    }
}
