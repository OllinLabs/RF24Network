/*
 *  Copyright 2015 OllinLabs, or its affiliates. All Rights Reserved.
 */

#include <SPI.h>
#include <RF24Network.h>
#include <RF24.h>

/**
 * Test the RX capacity of this node when set in receive mode.
 *
 * This node will start as base node (00) and print to the console when
 * a message is received along with the sender.
 *
 * @author ricardo
 */

const uint8_t CHIP_ENABLE = 9;
const uint8_t CHIP_SELECT = 10;

RF24 radio(CHIP_ENABLE, CHIP_SELECT);
RF24Network network(radio);

const uint16_t nodeAddress = 00;
const uint8_t channel = 90;

// Structure of our payload
struct payload_t {
    unsigned long ms;
    unsigned long counter;
};

void setup(void) {
    Serial.begin(57600);
    Serial.print("ListenerNode (");
    Serial.print(nodeAddress);
    Serial.println(")");

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
        Serial.print(header.from_node, OCT);
        Serial.print(" sentTo ");
        Serial.print(header.to_node, OCT);
        Serial.print(" at ");
        Serial.println(payload.ms);
    }
}
