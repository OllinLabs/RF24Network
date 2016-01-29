/*
 *  Copyright 2015 Svarmo, or its affiliates. All Rights Reserved.
 */

#include <SPI.h>
#include <RF24Network.h>
#include <RF24.h>

/**
 * Use this as the TX component to test the multicast capabilites.
 * This node will publish randomly a message to either the base
 * (00) and visible only to base or a broadcast message that should
 * be visible to ANY listening node.
 *
 * @author ricardo
 */

const uint8_t CHIP_ENABLE = 9;
const uint8_t CHIP_SELECT = 10;

RF24 radio(CHIP_ENABLE, CHIP_SELECT);
RF24Network network(radio);

const uint16_t baseNode = 00;
const uint16_t multicastAddress = 65535;
const uint16_t nodeAddress = 02;
const uint8_t channel = 90;

unsigned long counterIncrement = 0;
unsigned long lastRxTimestamp;

// Structure of our payload
struct payload_t {
 unsigned long ms;
 unsigned long counter;
};

void setup(void) {
    Serial.begin(57600);
    Serial.println("TXNode");
    Serial.print("Address: ");
    Serial.println(nodeAddress);

    setupLeds();

    SPI.begin();
    radio.begin();
    network.begin(channel, nodeAddress);
}

void loop(void) {
    network.update();

    unsigned long now = millis();
    bool sendToBase = (now % 2 == 0);
    if (now - lastRxTimestamp >= 2000) {
        lastRxTimestamp = now;
        payload_t payload = {
            millis(),
            counterIncrement++
        };

        bool ok = false;
        uint16_t destinationNode = sendToBase ? baseNode : multicastAddress;
        Serial.print("Sending message to: ");
        Serial.print(destinationNode);
        RF24NetworkHeader header(destinationNode);
        ok = network.write(header,&payload,sizeof(payload));
        Serial.println(ok ? " ... ok." : "... failed.");
        blinkLed(ok, destinationNode);
    }
}
