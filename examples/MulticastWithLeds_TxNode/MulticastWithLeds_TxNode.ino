/*
 *  Copyright 2015 OllinLabs, or its affiliates. All Rights Reserved.
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

// Led Pin setup
const uint8_t GREEN_LED = 6;
const uint8_t BLUE_LED = 2;
const uint8_t RED_LED = 5;

const uint8_t CHIP_ENABLE = 9;
const uint8_t CHIP_SELECT = 10;

RF24 radio(CHIP_ENABLE, CHIP_SELECT);
RF24Network network(radio);

const uint16_t baseNode = 00;
const uint16_t nodeAddress = 02;
const uint8_t channel = 90;

unsigned long counterIncrement = 0;
unsigned long lastRxTimestamp;

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

void blinkLed(bool ok, bool multicast) {
    if (!ok) {
        digitalWrite(RED_LED, HIGH);
    }
    if (multicast) {
        digitalWrite(BLUE_LED, HIGH);
        delay(500);
        digitalWrite(BLUE_LED, LOW);
    } else {
        digitalWrite(GREEN_LED, HIGH);
        delay(500);
        digitalWrite(GREEN_LED, LOW);
    }
    digitalWrite(RED_LED, LOW);
}

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
    bool multicast = (now % 2 == 0);
    if (now - lastRxTimestamp >= 2000) {
        lastRxTimestamp = now;
        payload_t payload = {
            millis(),
            counterIncrement++
        };

        bool ok = false;
        Serial.print("Sending message to: ");
        if (!multicast) {
            Serial.print(baseNode);
            RF24NetworkHeader header(baseNode);
            ok = network.write(header, &payload, sizeof(payload));

        } else {
            Serial.print("multicast");
            RF24NetworkHeader header(baseNode);
            ok = network.writeMulticast(header, &payload, sizeof(payload));
        }

        Serial.println(ok ? " ... ok." : "... failed.");
        blinkLed(ok, multicast);
    }
}
