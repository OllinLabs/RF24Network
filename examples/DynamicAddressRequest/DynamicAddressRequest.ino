/*
 *  Copyright 2016 OllinLabs, or its affiliates. All Rights Reserved.
 */
#include <SPI.h>
#include <RF24Network.h>
#include <RF24.h>

/**
 * This sketch will attempt to get an address from the network on startup
 * If an address is successfully obtained, the addressSetCallback() method
 * will be called from the RFNetwork layer indicated the assigned address.
 *
 * If for some reason, no address is obtained the beginAddressRequestProcedure
 * will timeout and return false.
 *
 * NOTE: Upload MulticastWithLeds_ListenerNode00 or any other sketch that
 *   initializes with a valid address. Hint, try changing the address from 00
 *   to any other valid node address (like 023, this node will get 0[1-5]23)
 */
const uint8_t CHIP_ENABLE = 9;
const uint8_t CHIP_SELECT = 10;

RF24 radio(CHIP_ENABLE, CHIP_SELECT);
RF24Network network(radio);

void addressSetCallback(uint8_t channel, uint16_t nodeAddress) {
    Serial.println("* Callback reached *");
    Serial.print("Channel: ");
    Serial.print(channel);
    Serial.print(", nodeAddress: ");
    Serial.println(nodeAddress, OCT);
}

void setup(void) {
    Serial.begin(57600);
    Serial.println("Address Request Node");

    SPI.begin();
    radio.begin();
    bool beginOk = network.beginAddressRequestProcedure(addressSetCallback);
    Serial.print("Begin Ok ");
    Serial.println(beginOk);
}

void loop(void) {
    network.update();
    Serial.println("Loop");
    delay(20000);
}
