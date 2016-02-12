/*
 * Copyright (C) 2011 James Coliz, Jr. <maniacbug@ymail.com>
 * 2015 Ricardo Rodriguez.  <ricardo@ollinlabs.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include "RF24Network_config.h"
#include "RF24.h"
#include "RF24Network.h"

uint16_t RF24NetworkHeader::next_id = 1;

uint64_t pipe_address(uint16_t node, uint8_t pipe);
bool is_valid_address(uint16_t node);
uint32_t precisePow(uint32_t base, uint8_t power);
uint32_t octalToDecimal(uint32_t n);

FrameQueue::FrameQueue() {
    tailPointer = 0;
    headPointer = 0;
    remainingCapacity = queueCapacity;
}

void FrameQueue::advanceTail() {
    tailPointer++;
    if (tailPointer >= queueCapacity) {
        tailPointer = 0;
    }
}

void FrameQueue::advanceHead() {
    headPointer++;
    if (headPointer >= queueCapacity) {
        headPointer = 0;
    }
}

bool FrameQueue::isNextFrameValid() {
    bool isValid = false;
    for (uint8_t index = 0; index < frameSize; index++) {
        isValid |= queue[headPointer][index];
    }
    return isValid;
}

bool FrameQueue::push(uint8_t* frameBuffer) {
    IF_SERIAL_DEBUG(
        Serial.print(F("Remaining cap: "));
        Serial.println(remainingCapacity);
    );

    bool result = false;

    // Copy the current frame into the frame queue
    if (remainingCapacity > 0) {
        memcpy(queue[tailPointer], frameBuffer, frameSize);
        advanceTail();
        remainingCapacity--;

        result = true;
        IF_SERIAL_DEBUG(
            Serial.println(F(" ... ok"));
        );
    } else {
        IF_SERIAL_DEBUG(
            Serial.println(F(" ... failed"));
        );
    }

    return result;
}

bool FrameQueue::pop(uint8_t* frameBuffer) {
    if (!isEmpty() && isNextFrameValid()) {
        // Copy the next available frame from the queue into the provided buffer
        memcpy(frameBuffer, queue[headPointer], frameSize);
        memset(queue[headPointer], 0, frameSize);

        advanceHead();
        remainingCapacity++;

        return true;
    }
    return false;
}

bool FrameQueue::isEmpty() {
    return remainingCapacity == queueCapacity;
}

bool FrameQueue::isFullMessageAvailable(RF24NetworkHeader& header) {
    if (header.type != MESSAGE_SHARD_START && header.type != MESSAGE_SHARD && header.part == 0) {
        return true;
    }

    uint8_t numberOfParts = header.part + 1;
    for (uint8_t frameIndex = 0; frameIndex < queueCapacity && numberOfParts >= 0; frameIndex++) {
        RF24NetworkHeader peekHeader;
        memcpy(&peekHeader, queue[frameIndex], sizeof(RF24NetworkHeader));
        if (header.id == peekHeader.id) {
            numberOfParts--;
        }
    }
    return numberOfParts == 0;
}

void FrameQueue::peekNextHeader(RF24NetworkHeader& header) {
    if (!isEmpty() && isNextFrameValid()) {
        // Copy the next available frame from the queue into the provided buffer
        memcpy(&header, queue[headPointer], sizeof(RF24NetworkHeader));
    }
}

/******************************************************************/
RF24Network::RF24Network(RF24& _radio): radio(_radio) {
    for (uint8_t index = 0; index < maxChildNodes; index++) {
        childNodes[index] = FREE;
        pendingTTL[index] = 0;
    }
    isValidAddressSet = false;
}

bool RF24Network::beginAddressRequestProcedure(void (*_callback)(uint8_t channel, uint16_t nodeAddress)) {
    callback = _callback;
    randomSeed(analogRead(0));
    uuid = random(254);
    uint8_t channel = 90;
    beginInMulticastMode(channel); // Begin in the default channel
    uint8_t bytes[] = {uuid};

    RF24NetworkHeader header;
    header.type = ADDRESS_REQUEST_START;

    IF_SERIAL_DEBUG(
        Serial.print("Sending address request message with UUID: ");
        Serial.print(uuid);
        Serial.print(" ... ");
        unsigned long startTime = millis();
    );

    // Retry until someone hears us up to 128 times (once per channel)
    uint8_t getAddressRetries = 0;
    while(!isValidAddressSet && getAddressRetries < 128) {
        uint8_t sendAddressRequestRetry = 0;
        bool isMessageSent = false;
        while (!isMessageSent && sendAddressRequestRetry < 5) {
            delay(75 * sendAddressRequestRetry);
            isMessageSent = write(header, &bytes, sizeof(bytes), true);
            sendAddressRequestRetry++;
            IF_SERIAL_DEBUG(
                Serial.print(isMessageSent ? " OK! " : " X,");
            );
        }
        if (!isMessageSent) {
            //No one heard us
            channel++;
            if (channel > 127) {
                channel = 1;
            }
            radio.setChannel(channel);
            IF_SERIAL_DEBUG(
                Serial.print(" -> No reply, changing channel: ");
                Serial.print(channel);
                Serial.print(" Try #");
                Serial.println(getAddressRetries);
            );
            delay(2);
            getAddressRetries++;
            continue;
        } else {
            // The address request message was sent, wait for a reply
            unsigned long waitReplyTimestamp = millis();
            while (millis() - waitReplyTimestamp < 5000) {
                update();
            }
        }
    }

    /**
     * Once we have a valid address, invoke the callback so that the library's
     * client can do something if the address is set.
     */
    if (isValidAddressSet) {
        (*callback)(radio.getChannel(), node_address);
    }
    return isValidAddressSet;
}
/******************************************************************/
void RF24Network::beginInMulticastMode(uint8_t _channel) {
    if (!radio.isValid() ) {
        return;
    }

    // Set up the radio the way we want it to look
    radio.setChannel(_channel);
    radio.setDataRate(RF24_1MBPS);
    radio.setCRCLength(RF24_CRC_16);

    // Use different retry periods to reduce data collisions
    uint8_t retryVar = 5;
    radio.setRetries(retryVar, 15);
    txTimeout = retryVar * 17;

    // Start multicast address
    radio.openReadingPipe(0, multicastAddress);
    radio.startListening();
}

void RF24Network::begin(uint8_t _channel, uint16_t _node_address ) {
    if (!is_valid_address(_node_address)) {
        return;
    }

    node_address = _node_address;
    isValidAddressSet = true;

    if (!radio.isValid() ) {
        return;
    }

    // Set up the radio the way we want it to look
    radio.setChannel(_channel);
    radio.setDataRate(RF24_1MBPS);
    radio.setCRCLength(RF24_CRC_16);

    // Use different retry periods to reduce data collisions
    uint8_t retryVar = (node_address % 7) + 5;
    radio.setRetries(retryVar, 15);
    txTimeout = retryVar * 17;

    // Setup our address helper cache
    setup_address();

    // Open up all listening pipes
    IF_SERIAL_DEBUG(
        Serial.println(F("Opening listening pipes "));
    );

    // Start multicast address
    radio.openReadingPipe(0, multicastAddress);

    // Open up all listening pipes
    for (uint8_t pipeNumber = 1; pipeNumber <= 5; pipeNumber++) {
        IF_SERIAL_DEBUG(
            Serial.print("pipe number: ");
            Serial.println(pipeNumber);
        );
        radio.openReadingPipe(pipeNumber, pipe_address(_node_address, pipeNumber));
    }

    radio.startListening();
}

/******************************************************************/

void RF24Network::update(void) {
    // if there is data ready
    uint8_t pipe_num; // radio.available() will populate on which pipe we received the message
    if (radio.available(&pipe_num)) {
        // Dump the payloads until we've gotten everything
        Serial.println("NET We got something on radio");
        while (radio.available()) {
            // Fetch the payload, and see if this was the last one.
            radio.read(frame_buffer, sizeof(frame_buffer));

            // Read the beginning of the frame as the header
            const RF24NetworkHeader& header = * reinterpret_cast<RF24NetworkHeader*>(frame_buffer);

            IF_SERIAL_DEBUG(
                Serial.print(F("NET Received on pipe "));
                Serial.print(pipe_num);
                Serial.print(F(" : "));
                Serial.print(header.from_node);
                Serial.print(F("->"));
                Serial.println(header.to_node);
            );

            // Throw it away if it's not a valid address
            if (header.to_node != multicastHeaderAddress && !is_valid_address(header.to_node)) {
                continue;
            }

            // Is this for us?
            if (header.to_node == node_address || header.to_node == multicastHeaderAddress) {
                if (header.type == ADDRESS_REQUEST_START
                        || header.type == ADDRESS_OFFER
                        || header.type == ADDRESS_CONFIRM_OFFER) {
                    // Handle AddressRequest messages internally
                    handleAddressRequestFrame();
                } else {
                    // Add it to the buffer of frames for us
                    frameQueue.push(frame_buffer);
                    memset(frame_buffer, 0, frame_size);
                }
            } else {
                // Relay it
                write(header.to_node);
            }
        }
    }
}

/******************************************************************/

bool RF24Network::available(void) {
    if (!frameQueue.isEmpty()) {
        RF24NetworkHeader peekHeader;
        frameQueue.peekNextHeader(peekHeader);
        return frameQueue.isFullMessageAvailable(peekHeader);
    }
    return false;
}

/******************************************************************/

uint16_t RF24Network::parent() const {
    if ( node_address == 0 )
        return -1;
    else
        return parent_node;
}

/******************************************************************/

void RF24Network::peek(RF24NetworkHeader& header) {
    if (available()) {
        // Copy the next available frame from the queue into the provided buffer
        // memcpy(&header, next_frame - frame_size, sizeof(RF24NetworkHeader));
    }
}

/******************************************************************/

size_t RF24Network::read(RF24NetworkHeader& header, void* message, size_t maxlen) {
    size_t bufsize = 0;

    bool pop = frameQueue.pop(frame_buffer);
    if (pop) {
        // Copy the header portion into the header pointer
        memcpy(&header, frame_buffer, sizeof(RF24NetworkHeader));
        IF_SERIAL_DEBUG(
            Serial.print("message type ");
            Serial.println(header.type);
        );

        if (maxlen > 0) {
            // How much buffer size should we actually copy?
            bufsize += min(maxlen, frame_size - sizeof(RF24NetworkHeader));

            // Copy the next available frame from the queue into the provided buffer
            memcpy(message, frame_buffer + sizeof(RF24NetworkHeader), bufsize);

            bool popMessages = header.type == MESSAGE_SHARD_START;
            while (popMessages) {
                popMessages = frameQueue.pop(frame_buffer);
                if (popMessages) {
                    memcpy(&header, frame_buffer, sizeof(RF24NetworkHeader));

                    IF_SERIAL_DEBUG(
                        Serial.print(" [shard] message type ");
                        Serial.println(header.type);
                    );

                    // Copy the next available frame from the queue into the provided buffer
                    memcpy(message + bufsize,
                            frame_buffer + sizeof(RF24NetworkHeader),
                            min(maxlen, frame_size - sizeof(RF24NetworkHeader)));
                    bufsize += bufsize;
                }
            }
        }
    }
    return bufsize;
}

/***************************** PUBLIC *************************************/
bool RF24Network::write(RF24NetworkHeader& header, const void* message, size_t len) {
    return write(header, message, len, false);
}

bool RF24Network::writeMulticast(RF24NetworkHeader& header, const void* message, size_t len) {
    header.to_node = multicastHeaderAddress;
    return write(header, message, len, true);
}

/***************************** Private *************************************/
bool RF24Network::write(RF24NetworkHeader& header, const void* message, size_t len, const bool multicast) {
    uint8_t originalHeaderType = header.type;
    size_t totalMessageSize = len + sizeof(RF24NetworkHeader);
    uint8_t totalParts = (totalMessageSize / frame_size) + 1;
    IF_SERIAL_DEBUG(
        Serial.print("TotalMessageSize ");
        Serial.println(totalMessageSize);
        Serial.print("Required messages ");
        Serial.println(totalParts);
    );

    /**
     * Prevent multicast sending sharded multicast messages
     */
    if (multicast && totalParts > 1) {
        return false;
    }

    // Fill out the header
    header.from_node = node_address;
    bool isTransmissionOk = true;
    for (uint8_t partNumber = 0; partNumber < totalParts && isTransmissionOk; partNumber++) {
        if (totalParts > 1 && partNumber < totalParts - 1) {
            header.type = partNumber == 0 ? MESSAGE_SHARD_START : MESSAGE_SHARD;
        } else {
            header.type = originalHeaderType;
        }

        header.part = (totalParts - partNumber) - 1;
        IF_SERIAL_DEBUG(
            Serial.print("Sending message ");
            Serial.print(header.id);
            Serial.print(" part ");
            Serial.print(header.part);
            Serial.print(" type ");
            Serial.print(header.type);
            Serial.print(", remaining payload size ");
            Serial.println(len);
        );
        // Copy the header
        memcpy(frame_buffer, &header, sizeof(RF24NetworkHeader));

        if (len) { // Not sure if this IF() is needed
            // Copy the message
            uint8_t effectivePayloadSize = frame_size - sizeof(RF24NetworkHeader);
            memcpy(frame_buffer + sizeof(RF24NetworkHeader), // Move the pointer after the already copied header
                    message + (effectivePayloadSize * partNumber), // Copy from message the next "unsent" portion
                    min(effectivePayloadSize, len)); // Copy
            // Remove from the length the amount of bytes sent
            len -= effectivePayloadSize;
        }

        IF_SERIAL_DEBUG( // Print frame_buffer
            Serial.println(F("**FrameBuffer dump **"));
            for (int i = 0; i < frame_size; i++) {
                Serial.print(F("["));
                Serial.print(frame_buffer[i], HEX);
                Serial.print(F("] "));
            }
            Serial.println(F("\n****"));
        );


        // If the user is trying to send it to himself
        if (header.to_node == node_address) {
            // Just queue it in the received queue
            // isTransmissionOk &= enqueue();
        } else if (multicast) {
            isTransmissionOk &= broadcast();
        } else {
            // Otherwise send it out over the air
            isTransmissionOk &= write(header.to_node);
        }
        memset (frame_buffer, 0, frame_size);
    }
    return isTransmissionOk;
}

/******************************************************************/
bool RF24Network::broadcast() {
    IF_SERIAL_DEBUG(
        Serial.print("NET Sending multicast ...");
    );
    return write_to_pipe(multicastAddress, true);
}

bool RF24Network::write(uint16_t to_node) {
    // Throw it away if it's not a valid address
    if ( !is_valid_address(to_node) ) {
        return false;
    }

    // Where do we send this?  By default, to our parent
    uint16_t send_node = parent_node;
    uint8_t send_pipe = parent_pipe;

    if (is_direct_child(to_node)) {
        /*
         * If the node is a direct child,
         * send it directly to the child address and pipe
         */
        send_node = to_node;
        send_pipe = 1;

    } else if (is_descendant(to_node)) {
        /*
         * If the node is a child of a child
         * talk on our child's listening pipe,
         * and let the direct child relay it.
         */
        send_node = direct_child_route_to(to_node);
        send_pipe = 1;
    }

    return write_to_pipe(send_node, send_pipe);
}

/******************************************************************/
bool RF24Network::write_to_pipe(uint16_t node, uint8_t pipe, const bool multicast) {
    uint64_t out_pipe = pipe_address(node, pipe);
    IF_SERIAL_DEBUG(
        Serial.print(F("NET Sending message to ("));
        Serial.print(node, HEX);
        Serial.print(F(") on pipe ("));
        Serial.print(pipe);
        Serial.print(F(") ... "));
    );
    return write_to_pipe(out_pipe);
}

bool RF24Network::write_to_pipe(uint64_t out_pipe, const bool multicast) {
    bool ok = false;

    // First, stop listening so we can talk
    radio.stopListening();
    // Open the correct pipe for writing.
    radio.openWritingPipe(out_pipe);
    // ok = radio.writeFast(frame_buffer, frame_size, node == multicastAddress);
    ok = radio.writeFast(frame_buffer, frame_size, multicast);
    ok = radio.txStandBy(txTimeout);

    IF_SERIAL_DEBUG(
        Serial.println(ok ? F("ok") : F("failed"));
    );

    // Now, continue listening
    radio.startListening();

    return ok;
}

/******************************************************************/

bool RF24Network::is_direct_child( uint16_t node ) {
    bool result = false;

    /* A direct child of ours has the same low numbers as us, and only
     * one higher number.
     * e.g. node 0234 is a direct child of 034, and node 01234 is a
     * descendant but not a direct child
     */

    // First, is it even a descendant?
    if (is_descendant(node)) {
        // Does it only have ONE more level than us?
        uint16_t child_node_mask = ( ~ node_mask ) << 3;
        result = ( node & child_node_mask ) == 0 ;
    }

    return result;
}

/******************************************************************/

bool RF24Network::is_descendant(uint16_t node) {
    return ( node & node_mask ) == node_address;
}

/******************************************************************/

void RF24Network::setup_address(void) {
    // First, establish the node_mask
    uint16_t node_mask_check = 0xFFFF;
    while (node_address & node_mask_check) {
        node_mask_check <<= 3;
    }

    node_mask = ~ node_mask_check;

    // parent mask is the next level down
    uint16_t parent_mask = node_mask >> 3;

    // parent node is the part IN the mask
    parent_node = node_address & parent_mask;

    // parent pipe is the part OUT of the mask
    uint16_t i = node_address;
    uint16_t m = parent_mask;
    while (m) {
        i >>= 3;
        m >>= 3;
    }
    parent_pipe = i;
}

/******************************************************************/

uint16_t RF24Network::direct_child_route_to(uint16_t node) {
    // Presumes that this is in fact a child!!
    uint16_t child_mask = ( node_mask << 3 ) | 0B111;
    return node & child_mask ;
}

/******************************************************************/

uint8_t RF24Network::pipe_to_descendant(uint16_t node) {
    uint16_t i = node;
    uint16_t m = node_mask;

    while (m) {
        i >>= 3;
        m >>= 3;
    }

    return i & 0B111;
}

/******************************************************************/

void RF24Network::handleAddressRequestFrame() {
    // Read the beginning of the frame as the header
    const RF24NetworkHeader& header = * reinterpret_cast<RF24NetworkHeader*>(frame_buffer);
    uint8_t bytes[5];
    // Copy the next available frame from the queue into the provided buffer
    memcpy(bytes, frame_buffer + sizeof(RF24NetworkHeader), 5);

    if (header.type == ADDRESS_REQUEST_START) {
        handleAddressRequestStart(bytes, sizeof(bytes));
    } else if (header.type == ADDRESS_OFFER) {
        handleAddressOffer(bytes, sizeof(bytes), bytes[1], header.from_node);
    } else if (header.type == ADDRESS_CONFIRM_OFFER) {
        handleAddressConfirm(bytes);
    }
}

/******************************************************************/

void RF24Network::handleAddressRequestStart(uint8_t* bytes, size_t payloadSize) {
    for (uint8_t index = 0; index < maxChildNodes; index++) {
        if (childNodes[index] == FREE) {
            IF_SERIAL_DEBUG(
                Serial.print("Found a free child slot from this node, sending address node# offer ");
                Serial.println(index + 1);
            );

            RF24NetworkHeader header;
            header.type = ADDRESS_OFFER;
            bytes[1] = index + 1; // Nodes go from 1 to 5

            // Add a random delay to prevent clashes when multiple nodes offer address
            uint8_t delayPeriod = random(5, 50);
            delay(delayPeriod);
            bool ok = write(header, bytes, payloadSize, true);
            if (ok) {
                IF_SERIAL_DEBUG(
                    Serial.println("someone got the message, hopefully the requester");
                );
                childNodes[index] = PENDING;
                pendingTTL[index] = millis();
            } else {
                IF_SERIAL_DEBUG(
                    Serial.println("no one got the message");
                );
            }
            break;
        } else if (childNodes[index] == PENDING) {
            unsigned long ttlDiff = pendingTTL[index] - millis();
            ttlDiff = abs(ttlDiff);
            if (ttlDiff >= pendingTimeout * 1000) {
                childNodes[index] = FREE;
            }
        }
    }
}

/******************************************************************/

void RF24Network::handleAddressOffer(uint8_t* bytes, size_t payloadSize, uint16_t offeredNodeAddress,
        uint16_t parentAddress) {
    // Verify the offered address is for us
    if (bytes[0] == uuid) {
        IF_SERIAL_DEBUG(
            Serial.print("Got address offer from parent: ");
            Serial.print(parentAddress, DEC);
            Serial.print(" - ");
            Serial.print(parentAddress, OCT);
            Serial.print(" - ");
            Serial.print(parentAddress, BIN);
            Serial.print(", offeredNode ");
            Serial.println(offeredNodeAddress);
        );

         // Determine the level of the parent
        uint8_t parentLevel = 0;
        uint16_t n = parentAddress;
        while (n!=0) {
            n /= 010;
            ++parentLevel;
        }
        IF_SERIAL_DEBUG(
            Serial.print("Computed parent level: ");
            Serial.println(parentLevel);
        );

        // Convert parent address to decimal
        uint16_t parentInDecimal = octalToDecimal(parentAddress);
        IF_SERIAL_DEBUG (
            Serial.print("Parent in decimal ");
            Serial.print(parentInDecimal, DEC);
            Serial.print(" - ");
            Serial.print(parentInDecimal, OCT);
            Serial.print(" - ");
            Serial.println(parentInDecimal, BIN);
        );
        uint16_t newNodeAddress = precisePow(8, parentLevel) * offeredNodeAddress;
        newNodeAddress += parentInDecimal;

        // Reset the network
        radio.stopListening();
        begin(radio.getChannel(), newNodeAddress);

        // Send message to parent to confirm offer
        RF24NetworkHeader header(parentAddress);
        header.type = ADDRESS_CONFIRM_OFFER;
        isValidAddressSet = write(header, bytes, payloadSize);
        IF_SERIAL_DEBUG(
            Serial.print("New node [");
            Serial.print(bytes[1]);
            Serial.print("] ");
            Serial.print(newNodeAddress, DEC);
            Serial.print(" - ");
            Serial.print(newNodeAddress, OCT);
            Serial.print(" - ");
            Serial.println(newNodeAddress, BIN);

            Serial.print("Confirming address ... ");
            Serial.println(isValidAddressSet ? " ok" : " fail");
        );
    }
}

/******************************************************************/

void RF24Network::handleAddressConfirm(uint8_t* bytes) {
    uint8_t indexOfReservedSlot = bytes[1] - 1;
    IF_SERIAL_DEBUG(
        Serial.println("The requester confirmed the address");
        Serial.print("Registering index as used ");
        Serial.println(indexOfReservedSlot);
    );

    // Verify bounds to prevent buffer overflow attacks
    if (indexOfReservedSlot < maxChildNodes) {
        childNodes[indexOfReservedSlot] = USED;
    }
}
/******************************************************************/

bool is_valid_address(uint16_t node) {
    if (node == RF24Network::multicastHeaderAddress) {
        return true;
    }

    bool result = true;

    while (node) {
        uint8_t digit = node & 0B111;
        if (digit < 1 || digit > 5) {
            result = false;
            break;
        }
        node >>= 3;
    }

    return result;
}

/******************************************************************/

uint64_t pipe_address(uint16_t node, uint8_t pipe) {
    static uint8_t pipe_segment[] = { 0x3c, 0x5a, 0x69, 0x96, 0xa5, 0xc3 };

    uint64_t result;
    uint8_t* out = reinterpret_cast<uint8_t*>(&result);

    out[0] = pipe_segment[pipe];

    uint8_t w;
    short i = 4;
    short shift = 12;
    while (i--) {
        w = ( node >> shift ) & 0xF ;
        w |= ~w << 4;
        out[i + 1] = w;

        shift -= 4;
    }
    return result;
}

uint32_t precisePow(uint32_t base, uint8_t power) {
    uint32_t result = 1;
    while(power > 0) {
        power--;
        result *= base;
    }
    return result;
}
uint32_t octalToDecimal(uint32_t n) {
    uint32_t decimal=0, i=0, rem;
    while (n!=0) {
        rem = n % 010;
        n /= 010;
        decimal += rem * precisePow(8, i);
        ++i;
    }
    return decimal;
}
