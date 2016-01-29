/*
 * Copyright (C) 2011 James Coliz, Jr. <maniacbug@ymail.com>
 * 2015 Ricardo Rodriguez.  <ricardo@ollinlabs.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#ifndef __RF24NETWORK_H__
#define __RF24NETWORK_H__

#define MESSAGE_SHARD 0x80
#define MESSAGE_SHARD_START 0x81

#define ADDRESS_REQUEST_START 0x82
#define ADDRESS_OFFER 0x83
#define ADDRESS_CONFIRM_OFFER 0x84

/**
 * @file RF24Network.h
 *
 * Class declaration for RF24Network
 */

#include <stddef.h>
#include <stdint.h>

enum CHILD_NODE_STATUS {FREE = 0, PENDING = 1, USED = 2};

class RF24;

/**
 * Header which is sent with each message
 *
 * The frame put over the air consists of this header and a message
 */
struct RF24NetworkHeader
{
    uint16_t from_node; /**< Logical address where the message was generated */
    uint16_t to_node; /**< Logical address where the message is going */
    uint16_t id; /**< Sequential message ID, incremented every message */
    uint8_t part;
    uint8_t type; /**< Type of the packet.  0-127 are user-defined types, 128-255 are reserved for system */
    static uint16_t next_id; /**< The message ID of the next message to be sent */

    /**
     * Default constructor
     *

     * Simply constructs a blank header
     */
    RF24NetworkHeader() {}

    /**
     * Send constructor
     *
     * Use this constructor to create a header and then send a message
     *
     * @code
     *  RF24NetworkHeader header(recipient_address,'t');
     *  network.write(header,&message,sizeof(message));
     * @endcode
     *
     * @param _to The logical node address where the message is going
     * @param _type The type of message which follows.  Only 0-127 are allowed for
     * user messages. By default will be set to 0x7f (127)
     */
    RF24NetworkHeader(uint16_t _to, unsigned char _type = 0): to_node(_to), id(next_id++), part(0), type(_type & 0x7f) {}
};


/**
  * Fifo tyoe queue
  */
class FrameQueue {
public:
    FrameQueue(void);

    bool isEmpty();

    /**
     * Insert an element into the queue
     */
    bool pop(uint8_t* frameBuffer);

    /**
     * Push an element to the back of the queue
     */
    bool push(uint8_t* frameBuffer);

    /**
     * Determines if all the frames <shards> of a message are available
     */
    bool isFullMessageAvailable(RF24NetworkHeader& header);

    /**
     * Returns the next header in the queue without removing it (peek)
     */
    void peekNextHeader(RF24NetworkHeader& header);

private:
    /**
     * Determines if the next frame in the queue is valid.
     * The validity will be determined by travesing the frame and ensuring it
     * does not contain all 0's
     */
    bool isNextFrameValid(void);

    /**
     * Queue control method.
     * Advance the FIFO tail.
     */
    void advanceTail(void);

    /**
     * Queue control method.
     * Advance the FIFO head.
     */
    void advanceHead(void);

    /**
     * Queue control method.
     * Rewind the FIFO head.
     */
    void rewindHead(void);

private:
    const static uint8_t queueCapacity = 5;
    const static uint8_t frameSize = 32;

    /**
     *
     */
    uint8_t remainingCapacity;

    /**
     * Space for a small set of frames that need to be delivered to the
     * app layer.
     * I has a capacity for 5, 32 bit frames
     */
    uint8_t queue[queueCapacity][frameSize];

    /**
     * Index of the position of the head of @p queue where we should read the
     * next frame
     */
    uint8_t headPointer;

    /**
     * Index of the position of the head of @p queue where we place the next
     * received frame
     */
    uint8_t tailPointer;
};


/**
 * 2014 - Optimized Network Layer for RF24 Radios
 *
 * This class implements an OSI Network Layer using nRF24L01(+) radios driven
 * by RF24 library.
 */
class RF24Network {
public:
    /**
     * Construct the network
     *
     * @param _radio The underlying radio driver instance
     *
     */

    RF24Network( RF24& _radio );

    /**
     * Bring up the network
     *
     * @warning Be sure to 'begin' the radio first.
     *
     * @param _channel The RF channel to operate on
     * @param _node_address The logical address of this node
     */
    void begin(uint8_t _channel, uint16_t _node_address );

    /**
     * Bring up the network in multicast mode only.
     * The node will not be reachable directly but will only listen to
     * multicast messages.
     *
     * @warning Be sure to 'begin' the radio first.
     *
     * @param _channel The RF channel to operate on
     */
    void beginInMulticastMode(uint8_t _channel);

    /**
     * Initiates the address request procedure in a node.
     * this method will return true if the procedure succeeds, false otherwise
     *
     * The provided callback will be executed when a valid address is found.
     */
    bool beginAddressRequestProcedure(void (*callback)(uint8_t channel, uint16_t nodeAddress));

    /**
     * Main layer loop
     *
     * This function must be called regularly to keep the layer going.  This is where all
     * the action happens!
     */
    void update(void);

    /**
     * Test whether there is a message available for this node
     *
     * @return Whether there is a message available for this node
     */
    bool available(void);

    /**
     * Read the next available header
     *
     * Reads the next available header without advancing to the next
     * incoming message.  Useful for doing a switch on the message type
     *
     * If there is no message available, the header is not touched
     *
     * @param[out] header The header (envelope) of the next message
     */
    void peek(RF24NetworkHeader& header);

    /**
     * Read a message
     *
     * @param[out] header The header (envelope) of this message
     * @param[out] message Pointer to memory where the message should be placed
     * @param maxlen The largest message size which can be held in @p message
     * @return The total number of bytes copied into @p message
     */
    size_t read(RF24NetworkHeader& header, void* message, size_t maxlen);

    /**
     * Send a message
     *
     * @note Optimization: Extended timeouts/retries enabled. See txTimeout for more info.
     * @param[in,out] header The header (envelope) of this message.  The critical
     * thing to fill in is the @p to_node field so we know where to send the
     * message.  It is then updated with the details of the actual header sent.
     * @param message Pointer to memory where the message is located
     * @param len The size of the message
     * @param multicast (optional, defaults to false) indicates if the message
     *              should be broadcasted across the network.
     * @return Whether the message was successfully received
     */
    bool write(RF24NetworkHeader& header, const void* message, size_t len);
    bool writeMulticast(RF24NetworkHeader& header, const void* message, size_t len);

    /**
     * This node's parent address
     *
     * @return This node's parent address, or -1 if this is the base
     */
    uint16_t parent() const;

    /**
     * @note: Optimization:This value is automatically assigned based on the node address
     * to reduce errors and increase throughput of the network.
     *
     * Sets the timeout period for individual payloads in milliseconds at staggered intervals.
     * Payloads will be retried automatically until success or timeout
     * Set to 0 to use the normal auto retry period defined by radio.setRetries()
     *
     */
    unsigned long txTimeout;

    /**
     * Used as the actual pipe address to send multicast messages. All nodes
     * will open pipe0 to listen to this address
     */
    const static uint64_t multicastAddress = 0xABCDABCD71LL; // 737889996145
    /**
     * Used as a logical address in the header to indicate it is destined FOR
     * multicast. The alternatives for this where to
     * a) Add a flag to the header (one mor bit to send on EVERY message)
     * b) Define a header_type to indicate its multicast. (Would hinder the
     *      ability to send messages with headers, necessary for addressRequest)
     */
    const static uint16_t multicastHeaderAddress = 065535;

private:
    void open_pipes(void);
    uint16_t find_node(uint16_t current_node, uint16_t target_node);
    bool write(uint16_t);
    bool write(RF24NetworkHeader& header, const void* message, size_t len, const bool multicast);
    bool broadcast(void);
    bool write_to_pipe(uint16_t node, uint8_t pipe, const bool multicast = false);
    bool write_to_pipe(uint64_t out_pipe, const bool multicast = false);
    bool enqueue(void);

    void handleAddressRequestFrame(void);
    void handleAddressRequestStart(uint8_t* bytes, size_t payloadSize);
    void handleAddressOffer(uint8_t* bytes, size_t payloadSize, uint16_t offeredNodeAddress, uint16_t parentAddress);
    void handleAddressConfirm(uint8_t* bytes);


    bool is_direct_child(uint16_t node);
    bool is_descendant(uint16_t node);
    uint16_t direct_child_route_to(uint16_t node);
    uint8_t pipe_to_descendant(uint16_t node);
    void setup_address(void);

private:
    RF24& radio; /**< Underlying radio driver, provides link/physical layers */
    uint16_t node_address; /**< Logical node address of this unit, 1 .. UINT_MAX */
    const static int frame_size = 32; /**< How large is each frame over the air */
    uint8_t frame_buffer[frame_size]; /**< Space to put the frame that will be sent/received over the air */
    FrameQueue frameQueue;

    uint16_t parent_node; /**< Our parent's node address */
    uint8_t parent_pipe; /**< The pipe our parent uses to listen to us */
    uint16_t node_mask; /**< The bits which contain signfificant node address information */

    const static uint8_t maxChildNodes = 5;
    CHILD_NODE_STATUS childNodes[maxChildNodes];
    unsigned long pendingTTL[4];
    const static uint8_t pendingTimeout = 20; //sec
    void (*callback)(uint8_t channel, uint16_t nodeAddress);
    bool isValidAddressSet;
    uint8_t uuid;
};

#endif // __RF24NETWORK_H__
