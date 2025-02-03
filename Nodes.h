/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: example-multihop.c,v 1.7 2010/01/15 10:24:36 nifi Exp $
 */
/**
 * \file
 *         Testing the multihop forwarding layer (multihop) in Rime
 * \author
 *         Adam Dunkels <adam@sics.se>
 *
 *         This example shows how to use the multihop Rime module, how
 *         to use the announcement mechanism, how to manage a list
 *         with the list module, and how to allocate memory with the
 *         memb module.
 *
 *         The multihop module provides hooks for forwarding packets
 *         in a multi-hops fashion, but does not implement any routing
 *         protocol. A routing mechanism must be provided by the
 *         application or protocol running on top of the multihop
 *         module. In this case, this example program provides the
 *         routing mechanism.
 *
 *         The routing mechanism implemented by this example program
 *         is very simple: it forwards every incoming packet to a
 *         random neighbor. The program maintains a list of neighbors,
 *         which it populated through the use of the announcement
 *         mechanism.
 *
 *         The neighbor list is populated by incoming announcements
 *         from neighbors. The program maintains a list of neighbors,
 *         where each entry is allocated from a MEMB() (memory block
 *         pool). Each neighbor has a timeout so that they do not
 *         occupy their list entry for too long.
 *
 *         When a packet arrives to the node, the function forward()
 *         is called by the multihop layer. This function picks a
 *         random neighbor to send the packet to. The packet is
 *         forwarded by every node in the network until it reaches its
 *         final destination (or is discarded in transit due to a
 *         transmission error or a collision).
 */

/**
 * \author
 *         Andreas Naoum <anaoum01@ucy.ac.cy>
 *
 *   Utilizing Mobile Nodes for a Fault Management Algorithm in WSNs and IoT Networks
 * 
 *
 */

#include "contiki.h"
#include "net/rime/rime.h"
#include "net/mac/csma.h"
#include "net/rime/packetqueue.h"
#include "lib/list.h"
#include "lib/memb.h"
#include "lib/random.h"
#include "dev/button-sensor.h"
#include "dev/light-sensor.h"
#include "dev/leds.h"
#include "dev/cc2420/cc2420.h"
#include "dev/serial-line.h"
#include "sys/energest.h"
#include "sys/clock.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <ctype.h>

/*--------------------------------- DECLARATIONS ------------------------------------------*/


#define CHANNEL 135
#define MAX_NEIGHBORS 8
#define UNKNOWN_HOPS -1
#define NUM_NODES 20
#define SPEED 0.65

#define DEBUG   0
#define DEBUG1  1
#define DEBUGF  1
#define DEBUGFT  1
#define DEBUGFM  1

#define MOBILENODESNUM 6

#define MAX_ENERGY 460800 // 460800J -> 460800000mJ => 128Wh (Watt/hour)

#define ENERGY_MAX_LIMIT 12960000    //max=12960J -> 12960000mJ
#define ENERGY_50_LIMIT /*6480*/ 6480000
#define ENERGY_70_LIMIT /*10000*/ 9072000
#define ENERGY_80_LIMIT /*10500*/ 10368000
#define ENERGY_85_LIMIT /*25000*/ 11016000
#define BUFFER_MAX_LIMIT /*4*/ 8
#define BUFFER_50_LIMIT /*2*/ 4
#define BUFFER_80_LIMIT /*3*/ 6
#define BUFFER_85_LIMIT /*4*/ 7

/*--------------------------------- DATA STRUCTURES ------------------------------------------*/

/*
 * [Modified]
 * This is the struct of a neigbor table entry.
 *  - next => pointer to the next table entry
 *  - addr => neigbor rime address 
 *  - rssi => rssi between current sensor and its neighbor
 *  - hops => hops from neighbor to sink
 *  - buffer_occupancy => the amount of information at any given time, in unit of bits
 *  - Flag => Network usable static node
 *  - flow => flow daLpas pano apo 1 node
 *  - next_sequence_packet_number => 
 *  - num_of_packets_received => number of packets received
 *  - last_clock => last time of receiving a packet 
 *  - mobileFlag => if the node is mobile node
 *  - isMobileNeighbor => if the node has a mobile node neighbor
 *  - isNextHop => if it is the selected one-hop path to the sink
*/
struct table_entry_neighbor{
  struct table_entry_neighbor *next;
  linkaddr_t addr;
  uint16_t rssi;    
  int hops; 
  int buffer_occupancy;
  double remaining_energy;
  bool next_sequence_packet_number;
  int num_of_packets_received;
  uint32_t last_clock;
  bool mobileFlag;
  bool isNextHop;
  bool isSourceNode;
 bool isCritical;
 bool haveAlternative;
};

struct faulty_entry{
	struct faulty_entry *next;
	int faultyID;
};

struct discovery_entry{
	struct discovery_entry *next;
	int id;
	bool noUpperNodes;
};

