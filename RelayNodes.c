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

#include "Nodes.h"

#define SCENARIO 12

#define RANDOM_RELAY 1

#define CRITICAL 0

//#define NUM_STATIC 31
//#define NUM_STATIC 21
// #define NUM_STATIC 16
#define NUM_STATIC 23

//#define RELAY_NODES 21
//#define RELAY_NODES 11
// #define RELAY_NODES 9
#define RELAY_NODES 13

// initialization variables
uint32_t start_time = 0;
bool initialization=false;

// sink node is fixed (ID:1.0)
linkaddr_t sink_node;

// node's location
float node_loc_x;
float node_loc_y;

// node's level
int my_hops=UNKNOWN_HOPS;

// best neighbor's level
int best_hop=UNKNOWN_HOPS;

// connection status
bool connection = true;
bool noUpperNodes = false;

// if I am faulty
bool faulty = false;

// packets received
int number_of_packets_received = 0;
int number_of_packets_received_bymyself = 0;

// packets forwarded
int number_of_packets_forwarded = 0;

// announcements received
bool total_received_announcement = false;

// the amount of information at the moment
int buffer_occupancy=0;

// node's remaining energy
double remaining_energy=0.0;
int energy_of_node_for_announcements = 0;

// fixed threshold for neighbor failt detection 
uint32_t timelimit_neighbor = 60; 

// flag for suspicious faulty neighbor
bool suspiciousFaultyFlag = false;
int faultyID = 0;
struct table_entry_neighbor *suspiciousFaulty;

// detection variables
bool neighborDetectionFlag = false;
bool responseDetect = false;

// TIMERS
static struct etimer et,timer_energy,timer_alive,timer_fault;

// how many packets sent
int total_sensor_packets = 0;

// flag for discovery response
bool discovery_introduced = false;

struct packetqueue *pqueue;
struct broadcast_message {
  uint8_t seqno;
};

unsigned short r;
int an;

// node's neighbor table
LIST(neighbor_table);
LIST(faulty_table);
MEMB(neighbor_mem, struct table_entry_neighbor, MAX_NEIGHBORS);
MEMB(faulty_mem, struct faulty_entry, MAX_NEIGHBORS);
/*---------------------------------------------------------------------------*/
PROCESS(example_multihop_process, "WSP Routing");
PROCESS(energy, "Energy");
PROCESS(alive, "Alive");
PROCESS(fault_detection, "Fault Detection");
AUTOSTART_PROCESSES(&example_multihop_process,&energy,&alive,&fault_detection);
static struct multihop_conn multihop;
static struct broadcast_conn broadcast;
static struct announcement example_announcement;
static struct unicast_conn uc;
/*---------------------------------------------------------------------------*/
static void displayNeighborTable();
static int getBestNeighborHopNum();
uint16_t concatenate(unsigned x, unsigned y);
static void recv(struct multihop_conn *c, const linkaddr_t *sender,const linkaddr_t *prevhop,uint8_t hops);
static void addNewNeighbor(int hops, int rssi, const linkaddr_t *from, bool mobile);
static bool alreadyNeighbor(int id);
static int levelNeighbor(int id);
static void changeLevelNeighbor(int id,int hops);
static linkaddr_t * find_path();
static linkaddr_t *forward(struct multihop_conn *c,const linkaddr_t *originator, const linkaddr_t *dest,const linkaddr_t *prevhop, uint8_t hops);
static bool faultyNode(int node);
static bool isFaultyNode(int node);
// static int countDownNodes();
// static bool anyCritical();
static void checkUpperNodes(int faulty);
struct table_entry_neighbor* neighborThreshold();
void resetThreshold();
struct table_entry_neighbor* findNodeEntry(int id);
void deleteNeighbor(int id);
static void faultdetection();
void failuremsg(int faultyID, int level);
void sendAliveMessage();
void send_broadcast(char *s);
void send_unicast(char *str,int from);
void send_announcement(int value);
void send_announcement_char(int character, unsigned value);
void sendPeriodicMessage();
static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from);
static void recv_uc(struct unicast_conn *c, const linkaddr_t *from);
/*---------------------------------------------------------------------------*/
/*
 * [Added]
 * This function is called when we need to display the neighbor table.
 * The function displays the table content.
*/
static void displayNeighborTable(){
    int pos = 0;
    struct table_entry_neighbor *current_entry;
    printf("- displayNeighborTable > ");
    for(current_entry=list_head(neighbor_table); current_entry!=NULL; current_entry=current_entry->next, pos++) {
        printf("t[%d] = %d.%d with RSSI %d and hops %d and nextSPN: %d and energy: %ld.%03u and last_time: %lu and mobile flag: %d Level: %d|| ", pos, 
		current_entry->addr.u8[0], current_entry->addr.u8[1],current_entry->rssi, current_entry->hops,current_entry->next_sequence_packet_number,(long)current_entry->remaining_energy,(unsigned)((current_entry->remaining_energy-floor(current_entry->remaining_energy))*1000),current_entry->last_clock,current_entry->mobileFlag, current_entry->hops);
    }
    printf("\n");
}


//put two integers to one
uint16_t concatenate(unsigned x, unsigned y){
    unsigned pow = 10;
    while(y >= pow)
        pow *= 10;
    uint16_t res = x * pow + y;
    return res;        
}
/*---------------------------------------------------------------------------
  ---------------------------------------------------------------------------*/

static void recv(struct multihop_conn *c, const linkaddr_t *sender,const linkaddr_t *prevhop,uint8_t hops){
	
	//if (DEBUGFM)
		//printf("multihop message received '%s'\n", (char *)packetbuf_dataptr());
  
}


/*---------------------------------------------------------------------------*/
/*  [Added by Naoum] 
 * 
 * SUPPORTIVE FUNCTIONS 
 *
 */
 /*---------------------------------------------------------------------------*/
static void addNewNeighbor(int hops, int rssi, const linkaddr_t *from, bool mobile){
	struct table_entry_neighbor *e = memb_alloc(&neighbor_mem);
	if(e != NULL) {
		linkaddr_copy(&e->addr, from);
		if (mobile && hops==0)
			e->hops =-1;
		else 
			e->hops = hops;
		e->rssi = rssi;
		e->next_sequence_packet_number=true;
		e->num_of_packets_received = 1;
		e->buffer_occupancy = 0;
		e->remaining_energy = 0.0;
		e->last_clock = clock_seconds();
		e->mobileFlag = mobile;
		e->isNextHop = false;
		e->isCritical = false; 
		list_add(neighbor_table, e);
	}
}

static bool alreadyNeighbor(int id){
	struct table_entry_neighbor *e;
	for(e = list_head(neighbor_table); e!=NULL; e=e->next)
		if(id == e->addr.u8[0])
				return true;
	return false;
}

static int levelNeighbor(int id){
	struct table_entry_neighbor *e;
	for(e = list_head(neighbor_table); e!=NULL; e=e->next)
		if(id == e->addr.u8[0])
				return e->hops;
	return 0;
}

static void changeLevelNeighbor(int id,int hops){
	struct table_entry_neighbor *e;
	for(e = list_head(neighbor_table); e!=NULL; e=e->next)
		if(id == e->addr.u8[0])
				e->hops = hops;
}

struct table_entry_neighbor* findNodeEntry(int id){
	struct table_entry_neighbor  *me;
	for(me=list_head(neighbor_table);me!=NULL;me=me->next)
		if(me->addr.u8[0] == id)
			return me;
	return NULL;
}

void deleteNeighbor(int id){
	struct table_entry_neighbor  *me;
	for(me=list_head(neighbor_table);me!=NULL;me=me->next)
		if(me->addr.u8[0] == id){
			list_remove(neighbor_table, me);
			return;
		}
}

/*---------------------------------------------------------------------------*/
/*  
 * 
 * MULTIHOP FUNCTIONS
 *
 */
 /*---------------------------------------------------------------------------*/
/*
 * [Added by Andreas Naoum]
 * Sends a periodic message to the sink node using the multihop protocol
 */
void sendPeriodicMessage(){
	packetbuf_copyfrom("Trust me!I am a benign!", 24);
	packetbuf_set_attr(PACKETBUF_ATTR_TRANSMIT_TIME, clock_seconds());
	if(!linkaddr_cmp(&sink_node, &linkaddr_node_addr)) {
		total_sensor_packets++;
		printf("Sensor %d: %d\n", linkaddr_node_addr.u8[0], total_sensor_packets);
		linkaddr_t to;
		to.u8[0] = 1;
		to.u8[1] = 0;
		multihop_send(&multihop, &to);
	}
}

/*
 * [Modified by Andreas Naoum]
 * This function is called to forward a packet. The function picks the
 * neighbor from the neighbor list that has the minimun hops number and 
 * returns its address. If there are two or more neigbors with the same 
 * hops number returns the one that has the maximum RSSI. The multihop 
 * layer sends the packet to this address. If no neighbor is
 * found, the function returns NULL to signal to the multihop layer
 * that the packet should be dropped.
 */
static linkaddr_t *forward(struct multihop_conn *c,const linkaddr_t *originator, const linkaddr_t *dest,const linkaddr_t *prevhop, uint8_t hops){
    if((originator->u8[0] != linkaddr_node_addr.u8[0] ) || (originator->u8[0] == linkaddr_node_addr.u8[0] )){
	struct table_entry_neighbor *e;
	for(e = list_head(neighbor_table); e!=NULL; e=e->next)
		if ((e->addr.u8[0]) == prevhop->u8[0])
			e->last_clock = clock_seconds();
        buffer_occupancy++;
        if(number_of_packets_received==0){
            start_time = clock_seconds();
        }
        //to skip the set up phase
        total_received_announcement = true;
        //to calculate the data rate
        number_of_packets_received++; 
        //new list is created with the avaiable nodes of the neighboring table.
        if((packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[0]) == 0 ){ 
            //it's me who send the packet
            number_of_packets_received_bymyself++;
        }
        //to calculate the trasmition rate
        number_of_packets_forwarded++;
        linkaddr_t * nexthop = find_path();
	if (nexthop == NULL)
		printf("Packet lost!\n");
        if(DEBUG1){
            if( nexthop != NULL )
                printf("%d.%d: Forwarding packet to %d.%d from %d  \n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],nexthop->u8[0], nexthop->u8[1], packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[0]);
            else
                printf("%d.%d: Forwarding packet to nowone! \n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
        }   
        if(nexthop !=NULL){
            energy_of_node_for_announcements = energy_of_node_for_announcements+1;
            if( energy_of_node_for_announcements>999){
                energy_of_node_for_announcements =  energy_of_node_for_announcements % 999;
            }
        }
        buffer_occupancy--;
        //return next hops
        return nexthop;
    }
    return NULL;
}

/*---------------------------------------------------------------------------*/
/*
 * [Added by Natalie Temene]
 * This function is called when a node sets its announcement. The checks the 
 * neighbor table to find out the minimum hops between that node and sink node.
*/
static int getBestNeighborHopNum(){
    if (faulty)
	return 0;
    int best_hops = UNKNOWN_HOPS;
    int first_flag = 1;
    struct table_entry_neighbor *current_entry;
    // 1. No entry in the neighbor table
    if(list_length(neighbor_table) == 0) {
        return 0;
    }
    // 2. Only one entry in the neigbor table
    if(list_length(neighbor_table) == 1) {
        current_entry = list_head(neighbor_table);
        if(current_entry->hops != UNKNOWN_HOPS){
            return current_entry->hops;
        }
        else
            return 0;
    }
    // 3. More entries in the neighbor table
    for(current_entry = list_head(neighbor_table); current_entry!=NULL; current_entry=current_entry->next) {
        if(first_flag == 1){
            if(current_entry->hops != UNKNOWN_HOPS){
                best_hops = current_entry->hops;
                first_flag = 0;
            }
        }
        else if(current_entry->hops != UNKNOWN_HOPS){
            if(best_hops > current_entry->hops){
                best_hops = current_entry->hops;
            }
        }
    }
    if(best_hops == UNKNOWN_HOPS)
        return 0;
    else
        return best_hops;
}

static linkaddr_t * find_path(){
	struct table_entry_neighbor *best = NULL,*current;
	int min=my_hops;
	 //find the mobile node!
	 for(current = list_head(neighbor_table); current!=NULL; current=current->next)
		if(current->hops > UNKNOWN_HOPS && current->hops < min && current->mobileFlag){
			min = current->hops;
			best = current;
		}
	if ( min != my_hops)
		return &(best->addr);	
	//find the node with the smallest hops! 
       for(current = list_head(neighbor_table); current!=NULL; current=current->next)
		if(current->hops > UNKNOWN_HOPS && current->hops < min){
			min = current->hops;
			best = current;
		}
	printf("find path is :%d\n", best->addr.u8[0]);
	if ( min != my_hops)
		return &(best->addr);	
	return NULL;
}

/*---------------------------------------------------------------------------*/
/*  
 * 
 * ANNOUNCEMENT FUNCTIONS
 *
 */
 /*---------------------------------------------------------------------------*/

void send_announcement(int value){
	int val = 0 - value;
	announcement_set_value(&example_announcement, val);
	announcement_bump(&example_announcement);
}

void send_announcement_char(int character, unsigned value){
	uint16_t val = concatenate(character,value);
	announcement_set_value(&example_announcement, val);
	announcement_bump(&example_announcement);
}

/*
 * [Modified by Andreas Naoum]
 * This function is called when an incoming announcement arrives. The
 * function checks the neighbor table to see if the neighbor is
 * already present in the list. If the neighbor is not present in the
 * list, a new neighbor table entry is allocated and is added to the
 * neighbor table.
 */
static void received_announcement(struct announcement *a,const linkaddr_t *from,uint16_t id, uint16_t value){
	struct table_entry_neighbor *e;
	int value1;
	if(DEBUGFM)
		printf("- Sensor %d.%d:Got announcement from sensor %d value %d \n", linkaddr_node_addr.u8[0],linkaddr_node_addr.u8[1],from->u8[0],(int)value);

	if(((int)value) == -11){ 
		if (discovery_introduced){
			discovery_introduced = false;
			return;
		}
		if (DEBUGFT)
			printf("Discovery request received from %d (Distributed Fault Detection)\n",from->u8[0]); 
		int value ;
		if (noUpperNodes)
			value = concatenate(68,1);
		else 
			 value = concatenate(68,0);
		announcement_set_value(&example_announcement, value);
		announcement_bump(&example_announcement);
		discovery_introduced = true;;
	}
	else if(((int)value) == -14){ 
		 if (connection == false){
			announcement_set_value(&example_announcement, -14);
			announcement_bump(&example_announcement);
			if (DEBUGFM)
				printf("Start sending packets (Distributed Fault Detection)\n"); 
			connection = true;
			noUpperNodes = false;
			resetThreshold();
			//displayNeighborTable();
		}
	}
    else { 
	char s[11]; 
      sprintf(s,"%d", (int)value); 
        if(strlen(s)!=2){
		int choice = concatenate(s[0] - '0', s[1] - '0');
		int v=s[2] - '0';
		int i=3;
		while(s[i]!='\0'){
			v=concatenate(v, s[i] - '0');
			i++;
		}
		value1 = v;
            	if (choice == 72){
			printf("Initialization -> received hops: %d from %d \n",value1,from->u8[0]); 
		        int hops = 0; //arxiki timi
		        int third_case = 0;
		        uint16_t rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI)-45;
			linkaddr_t sink;
			sink.u8[0] =1;
			sink.u8[1] = 0; 

		        if (linkaddr_cmp(&linkaddr_node_addr, &sink))
		            hops = 0;
		        else if(linkaddr_cmp(from, &sink))
		            hops = 1;
		        else
		            third_case = 1;

			for(e = list_head(neighbor_table); e != NULL; e = e->next) {
				if(linkaddr_cmp(from, &e->addr)) {
					if (e->hops == UNKNOWN_HOPS && value1>=1) { 
						e->rssi = rssi;
						e->hops = value1 + 1;
						best_hop = getBestNeighborHopNum();
						if(best_hop>my_hops ){
							my_hops = best_hop+1;
							uint16_t value = concatenate(72,getBestNeighborHopNum());
							announcement_set_value(&example_announcement, value);
							announcement_bump(&example_announcement);
						}
					}
				
		                	return;
				}
			}
			int new_hops = 0;
			if (third_case == 1) {
					if ( value1 >= 1 ) 
						 new_hops = value1 + 1;
					else 
						 new_hops = UNKNOWN_HOPS;
			} 
			else 
				 new_hops = hops;
			addNewNeighbor(new_hops,rssi,from,false);
			best_hop = getBestNeighborHopNum();
			if(best_hop>my_hops){
				my_hops = best_hop+1;
		                uint16_t value = concatenate(72,getBestNeighborHopNum());//'H'
				announcement_set_value(&example_announcement, value);
				announcement_bump(&example_announcement);
			}
		}
		else if(choice == 80){ //MOBILE HOPShops!

				if (alreadyNeighbor(from->u8[0]) == false && value1!=0){
					uint16_t rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI)-45;
					addNewNeighbor(value1, rssi, from, true);
					displayNeighborTable();
					printf("ADDED NEIGHBOR %d \n", from->u8[0]); 
					send_announcement_char(81,my_hops);
				}
				else if (levelNeighbor(from->u8[0])!=value1)
					changeLevelNeighbor(from->u8[0],value1);
		}
		// FAILURE
		else if(choice == 70){ 
			 if (connection){
				bool recourse = false;
				for(e = list_head(neighbor_table); e!=NULL; e=e->next)
					if( (e->hops) < value1)
						recourse = true;
				if (my_hops <= value1 && recourse){
					printf("Alternative path (Distributed Fault Detection)\n"); 
					char str[100];
					sprintf(str,"Alternative %d",my_hops);
					send_broadcast(str);
				}
				else {
					uint16_t value;
					int counter = 0;
					value = concatenate(70, value1); 
					printf("Static sending announcement %d -> %d  \n",counter,value);
					announcement_set_value(&example_announcement, value);
					announcement_bump(&example_announcement);
					if (DEBUGFM)
						printf("Stop sending packets (Distributed Fault Detection)\n"); 
					if (DEBUGFM)
						printf("No Alternative path (Distributed Fault Detection)\n"); 
					connection = false;
				}
			}
		}
		else if (choice == 71){ 
			 if (connection == false){
				my_hops = value1+1;
				uint16_t value;
				int counter = 0;
				while(counter <20){
					value = concatenate(71, my_hops); 
					printf("Static sending announcement %d -> %d  \n",counter,value);
					announcement_set_value(&example_announcement, value);
					announcement_bump(&example_announcement);
					announcement_listen(1);
					counter++;
				}
				if (DEBUGFM)
					printf("Start sending packets (Distributed Fault Detection)\n"); 
				connection = true;
				noUpperNodes = false;
				displayNeighborTable();
			}
			for(e = list_head(neighbor_table); e!=NULL; e=e->next)
				if (e->addr.u8[0] == from->u8[0] )
					(e->hops) = value1;
		}
        } //telioni to if gia ti strlen 
    } //telioni to arxiko else!
}

/*---------------------------------------------------------------------------*/
/*  
 * 
 * BROADCAST FUNCTIONS
 *
 */
 /*---------------------------------------------------------------------------*/
void sendAliveMessage(){
	packetbuf_copyfrom("Alive", 5);
	printf("broadcast message sent: Alive\n");
    	broadcast_send(&broadcast);
}

void send_broadcast(char *s){
	packetbuf_copyfrom(s, strlen(s)+1);
	broadcast_send(&broadcast);
	if (DEBUGFM)
		printf("Broadcast message sent: %s \n", s);
}

static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from){
	printf("broadcast message received from %d.%d: '%s'\n",from->u8[0], from->u8[1], (char *)packetbuf_dataptr());
	char str[50];
	strcpy(str,(char *)packetbuf_dataptr());
	if (strstr(str, "Alive") != NULL || strstr(str, "Mobile") != NULL){
		struct table_entry_neighbor *e;
		for(e = list_head(neighbor_table); e!=NULL; e=e->next)
			if ((e->addr.u8[0]) == from->u8[0]){
				e->last_clock = clock_seconds();
			}
	}
	else if (strstr(str, "Discovery") != NULL){
		if (DEBUGFM)
			printf("Discovery request received from %d (Distributed Fault Detection)\n",from->u8[0]); 
		if (noUpperNodes)	
			send_unicast("Discovered 1",from->u8[0]);
		else 
			send_unicast("Discovered 0",from->u8[0]);
	}
	else if (strstr(str, "Failure ") != NULL){
		if (connection && linkaddr_node_addr.u8[0]>6 ){
				char *token,*tok;
				token = strtok((char *)str,"Failure ");
				tok = strtok(token,",");
				int node = atoi(tok);
				tok = strtok(NULL,",");
				int level =  atoi(tok);
				printf("Received node : %d level: %d \n",node,level); 
				deleteNeighbor(node);
				struct table_entry_neighbor *e;
				bool recourse = false;
				for(e = list_head(neighbor_table); e!=NULL; e=e->next)
					if( (e->hops) < level)
						recourse = true;				
				if (my_hops >= level && recourse==false){ 
					char str[100];
					int t;
					for(t=0;t<10;t++){
						sprintf(str,"Failure %d,%d",node,level);
						send_broadcast(str);
					}
					if (DEBUGFM)
						printf("Stop sending packets (Distributed Fault Detection)\n"); 
					connection = false; 
				}
			}
	}
	else if (strstr(str, "Alternative") != NULL){
		char *token;
		token = strtok((char *)str,"Alternative");
		int level = atoi(token);
		struct table_entry_neighbor *e;
		for(e = list_head(neighbor_table); e!=NULL; e=e->next)
			if ((e->addr.u8[0]) == from->u8[0])
				e->hops = level;
		if ( !connection || my_hops > level){
			//my_hops = level+1;
			my_hops = level+1;
			char str[100];
			sprintf(str,"Alternative %d",my_hops);
			send_broadcast(str);
			if (DEBUGFM)
					printf("Start sending packets (Distributed Fault Detection)\n"); 
			connection = true;
		}
	}
	
}

/*---------------------------------------------------------------------------*/
/*  [Added by Naoum] 
 * 
 * UNICAST FUNCTIONS 
 *
*/
/*---------------------------------------------------------------------------*/
void send_unicast(char *str, int from){
	packetbuf_copyfrom(str, strlen(str)+1);	
	linkaddr_t mobile;
	mobile.u8[0] = from;
	mobile.u8[1] = 0; 
	unicast_send(&uc, &mobile);
	if (DEBUGFM)
		printf("Unicast message sent: %s \n", str);
}

static void recv_uc(struct unicast_conn *c, const linkaddr_t *from){
	printf("unicast message received from %d.%d\n", from->u8[0], from->u8[1]);
	char str[50];
	strcpy(str,(char *)packetbuf_dataptr());
	if (strstr(str, "Alive") != NULL){
		struct table_entry_neighbor *e;
		for(e = list_head(neighbor_table); e!=NULL; e=e->next)
			if ((e->addr.u8[0]) == from->u8[0])
				e->last_clock = clock_seconds();
	}
}

/*---------------------------------------------------------------------------*/
/*  [Added by Naoum] 
 * 
 * DESTROY MECHANISM 
 *
*/
/*---------------------------------------------------------------------------*/
static bool faultyNode(int node){
	int limit = 120;
	// other scenarios 140
	// SCENARIO 0 - THERE IS NO FAULT
	if (SCENARIO==0)
		return false;
	// SCENARIO 1 - RANDOM FAULTS, THE CHANCE A NODE TO BE DESTROYED IS 1000/32700 - 0.03%
	else if (SCENARIO==11){
		int r = (int) abs(random_rand());
		printf("random = %d \n",r);
		if (( r<200 && clock_seconds() >= limit) ){
			printf("Node %d is destroyed (Destroy Mechanism)\n",node);
			faulty=true;
			return true;
		}
	}
	// FIXED SCENARIOS 
	else if (SCENARIO==12){
		if ((node == 7 || node == 12 )  && clock_seconds() >= limit){
			printf("Node %d is destroyed (Destroy Mechanism) \n",node);
			faulty=true;
			return true;
		}
	}
	else if (SCENARIO==1){
		if ((node == 4 || node == 5 || node == 6 || node == 7 || node == 8 )  && clock_seconds() >= limit){
			printf("Node %d is destroyed (Destroy Mechanism) \n",node);
			faulty=true;
			return true;
		}
	}
	else if (SCENARIO==2){
		if ((node == 7 || node == 8 || node == 9  ) && clock_seconds() >= limit){
			printf("Node %d is destroyed (Destroy Mechanism) \n",node);
			faulty=true;
			return true;
		}
	}
	else if (SCENARIO==3){
		if ((node == 5 || node == 6 || node == 7 || node == 4 ) && clock_seconds() >= limit){
			printf("Node %d is destroyed (Destroy Mechanism)\n",node);
			faulty=true;
			return true;
		}
	}
	else if (SCENARIO==4){
		if ((node == 4 || node == 6 || node == 7 || node == 8 || node == 9  ) && clock_seconds() >= limit){
			printf("Node %d is destroyed (Destroy Mechanism) \n",node);
			faulty=true;
			return true;
		}
	}
	else if (SCENARIO==5){
		if ((node == 4 ||node == 5 || node == 6 || node == 7 || node == 8 || node == 9  ) && clock_seconds() >= limit){
			printf("Node %d is destroyed (Destroy Mechanism) \n",node);
			faulty=true;
			return true;
		}
	}
	else if (SCENARIO==7){
		if ((node == 4 ||node == 5 || node == 6 ) && clock_seconds() >= limit){
			printf("Node %d is destroyed (Destroy Mechanism) \n",node);
			faulty=true;
			return true;
		}
	}
	else if (SCENARIO==8){
		if ((node == 7 || node == 8 ) && clock_seconds() >= limit){
			printf("Node %d is destroyed (Destroy Mechanism) \n",node);
			faulty=true;
			return true;
		}
	}
	else if (SCENARIO==9){
		if ((node == 7 || node == 8|| node == 9 ) && clock_seconds() >= limit){
			printf("Node %d is destroyed (Destroy Mechanism) \n",node);
			faulty=true;
			return true;
		}
	}
	else if (SCENARIO==10){
		if ((node == 8  ) && clock_seconds() >= 350){
			printf("Node %d is destroyed (Destroy Mechanism) \n",node);
			faulty=true;
			return true;
		}
		/*
		else if ((node == 11  ) && clock_seconds() >= 160){
			printf("Node %d is destroyed (Destroy Mechanism) \n",node);
			faulty=true;
			return true;
		}*/
		else if ((node == 11  ) && clock_seconds() >= 390){
			printf("Node %d is destroyed (Destroy Mechanism) \n",node);
			faulty=true;
			return true;
		}
		else if ((node == 3 ) && clock_seconds() >= 800){
			printf("Node %d is destroyed (Destroy Mechanism) \n",node);
			faulty=true;
			return true;
		}
	}
	return false;
}

/*---------------------------------------------------------------------------*/
/*  [Added by Naoum] 
 * 
 * DISTRIBUTED FAULT DETECTION MECHANISM 
 *
*/
/*---------------------------------------------------------------------------*/
static bool isFaultyNode(int node){
	int pos=0;
	struct faulty_entry *current;
    	for(current=list_head(faulty_table); current!=NULL; current=current->next, pos++)
		if ((current->faultyID) == node)
			return true;
	return false;
}

/*
static int countDownNodes(){
	int counter = 0;
	struct table_entry_neighbor *current;
	for(current = list_head(neighbor_table); current != NULL; current=current->next) 
		if ( (current->hops) < my_hops && isFaultyNode(current->addr.u8[0]) == false)		
			counter++;
	return counter;
}

static bool anyCritical(){
	struct table_entry_neighbor *current;
	for(current = list_head(neighbor_table); current != NULL; current=current->next) 
		if ( current->addr.u8[0]>1  && (current->hops) > my_hops  && (current->isCritical) && !(current->haveAlternative))		
			return true;
	return false;
}
*/

static void checkUpperNodes(int faulty){
	int counter = 0;
	struct table_entry_neighbor *current;
	for(current = list_head(neighbor_table); current != NULL; current=current->next) 
		if ((current->addr.u8[0])>1 && (current->hops) < my_hops && !isFaultyNode(current->addr.u8[0]))		
			counter++;
	if (counter == 0 && linkaddr_node_addr.u8[0]>6){
		noUpperNodes = true; 
		connection = false;
		char str[100];
		int t;
		for (t=0;t<10;t++){
			sprintf(str,"Failure %d,%d",faulty,my_hops);
			send_broadcast(str);
		}
		if (DEBUGFM)
				printf("Stop sending packets (Distributed Fault Detection)\n"); 
	}
	if (DEBUGFM && noUpperNodes)
		printf("There is no upper node! (Distributed Fault Detection Mechanism)\n");
}

struct table_entry_neighbor* neighborThreshold(){
    struct table_entry_neighbor *current;
    for(current=list_head(neighbor_table); current!=NULL; current=current->next) {
	int suspicious =(current->addr.u8[0]);
	if (suspicious==1 || isFaultyNode(suspicious) || current->hops == my_hops || suspicious>NUM_STATIC)	
		continue;
	if ((clock_seconds()- current->last_clock) > timelimit_neighbor)
			return current;
    }
    return NULL;
}

void resetThreshold(){
    struct table_entry_neighbor *current;
    for(current=list_head(neighbor_table); current!=NULL; current=current->next) 
	current->last_clock = clock_seconds();
}

static void faultdetection(){
	if (suspiciousFaultyFlag){
		if ((clock_seconds() - suspiciousFaulty->last_clock) > timelimit_neighbor){
			printf("Node %d is NOT RESPONDING (Distributed Fault Detection Mechanism)\n", faultyID);
			struct faulty_entry *f = memb_alloc(&neighbor_mem);
			f->faultyID = faultyID;
			list_add(faulty_table, f);
			if (suspiciousFaulty->hops > my_hops)
				failuremsg(faultyID, suspiciousFaulty->hops);
			else 
				checkUpperNodes(faultyID);
			deleteNeighbor(faultyID);
		}
		suspiciousFaultyFlag = false;
		suspiciousFaulty = NULL;
	}
	else {
		suspiciousFaulty = neighborThreshold();
		if (suspiciousFaulty != NULL &&  !isFaultyNode(suspiciousFaulty->addr.u8[0])){
			suspiciousFaultyFlag = true;
			faultyID = suspiciousFaulty->addr.u8[0];
			if (DEBUGFM)
				printf("Node %d level %d is suspicious faulty node (Distributed Fault Detection Mechanism)\n", faultyID,suspiciousFaulty->hops);
			//communicate with the suspicious faulty node
			int suspicious = suspiciousFaulty->addr.u8[0];
			send_unicast("Alive",suspicious);
		}
	}
}
/*---------------------------------------------------------------------------*/
/*  [Added by Naoum] 
 * 
 * FAULT REPORTING MECHANISM
 *
 */
/*---------------------------------------------------------------------------*/
//send a packet to the sink to inform it about the failure - Faulty Notification Message
void failuremsg(int faultyID, int level){
	struct table_entry_neighbor *current;
	for(current = list_head(neighbor_table); current != NULL; current=current->next) 
		if ( CRITICAL && (current->addr.u8[0])==faultyID && (current->isCritical == false )){		
			printf("Message to the sink abord %d\n",faultyID);
			return;
		}
	char str[100];
	sprintf(str,"FNM %d,%d",faultyID,level);
	packetbuf_copyfrom(str, strlen(str)+1);					
	multihop_send(&multihop, &sink_node);
	if (DEBUGFM)
		printf("Message to the sink about the suspicious faulty node: %s sent (Fault Reporting Mechanism)\n",str);	
}

/*---------------------------------------------------------------------------*/
static const struct multihop_callbacks multihop_call = {recv, forward};
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static const struct unicast_callbacks unicast_callbacks = {recv_uc};
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_multihop_process, ev, data){
	PROCESS_EXITHANDLER(multihop_close(&multihop);)
	PROCESS_EXITHANDLER(announcement_remove(&example_announcement);)
	PROCESS_BEGIN();
	/* Initialize the memory for the neighbor table entries. */
	memb_init(&neighbor_mem);
	memb_init(&faulty_mem);
	/* Initialize the list used for the tables. */
	list_init(neighbor_table);
	list_init(faulty_table);
	printf("PROCESS MULTIHOP BEGINS\n");
	/* Open a multihop/broadcast/unicast connection on Rime channel CHANNEL. */
	multihop_open(&multihop, CHANNEL, &multihop_call);
	printf("Multihop connection opened\n");
	announcement_register(&example_announcement,130, received_announcement); //131
	printf("Announcement connection registered\n");
	sink_node.u8[0] = 1;
	sink_node.u8[1] = 0;
	random_init(linkaddr_node_addr.u8[0]+RANDOM_RELAY);
	//send_announcement_char(72,0);
	/* Allow some time for the network to settle. */
	etimer_set(&et, 60 * CLOCK_SECOND);
	PROCESS_WAIT_UNTIL(etimer_expired(&et));
	initialization = true;
	etimer_set(&et, random_rand() % (CLOCK_SECOND *15));
	PROCESS_WAIT_UNTIL(etimer_expired(&et));
	if (DEBUGFM)
			printf("My hops: %d\n",my_hops);
	/* Loop forever, send a packet. */
	while(1) {
		//FAULTY MECHANISM
		if (faultyNode(linkaddr_node_addr.u8[0])){
			printf("DEAD&%d\n",linkaddr_node_addr.u8[0] );
			break; // for loop
		}
		etimer_set(&et, CLOCK_SECOND *20);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
	}
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/*  [Added by Andreas Naoum] 
 * 
 * Process Energy
 * Calculate consumed energy every 20 seconds 
 *
*/
PROCESS_THREAD(energy, ev, data){
	PROCESS_BEGIN();
	printf("PROCESS ENERGY BEGINS\n");
	while(1){
		if (faulty)
			break; // for loop
		remaining_energy = (energest_type_time(ENERGEST_TYPE_TRANSMIT) * 19.5 + energest_type_time(ENERGEST_TYPE_LISTEN) *21.8 + energest_type_time(ENERGEST_TYPE_CPU) * 1.8 + energest_type_time(ENERGEST_TYPE_LPM) * 0.0545 ) * 3 / 4096 *8;
       	printf("Time: \t %lu \t Remain energy \t %ld.%03u\n",clock_time(),(long)remaining_energy,(unsigned)((remaining_energy-floor(remaining_energy))*1000));
		etimer_set(&timer_energy, CLOCK_SECOND*20);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer_energy));
	}
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/*  [Added by Andreas Naoum] 
 * 
 * Porcess Alive
*  Process to send alive messages to its neighbors
 *
 */
PROCESS_THREAD(alive, ev, data){
	PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
	PROCESS_BEGIN();
	printf("PROCESS ALIVE BEGINS\n");
	broadcast_open(&broadcast, 129, &broadcast_call); //129
	printf("Broadcast conncetion registered\n");
	etimer_set(&timer_alive, random_rand() % (CLOCK_SECOND *10));
	PROCESS_WAIT_UNTIL(etimer_expired(&timer_alive));
	while(1){
		if (faulty)
			break; // for loop
		if (connection)
			sendAliveMessage();
		etimer_set(&timer_alive, CLOCK_SECOND*13);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer_alive));
	}
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/*  [Added by Andreas Naoum] 
 * 
 * Distributed Fault Detection Process
 * Detect any faulty neighbor 
 *
 */
PROCESS_THREAD(fault_detection, ev, data){
	PROCESS_EXITHANDLER(unicast_close(&uc);)
	PROCESS_BEGIN();
	printf("PROCESS FAULT DETECTION BEGINS\n");
	unicast_open(&uc, 146, &unicast_callbacks);//146
	printf("Unicast conncetion registered\n");
	etimer_set(&timer_fault, random_rand() % (CLOCK_SECOND *15));
	PROCESS_WAIT_UNTIL(etimer_expired(&timer_fault));
	etimer_set(&timer_fault, CLOCK_SECOND*65);
	PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer_fault));
	while(1){
		if (faulty)
			break; // for loop
		// && clock_seconds()<=800
		if (connection )
			faultdetection();
		etimer_set(&timer_fault, CLOCK_SECOND*30);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer_fault));
	}
	PROCESS_END();
}
