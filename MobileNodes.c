/*
MOBILE NODES WILL BE FOLLOW MOBILECC ALGORITHMS WHEN IN HARD STAGE TO CALL A NEW MOBILE NODE!
*/
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

#define DISCOVERY_STRATEGY 0

#define RANDOM_MOBILE 0

// variables initialization 

//node's level 
int my_hops=UNKNOWN_HOPS;

//best neighbor's level
int best_hop=UNKNOWN_HOPS;

// packets received
int number_of_packets_received = 0;
int number_of_packets_received_bymyself = 0;

// packets forwarded
int number_of_packets_forwarded = 0;

// announcements received
bool total_received_announcement = false;

// the amount of information at the moment
int buffer_occupancy;

// node's remaining energy
double remaining_energy=0.0;
double remaining_energy1=0.0;
int energy_of_node_for_announcements = 0;

// flags
bool flagFirstTime = true;
bool flagActive = false;
bool flagSendingBecomeidlePacket = false;
bool flagSendingPackets = false;
bool flagComingBackPacket = false;
bool flagSendingshouldbeReplacePacket = false;
bool multipleFailures = false;

// node's starting location
float init_pos_x;
float init_pos_y; 

// node's new location
float new_pos_x;
float new_pos_y;

// node's current location
float current_pos_x;
float current_pos_y;

int discoveryCode = 0;
bool discoveryFlag = false;
bool movingFlag = false;

int moving_delay=0;
int loop = 0;

float pos_x, pos_y;
bool idleStatus = true;
double total_distance=0.0;
double total_energy;
int counter=1;

int hops_cur;

//packets_received_by_sink => how many packets received by the sink
int packets_received_by_sink = 0;
int total_sensor_packets = 0;

//packets_received_two_hop => how many packets received by the sink
int packets_received_one_hop2 = 0;

int mobile_neighbors =-1;
int t;

char type[20];
static struct announcement example_announcement;
struct packetqueue *pqueue;

// TIMERS
static struct etimer et,timer_alive,timer_energy;

linkaddr_t sink_node;

int an;

// node's neighbor table
LIST(neighbor_table);
LIST(discovery_table);

MEMB(neighbor_mem, struct table_entry_neighbor, MAX_NEIGHBORS);
MEMB(discovery_mem, struct discovery_entry, MAX_NEIGHBORS*2);
/*---------------------------------------------------------------------------*/
PROCESS(example_multihop_process, "WSP Routing");
PROCESS(alive, "Alive");
PROCESS(energy, "Energy");
AUTOSTART_PROCESSES(&example_multihop_process,&energy);
//,&alive
static struct multihop_conn multihop;
static struct broadcast_conn broadcast;
static struct unicast_conn uc;
/*---------------------------------------------------------------------------*/
static void displayNeighborTable();
static void displayDiscocveryTable();
static int countNeighborTable();
int found_active_neighbors();
static bool alreadyNeighbor(int id);
static int levelNeighbor(int id);
static void send_announcement_char(int character, unsigned value);
static void changeLevelNeighbor(int id,int hops);
double distancefromtwopoints(float x1, float y1, float x2, float y2);
double calcualte_total_move_energy();
void addNewNeighbor(int hops, int rssi,  linkaddr_t *from, bool mobile);
double calculate_total_remain_energy();
static int getBestNeighborHopNum();
uint16_t concatenate(unsigned x, unsigned y);
int isfromNeighborTable(int nid);
bool findDiscovery(int id);
void addDiscovery(int id, int upper);
static void received_announcement(struct announcement *a,const linkaddr_t *from,uint16_t id, uint16_t value);
static linkaddr_t * find_path();
static linkaddr_t *forward(struct multihop_conn *c,const linkaddr_t *originator, const linkaddr_t *dest,const linkaddr_t *prevhop, uint8_t hops);
void delete_neighborTable();
static void investigationNotificationMessage();
static void discoveryNotificationMessage(bool multihop_flag);
void delete_discoverylist();
static bool parse_sign(const char **const s);
static double parse_digits(const char **const s, int *const count);
double extended_atof(const char *s);
static void recv(struct multihop_conn *c, const linkaddr_t *sender, const linkaddr_t *prevhop, uint8_t hops);
void send_unicast(char *str,const linkaddr_t *from);
static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from);
void send_broadcast(char *s);
static void send_announcement(int value);
void sendPeriodicMessage();
static bool checkMultipleFailures();
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

static void displayDiscocveryTable(){
	int pos = 0;
	struct discovery_entry *current_entry;
	printf("- displayDiscoveryTable MN Neighbor>");
	for(current_entry=list_head(discovery_table); current_entry!=NULL; current_entry=current_entry->next, pos++) {
		printf("t[%d] = %d upper : %d || \n", pos, current_entry->id, current_entry->noUpperNodes);
	}
	printf("\n");
}

static int countNeighborTable(){
	int pos = 0;
	struct table_entry_neighbor *current_entry;
	for(current_entry=list_head(neighbor_table); current_entry!=NULL; current_entry=current_entry->next, pos++);
	return pos;
}
/*---------------------------------------------------------------------------*/
int found_active_neighbors(){
	int count=0;
	struct table_entry_neighbor *current;
	
	for(current = list_head(neighbor_table); current!=NULL; current=current->next) {
		if(current->num_of_packets_received >0){
			count++;
		}
	}
	return count;
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

static void send_announcement_char(int character, unsigned value){
	if (movingFlag)
		return;
	uint16_t val = concatenate(character,value);
	announcement_set_value(&example_announcement, val);
	announcement_bump(&example_announcement);
}

static void changeLevelNeighbor(int id,int hops){
	struct table_entry_neighbor *e;
	for(e = list_head(neighbor_table); e!=NULL; e=e->next)
		if(id == e->addr.u8[0])
				e->hops = hops;
}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
double distancefromtwopoints(float x1, float y1, float x2, float y2){
    double result=0.0,result1=0.0,result2=0.0,result3=0.0;
    result1 = pow(abs((x1-x2)),2);
    result2 = pow(abs((y1-y2)),2);
    result3 = result1+result2;
    result = sqrt(result3);
    return result;
}
/*---------------------------------------------------------------------------*/
double calcualte_total_move_energy(){
	double energy = (33.9 * total_distance)/SPEED;;
	return energy;
}

void addNewNeighbor(int hops, int rssi,  linkaddr_t *from, bool mobile){
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
		list_add(neighbor_table, e);
	}
}
/*---------------------------------------------------------------------------*/
double calculate_total_remain_energy(){
	double listen_remaining_energy = (energest_type_time(ENERGEST_TYPE_TRANSMIT) * 19.5 + energest_type_time(ENERGEST_TYPE_LISTEN) *21.8 + energest_type_time(ENERGEST_TYPE_CPU) * 1.8 + energest_type_time(ENERGEST_TYPE_LPM) * 0.0545 ) * 3 / 4096 *8;
	double move_energy = calcualte_total_move_energy();
	double total_energy = (listen_remaining_energy + move_energy);
	//total_energy = MAX_ENERGY - total_energy;
	return total_energy;
 }
/*---------------------------------------------------------------------------*/
static int getBestNeighborHopNum(){
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
		printf("bh: %d   ceh: %d\n",best_hop,current_entry->hops);
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
/*---------------------------------------------------------------------------*/
uint16_t concatenate(unsigned x, unsigned y){
    unsigned pow = 10;
    while(y >= pow)
        pow *= 10;
    uint16_t res = x * pow + y;
    return res;        
}
/*---------------------------------------------------------------------------
  ---------------------------------------------------------------------------*/
int isfromNeighborTable(int nid){
	struct table_entry_neighbor *e;
	for(e = list_head(neighbor_table); e!=NULL; e=e->next){
		if(nid == e->addr.u8[0]){
			if(e->hops > my_hops){
				return true;
			}
		}
	}
	
	return false;
}
/*---------------------------------------------------------------------------*/
bool findDiscovery(int id){
	struct discovery_entry *d;
	for(d = list_head(discovery_table); d!=NULL; d=d->next)
		if(id == d->id)
			return true;
	return false;
}

void addDiscovery(int id, int upper){
	bool noUpperNodes;
	if (upper==0)
		noUpperNodes = false;
	else
		noUpperNodes = true;
	struct discovery_entry *d = memb_alloc(&discovery_mem);
    	d->id = id;
	d->noUpperNodes = noUpperNodes;
   	list_add(discovery_table, d);
}
/*---------------------------------------------------------------------------*/
static void received_announcement(struct announcement *a,const linkaddr_t *from,uint16_t id, uint16_t value){

	if (movingFlag == true)
			return;


	struct table_entry_neighbor *e;
	if(DEBUG1){
		printf("- Sensor %d.%d:Got announcement from sensor %d value %d \n", linkaddr_node_addr.u8[0],linkaddr_node_addr.u8[1],from->u8[0],value);
		printf("flagActive: %d & flagFirstTime:%d\n",flagActive,flagFirstTime);
	}
	
	//only when I am active I will notice the announcements!
	if(flagActive == true){
		
		if((int) value == -15){
			printf("RECEIVE -15 \n");
			int c = 0;
			while(c <10 && my_hops > 0 && !movingFlag){
				uint16_t value1 = concatenate(80, my_hops); 
				printf("Mobile sending announcement %d -> %d  \n",c,value1);
				announcement_set_value(&example_announcement, value1);
				announcement_bump(&example_announcement);
				c++;
			}	
		}
		else if((int) value == 78){ //response from my neighbors in range!
			for(e = list_head(neighbor_table); e!=NULL; e=e->next){
				if(linkaddr_cmp(from, &e->addr)){
					uint16_t value = concatenate(78,e->addr.u8[0]);//'N+nodeID'
					announcement_set_value(&example_announcement, value);
					announcement_bump(&example_announcement);
					break;
				}
			}
		}
		else{ //to value periexi kapoio grama stin arxi!        
			char s[11]; 
			sprintf(s,"%d", value); 
			//printf("%d length of value\n",strlen(s));
			if(strlen(s)!=2){
				int choice = concatenate(s[0] - '0', s[1] - '0');
				int v=s[2] - '0';
				//printf("  %d\n",v);
				int i=3;
				while(s[i]!='\0'){
					v=concatenate(v, s[i] - '0');
					i++;
				}
				//create the new value without the two first numbers!
				value = v;
				if(choice == 72){


						printf("Initialization -> received hops: %d from %d \n",value,from->u8[0]); 
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
				
								if (e->hops == UNKNOWN_HOPS && value>=1) { 
									e->rssi = rssi;
									e->hops = value + 1;
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
								if ( value >= 1 ) 
									 new_hops = value + 1;
								else 
									 new_hops = UNKNOWN_HOPS;
						} 
						else 
							 new_hops = hops;

						linkaddr_t *node = NULL;
						linkaddr_copy(node, from);
						addNewNeighbor(new_hops,rssi,node,false);

						best_hop = getBestNeighborHopNum();

						if(best_hop>my_hops){
							my_hops = best_hop+1;
							uint16_t value = concatenate(72,getBestNeighborHopNum());//'H'
							announcement_set_value(&example_announcement, value);
							announcement_bump(&example_announcement);
						}
			}
				else if(choice == 68){
					bool found = findDiscovery(from->u8[0]);
					if (found == false)
						addDiscovery(from->u8[0],value);
				}
				else if(choice == 69){ //energy
					if(DEBUG){
						printf("Energy announcement is received!!!\n");
					}
					for(e = list_head(neighbor_table); e!=NULL; e=e->next){
						if(linkaddr_cmp(from, &e->addr)){
							if(e->remaining_energy < value){
								e->remaining_energy = value;
								break;
							}
							else if(e->remaining_energy > value && e->remaining_energy>999){
								e->remaining_energy = e->remaining_energy+value;
								break;
							}
						}
					}
					if(DEBUG){
						printf("Update list after energy announcement\n"); 
						displayNeighborTable();
					}
				}
				else if(choice == 66){
					//buffer!
				}
				else if(choice == 83){ // soft stage
					if(DEBUG){
						printf(" Linkaddr_node: %d value %d\n", linkaddr_node_addr.u8[0],value);
					}   
					//printf("Value is not false -> soft stage announcement\n");
					//soft stage
					for(e = list_head(neighbor_table); e!=NULL; e=e->next){
						if(linkaddr_cmp(from, &e->addr)){
							e->next_sequence_packet_number = false;
							break;
						}
					}
					if(DEBUG){
						printf("Update list after soft stage announcement\n"); 
						displayNeighborTable();
					}
				}
				else if(choice == 77){ //MOBILE HOPShops!
					int hops = 0; //arxiki timi
					int third_case = 0;
					// WSP: Find RSSI using packetbuf_attr(PACKETBUF_ATTR_RSSI)
					//  => find it from example-neighbor.c
					uint16_t rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI)-45;
					// Set sink info
					linkaddr_t sink;
					sink.u8[0] =1;
					sink.u8[1] = 0; 
					// 1st case > an o apostoleas einai o sink tote apexei o apo ton eauto t ane3artita apo ton apostolea
					if (linkaddr_cmp(&linkaddr_node_addr, &sink)){
						hops = 0;
					}// 2nd case > an o apostoleas einai o sink tote kanoume to hops = 1 dixnontas pws 
					//  o current sensor apexei mono 1 hops apo to sink
					else if(linkaddr_cmp(from, &sink)){
						hops = 1;
					} 
					// 3rd case
					else
						third_case = 1;
					/* We received an announcement from a neighbor so we need to update
					 the neighbor list, or add a new entry to the table. */
					for(e = list_head(neighbor_table); e != NULL; e = e->next) {
						if(linkaddr_cmp(from, &e->addr)) {
							// o geitonas(from) den ixere poso apexei apo ton sink 
							//  -> kai efoson twra xerei (value>=1)
							if ((e->hops == UNKNOWN_HOPS && value>=1) || (e->hops > value+1)) { 
								e->rssi = rssi;
								e->hops = value + 1;
							}
							if(DEBUG){
								printf("Update list\n"); 
								displayNeighborTable();
							}
					  return;
					}
				  }
				  /* The neighbor was not found in the list, so we add a new entry by
					 allocating memory from the neighbor_mem pool, fill in the
					 necessary fields, and add it to the list. */
					e = memb_alloc(&neighbor_mem);
					if(e != NULL) {
						e->rssi = rssi;
						//3rd case
						if (third_case == 1) {
							//knows the distance from the Sink
							if ( value >= 1 ) { //was value!=0 but announcement_set_value
								e->hops = value + 1;
							}
							//does not know the distance from the Sink
							else {
								e->hops = UNKNOWN_HOPS;
							}
						} 
						//1st & 2nd case
						else {
							e->hops = hops;
						}
						linkaddr_copy(&e->addr, from);
						e->next_sequence_packet_number=true;
						e->num_of_packets_received = 0;
						e->buffer_occupancy = 0;
						e->remaining_energy = 0.0;
						e->last_clock = clock_seconds();
						e->mobileFlag = true;
						e->isNextHop = false;
						list_add(neighbor_table, e);
						if(DEBUG) {
							printf("Insert in the list\n");
							displayNeighborTable();
						}
					}
					announcement_set_value(&example_announcement, 78);
					announcement_bump(&example_announcement);
				}
				else if(choice == 80){ 
					printf("MN my_hops = %d and sender's hops = %d\n",my_hops, value);
					uint16_t rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI)-45;
					if (alreadyNeighbor(from->u8[0]) == false || ( levelNeighbor(from->u8[0])!=value)){

						//if (levelNeighbor(from->u8[0])!=value)

						if (alreadyNeighbor(from->u8[0]) == false){
							linkaddr_t *node = NULL;
							linkaddr_copy(node, from);
							addNewNeighbor(value, rssi, node, true);
						}
						else
							changeLevelNeighbor(from->u8[0],value);

						printf("ADDED NEIGHBOR %d \n",from->u8[0]); 
						// int prev_hops = my_hops;
						int best_hop = getBestNeighborHopNum();

						if (best_hop+1 < my_hops && best_hop!=0)
							my_hops=best_hop+1;

						if (my_hops==0 && value>0)
							my_hops= value+1;

						printf("MN NEW HOPS %d \n",my_hops); 
						
							for (t =0; t<3;t++){
								uint16_t val = concatenate(80,my_hops);
								announcement_set_value(&example_announcement, val);
								announcement_bump(&example_announcement);
								announcement_listen(1);
							}
						
					}
				}
				else if(choice == 81){
					printf("MN 81 my_hops = %d and sender's hops = %d\n",my_hops, value);
					uint16_t rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI)-45;
					if (alreadyNeighbor(from->u8[0]) == false || ( levelNeighbor(from->u8[0])!=value)){
						linkaddr_t *node = NULL;
						linkaddr_copy(node, from);
						addNewNeighbor(value, rssi, node, false);
						printf("ADDED NEIGHBOR %d \n",from->u8[0]); 
						int best_hop = getBestNeighborHopNum();
						if (best_hop+1 < my_hops ){
							my_hops=best_hop+1;
							printf("MN NEW HOPS %d \n",my_hops); 
							uint16_t val = concatenate(80,my_hops);
							announcement_set_value(&example_announcement, val);
							announcement_bump(&example_announcement);
						}
						if(my_hops<0)
							my_hops=0;
						if (my_hops==0 ){
							my_hops= value+1;
							printf("MN NEW HOPS %d \n",my_hops); 
							for (t =0; t<3;t++){
								uint16_t val = concatenate(80,my_hops);
								announcement_set_value(&example_announcement, val);
								announcement_bump(&example_announcement);
								announcement_listen(1);
							}
						}

					}
				}
			} //telioni to if gia ti strlen 
		} //telioni to arxiko else!
		flagFirstTime = false;
	}
	
}
/*********************************************************************/
/*---------------------------------------------------------------------------*/
static linkaddr_t * find_path(){

	displayNeighborTable();

    struct table_entry_neighbor *current, *best = NULL;

    for(current = list_head(neighbor_table); current!=NULL; current=current->next)
		if(current->hops > UNKNOWN_HOPS && current->mobileFlag == true && (current->hops) < my_hops  )
			return &(current->addr);

	int min=1000;
	
    for(current = list_head(neighbor_table); current!=NULL; current=current->next)
		if(current->hops > UNKNOWN_HOPS && current->hops < min ){
			min = current->hops;
			best = current;
		}

	printf("find path is :%d\n", best->addr.u8[0]);

	return &(best->addr);	
}

/*---------------------------------------------------------------------------*/
/*
 * [Modified]
 * This function is called to forward a packet. The function picks the
 * neighbor from the neighbor list that has the minimun hops number and 
 * returns its address. If there are two or more neigbors with the same 
 * hops number returns the one that has the maximum RSSI. The multihop 
 * layer sends the packet to this address. If no neighbor is
 * found, the function returns NULL to signal to the multihop layer
 * that the packet should be dropped.
 */
static linkaddr_t *forward(struct multihop_conn *c,const linkaddr_t *originator, const linkaddr_t *dest,const linkaddr_t *prevhop, uint8_t hops){
			
	if (movingFlag == true)
			return NULL;

	if(flagSendingPackets == true && originator->u8[0] == linkaddr_node_addr.u8[0]){
		/*
		if(DEBUG1){
			printf("%d.%d: 1 Received packet from %d.%d '%s' for %d.%d\n",linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[0],packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[1],(char *)packetbuf_dataptr(),dest->u8[0],dest->u8[1]);
		}	*/
		flagSendingPackets = false;
		linkaddr_t * nexthop = find_path();
		//linkaddr_copy(nexthop, dest);
		printf("%d.%d: Forwarding packet to %d.%d \n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],nexthop->u8[0], nexthop->u8[1]);
		return nexthop;
	}
	else if(flagSendingBecomeidlePacket == true  && originator->u8[0] == linkaddr_node_addr.u8[0]){
		/*
		if(DEBUG1){
			printf("%d.%d: 2 Received packet from %d.%d '%s' for %d.%d\n",linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[0],packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[1],(char *)packetbuf_dataptr(),dest->u8[0],dest->u8[1]);
		}*/
		flagSendingBecomeidlePacket = false;
		linkaddr_t * nexthop = find_path();
        if(DEBUG1){
            if(nexthop!=NULL)
                printf("%d.%d: Forwarding packet to %d.%d \n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],nexthop->u8[0], nexthop->u8[1]);
            else
                printf("%d.%d: Forwarding packet to nowone! \n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
        }  
		return nexthop;
	}
	else if(flagSendingshouldbeReplacePacket == true && originator->u8[0] == linkaddr_node_addr.u8[0] && flagComingBackPacket == false){
		if(DEBUG1){
			printf("%d.%d: 3 Received packet from %d.%d '%s' for %d.%d\n",linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[0],packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[1],(char *)packetbuf_dataptr(),dest->u8[0],dest->u8[1]);
		}
		//flagSendingshouldbeReplacePacket = false;
		linkaddr_t * nexthop = find_path();
        if(DEBUG1){
            if(nexthop!=NULL)
                printf("%d.%d: Forwarding packet to %d.%d \n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],nexthop->u8[0], nexthop->u8[1]);
            else
                printf("%d.%d: Forwarding packet to nowone! \n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
        }  
		return nexthop;
	}
	else if(flagComingBackPacket == true &&  originator->u8[0] == linkaddr_node_addr.u8[0]){
		/*
		if(DEBUG1){
			printf("%d.%d: 4 Received packet from %d.%d '%s' for %d.%d\n",linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[0],packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[1],(char *)packetbuf_dataptr(),dest->u8[0],dest->u8[1]);
		}*/
		//flagComingBackPacket = false;
		printf("flagComingBackPacket = %d \n", flagComingBackPacket);
		linkaddr_t * nexthop = find_path();
        if(DEBUG1){
            if(nexthop!=NULL)
                printf("%d.%d: Forwarding packet to %d.%d \n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],nexthop->u8[0], nexthop->u8[1]);
            else
                printf("%d.%d: Forwarding packet to nowone! \n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
        }  

		return nexthop;
	}
	if(originator->u8[0] != linkaddr_node_addr.u8[0]){
		/*
		if(DEBUG1){
			printf("%d.%d: 1 Received packet from %d.%d '%s' for %d.%d\n",linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[0],packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[1],(char *)packetbuf_dataptr(),dest->u8[0],dest->u8[1]);
		}	*/
		linkaddr_t * nexthop = find_path();
		//linkaddr_copy(nexthop, dest);
		printf("%d.%d: Forwarding packet to %d.%d \n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],nexthop->u8[0], nexthop->u8[1]);
		return nexthop;
	}
	return NULL;
	
}

/*---------------------------------------------------------------------------*/
void delete_neighborTable(){
    while(list_length(neighbor_table)!=0){
        list_remove(neighbor_table,list_tail (neighbor_table));
        memb_free(&neighbor_mem, list_tail (neighbor_table));
    }
}

/*---------------------------------------------------------------------------*/
/*  [Added by Naoum] 
 * 
 * MOBILE NODE INVESTIGATION MECHANISM 
 *
 */
/*---------------------------------------------------------------------------*/

static void investigationNotificationMessage(){
	printf("MN msg to the sink about other suspicious faulty nodes (Mobile Investigation Mechanism)\n");	
	//send a packet to the sink to inform it about the failure
	char str[100];
	char my_neighbors[100];
	int index = 0;
	struct table_entry_neighbor *current=NULL;
	for(current = list_head(neighbor_table); current!=NULL; current=current->next)
		if ((current->next)==NULL)
			index += snprintf(&my_neighbors[index], 100-index, "%d", current->addr.u8[0]);
		else 
			index += snprintf(&my_neighbors[index], 100-index, "%d,", current->addr.u8[0]);
	sprintf(str,"IRNM %s",my_neighbors);
	packetbuf_copyfrom(str, strlen(str)+1);	
	packetbuf_set_attr(PACKETBUF_ATTR_TRANSMIT_TIME, clock_seconds());	
	flagSendingPackets = true;			
	linkaddr_t to;
	to.u8[0] = 1;
	to.u8[1] = 0;
	multihop_send(&multihop, &to);
	if (DEBUGFM)
		printf("MN msg to the sink about other faulty nodes: %s sent (Mobile Investigation Mechanism)\n",str);	
}

/*---------------------------------------------------------------------------*/
/*  [Added by Naoum] 
 * 
 * MOBILE NODE DISCOVERY MECHANISM 
 *
 */
/*---------------------------------------------------------------------------*/

static void discoveryNotificationMessage(bool multihop_flag){
	printf("MN msg to the sink about its findings (Mobile Discovery Mechanism)\n");	
	//send a packet to the sink to inform it about the failure
	char str[100],str2[100];
	char my_neighbors[100];
	int index = 0;
	if  (list_length(discovery_table)==0)
		strcpy(str,"DNM 0");
	else{
		struct discovery_entry *current=NULL;
		for(current = list_head(discovery_table); current != NULL; current=current->next){
			int upper = current->noUpperNodes;
			if ((current->next)==NULL)
				index += snprintf(&my_neighbors[index], 100-index, "%d%d", current->id,upper);
			else 
				index += snprintf(&my_neighbors[index], 100-index, "%d%d,", current->id,upper);
		}
		sprintf(str,"DNM %s",my_neighbors);
	}
	packetbuf_copyfrom(str, strlen(str)+1);	
	packetbuf_set_attr(PACKETBUF_ATTR_TRANSMIT_TIME, clock_seconds());	
	flagSendingPackets = true;		
	strcpy(str2,str);	
	linkaddr_t to;
	to.u8[0] = 1;
	to.u8[1] = 0;
	if (multihop_flag==true)
		multihop_send(&multihop, &to);
	else {
		unicast_send(&uc, &to);
		//clock_wait(5);
		/*
		packetbuf_copyfrom(str2, strlen(str)+1);	
		to.u8[0] = 1;
		to.u8[1] = 0;
		unicast_send(&uc, &to);
		*/
	}
	if (DEBUGFM)
		printf("MN msg to the sink about its findings: %s sent (Mobile Discovery Mechanism)\n",str);	
}

void delete_discoverylist(){
    while(list_length(discovery_table)!=0){
        list_remove(discovery_table,list_tail (discovery_table));
        memb_free(&discovery_mem, list_tail(discovery_table));
    }
}

/*---------------------------------------------------------------------------*/
/* return true for positive, false for negative,
   and advance `*s` to next position */
static bool parse_sign(const char **const s){
    switch (**s) {
    case '-': ++*s; return false;
    case '+': ++*s; return true;
    default: return true;
    }
}
/* return decimal value of digits,
   advancing `*s` to the next character,
   and storing the number of digits read into *count */
static double parse_digits(const char **const s, int *const count){
    double value = 0.0;
    int c = 0;
    while (isdigit(**s)) {
        value = value * 10.0 + (*(*s)++ - '0');
        ++c;
    }
    if (count)
        *count = c;
    return value;
}

double extended_atof(const char *s){
    /*skip white space*/
    while (isspace(*s))
        ++s;
    const bool valuesign = parse_sign(&s); /* sign of the number */
    double value = parse_digits(&s, NULL);
    if (*s == '.') {
        int d;                  /* number of digits in fraction */
        ++s;
        double fraction = parse_digits(&s, &d);
        while (d--)
            fraction /= 10.0;
        value += fraction;
    }
    if (!valuesign)
        value = -value;
    if (tolower(*s++) != 'e')
        return value;
    /* else, we have an exponent; parse its sign and value */
    const double exponentsign = parse_sign(&s) ? 10. : .1;
    int exponent = parse_digits(&s, NULL);
    while (exponent--)
        value *= exponentsign;
    return value;
}


static void recv(struct multihop_conn *c, const linkaddr_t *sender, const linkaddr_t *prevhop, uint8_t hops){
	printf("multihop message received '%s' from %d\n", (char *)packetbuf_dataptr(), sender->u8[0]);
}

void send_unicast(char *str,const linkaddr_t *from){
	packetbuf_copyfrom(str, strlen(str)+1);	
	linkaddr_t mobile;
	mobile.u8[0] = from->u8[0];
	mobile.u8[1] = from->u8[1]; 
	unicast_send(&uc, &mobile);
	if (DEBUGFM)
		printf("Unicast message sent: %s \n", str);
}

void sendAliveMessage(){
	//packetbuf_copyfrom("Mobile", 6);
	char str[50];
	sprintf(str,"Mobile %d",my_hops);
	packetbuf_copyfrom(str, strlen(str)+1);	
	printf("broadcast message sent: Mobile\n");
    	broadcast_send(&broadcast);
}

/*---------------------------------------------------------------------------*/
static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from){

	if(flagActive == false)
		return;

	if (movingFlag)
			return;
  	printf("broadcast message received from %d.%d: '%s'\n",from->u8[0], from->u8[1], (char *)packetbuf_dataptr());
	
	char str[50];
	strcpy(str,(char *)packetbuf_dataptr());

	if (strstr(str, "Alive") != NULL ){
		struct table_entry_neighbor *e;
		for(e = list_head(neighbor_table); e!=NULL; e=e->next)
			if ((e->addr.u8[0]) == from->u8[0])
				e->last_clock = clock_seconds();
	}
	else if (strstr(str, "Mobile") != NULL){

		/*
		struct table_entry_neighbor *e;
		for(e = list_head(neighbor_table); e!=NULL; e=e->next)
			if ((e->addr.u8[0]) == from->u8[0])
				e->last_clock = clock_seconds();
		*/
		return;
		char *token;
		token = strtok((char *)str,"Mobile");
		int level = atoi(token);
		if (!alreadyNeighbor(from->u8[0])){
			uint16_t rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI)-45;
			linkaddr_t *node = NULL;
			linkaddr_copy(node, from);
			addNewNeighbor(level, rssi, node, true);
		}
		else if(alreadyNeighbor(from->u8[0]) &&  levelNeighbor(from->u8[0])!=level)
			changeLevelNeighbor(from->u8[0],level);
	}

}

static void recv_uc(struct unicast_conn *c, const linkaddr_t *from){

	printf("unicast message received from %d.%d\n", from->u8[0], from->u8[1]);

	char str[50];
	strcpy(str,(char *)packetbuf_dataptr());

	if (strstr(str, "Sensor") != NULL){
		if (alreadyNeighbor(from->u8[0]))
			return;
		char *token;
		token = strtok((char *)str,"Sensor");
		int level = atoi(token);
		uint16_t rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI)-45;
		linkaddr_t *node = NULL;
		linkaddr_copy(node, from);
		addNewNeighbor(level, rssi, node, false);
	}
	else if (strstr(str, "Discovered") != NULL){
		char *token;
		token = strtok((char *)str,"Discovered");
		int upper = atoi(token);
		if (findDiscovery(from->u8[0]) == false)
			addDiscovery(from->u8[0],upper);
	}
	
}


/*---------------------------------------------------------------------------*/

void send_broadcast(char *s){
	packetbuf_copyfrom(s, strlen(s));
	broadcast_send(&broadcast);
	if (DEBUGFM)
		printf("Broadcast message sent: %s \n", s);
}

static void send_announcement(int value){
	int val = 0 - value;
	announcement_set_value(&example_announcement, val);
	announcement_bump(&example_announcement);
	if (DEBUGFM)
		printf("Announcement message sent: %d \n", value);
}

void sendPeriodicMessage(){
	packetbuf_copyfrom("Trust me!I am a mobile!", 24);
	packetbuf_set_attr(PACKETBUF_ATTR_TRANSMIT_TIME, clock_seconds());
	if(!linkaddr_cmp(&sink_node, &linkaddr_node_addr)) {
		total_sensor_packets++;
		printf("Sensor %d: %d\n", linkaddr_node_addr.u8[0], total_sensor_packets);
		multihop_send(&multihop, &sink_node);
	}
}

static void currentPosition(){
	int id = linkaddr_node_addr.u8[0];

	
	if (id==25){
		current_pos_x = -36.00;
		current_pos_y = 23.00;
	}
	else if (id==26){
		current_pos_x = -21.00;
		current_pos_y = 27.00;
	}
	else if (id==27){
		current_pos_x = -5.00;
		current_pos_y = 29.00;
	}
	else if (id==28){
		current_pos_x = 10.00;
		current_pos_y = 30.00;
	}
	else if (id==29){
		current_pos_x = 25.00;
		current_pos_y = 25.00;
	}
	else if (id==30){
		current_pos_x = 34.00;
		current_pos_y = 20.00;
	}

/*
	if (id==17){
		current_pos_x = 77.28;
		current_pos_y = -70.90;
	}
	else if (id==18){
		current_pos_x = 81.64;
		current_pos_y = -60.00;
	}
	else if (id==19){
		current_pos_x = 83.08;
		current_pos_y = -49.2;
	}
	else if (id==20){
		current_pos_x = 82.90;
		current_pos_y = -37.1;
	}
	else if (id==21){
		current_pos_x = 81.42;
		current_pos_y = -26.1;
	}
	else if (id==22){
		current_pos_x = -74.53;
		current_pos_y = -15.1;
	}
	*/

	/*
	if (id==17){
		current_pos_x = 15.00;
		current_pos_y = 28.00;
	}
	else if (id==18){
		current_pos_x = 5.0;
		current_pos_y = 30.0;
	}
	else if (id==19){
		current_pos_x = -3.0;
		current_pos_y = 32.0;
	}
	else if (id==20){
		current_pos_x = -14.0;
		current_pos_y = 31.0;
	}
	else if (id==21){
		current_pos_x = -23.0;
		current_pos_y = 31.0;
	}
	else if (id==22){
		current_pos_x = -32.0;
		current_pos_y = 29.0;
	}
	*/

	init_pos_x = current_pos_x;
	init_pos_y = current_pos_y;
}

static bool checkMultipleFailures(){
	int neighbors_found = countNeighborTable();
	if (DEBUGFM)
		printf("MN neighbors found %d  \n",neighbors_found);

	if (neighbors_found<mobile_neighbors ){
		multipleFailures = true;
		if (DEBUGFM)
			printf("MN detect a faulty path! \n");
		return true;
	}
	else {
		send_announcement(14);
		if (DEBUGFM)
			printf("MN does not detect a faulty path! found:%d neighbors:%d \n",neighbors_found ,mobile_neighbors);
		return false;
	}	
}

/*---------------------------------------------------------------------------*/
static const struct multihop_callbacks multihop_call = {recv,forward};
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static const struct unicast_callbacks unicast_callbacks = {recv_uc};
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_multihop_process, ev, data){
	PROCESS_EXITHANDLER(multihop_close(&multihop);)
	//PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
	PROCESS_EXITHANDLER(announcement_remove(&example_announcement);)
	PROCESS_EXITHANDLER(unicast_close(&uc);)

	PROCESS_BEGIN();

	/* Initialize the memory for the neighbor table entries. */
	memb_init(&neighbor_mem);;
	memb_init(&discovery_mem);

	/* Initialize the list used for the neighbor table. */
	list_init(neighbor_table);
	list_init(discovery_table);

	printf("PROCESS MULTIHOP BEGINS\n");

	/* Open a multihop connection on Rime channel CHANNEL. */
	multihop_open(&multihop, CHANNEL, &multihop_call);
	printf("Multihop connection opened\n");
	
	//broadcast_open(&broadcast, 129, &broadcast_call);
	//printf("Broadcast conncetion registered\n");

	announcement_register(&example_announcement,130,received_announcement);
	printf("Announcement connection registered\n");

	unicast_open(&uc, 146, &unicast_callbacks);
	printf("Unicast conncetion registered\n");

	sink_node.u8[0] = 1;
	sink_node.u8[1] = 0;

	/* Allow some time for the network to settle. */
	etimer_set(&et, 60 * CLOCK_SECOND);
	PROCESS_WAIT_UNTIL(etimer_expired(&et));

	currentPosition();

	random_init(linkaddr_node_addr.u8[0]+RANDOM_MOBILE);
	etimer_set(&et, random_rand() % (CLOCK_SECOND *10));
	PROCESS_WAIT_UNTIL(etimer_expired(&et));


/* Loop forever, send a packet. */
 while(1) {

	if(((new_pos_x == current_pos_x && new_pos_y == current_pos_y) || (current_pos_x == init_pos_x && init_pos_y == current_pos_y)) && idleStatus==true){

		if (DEBUGFM)
			printf("MN NEW POSITION \n");

		//get the nodes position via the cooja editor script
		printf("MN waiting new position\n");

		PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message);
		new_pos_x = extended_atof((char *)data);
		movingFlag = true;	
		printf("MN new_pos_x = %ld.%03u\n",(long)new_pos_x,(unsigned)((new_pos_x-floor(new_pos_x))*1000));
		PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message);
		new_pos_y = extended_atof((char *)data);
		printf("MN new_pos_y = %ld.%03u\n",(long)new_pos_y,(unsigned)((new_pos_y-floor(new_pos_y))*1000));
		PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message);
		hops_cur = atoi((char *)data);
		printf("MN hops = %d\n",hops_cur);
		PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message);
		mobile_neighbors = atoi((char *)data);
		printf("MN neighbors = %d\n",mobile_neighbors );
		PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message);
		discoveryCode = atoi((char *)data);
		printf("MN code = %d\n",discoveryCode);
		int dist = distancefromtwopoints(current_pos_x , current_pos_y,new_pos_x, new_pos_y); 
		total_distance = total_distance + dist;
		printf("total distance = %ld.%03u\n",(long)total_distance,(unsigned)((total_distance-floor(total_distance))*1000));
		moving_delay = dist/SPEED;
		printf("Moving to the new position, time estimation: %d seconds, distance: %d\n",moving_delay, dist );
		etimer_set(&et, moving_delay*CLOCK_SECOND);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		printf("Just arrived to the new position\n");
		current_pos_x = new_pos_x;
		current_pos_y = new_pos_y;
		movingFlag = false;
		
		if (discoveryCode == 0){
			printf("Discovery Code 0 - Start \n");
			my_hops = hops_cur;
			flagActive = true;
			send_announcement_char(80,my_hops);
			etimer_set(&et, 5*CLOCK_SECOND);
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
			send_announcement_char(80,my_hops);
			etimer_set(&et, 5*CLOCK_SECOND);
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
			send_announcement_char(80,my_hops);
			etimer_set(&et, 4*CLOCK_SECOND);
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
			/*
			for (t =0; t<7;t++){
				send_announcement_char(80,my_hops);
				etimer_set(&et, 3*CLOCK_SECOND);
				PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
				announcement_listen(1);
			}*/
			if (checkMultipleFailures() && DISCOVERY_STRATEGY==1)
				idleStatus=true;
			else {
				idleStatus=false;
				flagSendingPackets = true;
			}
			if (DEBUGFM)
				displayNeighborTable();
			if (DISCOVERY_STRATEGY == 0)
				send_announcement(14);
			printf("Discovery Code 0 - End \n");
		}
		else if (discoveryCode == 1){
			printf("Discovery Code 1 - Start \n");
			flagActive = true;
			send_announcement(11);
			etimer_set(&et, 5*CLOCK_SECOND);
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
			send_announcement(11);
			etimer_set(&et, 5*CLOCK_SECOND);
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
			send_announcement(11);
			etimer_set(&et, 4*CLOCK_SECOND);
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
			printf("Discovery Code 1 - End \n");
		}
		else if (discoveryCode == 2){
			printf("Discovery Code 2 - Start \n");
			discoveryNotificationMessage(true);
			etimer_set(&et, 14*CLOCK_SECOND);
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
			send_announcement(14);
			flagActive = true;
			idleStatus=false;
			flagSendingPackets = true;
			displayDiscocveryTable();
			delete_discoverylist();
			printf("Discovery Code 2 - End \n");
		}
		else if (discoveryCode == 3){
			printf("Discovery Code 3 - Start \n");
			my_hops = hops_cur;
			if (my_hops < 0)
				my_hops=0;
			flagActive = true;
			for(t=0;t<7;t++){
				send_announcement_char(80,my_hops);
				etimer_set(&et, 3*CLOCK_SECOND);
				PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
			}
			send_announcement(14);
			flagSendingPackets = true;
			idleStatus = false;
			if (DEBUGFM)
				displayNeighborTable();	
			printf("Discovery Code 3 - End \n");
			send_announcement(14);
		}
		else if (discoveryCode == 4){
			printf("Discovery Code 4 - Start \n");
			discoveryNotificationMessage(false);
			displayDiscocveryTable();
			delete_discoverylist();
			flagActive = false;
			idleStatus = true;
			printf("Discovery Code 4 - End \n");
		}
		else if (discoveryCode == 5){
			printf("Discovery Code 5 - Start \n");
			my_hops = hops_cur;
			if (my_hops < 0)
				my_hops=0;
			flagActive = true;
			etimer_set(&et, 14*CLOCK_SECOND);
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
			send_announcement(14);
			flagSendingPackets = true;
			idleStatus = false;
			if (DEBUGFM)
				displayNeighborTable();	
			printf("Discovery Code 5 - End \n");
		}
		else if (discoveryCode == 6){
			printf("Discovery Code 6 - Start \n");
			my_hops = hops_cur;
			// if (my_hops < 0)
			// my_hops=0;
			my_hops=5;
			// my_hops=7;
			flagActive = true;
			send_announcement_char(80,my_hops);
			etimer_set(&et, 5*CLOCK_SECOND);
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
			send_announcement_char(80,my_hops);
			etimer_set(&et, 5*CLOCK_SECOND);
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
			send_announcement_char(80,my_hops);
			etimer_set(&et, 4*CLOCK_SECOND);
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
			/*
			for (t =0; t<7;t++){
				send_announcement_char(80,my_hops);
				etimer_set(&et, 3*CLOCK_SECOND);
				PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
				announcement_listen(1);
			}*/
			send_announcement(14);
			flagSendingPackets = true;
			idleStatus = false;
			if (DEBUGFM)
				displayNeighborTable();	
			printf("Discovery Code 6 - End \n");
		}
	}


	if (multipleFailures == true){
		investigationNotificationMessage();
		etimer_set(&et, 1*CLOCK_SECOND);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		if (DISCOVERY_STRATEGY == 1){
			idleStatus = true;
			flagActive = true;
		}
		multipleFailures=false;
	}

	if(((new_pos_x == current_pos_x && new_pos_y == current_pos_y) && (new_pos_x!=init_pos_x && new_pos_y!=init_pos_y)) && idleStatus==false){
		etimer_set(&et, 20*CLOCK_SECOND);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
	} 
  }
  PROCESS_END();
}

PROCESS_THREAD(alive, ev, data){
	PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
	PROCESS_BEGIN();
	printf("PROCESS ALIVE BEGINS\n");
	broadcast_open(&broadcast, 129, &broadcast_call); //129
	printf("Broadcast conncetion registered\n");
	etimer_set(&timer_alive, random_rand() % (CLOCK_SECOND *15));
	PROCESS_WAIT_UNTIL(etimer_expired(&timer_alive));
	while(1){
		 if(((new_pos_x == current_pos_x && new_pos_y == current_pos_y) && (new_pos_x!=init_pos_x && new_pos_y!=init_pos_y)) && idleStatus==false)
			sendAliveMessage();
		etimer_set(&timer_alive, CLOCK_SECOND*20);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer_alive));
	}
	PROCESS_END();
}

PROCESS_THREAD(energy, ev, data){
	PROCESS_BEGIN();
	printf("PROCESS ENERGY BEGINS\n");
	while(1){
		remaining_energy1 = (energest_type_time(ENERGEST_TYPE_TRANSMIT) * 19.5 + energest_type_time(ENERGEST_TYPE_LISTEN) *21.8 + energest_type_time(ENERGEST_TYPE_CPU) * 1.8 + energest_type_time(ENERGEST_TYPE_LPM) * 0.0545 ) * 3 / 4096 *8;
		total_energy = calculate_total_remain_energy();
		if(((new_pos_x == current_pos_x && new_pos_y == current_pos_y) && (new_pos_x!=init_pos_x && new_pos_y!=init_pos_y)) && idleStatus==false)
			printf("total_energy = %ld.%03u\n",(long)total_energy,(unsigned)((total_energy-floor(total_energy))*1000));
		etimer_set(&timer_energy, CLOCK_SECOND*20);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer_energy));
	}

	PROCESS_END();
}
