/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
 * All rights reserved.
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
 *         in a multi-hop fashion, but does not implement any routing
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
 *   Centralized/Decentralized Fault Detection Mechanism
 *   Fault Reporting Mechanism 
 *   Mobile Node Investigation Mechanism 
 *   Mobile Node Discovery Mechanism
 *   Fault Recovery
 *
 */
#include "SinkNode.h"
#include <ctype.h>

#define REPLACEMENT 1
#define DISCOVERY_STRATEGY 0
#define DECENTRALIZED_DETECTION 0
#define CENTRALIZED_DETECTION 0


/*
//Scenario New
#define NUM_RELAY 13
#define NUM_SOURCE 10 
#define NUM_MOBILE 6

*/

//Scenario 1-7 
#define NUM_RELAY 8
#define NUM_SOURCE 7 
#define NUM_MOBILE 6

/*
// Scenario 8-9
#define NUM_RELAY 10
#define NUM_SOURCE 9 
#define NUM_MOBILE 6

/*
// Scenario Random
#define NUM_RELAY 20
#define NUM_SOURCE 10 
#define NUM_MOBILE 6
*/
#define SPEED 0.65

//node's location
float node_loc_x;
float node_loc_y;

uint8_t my_hops=0;
uint8_t best_hop=UNKNOWN_HOPS;

bool energy_efficient = false;

uint8_t total_flows = 0;

uint16_t number_of_packets_received = 0;
uint16_t number_of_packets_forwarded = 0;

uint16_t packets_received_by_sink = 0;
uint16_t total_sensor_packets = 0;

double remaining_energy=0;

static int iterations = 0;
int my_counter = 0;

struct KRepresentative *repr1;

// TIMER
static struct etimer et,timer_energy;

struct packetqueue *pqueue;

bool responseFlag = false;
bool discoveryFlag=false;
int discoveryAddress= 0;
struct discovery_position_entry *pp;
int drT = 0;
int current_next_hop = -1;

int node_idd = 0, node_pos_x=0, node_pos_y=0;
//static int num_nodes = 37,cur_node=1;
//static int num_nodes = 26,cur_node=1; //21

// static int num_nodes = 22,cur_node=1;
static int num_nodes = 30,cur_node=1;

int discovery_counter=0;
int discoveryWait = 0;
int discoveryStart = 0;
// struct KRepresentative *repr1;

//static bool init = false;
//static bool clusteringdone = false;

int suspiciousFaulty[MAX_NEIGHBORS];
int scounter = 0;
bool firstOnly =  true;
float letnew_x = 0.0;
float letnew_y = 0.0;

const double eps = 1e-14;

bool centralized_detection_flag = false, decentralized_detection_flag=false;

struct Point perfect,dest,*cur;
struct Point r1,r2;
struct Point position;

bool wait_results = false;
bool mustcheck=false;
bool print_energy = false;
int upperNodes[MAX_NODE_NEIGHBORS+1], ucounter=0;
int foundedNodes[MAX_NODE_NEIGHBORS+1], fcounter=0;


LIST(neighbor_table);
LIST(mobile_table);
MEMB(neighbor_mem, struct table_entry_neighbor, 30);
MEMB(mobile_mem, struct mobile_table_entry, 6);

LIST(discovery_positions);
LIST(kpoints_table);
LIST(krepresentatives_table);
LIST(faulty_table);
MEMB(discovery_mem, struct discovery_position_entry, 6);
MEMB(kpoints_mem, struct KPoint, MAX_NEIGHBORS);
MEMB(krepresentatives_mem, struct KRepresentative, 6);
MEMB(faulty_mem, struct faulty_entry, MAX_NEIGHBORS);

LIST(pointList);
LIST(targetList);
LIST(bestPathList);
MEMB(pointList_mem,struct pointList_entry, MAX_NEIGHBORS);
MEMB(targetList_mem,struct targetList_entry, 8);
MEMB(bestPathList_mem,struct bestPathList_entry, 8);

LIST(positionsList);
MEMB(positionsList_mem,struct positionsList_entry, MAX_NEIGHBORS);

/*---------------------------------------------------------------------------*/
static struct unicast_conn uc;
static struct announcement example_announcement;
PROCESS(multihop_process, "WSP Routing");
PROCESS(initialization, "Initialization");
PROCESS(energy, "Energy");
AUTOSTART_PROCESSES(&initialization,&multihop_process,&energy);
/*---------------------------------------------------------------------------*/
int distancefromtwopoints(int x1, int y1, int x2, int y2);
struct Point intersection_point_of_lineANDcircle(float xk, float yk, float xsink, float ysink, int r);
struct Point intersection_point_of_TWOcircle(float xk, float yk, float xsink, float ysink, int r);
point make_point(double x, double y);
void print_point(point p);
double sq(double x);
bool within(double x1, double y1, double x2, double y2, double x, double y);
int rxy(double x1, double y1, double x2, double y2, double x, double y, bool segment);
double fx(double n1, double n2, double n3, double x);
double fy(double n1, double n2, double n3, double y);
int intersects(point p1, point p2, point cp, double r, bool segment);
static void displayNeighborTable();
static void displayFaultyTable();
static void displayDiscoveryTable();
static void displayKPointsTable();
static void displayKRepresentativesTable();
static void displayKRepresentativesMembers();
static void displayMobileTable();
void addPositionNodes(int node_id,float node_x, float node_y);
struct Point searchnodesposition(int ad);
bool checkforrange(float x,float y,float x1, float y1,int range);
static bool checkforrangetimes(float x,float y,float x1, float y1,int range,float times);
double move_energy(float init_x, float init_y, float cur_x, float cur_y, float new_x, float new_y);
double threshold_function(float init_x, float init_y, float cur_x, float cur_y);
int selectMobileNodetosend(float new_x, float new_y);
int selectIdleMobileNodetosend(float new_x, float new_y);
struct Point getmobilenodePosition(int caddr);
void updatemobilenodePosition(int addr,float x, float y,int mhop);
int findLevelMobile(int addr);
uint16_t concatenate(unsigned x, unsigned y);
static int getBestNeighborHopNum();
void delete_targetList();
void delete_faultytable();
void delete_kpoints();
void delete_krepresentatives();
bool isFaultyNode(int node);
void addFaulty(int node_id);
void addTarget(int node_id);
void addKPoint(int id, float x, float y);
void addDiscoverPosition(float x, float y);
static int distancefromtwopoints2(int x1, int y1, int x2, int y2);
void replacedFaulty(int node_id);
void RandomNumberGenerator(int *array, const int nMin, const int nMax, const int nNumOfNumsToGenerate);
struct table_entry_neighbor* findNodeEntry(int id);
struct mobile_table_entry* findMobileNodeEntry(int id);
void updatemobilenodePosition2(int addr,float x, float y);
bool isSource(int node);
void addSuspiciousFaultyNodes(float x, float y);
void clearSuspiciousFaultyNodes();
static void deleteNeighbor(int id);
void fillKrepresentatives(int k);
bool clusterRange();
int alternativeCluster();
void newRepresentatives();
bool assignPoints();
void reassignPoints();
int estimateMinClusters();
bool kmeans(int k );
void fillMembers();
void clustering();
void fillKPoints(float x, float y, int found[], int cfound);
int estimateNumNeighbors(int id, float x, float y);
int missingNeighbor(int addr,float x, float y,int found[],int cfound);
void sendMobileInvestigate(int faulty, int faulty_lvl);
void sendMobileNodeReplacement(int id, int found[], int cfound);
void navigationAlgorithm(int addr,float x, float y,int found[], int cfound);
void startMobileNodeDiscover(int id, int found[], int cfound);
void sendMobileNodeDiscover();
void createSFNlist();
void mobileNodePlacement(int node);
void nodeReplacement();
void createSFNlist2();
void shortPath(float dx, float dy);
void navigationAlgorithm2();
void findTargetList(int cluster);
struct Point* getResult(int result);
int findMobileNodeLevel(int mobile);
void nodePlacement(int mobile);
void addPosition(float x, float y);
void DirectPathMobileCC_algorithm(float x, float y, float destx, float desty, int level);
void mobileNodePlacementTest(int mobile);
void resetThreshold();
void resetThresholdNode(int node_id);
static int neighborThreshold();
static void received_announcement(struct announcement *a,const linkaddr_t *from,uint16_t id, uint16_t value);
static void recv(struct multihop_conn *c, const linkaddr_t *sender,const linkaddr_t *prevhop,uint8_t hops);
static void recv_uc(struct unicast_conn *c, const linkaddr_t *sender);
static linkaddr_t *forward(struct multihop_conn *c,const linkaddr_t *originator, const linkaddr_t *dest,const linkaddr_t *prevhop, uint8_t hops);
static bool parse_sign(const char **const s);
static double parse_digits(const char **const s, int *const count);
double extended_atof(const char *s);
void insertNeighborTable(int node_id, float pos_x, float pos_y);
void insertMobileTable(int node_id, float pos_x, float pos_y);
/*---------------------------------------------------------------------------*/
/*  [Added by Natalie] 
 * 
 * GEOMETRY FUNCTIONS FOR MOBILE NODE POSITION CALCULATIONS 
 *
 */
/*---------------------------------------------------------------------------*/
int distancefromtwopoints(int x1, int y1, int x2, int y2){
    long int result=0,result1=0,result2=0,result3=0;
    result1 = pow(abs((x1-x2)),2);
    result2 = pow(abs((y1-y2)),2);
    result3 = result1+result2;
    result = sqrt(result3);
    return (int)result;
}

static int distancefromtwopoints2(int x1, int y1, int x2, int y2){
    long int result=0,result1=0,result2=0,result3=0;
    result1 = pow(abs((x1-x2)),2);
    result2 = pow(abs((y1-y2)),2);
    result3 = result1+result2;
    result = sqrt(result3);
    return (int)result;
}
/*---------------------------------------------------------------------------*/
struct Point intersection_point_of_lineANDcircle(float xk, float yk, float xsink, float ysink, int r){
    double a1=0,b1=0,A2=0,B2=0,C2=0,D2=0,m1=0,c1=0,p1=0,q1=0,da=0,db=0;
    double d2a=0,d2b=0,d2c=0,bn=0,c1n=0,m1n=0;
    float xa=0,xb=0,ya=0,yb=0,xnew=0,ynew=0;
    if(xk==xsink){
        if(yk<ysink){
            xnew=xk;
            ynew=yk+r;
        }
        else{
            xnew=xk;
            ynew=yk-r;
        }
    }
    else if(yk==ysink){
        if(xk<xsink){
            xnew=xk+r;
            ynew=yk;
        }
        else{
            xnew=xk-r;
            ynew=yk;
        }
    }
    else{
        a1 = xk - xsink;
        b1 = yk - ysink;
        m1 = b1/a1;
        if(m1<0){
            m1n = m1 * (-1);
        }
        else{
            m1n = m1;
        }
        c1= yk - (m1*xk);
        if(c1<0){
            c1n = c1 * (-1);
        }
        else{
            c1n = c1;
        }
        p1=xk;
        q1=yk;
        A2= pow(m1n,2)+1;
        B2=2*((m1*c1)-(m1*q1)-p1);
        C2=pow(q1,2)-pow(r,2)+pow(p1,2)-(2*c1*q1)+pow(c1n,2);
        if(B2<0){
            bn = B2 * (-1);
            d2a = pow(bn,2);
        }
        else{
            d2a = pow(B2,2);
        }
        d2b = 4*A2*C2;
        d2c = d2a - d2b;
        D2= sqrt(d2c);
		
        xa= (-B2+D2)/(2*A2);
        xb= (-B2-D2)/(2*A2);
        ya=(m1*xa)+c1;
        yb=(m1*xb)+c1;
        da=distancefromtwopoints(xa,ya,xsink,ysink);
        db=distancefromtwopoints(xb,yb,xsink,ysink);

        if(da<db){
            xnew =xa;
            ynew=ya;
        }
        else{
            xnew =xb;
            ynew=yb;
        }
    }
    struct Point p;
    p.x=xnew;
    p.y=ynew;
    return p;
}
/*---------------------------------------------------------------------------*/
struct Point intersection_point_of_TWOcircle(float xk, float yk, float xsink, float ysink, int r){
    double a1=0,b1=0,c1=0,d1=0,xnew=0,ynew=0,B2=0,ta=0,tb=0,xo=0,m1=0,da=0,db=0;
    double xkp=0,ykp=0,ysinkp=0,xsinkp=0,bn=0,sqd=0,yab=0,xab=0,xok=0,m1n=0;
    double D2,ac,bp,A2,C2;
    float xa=0,xb=0,ya=0,yb=0;

    a1 = 2*(xsink-xk);
    b1 = 2*(ysink-yk);
	
    if(xsink<0){
        xsinkp = xsink * (-1);
    }
    else{
        xsinkp = xsink;
    }
    if(xk<0){
        xkp = xk * (-1);
    }
    else{
        xkp = xk;
    }
    if(ysink<0){
        ysinkp = ysink * (-1);
    }
    else{
        ysinkp = ysink;
    }
    if(yk<0){
        ykp = yk * (-1);
    }
    else{
        ykp = yk;
    }
    c1=pow(xsinkp,2)-pow(xkp,2)+pow(ysinkp,2)-pow(ykp,2);
    if(a1==0 && b1==0){
        struct Point p;
        return p;
    }
    else if(a1==0){ //vertical line 
        yab=ysink-yk;
        if(yab<0){
            yab = yab* (-1);
        }
        d1=pow(r,2)-pow(yab,2);
        if(d1<0){
            struct Point p;
            return p;
        }
        xa = xk+sqrt(d1);
        xb = xk-sqrt(d1);
        ya=c1/b1;
        yb=c1/b1;
    }
    else if(b1==0){ //horizontal line 
        xab=xsink-xk;
        if(xab<0){
            xab = xab* (-1);
        }
        d1=pow(r,2)-pow(xab,2);
        if(d1<0){
            struct Point p;
            return p;
        }
        ya = yk+sqrt(d1);
        yb = yk-sqrt(d1);
        xa=c1/a1;
        xb=c1/a1;
    }
    else{
        xo=c1/a1;
        m1=a1/b1;
        if(m1<0){
            m1n = m1 * (-1);
        }
        else{
            m1n = m1;
        }
        A2=1+pow(m1n,2);
        B2=(xo-xk)+(m1*yk);
        if(yk<0){
            ykp = yk * (-1);
        }
        else{
            ykp = yk;
        }
        xok=xo-xk;
        if(xok<0){
            xok = xok* (-1);
        }
        C2=pow(xok,2)+pow(ykp,2)-pow(r,2);
        if(B2<0){
            bn = B2*(-1);
        }
        else{
            bn = B2;
        }
        ac = (A2*C2);
        bp = pow(bn,2);
        D2=bp-ac;
        if(D2<0){
            struct Point p;
            return p;
        }
        sqd = sqrt(D2);
        ta=(B2+sqd)/A2;
        tb=(B2-sqd)/A2;
        xa=xo-ta;
        xb=xo-tb;
        ya=m1*ta;
        yb=m1*tb;
    }
    da=distancefromtwopoints(xa,ya,xsink,ysink);
    db=distancefromtwopoints(xb,yb,xsink,ysink);

    if(da<db){
        xnew=xa;
        ynew=ya;
    }
    else{
        xnew=xb;
        ynew=yb;
    }
    struct Point p;
    p.x=xnew;
    p.y=ynew;
    return p;
}
/*---------------------------------------------------------------------------*/
/*  [Added by Naoum] 
 * 
 * GEOMETRY FUNCTIONS FOR MOBILE NODE POSITION CALCULATIONS 
 *
 */
/*---------------------------------------------------------------------------*/
point make_point(double x, double y){
    point p = { x, y };
    return p;
}

void print_point(point p){
    double x = p.x;
    double y = p.y;
    if (x == 0) {
        x = 0;
    }
    if (y == 0) {
        y = 0;
    }
	if (DEBUGFM)
		printf("INTERSECTION (%d, %d)\n", (int) x, (int) y);
}
 
double sq(double x){
    return x * x;
}
 
bool within(double x1, double y1, double x2, double y2, double x, double y){
    double d1 = sqrt(sq(x2 - x1) + sq(y2 - y1));    // distance between end-points
    double d2 = sqrt(sq(x - x1) + sq(y - y1));      // distance from point to one end
    double d3 = sqrt(sq(x2 - x) + sq(y2 - y));      // distance from point to other end
    double delta = d1 - d2 - d3;
    return fabs(delta) < eps;   // true if delta is less than a small tolerance
}
 
int rxy(double x1, double y1, double x2, double y2, double x, double y, bool segment){
    if (!segment || within(x1, y1, x2, y2, x, y)) {
        print_point(make_point(x, y));
        return 1;
    } else {
        return 0;
    }
}
 
double fx(double n1, double n2, double n3, double x){
    return -(n1 * x + n3) / n2;
}
 
double fy(double n1, double n2, double n3, double y){
    return -(n2 * y + n3) / n1;
}
 
// Prints the intersection points (if any) of a circle, center 'cp' with radius 'r',
// and either an infinite line containing the points 'p1' and 'p2'
// or a segment drawn between those points.
int intersects(point p1, point p2, point cp, double r, bool segment){
    double x0 = cp.x, y0 = cp.y;
    double x1 = p1.x, y1 = p1.y;
    double x2 = p2.x, y2 = p2.y;
    double n1 = y2 - y1;
    double B = x1 - x2;
    double n3 = x2 * y1 - x1 * y2;
    double a = sq(n1) + sq(B);
    double b, c, d;
    bool bnz = true;
    int cnt = 0;
 
    if (fabs(B) >= eps) {
        // if B isn't zero or close to it
        b = 2 * (n1 * n3 + n1 * B * y0 - sq(B) * x0);
        c = sq(n3) + 2 * B * n3 * y0 - sq(B) * (sq(r) - sq(x0) - sq(y0));
    } else {
        b = 2 * (B * n3 + n1 * B * x0 - sq(n1) * y0);
        c = sq(n3) + 2 * n1 * n3 * x0 - sq(n1) * (sq(r) - sq(x0) - sq(y0));
        bnz = false;
    }
    d = sq(b) - 4 * a * c; // discriminant
    if (d < 0) {
        // line & circle don't intersect
	if (DEBUGFM)
		printf("INTERSECTION NO POINT \n");
        return 0;
    }
 
    if (d == 0) {
	if (DEBUGFM)
		printf("INTERSECTION ONE POINT \n");
        // line is tangent to circle, so just one intersect at most
        if (bnz) {
            double x = -b / (2 * a);
            double y = fx(n1, B, n3, x);
            cnt = rxy(x1, y1, x2, y2, x, y, segment);
		r1.x = (float) x;
		r1.y = (float) y;
		return 1;
		
        } else {
            double y = -b / (2 * a);
            double x = fy(n1, B, n3, y);
            cnt = rxy(x1, y1, x2, y2, x, y, segment);
		r1.x = (float) x;
		r1.y = (float) y;
		return 1;
        }
    } else {
        // two intersects at most
	if (DEBUGFM)
		printf("INTERSECTION TWO POINT \n");
        d = sqrt(d);
		bool flag1,flag2;
        if (bnz) {
            double x = (-b + d) / (2 * a);
            double y = fx(n1, B, n3, x);
           // cnt = rxy(x1, y1, x2, y2, x, y, segment);

	 if (rxy(x1, y1, x2, y2, x, y, segment)){
			r1.x = (float) x;
			r1.y = (float) y;
			flag1=true;
		}

          x = (-b - d) / (2 * a);
          y = fx(n1, B, n3, x);

            if (rxy(x1, y1, x2, y2, x, y, segment)){
		r2.x = (float) x;
		r2.y = (float) y;
		flag2=true;
	    }

        } else {
		
            double y = (-b + d) / (2 * a);
            double x = fy(n1, B, n3, y);
             if (rxy(x1, y1, x2, y2, x, y, segment)){
			r1.x = (float) x;
			r1.y = (float) y;
			flag1=true;
		}
 
            y = (-b - d) / (2 * a);
            x = fy(n1, B, n3, y);
               if (rxy(x1, y1, x2, y2, x, y, segment)){
			r2.x = (float) x;
			r2.y = (float) y;
			flag2=true;
		}
        }

	if (flag1 && flag2){
			if (DEBUGFM){
				printf("INTERSECTION DISTANCE 1 %d \n",distancefromtwopoints2((int) r1.x, (int) r1.y, (int) p2.x, (int) p2.y) );
				printf("INTERSECTION DISTANCE 2 %d \n",distancefromtwopoints2( (int) r2.x,(int) r2.y,(int) p2.x,(int) p2.y) );
			}
			if (distancefromtwopoints2((int) r1.x, (int) r1.y, (int) p2.x, (int) p2.y) < distancefromtwopoints2( (int) r2.x,(int) r2.y,(int) p2.x,(int) p2.y) )
				return 1;
			else 
				return 2;
		}
		else if (flag1){
			if (DEBUGFM)
				printf("INTERSECTION DISTANCE 1 %d \n",distancefromtwopoints2((int) r1.x, (int) r1.y, (int) p2.x, (int) p2.y) );
			return 1;
		}
		else{
			if (DEBUGFM)
				printf("INTERSECTION DISTANCE 2 %d \n",distancefromtwopoints2( (int) r2.x,(int) r2.y,(int) p2.x,(int) p2.y) ); printf("INTERSECTION DISTANCE 2 %d \n",distancefromtwopoints2( (int) r2.x,(int) r2.y,(int) p2.x,(int) p2.y) );
			return 2;

		}
    }
 
    if (cnt <= 0) {
	if (DEBUGFM)
		printf("INTERSECTION NO POINT 2 \n");
    }
	return 0;
}

/*---------------------------------------------------------------------------*/
/*  DISPLAY FUNCTIONS   */
/*---------------------------------------------------------------------------*/
static void displayNeighborTable(){
    int pos = 0;
    struct table_entry_neighbor *current_entry;
    printf("- displayNeighborTable > \n");
    for(current_entry=list_head(neighbor_table); current_entry!=NULL; current_entry=current_entry->next, pos++) {
        printf("t[%d] = %d.%d with x: %ld.%03u and y: %ld.%03u LEVEL: %d|| \n",pos, current_entry->addr.u8[0], current_entry->addr.u8[1],	(long)current_entry->pos_x,(unsigned)((current_entry->pos_x-floor(current_entry->pos_x))*1000), (long)current_entry->pos_y,(unsigned)((current_entry->pos_y-floor(current_entry->pos_y))*1000),current_entry->hops);
		}
    printf("\n");
}
/*---------------------------------------------------------------------------*/
static void displayFaultyTable(){
    int pos = 0;
    struct faulty_entry *current_entry;
    printf("- displayFaultyTable > \n");
    for(current_entry=list_head(faulty_table); current_entry!=NULL; current_entry=current_entry->next, pos++) 
        printf("t[%d] = %d \n",pos, current_entry->faultyID);
    printf("\n");
}
/*---------------------------------------------------------------------------*/
static void displayDiscoveryTable(){
    int pos = 0;
    struct discovery_position_entry *current_entry;
    printf("- displayDiscoveryTable > \n");
    for(current_entry=list_head(discovery_positions); current_entry!=NULL; current_entry=current_entry->next, pos++) {
        printf("t[%d] with x: %ld.%03u and y: %ld.%03u || \n",pos, (long)current_entry->pos_x,(unsigned)((current_entry->pos_x-floor(current_entry->pos_x))*1000), (long)current_entry->pos_y,(unsigned)((current_entry->pos_y-floor(current_entry->pos_y))*1000));
		}
    printf("\n");
}
/*---------------------------------------------------------------------------*/
static void displayKPointsTable(){
    int pos = 0;
    struct KPoint *current_entry;
    printf("- displayKPointsTable > \n");
    for(current_entry=list_head(kpoints_table); current_entry!=NULL; current_entry=current_entry->next, pos++) {
        printf("t[%d] = %d with x: %ld.%03u and y: %ld.%03u cluster: %d || \n",pos, current_entry->id, (long)current_entry->x,(unsigned)((current_entry->x-floor(current_entry->x))*1000), (long)current_entry->y,(unsigned)((current_entry->y-floor(current_entry->y))*1000),current_entry->cluster);
		}
    printf("\n");
}
/*---------------------------------------------------------------------------*/
static void displayKRepresentativesTable(){
    int pos = 0;
    struct KRepresentative *current_entry;
    printf("- displayKRepresentativesTable > \n");
    for(current_entry=list_head(krepresentatives_table); current_entry!=NULL; current_entry=current_entry->next, pos++) {
        printf("t[%d] with x: %ld.%03u and y: %ld.%03u cluster: %d || \n",pos, (long)current_entry->x,(unsigned)((current_entry->x-floor(current_entry->x))*1000), (long)current_entry->y,(unsigned)((current_entry->y-floor(current_entry->y))*1000),current_entry->cluster);
		}
    printf("\n");
}
/*---------------------------------------------------------------------------*/
static void displayKRepresentativesMembers(){
    int pos = 0,i;
    struct KRepresentative *current_entry;
    printf("- displayKRepresentativesMembers > \n");
    for(current_entry=list_head(krepresentatives_table); current_entry!=NULL; current_entry=current_entry->next, pos++) {
        printf("t[%d] with ||  ",pos);
	  for (i = 0; i< current_entry -> counter; i++)
		printf("%d  || ", (current_entry -> members)[i]);
	  printf("\n");
	}
    printf("\n");
}
/*---------------------------------------------------------------------------*/
static void displayMobileTable(){
    int pos = 0;
    struct mobile_table_entry *current_entry;
    printf("- display Mobile Table > \n");
    for(current_entry=list_head(mobile_table); current_entry!=NULL; current_entry=current_entry->next, pos++) {
		printf("t[%d] = %d.%d with x: %ld.%03u and y: %ld.%03u & con_node_addr: %d  & availability: %d|| \n",pos, current_entry->addr.u8[0], current_entry->addr.u8[1],	(long)current_entry->current_pos_x,(unsigned)((current_entry->current_pos_x-floor(current_entry->current_pos_x))*1000), (long)current_entry->current_pos_y,(unsigned)((current_entry->current_pos_y-floor(current_entry->current_pos_y))*1000),current_entry->congested_node_addr,current_entry->favailable);
    }
    printf("\n");
}

/*---------------------------------------------------------------------------*/
void addPositionNodes(int node_id,float node_x, float node_y){
	struct table_entry_neighbor *ce;
	for(ce=list_head(neighbor_table); ce!=NULL; ce=ce->next){
		if(ce->addr.u8[0] == node_id){
			ce->pos_x = node_x;
			ce->pos_y = node_y;
			return;
		}
	}
	struct mobile_table_entry *me;
	for(me=list_head(mobile_table); me!=NULL; me=me->next){
		if(me->addr.u8[0] == node_id){
			me->init_pos_x = node_x;
			me->init_pos_y = node_y;
			me->current_pos_x = node_x;
			me->current_pos_y = node_y;
			return;
		}
	}	
}
/*---------------------------------------------------------------------------*/
/*  SEARCH FUNCTIONS    */
/*---------------------------------------------------------------------------*/
struct Point searchnodesposition(int ad){
	struct Point p;
	if (ad == 1){
		p.x=node_loc_x;
            p.y=node_loc_y;
	}
	struct table_entry_neighbor *ce;
	for(ce=list_head(neighbor_table); ce!=NULL; ce=ce->next){
		if(ce->addr.u8[0] == ad){
			p.x=ce->pos_x;
			p.y=ce->pos_y;
		}
	}
	struct mobile_table_entry *me;
	for(me=list_head(mobile_table); me!=NULL; me=me->next){
		if(me->addr.u8[0] == ad){
			p.x=me->current_pos_x;
			p.y=me->current_pos_y;
		}
	}
	return p;
}
/*---------------------------------------------------------------------------*/
/*  CHECK FUNCTIONS */
/*---------------------------------------------------------------------------*/
bool checkforrange(float x,float y,float x1, float y1,int range){
    int distance = distancefromtwopoints(x1,y1,x,y);
    if(distance <= range){
        return true;
    }
    return false;
}

static bool checkforrangetimes(float x,float y,float x1, float y1,int range,float times){
    int distance = distancefromtwopoints2(x1,y1,x,y);
    if(distance <= times*range){
        return true;
    }
    return false;
}
/*---------------------------------------------------------------------------*/
double move_energy(float init_x, float init_y, float cur_x, float cur_y, float new_x, float new_y){
	double distance1 =distancefromtwopoints(cur_x,cur_y, new_x,new_y);
	double distance2 = distancefromtwopoints(new_x,new_y,init_x, init_y);
	double totalD = distance1+distance2;
	double energy = 33.9 * totalD/SPEED;
	return energy;
}
double threshold_function(float init_x, float init_y, float cur_x, float cur_y){
	double distance = distancefromtwopoints(init_x, init_y, cur_x,cur_y);
	double energy = (2*((33.9 * distance)/SPEED));
	return energy;
}
/*Select the mobile node which is available to send him the position*/
int selectMobileNodetosend(float new_x, float new_y){
	double moveEnergy, lowThreshold;
	double distance,bestdistanceM=-1,bestdistanceIM=-1;
	int maddr=-1,imaddr=-1;
	struct mobile_table_entry *m;
	
	//step1: find all idle MN that have moveEnergy>lowThreshold
	//step2: find the closest to the new position
	//step3: find the closest to the new position from the new MN
    for(m=list_head(mobile_table); m!=NULL; m=m->next){
		if (DEBUGFM)
			printf("maddr: %d -> favailable: %d -> idle_status: %d \n",m->addr.u8[0], m->favailable, m->idle_status);
		if(m->favailable == true && m->idle_status == true){
			printf("in if\n");
            //check for moveEnergy>lowThreshold
			moveEnergy = move_energy(m->init_pos_x,m->init_pos_y,m->current_pos_x,m->current_pos_y,new_x,new_y);
			lowThreshold = threshold_function(m->init_pos_x,m->init_pos_y,m->current_pos_x,m->current_pos_y);
			if(moveEnergy>lowThreshold){
				//addAvailableIdleMobileNodesTable(m->addr.u8[0],m->current_pos_x,m->current_pos_y,distance);
				distance = distancefromtwopoints(m->current_pos_x,m->current_pos_y,new_x,new_y);
				if (DEBUGFM)
					printf("distance: %ld.%03u\n",(long)distance,(unsigned)((distance-floor(distance))*1000));
				if(bestdistanceIM==-1){
					bestdistanceIM = distance;
					imaddr = m->addr.u8[0];
				}
				else if(distance<bestdistanceIM){
					bestdistanceIM = distance;
					imaddr = m->addr.u8[0];
				}
			}
        }
		else if(m->favailable == true && m->idle_status == false){
		// I am a mobile node near the sink
			distance = distancefromtwopoints(m->current_pos_x,m->current_pos_y,new_x,new_y);
			if(bestdistanceM==-1){
				bestdistanceM = distance;
				maddr = m->addr.u8[0];
			}
			else if(distance<bestdistanceM){
				bestdistanceM = distance;
				maddr = m->addr.u8[0];
			}
		}	
	}
	//step4: compare the MN selected from s2+s3. Retrun the closest
	if(maddr==-1 && imaddr!=-1){
		return imaddr;
	}
	else if(maddr!=-1 && imaddr==-1){
		return maddr;
	}
	else{	
		if(bestdistanceM<bestdistanceIM){
			return maddr;
		}
		else{ //bestdistanceM>bestdistanceIM
			return imaddr;
		}
	}
	return 22;
	//return -1;
}
int selectIdleMobileNodetosend(float new_x, float new_y){
	double distance,bestdistanceIM = 0;
	int imaddr=-1;
	struct mobile_table_entry *m;
	bool flagfirst = true;

    for(m=list_head(mobile_table); m!=NULL; m=m->next){
		if(m->favailable == true && m->idle_status == true){
			distance = distancefromtwopoints(m->current_pos_x,m->current_pos_y,new_x,new_y);
			
			if(flagfirst == true){
				bestdistanceIM = distance;
				imaddr = m->addr.u8[0];
				flagfirst = false;
			}
			else{
				if(distance<bestdistanceIM){
					bestdistanceIM = distance;
					imaddr = m->addr.u8[0];
				}
			}
        	}
	}
	
	return imaddr;
	
}
/*---------------------------------------------------------------------------*/
struct Point getmobilenodePosition(int caddr){
    struct mobile_table_entry *current_entry;
    struct Point p;
    for(current_entry=list_head(mobile_table); current_entry!=NULL; current_entry=current_entry->next) {
        if(current_entry->congested_node_addr == caddr){
            p.x = current_entry->current_pos_x;
            p.y = current_entry->current_pos_y;
            return p;
        }
    }
    return p;
}
/*---------------------------------------------------------------------------*/
void updatemobilenodePosition(int addr,float x, float y,int mhop){
    struct mobile_table_entry *current_entry;
    for(current_entry=list_head(mobile_table); current_entry!=NULL; current_entry=current_entry->next) {
        if(addr == current_entry->addr.u8[0]){
		current_entry->current_pos_x = x;
		current_entry->current_pos_y = y;
		current_entry->hops = mhop;
		current_entry->favailable = false;
            return;
        }
    }
    if(true){
        printf("Mobile List after update:\n");
        displayMobileTable();
    }
    return;
}

void updatemobilenodePosition2(int addr,float x, float y){
    struct mobile_table_entry *current_entry;
    for(current_entry=list_head(mobile_table); current_entry!=NULL; current_entry=current_entry->next) {
        if(addr == current_entry->addr.u8[0]){
             current_entry->current_pos_x = x;
             current_entry->current_pos_y = y;
		current_entry->favailable = false;
        }
    }
}

void updatemobilenodeAvailable(int addr){
    struct mobile_table_entry *current_entry;
    for(current_entry=list_head(mobile_table); current_entry!=NULL; current_entry=current_entry->next) {
        if(addr == current_entry->addr.u8[0]){
		current_entry->favailable = true;
        }
    }
}

int findLevelMobile(int addr){
    struct mobile_table_entry *current_entry;
    for(current_entry=list_head(mobile_table); current_entry!=NULL; current_entry=current_entry->next)
        if(addr == current_entry->addr.u8[0])
            return current_entry->hops;
    return -1;
}
/*---------------------------------------------------------------------------*/
uint16_t concatenate(unsigned x, unsigned y){
    unsigned pow = 10;
    while(y >= pow)
        pow *= 10;
    uint16_t res = x * pow + y;
    return res;        
}
/*---------------------------------------------------------------------------*/
/*
 * [Added]
 * This function is called when a node sets its announcement. The checks the 
 * neighbor table to find out the minimum hops between that node and sink node.
*/
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
/*  [Added by Naoum] 
 * 
 * SUPPORTIVE FUNCTIONS 
 *
 */
/*---------------------------------------------------------------------------*/
void delete_targetList(){
    while(list_length(targetList)!=0){
        list_remove(targetList,  list_tail(targetList));
    }
}

void delete_faultytable(){
    while(list_length(faulty_table)!=0){
        list_remove(faulty_table,  list_tail(faulty_table));
    }
}

void delete_kpoints(){
    while(list_length(kpoints_table)!=0){
        list_remove(kpoints_table,  list_tail (kpoints_table));
    }
}

void delete_krepresentatives(){
    while(list_length(krepresentatives_table)!=0){
        list_remove(krepresentatives_table,  list_tail (krepresentatives_table));
    }
}

bool isFaultyNode(int node){
	int pos=0;
	struct faulty_entry *current;
    	for(current=list_head(faulty_table); current!=NULL; current=current->next, pos++)
		if ((current->faultyID) == node)
			return true;
	return false;
}

void addFaulty(int node_id){
	if(isFaultyNode(node_id)==true)
		return;
	bool source = false;
	struct Point p;
	struct table_entry_neighbor  *me;
	for(me=list_head(neighbor_table);me!=NULL;me=me->next)
		if((me->addr.u8[0]) ==node_id){
			p.x = me->pos_x;
			p.y = me->pos_y;
			source = (me->isSourceNode);
			//ff=me;
		}
	struct faulty_entry *f = memb_alloc(&faulty_mem);
    	f->faultyID = node_id;
	f->p = p;
	f->source = source;
	f->replaced = false;
   	list_add(faulty_table, f);
	//list_remove(neighbor_table,ff);
}

void addTarget(int node_id){
	struct Point p;
	struct table_entry_neighbor  *me;
	for(me=list_head(neighbor_table);me!=NULL;me=me->next)
		if((me->addr.u8[0]) ==node_id){
			p.x = me->pos_x;
			p.y = me->pos_y;
		}
	struct targetList_entry *f = memb_alloc(&targetList_mem);
    	f->id = node_id;
	f->p = p;
   	list_add(targetList, f);
}

void addKPoint(int id, float x, float y){
	struct KPoint *current_entry;
	for(current_entry=list_head(kpoints_table); current_entry!=NULL; current_entry=current_entry->next) 
		if (current_entry -> id == id)
			return;			

	struct KPoint *p = memb_alloc(&kpoints_mem);
	p->id = id;
	p->x = x;
	p->y = y;
	p->cluster = -1;
	p->dist = 0;
	list_add(kpoints_table,p);
}

void addDiscoverPosition(float x, float y){
		struct discovery_position_entry  *p = memb_alloc(&discovery_mem);
		p->pos_x = x;
		p->pos_y = y;
		list_add(discovery_positions, p);		
}

void replacedFaulty(int node_id){
	struct faulty_entry *current;
    	for(current=list_head(faulty_table); current!=NULL; current=current->next)
		if (current->faultyID == node_id) 
			current->replaced = true;
	
}

void RandomNumberGenerator(int *array, const int nMin, const int nMax, const int nNumOfNumsToGenerate){
	int nRandonNumber = 0,i;
	for (i = 0; i < nNumOfNumsToGenerate; i++){
		nRandonNumber = (int) abs(random_rand())%(nMax-nMin) + nMin;
   		array[i] = nRandonNumber;
  	}
}

struct table_entry_neighbor* findNodeEntry(int id){
	struct table_entry_neighbor  *me;
	for(me=list_head(neighbor_table);me!=NULL;me=me->next)
		if(me->addr.u8[0] == id)
			return me;
	return NULL;
}

struct mobile_table_entry* findMobileNodeEntry(int id){
	struct mobile_table_entry *me;
	for(me=list_head(mobile_table);me!=NULL;me=me->next)
		if(me->addr.u8[0] == id)
			return me;
	return NULL;
}

bool isSource(int node){
	struct table_entry_neighbor  *me;
	for(me=list_head(neighbor_table);me!=NULL;me=me->next)
		if (me->addr.u8[0] == node && (me->isSourceNode)==true)
			return true;
	return false;
}

void addSuspiciousFaultyNodes(float x, float y){
	int s;
	struct table_entry_neighbor  *me;
	for(me=list_head(neighbor_table);me!=NULL;me=me->next)
		if (checkforrange(x,y,me->pos_x,me->pos_y,RANGE)==true){
			int id = me->addr.u8[0];
			bool find = false;
			for (s=0;s<scounter;s++)
				if (suspiciousFaulty[s] == id)
					find = true;
			if (find == false)			
				suspiciousFaulty[scounter++] = id;
		}
}

void clearSuspiciousFaultyNodes(){
	int s;
	for (s=0;s<scounter;s++)
		suspiciousFaulty[s] =0;
	scounter=0;
}

static void deleteNeighbor(int id){
	struct table_entry_neighbor  *me;
	for(me=list_head(neighbor_table);me!=NULL;me=me->next)
		if(me->addr.u8[0] == id){
			list_remove(neighbor_table, me);
			return;
		}
}
/*---------------------------------------------------------------------------*/
/*  [Added by Naoum] 
 * 
 * CLUSTERING - MODIFIED KMEANS ALGORITHM
 *
 */
/*---------------------------------------------------------------------------*/
void fillKrepresentatives(int k){
	float min_x = 1000.0;
	float min_y = 1000.0;
	float max_x = -1000.0;
	float max_y = -1000.0;
	struct KPoint *current_entry;
	for(current_entry=list_head(kpoints_table); current_entry!=NULL; current_entry=current_entry->next) {
		if (current_entry->x > max_x)
			max_x = current_entry->x;
		if (current_entry->y > max_y)
			max_y = current_entry->y;
		if (current_entry->x < min_x)
			min_x = current_entry->x;
		if (current_entry->y < min_y)
			min_y = current_entry->y;
	}
	int *reprX, *reprY;
	reprX = (int*) malloc (k * (sizeof(int)));
	reprY = (int*) malloc (k * (sizeof(int)));
	RandomNumberGenerator(reprX,min_x,max_x,k);
	RandomNumberGenerator(reprY,min_y,max_y,k);
	int i=0;
	struct KRepresentative *repr;
	for(repr=list_head(krepresentatives_table); repr!=NULL; repr= repr->next,i++) {
		repr->x = (float) reprX[i];
		repr->y = (float) reprY[i];
	}	
	for (i=list_length(krepresentatives_table);i<k;i++){
		struct KRepresentative *r;
		r = memb_alloc(&krepresentatives_mem);
		r->cluster = i+1;
		r->x = (float) reprX[i];
		r->y = (float) reprY[i];
		list_add(krepresentatives_table,r);
	}
	free(reprX);
	free(reprY);
}

bool clusterRange(){
	struct KPoint *point;	
	struct KRepresentative *repr;
	for(repr=list_head(krepresentatives_table); repr!=NULL; repr= repr->next)
		for(point=list_head(kpoints_table); point!=NULL; point=point->next) 
			if (point->cluster == repr->cluster)
				if (checkforrange(repr->x, repr->y, point->x, point->y, RANGE) == false)
					return false;
	return true;
}

//return a cluster with no members
int alternativeCluster(){
	struct KPoint *point;
	struct KRepresentative *repr;	
	int num;
	for(repr=list_head(krepresentatives_table); repr!=NULL; repr= repr->next) {
		num=0;
		for(point=list_head(kpoints_table); point!=NULL; point=point->next) 
			if (point->cluster == repr->cluster)
				num++;
		if (num==0)
			return repr->cluster;
	}
	//return abs(rand()%list_length(krepresentatives_table));
	return 0;								
}

// compute new representatives
void newRepresentatives(){
	struct KPoint *point;
	struct KRepresentative *repr;
	for(repr=list_head(krepresentatives_table); repr!=NULL; repr= repr->next) {
		int num=0;
		float total_x = 0.0;
		float total_y = 0.0;
		 for(point=list_head(kpoints_table); point!=NULL; point=point->next) {
			if (point->cluster == repr->cluster){
				total_x += point->x;
				total_y += point->y;
				num++;
			}
		}
		if (num>0){					
			repr->x = total_x / num;
			repr->y = total_y / num;
		}
	}
	for(point=list_head(kpoints_table); point!=NULL; point=point->next) 
       		 for(repr=list_head(krepresentatives_table); repr!=NULL; repr= repr->next) 
			if (point->cluster == repr->cluster)
				point->dist = distancefromtwopoints2(point->x, point->y, repr->x, repr->y);
}

bool assignPoints(){
	bool nochange = true;
	struct KPoint *point;
	struct KRepresentative *repr;
	int min;
	// assign each point to the closest representative
  	for(point=list_head(kpoints_table); point!=NULL; point=point->next) {
		if (point->cluster >=1)
				min = point-> dist;
			else
				min = INT_MAX; 	
       		 for(repr=list_head(krepresentatives_table); repr!=NULL; repr= repr->next) {
			int dist = distancefromtwopoints2(point->x, point->y, repr->x, repr->y);	
			if (dist < min  || ((point->cluster) <1 )){
				min = dist;
				point->cluster = repr->cluster;	
				nochange = false;
			}
		}
	}
	return nochange;
}

void reassignPoints(){
	struct KPoint *point;
	struct KRepresentative *repr;
	for(point=list_head(kpoints_table); point!=NULL; point=point->next) 
		 for(repr=list_head(krepresentatives_table); repr!=NULL; repr= repr->next) 	
			if (repr->cluster == point->cluster && checkforrange(repr->x, repr->y, point->x, point->y, RANGE) == false){
				int cluster = alternativeCluster();
				if (cluster!=0)
					point->cluster = cluster;
			}
}

int estimateMinClusters(){
	int min = list_length(kpoints_table)/5;
	if (min>1)
		return min;
	return 1;
}

bool kmeans(int k ){
	int max_iterations=10;
	iterations = 0;
	bool nochange;
	fillKrepresentatives(k);

	do{
		printf("CLUSTERING WITH K= %d AND ITERATION %d\n",k,iterations);
		iterations++;
		nochange = assignPoints();
		newRepresentatives();
		if (nochange==false){
			if (clusterRange()==false){
				reassignPoints();
				//noemptyclusters();
				newRepresentatives();
			}
		}

	}while (nochange == false && iterations<max_iterations );
	if (clusterRange()==false)
		return false;
	return true;
}

void fillMembers(){
	struct KPoint *point;
	struct KRepresentative *repr;
	int members[MAX_NEIGHBORS],num=0,i;
	for(repr=list_head(krepresentatives_table); repr!=NULL; repr= repr->next) {
		for(point=list_head(kpoints_table); point!=NULL; point=point->next) 
			if (point->cluster == repr->cluster)
				members[num++]= point->id;
		if (num>0){
			repr->members = malloc(num*sizeof(int));
			repr->counter = num;
			for (i=0; i < num; i++)
				(repr->members)[i] = members[i];
			num=0;
		}
	}
}

void clustering(){
	int k = estimateMinClusters(), iter;
	bool clustering = false;
	do{
		for (iter=1; iter<=30; iter++){
			if (kmeans(k) == true){
				clustering = true;
				break;
			}
			else {
				struct KPoint *current_entry;
				for(current_entry=list_head(kpoints_table); current_entry!=NULL; current_entry=current_entry->next) 
						current_entry -> cluster = -1;
			}
		}
		if (clustering == false)		
			k++;
	} while (clustering == false);
	fillMembers();
	if (DEBUGFM){
		displayKPointsTable();
		displayKRepresentativesTable();
		displayKRepresentativesMembers();
	}
}

void fillKPoints(float x, float y, int found[], int cfound){
	struct table_entry_neighbor  *me;
	for(me=list_head(neighbor_table);me!=NULL;me=me->next)
		if (checkforrange(x,y,me->pos_x,me->pos_y,RANGE)==true){
			int id = me->addr.u8[0],c;
			bool find = false;
			for (c=0;c<cfound;c++)
				if (found[c] == id)
					find = true;
			if (find == false)			
				addKPoint(id,me->pos_x,me->pos_y);
		}
}

/*---------------------------------------------------------------------------*/
/*  [Added by Naoum] 
 * 
 * MOBILE NODE INVESTIGATION MECHANISM 
 *
 */
/*---------------------------------------------------------------------------*/
int estimateNumNeighbors(int id, float x, float y){
	int neighbors=0;
	struct table_entry_neighbor  *me;
	struct mobile_table_entry *current_entry;
	for(me=list_head(neighbor_table);me!=NULL;me=me->next)
		if(me->addr.u8[0] != id)
			if (checkforrange(x,y,me->pos_x,me->pos_y,RANGE)==true)
				neighbors++;
	    for(current_entry=list_head(mobile_table); current_entry!=NULL; current_entry=current_entry->next) 
		if(current_entry->addr.u8[0] != id)
			if (checkforrange(x,y,current_entry->current_pos_x,current_entry->current_pos_y,RANGE)==true)
						neighbors++;
	return neighbors;
}

int missingNeighbor(int addr,float x, float y,int found[],int cfound){
	int i;	
	struct table_entry_neighbor  *me;
	for(me=list_head(neighbor_table);me!=NULL;me=me->next){
		if((me->addr.u8[0]) != addr){
			if (checkforrange(x,y,me->pos_x,me->pos_y,RANGE) == true){
				bool f=false;
				for (i=0; i<cfound; i++)
					if (found[i] == (me->addr.u8[0])){
						f=true;
						break;
					}
					if (f==false)
						return me->addr.u8[0];
				}
		}
	}
	
	return 0;
}

void sendMobileInvestigate(int faulty, int faulty_lvl){
	int discoveryCode = INVESTIGATION;
	float new_x,new_y;
	struct table_entry_neighbor  *me =  findNodeEntry(faulty);
	new_x = me -> pos_x;
	new_y = me -> pos_y;
	list_remove(neighbor_table, me);
	//memb_free(&neighbor_table, me);
	int numNeighbors = estimateNumNeighbors(faulty,new_x,new_y);
	int maddr = selectMobileNodetosend(new_x,new_y);
	updatemobilenodePosition(maddr,new_x,new_y,faulty_lvl);	
	if (DEBUGFM){
		printf("Faulty Notification Message packet received!!!!! id: %d level %d (Fault Reporting Mechanism)\n",faulty,faulty_lvl );
		printf("Position for investigation: %ld.%03u , %ld.%03u (Mobile Investigation Mechanism)\n",(long)new_x,(unsigned)((new_x-floor(new_x))*1000),(long)new_y,(unsigned)((new_y-floor(new_y))*1000) );
		printf("Expected neighbors = %d (Mobile Investigation Mechanism)\n",numNeighbors);	
	}
	printf("XYFSN&%d&%ld.%03u&%ld.%03u&%d&%d&%d\n",maddr,(long)new_x,(unsigned)((new_x-floor(new_x))*1000),(long)new_y,(unsigned)((new_y-floor(new_y))*1000),faulty_lvl,numNeighbors,discoveryCode );
}

/*---------------------------------------------------------------------------*/
/*  [Added by Naoum] 
 * 
 * MOBILE NODE REPLACEMENT 
 *
 */
/*---------------------------------------------------------------------------*/
void sendMobileNodeReplacement(int id, int found[], int cfound){
	struct Point p = searchnodesposition(id);
	int estNeighbors = estimateNumNeighbors(id,p.x,p.y),faulty;
	int faulty_lvl = findLevelMobile(id)+1;
	while (cfound < estNeighbors){
		faulty = missingNeighbor(id,p.x,p.y,found,cfound);	
		if (faulty ==0)
			break;
		found[cfound++] = faulty;
		addFaulty(faulty);
		sendMobileInvestigate(faulty,faulty_lvl);
		if (DEBUGFM)
			printf("Faulty node is %d  (Mobile Node Discovery Mechanism)\n",faulty);	
	}	
}

/*---------------------------------------------------------------------------*/
/*  [Added by Naoum] 
 * 
 * MOBILE NODE DISCOVERY MECHANISM 
 *
 */
/*---------------------------------------------------------------------------*/
void navigationAlgorithm(int addr,float x, float y,int found[], int cfound){
	int estNeighbors = estimateNumNeighbors(addr,x,y);
	while (cfound < estNeighbors){
		int faulty = missingNeighbor(addr,x,y,found,cfound);
		found[cfound++] = faulty;
		struct Point p = searchnodesposition(faulty);
		addDiscoverPosition(p.x,p.y);
		addSuspiciousFaultyNodes(p.x,p.y);
		addFaulty(faulty);
		if (DEBUGFM)
			printf("Faulty node is %d  (Mobile Node Discovery Mechanism)\n",faulty);
		struct table_entry_neighbor  *me =  findNodeEntry(faulty);
		list_remove(neighbor_table, me);	
	}
	addDiscoverPosition(x,y);
	//shortestPath(x,y); //fillDiscoveryPositions(faulty,cfound);
}

void startMobileNodeDiscover(int id, int found[], int cfound){
	struct mobile_table_entry *m = findMobileNodeEntry(id);
	float new_xx = m->current_pos_x;
	float new_yy = m->current_pos_y;
	navigationAlgorithm(id, new_xx, new_yy, found, cfound);
	if (DEBUGFM){
		printf("Mobile Node %d is starting Discovery Process (Mobile Discovery Mechanism) \n", id);
		displayDiscoveryTable();
	}
	discoveryAddress = id;
	discoveryFlag=true;
}

void sendMobileNodeDiscover(){
	struct discovery_position_entry *p;
     	int discoveryCode;

	if (list_length(discovery_positions)==1){
		if (centralized_detection_flag)
			discoveryCode = 4;
		else 
			discoveryCode = 2;
	}
	else 
		discoveryCode = DISCOVERY_IN_PROGRESS;
	p=list_head(discovery_positions);
	float new_x = p->pos_x;
	float new_y = p->pos_y;
	struct Point m = searchnodesposition(discoveryAddress);
	int dist = distancefromtwopoints(m.x, m.y, new_x , new_y); 
	discoveryWait = dist/SPEED + 15.0 +1.0;
	discoveryStart = clock_seconds();
	printf("Moving to the new position, time estimation: %d seconds, distance: %d mobile node: %d\n",discoveryWait, dist,discoveryAddress);
	printf("XYFSN&%d&%ld.%03u&%ld.%03u&%d&%d&%d\n",discoveryAddress,(long)new_x,(unsigned)((new_x-floor(new_x))*1000),(long)new_y,(unsigned)((new_y-floor(new_y))*1000),0,0,discoveryCode);
	updatemobilenodePosition2(discoveryAddress,new_x,new_y);
	list_remove(discovery_positions,p);
      memb_free(&discovery_mem, p);
	if (list_length(discovery_positions) == 0)
		discoveryFlag=false;	
}
/*---------------------------------------------------------------------------*/
/*  [Added by Naoum] 
 * 
 * MOBILE NODE RECOVERY MECHANISM - DECENTRALIZED

 *
 */
/*---------------------------------------------------------------------------*/
void createSFNlist(){
	int j;
	struct faulty_entry *current;
    	for(current=list_head(faulty_table); current!=NULL; current=current->next){
		for(j=0;j<fcounter;j++)
			if(current->faultyID == foundedNodes[j])
				continue;
		if ((current->source) == true) {
			printf("Added Faulty node %d  \n",current->faultyID);	
			addKPoint(current->faultyID,current->p.x,current->p.y);
		}
	}
}

void mobileNodePlacement(int node){
	//nodeReplacement();
	//createSFNlist();
	if (list_length(kpoints_table)==0)
		return;
	// printf("NODECLUSTER\n");
	clustering();
	//nodeReplacement();
	nodePlacement(node);
	//PathMobileFM_algorithm(mobile);
	delete_kpoints();
	delete_krepresentatives();
}

void nodeReplacement(){
	int discoveryCode = PLACEMENT;
	struct faulty_entry *current;
    	for(current=list_head(faulty_table); current!=NULL; current=current->next){
		if ((current->source) == true && (current->replaced) == false){
			float new_x = current->p.x;
			float new_y = current->p.y;;
			int maddr = selectMobileNodetosend(new_x,new_y);
			updatemobilenodePosition(maddr,new_x,new_y,0);	
			printf("XYFSN&%d&%ld.%03u&%ld.%03u&%d&%d&%d\n",maddr,(long)new_x,(unsigned)((new_x-floor(new_x))*1000),(long)new_y,(unsigned)((new_y-floor(new_y))*1000),-1,0,discoveryCode);
			current->replaced = true;
		}
	}
}

/*---------------------------------------------------------------------------*/
/*  [Added by Naoum] 
 * 
 * MOBILE NODE RECOVERY MECHANISM - CENTRALIZED
 *
 */
/*---------------------------------------------------------------------------*/
void createSFNlist2(){
	float total_x = 0.0;
	float total_y = 0.0;
	int nodes = 0;
	struct faulty_entry *current;
    	for(current=list_head(faulty_table); current!=NULL; current=current->next)
		if ((current->source) == true) {
			total_x += current->p.x;
			total_y += current->p.y;
			nodes++;
		}
	float new_x = total_x/nodes;
	float new_y = total_y/nodes;
	struct table_entry_neighbor  *me;
	for(me=list_head(neighbor_table);me!=NULL;me=me->next)
		if (checkforrangetimes(new_x,new_y,me->pos_x,me->pos_y,RANGE,2)==true){
			addKPoint(me->addr.u8[0],me->pos_x,me->pos_y);
			printf("Add to SFN list : %d ( Centralized Detection ) \n",me->addr.u8[0] );
		}
}

void createSFNlist3(){
	float total_x = 0.0;
	float total_y = 0.0;
	int nodes = 0;
	struct faulty_entry *current;
	firstOnly = true;
	letnew_x = 0.0;
	letnew_y = 0.0;
    	for(current=list_head(faulty_table); current!=NULL; current=current->next){
		printf("DEBUG -1 NEWX NEWY %ld.%03u&%ld.%03u with id: %d \n",(long)letnew_x,(unsigned)((letnew_x-floor(letnew_x))*1000),(long)letnew_y,(unsigned)((letnew_y-floor(letnew_y))*1000),current->faultyID);
		// printf("DEBUG 1 NEWX NEWY %ld.%03u&%ld.%03u with id: %d \n",(long)letnew_x,(unsigned)((letnew_x-floor(letnew_x))*1000),(long)letnew_y,(unsigned)((letnew_y-floor(letnew_y))*1000),current->faultyID);
		printf("DEBUG 0 %d with id: %d \n",checkforrangetimes(letnew_x,letnew_y,current->p.x,current->p.y,RANGE,2),current->faultyID);
		if (firstOnly){
			total_x += current->p.x;
			total_y += current->p.y;
			letnew_x = current->p.x;
			letnew_y = current->p.y;
			firstOnly = false;
			nodes++;
			printf("DEBUG 1 NEWX NEWY %ld.%03u&%ld.%03u with id: %d \n",(long)letnew_x,(unsigned)((letnew_x-floor(letnew_x))*1000),(long)letnew_y,(unsigned)((letnew_y-floor(letnew_y))*1000),current->faultyID);
		} else if ((current->source) == true && checkforrangetimes(letnew_x,letnew_y,current->p.x,current->p.y,50,2.0)) {
			total_x += current->p.x;
			total_y += current->p.y;
			nodes++;
			letnew_x = total_x/nodes;
			letnew_y = total_y/nodes;
			printf("DEBUG 2 NEWX NEWY %ld.%03u&%ld.%03u with id: %d \n",(long)letnew_x,(unsigned)((letnew_x-floor(letnew_x))*1000),(long)letnew_y,(unsigned)((letnew_y-floor(letnew_y))*1000),current->faultyID);
		}
		
	}
	// new_x = total_x/nodes;
	// new_y = total_y/nodes;
	printf("DEBUG NEWX NEWY %ld.%03u&%ld.%03u\n",(long)letnew_x,(unsigned)((letnew_x-floor(letnew_x))*1000),(long)letnew_y,(unsigned)((letnew_y-floor(letnew_y))*1000));
	struct table_entry_neighbor  *me;
	for(me=list_head(neighbor_table);me!=NULL;me=me->next)
		if (checkforrangetimes(letnew_x,letnew_y,me->pos_x,me->pos_y,RANGE,2)==true){
			addKPoint(me->addr.u8[0],me->pos_x,me->pos_y);
			printf("Add to SFN list : %d ( Centralized Detection ) \n",me->addr.u8[0] );
		}
}

void shortPath(float dx, float dy){
	int max = list_length(krepresentatives_table),i;
	struct KRepresentative *repr, *minrepr = NULL;
	float curx = dx, cury=dy;
	int visited[max],counter;
	for (counter = 0; counter < max; counter++)
		visited[counter] = false;
	for (counter = 0; counter < max; counter++){
			int min = INT_MAX;
			for(repr=list_head(krepresentatives_table); repr!=NULL; repr= repr->next){
				int dist =distancefromtwopoints2(repr->x, repr->y, curx, cury);
				if (min > dist && visited[(repr->cluster)-1] == false){
					minrepr = repr;
					min = dist;
				}
			}
			visited[(minrepr->cluster)-1] = true;
			curx = minrepr->x;
			cury = minrepr->y;
			addDiscoverPosition(minrepr->x,minrepr->y);
			for(i=0; i<(minrepr->counter);i++)
				suspiciousFaulty[scounter++] = (minrepr->members)[i];
	}		
	addDiscoverPosition(dx,dy);
}

void navigationAlgorithm2(){
	// KANONIKA TREXEI O 2
	// createSFNlist2();

	// DELETE FAULTY TABLE GIA NA TA KAMI ME SIRA (CENTRALIZED)
	// DEN KSEROUME SIDE EFFECTS, GIA DECENTRALIZED KAI CENTRALIZED LOGIKA PAME ME TO createSFNlist2
	delete_faultytable();
	neighborThreshold();
	displayFaultyTable();
	createSFNlist3();
	clustering();
	struct KRepresentative *repr;
	repr=list_head(krepresentatives_table); 
	int maddr = selectMobileNodetosend(repr->x,repr->y);
	discoveryAddress = maddr;
	struct mobile_table_entry *m = findMobileNodeEntry(maddr);
	float new_x = m->current_pos_x;
	float new_y = m->current_pos_y;
	shortPath(new_x,new_y);
	delete_kpoints();
	delete_krepresentatives();
	delete_faultytable();
	discoveryFlag=true;
}


/*---------------------------------------------------------------------------*/
/*  [Added by Naoum] 
 * 
 * MOBILE NODE PLACEMENT MECHANISM 
 *
 */
/*---------------------------------------------------------------------------*/
void findTargetList(int cluster){
	int i;
	struct KRepresentative *repr;
	for(repr=list_head(krepresentatives_table); repr!=NULL; repr= repr->next)
		if (repr->cluster == cluster)
			for(i=0; i<((repr->counter)); i++)
				addTarget((repr->members)[i]);
}

struct Point* getResult(int result){
	struct Point *perfect = malloc(sizeof(struct Point));
	if (result == 1){
		perfect->x=r1.x;
		perfect->y=r1.y;
	}
	else {
		perfect->x=r2.x;
		perfect->y=r2.y;
	}
	return perfect;
}

int findMobileNodeLevel(int mobile){
	struct mobile_table_entry *current_entry;
	for(current_entry=list_head(mobile_table); current_entry!=NULL; current_entry=current_entry->next)
		if(mobile == current_entry->addr.u8[0])
			return current_entry->hops;
	return -1;
}

void nodePlacement(int mobile){
	int i;
	
	for(repr1=list_head(krepresentatives_table); repr1!=NULL; repr1= repr1->next){
		bool inrange = false;
		struct mobile_table_entry *current;
		for(current=list_head(mobile_table); current!=NULL; current=current->next){
			int mobile_counter=0;
			for(i=0; i<((repr1->counter)); i++){
				int m = (repr1->members)[i];
				struct Point mm = searchnodesposition(m);
				if (checkforrange(current->current_pos_x,current->current_pos_y,mm.x,mm.y,RANGE))
						mobile_counter++;
			}
			if (mobile_counter==(repr1->counter)){
				inrange=true;
				break;
			}
			
		}
		if (inrange){
			printf("SKIP CLUSTER =  %d\n", repr1->cluster);
			//continue;
		}
		position.x = repr1->x;
		position.y = repr1->y;
		struct Point dests = searchnodesposition(mobile);
		int min = distancefromtwopoints2(repr1->x,repr1->y,dests.x,dests.y);
		//printf("INTERSECTION minimum distance =  %d\n",min);
		displayKRepresentativesMembers();
		int y;
		int *mymembers = repr1 -> members;
		for(y=0; y<(repr1->counter); y++){
			int node = (mymembers )[y];
			printf("TESTING INTERSECTION before i = %d member: %d \n",y, node);
			// resetThresholdNode(node);
			printf("TESTING INTERSECTION i = %d member: %d \n",y, node);
			displayKRepresentativesMembers();
			struct Point alone = searchnodesposition(node);
			//printf("INTERSECTION FOUND node: %d x: %ld.%03u y: %ld.%03u\n", node, (long)alone.x,(unsigned)((alone.x-floor(alone.x))*1000),(long)alone.y,(unsigned)((alone.y-floor(alone.y))*1000));
			struct Point r;
			r.x = repr1->x;
			r.y = repr1->y;
			struct Point dest = searchnodesposition(mobile);
			//printf("INTERSECTION MOBILE node: %d x: %ld.%03u y: %ld.%03u\n",mobile, (long)dest.x,(unsigned)((dest.x-floor(alone.x))*1000),(long)dest.y,(unsigned)((dest.y-floor(dest.y))*1000));
			//int result = intersects(make_point(alone.x, alone.y), make_point(dest.x, dest.y), make_point(alone.x,alone.y), 45.0, false);
			// 40 before
			int result = intersects(make_point(r.x, r.y), make_point(dest.x, dest.y), make_point(alone.x,alone.y), 45.0, false);
			struct Point perfect;
			if (result == 1){
				perfect.x=r1.x;
				perfect.y=r1.y;
			}
			else if (result == 2){
				perfect.x=r2.x;
				perfect.y=r2.y;
			}
			else {
				perfect.x=-1;
				perfect.y=-1;
			}
			//printf("INTERSECTION %d result = %d position: x: %ld.%03u y: %ld.%03u\n",repr->cluster,result,(long)perfect.x,(unsigned)((perfect.x-floor(perfect.x))*1000),(long)perfect.y,(unsigned)((perfect.y-floor(perfect.y))*1000));
			if ( (result == 1 || result == 2) && min > distancefromtwopoints2(perfect.x,perfect.y,dest.x,dest.y) ){
				//bool inrange = false;
				int ka,pointscounter=0;
				for(ka=0; ka<(repr1->counter); ka++){
					struct Point pp = searchnodesposition((mymembers)[ka]);
					if (checkforrange(perfect.x,perfect.y,pp.x,pp.y,RANGE)){
						pointscounter++;
						printf("TESTING: ka = %d Points counter = %d  in range: %d node is %d\n",ka,  pointscounter,(mymembers )[ka], node);
					} 
					else {
						printf("TESTING: Points counter = %d  not in range: %d, node is %d\n", pointscounter,(mymembers )[ka], node);
					}
				}
				printf("TESTING: i= %d Points counter = %d repr counter = %d\n",y, pointscounter, (mymembers ));
				if (pointscounter == (repr1->counter)){
					min = distancefromtwopoints2(perfect.x,perfect.y,dest.x,dest.y);
					position.x = perfect.x;
					position.y = perfect.y;
				}
			}
		}
		float new_x = position.x;
		float new_y = position.y;
		int mobile_lvl = 0;
		struct mobile_table_entry *current_entry;
		    for(current_entry=list_head(mobile_table); current_entry!=NULL; current_entry=current_entry->next)
			if(mobile == current_entry->addr.u8[0])
			    mobile_lvl = current_entry->hops;
		int new_lvl = mobile_lvl+1;
		//discoveryCode = PLACEMENT;
		DirectPathMobileCC_algorithm(new_x,new_y,dests.x,dests.y, new_lvl);
	}
}

void addPosition(float x, float y){
	struct positionsList_entry *pos;
	pos = memb_alloc(&positionsList_mem);
	(pos->p).x= x;
	(pos->p).y= y;
	list_add(positionsList,pos);
}

void DirectPathMobileCC_algorithm(float x, float y, float destx, float desty, int level){
	//int discoveryCode = PLACEMENT;
	bool flagdp = false;
	bool isSinkRange = false;
	struct Point p;
	int maddr;
	while(flagdp == false){
		if(DEBUGFM)
			printf("Direct path position: x %d  -- y %d  && sink: x %d y %d\n",(int)x,(int)y,(int)destx,(int)desty);
		
		if (centralized_detection_flag){
			struct table_entry_neighbor  *me;
			for(me=list_head(neighbor_table);me!=NULL;me=me->next)
				if (checkforrange(x,y,me->pos_x,me->pos_y,RANGE)==true && (me->isSourceNode) == false && y<(me->pos_y) && isFaultyNode(me->addr.u8[0]) == false){
					printf("Stop by %d && sink: x %d y %d\n",me->addr.u8[0],(int) me->pos_x,(int)me->pos_y);
					isSinkRange = true;
					break;
				}
				if (isSinkRange == false){
					struct mobile_table_entry *current_entry;
					    for(current_entry=list_head(mobile_table); current_entry!=NULL; current_entry=current_entry->next)
						if ((current_entry->current_pos_x) != x && checkforrange(x,y,current_entry->current_pos_x,current_entry->current_pos_y,RANGE)==true  && y<(current_entry->current_pos_y) ){
							printf("Stop by %d && sink: x %d y %d\n",current_entry->addr.u8[0], (int) current_entry->current_pos_x, (int) current_entry->current_pos_y);
							isSinkRange = true;
							break;
						}
				}
		}
		else
			isSinkRange = checkforrange(x,y,destx,desty,RANGE);

		
		if(isSinkRange == true){
			addPosition(x,y);
			flagdp = true;
		}
		else{
			addPosition(x,y);
			p = intersection_point_of_lineANDcircle(x,y,destx,desty,RANGE);
			x=p.x;
			y=p.y;
		}
	}

	struct positionsList_entry *entry;
	int discoverycode = 3;
	if (centralized_detection_flag){
			level = 0;
			discoverycode = 5;
	}
	while(list_length(positionsList)!=0){
		if (list_length(positionsList)==1)
			discoverycode = 6;
		entry = list_tail(positionsList);
		x = (entry->p).x;
		y = (entry->p).y;
		maddr = selectMobileNodetosend(x,y);
		updatemobilenodePosition(maddr,x,y,level);
		printf("XYFSN&%d&%ld.%03u&%ld.%03u&%d&%d&%d\n",maddr,(long)x,(unsigned)((x-floor(x))*1000),(long)y,(unsigned)((y-floor(y))*1000),level,0,discoverycode);
		if (!centralized_detection_flag)
			level++;
		list_remove(positionsList,  list_tail(positionsList));
    }
	return;
}

/*---------------------------------------------------------------------------*/
/*  [Added by Naoum] 
 * 
 * TESTING
 *
*/
/*---------------------------------------------------------------------------*/

void mobileNodePlacementTest(int mobile){
	addFaulty(15);
	addFaulty(14);
	addFaulty(6);
	addFaulty(8);
	addFaulty(7);
	addFaulty(17);
	addFaulty(16);
	createSFNlist();
	clustering();
	nodePlacement(mobile);
	//PathMobileFM_algorithm(mobile);
	delete_kpoints();
	delete_krepresentatives();
}

/*---------------------------------------------------------------------------*/
/*  [Added by Naoum] 
 * 
 * CENTRALIZED FAULT DETECTION MECHANISM 
 *
*/
/*---------------------------------------------------------------------------*/
void resetThreshold(){
    struct table_entry_neighbor *current;
    for(current=list_head(neighbor_table); current!=NULL; current=current->next) 
	current->last_clock = clock_seconds();
}

void resetThresholdNode(int node_id){
    struct table_entry_neighbor *current;
    for(current=list_head(neighbor_table); current!=NULL; current=current->next) 
	if (current->addr.u8[0] == node_id)
		current->last_clock = clock_seconds();
}

static int neighborThreshold2(){
	//if (clock_seconds() < 230 )
	//	return 0; 
	uint32_t limit = 60; 
	bool flag = false;
	// delete_faultytable();
	struct table_entry_neighbor *current;
	for(current=list_head(neighbor_table); current!=NULL; current=current->next) {
		int suspicious =(current->addr.u8[0]);
		if (suspicious==1 || isFaultyNode(suspicious) || current->isSourceNode == false)	
			continue;
		if ((clock_seconds()- current->last_clock) > limit){
			// addFaulty(suspicious);
			flag=true;
			printf("Threshold detection 2: %d ( Decentralized Fault Detection ) \n",suspicious );
		}
    }

	if (flag)
		return 1;
	return 0;
}

static int neighborThreshold(){
	if (clock_seconds() < 260 )
		return 0; 
	uint32_t limit = 200; 
	displayFaultyTable();
	bool flag = false;
	//delete_faultytable();
	struct table_entry_neighbor *current;
	for(current=list_head(neighbor_table); current!=NULL; current=current->next) {
		int suspicious =(current->addr.u8[0]);
		if (suspicious==1 || isFaultyNode(suspicious) || current->isSourceNode == false)	
			continue;
		if ((clock_seconds()- current->last_clock) > limit){
			addFaulty(suspicious);
			flag=true;
			printf("Threshold detection : %d ( Centralized Fault Detection ) \n",suspicious );
		}
    }

	if (flag)
		return 1;
	return 0;
}

/*---------------------------------------------------------------------------*/
/*
 * [Modified]
 * This function is called when an incoming announcement arrives. The
 * function checks the neighbor table to see if the neighbor is
 * already present in the list. If the neighbor is not present in the
 * list, a new neighbor table entry is allocated and is added to the
 * neighbor table.
 */
//put two integers to one
static void received_announcement(struct announcement *a,const linkaddr_t *from,uint16_t id, uint16_t value){
	struct table_entry_neighbor *e;
	struct mobile_table_entry *m;
	if(DEBUG)
		printf("- Sensor %d.%d:Got announcement from sensor %d value %d \n", linkaddr_node_addr.u8[0],linkaddr_node_addr.u8[1],from->u8[0],value);
   
	if(value == -6){ //moving/idle
			for(e = list_head(neighbor_table); e!=NULL; e=e->next){
				if(e->addr.u8[0] == from->u8[0]){
					e->isMobileNeighbor = false;
				}
			}
		}
    else{ //to value periexi kapoio grama stin arxi!        
        char s[11]; 
        sprintf(s,"%d", value); 
        if(strlen(s)!=2){
            int choice = concatenate(s[0] - '0', s[1] - '0');
            int v=s[2] - '0';
            int i=3;
            while(s[i]!='\0'){
                v=concatenate(v, s[i] - '0');
                i++;
            }
            //create the new value without the two first numbers!
            value = v;
            if(DEBUG){
                printf("- Sensor %d.%d:Got announcement from sensor %d value %d \n", linkaddr_node_addr.u8[0],linkaddr_node_addr.u8[1],from->u8[0],value);
                printf("Char array s:");
                int i=0;
                while(s[i]!='\0'){
                    printf("%c",s[i]);
                    i++;
                }
                printf(" ==== %d\n",v);
            }
            if(choice == 72){ //hops!
                if(DEBUG){
                    printf("Hops Announcement!! \n");
                }
                int hops = 0; //arxiki timi
                int third_case = 0;
                //WSP: Find RSSI using packetbuf_attr(PACKETBUF_ATTR_RSSI) => find it from example-neighbor.c
                uint16_t rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI)-45;
                //Set sink info
                linkaddr_t sink;
                sink.u8[0] =1;
                sink.u8[1] = 0; 
                //1st case > an o apostoleas einai o sink tote apexei o apo ton eauto t ane3artita apo ton apostolea
                if (linkaddr_cmp(&linkaddr_node_addr, &sink)){
                    hops = 0;
                }// 2nd case > an o apostoleas einai o sink tote kanoume to hops = 1 dixnontas pws  o current sensor apexei mono 1 hop apo to sink
                else if(linkaddr_cmp(from, &sink)){
                    hops = 1;
                } 
                //3rd case
                else
                    third_case = 1;
                /* We received an announcement from a neighbor so we need to update
                the neighbor list, or add a new entry to the table. */
                for(e = list_head(neighbor_table); e != NULL; e = e->next) {
                    if(linkaddr_cmp(from, &e->addr)) {
                        //o geitonas(from) den ixere poso apexei apo ton sink  -> kai efoson twra xerei (value>=1)
                        if (e->hops == UNKNOWN_HOPS && value>=1) { 
                            e->rssi = rssi;
                            e->hops = value + 1;
                            //Update announcement value
                            best_hop = getBestNeighborHopNum()+1;
                            if((best_hop-1)>my_hops){
                                uint16_t value = concatenate(72,getBestNeighborHopNum());//'H'
                                printf("my hops are: %d",my_hops);
                                announcement_set_value(&example_announcement, value);
                                announcement_bump(&example_announcement);
                            }
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
                    e->flow = false;
                    e->Flag = true;
                    e->next_sequence_packet_number=true;
                    e->num_of_packets_received = 0;
                    e->buffer_occupancy = 0;
                    e->remaining_energy = 0;
                    e->last_clock = 0;
                    e->mobileFlag = false;
                    e->isMobileNeighbor=false;
                    list_add(neighbor_table, e);
                    if(DEBUG) {
                        printf("Insert in the list\n");
                        displayNeighborTable();
                    }
                    //Update announcement value
                    best_hop = getBestNeighborHopNum()+1;
                    if((best_hop-1)>my_hops){
                        uint16_t value = concatenate(72,getBestNeighborHopNum());//'H'
                        printf("my hops are: %d",my_hops);
                        announcement_set_value(&example_announcement, value);
                        announcement_bump(&example_announcement);
                    }
                }
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
                if(DEBUG){
                    printf("Mobile Hops Announcement!! \n");
                }
                int hops = 0; //arxiki timi
                int third_case = 0;
                // WSP: Find RSSI using packetbuf_attr(PACKETBUF_ATTR_RSSI)
                //  => find it from example-neighbor.c
                //uint16_t rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI)-45;
                // Set sink info
                linkaddr_t sink;
                sink.u8[0] =1;
                sink.u8[1] = 0; 
                // 1st case > an o apostoleas einai o sink tote apexei o apo ton eauto t ane3artita apo ton apostolea
                if (linkaddr_cmp(&linkaddr_node_addr, &sink)){
                    hops = 0;
                }// 2nd case > an o apostoleas einai o sink tote kanoume to hops = 1 dixnontas pws 
                //  o current sensor apexei mono 1 hop apo to sink
                else if(linkaddr_cmp(from, &sink)){
                    hops = 1;
                } 
                // 3rd case
                else
                    third_case = 1;
                /* We received an announcement from a neighbor so we need to update
                 the neighbor list, or add a new entry to the table. */
                for(m = list_head(mobile_table); m != NULL; m = m->next) {
                    printf("from: %d.%d -- addr: %d.%d\n",from->u8[0],from->u8[1], m->addr.u8[0], m->addr.u8[1]);
                    if(/*linkaddr_cmp(from, &m->addr)*/from->u8[0] == m->addr.u8[0]) {
                        // o geitonas(from) den ixere poso apexei apo ton sink 
                        //  -> kai efoson twra xerei (value>=1)
                        printf("hops of m: %d \n", m->hops);
                        if (m->hops == UNKNOWN_HOPS /*&& value>=1*/){ 
                            m->hops = value;
                        }
                        if(DEBUG){
                            printf("Update mobile list\n"); 
                            displayMobileTable();
                        }
                  return;
                }
              }
              /* The neighbor was not found in the list, so we add a new entry by
                 allocating memory from the neighbor_mem pool, fill in the
                 necessary fields, and add it to the list. */
                m = memb_alloc(&mobile_mem);
                if(m != NULL) {
                    //3rd case
                    if (third_case == 1) {
                        //knows the distance from the Sink
                        if ( value >= 1 ) { //was value!=0 but announcement_set_value
                            m->hops = value;
                        }
                        //does not know the distance from the Sink
                        else {
                            m->hops = UNKNOWN_HOPS;
                        }
                    } 
                    //1st & 2nd case
                    else {
                        m->hops = hops;
                    }
                    linkaddr_copy(&m->addr, from);
					m->init_pos_x = 0.0;
                    m->init_pos_y = 0.0;
					m->current_pos_x = 0.0;
					m->current_pos_y = 0.0;
                    m->congested_node_addr = -1;
                    m->favailable = true;
					m->idle_status = false;
                    list_add(mobile_table, m);
                    if(DEBUG) {
                        printf("Insert in the mobile list\n");
                        displayMobileTable();
                    }
                }
                // Update announcement value
            }
        } //telioni to if gia ti strlen
        else{} // telioni to else t strlen
    } //telioni to arxiko else!
}

/*---------------------------------------------------------------------------*/
/*
 * [Modified]
 * This function is called at the final recipient of the message.
 */
static void recv(struct multihop_conn *c, const linkaddr_t *sender,const linkaddr_t *prevhop,uint8_t hops){

	struct table_entry_neighbor  *s;
	for(s=list_head(neighbor_table);s!=NULL;s=s->next){
		if(s->addr.u8[0] == sender->u8[0]){
			s->last_clock = clock_seconds();
			break;
		}
	}
    packets_received_by_sink++;
    char str[50];
   strcpy(str,(char *)packetbuf_dataptr());

	if(strstr(str,"Trust me!I am a benign!")!=NULL){
			char *token;
			token = strtok((char *)str,"Trust me!I am a benign! ");

			uint16_t time = clock_seconds();
			uint16_t time_packet = atoi(token);
			uint16_t delay = time - time_packet;

			 printf("wsp Multihop message received from %d.%d '%s' and Delay: %u  (%d)\n",sender->u8[0],sender->u8[1],(char *)packetbuf_dataptr(),delay,hops);
			printf("Packet Received! %u\n",time_packet);
			
	}
	else if(strstr(str,"Level")!=NULL){
			char *token;
			token = strtok((char *)str,"Level");
			int lvl = atoi(token);
			struct table_entry_neighbor  *me;
			printf("Level packet received!!!!! id: %d \n",sender->u8[0]);
			for(me=list_head(neighbor_table);me!=NULL;me=me->next){
				if( me->addr.u8[0] == sender->u8[0]){
					me->hops = lvl;
					break;
				}
			}
	}
	// FAULTY NOTIFICATION MESSAGE
	else if(strstr(str,"FNM ")!=NULL){
		//if (neighborThreshold2()==0)
		//	return;
		if (CENTRALIZED_DETECTION == 1 && DECENTRALIZED_DETECTION == 0)
			return;
		if (REPLACEMENT==1 || (DISCOVERY_STRATEGY== 1 && discoveryFlag==false)){
			decentralized_detection_flag = true;
			char *token,*tok;
			token = strtok((char *)str,"FNM ");
			int faulty,faulty_lvl;
			tok = strtok(token,",");
			faulty = atoi(tok);
			if (faulty==2)
				return;
			tok = strtok(NULL,",");
			faulty_lvl =  atoi(tok);
			printf("DEBUGG Faulty Notification Message packet received!!!!! id: %d level %d  from: %d (Fault Reporting Mechanism)\n",faulty,faulty_lvl, sender->u8[0]);
			if (isFaultyNode(faulty)==false){
				addFaulty(faulty);
				sendMobileInvestigate(faulty,faulty_lvl);
			}
		}
	}
	// INVESTIGATION RESULTS NOTIFICATION MESSAGE
	else if(strstr(str,"IRNM ") != NULL){	
		if (REPLACEMENT==1 || DISCOVERY_STRATEGY==1){
			if (DEBUGFM)
				printf("Investigation packet received!!!!! id: %d  (Mobile Investigation Mechanism)\n",sender->u8[0] );
			char *token, *tok;
			int counter=0, neighbors[MAX_NODE_NEIGHBORS+1];
			token = strtok((char *)str,"IRNM ");
			tok = strtok(token,",");
			while (tok != NULL){
				neighbors[counter++]= atoi(tok);
				tok = strtok(NULL,",");
			}
			if (DISCOVERY_STRATEGY == 1)
				startMobileNodeDiscover(sender->u8[0],neighbors,counter);
			else 
				sendMobileNodeReplacement(sender->u8[0],neighbors,counter);
		}
	}
	//Discovery Notification Message
	else if(strstr(str,"DNM ")!=NULL){	
			if (DEBUGFM)
				printf("DNM packet received!!!!! id: %d  (FT recovery)\n",sender->u8[0] );
			char *token, *tok;
			//int faultyNodes[MAX_NODE_NEIGHBORS+1], facounter=0;
			int num,id,upper;
			token = strtok((char *)str,"DNM ");
			tok = strtok(token,",");
			while (tok != NULL){
				num = atoi(tok);
				id = num/10;
				upper = num%10;
				printf("Node : num %d id %d upper %d \n",num,id,upper);
				if ( upper == 1)
					upperNodes[ucounter++] = id;
				else  
					foundedNodes[fcounter++] = id;
				tok = strtok(NULL,",");
			}

			int i,s;
			for (i=0;i<ucounter;i++){
				printf("Upper node : %d \n",upperNodes[i]);
				resetThresholdNode(upperNodes[i]);
				struct Point p = searchnodesposition(upperNodes[i]);
				addKPoint(upperNodes[i],p.x,p.y);
				//addTarget(upperNodes[i]);
			}

			for (i=0;i<fcounter;i++)
				printf("Founded node : %d \n",foundedNodes[i] );

			for (i=0;i<scounter;i++)
				printf("Suspicious node : %d \n",suspiciousFaulty[i] );
			
			for (s=0;s<scounter;s++){
				bool find = false;
				for (i=0;i<ucounter;i++)
					if (upperNodes[i] == suspiciousFaulty[s])
						find=true;
				for (i=0;i<fcounter;i++)
					if (foundedNodes[i] == suspiciousFaulty[s])
						find=true;
				if (find ==false){
					printf("DNM Faulty node : %d \n",suspiciousFaulty[s] );
					addFaulty(suspiciousFaulty[s]);
					/*
					if (isSource(suspiciousFaulty[s])){
						printf("DNM Faulty node Source : %d \n",suspiciousFaulty[s] );
						struct Point p = searchnodesposition(suspiciousFaulty[s]);
						addKPoint(suspiciousFaulty[s],p.x,p.y);
					}			*/
				}
			}
			clearSuspiciousFaultyNodes();
			displayFaultyTable();
			int mobile = sender->u8[0];
			mobileNodePlacement(mobile);
			decentralized_detection_flag = false;
			if (CENTRALIZED_DETECTION ==1)
				mustcheck=true;
			my_counter = 0;
	}

}

/*---------------------------------------------------------------------------*/
/*  [Added by Naoum] 
 * 
 * UNICAST RECEIVE FUNCTION
 *
 * This function is been used to sink be able to receive messages by mobile nodes, especially reports about the discovery. 
 * When a mobile node will discover an area, then it comes backs to its initialized position and send a unicast message to the sink node about its findings.
 *
 */
/*---------------------------------------------------------------------------*/
static void recv_uc(struct unicast_conn *c, const linkaddr_t *sender){
	printf("unicast message received from %d.%d\n", sender->u8[0], sender->u8[1]);

	if (wait_results == false)
		return;
	else 
		wait_results = false;
	char str[50];
	strcpy(str,(char *)packetbuf_dataptr());

	if (strstr(str, "DNM") != NULL){
		updatemobilenodeAvailable(sender->u8[0]);
		if (DEBUGFM)
				printf("DNM packet received!!!!! id: %d  (FT recovery)\n",sender->u8[0] );
			char *token, *tok;
			int upperNodes[MAX_NODE_NEIGHBORS+1], ucounter=0;
			int foundedNodes[MAX_NODE_NEIGHBORS+1], fcounter=0;
			// int faultyNodes[MAX_NODE_NEIGHBORS+1], facounter=0;
			int num,id,upper;
			token = strtok((char *)str,"DNM ");
			tok = strtok(token,",");
			while (tok != NULL){
				num = atoi(tok);
				id = num/10;
				upper = num%10;
				printf("Node : num %d id %d upper %d \n",num,id,upper);
				if ( upper == 1)
					upperNodes[ucounter++] = id;
				else  
					foundedNodes[fcounter++] = id;
				tok = strtok(NULL,",");
			}
			int i,s;
			for (i=0;i<ucounter;i++){
				printf("Upper node : %d \n",upperNodes[i]);
				resetThresholdNode(upperNodes[i]);
				struct Point p = searchnodesposition(upperNodes[i]);
				addKPoint(upperNodes[i],p.x,p.y);
			}

			for (i=0;i<fcounter;i++){
				printf("Founded node : %d \n",foundedNodes[i]);
				resetThresholdNode(foundedNodes[i]);
			}

			for (i=0;i<scounter;i++)
				printf("Suspicious node : %d \n",suspiciousFaulty[i] );
			
			for (s=0;s<scounter;s++){
				bool find = false;
				for (i=0;i<ucounter;i++)
					if (upperNodes[i] == suspiciousFaulty[s])
						find=true;
				for (i=0;i<fcounter;i++)
					if (foundedNodes[i] == suspiciousFaulty[s])
						find=true;
				if (find ==false){
					printf("DNM Faulty node : %d \n",suspiciousFaulty[s] );
					addFaulty(suspiciousFaulty[s]);
					if (isSource(suspiciousFaulty[s]) == false)
						deleteNeighbor(suspiciousFaulty[s]);
					//if (isSource(suspiciousFaulty[s])){
					//	struct Point p = searchnodesposition(suspiciousFaulty[s]);
					//	addKPoint(suspiciousFaulty[s],p.x,p.y);
					//}			
				}
			}

			displayFaultyTable();
			//int mobile = sender->u8[0];
			mobileNodePlacement(1);
			centralized_detection_flag = false;
	}
	
	
}
/*---------------------------------------------------------------------------
  Sink does not forward packets :)
---------------------------------------------------------------------------*/
static linkaddr_t *forward(struct multihop_conn *c,const linkaddr_t *originator, const linkaddr_t *dest,const linkaddr_t *prevhop, uint8_t hops){
    return NULL;
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
    //skip white space
    while (isspace(*s))
        ++s;
    const bool valuesign = parse_sign(&s); // sign of the number
    double value = parse_digits(&s, NULL);
    if (*s == '.') {
        int d;                  // number of digits in fraction
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
    // else, we have an exponent; parse its sign and value
    const double exponentsign = parse_sign(&s) ? 10. : .1;
    int exponent = parse_digits(&s, NULL);
    while (exponent--)
        value *= exponentsign;
    return value;
}
/*---------------------------------------------------------------------------*/
/*  [Added by Naoum] 
 * 
 * INITIALIZATION FUNCTIONS
 *
 */
/*---------------------------------------------------------------------------*/

/*  [Added by Naoum] 
 * 
 *  Given the node's id and position, the node is added to the neighbor table with the defaults values 
 *
 */
void insertNeighborTable(int node_id, float pos_x, float pos_y){
	struct table_entry_neighbor *e;
	e = memb_alloc(&neighbor_mem);
	e->hops = UNKNOWN_HOPS;
	e->addr.u8[0]  = node_id;
	e->addr.u8[1]  = 0;
	e->flow = false;
	e->Flag = true;
	e->pos_x= pos_x;
	e->pos_y= pos_y;
	e->next_sequence_packet_number=true;
	e->num_of_packets_received = 0;
	e->buffer_occupancy = 0;
	e->remaining_energy = 0;
	e->last_clock = 0;
	// CHANGE IF NEEDED - BASED ON SCENARIO
	if (node_id > (NUM_RELAY+1))
		e->isSourceNode = true;
	else 
		e->isSourceNode = false;
	list_add(neighbor_table, e);
} 

/*   [Added by Naoum] 
 * 
 *  Given the mobile node's id and position, the mobile node is added to the mobile table with the defaults values 
 *
 */
void insertMobileTable(int node_id, float pos_x, float pos_y){
	struct mobile_table_entry *m;
	m = memb_alloc(&mobile_mem);
   	m->hops = UNKNOWN_HOPS;
    	m->addr.u8[0]  = node_id;
    	m->addr.u8[1]  = 0;
    	m->init_pos_x = pos_x;
    	m->init_pos_y= pos_y;
	m->current_pos_x = pos_x;
    	m->current_pos_y = pos_y;
	m->congested_node_addr = 0;
	// TEST CLUSTERING
	//if (node_id == 26)
	//	m->favailable = false;
	//else
	m->favailable = true;
	m->idle_status = false;
	list_add(mobile_table, m);
} 

static const struct multihop_callbacks multihop_call = {recv, forward};
static struct multihop_conn multihop;
static const struct unicast_callbacks unicast_callbacks = {recv_uc};


/*---------------------------------------------------------------------------*/
/*  [Added by Naoum] 
 * 
 * INITIALIZATION PROCESS
 *
 */
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(initialization, ev, data){
    PROCESS_BEGIN();

     /* Initialize the memory for the lists. */
	memb_init(&neighbor_mem);
	memb_init(&discovery_mem);
	memb_init(&kpoints_mem);
	memb_init(&krepresentatives_mem);
	memb_init(&faulty_mem);
	memb_init(&pointList_mem);
	memb_init(&targetList_mem);
	memb_init(&bestPathList_mem);
	memb_init(&positionsList_mem);

	 /* Initialize the lists */
	list_init(neighbor_table);
	list_init(mobile_table);
	list_init(discovery_positions);
	list_init(kpoints_table);
	list_init(krepresentatives_table);
	list_init(pointList);
	list_init(targetList);
	list_init(bestPathList);
	list_init(positionsList);

	printf("Initialization begins\n");

	while(cur_node<=num_nodes){

		printf("INIT_NEIGHBORS\n");
		printf("current: %d\n", cur_node);

		// get the id of the node
		PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message);
		node_idd = atoi((char *)data);

		if (DEBUG_ALL)
			printf("Read id  %d \n",node_idd);

		// get the x coordinate of the node
		PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message);
		node_pos_x = extended_atof((char *)data);
		if (DEBUG_ALL)
			printf("pos_x = %ld.%03u\n",(long)node_pos_x,(unsigned)((node_pos_x-floor(node_pos_x))*1000));

		// get the y coordinate of the node
		PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message);
		node_pos_y = extended_atof((char *)data);
		if (DEBUG_ALL)
			printf("pos_x = %ld.%03u\n",(long)node_pos_y,(unsigned)((node_pos_y-floor(node_pos_y))*1000));

		// sink node position
		if(node_idd==1){
			node_loc_x = node_pos_x;
			node_loc_y = node_pos_y;
		}
		// source and relay nodes' positions
		else if (node_idd<=(NUM_SOURCE+NUM_RELAY+1)){
			insertNeighborTable(node_idd, node_pos_x,node_pos_y);
			//TESTING CLUSTERING
			//addKPoint(node_idd, node_pos_x,node_pos_y);
		}
		// mobile nodes' positions
		else 
			insertMobileTable(node_idd, node_pos_x,node_pos_y);

		//etimer_set(&et, 1 * CLOCK_SECOND);
		//PROCESS_WAIT_UNTIL(etimer_expired(&et));
		cur_node++;
	}

	if (DEBUGFM){
		displayNeighborTable();
		displayMobileTable();
	}

	// TESTING CLUSTERING
	//clustering();

	process_start(&multihop_process,NULL);

  	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
/*  [Added by Naoum] 
 * 
 *  MAIN PROCESS
 *  
 *  SINK RECEIVES MESSAGES FROM SOURCE NODES
 *  
 *  SINK MANAGES FAULTS - FAULT DETECTION, FAULT DIAGNOSIS AND FAULT RECOVERY
 *  
 */
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(multihop_process, ev, data){
	PROCESS_EXITHANDLER(multihop_close(&multihop);)
	PROCESS_EXITHANDLER(unicast_close(&uc);)
	PROCESS_BEGIN();

	printf("PROCESS MULTIHOP BEGINS\n");
     
	 /* Open a multihop connection on Rime channel CHANNEL. */
	multihop_open(&multihop, CHANNEL, &multihop_call);
	printf("Multihop connection opened\n");

	/* Register an announcement with the same announcement ID as the
	Rime channel we use to open the multihop connection above. */
	announcement_register(&example_announcement,130,received_announcement);
	/* Set a dummy value to start sending out announcments. */
  	announcement_set_value(&example_announcement, 0);
	printf("Announcement connection registered\n");

	// unicast connection
	unicast_open(&uc, 146, &unicast_callbacks);
	printf("Unicast conncetion registered\n");

    	// Sink node sends announcement to build the topology
	uint16_t value = concatenate(72,0);
	announcement_set_value(&example_announcement, value);
	announcement_bump(&example_announcement);

	// TESTING PLACEMENT ALGORITHM
	//mobileNodePlacementTest(26);

	/* Allow some time for the network to settle. */
	etimer_set(&et, 60 * CLOCK_SECOND);
	PROCESS_WAIT_UNTIL(etimer_expired(&et));
     
	/* Loop forever.*/
	while(1) {

	      etimer_set(&et, CLOCK_SECOND*10);
	      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
	      announcement_listen(1); 
		
		// if the discoveryFlag is enabled, means that a node is discovering for other failures (based on centralized or decentralized detection)
		// sink send the mobile node to a different position based on the time
		if (discoveryFlag && (clock_seconds()-discoveryStart)>=discoveryWait)
			sendMobileNodeDiscover();

		my_counter++;
		
		// Sink check for failures every minute (sixty seconds)
		// It s not efficient to check frequently and it can be error-prone
		// my_counter is helping the sink to count the sixty seconds
		// clock_seconds()<1400 && 
		if ( ( (mustcheck) || (CENTRALIZED_DETECTION==1 && my_counter>=6 && decentralized_detection_flag==false && centralized_detection_flag==false))){
			my_counter = 0;
			mustcheck=false;
			printf("Checking... ( Centralized Detection ) \n" );
			if (neighborThreshold() == 1){
				wait_results = true;
				centralized_detection_flag = true;
				navigationAlgorithm2();
				printf("Problem occured! ( Centralized Detection ) \n");
			}
		}
    }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
// every twenty seconds, sink should print its energy
PROCESS_THREAD(energy, ev, data){
	PROCESS_BEGIN();

	while(1){
		remaining_energy = (energest_type_time(ENERGEST_TYPE_TRANSMIT) * 19.5 + energest_type_time(ENERGEST_TYPE_LISTEN) *21.8 + energest_type_time(ENERGEST_TYPE_CPU) * 1.8 + energest_type_time(ENERGEST_TYPE_LPM) * 0.0545 ) * 3 / 4096 *8;
		printf("Time: \t %lu \t Remain energy \t %ld.%03u\n",clock_time(),(long)remaining_energy,(unsigned)((remaining_energy-floor(remaining_energy))*1000));
		etimer_set(&timer_energy, CLOCK_SECOND*20);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer_energy));
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
