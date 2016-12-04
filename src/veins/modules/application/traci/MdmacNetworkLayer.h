//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef __CLUSTERLIB_CLUSTERNETWORKLAYER_H_
#define __CLUSTERLIB_CLUSTERNETWORKLAYER_H_

//#include <omnetpp.h>
//
//#include <MiXiMDefs.h>
//#include <BaseNetwLayer.h>

//#include <fstream>
#include <set>
#include <map>

#include "veins/modules/messages/MdmacControlMessage_m.h"
#include "veins/modules/messages/WaveShortMessageWithDst_m.h"
#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"

#include "ClusterAlgorithm.h"

//#define BEAT_LENGTH	0.25	// measured in second.
#define BEAT_LENGTH	2	// measured in second.

using Veins::AnnotationManager;
using Veins::TraCIMobility;
using Veins::TraCIMobilityAccess;
using Veins::TraCIScenarioManager;
using Veins::TraCIScenarioManagerAccess;
using Veins::TraCICommandInterface;

/**
 * This module implements the clustering mechanism for
 * Modified Distributed Mobility-Aware Clustering (M-DMAC).
 * There is a pure virtual function that must be implemented by
 * inheriting cluster modules, which provides the weighting function
 * for the particular cluster algorithm being tested.
 */
class MdmacNetworkLayer : public ClusterAlgorithm
{

public:

	/**
	 * @brief Messages used by the clustering mechanism.
	 */
	enum ClusterMessageKinds {
		HELLO_MESSAGE = LAST_BASE_APPL_MESSAGE_KIND + 1,	/**< Periodic beacon used to update node weights. */
		CH_MESSAGE,										/**< Announcement that a node has become a Cluster Head (CH). */
		JOIN_MESSAGE,									/**< Announcement of a node's intention to join a CH. */
		DATA,											/**< A datagram. */
		LAST_CLUSTER_MESSAGE_KIND
	};


    typedef std::list<std::string> RouteLinkList;

	struct Neighbour {
		double mWeight;						/**< Weight of this node. */
		Coord mPosition;					/**< Position of the Neighbour. */
		Coord mVelocity;					/**< Velocity of the Neighbour. */
		std::string mRoadID;				/**< The ID of the road this car is on. */
		std::string mLaneID;				/**< The ID of the lane this car is on. */
		Coord mDestination;					/**< The node's destination. */
		RouteLinkList mRouteLinks;			/**< The next N links in this neighbour's route. */
		bool mIsClusterHead;				/**< Is this node a CH? */
		unsigned int mFreshness;			/**< How long this node will stay in range of this neighbour. Measured in beats. */
	};

protected:

	TraCIScenarioManager* mTraciManager;

	typedef std::map<unsigned int,Neighbour> NeighbourSet;
	typedef std::map<unsigned int,Neighbour>::iterator NeighbourIterator;

	//unsigned int mID;						/**< Node's unique ID. */
	double mWeight;							/**< Weight of this node. */

	std::string mRoadID;					/**< The ID of the road this car is on. */
	std::string mLaneID;					/**< The ID of the lane this car is on. */

	bool mIncludeDestination;				/**< Include the destination in the HELLO messages. */

	bool mIsClusterHead;					/**< Is this node a CH? */
	NeighbourSet mNeighbours;				/**< The set of neighbours near this node. */

	double mTransmitRangeSq;				/**< Required for the freshness calculation. Obtained from the PhyLayer module. */

	bool mInitialised;						/**< Set to true if the init function has been called. */

    /**
     * @name Messages
     * @brief Messages this module sends itself.
     *
     * These are used for timed events.
     *
     **/
    /*@{*/

	cMessage *mFirstInitMessage;			/**< Run the cluster init function for the first time. */
	cMessage *mSendHelloMessage;			/**< Send a HELLO message. */
	cMessage *mBeatMessage;					/**< Process the neighbour table for out-of-date node entries. */
	cMessage *mSendData;

    /*@}*/


    /**
     * @name Parameters
     * @brief Configurations for the module.
     *
     * These variables are set in the simulation configuration.
     *
     **/
    /*@{*/

	unsigned int mInitialFreshness;			/**< The initial node freshness (measured in beats). */
	unsigned int mFreshnessThreshold;		/**< The minimum freshness for which a node is eligible to be a CH. */
	double mAngleThreshold;					/**< The maximum angle between the directions of this node and another node for it to be considered for CH. */
	unsigned int mHopCount;					/**< The number of hops for cluster control messages. */
	double mBeaconInterval;					/**< The interval between each HELLO message. */

    /*@}*/




public:
    //Module_Class_Members(BaseNetwLayer,BaseLayer,0);
	MdmacNetworkLayer() : ClusterAlgorithm() {}


    /** @brief Initialization of the module and some variables*/
    virtual void initialize(int);

    /** @brief Cleanup*/
    virtual void finish();


	int GetStateCount();
	int GetClusterState();
	bool IsClusterHead();
	bool IsSubclusterHead();
	bool IsHierarchical();
	void UpdateMessageString();
	int GetMinimumClusterSize();

	void ClusterStarted();
	void ClusterDied( int deathType );

protected:
	AnnotationManager* annotations;
	BaseConnectionManager* bcm;

	double recvDataLength;
	double sendDataLength;
	double forwardDataLength;

	double sendNearPosibility;
	double sendNodePercent;
	unsigned long packetSentInterval;
	unsigned long packetSentIntervalBeg;

	int sequenceNum;
	unsigned long packetLenMin;
	unsigned long packetLenMax;

	virtual void onBeacon(WaveShortMessage* wsm);
	virtual void onData(WaveShortMessage* wsm);
	virtual void handlePositionUpdate(cObject* obj);
	virtual void handleParkingUpdate(cObject* obj);
	virtual void sendWSM(WaveShortMessage* wsm);
	virtual MdmacControlMessage* prepareWSMCB(int kind, int dest, int nHops);

    /** @brief Handle messages from upper layer */
    //virtual void handleUpperMsg(cMessage* msg);

    /** @brief Handle messages from lower layer */
    //virtual void handleLowerMsg(cMessage* msg);

    /** @brief Handle self messages */
    virtual void handleSelfMsg(cMessage* msg);

    /** @brief decapsulate higher layer message from ClusterControlMessage */
    //virtual cMessage* decapsMsg(MdmacControlMessage*);

    /** @brief Encapsulate higher layer packet into a ClusterControlMessage */
    //virtual NetwPkt* encapsMsg(cPacket*);

    /** @brief Compute the CH weight for this node. */
    virtual double calculateWeight();

	/** Add the destination data to a packet. */
	virtual int AddDestinationData( MdmacControlMessage *pkt );

	/** Store the destination data from a packet. */
	virtual void StoreDestinationData( MdmacControlMessage *pkt );

	/**
     * @name Cluster Methods
     * @brief Methods handling the formation and maintenance of clusters.
     *
     * These methods are implementations of the M-DMAC functions as specified
     * in the paper "Modified DMAC Clustering Algorithm for VANETs" by G. Wolny
     *
     **/
    /*@{*/

    /** @brief Initiate clustering. */
    void init();

    /** @brief Process the neighbour table in one beat. Also, update the node's weight. */
    void processBeat();

    /** @brief Select a CH from the neighbour table. */
    int chooseClusterHead();

    /** @brief Handle a link failure. Link failure is detected when a CMs freshness reaches 0. */
    void linkFailure( unsigned int );

    /** @brief Calculate the freshness of the given neighbour. */
    void calculateFreshness( unsigned int );

    /** @brief Determine whether the given node is a suitable CH. */
    bool testClusterHeadChange( unsigned int );

    /** @brief Handle a HELLO message. */
    void receiveHelloMessage( MdmacControlMessage* );

    /** @brief Handle a CH message. */
    void receiveChMessage( MdmacControlMessage* );

    /** @brief Handle a JOIN message. */
    void receiveJoinMessage( MdmacControlMessage* );

    /** @brief Update a neighbour's data. */
    void updateNeighbour( MdmacControlMessage* );

    /** @brief Sends a given cluster message. */
    void sendClusterMessage( int, int = -1, int = -1 );

    /*@}*/

	inline bool getRandomPermit(double p) {
			ASSERT(p >= .0 && p <= 1.0);
			return uniform(0, 1) <= p;
	}

};

#endif
