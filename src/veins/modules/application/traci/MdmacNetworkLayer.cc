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

#include <cassert>
#include <iostream>
using std::cout;
using std::endl;

//#include "ClusterAnalysisScenarioManager.h"
#include "MdmacNetworkLayer.h"
using Veins::TraCIScenarioManager;

Define_Module(MdmacNetworkLayer);

#define expEV EV

std::ostream& operator<<( std::ostream& os, const MdmacNetworkLayer::Neighbour& n ) {

	os << "Weight = " << n.mWeight << "; Pos = " << n.mPosition << "; Vel = " << n.mVelocity << "; C" << ( n.mIsClusterHead ? "H" : "M" ) << "; freshness = " << n.mFreshness;
	return os;

}

MdmacNetworkLayer::NodeVector
MdmacNetworkLayer::getFarNodes(NeighborNodeSet* nns, TraCIScenarioManagerLaunchd* traciSMLd,
			const BaseConnectionManager* const bcm,
			const cModule* host) {

	ASSERT(nns != NULL && traciSMLd != NULL && bcm != NULL && host != NULL);
	const std::map<std::string, cModule*> &hosts = traciSMLd->getManagedHosts();

	MdmacNetworkLayer::NodeVector result;
	result.reserve(hosts.size() - nns->size());

	for (auto iter = hosts.begin(); iter != hosts.end(); ++iter) {
		if (nns->find(iter->second) == nns->end()) {
			result.push_back(iter->second);
		}
	}

	return result;
}

void MdmacNetworkLayer::initialize(int stage)
{
	ClusterAlgorithm::initialize(stage);

    if(stage == 0) {

    	mId = findHost()->getId();
		ApplMapManager::getApplMapManager()->registerAppl(mId, this);

		mobility = TraCIMobilityAccess().get(getParentModule());
    	mInitialised = false;

    	// set up the node.
    	//mId = getId();
    	mWeight = calculateWeight();
    	//mMobility = FindModule<BaseMobility*>::findSubModule(findHost());
		mMobility = TraCIMobilityAccess().get(getParentModule());
    	mClusterHead = -1;
    	mIsClusterHead = false;
		mIsGateWay = false;
    	mCurrentMaximumClusterSize = 0;
    	mClusterStartTime = 0;

		bcm = FindModule<BaseConnectionManager*>::findGlobalModule();
		ASSERT(bcm != NULL);

		// register nic to connection manager
		Coord pos = mobility->getCurrentPosition();
		bcm->registerNic(this->getParentModule()->getSubmodule("nic"), (ChannelAccess*) this->getParentModule()->getSubmodule("nic")->getSubmodule("phy80211p"), &pos);

    	//mTransmitRangeSq = pow( channelAccess->getConnectionManager( channelAccess->getParentModule() )->getMaxInterferenceDistance(), 2 );
    	mTransmitRangeSq = pow( bcm->getMaxInterferenceDistance(), 2 );

    	// load configurations
    	mInitialFreshness = par("initialFreshness").longValue();
    	mFreshnessThreshold = par("freshnessThreshold").longValue();
    	mAngleThreshold = par("angleThreshold").doubleValue() * M_PI;
    	mHopCount = par("hopCount").longValue();
    	mBeaconInterval = par("beaconInterval").doubleValue();

		mSendData = NULL;
		mSendHelloMessage = NULL;
		mFirstInitMessage = NULL;
		mBeatMessage = NULL;
		mWaitSendData = new cMessage();

    	// set up watches
    	WATCH_SET( mClusterMembers );
    	WATCH_MAP( mNeighbours );
    	WATCH( mWeight );
    	WATCH( mClusterHead );

		mIncludeDestination = false;

		mTraciManager = TraCIScenarioManagerLaunchdAccess().get();
		mClusterManager = ClusterManager::getClusterManager();

		/**
		 * init cluster info
		 * also mNeighbourClusters is an empty set
		 */
		//mClusterManager->nodeNeighbourClusterInfoUpdate(mId, &mNeighbourClusters);
		getNeighbourClusters(true);

		sequenceNum = 0;
		sendNearPosibility = hasPar("sendNearPosibility") ?
			par("sendNearPosibility") : .5;

		sendNodePercent = hasPar("sendNodePercent") ?
			par("sendNodePercent") : .5;

		packetSentInterval = hasPar("packetSentInterval") ?
			par("packetSentInterval") : 10;

		packetSentIntervalBeg = hasPar("packetSentIntervalBeg") ?
			par("packetSentIntervalBeg") : 100;

		packetLenMin = hasPar("packetLenMin") ?
			par("packetLenMin") : 1024;

		packetLenMax = hasPar("packetLenMax") ?
			par("packetLenMax") : 1024;

		packetDelay.setName("packet_delay");
		packetPathLen.setName("packet_path_length");

//     	TraCIScenarioManager *pManager = TraCIScenarioManagerAccess().get();
//     	char strNodeName[50];
//     	sprintf( strNodeName, "node%i_conn", mId );
//     	std::list<Coord> points;
//     	points.push_back( mMobility->getCurrentPosition() );
//     	pManager->commandAddPolygon( strNodeName, "clusterConn", TraCIScenarioManager::Color( 255, 0, 0, 255 ), true, 5, points );

		annotations = AnnotationManagerAccess().getIfExists();
		ASSERT(annotations);
    } else if (stage == 1) {
		if (sendData && packetSentInterval > 0) {
			//scheduleAt(simTime() + packetSentInterval, &sendMessageSignal);
			mSendData = new cMessage();
			scheduleAt(simTime() + packetSentIntervalBeg + uniform(0, packetSentInterval), mSendData);
		}
    	// set up self-messages
    	mSendHelloMessage = new cMessage();
    	scheduleAt( simTime() + mBeaconInterval * float(rand()) / RAND_MAX, mSendHelloMessage );
    	mFirstInitMessage = new cMessage();
    	scheduleAt( simTime(), mFirstInitMessage );
    	mBeatMessage = new cMessage();
    	scheduleAt( simTime() + BEAT_LENGTH * float(rand()) / RAND_MAX, mBeatMessage );
	}

}


/** @brief Cleanup*/
void MdmacNetworkLayer::finish() {
	//if ( IsClusterHead() ) {
	//  ClusterDied( CD_Cannibal );
	//}
	
	if (mIsClusterHead) {
	  ClusterDied( CD_Cannibal );
	}

	mClusterManager->leaveCluster(mClusterHead, mId, simTime().dbl());
	mClusterManager->nodeNeighbourClusterInfoDelete(mId);
	mClusterManager->nodeFinished(mId);

	ApplMapManager::getApplMapManager()->unregisterAppl(mId);
	if (mSendHelloMessage && mSendHelloMessage->isScheduled() )
		cancelEvent( mSendHelloMessage );
	delete mSendHelloMessage;

	if ( mFirstInitMessage && mFirstInitMessage->isScheduled() )
		cancelEvent( mFirstInitMessage );
	delete mFirstInitMessage;

	if ( mBeatMessage && mBeatMessage->isScheduled() )
		cancelEvent( mBeatMessage );
	delete mBeatMessage;

	if ( mSendData && mSendData->isScheduled() ) {
		cancelEvent( mSendData);
	}
	delete mSendData;

	if (mWaitSendData && mWaitSendData->isScheduled()) {
		cancelEvent(mWaitSendData);
	}
	delete mWaitSendData;
	if (!msgQueue.empty()) {
		WaveShortMessageWithDst* m = msgQueue.front();
		delete m;
		msgQueue.pop_front();
	}

// 	TraCIScenarioManager *pManager = TraCIScenarioManagerAccess().get();
// 	char strNodeName[10];
// 	sprintf( strNodeName, "node%i_conn", mId );
// 	pManager->commandRemovePolygon( strNodeName, 5 );

	//std::cerr << "Node " << mId << " deleted.\n";
	ClusterAlgorithm::finish();

}

bool MdmacNetworkLayer::isVehicleAlive(string id) {

	const std::map<std::string, cModule*> &hosts = mTraciManager->getManagedHosts();
	auto iter = hosts.find(id);
	if (iter == hosts.end()) {
		return false;
	}
	return true;
}


void MdmacNetworkLayer::onBeacon(WaveShortMessage* wsm) {

    MdmacControlMessage *m = dynamic_cast<MdmacControlMessage *>(wsm);
	ASSERT(m != NULL);
    coreEV << " handling packet from " << m->getSenderAddress() << std::endl;

    switch( m->getKind() ) {

		case HELLO_MESSAGE:
			receiveHelloMessage( m );
			break;

		case CH_MESSAGE:
			receiveChMessage( m );
			break;

		case JOIN_MESSAGE:
			receiveJoinMessage( m );
			break;

		case DATA:
			ASSERT(false);
			//sendUp( decapsMsg( m ) );
			break;

		default:
		    coreEV << " received packet from " << m->getSenderAddress() << " of unknown type: " << m->getKind() << std::endl;
		    //delete msg;
		    break;

    };
}

void MdmacNetworkLayer::onData(WaveShortMessage* wsm) {
	WaveShortMessageWithDst* wsmd = dynamic_cast<WaveShortMessageWithDst*>(wsm);
	ASSERT(wsmd != NULL);

	cModule *host = findHost();
	findHost()->getDisplayString().updateWith("r=16,green");
	host->getDisplayString().updateWith("r=16,green");
	annotations->scheduleErase(1, annotations->drawLine(wsm->getSenderPos(), mobility->getPositionAt(simTime()), "blue"));

	const std::map<std::string, cModule*> &hosts = mTraciManager->getManagedHosts();

	/**
	 * I'm the dst node
	 */
	if (findHost()->getId() == wsmd->getRecipientAddress()) {

		recvDataLength += wsmd->getByteLength();

		expEV << simTime().dbl() << " " 
			<< wsmd->getSenderAddress() << " " << wsmd->getSerial() << " packet arrived at: " << wsmd->getRecipientAddress()
			<< std::endl
			<< "current hosts number: " << hosts.size() << std::endl;
		PathQueue& pq = wsmd->getPathNodes();

		expEV << wsmd->getSenderAddress() << "-->";
		unsigned long len = pq.size() + 1;
		while (!pq.empty()) {
			expEV << pq.front() << "-->";
			pq.pop();
		}
		expEV << wsmd->getRecipientAddress() << std::endl;
		expEV << "length of path: " << len << std::endl;

		packetDelay.record(simTime() - wsmd->getTimestamp());
		packetPathLen.record(len);

		return;
	}

	/**
	 * If the dst node is out of our study region
	 * Record data and drop packet
	 */
	std::string dstNodeId = wsmd->getDstNodeId();

	//auto iter = hosts.find(dstNodeId);
	//if (iter == hosts.end()) {
	if (!isVehicleAlive(dstNodeId)) {
		expEV << "[Shit] " << dstNodeId << " is out of our study region" << std::endl;
		return;
	}

	if (findHost()->getId() == wsmd->getNextHopId()) {

		int nextHopId = getNextHopId(wsmd->getRecipientAddress());
		expEV << wsmd->getSenderAddress() << " send to "
			<< wsmd->getRecipientAddress() << " via "
			<< mId << " next hop: " << nextHopId << endl;

		/**
		 * Currently, no neighbor can help send msg
		 * push to buffer
		 */
		if (nextHopId < 0) {
			wsmd->getPathNodes().push(wsmd->getNextHopId());
			msgQueue.push_back(wsmd->dup());

			ASSERT(mWaitSendData);
			if (!mWaitSendData->isScheduled()) {
				scheduleAt(simTime() + 1, mWaitSendData);
			}
			return;
		}
		wsmd->getPathNodes().push(wsmd->getNextHopId());
		wsmd->setNextHopId(nextHopId);

		forwardDataLength += wsmd->getByteLength();

		sendWSM(wsmd->dup());

	} else {
		/**
		 * I'm not the next hop, do nothing, just drop it
		 */
	}
}

int MdmacNetworkLayer::GetStateCount() {
	return 4;
	//return 3;
}

int MdmacNetworkLayer::GetClusterState() {
	if ( !IsClusterHead() ) {
		if ( mClusterHead == -1 ) {
			return SINGLE;	// Unclustered state
		}
		else if (IsClusterGateWay()) {
			return GATEWAY;
		} else {
			return MEMBER;	// Cluster Member
		}
	} else {
		if (IsClusterGateWay()) {
			return HEAD | GATEWAY;		// Cluster Head
		} else {
			return GATEWAY;
		}
	}
}


bool MdmacNetworkLayer::IsClusterHead() {
	return mIsClusterHead && ( mClusterMembers.size() > 1 );
}

bool MdmacNetworkLayer::IsClusterGateWay() {

	return mClusterHead != -1 && getNeighbourClusters(true)->size() > 1;
}


bool MdmacNetworkLayer::IsSubclusterHead() {
	return false;
}


bool MdmacNetworkLayer::IsHierarchical() {
	return false;
}

void MdmacNetworkLayer::UpdateMessageString() {

}


int MdmacNetworkLayer::GetMinimumClusterSize() {
	return 1;
}

void MdmacNetworkLayer::ClusterStarted() {
	ClusterAlgorithm::ClusterStarted();
	mIsClusterHead = true;

}


void MdmacNetworkLayer::ClusterDied( int deathType ) {
	ClusterAlgorithm::ClusterDied( deathType );
	mIsClusterHead = false;

	if ( deathType == CD_Attrition )
		init();
}


/** @brief Handle messages from upper layer */
//void MdmacNetworkLayer::handleUpperMsg(cMessage* msg) {
//
//	assert(dynamic_cast<cPacket*>(msg));
//    sendDown(encapsMsg(static_cast<cPacket*>(msg)));
//
//}



/** @brief Handle messages from lower layer */
//void MdmacNetworkLayer::handleLowerMsg(cMessage* msg) {
//
//    MdmacControlMessage *m = static_cast<MdmacControlMessage *>(msg);
//    coreEV << " handling packet from " << m->getSrcAddr() << std::endl;
//
//    switch( m->getKind() ) {
//
//		case HELLO_MESSAGE:
//			receiveHelloMessage( m );
//			break;
//
//		case CH_MESSAGE:
//			receiveChMessage( m );
//			break;
//
//		case JOIN_MESSAGE:
//			receiveJoinMessage( m );
//			break;
//
//		case DATA:
//			sendUp( decapsMsg( m ) );
//			break;
//
//		default:
//		    coreEV << " received packet from " << m->getSrcAddr() << " of unknown type: " << m->getKind() << std::endl;
//		    delete msg;
//		    break;
//
//    };
//
//}

Coord MdmacNetworkLayer::getHostPosition(cModule* const host,
			simtime_t t) {
	ASSERT(host != NULL);
	for (cModule::SubmoduleIterator iter(host); !iter.end(); ++iter) {
		cModule* submod = SUBMODULE_ITERATOR_TO_MODULE(iter);
		TraCIMobility* mm = dynamic_cast<TraCIMobility*>(submod);

		if (!mm) continue;
		return mm->getPositionAt(t);
	}
	ASSERT(false);
	return Coord();
}

Coord MdmacNetworkLayer::getHostPosition(cModule* const host) {
	return getHostPosition(host, simTime());
}

Coord MdmacNetworkLayer::getHostPosition(int hostId) {

	cModule* host = cSimulation::getActiveSimulation()->getModule(hostId);
	ASSERT(host != NULL);
	return MdmacNetworkLayer::getHostPosition(host);
}

/** @brief Handle self messages */
void MdmacNetworkLayer::handleSelfMsg(cMessage* msg) {

	if ( msg == mSendHelloMessage ) {

		sendClusterMessage( HELLO_MESSAGE );
    	scheduleAt( simTime() + mBeaconInterval, mSendHelloMessage );

	} else if ( msg == mFirstInitMessage ) {

		init();
		delete mFirstInitMessage;
		mFirstInitMessage = NULL;

	} else if ( msg == mBeatMessage ) {

		processBeat();
		scheduleAt( simTime() + BEAT_LENGTH, mBeatMessage );

	} else if (msg == mSendData) {
		scheduleAt(simTime() + packetSentIntervalBeg + uniform(0, packetSentInterval), mSendData);

		if (!getRandomPermit(sendNodePercent)) { return ; }
		cModule* dstMod = getDstNode();
		if (!dstMod) { 
			expEV << "can't get dst node,fuck" << endl;
			return; 
		}

		Coord dstPos = getHostPosition(dstMod);
		unsigned long pkgLen = getPkgLen();

		int nextHopId = getNextHopId(dstMod->getId());
		expEV << mId << " send data to " << dstMod->getId()
			<< " nextHopId is " << nextHopId << endl;

		/**
		 * currently no available next hop
		 */
		if (nextHopId < 0) {
			WaveShortMessageWithDst* wsmd = prepareAndInitWSMWithDst(dstMod,
						-1 /* next hop id is not sure */, /*ss.str()*/ "", pkgLen);

			sendDataLength += pkgLen;
			msgQueue.push_back(wsmd);

			ASSERT(mWaitSendData);
			if (!mWaitSendData->isScheduled()) {
				scheduleAt(simTime() + 1, mWaitSendData);
			}
			return;
		}

		sendDataLength += pkgLen;
		sendMessage(dstMod, nextHopId, /*ss.str()*/ "", pkgLen);
	} else if (msg == mWaitSendData) {
		int msgQueueSize = msgQueue.size();
		while(msgQueueSize--) {
			WaveShortMessageWithDst* tmp = msgQueue.front();
			msgQueue.pop_front();
			int rcvId = tmp->getRecipientAddress();
			std::string dstNodeId = tmp->getDstNodeId();

			if (!isVehicleAlive(dstNodeId)) {
				/**
				 * TODO: record
				 */
				delete tmp;
				tmp = NULL;
				expEV << dstNodeId << " is out of our region" << endl;
				continue;
			}

			int nextHopId = getNextHopId(rcvId);

			/**
			 * Still not suitable to send
			 */
			if (nextHopId < 0) {
				msgQueue.push_back(tmp);
			} else {
				tmp->setNextHopId(nextHopId);
				sendWSM(tmp);
			}
		}
		ASSERT(mWaitSendData);
		if(!msgQueue.empty() && !mWaitSendData->isScheduled()) {
			scheduleAt(simTime() + 1, mWaitSendData);
		}
	}
}

WaveShortMessageWithDst* MdmacNetworkLayer::prepareWSMWithDst(std::string name, int lengthBits,
			t_channel channel, int priority, int rcvId, int serial, Coord &rcvPos) {
	WaveShortMessageWithDst* wsm = new WaveShortMessageWithDst(name.c_str());
	wsm->addBitLength(headerLength);
	wsm->addBitLength(lengthBits);

	switch (channel) {
		case type_SCH: wsm->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
		case type_CCH: wsm->setChannelNumber(Channels::CCH); break;
	}
	wsm->setPsid(0);
	wsm->setPriority(priority);
	wsm->setWsmVersion(1);
	wsm->setTimestamp(simTime());
	wsm->setSenderAddress(mId);
	wsm->setRecipientAddress(rcvId);
	wsm->setSenderPos(curPosition);
	wsm->setSerial(serial);
	wsm->setDstHostPos(rcvPos);

	if (name == "beacon") {
		EV << "Creating Beacon with Priority " << priority << " at Applayer at " << wsm->getTimestamp() << std::endl;
	}
	if (name == "data") {
		EV << "Creating Data with Priority " << priority << " at Applayer at " << wsm->getTimestamp() << std::endl;
	}

	return wsm;
}

void MdmacNetworkLayer::sendMessage(cModule* dstMod, int nextHopId, std::string content, unsigned long pkgLen) {
	WaveShortMessageWithDst* wsm = prepareAndInitWSMWithDst(dstMod, nextHopId, content, pkgLen);
	sendWSM(wsm);
}

int MdmacNetworkLayer::getNextHopId(int dstId) {
	error("MdmacNetworkLayer do not provide this, subclass it!");
	return -1;
}

cModule* MdmacNetworkLayer::getDstNode(int option) {
	switch (option) {
		case NEIGHBOR_NODE:
			{
				const NeighborNodeSet* nns = getCachedNeighborNodes();
				if (nns->size() == 0) { return NULL; }
				int idx = int(uniform(0, nns->size()));
				auto iter = nns->begin();
				for (int i = 0; i < idx; ++i) {
					++iter;
				}
				return (*iter);
				//if (mNeighbours.size() == 0) { return NULL; }
				//int idx = int(uniform(0, mNeighbours.size()));
				//auto iter = mNeighbours.begin();
				//for (int i = 0; i < idx; ++i) {
				//	++iter;
				//}
				//return (iter->second.neighborMod);
			}
			break;
		case FAR_NODE:
			{
				// get dst from far nodes
				NodeVector* nv = getCachedFarNodes();
				if (nv->size() == 0) {
					return NULL;
				}
				int idx = int(uniform(0, nv->size()));
				return (*nv)[idx];
			}
			break;
		case PICK_ONE_NODE:
			{
				//double seed = uniform(0, 1);
				//if (seed <= sendNearPosibility) {
				if (getRandomPermit(sendNearPosibility)) {
					// get dst from neighbors
					NeighborNodeSet* nns = getCachedNeighborNodes();
					int idx = int(uniform(0, nns->size()));

					/**
					 * XXX: Should we retry to get newest neighbors?
					 */
					//if (nns->size() == 0) {
					//	nns = getCachedNeighborNodes(true);
					//}

					if (nns->size() != 0) {
						auto iter = nns->begin();
						for(int i = 0; i < idx; ++i) {
							++iter;
						}
						//return (iter->second.neighborMod);
						return *iter;
					}
					/**
					 * No neighbors are available, return far node
					 */
					return getDstNode(FAR_NODE);
				} else {
					// get dst from far nodes
					NodeVector* nv = getCachedFarNodes();
					int idx = int(uniform(0, nv->size()));
					return (*nv)[idx];
				}
			}
			break;
	};
	return NULL;

}

const NicEntry::GateList* MdmacNetworkLayer::getHostNicGateList(const BaseConnectionManager* const bcm,
			const cModule* host) {
	ASSERT(bcm != NULL && host != NULL);
	cModule* nicSubMod = host->getSubmodule("nic");
	return &bcm->getGateList(nicSubMod->getId());
}

const MdmacNetworkLayer::NeighborNodeSet
MdmacNetworkLayer::getRealNeighborNodes(const BaseConnectionManager* const bcm, const cModule* host) {

	const NicEntry::GateList* gl = getHostNicGateList(bcm, host);
	ASSERT(gl != NULL);
	NeighborNodeSet result;

	for (auto iter = gl->begin(); iter != gl->end(); ++iter) {
		cModule* host = cSimulation::getActiveSimulation()->getModule(iter->first->hostId);
		/**
		 * Do not add rsu as neighbors
		 */
		if (strcmp(host->getName(), "rsu") != 0) {
			result.insert(host);
		}
	}

	return result;
}

/** @brief decapsulate higher layer message from MdmacControlMessage */
//cMessage* MdmacNetworkLayer::decapsMsg( MdmacControlMessage *msg ) {
//
//    cMessage *m = msg->decapsulate();
//    setUpControlInfo(m, msg->getSrcAddr());
//
//    // delete the netw packet
//    delete msg;
//    return m;
//
//}


/** @brief Encapsulate higher layer packet into a MdmacControlMessage */
//NetwPkt* MdmacNetworkLayer::encapsMsg( cPacket *appPkt ) {
//
//    LAddress::L2Type macAddr;
//    LAddress::L3Type netwAddr;
//
//    coreEV <<"in encaps...\n";
//
//    MdmacControlMessage *pkt = new MdmacControlMessage(appPkt->getName(), DATA);
//
//    pkt->setBitLength(headerLength);	// ordinary IP packet
//
//    cObject* cInfo = appPkt->removeControlInfo();
//
//    if(cInfo == NULL){
//		EV << "warning: Application layer did not specifiy a destination L3 address\n"
//		   << "\tusing broadcast address instead\n";
//		netwAddr = LAddress::L3BROADCAST;
//    } else {
//		coreEV <<"CInfo removed, netw addr="<< NetwControlInfo::getAddressFromControlInfo( cInfo ) << std::endl;
//			netwAddr = NetwControlInfo::getAddressFromControlInfo( cInfo );
//		delete cInfo;
//    }
//
//    pkt->setSrcAddr(myNetwAddr);
//    pkt->setDestAddr(netwAddr);
//    coreEV << " netw "<< myNetwAddr << " sending packet" <<std::endl;
//    if(LAddress::isL3Broadcast( netwAddr )) {
//        coreEV << "sendDown: nHop=L3BROADCAST -> message has to be broadcasted"
//           << " -> set destMac=L2BROADCAST\n";
//        macAddr = LAddress::L2BROADCAST;
//    }
//    else{
//        coreEV <<"sendDown: get the MAC address\n";
//        macAddr = arp->getMacAddr(netwAddr);
//    }
//
//    setDownControlInfo(pkt, macAddr);
//
//    //encapsulate the application packet
//    pkt->encapsulate(appPkt);
//    coreEV <<" pkt encapsulated\n";
//    return (NetwPkt*)pkt;
//
//}


/** @brief Compute the CH weight for this node. */
double MdmacNetworkLayer::calculateWeight() {

	error( "MdmacNetworkLayer::calculateWeight MUST be overloaded to specify weight metric!" );
	return 0;

}



/** Add the destination data to a packet. */
int MdmacNetworkLayer::AddDestinationData( MdmacControlMessage *pkt ) {

	error( "MdmacNetworkLayer::AddDestinationData MUST be overloaded to specify destination!" );
	return 0;

}



/** Store the destination data from a packet. */
void MdmacNetworkLayer::StoreDestinationData( MdmacControlMessage *pkt ) {

	error( "MdmacNetworkLayer::StoreDestinationData MUST be overloaded to specify destination!" );

}



/** @brief Initiate clustering. */
void MdmacNetworkLayer::init() {

	// First get the lane we're in.
	//TraCICommandInterface* traci = mTraciManager->getCommandInterface();
	TraCICommandInterface::Vehicle* traciVehicle = mMobility->getVehicleCommandInterface();
	//mRoadID = traci->vehicle(mMobility->getExternalId()).getRoadId();
	//mLaneID = traci->vehicle(mMobility->getExternalId()).getLaneId();
	mRoadID = traciVehicle->getRoadId();
	mLaneID = traciVehicle->getLaneId();

	int nMax = chooseClusterHead();
	if ( nMax == -1 ) {

		// this node is the best CH around, so declare it
		mIsClusterHead = true;

		/**
		 * if this node belong to other cluster before
		 * tell cluster manager to remove it from the original cluster
		 */
		if (mClusterHead != -1) {
			mClusterManager->leaveCluster(mClusterHead, mId, simTime().dbl());
		}

		mClusterHead = mId;
		mClusterMembers.clear();
		mClusterMembers.insert( mId );
		mCurrentMaximumClusterSize = 1;
		sendClusterMessage( CH_MESSAGE );


	} else {

		// we found a neighbour that's a better CH
		mIsClusterHead = false;

		/**
		 * if this node belong to other cluster before
		 * tell cluster manager to remove it from the original cluster
		 */
		if (mClusterHead != -1) {
			mClusterManager->leaveCluster(mClusterHead, mId, simTime().dbl());
		}

		mClusterHead = nMax;
		mClusterMembers.clear();
    	//std::cerr << "Node: " << mId << " joined CH!: " << nMax << "\n";
		sendClusterMessage( JOIN_MESSAGE, nMax );
		//ClusterAnalysisScenarioManagerAccess::get()->joinMessageSent(mId, nMax);

	}

	getNeighbourClusters();
	mInitialised = true;

}



/** @brief Process the neighbour table in one beat. Also, update the node's weight. */
void MdmacNetworkLayer::processBeat() {

	// update the node's weight
	mWeight = calculateWeight();

	// process the neighbour table
	NeighbourIterator it = mNeighbours.begin();
	for ( ; it != mNeighbours.end(); ) {

		it->second.mFreshness -= 1;
		if ( it->second.mFreshness == 0 ) {

			unsigned int nodeId = it->first;
			it++;
			linkFailure( nodeId );

		} else {

			++it;

		}

	}

	//TraCICommandInterface* traci = mTraciManager->getCommandInterface();
	TraCICommandInterface::Vehicle* traciVehicle = mMobility->getVehicleCommandInterface();
	//mRoadID = traci->vehicle(mMobility->getExternalId()).getRoadId();
	//mLaneID = traci->vehicle(mMobility->getExternalId()).getLaneId();
	mRoadID = traciVehicle->getRoadId();
	mLaneID = traciVehicle->getLaneId();

// 	TraCIScenarioManager *pManager = TraCIScenarioManagerAccess().get();
// 	char strNodeName[10];
// 	sprintf( strNodeName, "node%i_conn", mId );
// 
// 	std::list<Coord> points;
// 	points.push_back( mMobility->getCurrentPosition() );
// 
// 	if ( !mIsClusterHead )
// 		points.push_back( mNeighbours[mClusterHead].mPosition );
// 
// 	pManager->commandSetPolygonShape( strNodeName, points );

}



/** @brief Select a CH from the neighbour table. */
int MdmacNetworkLayer::chooseClusterHead() {

	int nCurr = -1;
	double wCurr = mWeight;

	NeighbourIterator it = mNeighbours.begin();
	for ( ; it != mNeighbours.end(); it++ ) {

		if ( it->second.mIsClusterHead && it->second.mWeight > wCurr ) {

			nCurr = it->first;
			wCurr = it->second.mWeight;

		}

	}

	return nCurr;

}



/** @brief Handle a link failure. Link failure is detected when a CMs freshness reaches 0. */
void MdmacNetworkLayer::linkFailure( unsigned int nodeId ) {

	mNeighbours.erase( nodeId );
	if ( IsClusterHead() ) {

		if ( mClusterMembers.find( nodeId ) == mClusterMembers.end() )
			return;		// we don't need to do anything, because it wasn't a member of our cluster.

		/*
		 *  A member of this cluster is out of range, so erase it's ID
		 *  and emit the cluster size change signal.
		 */
		ClusterMemberRemoved( nodeId );

	} else if ( nodeId == mClusterHead ) {

		/*
		 *	This CM lost its CH, so recluster and emit the CH change signal.
		 */
		init();
		emit( mSigHeadChange, 1 );

	}

}



/** @brief Calculate the freshness of the given neighbour. */
void MdmacNetworkLayer::calculateFreshness( unsigned int nodeId ) {

	unsigned int freshness = mInitialFreshness;
	Coord v =    mMobility->getCurrentSpeed() - mNeighbours[ nodeId ].mVelocity;
	Coord p = mMobility->getCurrentPosition() - mNeighbours[ nodeId ].mPosition;

	double a = v.squareLength(), b, c;
	if ( a > 0 ) {

		b = ( v.x * p.x + v.y * p.y ) / a;
		c = ( p.squareLength() - mTransmitRangeSq ) / a;

		double detSq = b*b - c;
		unsigned int r = UINT_MAX;
		if ( detSq > 0 )
			r = (int)floor( ( sqrt( detSq ) - b ) / BEAT_LENGTH );

		freshness = std::min( 3*mInitialFreshness, r );

	}

	mNeighbours[nodeId].mFreshness = freshness;

}



/** @brief Determine whether the given node is a suitable CH. */
bool MdmacNetworkLayer::testClusterHeadChange( unsigned int nodeId ) {

	bool t1 = mNeighbours[nodeId].mIsClusterHead;
	bool t2 = mNeighbours[nodeId].mWeight > ( mIsClusterHead ? mWeight : mNeighbours[mClusterHead].mWeight );
	bool t3 = mNeighbours[nodeId].mFreshness >= mFreshnessThreshold;

	Coord v1 = mMobility->getCurrentSpeed();
	Coord v2 = mNeighbours[ nodeId ].mVelocity;
	double angle = cos( ( v1.x*v2.x + v1.y*v2.y ) / ( v1.length() * v2.length() ) );

	bool t4 = angle <= mAngleThreshold;

	return t1 && t2 && t3 && t4;

}


/** @brief Handle a HELLO message. */
void MdmacNetworkLayer::receiveHelloMessage( MdmacControlMessage *m ) {

	updateNeighbour(m);

	if ( testClusterHeadChange( m->getNodeId() ) ) {

		// If this was a CH, the cluster is dead, so log lifetime
		if ( IsClusterHead() ) {
			ClusterDied( CD_Cannibal );
		}

        emit( mSigHeadChange, 1 );
		mIsClusterHead = false;
		mClusterMembers.clear();

		/**
		 * if this node belong to other cluster before
		 * tell cluster manager to remove it from the original cluster
		 */
		if (mClusterHead != -1) {
			mClusterManager->leaveCluster(mClusterHead, mId, simTime().dbl());
		}

		mClusterHead = m->getNodeId();

		mClusterManager->joinCluster(mClusterHead, mId, simTime().dbl());
		getNeighbourClusters(false);
		sendClusterMessage( JOIN_MESSAGE, m->getNodeId() );
		//ClusterAnalysisScenarioManagerAccess::get()->joinMessageSent( mId, m->getNodeId() );

	}

	if ( m->getTtl() > 1 )
		sendClusterMessage( HELLO_MESSAGE, -1, m->getTtl()-1 );

	//delete m;

}



/** @brief Handle a CH message. */
void MdmacNetworkLayer::receiveChMessage( MdmacControlMessage *m ) {

	updateNeighbour(m);

	if ( testClusterHeadChange( m->getNodeId() ) ) {

		// If this was a CH, the cluster has been cannibalised, so log lifetime
		if ( IsClusterHead() ) {
			ClusterDied( CD_Cannibal );
		}

        emit( mSigHeadChange, 1 );
		mIsClusterHead = false;
		mClusterMembers.clear();
		/**
		 * if this node belong to other cluster before
		 * tell cluster manager to remove it from the original cluster
		 */
		if (mClusterHead != -1) {
			mClusterManager->leaveCluster(mClusterHead, mId, simTime().dbl());
		}
		mClusterHead = m->getNodeId();

		mClusterManager->joinCluster(mClusterHead, mId, simTime().dbl());
		sendClusterMessage( JOIN_MESSAGE, m->getNodeId() );
		//ClusterAnalysisScenarioManagerAccess::get()->joinMessageSent( mId, m->getNodeId() );

	} else {

		if ( IsClusterHead() ) {

			bool sizeChanged = false;

			if ( mClusterMembers.find( m->getNodeId() ) != mClusterMembers.end() ) {

				mClusterMembers.erase( m->getNodeId() );
				sizeChanged = true;

			}

			if ( sizeChanged )
				mCurrentMaximumClusterSize = std::max( mCurrentMaximumClusterSize, (int)mClusterMembers.size() );

		}

	}

	if ( m->getTtl() > 1 )
		sendClusterMessage( CH_MESSAGE, -1, m->getTtl()-1 );

	//delete m;

}



/** @brief Handle a JOIN message. */
void MdmacNetworkLayer::receiveJoinMessage( MdmacControlMessage *m ) {

	updateNeighbour(m);

	if ( mIsClusterHead ) {

		bool sizeChanged = false;
		if ( m->getTargetNodeId() == mId ) {

			ClusterMemberAdded( m->getNodeId() );
			mClusterMembers.insert( m->getNodeId() );
			sizeChanged = true;
			if ( mClusterMembers.size() == 2 ) {
				ClusterStarted();
			}
			//ClusterAnalysisScenarioManagerAccess::get()->joinMessageReceived( m->getNodeId(), mId );

		} else if ( mClusterMembers.find( m->getNodeId() ) != mClusterMembers.end() ) {

			sizeChanged = mClusterMembers.size() != 2;
			// This join message came from a member of our cluster
			ClusterMemberRemoved( m->getNodeId() );

		}

		if ( sizeChanged )
			mCurrentMaximumClusterSize = std::max( mCurrentMaximumClusterSize, (int)mClusterMembers.size() );

	} else if ( mClusterHead == m->getNodeId() ) {

		init();
		emit( mSigHeadChange, 1 );

	}

	if ( m->getTtl() > 1 )
		sendClusterMessage( JOIN_MESSAGE, m->getTargetNodeId(), m->getTtl()-1 );

	//delete m;

}

/** @brief Update a neighbour's data. */
void MdmacNetworkLayer::updateNeighbour( MdmacControlMessage *m ) {

	// update the neighbour data
	
	if (mNeighbours.find(m->getNodeId()) == mNeighbours.end()) {
		mNeighbours[m->getNodeId()].neighborMod = getSimulation()->getModule(m->getNodeId());
	}
	mNeighbours[m->getNodeId()].mWeight = m->getWeight();
	mNeighbours[m->getNodeId()].mIsClusterHead = m->getIsClusterHead();
	mNeighbours[m->getNodeId()].mRoadID = m->getRoadId();
	mNeighbours[m->getNodeId()].mLaneID = m->getLaneId();
	mNeighbours[m->getNodeId()].mPosition.x = m->getXPosition();
	mNeighbours[m->getNodeId()].mPosition.y = m->getYPosition();
	mNeighbours[m->getNodeId()].mVelocity.x = m->getXVelocity();
	mNeighbours[m->getNodeId()].mVelocity.y = m->getYVelocity();

	if ( mIncludeDestination ) {
		StoreDestinationData( m );
		//std::cerr << "Dest(" << m->getNodeId() << ") = (" << m->getXDestination() << "," << m->getYDestination() << ")\n";
	}

	calculateFreshness( m->getNodeId() );
}

MdmacControlMessage* MdmacNetworkLayer::prepareWSMCB(int kind, int dest, int nHops) {
	static unsigned long beaconSeqNum = 0;
	t_channel channel = dataOnSch ? type_SCH : type_CCH;

	MdmacControlMessage* pkt = new MdmacControlMessage("beacon");
	//MdmacControlMessage* pkt = new MdmacControlMessage("kkk");
	int pkgLen = 432;

	if ( mIncludeDestination ) {
		pkgLen += AddDestinationData( pkt );
		//std::cerr << "My dest = (" << mCurrentDestination.x << "," << mCurrentDestination.y << ")\n";
	}
	pkt->setBitLength(pkgLen);

    if ( kind != HELLO_MESSAGE )
    	emit( mSigOverhead, pkgLen );
    else
    	emit( mSigHelloOverhead, pkgLen );

	switch (channel) {
		case type_SCH: pkt->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
		case type_CCH: pkt->setChannelNumber(Channels::CCH); break;
	}

	if (GetClusterState() == SINGLE) {
		pkt->setClusterId(-1);
	}

	pkt->setClusterId(mClusterHead);
	pkt->setKind(kind);
	pkt->setPsid(0);
	pkt->setPriority(beaconPriority);
	pkt->setWsmVersion(1);
	pkt->setTimestamp(simTime());
	pkt->setSenderAddress(mId);

	pkt->setRecipientAddress(-1);
	pkt->setSerial(beaconSeqNum++);
	//pkt->setSenderPos(curPosition);
	//pkt->setClusterBeaconType(type);

    pkt->setNodeId( mId );
    pkt->setWeight( mWeight );
    if ( nHops == -1 )
    	nHops = mHopCount;
    pkt->setTtl( nHops );

    pkt->setRoadId( mRoadID.c_str() );
    pkt->setLaneId( mLaneID.c_str() );

    Coord p = mMobility->getCurrentPosition();
    pkt->setXPosition( p.x );
    pkt->setYPosition( p.y );

    p = mMobility->getCurrentSpeed();
    pkt->setXVelocity( p.x );
    pkt->setYVelocity( p.y );

    pkt->setIsClusterHead( mIsClusterHead );
    pkt->setTargetNodeId( dest );

	return pkt;
}

void MdmacNetworkLayer::sendWSM(WaveShortMessage* wsm) {
	sendDelayedDown(wsm,individualOffset);
}

/** @brief Sends a given cluster message. */
void MdmacNetworkLayer::sendClusterMessage( int kind, int dest, int nHops ) {
	MdmacControlMessage* m = prepareWSMCB(kind, dest, nHops);
	sendWSM(m);
}
void MdmacNetworkLayer::handlePositionUpdate(cObject* obj) {}
void MdmacNetworkLayer::handleParkingUpdate(cObject* obj) {}

/** @brief Sends a given cluster message. */
//void MdmacNetworkLayer::sendClusterMessage( int kind, int dest, int nHops ) {
//
//    LAddress::L2Type macAddr;
//    LAddress::L3Type netwAddr;
//
//    coreEV <<"sending cluster control message...\n";
//
//	int packetSize = 432;
//
//	if ( mIncludeDestination ) {
//		packetSize += AddDestinationData( pkt );
//		//std::cerr << "My dest = (" << mCurrentDestination.x << "," << mCurrentDestination.y << ")\n";
//	}
//
//    pkt->setBitLength(packetSize);	// size of the control packet packet.
//
//    if ( kind != HELLO_MESSAGE )
//    	emit( mSigOverhead, packetSize );
//    else
//    	emit( mSigHelloOverhead, packetSize );
//
//    // fill the cluster control fields
//    pkt->setNodeId( mId );
//    pkt->setWeight( mWeight );
//    if ( nHops == -1 )
//    	nHops = mHopCount;
//    pkt->setTtl( nHops );
//
//    pkt->setRoadId( mRoadID.c_str() );
//    pkt->setLaneId( mLaneID.c_str() );
//
//    Coord p = mMobility->getCurrentPosition();
//    pkt->setXPosition( p.x );
//    pkt->setYPosition( p.y );
//
//    p = mMobility->getCurrentSpeed();
//    pkt->setXVelocity( p.x );
//    pkt->setYVelocity( p.y );
//
//    pkt->setIsClusterHead( mIsClusterHead );
//    pkt->setTargetNodeId( dest );
//
//    netwAddr = LAddress::L3BROADCAST;
//
//    pkt->setSrcAddr(myNetwAddr);
//    pkt->setDestAddr(netwAddr);
//    coreEV << " netw "<< myNetwAddr << " sending packet" <<std::endl;
//	macAddr = LAddress::L2BROADCAST;
//
//    setDownControlInfo(pkt, macAddr);
//
//    sendDown( pkt );
//
//}
