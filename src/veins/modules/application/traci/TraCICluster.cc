//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include "veins/base/utils/asserts.h"
#include "veins/modules/application/traci/TraCICluster.h"

using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;

using std::endl;

#define expEV std::cout

const simsignalwrap_t TraCICluster::parkingStateChangedSignal = simsignalwrap_t(TRACI_SIGNAL_PARKING_CHANGE_NAME);

const simsignalwrap_t TraCICluster::neighborCntStatistic = simsignalwrap_t("neighborCntStatistic");

Define_Module(TraCICluster);

Coord TraCICluster::getMyPosition() const {
	return mobility->getCurrentPosition();
}

Coord TraCICluster::getHostPosition(cModule* const host) {

//	TraCIMobility* mobility = TraCIMobilityAccess().get(host);
//
//	return mobility->getPositionAt(simTime());

	ASSERT(host != NULL);
	for (cModule::SubmoduleIterator iter(host); !iter.end(); ++iter) {
		cModule* submod = SUBMODULE_ITERATOR_TO_MODULE(iter);
		TraCIMobility* mm = dynamic_cast<TraCIMobility*>(submod);

		if (!mm) continue;
		return mm->getPositionAt(simTime());
	}
	//expEV << host->getName() << std::endl;
	ASSERT(false);
	return Coord();
	//assertTrue("can not get TraCIMobility from host", mobility != NULL);
	//return mobility->getCurrentPosition();
}

Coord TraCICluster::getHostPosition(int hostId) {

	cModule* host = cSimulation::getActiveSimulation()->getModule(hostId);
	ASSERT(host != NULL);
	return TraCICluster::getHostPosition(host);
}

const NicEntry::GateList* TraCICluster::getMyNicGateList() const {
	return getHostNicGateList(bcm, findHost());
}

const NicEntry::GateList* TraCICluster::getHostNicGateList(const BaseConnectionManager* const bcm,
			const cModule* host) {
	ASSERT(bcm != NULL && host != NULL);
	cModule* nicSubMod = host->getSubmodule("nic");
	return &bcm->getGateList(nicSubMod->getId());
}

const TraCICluster::NeighborNodeSet 
TraCICluster::getNeighborNodes(const BaseConnectionManager* const bcm,
			const cModule* host) {

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

TraCICluster::NodeVector
TraCICluster::getFarNodes(NeighborNodeSet* nns, TraCIScenarioManagerLaunchd* traciSMLd,
			const BaseConnectionManager* const bcm,
			const cModule* host) {

	ASSERT(nns != NULL && traciSMLd != NULL && bcm != NULL && host != NULL);
	const std::map<std::string, cModule*> &hosts = traciSMLd->getManagedHosts();

	TraCICluster::NodeVector result;
	result.reserve(hosts.size() - nns->size());

	for (auto iter = hosts.begin(); iter != hosts.end(); ++iter) {
		if (nns->find(iter->second) == nns->end()) {
			result.push_back(iter->second);
		}
	}

	return result;
}

TraCICluster::NodeVector TraCICluster::getHostFarNodes() {
	NeighborNodeSet* nns = getCachedNeighborNodes();
	return getFarNodes(nns, traciSMLd, bcm, findHost());
}

void TraCICluster::initialize(int stage) {
	BaseWaveApplLayer::initialize(stage);
	if (stage == 0) {
		mobility = TraCIMobilityAccess().get(getParentModule());
		traci = mobility->getCommandInterface();
		traciVehicle = mobility->getVehicleCommandInterface();

		neighborNodesUpdateTime = simTime();
		farNodesUpdateTime = simTime();
		findHost()->subscribe(parkingStateChangedSignal, this);
		recvDataLength = .0;
		sendDataLength = .0;
		forwardDataLength = .0;

		traciSMLd = TraCIScenarioManagerLaunchdAccess().get();
		bcm = FindModule<BaseConnectionManager*>::findGlobalModule();

		if (bcm == NULL) {
			assertTrue("Could not find BaseConnectionManager module", false);
		}

		// register nic to connection manager
		Coord pos = getMyPosition();
		bcm->registerNic(this->getParentModule()->getSubmodule("nic"), (ChannelAccess*) this->getParentModule()->getSubmodule("nic")->getSubmodule("phy80211p"), &pos);

		/**
		 * Set neighbor node count statistics
		 */
		currentNeighborCnt.setName("neighbor_cnt");
		packetDelay.setName("packet_delay");
		packetPathLen.setName("packet_path_length");

		if (hasPar("sendNearPosibility")) {
			sendNearPosibility = par("sendNearPosibility").doubleValue();
		} else {
			sendNearPosibility = .5;
		}

		if (hasPar("packetSentInterval")) {
			packetSentInterval = par("packetSentInterval");
		} else {
			packetSentInterval = 10;
		}

		if (hasPar("packetLenMin")) {
			packetLenMin = par("packetLenMin");
		} else {
			packetLenMin = 1024;
		}
	
		if (hasPar("packetLenMax")) {
			packetLenMax = par("packetLenMax");
		} else {
			packetLenMax = 1024;
		}

		if (hasPar("sendNodePercent")) {
			sendNodePercent = par("sendNodePercent");
		} else {
			sendNodePercent = 0.5;
		}

		packetSentIntervalBeg = hasPar("packetSentIntervalBeg") ?
			par("packetSentIntervalBeg") : 100;

		clusterMaxSize = hasPar("clusterMaxSize") ?
			par("clusterMaxSize") : 0;

		clusterBeaconInterval = hasPar("clusterBeaconInterval") ?
			par("clusterBeaconInterval") : 5;

		clusterRole = SINGLE;
		clusterNodeData.hd = NULL;

		ASSERT(packetLenMax >= packetLenMin);

	} else if (stage == 1) {
		if (packetSentInterval > 0) {
			packetSendSelfMsg = new cMessage();
			//scheduleAt(simTime() + packetSentIntervalBeg + uniform(0, packetSentInterval), new cMessage());
			clusterBeaconSelfMsg = new cMessage();
			scheduleAt(simTime() + clusterBeaconInterval, clusterBeaconSelfMsg);
		}
	}
}

void TraCICluster::finish() {

	recordScalar("recv_data_len", recvDataLength);
	recordScalar("forward_data_len", forwardDataLength);
	recordScalar("send_data_len", sendDataLength);

	delete packetSendSelfMsg;
	delete clusterBeaconSelfMsg;
}

void TraCICluster::onBeacon(WaveShortMessage* wsm) {
}

void TraCICluster::onData(WaveShortMessage* wsm) {
	WaveShortMessageWithDst* wsmd = dynamic_cast<WaveShortMessageWithDst*>(wsm);
	/**
	 * Note, current RSU appl will send WaveShortMessage instead of WaveShortMessageWithDst
	 * which will violate this assert
	 */
	ASSERT(wsmd != NULL);
	cModule *host = findHost();
	findHost()->getDisplayString().updateWith("r=16,green");
	host->getDisplayString().updateWith("r=16,green");

	/**
	 * I'm the dst node
	 */
	if (findHost()->getId() == wsmd->getRecipientAddress()) {

		recvDataLength += wsmd->getByteLength();

		const std::map<std::string, cModule*> &hosts = traciSMLd->getManagedHosts();
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
	const std::map<std::string, cModule*> &hosts = traciSMLd->getManagedHosts();

	auto iter = hosts.find(dstNodeId);
	if (iter == hosts.end()) {
		expEV << "[Shit] " << dstNodeId << " is out of our study region" << std::endl;
		return;
	}

	if (findHost()->getId() == wsmd->getNextHopId()) {

		//expEV << wsmd->getNextHopId() << " route id " << std::endl;
		NeighborNodeSet* nns = getCachedNeighborNodes();

		/**
		 * Currently, no neighbor can help send msg
		 * push to buffer
		 */
		if (nns->size() == 0) {
			wsmd->getPathNodes().push(wsmd->getNextHopId());
			msgQueue.push(wsmd->dup());
			return;
		}
		/**
		 * query current dst pos
		 */
		//Coord dstPos = getHostPosition(wsmd->getRecipientAddress());
		Coord dstPos = getHostPosition(iter->second);
		int nextHopId = getNearestNodeToPos(*nns, dstPos);
		wsmd->getPathNodes().push(wsmd->getNextHopId());
		wsmd->setNextHopId(nextHopId);

		forwardDataLength += wsmd->getByteLength();

		sendWSM(wsmd->dup());

	} else {
		/**
		 * I'm not the next hop, do nothing, just drop it
		 */
	}
	//expEV << wsm->getWsmData() << std::endl;
}

void TraCICluster::sendMessage(cModule* dstMod, int nextHopId, std::string content, unsigned long pkgLen) {
	WaveShortMessageWithDst* wsm = prepareAndInitWSMWithDst(dstMod, nextHopId, content, pkgLen);
	sendWSM(wsm);
}

void TraCICluster::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
	Enter_Method_Silent();
	if (signalID == mobilityStateChangedSignal) {
		handlePositionUpdate(obj);
	}
	else if (signalID == parkingStateChangedSignal) {
		handleParkingUpdate(obj);
	}
}

WaveShortMessageClusterBeacon* TraCICluster::prepareWSMCB(int type, unsigned long pkgLen) {
	static unsigned long beaconSeqNum = 0;
	t_channel channel = dataOnSch ? type_SCH : type_CCH;

	WaveShortMessageClusterBeacon* wsmcb = new WaveShortMessageClusterBeacon("beacon");
	wsmcb->setByteLength(pkgLen);

	switch (channel) {
		case type_SCH: wsmcb->setChannelNumber(Channels::SCH1); break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
		case type_CCH: wsmcb->setChannelNumber(Channels::CCH); break;
	}

	wsmcb->setPsid(0);
	wsmcb->setPriority(beaconPriority);
	wsmcb->setWsmVersion(1);
	wsmcb->setTimestamp(simTime());
	wsmcb->setSenderAddress(myId);

	/**
	 * For one-hop broadcasting
	 */
	wsmcb->setRecipientAddress(-1);
	wsmcb->setSenderPos(curPosition);
	wsmcb->setSerial(beaconSeqNum++);
	wsmcb->setClusterBeaconType(type);
	wsmcb->setSenderRole(clusterRole);

	return wsmcb;
}

namespace {
	inline unsigned long getGateWayDataSize(GateWayData& gwd) {
		return gwd.connectedClusters.size() * sizeof(int) * 2;
	}

	inline unsigned long getHeadDataSize(HeadData& hd) {
		unsigned long ret = sizeof(int);

		ret += hd.memberIds.size() * sizeof(int);
		ret += hd.gateWayIds.size() * sizeof(int);
		ret += hd.gateWayInfo.size() * sizeof(int);

		for (auto iter = hd.gateWayInfo.begin(); iter != hd.gateWayInfo.end();
					++iter) {
			ret += iter->second.size() * sizeof(int);
		}
		return ret;
	}
};

void TraCICluster::sendClusterHello() {
	WaveShortMessageClusterBeacon* wsmcb = prepareWSMCB(HELLO, headerLength);

	if (clusterRole == GATEWAY || clusterRole == GATEWAY_BAK) {
		/**
		 * Attach gateway info to packet
		 */
		wsmcb->addBitLength(getGateWayDataSize(*clusterNodeData.gwd) * 8);
		wsmcb->setNodeData(clusterNodeData);
	}

	sendWSM(wsmcb);
}

void TraCICluster::sendClusterJoinRequest() {
	WaveShortMessageClusterBeacon* wsmcb = prepareWSMCB(JOIN_REQUEST, headerLength);
	sendWSM(wsmcb);
}

void TraCICluster::sendClusterStatus() {
	WaveShortMessageClusterBeacon* wsmcb = prepareWSMCB(CLUSTER_STATUS, headerLength);
	sendWSM(wsmcb);
}

void TraCICluster::handleClusterBeaconSelfMsg(cMessage* msg) {
	scheduleAt(simTime() + clusterBeaconInterval, msg);

	sendClusterHello();
	//switch (clusterRole) {
	//	case SINGLE:
	//		sendClusterHello();
	//		break;
	//	case HEAD:
	//	case HEAD_BAK: /* Fall through */
	//		sendClusterStatus();
	//		break;
	//	case GATEWAY:
	//	case GATEWAY_BAK: /* Fall through */
	//		sendClusterHello();
	//		break;
	//	case MEMBER:
	//		sendClusterHello();
	//		break;
	//	default:
	//		ASSERT(false);
	//		break;
	//}

}

void TraCICluster::handleSendPacketSelfMsg(cMessage* msg) {
	scheduleAt(simTime() + packetSentIntervalBeg + uniform(0, packetSentInterval), msg);

	if (!getRandomPermit(sendNodePercent)) {
		return;
	}

	cModule* dstMod = getDstNode();

	/**
	 * No available node for dst
	 * Do not send anything
	 */
	if (!dstMod) {
		return;
	}

	Coord dstPos = getHostPosition(dstMod);

	NeighborNodeSet* nns = getCachedNeighborNodes();

	//std::stringstream ss;
	unsigned long pkgLen = getPkgLen();
	//ss << pkgLen;

	/**
	 * if no neighbors are available for sending packets
	 * just prepare WSM and store it in the message queue
	 */
	if (nns->size() == 0) {
		WaveShortMessageWithDst* wsmd = prepareAndInitWSMWithDst(dstMod,
					-1 /* next hop id is not sure */, /*ss.str()*/ "", pkgLen);

		sendDataLength += pkgLen;
		msgQueue.push(wsmd);
		return;
	}

	int nextHopId = getNearestNodeToPos(*nns, dstPos);

	sendDataLength += pkgLen;
	sendMessage(dstMod, nextHopId, /*ss.str()*/ "", pkgLen);
}

void TraCICluster::handleSelfMsg(cMessage* msg) {

	if (msg == packetSendSelfMsg) {
		handleSendPacketSelfMsg(msg);
	} else if (msg == clusterBeaconSelfMsg) {
		handleClusterBeaconSelfMsg(msg);
	} else {
		ASSERT(false);
	}

}

WaveShortMessageWithDst* TraCICluster::prepareWSMWithDst(std::string name, int lengthBits,
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
	wsm->setSenderAddress(myId);
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

void TraCICluster::handleParkingUpdate(cObject* obj) {
	//isParking = mobility->getParkingState();
	//if (sendWhileParking == false) {
	//	if (isParking == true) {
	//		(FindModule<BaseConnectionManager*>::findGlobalModule())->unregisterNic(this->getParentModule()->getSubmodule("nic"));
	//	}
	//	else {
	//		Coord pos = mobility->getCurrentPosition();
	//		(FindModule<BaseConnectionManager*>::findGlobalModule())->registerNic(this->getParentModule()->getSubmodule("nic"), (ChannelAccess*) this->getParentModule()->getSubmodule("nic")->getSubmodule("phy80211p"), &pos);
	//	}
	//}
}

void TraCICluster::handlePositionUpdate(cObject* obj) {
	BaseWaveApplLayer::handlePositionUpdate(obj);

	NeighborNodeSet* neighborNodes = getCachedNeighborNodes(true);
	//NeighborNodeSet* neighborNodes = getCachedNeighborNodes();

	//for (auto iter = neighborNodes->begin(); iter != neighborNodes->end(); ++iter) {
	//	Coord nodePos = getHostPosition(*iter);
	//}

	/**
	 * If there is neighbor nodes
	 */
	if (!neighborNodes->empty()) {

		if (!msgQueue.empty()) {
			expEV << "neighborNodes cnt: " << neighborNodes->size() << std::endl;
			expEV << "msgQueue size: " << msgQueue.size() << std::endl;
		}

		std::string dstNodeId;
		while (!msgQueue.empty()) {
			WaveShortMessageWithDst* wsmd = msgQueue.front();
			msgQueue.pop();

			dstNodeId = wsmd->getDstNodeId();
			const std::map<std::string, cModule*> &hosts = traciSMLd->getManagedHosts();

			/**
			 * dst node is out of study region
			 */
			auto iter = hosts.find(dstNodeId);
			if (iter == hosts.end()) {
				expEV << "[Shit] " << dstNodeId << " is out of our study region" << std::endl;
				delete wsmd;
				continue;
			}

			Coord dstPos = getHostPosition(wsmd->getRecipientAddress());
			int nextHopId = getNearestNodeToPos(*neighborNodes, dstPos);
			wsmd->setNextHopId(nextHopId);
			//sendWSM(wsmd->dup());

			forwardDataLength += wsmd->getByteLength();
			sendWSM(wsmd);
		}
	}

}

void TraCICluster::sendWSM(WaveShortMessage* wsm) {
	//if (isParking && !sendWhileParking) return;
	sendDelayedDown(wsm,individualOffset);
}

cModule* TraCICluster::getDstNode(int option) {
	switch (option) {
		case NEIGHBOR_NODE:
			{
				const TraCICluster::NeighborNodeSet* nns = getCachedNeighborNodes();
				if (nns->size() == 0) { return NULL; }
				int idx = int(uniform(0, nns->size()));
				auto iter = nns->begin();
				for (int i = 0; i < idx; ++i) {
					++iter;
				}
				return (*iter);
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
					const TraCICluster::NeighborNodeSet* nns = getCachedNeighborNodes();
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
						return (*iter);
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

