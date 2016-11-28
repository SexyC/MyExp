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

#ifndef TraCICluster_H
#define TraCICluster_H

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/mobility/traci/TraCIScenarioManagerLaunchd.h"

#include "veins/modules/messages/WaveShortMessageWithDst_m.h"
#include "veins/modules/messages/WaveShortMessageClusterBeacon_m.h"
#include "veins/modules/application/traci/NodeData.h"
#include "veins/modules/application/traci/NodeClusterRole.h"
#include "veins/modules/application/traci/NodeStrategy.h"

#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <list>

using Veins::TraCIMobility;
using Veins::TraCICommandInterface;
using Veins::TraCIScenarioManagerLaunchd;
using Veins::TraCIScenarioManagerLaunchdAccess;
using Veins::AnnotationManager;

using std::unordered_map;
using std::unordered_set;
using std::vector;
using std::queue;
using std::list;

namespace yy {
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

/**
 * Small IVC Demo using 11p
 */
class TraCICluster : public BaseWaveApplLayer, public NodeClusterRole {
	public:
		virtual void initialize(int stage);
		virtual void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details);
		virtual void finish();

		friend class NodeStrategy;
		friend class SingleNodeStrategy;
		friend class HeadNodeStrategy;
		friend class GateWayNodeStrategy;
		friend class MemberNodeStrategy;

		class NeighborClusterInfo {
			public:
				int nodeId;
				int neighborClusterId;
				simtime_t timeStamp;
		};

		class NeighborClusterInfoGreatComp {
			public:
				bool operator() (NeighborClusterInfo& n1, NeighborClusterInfo& n2) {
					return n1.timeStamp > n2.timeStamp;
				}
		};

		class NeighborClusterInfoLessComp {
			public:
				bool operator() (NeighborClusterInfo& n1, NeighborClusterInfo& n2) {
					return n1.timeStamp < n2.timeStamp;
				}
		};
		static inline list<NeighborClusterInfo>::iterator findClusterNodeInfo(list<NeighborClusterInfo>& l, NeighborClusterInfo& n) {
			for(auto iter = l.begin(); iter != l.end(); ++iter) {
				if (iter->nodeId == n.nodeId && iter->neighborClusterId == n.neighborClusterId) {
					return iter;
				}
			}
			return l.end();
		}

		static inline list<NeighborClusterInfo>::iterator 
		findClusterNodeInfoInsertPos(list<NeighborClusterInfo>& l, NeighborClusterInfo& n) {
			for (auto iter = l.begin(); iter != l.end(); ++iter) {
				/**
				 * Find info earlier than this one
				 */
				if (iter->timeStamp <= n.timeStamp) {
					return iter;
				}
			}
			return l.end();
		}

	protected:
		enum { NEIGHBOR_NODE, FAR_NODE, PICK_ONE_NODE };
		typedef std::unordered_set<cModule*> NeighborNodeSet;
		typedef std::vector<cModule*> NodeVector;

		/**
		 * Do not access this below four vars directly
		 * use getCachedFarNodes and getCachedNeighborNodes
		 */
		TraCICluster::NeighborNodeSet neighborNodes;
		simtime_t neighborNodesUpdateTime;
		TraCICluster::NodeVector farNodes;
		simtime_t farNodesUpdateTime;

		TraCICluster::NeighborNodeSet* getCachedNeighborNodes(bool forceUpdate = false) {
			if (forceUpdate || simTime() != neighborNodesUpdateTime) {
				neighborNodesUpdateTime = simTime();
				neighborNodes = getNeighborNodes(bcm, findHost());
			}
			return &neighborNodes;
		}

		TraCICluster::NodeVector* getCachedFarNodes(bool forceUpdate = false) {
			if (forceUpdate || simTime() != farNodesUpdateTime) {
				farNodesUpdateTime = simTime();
				farNodes = getHostFarNodes();
			}
			return &farNodes;
		}

		TraCIMobility* mobility;
		TraCICommandInterface* traci;
		TraCICommandInterface::Vehicle* traciVehicle;
		//AnnotationManager* annotations;
		//simtime_t lastDroveAt;
		//bool sentMessage;
		//bool isParking;
		//bool sendWhileParking;
		static const simsignalwrap_t parkingStateChangedSignal;
		double recvDataLength;
		double sendDataLength;
		double forwardDataLength;

		double sendNearPosibility;
		double sendNodePercent;
		unsigned long packetSentInterval;
		unsigned long packetSentIntervalBeg;

		unsigned long packetLenMin;
		unsigned long packetLenMax;

		BaseConnectionManager *bcm;
		TraCIScenarioManagerLaunchd* traciSMLd;

		/**
		 * record the current neighbor count
		 */
		cOutVector currentNeighborCnt;
		cOutVector packetDelay;
		cOutVector packetPathLen;

		/**
		 * to store packets that need forwarding
		 */
		queue<WaveShortMessageWithDst*> msgQueue;

		//cMessage sendMessageSignal;
		const static simsignalwrap_t neighborCntStatistic;

		cMessage* packetSendSelfMsg;

		/**
		 * Cluster information related
		 */
		int clusterId;

		int clusterHeadId;

		// setting to 0 indicate to no limit
		int clusterMaxSize;
		int clusterBeaconInterval;
		cMessage* clusterBeaconSelfMsg;

		/**
		 * Min-heap, easy access to the earlist info
		 */
		//unordered_map<int, priority_queue<NeighborClusterInfo, vector<NeighborClusterInfo>
		//	, NeighborClusterInfoGreatComp> neighborClusters;

		/**
		 * easy implement using list
		 * key -- cluster id
		 * value -- [node's NeighborClusterInfo]
		 */
		unordered_map<int, list<NeighborClusterInfo> > neighborClusters;

		void updateLastHeartBeat(WaveShortMessageClusterBeacon* msg) {
			NeighborClusterInfo nci = {
				.nodeId = msg->getSenderAddress(),
				.neighborClusterId = msg->getSenderClusterId(),
				.timeStamp = msg->getTimestamp()
			};

			TraCICluster* tc = this;

			auto iter = tc->neighborClusters.find(nci.neighborClusterId);
			/**
			 * First node in the cluster
			 */
			if (iter == tc->neighborClusters.end()) {
				tc->neighborClusters[nci.neighborClusterId] = list<TraCICluster::NeighborClusterInfo>();
				tc->neighborClusters[nci.neighborClusterId].push_front(nci);
			} else {
				auto i = TraCICluster::findClusterNodeInfo(iter->second, nci);
				/**
				 * Found the node in the cluster
				 */
				if (i == iter->second.end()) {
					auto insertPos = TraCICluster::findClusterNodeInfoInsertPos(iter->second, nci);
					iter->second.insert(insertPos, nci);
				} else {
					/**
					 * update only when the packet is a newer packet
					 */
					if (nci.timeStamp > i->timeStamp) {
						iter->second.erase(i);
						/**
						 * insert after all the newer info
						 */
						auto insertPos = TraCICluster::findClusterNodeInfoInsertPos(iter->second, nci);
						iter->second.insert(insertPos, nci);
					}
				}
			}
		}

	protected:
		virtual void onBeacon(WaveShortMessage* wsm);
		virtual void onData(WaveShortMessage* wsm);
		void sendMessage(cModule* dstMod, int nextHopId, std::string content, unsigned long pkgLen);
		virtual void handlePositionUpdate(cObject* obj);
		virtual void handleParkingUpdate(cObject* obj);
		virtual void sendWSM(WaveShortMessage* wsm);

		virtual cModule* getDstNode(int option = PICK_ONE_NODE);
		virtual unsigned long getPkgLen() {
			if (packetLenMax == packetLenMin) { return packetLenMax; }
			return (unsigned long)(uniform(packetLenMin, packetLenMax));
		}

		//virtual void sendClusterHello();
		//virtual void sendClusterJoinRequest();
		//virtual void sendClusterJoinResponse();
		//virtual void sendClusterStatus();
		virtual void handleClusterBeaconSelfMsg(cMessage* msg);
		virtual void handleSendPacketSelfMsg(cMessage* msg);
		virtual void handleSelfMsg(cMessage* msg);

		virtual WaveShortMessageClusterBeacon* prepareWSMCB(int type, unsigned long pkgLen);

		virtual WaveShortMessageWithDst* prepareWSMWithDst(std::string name, int lengthBits,
					t_channel channel, int priority, int rcvId, int serial, Coord &rcvPos);

		virtual WaveShortMessageWithDst* prepareAndInitWSMWithDst(cModule* dstMod, int nextHopId, std::string content,
					unsigned long pkgLen) {

			static unsigned long sequenceNum = 0;

			t_channel channel = dataOnSch ? type_SCH : type_CCH;

			Coord currPos = getMyPosition();
			WaveShortMessageWithDst* wsm = prepareWSMWithDst("data", dataLengthBits, channel, dataPriority,
						dstMod->getId(), sequenceNum++, currPos);

			wsm->setByteLength(pkgLen);
			wsm->setNextHopId(nextHopId);
			wsm->setWsmData(content.c_str());
			wsm->setDstNodeId(dstMod->getName());
			return wsm;
		}

		Coord getMyPosition() const;
		const NicEntry::GateList* getMyNicGateList() const;

		static Coord getHostPosition(cModule* const host);
		static Coord getHostPosition(int hostId);
		static const NicEntry::GateList* getHostNicGateList(const BaseConnectionManager* const bcm,
					const cModule* host);


		static const NeighborNodeSet getNeighborNodes(const BaseConnectionManager* const bcm,
					const cModule* host);


		static NodeVector getFarNodes(NeighborNodeSet* nns, TraCIScenarioManagerLaunchd* traciSMLd,
					const BaseConnectionManager* const bcm,
					const cModule* host);

		NodeVector getHostFarNodes();

		static double distSqr(Coord& c1, Coord& c2) {
			return (c1.x - c2.x) * (c1.x - c2.x)
				+ (c1.y - c2.y) * (c1.y - c2.y)
				+ (c1.z - c2.z) * (c1.z - c2.z);
		}

		inline bool getRandomPermit(double p) {
			ASSERT(p >= .0 && p <= 1.0);
			return uniform(0, 1) <= p;
		}

		static int getNearestNodeToPos(NeighborNodeSet& nodes, Coord& pos) {
			double minDistSqr = (std::numeric_limits<double>::max)();
			int minDistNodeId = 0;

			for (auto iter = nodes.begin(); iter != nodes.end(); ++iter) {
				ASSERT(*iter != NULL);
				Coord nodePos = getHostPosition(*iter);
				double currDistSqr = distSqr(nodePos, pos);
				if (currDistSqr < minDistSqr) {
					minDistSqr = currDistSqr;
					minDistNodeId = (*iter)->getId();
				}
			}

			return minDistNodeId;
		}
};



class SingleNodeStrategy : public NodeStrategy {
	public:
		virtual void sendClusterHello(TraCICluster* tc) const {
			/**
			 * single node do not send hello
			 */
			ASSERT(false);
			//WaveShortMessageClusterBeacon* wsmcb = tc->prepareWSMCB(NodeClusterRole::HELLO, tc->headerLength);
			//tc->sendWSM(wsmcb);
		}

		virtual void sendClusterJoinRequest(TraCICluster* tc, int headId) const {
			/**
			 * unicast to head node
			 */
			WaveShortMessageClusterBeacon* wsmcb = tc->prepareWSMCB(NodeClusterRole::JOIN_REQUEST, tc->headerLength);
			wsmcb->setRecipientAddress(headId);
			tc->sendWSM(wsmcb);
		}

		virtual void sendClusterJoinResponse(TraCICluster* tc, int hostId, int result) const {
			/**
			 * single node can not send join response
			 */
			ASSERT(false);
		}

		virtual void sendClusterStatus(TraCICluster* tc, int hostId) const {
			/**
			 * single node can not send cluster status
			 */
			ASSERT(false);
		}

		virtual void onClusterHello(TraCICluster* tc, WaveShortMessageClusterBeacon* msg) const {
			tc->updateLastHeartBeat(msg);
		}

		virtual void onClusterJoinRequest(TraCICluster* tc, WaveShortMessageClusterBeacon* msg) const {
			// drop
		}

		virtual void onClusterJoinResponse(TraCICluster* tc, WaveShortMessageClusterBeacon* msg) const {
			const char* result = msg->getWsmData();
			if (strcmp(result, "y") == 0) {
				tc->clusterId = msg->getSenderClusterId();
				tc->clusterHeadId = msg->getSenderAddress();
				tc->setClusterRole(NodeClusterRole::MEMBER);
				/**
				 * TODO: Cancel self elect timer
				 */
			} else if (strcmp(result, "n") == 0) {
				/**
				 * TODO: Add statistic code
				 */
			} else {
				ASSERT(false);
			}
		}

		virtual void onClusterStatus(TraCICluster* tc, WaveShortMessageClusterBeacon* msg) const {
			/**
			 * Not implemented
			 */
			ASSERT(false);
		}
};

class HeadNodeStrategy : public NodeStrategy {
	public:
		virtual void sendClusterHello(TraCICluster* tc) const {
			WaveShortMessageClusterBeacon* wsmcb = tc->prepareWSMCB(NodeClusterRole::HELLO, tc->headerLength);
			wsmcb->setNodeData(tc->getClusterNodeData());
			tc->sendWSM(wsmcb);
		}
		virtual void sendClusterJoinRequest(TraCICluster* tc, int headId) const {
			/**
			 * head id want to join other cluster
			 * TODO: More actions may be added
			 */
			WaveShortMessageClusterBeacon* wsmcb = tc->prepareWSMCB(NodeClusterRole::JOIN_REQUEST, tc->headerLength);
			wsmcb->setNodeData(tc->getClusterNodeData());
			wsmcb->addByteLength(yy::getHeadDataSize(*tc->getClusterNodeData().hd));
			tc->sendWSM(wsmcb);
		}
		virtual void sendClusterJoinResponse(TraCICluster* tc, int hostId, int result) const {
			ASSERT(result == NodeClusterRole::JOIN_ACC || result == NodeClusterRole::JOIN_REJ);
			const char* content = (result == NodeClusterRole::JOIN_ACC ? "y" : "n");
			WaveShortMessageClusterBeacon* wsmcb = tc->prepareWSMCB(NodeClusterRole::JOIN_REQUEST, tc->headerLength);
			wsmcb->setRecipientAddress(hostId);
			wsmcb->setWsmData(content);
		}
		virtual void sendClusterStatus(TraCICluster* tc, int hostId) const {
			/**
			 * TODO: Currently do not support cluster status query
			 */
			ASSERT(false);
		}

		virtual void onClusterHello(TraCICluster* tc, WaveShortMessageClusterBeacon* msg) const {

			tc->updateLastHeartBeat(msg);
			/**
			 * hello from other clusters, just update neighbor info
			 */
			if (msg->getSenderClusterId() != tc->clusterId) {
				return;
			}

			switch(msg->getSenderRole()) {
				case NodeClusterRole::SINGLE:
					/**
					 * SINGLE node do not send hello
					 */
					ASSERT(false);
					break;
				case NodeClusterRole::HEAD:
				case NodeClusterRole::HEAD_BAK:
					/**
					 * TODO: current only one head in one cluster
					 */
					ASSERT(false);
					break;
				case NodeClusterRole::GATEWAY:
				case NodeClusterRole::GATEWAY_BAK: /* Fall through */
					{
						HeadData* hd = tc->getClusterNodeData().hd;
						GateWayData* gwd = msg->getNodeData().gwd;
						int hostId = msg->getSenderAddress();
						hd->gateWayIds.insert(hostId);
						for (auto iter = gwd->connectedClusters.begin();
									iter != gwd->connectedClusters.end(); ++iter) {
							int clusterId = iter->first;
							int degree = iter->second;

							/**
							 * Never attach to this cluster before
							 */
							if (hd->gateWayInfo.find(clusterId) == hd->gateWayInfo.end()) {
								hd->gateWayInfo[clusterId] = unordered_map<int, int>();
							}
							hd->gateWayInfo[clusterId][hostId] = degree;
						}
					}
					break;
				case NodeClusterRole::MEMBER:
					{
						HeadData* hd = tc->getClusterNodeData().hd;
						hd->memberIds.insert(msg->getSenderAddress());
					}
					break;
				default:
					ASSERT(false);
					break;

			}
		}

		virtual void onClusterJoinRequest(TraCICluster* tc, WaveShortMessageClusterBeacon* msg) const {
			HeadData* hd = tc->getClusterNodeData().hd;
			int result = NodeClusterRole::JOIN_ACC;
			/**
			 * Reach cluster size limitation
			 */
			if (tc->clusterMaxSize &&
					hd->gateWayIds.size() + hd->memberIds.size() + 1 >= tc->clusterMaxSize) {
				result = NodeClusterRole::JOIN_REJ;
			}

			sendClusterJoinResponse(tc, msg->getSenderAddress(), result);
		}

		virtual void onClusterJoinResponse(TraCICluster* tc, WaveShortMessageClusterBeacon* msg) const {
			/**
			 * Join other cluster
			 */
			int clusterId = msg->getSenderClusterId();
			int headId = msg->getSenderAddress();

			tc->setClusterRole(NodeClusterRole::MEMBER);
			tc->clusterId = clusterId;
			tc->clusterHeadId = headId;
		}

		virtual void onClusterStatus(TraCICluster* tc, WaveShortMessageClusterBeacon* msg) const {
			/**
			 * Not implemented
			 */
			ASSERT(false);
		}
};

class GateWayNodeStrategy : public NodeStrategy {
	public:
		virtual void sendClusterHello(TraCICluster* tc) const {
			WaveShortMessageClusterBeacon* wsmcb = tc->prepareWSMCB(NodeClusterRole::HELLO, tc->headerLength);
			wsmcb->setNodeData(tc->getClusterNodeData());
			tc->sendWSM(wsmcb);
		}
		virtual void sendClusterJoinRequest(TraCICluster* tc, int headId) const {
			/**
			 * Gateway node want to join other cluster
			 * TODO: More actions may be added
			 */
			WaveShortMessageClusterBeacon* wsmcb = tc->prepareWSMCB(NodeClusterRole::JOIN_REQUEST, tc->headerLength);
			wsmcb->setNodeData(tc->getClusterNodeData());
			wsmcb->addByteLength(yy::getHeadDataSize(*tc->getClusterNodeData().hd));
			tc->sendWSM(wsmcb);
		}
		virtual void sendClusterJoinResponse(TraCICluster* tc, int hostId, int result) const {
		}
		virtual void sendClusterStatus(TraCICluster* tc, int hostId) const {
		}

		virtual void onClusterHello(TraCICluster* tc, WaveShortMessageClusterBeacon* msg) const {
		}

		virtual void onClusterJoinRequest(TraCICluster* tc, WaveShortMessageClusterBeacon* msg) const {
		}

		virtual void onClusterJoinResponse(TraCICluster* tc, WaveShortMessageClusterBeacon* msg) const {
		}

		virtual void onClusterStatus(TraCICluster* tc, WaveShortMessageClusterBeacon* msg) const {
		}
};

class MemberNodeStrategy : public NodeStrategy {
	public:
		virtual void sendClusterHello(TraCICluster* tc) const {
			WaveShortMessageClusterBeacon* wsmcb = tc->prepareWSMCB(NodeClusterRole::HELLO, tc->headerLength);
			wsmcb->setNodeData(tc->getClusterNodeData());
			tc->sendWSM(wsmcb);
		}
		virtual void sendClusterJoinRequest(TraCICluster* tc, int headId) const {
		}
		virtual void sendClusterJoinResponse(TraCICluster* tc, int hostId, int result) const {
		}
		virtual void sendClusterStatus(TraCICluster* tc, int hostId) const {
		}

		virtual void onClusterHello(TraCICluster* tc, WaveShortMessageClusterBeacon* msg) const {
		}

		virtual void onClusterJoinRequest(TraCICluster* tc, WaveShortMessageClusterBeacon* msg) const {
		}

		virtual void onClusterJoinResponse(TraCICluster* tc, WaveShortMessageClusterBeacon* msg) const {
		}

		virtual void onClusterStatus(TraCICluster* tc, WaveShortMessageClusterBeacon* msg) const {
		}
};


#endif
