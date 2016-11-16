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

#ifndef TraCIMyExp11p_H
#define TraCIMyExp11p_H

#include <unordered_set>
#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/mobility/traci/TraCIScenarioManagerLaunchd.h"

#include "veins/modules/messages/WaveShortMessageWithDst_m.h"

#include <limits>

using Veins::TraCIMobility;
using Veins::TraCICommandInterface;
using Veins::TraCIScenarioManagerLaunchd;
using Veins::TraCIScenarioManagerLaunchdAccess;
using Veins::AnnotationManager;

using std::vector;

/**
 * Small IVC Demo using 11p
 */
class TraCIMyExp11p : public BaseWaveApplLayer {
	public:
		virtual void initialize(int stage);
		virtual void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details);
		virtual void finish();
	protected:
		typedef std::unordered_set<cModule*> NeighborNodeSet;
		typedef std::vector<cModule*> NodeVector;

		/**
		 * Do not access this below four vars directly
		 * use getCachedFarNodes and getCachedNeighborNodes
		 */
		TraCIMyExp11p::NeighborNodeSet neighborNodes;
		simtime_t neighborNodesUpdateTime;
		TraCIMyExp11p::NodeVector farNodes;
		simtime_t farNodesUpdateTime;

		TraCIMyExp11p::NeighborNodeSet* getCachedNeighborNodes() {
			if (simTime() != neighborNodesUpdateTime) {
				neighborNodesUpdateTime = simTime();
				neighborNodes = getNeighborNodes(bcm, findHost());
			}
			return &neighborNodes;
		}

		TraCIMyExp11p::NodeVector* getCachedFarNodes() {
			if (simTime() != farNodesUpdateTime) {
				farNodesUpdateTime = simTime();
				farNodes = getHostFarNodes();
			}
			return &farNodes;
		}

		TraCIMobility* mobility;
		TraCICommandInterface* traci;
		TraCICommandInterface::Vehicle* traciVehicle;
		AnnotationManager* annotations;
		simtime_t lastDroveAt;
		bool sentMessage;
		bool isParking;
		bool sendWhileParking;
		static const simsignalwrap_t parkingStateChangedSignal;
		double recvDataLength;
		double sendDataLength;

		double sendNearPosibility;
		unsigned long packetSentInterval;

		int sequenceNum;
		unsigned long packetLenMin;
		unsigned long packetLenMax;

		BaseConnectionManager *bcm;
		TraCIScenarioManagerLaunchd* traciSMLd;

		/**
		 * record the current neighbor count
		 */
		cOutVector currentNeighborCnt;
		int prevNeighborCnt;

		cMessage sendMessageSignal;
		const static simsignalwrap_t neighborCntStatistic;
	protected:
		virtual void onBeacon(WaveShortMessage* wsm);
		virtual void onData(WaveShortMessage* wsm);
		void sendMessage(int dstId, int nextHopId, std::string content);
		virtual void handlePositionUpdate(cObject* obj);
		virtual void handleParkingUpdate(cObject* obj);
		virtual void sendWSM(WaveShortMessage* wsm);

		virtual cModule* getDstNode();
		virtual unsigned long getPkgLen() {
			if (packetLenMax == packetLenMin) { return packetLenMax; }
			return (unsigned long)(uniform(packetLenMin, packetLenMax));
		}

		virtual void handleSelfMsg(cMessage* msg);
		virtual WaveShortMessageWithDst* prepareWSMWithDst(std::string name, int lengthBits,
					t_channel channel, int priority, int rcvId, int serial, Coord &rcvPos);

		Coord getMyPosition() const;
		const NicEntry::GateList* getMyNicGateList() const;

		static Coord getHostPosition(cModule* const host);
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

		static int getNearestNodeToPos(NeighborNodeSet& nodes, Coord& pos) {
			double minDistSqr = (std::numeric_limits<double>::max)();
			int minDistNodeId = 0;

			for (auto iter = nodes.begin(); iter != nodes.end(); ++iter) {
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

#endif
