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
#include "veins/modules/application/traci/TraCIStat.h"

using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;

const simsignalwrap_t TraCIStat::parkingStateChangedSignal = simsignalwrap_t(TRACI_SIGNAL_PARKING_CHANGE_NAME);

const simsignalwrap_t TraCIStat::neighborCntStatistic = simsignalwrap_t("TraCIStat_neighborCntStatistic");

Define_Module(TraCIStat);

Coord TraCIStat::getMyPosition() const {
	return mobility->getCurrentPosition();
}

Coord TraCIStat::getHostPosition(cModule* const host) {

	ASSERT(host != NULL);
	for (cModule::SubmoduleIterator iter(host); !iter.end(); ++iter) {
		cModule* submod = SUBMODULE_ITERATOR_TO_MODULE(iter);
		TraCIMobility* mm = dynamic_cast<TraCIMobility*>(submod);

		if (!mm) continue;
		return mm->getPositionAt(simTime());
	}
	ASSERT(false);
	return Coord();
}

Coord TraCIStat::getHostPosition(int hostId) {

	cModule* host = cSimulation::getActiveSimulation()->getModule(hostId);
	ASSERT(host != NULL);
	return TraCIStat::getHostPosition(host);
}

const NicEntry::GateList* TraCIStat::getMyNicGateList() const {
	return getHostNicGateList(bcm, findHost());
}

const NicEntry::GateList* TraCIStat::getHostNicGateList(const BaseConnectionManager* const bcm,
			const cModule* host) {
	ASSERT(bcm != NULL && host != NULL);
	cModule* nicSubMod = host->getSubmodule("nic");
	return &bcm->getGateList(nicSubMod->getId());
}

const TraCIStat::NeighborNodeSet 
TraCIStat::getNeighborNodes(const BaseConnectionManager* const bcm,
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

TraCIStat::NodeVector
TraCIStat::getFarNodes(NeighborNodeSet* nns, TraCIScenarioManagerLaunchd* traciSMLd,
			const BaseConnectionManager* const bcm,
			const cModule* host) {

	ASSERT(nns != NULL && traciSMLd != NULL && bcm != NULL && host != NULL);
	const std::map<std::string, cModule*> &hosts = traciSMLd->getManagedHosts();

	TraCIStat::NodeVector result;
	result.reserve(hosts.size() - nns->size());

	for (auto iter = hosts.begin(); iter != hosts.end(); ++iter) {
		if (nns->find(iter->second) == nns->end()) {
			result.push_back(iter->second);
		}
	}

	return result;
}

TraCIStat::NodeVector TraCIStat::getHostFarNodes() {
	NeighborNodeSet* nns = getCachedNeighborNodes();
	return getFarNodes(nns, traciSMLd, bcm, findHost());
}

void TraCIStat::initialize(int stage) {
	BaseWaveApplLayer::initialize(stage);
	if (stage == 0) {
		mobility = TraCIMobilityAccess().get(getParentModule());
		traci = mobility->getCommandInterface();
		traciVehicle = mobility->getVehicleCommandInterface();
		annotations = AnnotationManagerAccess().getIfExists();
		ASSERT(annotations);

		neighborNodesUpdateTime = simTime();
		farNodesUpdateTime = simTime();
		findHost()->subscribe(parkingStateChangedSignal, this);

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
		prevNeighborCnt = -1;

		neighborRecordInterval = hasPar("neighborRecordInterval") ?
			par("neighborRecordInterval") : 5;

	} else if (stage == 1) {
		if (neighborRecordInterval > 0) {
			//scheduleAt(simTime() + packetSentInterval, &sendMessageSignal);
			scheduleAt(simTime() + neighborRecordInterval, new cMessage());
		}
	}
}

void TraCIStat::finish() {

	//recordScalar("final_neighbor_count", prevNeighborCnt);
}

void TraCIStat::onBeacon(WaveShortMessage* wsm) {
}

void TraCIStat::onData(WaveShortMessage* wsm) {
	ASSERT(false);
}

void TraCIStat::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
	Enter_Method_Silent();
	if (signalID == mobilityStateChangedSignal) {
		handlePositionUpdate(obj);
	}
	else if (signalID == parkingStateChangedSignal) {
		handleParkingUpdate(obj);
	}
}

void TraCIStat::handleSelfMsg(cMessage* msg) {
	scheduleAt(simTime() + neighborRecordInterval, msg);

	NeighborNodeSet* nns = getCachedNeighborNodes();
	currentNeighborCnt.record(nns->size());
}

void TraCIStat::handleParkingUpdate(cObject* obj) { }

void TraCIStat::handlePositionUpdate(cObject* obj) {
	BaseWaveApplLayer::handlePositionUpdate(obj);

	NeighborNodeSet* neighborNodes = getCachedNeighborNodes(true);

	if ((int) neighborNodes->size() != prevNeighborCnt) {
		currentNeighborCnt.record(neighborNodes->size());
		prevNeighborCnt = neighborNodes->size();
		emit(neighborCntStatistic, neighborNodes->size());
	}

}

