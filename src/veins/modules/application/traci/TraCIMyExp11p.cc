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
#include "veins/modules/application/traci/TraCIMyExp11p.h"

using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;

const simsignalwrap_t TraCIMyExp11p::parkingStateChangedSignal = simsignalwrap_t(TRACI_SIGNAL_PARKING_CHANGE_NAME);

const simsignalwrap_t TraCIMyExp11p::neighborCntStatistic = simsignalwrap_t("neighborCntStatistic");

Define_Module(TraCIMyExp11p);

Coord TraCIMyExp11p::getMyPosition() const {
	return mobility->getCurrentPosition();
}

Coord TraCIMyExp11p::getHostPosition(cModule* const host) {

	//TraCIMobility* mobility = TraCIMobilityAccess().get(host);

	for (cModule::SubmoduleIterator iter(host); !iter.end(); ++iter) {
		cModule* submod = SUBMODULE_ITERATOR_TO_MODULE(iter);
		TraCIMobility* mm = dynamic_cast<TraCIMobility*>(submod);

		if (!mm) continue;
		return mm->getPositionAt(simTime());
	}
	assertTrue("TraCIMyExp11p::getHostPosition host TraCIMobility not found", false);
	return Coord();
	//assertTrue("can not get TraCIMobility from host", mobility != NULL);
	//return mobility->getCurrentPosition();
}

const NicEntry::GateList* TraCIMyExp11p::getMyNicGateList() const {
	return getHostNicGateList(bcm, findHost());
}

const NicEntry::GateList* TraCIMyExp11p::getHostNicGateList(const BaseConnectionManager* const bcm,
			const cModule* host) {
	cModule* nicSubMod = host->getSubmodule("nic");
	return &bcm->getGateList(nicSubMod->getId());
}

const TraCIMyExp11p::NeighborNodeSet 
TraCIMyExp11p::getNeighborNodes(const BaseConnectionManager* const bcm,
			const cModule* host) {

	const NicEntry::GateList* gl = getHostNicGateList(bcm, host);
	NeighborNodeSet result;

	for (auto iter = gl->begin(); iter != gl->end(); ++iter) {
		cModule* host = cSimulation::getActiveSimulation()->getModule(iter->first->hostId);
		result.insert(host);
	}

	return result;

}

TraCIMyExp11p::NodeVector
TraCIMyExp11p::getFarNodes(TraCIScenarioManagerLaunchd* traciSMLd,
			const BaseConnectionManager* const bcm,
			const cModule* host) {

	NeighborNodeSet nns = getNeighborNodes(bcm, host);
	const std::map<std::string, cModule*> &hosts = traciSMLd->getManagedHosts();

	TraCIMyExp11p::NodeVector result;
	result.reserve(hosts.size() - nns.size());

	for (auto iter = hosts.begin(); iter != hosts.end(); ++iter) {
		if (nns.find(iter->second) == nns.end()) {
			result.push_back(iter->second);
		}
	}

	return result;
}

TraCIMyExp11p::NodeVector
TraCIMyExp11p::getHostFarNodes() {
	return getFarNodes(traciSMLd, bcm, findHost());
}

void TraCIMyExp11p::initialize(int stage) {
	BaseWaveApplLayer::initialize(stage);
	if (stage == 0) {
		mobility = TraCIMobilityAccess().get(getParentModule());
		traci = mobility->getCommandInterface();
		traciVehicle = mobility->getVehicleCommandInterface();
		annotations = AnnotationManagerAccess().getIfExists();
		ASSERT(annotations);

		sentMessage = false;
		lastDroveAt = simTime();
		findHost()->subscribe(parkingStateChangedSignal, this);
		isParking = false;
		sendWhileParking = par("sendWhileParking").boolValue();
		recvDataLength = .0;
		sendDataLength = .0;

		traciSMLd = TraCIScenarioManagerLaunchdAccess().get();
		bcm = FindModule<BaseConnectionManager*>::findGlobalModule();

		if (bcm == NULL) {
			assertTrue("Could not find BaseConnectionManager module", false);
		}

		//const std::map<std::string, cModule*> &hosts = traciSMLd->getManagedHosts();

		// register nic to connection manager
		Coord pos = getMyPosition();
		bcm->registerNic(this->getParentModule()->getSubmodule("nic"), (ChannelAccess*) this->getParentModule()->getSubmodule("nic")->getSubmodule("phy80211p"), &pos);

		currentNeighborCnt.setName("neighbor_cnt");
		prevNeighborCnt = -1;

		sendNearPosibility = par("sendNearPosibility").doubleValue();

		//for(auto i = hosts.begin(); i != hosts.end(); ++i) {
		//	std::cout << i->first << " " << i->second->getName() << std::endl;
		//}
		//std::cout << std::endl;
	} else if (stage == 1) {
		scheduleAt(simTime() + 10, &sendMessageSignal);
	}
}

void TraCIMyExp11p::finish() {

	recordScalar("final_neighbor_count", prevNeighborCnt);
}

void TraCIMyExp11p::onBeacon(WaveShortMessage* wsm) {
}

void TraCIMyExp11p::onData(WaveShortMessage* wsm) {
	cModule *host = findHost();
	//findHost()->getDisplayString().updateWith("r=16,green");
	host->getDisplayString().updateWith("r=16,green");
	annotations->scheduleErase(1, annotations->drawLine(wsm->getSenderPos(), mobility->getPositionAt(simTime()), "blue"));

	EV << wsm->getWsmData() << std::endl;
	//if (mobility->getRoadId()[0] != ':') traciVehicle->changeRoute(wsm->getWsmData(), 9999);
	//if (!sentMessage) sendMessage(wsm->getWsmData());
}

void TraCIMyExp11p::sendMessage(std::string blockedRoadId) {
	sentMessage = true;

	t_channel channel = dataOnSch ? type_SCH : type_CCH;
	WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
	wsm->setWsmData(blockedRoadId.c_str());
	sendWSM(wsm);
}

void TraCIMyExp11p::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
	Enter_Method_Silent();
	if (signalID == mobilityStateChangedSignal) {
		handlePositionUpdate(obj);
	}
	else if (signalID == parkingStateChangedSignal) {
		handleParkingUpdate(obj);
	}
}

void TraCIMyExp11p::handleSelfMsg(cMessage* msg) {
}

void TraCIMyExp11p::handleParkingUpdate(cObject* obj) {
	isParking = mobility->getParkingState();
	if (sendWhileParking == false) {
		if (isParking == true) {
			(FindModule<BaseConnectionManager*>::findGlobalModule())->unregisterNic(this->getParentModule()->getSubmodule("nic"));
		}
		else {
			Coord pos = mobility->getCurrentPosition();
			(FindModule<BaseConnectionManager*>::findGlobalModule())->registerNic(this->getParentModule()->getSubmodule("nic"), (ChannelAccess*) this->getParentModule()->getSubmodule("nic")->getSubmodule("phy80211p"), &pos);
		}
	}
}

void TraCIMyExp11p::handlePositionUpdate(cObject* obj) {
	BaseWaveApplLayer::handlePositionUpdate(obj);

	// stopped for for at least 10s?
	if (mobility->getSpeed() < 1) {
		if (simTime() - lastDroveAt >= 10) {
			findHost()->getDisplayString().updateWith("r=16,red");
			if (!sentMessage) sendMessage(mobility->getRoadId());
		}
	}
	else {
		lastDroveAt = simTime();
	}
	Coord pos = mobility->getCurrentPosition();
	//EV << simTime() << " " << findHost()->getName()
	//	<< "change position to: " << pos.info() << std::endl;
	const std::map<std::string, cModule*> &hosts = traciSMLd->getManagedHosts();

	const NicEntry::GateList* gl = getMyNicGateList();
	const cModule* host = this->getParentModule();
//	std::cout << host->getName() << " " << gl->size() << " gl size" << std::endl;
//
//	for(auto i = hosts.begin(); i != hosts.end(); ++i) {
//		std::cout << i->first << " " << getHostPosition(i->second).info() << std::endl;
//	}
//	std::cout << std::endl;
	NeighborNodeSet neighborNodes = getNeighborNodes(bcm, findHost());

	if ((int) neighborNodes.size() != prevNeighborCnt) {
		currentNeighborCnt.record(neighborNodes.size());
		prevNeighborCnt = neighborNodes.size();
		emit(neighborCntStatistic, neighborNodes.size());
	}



	//std::cout << "Host name: " << findHost()->getName() 
	//	<< " pos: " << getMyPosition()
	//	<< std::endl;
	
	//for (auto iter = neighborNodes.begin(); iter != neighborNodes.end(); ++iter) {
	//	std::cout << (*iter)->getName() 
	//		<< " pos: " << getHostPosition(*iter)
	//		<< std::endl;
	//}
	//std::cout << std::endl;
	
	NodeVector nv = getHostFarNodes();

	for (auto iter = nv.begin(); iter != nv.end(); ++iter) {
		std::cout << (*iter)->getName() << std::endl;
	}
	std::cout << std::endl;
}

void TraCIMyExp11p::sendWSM(WaveShortMessage* wsm) {
	if (isParking && !sendWhileParking) return;
	sendDelayedDown(wsm,individualOffset);
}
