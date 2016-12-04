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

#include "veins/modules/application/traci/NopAppl.h"


Define_Module(NopAppl);

void NopAppl::initialize(int stage) {
	BaseWaveApplLayer::initialize(stage);
	if (stage == 0) {
	}
}

void NopAppl::onBeacon(WaveShortMessage* wsm) {
}

void NopAppl::onData(WaveShortMessage* wsm) {
}

void NopAppl::sendMessage(std::string blockedRoadId) {

	t_channel channel = dataOnSch ? type_SCH : type_CCH;
	WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
	wsm->setWsmData(blockedRoadId.c_str());
	sendWSM(wsm);
}
void NopAppl::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
}
void NopAppl::handleParkingUpdate(cObject* obj) {
}
void NopAppl::handlePositionUpdate(cObject* obj) {
}
void NopAppl::sendWSM(WaveShortMessage* wsm) {
	sendDelayedDown(wsm,individualOffset);
}
