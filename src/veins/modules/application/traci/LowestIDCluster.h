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

#ifndef __CLUSTERLIB_HIGHESTDEGREECLUSTER_H_
#define __CLUSTERLIB_HIGHESTDEGREECLUSTER_H_

#include <omnetpp.h>

//#include <MiXiMDefs.h>
//#include <BaseNetwLayer.h>

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/mobility/traci/TraCIScenarioManagerLaunchd.h"

#include <set>
#include <map>

//#include "MdmacControlMessage_m.h"
#include "MdmacNetworkLayer.h"

/**
 * Implements the Highest Degree Clustering mechanism.
 * The degree is the number of nodes with which this node has a connection.
 */
class LowestIDCluster : public MdmacNetworkLayer {

protected:
    /** @brief Compute the CH weight for this node. */
    double calculateWeight();
	virtual int getNextHopId(int dstId);
	virtual int getNearestNodeToPos(const Coord& pos);
	virtual int getNearestNodeToPos(const unordered_map<int, unordered_map<int, int> >& neighbors, const Coord& pos);

	virtual int headGateWayGetNextHopId(int dstId);
	virtual int headGetNextHopId(int dstId);
	virtual int gateWayGetNextHopId(int dstId);
	virtual int memberGetNextHopId(int dstId);

};

#endif
