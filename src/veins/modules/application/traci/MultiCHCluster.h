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

#ifndef __CLUSTERLIB_MULTICHCLUSTER_H_
#define __CLUSTERLIB_MULTICHCLUSTER_H_

#include <omnetpp.h>

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/mobility/traci/TraCIScenarioManagerLaunchd.h"

#include <set>
#include <map>
#include <vector>
#include <queue>
using std::priority_queue;

//#include "MdmacControlMessage_m.h"
#include "MdmacNetworkLayer.h"
#include "HighestDegreeCluster.h"
#include "ApplMapManager.h"

/**
 * Implements the Highest Degree Clustering mechanism.
 * The degree is the number of nodes with which this node has a connection.
 */
class MultiCHCluster : public HighestDegreeCluster {
public:
	virtual void initialize(int);
	struct NodeWeight {
		int nodeId;
		double weight;
		NodeWeight(int id, int w) {
			nodeId = id;
			weight = w;
		}
	};
	class NodeWeightGreater {
		public:
			bool operator()(NodeWeight& n1, NodeWeight& n2) {
				return n1.weight > n2.weight;
			}
	};

protected:
    /** @brief Compute the CH weight for this node. */
    double calculateWeight();

	vector<int> chooseBackupHeads();

	virtual int getNearestNodeToPos(const Coord& pos);
	virtual int getNearestNodeToPos(const unordered_map<int, unordered_map<int, int> >& neighbors, const Coord& pos);

	int headGateWayGetNextHopId(int dstId);
	int headGetNextHopId(int dstId);
	int gateWayGetNextHopId(int dstId);
	int memberGetNextHopId(int dstId);

	int getBestHeadAsNextHop(unordered_set<int>&, int);
	int getBestGateWayAsNextHop(unordered_map<int, int>&, int);

    /** @brief Handle a HELLO message. */
    virtual void receiveHelloMessage( MdmacControlMessage* );

    /** @brief Handle a CH message. */
    virtual void receiveChMessage( MdmacControlMessage* );

    /** @brief Handle a JOIN message. */
    virtual void receiveJoinMessage( MdmacControlMessage* );

	virtual void processBeat();

	vector<double> futureTimes;
	vector<double> futureConfidenceFactor;
	int backupHeadMaxLimit;
	int backupSelectFactor;
};

#endif
