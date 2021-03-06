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

#ifndef ClusterManaget_H
#define ClusterManaget_H

//#include <omnetpp.h>
//
//#include <MiXiMDefs.h>
//#include <BaseNetwLayer.h>

//#include <fstream>
#include <set>
#include <map>

#include <vector>
#include <unordered_map>
#include <unordered_set>

#include "veins/modules/messages/MdmacControlMessage_m.h"
#include "veins/modules/messages/WaveShortMessageWithDst_m.h"
#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"

using Veins::AnnotationManager;
using Veins::TraCIMobility;
using Veins::TraCIMobilityAccess;
using Veins::TraCIScenarioManager;
using Veins::TraCIScenarioManagerAccess;
using Veins::TraCICommandInterface;

using std::unordered_map;
using std::unordered_set;
using std::vector;
using std::set;
using std::cout;
using std::endl;
#define csEV EV

/**
 * This module implements the clustering mechanism for
 * Modified Distributed Mobility-Aware Clustering (M-DMAC).
 * There is a pure virtual function that must be implemented by
 * inheriting cluster modules, which provides the weighting function
 * for the particular cluster algorithm being tested.
 */
class ClusterManager
{

public:
	static ClusterManager* getClusterManager() {
		static ClusterManager cm;
		return &cm;
	}
	class ClusterStat {
		public:
			ClusterStat() { }
			ClusterStat(double time) {
				startTime = time;
				members = NULL;
			}
			unordered_set<int> heads;
			//unordered_set<int> gateWays;
			//unordered_set<int> members;
			std::set<int>* members;
			/**
			 * key -- cluster id
			 * val -- map
			 *          key -- node id
			 *          val -- connection cnt with that cluster
			 */
			unordered_map<int, unordered_map<int, int> > neighborClusters;

			double neighborClusterUpdateTime;
			double startTime;
			double endTime;
	};

	void clusterInit(int id, int headId, set<int>& members, double time);
	void clusterDie(int id, double time);
	void joinCluster(int clusterId, int nodeId, double time);
	void leaveCluster(int clusterId, int nodeId, double time);

	void registerBackHead(int clutserId, const vector<int>& backHeads);
	bool isBackHead(int clusterId, int nodeId);

	int getClusterIdByNodeId(int nodeId) {
		auto iter = nodeClusterMap.find(nodeId);
		if (iter == nodeClusterMap.end()) {
			return -1;
		}
		return iter->second;
	}

	void nodeNeighbourClusterInfoDelete(int id) {
		nodeNeighbourClusterInfo.erase(id);
	}
	void nodeNeighbourClusterInfoUpdate(int id, unordered_map<int, int>* s) {
		nodeNeighbourClusterInfo[id] = s;
	}

	unordered_map<int, ClusterStat>::size_type
		getClusterCount() { return clustersInfo.size(); }

	/**
	 * key -- cluster id
	 * val -- map
	 *         key -- gateway node id
	 *         val -- connection cnt with this cluster
	 */
	unordered_map<int, unordered_map<int, int> >* getNeighborClusters(int id, double time, bool forceUpdate = false);
	/**
	 * get the cluster connected to this node's neighbor node
	 * return the same as above
	 */
	unordered_map<int, unordered_map<int, int> > getNeighborClusters(vector<int>& neighborNodes, bool forceUpdate = false);

	void nodeFinished(int nodeId);

	unordered_set<int>* getClusterHeads(int clusterId) {
		auto iter = clustersInfo.find(clusterId);
		if (iter == clustersInfo.end()) { return NULL; }
		return &iter->second.heads;
	}

protected:
	unordered_map<int, ClusterStat> clustersInfo;

	/**
	 * key -- node id
	 * val -- cluster id
	 */
	unordered_map<int, int> nodeClusterMap;

	unordered_map<int, unordered_map<int, int>* > nodeNeighbourClusterInfo;

private:
	ClusterManager() {}
	ClusterManager(const ClusterManager&) = delete;
	const ClusterManager operator=(const ClusterManager&) = delete;
	~ClusterManager() {}

};

#endif
