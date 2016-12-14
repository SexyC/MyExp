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

#include "ClusterManager.h"
using Veins::TraCIScenarioManager;

void ClusterManager::clusterInit(int id, int headId, set<int>& members, double time) {

	if (clustersInfo.find(id) != clustersInfo.end()) {
		clustersInfo.erase(id);
	}

	ClusterStat cs(time);
	cs.heads.insert(headId);
	for(auto iter = members.begin(); iter != members.end(); ++iter) {
		//cs.members.insert(*iter);
		nodeClusterMap[*iter] = id;
	}

	cs.members = &members;
	clustersInfo[id] = cs;
	csEV << "cluster init, id: " << id << ", headId: " << headId
		<< ", time: " << time << endl;

	nodeClusterMap[headId] = id;
}

void ClusterManager::clusterDie(int id, double time) {
	csEV << "cluster died, id: " << id << ", time: " << time << endl;

	auto iter = clustersInfo.find(id);
	if (iter != clustersInfo.end()) {
		iter->second.endTime = time;
		for (auto i = iter->second.heads.begin(); i != iter->second.heads.end();
					++i) {
			nodeClusterMap[*i] = -1;
		}
		for (auto i = iter->second.members->begin(); i != iter->second.members->end();
					++i) {
			nodeClusterMap[*i] = -1;
		}
		clustersInfo.erase(id);
	}
}

void ClusterManager::joinCluster(int clusterId, int nodeId, double time) {
	/**
	 * FIXME: At first, before the head node init the cluster
	 * other nodes can call joinCluster at the same time head node init the cluster
	 * But they were called first
	 * This could cause there is no cluster found
	 *
	 * Work around: leave it, cluster init will take care
	 */

	/**
	 * If previously it's a cluster head
	 */
	if (clustersInfo.find(nodeId) != clustersInfo.end()) {
		clustersInfo.erase(nodeId);
	}

	if (clustersInfo.find(clusterId) == clustersInfo.end()) {
		csEV << time << " join cluster failed, cluster:" << clusterId <<  " not exist any more" << endl;
		return;
	}
	csEV << "join cluster, id: " << clusterId << ", node id: "
		<< nodeId << ", time: " << time << endl;
	//clustersInfo[clusterId].members.insert(nodeId);

	nodeClusterMap[nodeId] = clusterId;
}

void ClusterManager::leaveCluster(int clusterId, int nodeId, double time) {

	//if (clustersInfo.find(nodeId) != clustersInfo.end()) {
	//	clustersInfo.erase(nodeId);
	//}

	if (clustersInfo.find(clusterId) == clustersInfo.end()) {
		csEV << time << " leave cluster failed, cluster:" << clusterId << " not exist any more" << endl;
	} else {
		//clustersInfo[clusterId].members->erase(nodeId);
	}
	csEV << "leave cluster, id: " << clusterId << ", node id: "
		<< nodeId << ", time: " << time << endl;
	nodeClusterMap[nodeId] = -1;
}

void ClusterManager::registerBackHead(int clusterId, const vector<int>& backHeads) {
	auto iter = clustersInfo.find(clusterId);
	ASSERT(iter != clustersInfo.end());
	iter->second.heads.clear();
	for (auto i = backHeads.begin(); i != backHeads.end(); ++i) {
		iter->second.heads.insert(*i);
	}
	iter->second.heads.insert(clusterId);
}

bool ClusterManager::isBackHead(int clusterId, int nodeId) {

	auto iter = clustersInfo.find(clusterId);
	/**
	 * This cluster is dead,
	 * poor node doesn't know it
	 */
	if (iter == clustersInfo.end()) {
		return false;
	}

	return (iter->second.heads.find(nodeId) != iter->second.heads.end())
		&& (nodeId != clusterId);
}

void ClusterManager::nodeFinished(int nodeId) {
	if (clustersInfo.find(nodeId) != clustersInfo.end()) {
		clustersInfo.erase(nodeId);
	}
	auto iter = nodeClusterMap.find(nodeId);
	if (iter != nodeClusterMap.end()) {
		int clusterId = iter->second;
		auto i = clustersInfo.find(clusterId);
		if (i != clustersInfo.end()) {
			i->second.heads.erase(nodeId);
			i->second.members->erase(nodeId);
		}
	}
}

/**
 * could return NULL if the cluster is died
 * or not init by head yet
 */
unordered_map<int, unordered_map<int, int> >*
ClusterManager::getNeighborClusters(int id, double time, bool forceUpdate) {
	/**
	 * iter->first -- cluster id
	 * iter->second -- cluster stat
	 */
	auto iter = clustersInfo.find(id);
	//ASSERT(iter != clustersInfo.end());
	/**
	 * If the cluster is not found, could be dead
	 * or not init by the cluster head yet
	 * return NULL
	 */
	if (iter == clustersInfo.end()) { return NULL; }

	/**
	 * key -- cluster id
	 * val -- connection cnt with the cluster
	 */
	unordered_map<int, int>* nodeNeighbourCluster = NULL;
	if (forceUpdate || iter->second.neighborClusterUpdateTime < time) {
		iter->second.neighborClusters.clear();

		/**
		 * iterate over all members in the cluster
		 * *i -- node id
		 */
		for(auto i = iter->second.members->begin(); i != iter->second.members->end();
					++i) {

			nodeNeighbourCluster = nodeNeighbourClusterInfo[*i];
			//ASSERT(nodeNeighbourCluster != NULL);
			if (nodeNeighbourCluster == NULL) {
				/**
				 * maybe this node is finished
				 */
				continue;
			}

			/**
			 * iterate over all the neighbor cluster id of one single node in this cluster
			 * cIter->first -- neighbor cluster id
			 * cIter->second -- connection cnt with this neighbor cluster
			 */
			for(auto cIter = nodeNeighbourCluster->begin(); cIter != nodeNeighbourCluster->end(); ++cIter) {
				if (iter->second.neighborClusters.find(cIter->first)
							== iter->second.neighborClusters.end()) {
					iter->second.neighborClusters[cIter->first] = unordered_map<int, int>();
				}
				iter->second.neighborClusters[cIter->first][*i] = cIter->second;
				//cout << "cluster id: " << cIter->first << " gateway: " << *i << " degree: " << cIter->second << endl;
			}
		}

		/**
		 * iterate over all heads in the cluster
		 */
		for(auto i = iter->second.heads.begin(); i != iter->second.heads.end();
					++i) {
			nodeNeighbourCluster = nodeNeighbourClusterInfo[*i];
			//ASSERT(nodeNeighbourCluster != NULL);
			if (nodeNeighbourCluster == NULL) {
				/**
				 * maybe this back head is finished
				 */
				continue;
			}

			/**
			 * iterate over all the neighbor cluster id of one single node in this cluster
			 * cIter->first -- neighbor cluster id
			 * cIter->second -- connection cnt with this neighbor cluster
			 */
			for(auto cIter = nodeNeighbourCluster->begin(); cIter != nodeNeighbourCluster->end(); ++cIter) {
				if (iter->second.neighborClusters.find(cIter->first)
							== iter->second.neighborClusters.end()) {
					iter->second.neighborClusters[cIter->first] = unordered_map<int, int>();
				}
				iter->second.neighborClusters[cIter->first][*i] = cIter->second;
			}
		}
	}
	return &iter->second.neighborClusters;
}

unordered_map<int, unordered_map<int, int> >
ClusterManager::getNeighborClusters(vector<int>& neighborNodes, bool forceUpdate) {

	/**
	 * key -- cluster id
	 * val -- connection cnt with the cluster
	 */
	unordered_map<int, int>* nodeNeighbourCluster = NULL;
	unordered_map<int, unordered_map<int, int> > result;

	/**
	 * iterate over all members in the cluster
	 * *i -- node id
	 */
	for (auto i = neighborNodes.begin(); i != neighborNodes.end(); ++i) {

		nodeNeighbourCluster = nodeNeighbourClusterInfo[*i];
		if (nodeNeighbourCluster == NULL) {
			/**
			 * maybe this node is finished
			 */
			continue;
		}

		/**
		 * iterate over all the neighbor cluster id of one single node in this cluster
		 * cIter->first -- neighbor cluster id
		 * cIter->second -- connection cnt with this neighbor cluster
		 */
		for(auto cIter = nodeNeighbourCluster->begin(); cIter != nodeNeighbourCluster->end(); ++cIter) {
			if (result.find(cIter->first) == result.end()) {
				result[cIter->first] = unordered_map<int, int>();
			}
			result[cIter->first][*i] = cIter->second;
		}
	}

	return result;
}

