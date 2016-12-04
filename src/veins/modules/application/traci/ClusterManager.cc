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

void ClusterManager::clusterInit(int id, int headId, double time) {
	ClusterStat cs(time);
	cs.heads.insert(headId);
	clustersInfo[id] = cs;
	csEV << "cluster init, id: " << id << ", headId: " << headId
		<< ", time: " << time << endl;
}

void ClusterManager::clusterDie(int id, double time) {
	csEV << "cluster died, id: " << id << ", time: " << time << endl;
	clustersInfo.erase(id);
}

void ClusterManager::joinCluster(int clusterId, int nodeId, double time) {
	if (clustersInfo.find(clusterId) == clustersInfo.end()) {
		csEV << "join cluster failed, cluster not exist any more" << endl;
		return;
	}
	csEV << "join cluster, id: " << clusterId << ", node id: "
		<< nodeId << ", time: " << time << endl;
	clustersInfo[clusterId].members.insert(nodeId);
}

void ClusterManager::leaveCluster(int clusterId, int nodeId, double time) {
	csEV << "leave cluster, id: " << clusterId << ", node id: "
		<< nodeId << ", time: " << time << endl;
	clustersInfo[clusterId].members.erase(nodeId);
}

