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
#include <limits>

#include "MultiCHCluster.h"
#define hdcEV  EV

Define_Module(MultiCHCluster);

void MultiCHCluster::initialize(int stage) {
	HighestDegreeCluster::initialize(stage);
	if (stage == 0) {
		const char *vstr = par("futureTimes").stringValue();
		futureTimes = cStringTokenizer(vstr).asDoubleVector();
		for (auto iter = futureTimes.begin(); iter != futureTimes.end();
					++iter) {
			cout << *iter << " ";
		}
		cout << endl;
	} else if (stage == 1) {
	}
}

/** @brief Compute the CH weight for this node. */
double MultiCHCluster::calculateWeight() {
	NeighborNodeSet* nns = getCachedNeighborNodes();
	
	double weight = .0;
	
	vector<double> weightFactors(futureTimes.size());

	for (int wIdx = 0; wIdx < weightFactors.size(); ++wIdx) {
		for (auto iter = nns->begin(); iter != nns->end(); ++iter) {
			Coord nodePos = getHostPosition(*iter);
		}
	}

	return weight;
	//return mNeighbours.size();

}

int MultiCHCluster::getNearestNodeToPos(const Coord& pos) {
	/**
	 * TODO: Add moving direction into consideration
	 */
	double minDistSqr = (std::numeric_limits<double>::max)();
	int minDistNodeId = -1;
	for(auto iter = mNeighbours.begin(); iter != mNeighbours.end();
				++iter) {
		Coord nodePos = iter->second.mPosition;
		double currDistSqr = distSqr(nodePos, pos);

		if (currDistSqr < minDistSqr) {
			minDistSqr = currDistSqr;
			minDistNodeId = iter->first;
		}
	}

	if (minDistNodeId == -1) {
		hdcEV << "getNearestNodeToPos(pos) failed with: neighbor node cnt: "
			<< mNeighbours.size() << endl;
	}

	return minDistNodeId;
}

int MultiCHCluster::getNearestNodeToPos(const unordered_map<int, unordered_set<int> >& neighbors,
			const Coord& pos) {

	/**
	 * TODO: Add moving direction into consideration
	 */
	double minDistSqr = (std::numeric_limits<double>::max)();
	int minDistNodeId = -1;
	for(auto iter = neighbors.begin(); iter != neighbors.end();
				++iter) {

		cModule* host = cSimulation::getActiveSimulation()->getModule(iter->first);
		if (!host) {
			continue;
		}

		Coord nodePos = getHostPosition(host);
		double currDistSqr = distSqr(nodePos, pos);

		if (currDistSqr < minDistSqr) {
			minDistSqr = currDistSqr;
			minDistNodeId = iter->first;
		}
	}

	return minDistNodeId;

}

int MultiCHCluster::headGateWayGetNextHopId(int dstId) {
	hdcEV << "headGateWay get next Hop id, cluster id: " << mClusterHead << endl;
	unordered_map<int, unordered_set<int> >* n = mClusterManager->getNeighborClusters(mClusterHead, simTime().dbl());

	if (!n) { 
		hdcEV << "headgateway get next hop id, current cluster is dead, return -1" << endl;
		return -1; 
	}

	Coord dstPos = getHostPosition(dstId);
	int nextClusterId = getNearestNodeToPos(*n, dstPos);

	/**
	 * If the neighbor cluster are all died
	 */
	if (nextClusterId == -1) { 
		hdcEV << "headgateway get next id failed with: all neighbor cluster"
			" is died or no neighbor clusters"
			<< endl;
		return -1;
	}

	/**
	 * if nearest cluster is not connected to this gateway
	 * use this node as head
	 */
	unordered_set<int>* pNeighborClusters = getNeighbourClusters();
	if (pNeighborClusters->find(nextClusterId) == pNeighborClusters->end()) {
		return headGetNextHopId(dstId);
	}

	/**
	 * if nearest cluster is connected to this gateway
	 * use this node as gateway
	 */
	for(auto iter = mNeighbours.begin(); iter != mNeighbours.end(); ++iter) {
		int cid = mClusterManager->getClusterIdByNodeId(iter->first);
		if (cid == nextClusterId) {
			return iter->first;
		}
	}

	/**
	 * use nearest neighbor node
	 */
	return getNearestNodeToPos(dstPos);
}

int MultiCHCluster::headGetNextHopId(int dstId) {
	hdcEV << "head get next Hop id, cluster id: " << mClusterHead << endl;
	unordered_map<int, unordered_set<int> >* n = mClusterManager->getNeighborClusters(mClusterHead, simTime().dbl());

	if (!n) { 
		hdcEV << "head get next id failed with: all neighbor cluster"
			" is died or no neighbor clusters"
			<< endl;
		return -1; 
	}

	/**
	 * Get the most near cluster
	 */
	Coord dstPos = getHostPosition(dstId);
	int nextClusterId = getNearestNodeToPos(*n, dstPos);
	//ASSERT(nextClusterId != -1);
	
	/**
	 * if all the neighbor cluster are all died
	 * or no neighbor cluster
	 */
	if (nextClusterId == -1) { 
		hdcEV << "head get next id failed with: we found neighbor cluster"
			" but the cluster lost head, died"
			<< endl;
		return -1; 
	}

	/**
	 * I'm in the most near cluster
	 */
	if (nextClusterId == mClusterHead) {
		if (mNeighbours.find(dstId) == mNeighbours.end()) {
			return getNearestNodeToPos(dstPos);
		} else {
			return dstId;
		}
	}

	auto iter = n->find(nextClusterId);
	ASSERT(iter != n->end());
	return *(iter->second.begin());
}

int MultiCHCluster::gateWayGetNextHopId(int dstId) {

	hdcEV << "gateway get next Hop id, cluster id: " << mClusterHead << endl;
	unordered_map<int, unordered_set<int> >* n = mClusterManager->getNeighborClusters(mClusterHead, simTime().dbl());

	if (!n) { 
		hdcEV << "gateway get next id failed with: all neighbor cluster"
			" is died or no neighbor clusters"
			<< endl;
		return -1; 
	}

	Coord dstPos = getHostPosition(dstId);
	int nextClusterId = getNearestNodeToPos(*n, dstPos);
	//ASSERT(nextClusterId != -1);
	
	if (nextClusterId == -1) { 
		hdcEV << "head get next id failed with: we found neighbor cluster"
			" but the cluster lost head, died"
			<< endl;
		return -1; 
	}

	if (nextClusterId == mClusterHead) {
		return getNearestNodeToPos(dstPos);
	}

	/**
	 * if nearest cluster is not connected to this gateway
	 * send to head
	 */
	unordered_set<int>* pNeighborClusters = getNeighbourClusters();
	if (pNeighborClusters->find(nextClusterId) == pNeighborClusters->end()) {
		if (mNeighbours.find(mClusterHead) == mNeighbours.end()) {
			return getNearestNodeToPos(dstPos);
		}
		if (mClusterHead == -1) {
			hdcEV << "cluster head is -1" << endl;
		}
		return mClusterHead;
	}
	
	for(auto iter = mNeighbours.begin(); iter != mNeighbours.end(); ++iter) {
		int cid = mClusterManager->getClusterIdByNodeId(iter->first);
		if (cid == nextClusterId) {
			return iter->first;
		}
	}

	return getNearestNodeToPos(dstPos);
}

int MultiCHCluster::memberGetNextHopId(int dstId) {
	hdcEV << "member get next Hop id, cluster id: " << mClusterHead << endl;
	if (mNeighbours.find(dstId) == mNeighbours.end()) {
		return dstId;
	}
	if (mClusterHead == -1) {
		hdcEV << "cluster head is -1" << endl;
	}
	return mClusterHead;
}

