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


//#include "BaseNetwLayer.h"

#include <cassert>

//#include "NetwControlInfo.h"
//#include "BaseMacLayer.h"
//#include "AddressingInterface.h"
//#include "SimpleAddress.h"
//#include "FindModule.h"
//#include "MdmacControlMessage_m.h"
//#include "ArpInterface.h"
//#include "NetwToMacControlInfo.h"
//#include "BaseMobility.h"
//#include "BaseConnectionManager.h"
//#include "ChannelAccess.h"

#include "HighestDegreeCluster.h"
#define hdcEV  EV

Define_Module(HighestDegreeCluster);


/** @brief Compute the CH weight for this node. */
double HighestDegreeCluster::calculateWeight() {

	return mNeighbours.size();

}

int HighestDegreeCluster::getNearestNodeToPos(const Coord& pos) {
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

	return minDistNodeId;
}

int HighestDegreeCluster::getNearestNodeToPos(const unordered_map<int, unordered_set<int> >& neighbors,
			const Coord& pos) {

	double minDistSqr = (std::numeric_limits<double>::max)();
	int minDistNodeId = -1;
	for(auto iter = neighbors.begin(); iter != neighbors.end();
				++iter) {
		Coord nodePos = getHostPosition(iter->first);
		double currDistSqr = distSqr(nodePos, pos);

		if (currDistSqr < minDistSqr) {
			minDistSqr = currDistSqr;
			minDistNodeId = iter->first;
		}
	}

	return minDistNodeId;

}

int HighestDegreeCluster::getNextHopId(int dstId) {

	int clusterId = mClusterManager->getClusterIdByNodeId(dstId);

	/**
	 * Single node
	 */
	if (clusterId == -1) {
		return getNearestNodeToPos(getHostPosition(dstId));
	}

	int state = GetClusterState();
	switch (state) {
		case SINGLE:
			ASSERT(false);
			return getNearestNodeToPos(getHostPosition(dstId));
			break;
		case HEAD | GATEWAY:
			headGateWayGetNextHopId(dstId);
			break;
		case HEAD:
			headGetNextHopId(dstId);
			break;
		case GATEWAY:
			gateWayGetNextHopId(dstId);
			break;
		case MEMBER:
			memberGetNextHopId(dstId);
			break;
		default:
			ASSERT(false);
			break;
	}

	return -1;
}

int HighestDegreeCluster::headGateWayGetNextHopId(int dstId) {
	hdcEV << "headGateWay get next Hop id, cluster id: " << mClusterHead << endl;
	unordered_map<int, unordered_set<int> >* n = mClusterManager->getNeighborClusters(mClusterHead, simTime().dbl());

	if (!n) { return -1; }

	Coord dstPos = getHostPosition(dstId);
	int nextClusterId = getNearestNodeToPos(*n, dstPos);

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

	return getNearestNodeToPos(*n, dstPos);
}

int HighestDegreeCluster::headGetNextHopId(int dstId) {
	hdcEV << "head get next Hop id, cluster id: " << mClusterHead << endl;
	unordered_map<int, unordered_set<int> >* n = mClusterManager->getNeighborClusters(mClusterHead, simTime().dbl());

	if (!n) { return -1; }

	/**
	 * Get the most near cluster
	 */
	Coord dstPos = getHostPosition(dstId);
	int nextClusterId = getNearestNodeToPos(*n, dstPos);
	ASSERT(nextClusterId != -1);

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

int HighestDegreeCluster::gateWayGetNextHopId(int dstId) {

	hdcEV << "gateway get next Hop id, cluster id: " << mClusterHead << endl;
	unordered_map<int, unordered_set<int> >* n = mClusterManager->getNeighborClusters(mClusterHead, simTime().dbl());

	if (!n) { return -1; }

	Coord dstPos = getHostPosition(dstId);
	int nextClusterId = getNearestNodeToPos(*n, dstPos);
	ASSERT(nextClusterId != -1);

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

int HighestDegreeCluster::memberGetNextHopId(int dstId) {
	hdcEV << "member get next Hop id, cluster id: " << mClusterHead << endl;
	if (mNeighbours.find(dstId) == mNeighbours.end()) {
		return dstId;
	}
	return mClusterHead;
}

