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

		const char *vstrConFac = par("futureConfidenceFactor").stringValue();
		futureConfidenceFactor = cStringTokenizer(vstrConFac).asDoubleVector();
		ASSERT(futureConfidenceFactor.size() == futureTimes.size());

		backupHeadMaxLimit = par("backupHeadMaxLimit");
		backupSelectFactor = par("backupSelectFactor");

		headFactorOfRemainCap = par("headFactorOfRemainCap");

		if (!(headFactorOfRemainCap >= 0 && headFactorOfRemainCap <= 1)) {
			error("headFactorOfRemainCap setting wrong");
		}

		gateWayFactorOfRemainCap = par("gateWayFactorOfRemainCap");
		if (!(gateWayFactorOfRemainCap >= 0 && gateWayFactorOfRemainCap <= 1)) {
			error("gateWayFactorOfRemainCap setting wrong");
		}

	} else if (stage == 1) { }
}

/** @brief Compute the CH weight for this node. */
double MultiCHCluster::calculateWeight() {
	NeighborNodeSet* nns = getCachedNeighborNodes();

	double weight = .0;

	vector<double> weightFactors(futureTimes.size());
	Coord myPos = mobility->getCurrentPosition();

	for (int wIdx = 0; wIdx < weightFactors.size(); ++wIdx) {
		for (auto iter = nns->begin(); iter != nns->end(); ++iter) {
			Coord nodePos = getHostPosition(*iter, simTime() + futureTimes[wIdx]);
			double disSq = myPos.sqrdist(nodePos);

			if (disSq > mTransmitRangeSq) {
				continue;
			}

			if (disSq < mTransmitRangeSq / 2) {
				weightFactors[wIdx] += 1;
			} else {
				weightFactors[wIdx] += (mTransmitRangeSq - disSq) / mTransmitRangeSq;
			}
		}
	}

	for (int wIdx = 0; wIdx < weightFactors.size(); ++wIdx) {
		weight += weightFactors[wIdx] * futureConfidenceFactor[wIdx];
	}

	return weight;
}

vector<int> MultiCHCluster::chooseBackupHeads() {
	/**
	 * no back up header needed
	 */
	ASSERT(IsClusterHead());
	if (!backupHeadMaxLimit) { return vector<int>(); }

	vector<int> result;
	/**
	 * min heap
	 */
	priority_queue<NodeWeight, vector<NodeWeight>, NodeWeightGreater> tmp;
	if (backupHeadMaxLimit < 0) {
		result.reserve(16);
		/**
		 * select from head neighbors
		 */
		for (auto iter = mNeighbours.begin();
					iter != mNeighbours.end();
					++iter) {
			if (iter->second.mWeight
						>= mWeight * backupSelectFactor) {
				result.push_back(iter->first);
			}
		}

		if (!result.empty()) {
			hdcEV << mId << " select " << result.size()
				<< " backup head node" << endl;
		}

		return result;
	}
	/**
	 * select from head neighbors
	 */
	for (auto iter = mNeighbours.begin();
				iter != mNeighbours.end();
				++iter) {
		if (iter->second.mWeight
					>= mWeight * backupSelectFactor) {
			/**
			 * limit size to backupHeadMaxLimit
			 */
			if (tmp.size() < backupHeadMaxLimit) {
				tmp.push(NodeWeight(iter->first, iter->second.mWeight));
			} else {
				NodeWeight nw = tmp.top();
				if (nw.weight < iter->second.mWeight) {
					tmp.pop();
					tmp.push(NodeWeight(iter->first, iter->second.mWeight));
				}
			}
		}
	}

	if (!tmp.empty()) {
		hdcEV << mId << " select " << tmp.size()
			<< " backup head node" << endl;
	}

	result.reserve(tmp.size());
	while(!tmp.empty()) {
		NodeWeight nw = tmp.top();
		result.push_back(nw.nodeId);
		tmp.pop();
	}

	return result;
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
		double currDistSqr = pos.sqrdist(nodePos);
		//double currDistSqr = distSqr(nodePos, pos);

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

int MultiCHCluster::getNearestNodeToPos(const unordered_map<int, unordered_map<int, int> >& neighbors,
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
		double currDistSqr = pos.sqrdist(nodePos);

		if (currDistSqr < minDistSqr) {
			minDistSqr = currDistSqr;
			minDistNodeId = iter->first;
		}
	}

	return minDistNodeId;

}

int MultiCHCluster::getBestHeadAsNextHop(int clusterId, int dstId) {
	unordered_set<int>* heads = mClusterManager->getClusterHeads(clusterId);
	if (!heads) { return -1; }
	return getBestHeadAsNextHop(*heads, dstId);
}

/**
 * clusterId -- current cluster the node is in
 * neighbors -- current neighbor node that the node is connected to
 * dstId -- packet dst node id
 */
int MultiCHCluster::getBestHeadAsNextHop(int clusterId, vector<int>& neighbors, int dstId) {
	unordered_set<int>* heads = mClusterManager->getClusterHeads(clusterId);
	if (!heads) { return -1; }

	unordered_set<int> neighborHead;

	for (auto iter = neighbors.begin(); iter != neighbors.end(); ++iter) {
		if (heads->find(*iter) != heads->end()) {
			neighborHead.insert(*iter);
		}
	}
	
	return getBestHeadAsNextHop(neighborHead, dstId);
}

int MultiCHCluster::getBestHeadAsNextHop(unordered_set<int>& heads, int dstId) {

	double maxWeight = 0;
	double maxRemainCap = 0;
	/**
	 * find out max weight and max remain cap for normalization
	 */
	unordered_map<int, double> weightFactors;
	weightFactors.reserve(heads.size());

	unordered_map<int, double> capacityFactor;
	capacityFactor.reserve(heads.size());

	for (auto iter = heads.begin(); iter != heads.end(); ++iter) {
		MultiCHCluster* mch = 
			dynamic_cast<MultiCHCluster*>(ApplMapManager::getApplMapManager()->getBaseWaveApplLayerById(*iter));
		ASSERT(mch);

		double nodeWeight = mch->getWeight();

		weightFactors[*iter] = nodeWeight;
		if (nodeWeight > maxWeight) {
			maxWeight = nodeWeight;
		}

		double nodeForwardBufferSize = mch->getForwardBufferSize();
		double nodeCurrentBufferOccupied = mch->getCurrentBufferOccupied();
		double nodeRemainCap = nodeForwardBufferSize - nodeCurrentBufferOccupied;

		capacityFactor[*iter] = nodeRemainCap;
		if (nodeRemainCap > maxRemainCap) {
			maxRemainCap = nodeRemainCap;
		}
	}

	if (maxWeight == 0 || maxRemainCap == 0) {
		cout << "maxWeight == 0 || maxRemainCap == 0" << endl;
		return -1;
	}

	/**
	 * normalize weight
	 */
	for (auto iter = weightFactors.begin(); iter != weightFactors.end(); ++iter) {
		iter->second /= maxWeight;
	}

	/**
	 * normalize remain cap
	 */
	double maxFactor = std::numeric_limits<double>::min();
	int nodeId = -1;
	for (auto iter = capacityFactor.begin(); iter != capacityFactor.end(); ++iter) {
		iter->second /= maxRemainCap;
		iter->second = weightFactors[iter->first] * (1 - headFactorOfRemainCap)
			+ iter->second * headFactorOfRemainCap;

		if (iter->second > maxFactor) {
			maxFactor = iter->second;
			nodeId = iter->first;
		}
	}

	cout << "cluster head: " << mClusterHead;
	cout << " selecting head: " << nodeId << " with factor: " << capacityFactor[nodeId] << endl;

	return nodeId;
}

int MultiCHCluster::getBestGateWayAsNextHop(unordered_map<int, int>& gateWays, int dstId) {

	unordered_map<int, double> capacityFactor;
	capacityFactor.reserve(gateWays.size());
	double maxRemainCap = 0;

	int maxDegree = 0;
	for (auto iter = gateWays.begin(); iter != gateWays.end(); ++iter) {
		if (iter->second > maxDegree) {
			maxDegree = iter->second;
		}
		MultiCHCluster* mch =
			dynamic_cast<MultiCHCluster*>(ApplMapManager::getApplMapManager()->getBaseWaveApplLayerById(iter->first));
		ASSERT(mch);
		
		double nodeForwardBufferSize = mch->getForwardBufferSize();
		double nodeCurrentBufferOccupied = mch->getCurrentBufferOccupied();
		double nodeRemainCap = nodeForwardBufferSize - nodeCurrentBufferOccupied;

		capacityFactor[iter->first] = nodeRemainCap;
		if (nodeRemainCap > maxRemainCap) {
			maxRemainCap = nodeRemainCap;
		}
	}

	if (maxRemainCap == 0 || maxDegree == 0) {
		cout << "maxRemainCap == 0 || maxDegree == 0" << endl;
		return -1;
	}

	/**
	 * normalize remainCap
	 */
	int gateWayId = -1;
	double maxGateWayFactor = std::numeric_limits<double>::min();
	for (auto iter = capacityFactor.begin(); iter != capacityFactor.end();
				++iter) {
		iter->second /= maxRemainCap;
		iter->second = ((double) gateWays[iter->first] / (double) maxDegree) * (1 - gateWayFactorOfRemainCap)
			+ iter->second * gateWayFactorOfRemainCap;

		if (iter->second > maxGateWayFactor) {
			maxGateWayFactor = iter->second;
			gateWayId = iter->first;
		}
	}
	cout << "cluster head: " << mClusterHead;
	cout << " selecting head: " << gateWayId << " with factor: " << capacityFactor[gateWayId] << endl;

	return gateWayId;
}

/**
 * This function works as following steps:
 * 0. If the dst is my neighbor, forward to it
 * 1. Find nearest cluster, if I'm connected to it, then work as gateway
 * 2. If I'm not, work as head
 */
int MultiCHCluster::headGateWayGetNextHopId(int dstId) {
	hdcEV << "headgateway: headGateWay get next Hop id, cluster id: " << mClusterHead << endl;

	if (mNeighbours.find(dstId) == mNeighbours.end()) {
		return dstId;
	}

	unordered_map<int, unordered_map<int, int> >* n = mClusterManager->getNeighborClusters(mClusterHead, simTime().dbl());

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
	unordered_map<int, int>* pNeighborClusters = getNeighbourClusters();
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

/**
 * This function follows the next steps:
 * 0. If the dst is my neighbor, forward to it
 * 1. find best forward cluster
 * 2. if step 1 failed, return -1
 * 3. If step 1 return my cluster is nearest, forward to closest node
 * 4. Find a best gateway to forward to the neighbor cluster
 */
int MultiCHCluster::headGetNextHopId(int dstId) {
	hdcEV << "head get next Hop id, cluster id: " << mClusterHead << endl;

	if (mNeighbours.find(dstId) == mNeighbours.end()) {
		return dstId;
	}
	unordered_map<int, unordered_map<int, int> >* n = mClusterManager->getNeighborClusters(mClusterHead, simTime().dbl());

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
			/**
			 * FIXME: maybe we should return -1
			 */
			return getNearestNodeToPos(dstPos);
		} else {
			return dstId;
		}
	}

	auto iter = n->find(nextClusterId);
	ASSERT(iter != n->end());

	int selectedGateWayId = getBestGateWayAsNextHop(iter->second, dstId);
	hdcEV << mClusterHead << "-->" << iter->first << " ";
	hdcEV << "selecting best gateway as next hop: " << selectedGateWayId;
	hdcEV << " while gateway size: " << iter->second.size() << endl;
	//return (iter->second.begin()->first);
	return selectedGateWayId;
}

/**
 * This function work as following steps:
 * 1. Find the clusters that connected to the gateway this back head connected to
 * 2. Find the best cluster to forward in the clusters found in step 1
 * 3. If we can't find best cluster in step 2, find if we can find best cluster using real head
 * 4. If step 3 return yes, then forward to real head, else return -1
 * 5. If step 2 return a valid best cluster id, then find best gateway to forward it
 */
int MultiCHCluster::backHeadGetNextHopId(int dstId) {

	cout << "backHead is working" << endl;

	vector<int> myNeighborNodes;
	myNeighborNodes.reserve(mNeighbours.size());

	for (auto iter = mNeighbours.begin(); iter != mNeighbours.end(); ++iter) {
		myNeighborNodes.push_back(iter->first);
	}

	unordered_map<int, unordered_map<int, int> > myNeighborCluster
		= mClusterManager->getNeighborClusters(myNeighborNodes);

	Coord dstPos = getHostPosition(dstId);
	int nextClusterId = getNearestNodeToPos(myNeighborCluster, dstPos);

	if (nextClusterId == -1) {
		hdcEV << "back head get next id failed with: we found neighbor cluster"
			" but the cluster lost head, died"
			<< endl;

		unordered_map<int, unordered_map<int, int> >* n = mClusterManager->getNeighborClusters(mClusterHead, simTime().dbl());
		
		if (!n) { 
			hdcEV << "back head get next id failed with: all cluster neighbor cluster"
				" is died or no neighbor clusters"
				<< endl;
			return -1; 
		}
		nextClusterId = getNearestNodeToPos(*n, dstPos);

		if (nextClusterId == -1) {
			hdcEV << "back head get next id failed with: we found neighbor cluster"
				" but the cluster lost head, died"
				<< endl;
			return -1;
		}
		/**
		 * if back head can not find gateway but cluster head can
		 * forward to cluster head
		 */
		return mClusterHead;
	}

	auto iter = myNeighborCluster.find(nextClusterId);
	ASSERT(iter != myNeighborCluster.end());

	int selectedGateWayId = getBestGateWayAsNextHop(iter->second, dstId);
	hdcEV << mClusterHead << "-->" << iter->first << " ";
	hdcEV << "selecting best gateway as next hop: " << selectedGateWayId;
	hdcEV << " while gateway size: " << iter->second.size() << endl;

	return selectedGateWayId;
}

/**
 * This one follow the next steps:
 * 0. check if dst is my neighbor, if yes, forward to it(work as any fucking node)
 * 1. check if the best forward cluster is connected to me, if yes, forward to it(work as gateway)
 * 2. check if I'm the back up head, if yes, work as back up head
 * 3. check if I'm connected to some head(real head or back up head), if yes, forward to it(work as member)
 * 4. find the nearest node to dst node
 */
int MultiCHCluster::gateWayGetNextHopId(int dstId) {

	hdcEV << "gateway get next Hop id, cluster id: " << mClusterHead << endl;

	if (mNeighbours.find(dstId) == mNeighbours.end()) {
		return dstId;
	}
	unordered_map<int, unordered_map<int, int> >* n = mClusterManager->getNeighborClusters(mClusterHead, simTime().dbl());

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
		hdcEV << "gateway get next id failed with: we found neighbor cluster"
			" but the cluster lost head, died"
			<< endl;
		return -1; 
	}

	/**
	 * FIXME: Should forward to head
	 */
	if (nextClusterId == mClusterHead) {
		return getNearestNodeToPos(dstPos);
	}

	/**
	 * if nearest cluster is not connected to this gateway
	 * send to head
	 */
	unordered_map<int, int>* pNeighborClusters = getNeighbourClusters();
	if (pNeighborClusters->find(nextClusterId) == pNeighborClusters->end()) {

		/**
		 * If I'm the back head
		 */
		if (mClusterManager->isBackHead(mClusterHead, mId)) {
			return backHeadGetNextHopId(dstId);
		}

		vector<int> myNeighborNodeIds;
		myNeighborNodeIds.reserve(mNeighbours.size());

		for (auto iter = mNeighbours.begin(); iter != mNeighbours.end(); ++iter) {
			myNeighborNodeIds.push_back(iter->first);
		}

		int headId = getBestHeadAsNextHop(mClusterHead, myNeighborNodeIds, dstId);

		if (headId == -1) {
			hdcEV << "can't get forward head or back head" << endl;
			return getNearestNodeToPos(dstPos);
		}

		return headId;

		//if (mNeighbours.find(mClusterHead) == mNeighbours.end()) {
		//	return getNearestNodeToPos(dstPos);
		//}
		//if (mClusterHead == -1) {
		//	hdcEV << "cluster head is -1" << endl;
		//}
		//return mClusterHead;
	}
	
	for(auto iter = mNeighbours.begin(); iter != mNeighbours.end(); ++iter) {
		int cid = mClusterManager->getClusterIdByNodeId(iter->first);
		if (cid == nextClusterId) {
			return iter->first;
		}
	}

	return getNearestNodeToPos(dstPos);
}

/**
 * This function work as the following steps:
 * 0. check if dst is my neighbor, if yes, forward to it(work as any fucking node)
 * 1. check if I'm the back up head, if yes, work as back up head
 * 2. get neighbor best head to forward(work as a member node)
 */
int MultiCHCluster::memberGetNextHopId(int dstId) {
	hdcEV << "member get next Hop id, cluster id: " << mClusterHead << endl;
	if (mNeighbours.find(dstId) == mNeighbours.end()) {
		return dstId;
	}

	if (mClusterManager->isBackHead(mClusterHead, mId)) {
		return backHeadGetNextHopId(dstId);
	}

	vector<int> myNeighborNodeIds;
	myNeighborNodeIds.reserve(mNeighbours.size());

	for (auto iter = mNeighbours.begin(); iter != mNeighbours.end(); ++iter) {
		myNeighborNodeIds.push_back(iter->first);
	}

	/**
	 * Find neighbor heads
	 */
	int headId = getBestHeadAsNextHop(mClusterHead, myNeighborNodeIds, dstId);

	if (headId == -1) {
		hdcEV << "member id: " << mId <<
			" can't get correct head for forwarding" << endl;
	}
	return headId;
}

void MultiCHCluster::receiveHelloMessage(MdmacControlMessage* m) {

	updateNeighbour(m);

	if ( testClusterHeadChange( m->getNodeId() ) ) {
		cModule* nodeHost = cSimulation::getActiveSimulation()->getModule(m->getNodeId());
		/**
		 * If this node is finished
		 * do nothing
		 */
		if (!nodeHost) { return; }

		/**
		 * check next time(+1s) will they be connected
		 * if not, do not add
		 */
		Coord nodeNextPos = getHostPosition(nodeHost, simTime() + 1);
		Coord myNextPos = mobility->getPositionAt(simTime() + 1);

		double distanceSqr = myNextPos.sqrdist(nodeNextPos);
		if (distanceSqr > mTransmitRangeSq) {
			hdcEV << "recv hello, but ";
			hdcEV << distanceSqr << " > " << mTransmitRangeSq << " ";
			hdcEV << "next time, " << mId << " " << m->getNodeId()
				<< "do not connect, do nothing" << endl;
			return;
		}

		// If this was a CH, the cluster is dead, so log lifetime
		if ( IsClusterHead() ) {
			ClusterDied( CD_Cannibal );
		}


        emit( mSigHeadChange, 1 );
		mIsClusterHead = false;
		mClusterMembers.clear();

		/**
		 * if this node belong to other cluster before
		 * tell cluster manager to remove it from the original cluster
		 */
		if (mClusterHead != -1) {
			mClusterManager->leaveCluster(mClusterHead, mId, simTime().dbl());
		}

		mClusterHead = m->getNodeId();

		mClusterManager->joinCluster(mClusterHead, mId, simTime().dbl());
		getNeighbourClusters(false);
		sendClusterMessage( JOIN_MESSAGE, m->getNodeId() );
		//ClusterAnalysisScenarioManagerAccess::get()->joinMessageSent( mId, m->getNodeId() );

	}

	if ( m->getTtl() > 1 )
		sendClusterMessage( HELLO_MESSAGE, -1, m->getTtl()-1 );

	//delete m;

}

void MultiCHCluster::receiveChMessage(MdmacControlMessage* m) {

	updateNeighbour(m);

	if ( testClusterHeadChange( m->getNodeId() ) ) {

		cModule* nodeHost = cSimulation::getActiveSimulation()->getModule(m->getNodeId());
		/**
		 * If this node is finished
		 * do nothing
		 */
		if (!nodeHost) { return; }

		/**
		 * check next time(+1s) will they be connected
		 * if not, do not add
		 */
		Coord nodeNextPos = getHostPosition(nodeHost, simTime() + 1);
		Coord myNextPos = mobility->getPositionAt(simTime() + 1);

		double distanceSqr = myNextPos.sqrdist(nodeNextPos);
		if (distanceSqr > mTransmitRangeSq) {
			hdcEV << "recv CH msg, but ";
			hdcEV << distanceSqr << " > " << mTransmitRangeSq << " ";
			hdcEV << "next time, " << mId << " " << m->getNodeId()
				<< "do not connect, do nothing" << endl;
			return;
		}

		// If this was a CH, the cluster has been cannibalised, so log lifetime
		if ( IsClusterHead() ) {
			ClusterDied( CD_Cannibal );
		}

        emit( mSigHeadChange, 1 );
		mIsClusterHead = false;
		mClusterMembers.clear();
		/**
		 * if this node belong to other cluster before
		 * tell cluster manager to remove it from the original cluster
		 */
		if (mClusterHead != -1) {
			mClusterManager->leaveCluster(mClusterHead, mId, simTime().dbl());
		}
		mClusterHead = m->getNodeId();

		mClusterManager->joinCluster(mClusterHead, mId, simTime().dbl());
		sendClusterMessage( JOIN_MESSAGE, m->getNodeId() );
		//ClusterAnalysisScenarioManagerAccess::get()->joinMessageSent( mId, m->getNodeId() );

	} else {

		if ( IsClusterHead() ) {

			bool sizeChanged = false;

			/**
			 * little brother become big brother
			 */
			if ( mClusterMembers.find( m->getNodeId() ) != mClusterMembers.end() ) {

				mClusterMembers.erase( m->getNodeId() );
				sizeChanged = true;

			}

			if ( sizeChanged )
				mCurrentMaximumClusterSize = std::max( mCurrentMaximumClusterSize, (int)mClusterMembers.size() );

		}

	}

	if ( m->getTtl() > 1 )
		sendClusterMessage( CH_MESSAGE, -1, m->getTtl()-1 );

	//delete m;

}

void MultiCHCluster::receiveJoinMessage(MdmacControlMessage* m) {
	HighestDegreeCluster::receiveJoinMessage(m);
}

void MultiCHCluster::processBeat() {
	HighestDegreeCluster::processBeat();
	if (IsClusterHead()) {
		vector<int> backupHeads = chooseBackupHeads();
		mClusterManager->registerBackHead(mClusterHead, backupHeads);
	}
}

