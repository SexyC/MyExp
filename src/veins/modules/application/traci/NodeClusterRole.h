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

#ifndef NodeClusterRole_H
#define NodeClusterRole_H

#include "veins/modules/application/traci/NodeData.h"
#include "veins/modules/application/traci/NodeStrategy.h"
//#include "veins/modules/application/traci/SingleNodeStrategy.h"
class SingleNodeStrategy;
class HeadNodeStrategy;
class GateWayNodeStrategy;
class MemberNodeStrategy;

class NodeClusterRole {
	public:
		enum NodeRole {
			SINGLE,
			HEAD,
			HEAD_BAK,
			GATEWAY,
			GATEWAY_BAK,
			MEMBER
		};
		enum ClusterBeaconType {
			HELLO,
			HELLO_REPLY,
			CLUSTER_STATUS,
			JOIN_REQUEST,
			JOIN_RESPONSE
		};
		enum ResponseType {
			JOIN_ACC,
			JOIN_REJ,
		};
	public:
		void init() {
			clusterRole = SINGLE;
			clusterNodeData.hd = NULL;
			st = (NodeStrategy*)&singleNodeSt;
		}
		NodeRole& getClusterRole() {
			return clusterRole;
		}
		void setClusterRole(NodeRole nr) {
			switch (nr) {
				case SINGLE:
					st = (NodeStrategy*)&singleNodeSt;
					break;
				case HEAD:
				case HEAD_BAK: /* Fall through */
					st = (NodeStrategy*)&headNodeSt;
					break;
				case GATEWAY:
				case GATEWAY_BAK: /* Fall through */
					st = (NodeStrategy*)&gateWayNodeSt;
					break;
				case MEMBER:
					st = (NodeStrategy*)&memberNodeSt;
					break;
				default:
					ASSERT(false);
					break;
			}
			clusterRole = nr;
		}
		NodeData& getClusterNodeData() {
			return clusterNodeData;
		}
		void setClusterNodeData(NodeData& nd) {
			clusterNodeData = nd;
		}
		const NodeStrategy* const getNodeStrategy() {
			return st;
		}
	private:
		static SingleNodeStrategy singleNodeSt; 
		static HeadNodeStrategy headNodeSt;
		static GateWayNodeStrategy gateWayNodeSt;
		static MemberNodeStrategy memberNodeSt;
		NodeRole clusterRole;
		NodeData clusterNodeData;
		NodeStrategy* st;

};

#endif

