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

#ifndef NodeData_H
#define NodeData_H

#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>

using std::unordered_map;
using std::unordered_set;
using std::vector;
using std::queue;

/**
 * Node Stored Data structure
 */
class HeadData {
	public:
		int degree;
		unordered_set<int> memberIds;
		unordered_set<int> gateWayIds;
		// cluster id -- [gateway id1, id2, ...]
		unordered_map<int, unordered_set<int> > gateWayInfo;
};

class GateWayData {
	public:
		// cluster id -- connection degree
		unordered_map<int, int> connectedClusters;
};

class MemberData {
	public:
		// Major heads and back up heads
		vector<int> headIds;
};

union NodeData {
	HeadData* hd;
	GateWayData* gwd;
	MemberData* md;
};

#endif
