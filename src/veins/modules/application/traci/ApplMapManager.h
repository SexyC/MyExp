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

#ifndef ApplMapManager_H
#define ApplMapManager_H

//#include <omnetpp.h>
//
//#include <MiXiMDefs.h>
//#include <BaseNetwLayer.h>

//#include <fstream>
#include <set>
#include <map>

#include <unordered_map>
#include <unordered_set>

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"

using std::unordered_map;
using std::unordered_set;
using std::set;
using std::cout;
using std::endl;
#define ammEV EV

/**
 * This module implements the clustering mechanism for
 * Modified Distributed Mobility-Aware Clustering (M-DMAC).
 * There is a pure virtual function that must be implemented by
 * inheriting cluster modules, which provides the weighting function
 * for the particular cluster algorithm being tested.
 */
class ApplMapManager
{

public:
	static ApplMapManager* getApplMapManager() {
		static ApplMapManager amm;
		return &amm;
	}

	BaseWaveApplLayer* getBaseWaveApplLayerById(int id) {
		auto iter = applInfo.find(id);
		return iter == applInfo.end() ? NULL : iter->second;
	}

	void registerAppl(int id, BaseWaveApplLayer* mod) {
		auto iter = applInfo.find(id);
		ASSERT(iter == applInfo.end());
		applInfo[id] = mod;
	}

	void unregisterAppl(int id) {
		auto iter = applInfo.find(id);
		ASSERT(iter != applInfo.end());
		applInfo.erase(id);
	}

protected:
	unordered_map<int, BaseWaveApplLayer*> applInfo;

private:
	ApplMapManager() {}
	ApplMapManager(const ApplMapManager&) = delete;
	const ApplMapManager operator=(const ApplMapManager&) = delete;
	~ApplMapManager() {}

};

#endif

