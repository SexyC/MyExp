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

package org.car2x.veins.modules.application.traci;
import org.car2x.veins.modules.application.ieee80211p.BaseWaveApplLayer;

//
// Implements the multiple ch algorithm
//
simple MultiCHCluster extends HighestDegreeCluster
{
    parameters:
        @class(MultiCHCluster);
		string futureTimes = default("1 2 3");
		string futureConfidenceFactor = default("0.5 0.3 0.2");
		// setting this to 0 means need back up head needed
		// setting this to negative number means unlimited
		int backupHeadMaxLimit = default(6);
		// min(head_back_weight / head_weight)
		int backupSelectFactor = default(.8);

		// (1-headFactorOfRemainCap) * weight + headFactorOfRemainCap * remainCap
		double headFactorOfRemainCap = default(.6);
		double gateWayFactorOfRemainCap = default(.6);
}
