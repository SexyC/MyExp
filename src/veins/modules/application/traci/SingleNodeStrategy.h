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

#ifndef SingleNodeStrategy_H
#define SingleNodeStrategy_H

#include "veins/modules/application/traci/NodeStrategy.h"
#include "veins/modules/messages/WaveShortMessageClusterBeacon_m.h"
#include "veins/modules/application/traci/TraCICluster.h"

class SingleNodeStrategy : public NodeStrategy {
	public:
		virtual void sendClusterHello(TraCICluster* tc) const {
			WaveShortMessageClusterBeacon* wsmcb = tc->prepareWSMWithDst(HELLO, tc->headerLength);
		}
		virtual void sendClusterJoinRequest(TraCICluster*) const {
		}
		virtual void sendClusterJoinResponse(TraCICluster*) const {
		}
		virtual void sendClusterStatus(TraCICluster*) const {
		}
};

#endif


