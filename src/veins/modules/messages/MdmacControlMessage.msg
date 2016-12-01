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

cplusplus {{
//#include "NetwPkt_m.h"
#include "WaveShortMessage_m.h"

//typedef std::list<std::string> StringList;
typedef std::list<int> MdmacPathQueue;
}}
//packet NetwPkt;

//class noncobject StringList;
class noncobject MdmacPathQueue;
class WaveShortMessage;

//
// Describes the M-DMAC cluster control message.
// Extends the NetwPkt, so this can be used to encapsulate
// application layer packets as well.
//
//packet MdmacControlMessage extends NetwPkt {
packet MdmacControlMessage extends WaveShortMessage {

	int nodeId;
	double weight;
	bool isClusterHead;
	double xPosition;
	double yPosition;
	double xVelocity;
	double yVelocity;
	string roadId;
	string laneId;
	double xDestination;
	double yDestination;
	//StringList route;
	MdmacPathQueue pathNodes; // record packet path
	int targetNodeId;	// ID of the CH I wish to join (used for JOIN_MESSAGE type)
	int ttl;

}
