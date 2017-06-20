#include "gpsr-ptable.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include <algorithm>
#include <cfloat>

NS_LOG_COMPONENT_DEFINE ("GpsrTable");

namespace ns3 {
namespace gpsr {

/*
 GPSR position table
 */

PositionTable::PositionTable() {
	m_txErrorCallback = MakeCallback(&PositionTable::ProcessTxError, this);
	m_entryLifeTime = Seconds(2); //FIXME fazer isto parametrizavel de acordo com tempo de hello

}

Time PositionTable::GetEntryUpdateTime(Ipv4Address id) {
	if (id == Ipv4Address::GetZero()) {
		return Time(Seconds(0));
	}
	std::map<Ipv4Address, std::pair<Vector, Time> >::iterator i = m_table.find(
			id);
	return i->second.second;
}

/**
 * \brief Adds entry in position table
 */
void PositionTable::AddEntry(Ipv4Address id, Vector position) {
	std::map<Ipv4Address, std::pair<Vector, Time> >::iterator i = m_table.find(
			id);
	if (i != m_table.end() || id.IsEqual(i->first)) {
		m_table.erase(id);
		m_table.insert(
				std::make_pair(id, std::make_pair(position, Simulator::Now())));
	} else {
		m_table.insert(
				std::make_pair(id, std::make_pair(position, Simulator::Now())));
	}
}

/**
 * \brief Deletes entry in position table and from planarized neighbors
 */
void PositionTable::DeleteEntry(Ipv4Address id) {
	m_table.erase(id);
	//m_planarized_neighbors.erase(id);
}

/**
 * \brief Gets position from position table
 * \param id Ipv4Address to get position from
 * \return Position of that id or NULL if not known
 */
Vector PositionTable::GetPosition(Ipv4Address id) {

	NodeList::Iterator listEnd = NodeList::End();
	for (NodeList::Iterator i = NodeList::Begin(); i != listEnd; i++) {
		Ptr<Node> node = *i;
		if (node->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal() == id) {
			return node->GetObject<MobilityModel>()->GetPosition();
		}
	}
	return PositionTable::GetInvalidPosition();

}

/**
 * \brief Checks if a node is a neighbour
 * \param id Ipv4Address of the node to check
 * \return True if the node is neighbour, false otherwise
 */
bool PositionTable::isNeighbour(Ipv4Address id) {

	std::map<Ipv4Address, std::pair<Vector, Time> >::iterator i = m_table.find(
			id);
	if (i != m_table.end() || id.IsEqual(i->first)) {
		return true;
	}

	return false;
}

/**
 * \brief remove entries with expired lifetime
 */
void PositionTable::Purge() {

	if (m_table.empty()) {
		return;
	}

	std::list<Ipv4Address> toErase;

	std::map<Ipv4Address, std::pair<Vector, Time> >::iterator i =
			m_table.begin();
	std::map<Ipv4Address, std::pair<Vector, Time> >::iterator listEnd =
			m_table.end();

	for (; !(i == listEnd); i++) {

		if (m_entryLifeTime + GetEntryUpdateTime(i->first)
				<= Simulator::Now()) {
			toErase.insert(toErase.begin(), i->first);

		}
	}
	toErase.unique();

	std::list<Ipv4Address>::iterator end = toErase.end();

	for (std::list<Ipv4Address>::iterator it = toErase.begin(); it != end;
			++it) {

		m_table.erase(*it);
//		m_planarized_neighbors.erase(*it);
	}
}

/**
 * \brief clears all entries
 */
void PositionTable::Clear() {
	m_table.clear();
	m_planarized_neighbors.clear();
}

void PositionTable::PrintNeighbors(std::ostream &os) {
	Purge();
	os << "Neighbors: ";
	if (m_table.empty()) {
		os << "Neighbor table is empty!";
		return;
	}     //if table is empty (no neighbours)

	std::map<Ipv4Address, std::pair<Vector, Time> >::iterator i;
	for (i = m_table.begin(); !(i == m_table.end()); i++) {
		Ipv4Address ip = i->first;
		bool is_planarized = m_planarized_neighbors.find(ip)
				!= m_planarized_neighbors.end();
		Vector pos = i->second.first;
		os << "ip=" << ip << " [" << pos.x << "," << pos.y << ","
				<< is_planarized << "] ";
	}
}

/**
 * \brief Gets next hop according to GPSR protocol
 * \param position the position of the destination node
 * \param nodePos the position of the node that has the packet
 * \return Ipv4Address of the next hop, Ipv4Address::GetZero () if no nighbour was found in greedy mode
 */
Ipv4Address PositionTable::BestNeighbor(Vector position, Vector nodePos) {
	Purge();

	double initialDistance = CalculateDistance(nodePos, position);

	if (m_table.empty()) {
		NS_LOG_DEBUG("BestNeighbor table is empty; Position: " << position);
		return Ipv4Address::GetZero();
	}     //if table is empty (no neighbours)

	Ipv4Address bestFoundID = m_table.begin()->first;
	double bestFoundDistance = CalculateDistance(m_table.begin()->second.first,
			position);
	std::map<Ipv4Address, std::pair<Vector, Time> >::iterator i;
	for (i = m_table.begin(); !(i == m_table.end()); i++) {
		if (bestFoundDistance > CalculateDistance(i->second.first, position)) {
			bestFoundID = i->first;
			bestFoundDistance = CalculateDistance(i->second.first, position);
		}
	}

	if (initialDistance > bestFoundDistance)
		return bestFoundID;
	else
		return Ipv4Address::GetZero(); //so it enters Recovery-mode

}

/**
 * \brief Gets next hop according to Electrostatics based Greedy Forwarding
 * \param position the position of the destination node
 * \param nodePos the position of the node that has the packet
 * \return Ipv4Address of the next hop, Ipv4Address::GetZero () if no nighbour was found in Repulsion mode
 */
Ipv4Address PositionTable::ElectrostaticBestNeighbor(Vector position,
		Vector nodePos) {
	Purge();

	//FIXME allow parametrization of this parameters
	double q = 1;
	double n = 2;
	//FIXME add hole detection mechanism
	Vector holeC(200,325,0);//(375, 325, 0); // be aware these values are hardcoded for particular experiment
	double holeR = 350;//388.909; // be aware these values are hardcoded for particular experiment

	//calculate hole charge ql
	double b = CalculateDistance(holeC, position);
	double ql = (q * std::pow(holeR, n + 1)) / (n * std::pow(b + holeR, 2));

	double initPotential = -q / CalculateDistance(nodePos, position)
			+ ql / (std::pow(CalculateDistance(nodePos, holeC), n));

	if (m_table.empty()) {
		NS_LOG_DEBUG("BestNeighbor table is empty; Position: " << position);
		return Ipv4Address::GetZero();
	}     //if table is empty (no neighbours)

	Ipv4Address bestFoundID = Ipv4Address::GetZero();
	//m_table.begin()->first;
	double minPotential = DBL_MAX;
	//-q / CalculateDistance(m_table.begin()->second.first, position) + ql / (std::pow(CalculateDistance(m_table.begin()->second.first, holeC), n));
	std::map<Ipv4Address, std::pair<Vector, Time> >::iterator i;
	for (i = m_table.begin(); !(i == m_table.end()); i++) {
		double tmpPotential = -q / CalculateDistance(i->second.first, position)
				+ ql / (std::pow(CalculateDistance(i->second.first, holeC), n));
		if (minPotential > tmpPotential) {
			bestFoundID = i->first;
			minPotential = tmpPotential;
		}
	}

	if (initPotential > minPotential)
		return bestFoundID;
	else
		return Ipv4Address::GetZero(); //so it enters Recovery-mode
}

/**
 * \brief Gets next hop according to GPSR recovery-mode protocol (right hand rule)
 * \param previousHop the position of the node that sent the packet to this node
 * \param nodePos the position of the destination node
 * \return Ipv4Address of the next hop, Ipv4Address::GetZero () if no nighbour was found in greedy mode
 */
Ipv4Address PositionTable::BestAngle(Vector previousHop, Vector nodePos) {
	Purge();
	PlanarizeNeighbors(nodePos);

	if (m_table.empty()) {
		NS_LOG_DEBUG("BestNeighbor table is empty; Position: " << nodePos);
		return Ipv4Address::GetZero();
	}     //if table is empty (no neighbours)

	double tmpAngle;
	Ipv4Address bestFoundID = Ipv4Address::GetZero();
	double bestFoundAngle = 360;
	std::map<Ipv4Address, std::pair<Vector, Time> >::iterator i;

	for (i = m_table.begin(); !(i == m_table.end()); i++) {
		if (m_planarized_neighbors.find(i->first)
				== m_planarized_neighbors.end()) {
			tmpAngle = GetAngle(nodePos, previousHop, i->second.first);
			if (bestFoundAngle > tmpAngle && tmpAngle != 0) {
				bestFoundID = i->first;
				bestFoundAngle = tmpAngle;
			}
		}
	}

	if (bestFoundID == Ipv4Address::GetZero())
	{
		bestFoundID = m_table.begin()->first;
	}

	return bestFoundID;
}

//Gives angle between the vector CentrePos-Refpos to the vector CentrePos-node counterclockwise
double PositionTable::GetAngle(Vector centrePos, Vector refPos, Vector node) {
	double const PI = 4 * atan(1);

	std::complex<double> A = std::complex<double>(centrePos.x, centrePos.y);
	std::complex<double> B = std::complex<double>(node.x, node.y);
	std::complex<double> C = std::complex<double>(refPos.x, refPos.y); //Change B with C if you want angles clockwise

	std::complex<double> AB; //reference edge
	std::complex<double> AC;
	std::complex<double> tmp;
	std::complex<double> tmpCplx;

	std::complex<double> Angle;

	AB = B - A;
	AB = (real(AB) / norm(AB))
			+ (std::complex<double>(0.0, 1.0) * (imag(AB) / norm(AB)));

	AC = C - A;
	AC = (real(AC) / norm(AC))
			+ (std::complex<double>(0.0, 1.0) * (imag(AC) / norm(AC)));

	tmp = log(AC / AB);
	tmpCplx = std::complex<double>(0.0, -1.0);
	Angle = tmp * tmpCplx;
	Angle *= (180 / PI);
	if (real(Angle) < 0)
		Angle = 360 + real(Angle);

	return real(Angle);

}

/**
 * \ProcessTxError
 */
void PositionTable::ProcessTxError(WifiMacHeader const & hdr) {
}

//FIXME ainda preciso disto agr que o LS ja n está aqui???????

/**
 * \brief Returns true if is in search for destionation
 */
bool PositionTable::IsInSearch(Ipv4Address id) {
	return false;
}

bool PositionTable::HasPosition(Ipv4Address id) {
	return true;
}

void PositionTable::PlanarizeNeighbors(Vector nodePos) {
	//check planarization
	/*
	 Sensor u = s;
	 for (Sensor v : u.getNeighbors())
	 for (Sensor w : u.getNeighbors())
	 if (w.equals(v))
	 continue;
	 else if (EuclDist.d(u, v) > Math.max(EuclDist.d(u, w), EuclDist.d(v, w)))
	 {
	 u.addProhibitedNeighbor(v);
	 break;
	 }*/

	m_planarized_neighbors.clear();
	Vector u = nodePos;
	std::map<Ipv4Address, std::pair<Vector, Time> >::iterator i;
	std::map<Ipv4Address, std::pair<Vector, Time> >::iterator j;
	for (i = m_table.begin(); !(i == m_table.end()); i++) {
		Vector v = i->second.first;
		for (j = m_table.begin(); !(j == m_table.end()); j++) {
			Vector w = j->second.first;
			if (i->first.IsEqual(j->first)) {
				continue;
			} else if (CalculateDistance(u, v)
					> std::max(CalculateDistance(u, w),
							CalculateDistance(v, w))) {
				m_planarized_neighbors.insert(i->first);
				break;
			}
		}
	}
}

}   // gpsr
} // ns3
