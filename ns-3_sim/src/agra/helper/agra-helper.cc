/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/****************************************************************************/
/* This file is part of AGRA project.                                       */
/*                                                                          */
/* AGRA is free software: you can redistribute it and/or modify             */
/* it under the terms of the GNU General Public License as published by     */
/* the Free Software Foundation, either version 3 of the License, or        */
/* (at your option) any later version.                                      */
/*                                                                          */
/* AGRA is distributed in the hope that it will be useful,                  */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of           */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            */
/* GNU General Public License for more details.                             */
/*                                                                          */
/* You should have received a copy of the GNU General Public License        */
/* along with AGRA.  If not, see <http://www.gnu.org/licenses/>.            */
/*                                                                          */
/****************************************************************************/
/*                                                                          */
/*  Author:    Dmitrii Chemodanov, University of Missouri-Columbia          */
/*  Title:     AGRA: AI-augmented Geographic Routing Approach for IoT-based */
/*             Incident-Supporting Applications                             */
/*  Revision:  1.0         6/19/2017                                        */
/****************************************************************************/
#include "agra-helper.h"
#include "ns3/agra.h"
#include "ns3/node-list.h"
#include "ns3/names.h"
#include "ns3/ipv4-list-routing.h"
#include "ns3/node-container.h"
#include "ns3/callback.h"
#include "ns3/udp-l4-protocol.h"
#include "ns3/tcp-l4-protocol.h"

namespace ns3 {

AgraHelper::AgraHelper ()
  : Ipv4RoutingHelper ()
{
  m_agentFactory.SetTypeId ("ns3::agra::RoutingProtocol");
}

AgraHelper*
AgraHelper::Copy (void) const
{
  return new AgraHelper (*this);
}

Ptr<Ipv4RoutingProtocol>
AgraHelper::Create (Ptr<Node> node) const
{
  //Ptr<Ipv4L4Protocol> ipv4l4 = node->GetObject<Ipv4L4Protocol> ();
  Ptr<agra::RoutingProtocol> agra = m_agentFactory.Create<agra::RoutingProtocol> ();
  //agra->SetDownTarget (ipv4l4->GetDownTarget ());
  //ipv4l4->SetDownTarget (MakeCallback (&agra::RoutingProtocol::AddHeaders, agra));
  node->AggregateObject (agra);
  return agra;
}

void
AgraHelper::Set (std::string name, const AttributeValue &value)
{
  m_agentFactory.Set (name, value);
}


void 
AgraHelper::Install (void) const
{
  NodeContainer c = NodeContainer::GetGlobal ();
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      Ptr<Node> node = (*i);
      Ptr<UdpL4Protocol> udp = node->GetObject<UdpL4Protocol> ();
      Ptr<TcpL4Protocol> tcp = node->GetObject<TcpL4Protocol> ();
      Ptr<agra::RoutingProtocol> agra = node->GetObject<agra::RoutingProtocol> ();
      //Ptr<LocationService> lS = CreateObject<GodLocationService>();
      //agra->SetLS(lS);
      agra->SetUdpDownTarget (udp->GetDownTarget ());
      udp->SetDownTarget (MakeCallback(&agra::RoutingProtocol::AddHeaders, agra));
      agra->SetTcpDownTarget (tcp->GetDownTarget ());
      tcp->SetDownTarget (MakeCallback(&agra::RoutingProtocol::AddHeaders, agra));
    }


}


}
