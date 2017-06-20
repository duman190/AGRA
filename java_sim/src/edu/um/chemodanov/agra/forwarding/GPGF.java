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

package edu.um.chemodanov.agra.forwarding;

import edu.um.chemodanov.agra.model.Node;
import edu.um.chemodanov.agra.model.Topology;
import edu.um.chemodanov.agra.util.EuclDist;

import java.util.*;

public class GPGF {
    /**
     * Forward packet from the source to the destination using GPGF
     *
     * @param src - source node
     * @param dst - destination node
     * @param t - topology
     * @param ttl - i.e., the packet's TTL
     * @return Resulting path
     */
    public List<Node> greedyForwarding(int src, int dst, Topology t, int ttl) {
        return greedyForwarding(src, dst, t, ttl, new ArrayList<Integer>(1));
    }

    /**
     * Forward packet from the source to the destination using GPGF
     * This method also saves information about header size into headerSize list
     * @param src - source node
     * @param dst - destination node
     * @param t - topology
     * @param ttl - the packet's TTL
     * @param headerSize - list to store information about GPGF header
     * @return Resulting path
     */
    public List<Node> greedyForwarding(int src, int dst, Topology t, int ttl, List<Integer> headerSize) {
        List<Node> nodes = t.getNodes();
        List<Node> path = new ArrayList<>();
        Map<Node, Integer> visits = new HashMap<>();
        path.add(nodes.get(src));
        Node dstN = nodes.get(dst);

        while (path.get(path.size() - 1).getId() != dst && path.size() < ttl) {
            Node n = path.get(path.size() - 1);
            Node next = null;
            double min = EuclDist.d(n.getX(), n.getY(), dstN.getX(), dstN.getY());

            for (Node neighbor : n.getNeighbors())
                if (EuclDist.d(neighbor.getX(), neighbor.getY(), dstN.getX(), dstN.getY()) < min) {
                    next = neighbor;
                    min = EuclDist.d(neighbor.getX(), neighbor.getY(), dstN.getX(), dstN.getY());
                }

            if (next == null) //indicates local minimum
                try {
                    if (visits.containsKey(n))
                        visits.put(n, visits.get(n) + 1);
                    else
                        visits.put(n, 1);
                    pressureModeForwarding(path, dstN, ttl, visits);
                } catch (PerimeterForwardingException e) //indicates absence of neighbors for n (disconnected node)
                {
                    System.out.println(e.toString());
                    headerSize.add(visits.size());
                    return path;
                }
            else
                path.add(next);
        }

        headerSize.add(visits.size());
        return path;
    }

    /**
     * This method forwards packet in a Pressure mode of GPGF (when possible returns back to Greedy Forwarding mode)
     * @param path - path traversed by a packet so far
     * @param dstN - destination node
     * @param ttl - packet's TTL
     * @param visits - map to store information about packet's node visits
     * @throws PerimeterForwardingException
     */
    private void pressureModeForwarding(List<Node> path, Node dstN, int ttl, Map<Node, Integer> visits) throws PerimeterForwardingException {
        Node n = path.get(path.size() - 1);
        double sDist = EuclDist.d(n.getX(), n.getY(), dstN.getX(), dstN.getY());
        while (path.get(path.size() - 1).getId() != dstN.getId() && path.size() < ttl) {
            n = path.get(path.size() - 1);
            int minVisits = Integer.MAX_VALUE;
            for (Node neighbor : n.getNeighbors())
                if (!visits.containsKey(neighbor)) {
                    minVisits = 0;
                    break;
                } else if (visits.get(neighbor) < minVisits)
                    minVisits = visits.get(neighbor);

            List<Node> candidates = new ArrayList<>();
            for (Node neighbor : n.getNeighbors())
                if (!visits.containsKey(neighbor))
                    candidates.add(neighbor);
                else if (visits.get(neighbor) == minVisits)
                    candidates.add(neighbor);

            Node next = null;
            double min = Double.MAX_VALUE;
            for (Node candidate : candidates)
                if (EuclDist.d(candidate.getX(), candidate.getY(), dstN.getX(), dstN.getY()) < min) {
                    next = candidate;
                    min = EuclDist.d(next.getX(), next.getY(), dstN.getX(), dstN.getY());
                }

            if (next == null)
                throw new PerimeterForwardingException("Pressure Mode Forwarding didn't found next hop!");

            path.add(next);

            //store new visits
            if (!visits.containsKey(next))
                visits.put(next, 1);
            else
                visits.put(next, visits.get(next) + 1);

            if (EuclDist.d(next.getX(), next.getY(), dstN.getX(), dstN.getY()) < sDist) //return to GF
                return;
        }
    }
}
