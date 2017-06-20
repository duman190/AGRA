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
import edu.um.chemodanov.agra.model.Obstacle;
import edu.um.chemodanov.agra.model.Topology;
import edu.um.chemodanov.agra.util.EuclDist;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class ARGF_Global {
    private double deg;

    public ARGF_Global() {
        this.deg = 2;
    }

    public void setDeg(double deg) {
        this.deg = deg;
    }

    /**
     * Forward packet from the source to the destination using ARGF based on the global obstacles knowledge
     * @param src - source node
     * @param dst - destination node
     * @param t - topology
     * @param maxLength - i.e., the packet's TTL
     * @return Resulting path
     */
    public List<Node> potentialGreedyForwarding(int src, int dst, Topology t, int maxLength) {
        List<Node> nodes = t.getNodes();
        List<Obstacle> obstacles = t.getCircumscribedObstacles(); //get global obstacles
        List<Node> path = new ArrayList<>();
        path.add(nodes.get(src));
        Node dstN = nodes.get(dst);
        double q = 1;

        //information stored on the packet
        double lastPRepulsion = Double.MAX_VALUE;
        while (path.get(path.size() - 1).getId() != dst && path.size() < maxLength) {
            Node n = path.get(path.size() - 1);
            Node next = null;
            //induce charges on global obstacles
            induceCharge(dstN, obstacles, q);
            //compute current dist
            double nDist = EuclDist.d(n.getX(), n.getY(), dstN.getX(), dstN.getY());
            ////compute n potential
            double nPotential = -q / nDist;
            for (Obstacle h : obstacles)
                nPotential += h.getQ() / (Math.pow(EuclDist.d(n.getX(), n.getY(), h.getX(), h.getY()), deg));
            //if last Repulsion potential is greater than for current hop and in Repulsion zone, proceed safely with Repulsion mode
            if (nPotential < lastPRepulsion && !obstacles.isEmpty()) {
                //learn potential of current Repulsion mode
                lastPRepulsion = nPotential;
                //compute neighbors potential in Repulsion mode
                double minPotential = nPotential;
                for (Node neighbor : n.getNeighbors()) {
                    double potential = -q / EuclDist.d(neighbor.getX(), neighbor.getY(), dstN.getX(), dstN.getY());
                    for (Obstacle h : obstacles)
                        potential += h.getQ() / (Math.pow(EuclDist.d(neighbor.getX(), neighbor.getY(), h.getX(), h.getY()), deg));
                    if (potential < minPotential) {
                        next = neighbor;
                        minPotential = potential;
                    }
                }
            }
            if (next == null) { //Repulsion local minimum, proceed in Attraction mode (i.e., GF mode)
                //compute neighbors potential in Attraction mode
                double minPotential = -q / nDist;
                for (Node neighbor : n.getNeighbors()) {
                    double potential = -q / EuclDist.d(neighbor.getX(), neighbor.getY(), dstN.getX(), dstN.getY());
                    if (potential < minPotential) {
                        next = neighbor;
                        minPotential = potential;
                    }
                }
                if (next == null) {
                    System.out.println("ARGF Global faced LM!!!:" + path.size() + " Path " + src + "->" + dst +
                            " Last Proximity=" + lastPRepulsion);
                    return path;
                }
            }
            path.add(next);
        }
        return path;
    }

    /**
     * This method is used by a node to generate charges at its locally known obstacles
     * (i.e., using Equation 16 in the AGRA paper)
     * @param dstN - destination node (i.e., its coordinates)
     * @param obstacles - collection of global obstacles
     * @param initialQ - intial charge with is by default is 1
     */
    private void induceCharge(Node dstN, Collection<Obstacle> obstacles, double initialQ) {
        for (Obstacle h : obstacles)
        {
            double b = EuclDist.d(h.getX(), h.getY(), dstN.getX(), dstN.getY());
            double r = h.getR();
            double newQ2 = (initialQ * Math.pow(r, deg + 1)) / (deg * Math.pow(b + r, 2)); //electric tension on border = 0
            double qx = h.getX();
            double qy = h.getY();
            h.setQ(newQ2);
            h.setQx(qx);
            h.setQy(qy);
        }
    }
}