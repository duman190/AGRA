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

import java.util.*;

public class ARPSR {
    private double deg;

    public ARPSR() {
        this.deg = 2;
    }

    public void setDeg(double deg) {
        this.deg = deg;
    }

    /**
     * Forward packet from the source to the destination using ARPSR based on the local obstacles knowledge
     *
     * @param src - source node
     * @param dst - destination node
     * @param t - topology
     * @param ttl - the packet's TTL
     * @return Resulting path
     */
    public List<Node> potentialGreedyForwarding(int src, int dst, Topology t, int ttl) {
        List<Node> nodes = t.getNodes();
        List<Obstacle> obstacles = t.getCircumscribedObstacles();
        List<Node> path = new ArrayList<>();
        path.add(nodes.get(src));
        Node dstN = nodes.get(dst);
        double q = 1;

        //information stored on the packet
        double lastPRepulsion = Double.MAX_VALUE;
        double lastPAttraction = Double.MAX_VALUE;
        Node Lp = null;
        List<Double> Lf = new ArrayList<>(2);
        List<Node> e0 = new ArrayList<>(2);
        boolean recovery = false;
        //start forwarding
        while (path.get(path.size() - 1).getId() != dst && path.size() < ttl) {
            Node n = path.get(path.size() - 1);
            Node next = null;
            //check if some local obstacles are known
            Set<Obstacle> currentObstacles = inObstacleRepulseZone(n, obstacles, n.getR());
            //induce charges on local obstacles
            induceCharge(dstN, currentObstacles, q);
            //compute current dist
            double nDist = EuclDist.d(n.getX(), n.getY(), dstN.getX(), dstN.getY());
            ////compute n potential
            double nPotential = -q / nDist;
            for (Obstacle h : currentObstacles)
                nPotential += h.getQ() / (Math.pow(EuclDist.d(n.getX(), n.getY(), h.getX(), h.getY()), deg));
            //if last Repulsion potential is greater than for current hop and in Repulsion zone, proceed safely with Repulsion mode
            if (nPotential < lastPRepulsion && !currentObstacles.isEmpty() && !recovery) {
                //learn potential of current Repulsion mode
                lastPRepulsion = nPotential;
                //compute neighbors potential in Repulsion mode
                double minPotential = nPotential;
                for (Node neighbor : n.getNeighbors()) {
                    double potential = -q / EuclDist.d(neighbor.getX(), neighbor.getY(), dstN.getX(), dstN.getY());
                    for (Obstacle h : currentObstacles)
                        potential += h.getQ() / (Math.pow(EuclDist.d(neighbor.getX(), neighbor.getY(), h.getX(), h.getY()), deg));
                    if (potential < minPotential) {
                        next = neighbor;
                        minPotential = potential;
                    }
                }
            }
            if (next == null && -q / nDist < lastPAttraction) { //Repulsion local minimum, proceed in Attraction mode (i.e., GF mode)
                recovery = false;
                //restore default values of Planar Traverse Mode
                Lp = null;
                Lf = new ArrayList<>(2);
                e0 = new ArrayList<>(2);
                //learn potential of current Attraction mode
                lastPAttraction = -q / nDist;
                //compute neighbors potential in Attraction mode
                double minPotential = -q / nDist;
                for (Node neighbor : n.getNeighbors()) {
                    double potential = -q / EuclDist.d(neighbor.getX(), neighbor.getY(), dstN.getX(), dstN.getY());
                    if (potential < minPotential) {
                        next = neighbor;
                        minPotential = potential;
                    }
                }
            }
            if (next == null) { //ARPSR in both Repulsion and Attraction modes is unavailable to route packet proceed in Planar Traverse mode
                recovery = true;
                if (Lp == null) {
                    //set-up initial packet fields
                    Lp = n;
                    Lf.clear();
                    Lf.add(Double.valueOf(n.getX()));
                    Lf.add(Double.valueOf(n.getY()));
                    next = periInitForward2(n, dstN.getX(), dstN.getY());
                    //set-up first visited edge on the face
                    e0.clear();
                    e0.add(n);
                    e0.add(next);
                } else {
                    next = rightHandForward2(n, path.get(path.size() - 2));
                    if (!e0.isEmpty() && n.equals(e0.get(0)) && next.equals(e0.get(1))) {
                        System.out.println("ARPSR Local: Esrc=" + e0.get(0).getId() + " Edst=" + e0.get(1).getId() + " n=" + n.getId() + "n=" + next.getId());
                        return path;
                    } else {
                        next = faceChange(n, next, Lp, dstN, Lf, e0);
                    }
                }
                if (next == null) {
                    System.out.println("ARPSR Local: Perimeter Forwarding didn't found next hop!");
                    return path;
                }
            }
            path.add(next);
        }

        if (path.size() >= ttl)
            System.out.println("ARPSR Local DID NOT FOUND DST!!!:" + path.size() + " Path " + src + "->" + dst +
                    " Last Repulsion Proximity=" + lastPRepulsion + "Last Attraction Proximity=" + lastPAttraction);
        return path;
    }

    /**
     * This method is used to retrieve local obstacles, i.e., which repulsion zones contain node n
     * (based on Equation 18 in the AGRA paper)
     * @param n - current node
     * @param obstacles - Collection of the detected at the edge obstacles
     * @param radioRange - radio range of the node
     * @return set of local obstacles for node n
     */
    private Set<Obstacle> inObstacleRepulseZone(Node n, Collection<Obstacle> obstacles, double radioRange) {
        Set<Obstacle> currentObstacles = new HashSet<>();
        for (Obstacle h : obstacles)
            if (EuclDist.d(h.getX(), h.getY(), n.getX(), n.getY()) <= ((1 + 1 / deg) * h.getR() + radioRange))
                currentObstacles.add(h);
        return currentObstacles;
    }

    /**
     * This method is used by a node to generate charges at its locally known obstacles
     * (i.e., using Equation 16 in the AGRA paper)
     * @param dstN - destination node (i.e., its coordinates)
     * @param obstacles - collection of local obstacles
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

    /**
     * internal method for GPSR perimeter mode
     * @param n
     * @param dstX
     * @param dstY
     * @return
     */
    private Node periInitForward2(Node n, double dstX, double dstY) {
        double sDist = EuclDist.d(n.getX(), n.getY(), dstX, dstY);

        Node minne = null;
        double min = Double.MAX_VALUE;

        for (Node ne : n.getNeighbors())
            if (!n.isProhibited(ne)) {
                double neDist = EuclDist.d(n, ne);
                double angle = Math.acos(((dstX - n.getX()) * (ne.getX() - n.getX())
                        + (dstY - n.getY()) * (ne.getY() - n.getY())) / (sDist * neDist)); // cos(q)= a*b/|a||b|
                if (!isNeighborOnLeftSide(ne, n, dstX, dstY))
                    angle = 2 * Math.PI - angle;

                if (angle < min) //at least one sensor will be found
                {
                    minne = ne;
                    min = angle;
                }
            }
        return minne;
    }

    /**
     * internal method for GPSR perimeter mode
     * @param n
     * @param inne
     * @return
     */
    private Node rightHandForward2(Node n, Node inne) {
        double sDist = EuclDist.d(n, inne);

        Node minne = null;
        double min = Double.MAX_VALUE;

        for (Node ne : n.getNeighbors())
            if (!ne.equals(inne) && !n.isProhibited(ne)) {
                double neDist = EuclDist.d(n, ne);
                double angle = Math.acos(((inne.getX() - n.getX()) * (ne.getX() - n.getX())
                        + (inne.getY() - n.getY()) * (ne.getY() - n.getY())) / (sDist * neDist)); // cos(q)= a*b/|a||b|
                if (!isNeighborOnLeftSide(ne, n, inne))
                    angle = 2 * Math.PI - angle;

                if (angle < min) //at least one sensor will be found
                {
                    minne = ne;
                    min = angle;
                }
            }

        if (minne == null)
            return inne;
        else
            return minne;
    }

    /**
     * internal method for GPSR perimeter mode
     * @param n
     * @param next
     * @param Lp
     * @param dstS
     * @param Lf
     * @param e0
     * @return
     */
    private Node faceChange(Node n, Node next, Node Lp, Node dstS, List<Double> Lf, List<Node> e0) {
        List<Double> intersection = lineCrossed(Lp, dstS, n, next);
        if (!intersection.isEmpty() &&
                EuclDist.d(intersection.get(0), intersection.get(1), dstS.getX(), dstS.getY())
                        < EuclDist.d(Lf.get(0), Lf.get(1), dstS.getX(), dstS.getY())) {
            Lf = intersection;
            next = rightHandForward2(n, next);
            next = faceChange(n, next, Lp, dstS, Lf, e0);
            e0.clear();
            e0.add(n);
            e0.add(next);
        }

        return next;
    }

    /**
     * internal method for GPSR perimeter mode
     * @param Lp
     * @param dstS
     * @param s
     * @param next
     * @return
     */
    private List<Double> lineCrossed(Node Lp, Node dstS, Node s, Node next) {
        double[] dy = new double[2];
        double[] dx = new double[2];
        double[] m = new double[2];
        double[] b = new double[2];
        double xint, yint;
        List<Double> intersection = new ArrayList<>(2);

        double x1 = Lp.getX();
        double y1 = Lp.getY();
        double x2 = dstS.getX();
        double y2 = dstS.getY();
        double x3 = s.getX();
        double y3 = s.getY();
        double x4 = next.getX();
        double y4 = next.getY();

        dy[0] = y2 - y1;
        dx[0] = x2 - x1;
        dy[1] = y4 - y3;
        dx[1] = x4 - x3;
        m[0] = dy[0] / dx[0];
        m[1] = dy[1] / dx[1];
        b[0] = y1 - m[0] * x1;
        b[1] = y3 - m[1] * x3;
        if (m[0] != m[1]) {
            // slopes not equal, compute intercept
            xint = (b[0] - b[1]) / (m[1] - m[0]);
            yint = m[1] * xint + b[1];
            // is intercept in both line segments?
            if ((xint <= Math.max(x1, x2)) && (xint >= Math.min(x1, x2)) &&
                    (yint <= Math.max(y1, y2)) && (yint >= Math.min(y1, y2)) &&
                    (xint <= Math.max(x3, x4)) && (xint >= Math.min(x3, x4)) &&
                    (yint <= Math.max(y3, y4)) && (yint >= Math.min(y3, y4))) {
                intersection.add(xint);
                intersection.add(yint);
            }
        }
        return intersection;
    }

    /**
     * internal method for GPSR perimeter mode
     * @param ne
     * @param s
     * @param inne
     * @return
     */
    private boolean isNeighborOnLeftSide(Node ne, Node s, Node inne) {
        return isNeighborOnLeftSide(ne, s, inne.getX(), inne.getY());
    }

    /**
     * internal method for GPSR perimeter mode
     * @param ne
     * @param s
     * @param dstX
     * @param dstY
     * @return
     */
    private boolean isNeighborOnLeftSide(Node ne, Node s, double dstX, double dstY) //move counterclock-wise
    {
        if (((dstX - s.getX()) * (ne.getY() - s.getY())
                - (dstY - s.getY()) * (ne.getX() - s.getX())) >= 0)
            return true;
        else
            return false;
    }
}
