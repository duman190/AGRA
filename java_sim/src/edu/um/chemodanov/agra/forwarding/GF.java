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

import java.util.ArrayList;
import java.util.List;

public class GF
{
    /**
     * Forward packet from the source to the destination using ordinar Greedy Forwarding based on the local obstacles knowledge
     *
     * @param src - source node
     * @param dst - destination node
     * @param t - topology
     * @param rightHandRule - if true than use perimeter mode for local minimum recovery (i.e., use GPSR)
     * @param ttl - the packet's TTL
     * @return Resulting path
     */
    public List<Node> greedyForwarding(int src, int dst, Topology t, boolean rightHandRule, int ttl)
    {
        List<Node> nodes = t.getNodes();
        List<Node> path = new ArrayList<>();
        path.add(nodes.get(src));
        Node dstS = nodes.get(dst);

        while (path.get(path.size() - 1).getId() != dst && path.size() < ttl)
        {
            Node n = path.get(path.size() - 1);
            Node next = null;
            double min = EuclDist.d(n.getX(), n.getY(), dstS.getX(), dstS.getY());

            for (Node neighbor : n.getNeighbors())
                if (EuclDist.d(neighbor.getX(), neighbor.getY(), dstS.getX(), dstS.getY()) < min)
                {
                    next = neighbor;
                    min = EuclDist.d(neighbor.getX(), neighbor.getY(), dstS.getX(), dstS.getY());
                }

            if (next == null) //indicates local minimum
                if (rightHandRule)
                    try
                    {
                        perimeterModeForwarding(path, dstS, ttl);
                    } catch (PerimeterForwardingException e) //indicates absence of neighbors for n (disconnected node)
                    {
                        System.out.println(e.toString());
                        return path;
                    }
                else
                {
                    System.out.println("GF faced local minimum!");
                    return path;
                }
            else
                path.add(next);
        }

        if (path.get(path.size() - 1).getId() == dst)
            return path;
        else
            return path;
    }

    /**
     * This method forwards packet in a Perimeter mode of GPSR (when possible returns back to Greedy Forwarding mode)
     * @param path - path traversed by a packet so far
     * @param dstN - destination node
     * @param ttl - packet's TTL
     * @throws PerimeterForwardingException
     */
    private void perimeterModeForwarding(List<Node> path, Node dstN, int ttl) throws PerimeterForwardingException
    {
        Node Lp = null;
        List<Double> Lf = new ArrayList<>(2);
        List<Node> e0 = new ArrayList<>(2);

        while (path.get(path.size() - 1).getId() != dstN.getId() && path.size() < ttl)
        {
            Node n = path.get(path.size() - 1);
            Node next = null;
            if (Lp == null)
            {
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
            } else
            {
                next = rightHandForward2(n, path.get(path.size() - 2));

                if (!e0.isEmpty() && n.equals(e0.get(0)) && next.equals(e0.get(1)))
                {
                    System.out.println("Exception 1: src=" + e0.get(0).getId() + " dst=" + e0.get(1).getId() + " n=" + n.getId() + "n=" + next.getId());
                    throw new PerimeterForwardingException("Perimeter Forwarding went through the same edge in the same direction. Terminate!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                } else
                {

                    next = faceChange(n, next, Lp, dstN, Lf, e0);
                }
            }

            if (next == null)
                throw new PerimeterForwardingException("Perimeter Forwarding didn't found next hop!");

            path.add(next);

            if (EuclDist.d(next, dstN) < EuclDist.d(Lp, dstN)) //return to GF
                return;
        }
    }

    /**
     * internal method for GPSR perimeter mode
     * @param n
     * @param dstX
     * @param dstY
     * @return
     */
    private Node periInitForward2(Node n, double dstX, double dstY)
    {
        double nDist = EuclDist.d(n.getX(), n.getY(), dstX, dstY);

        Node minne = null;
        double min = Double.MAX_VALUE;

        for (Node ne : n.getNeighbors())
            if (!n.isProhibited(ne))
            {
                double neDist = EuclDist.d(n, ne);
                double angle = Math.acos(((dstX - n.getX()) * (ne.getX() - n.getX())
                        + (dstY - n.getY()) * (ne.getY() - n.getY())) / (nDist * neDist)); // cos(q)= a*b/|a||b|
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
    private Node rightHandForward2(Node n, Node inne)
    {
        double nDist = EuclDist.d(n, inne);

        Node minne = null;
        double min = Double.MAX_VALUE;

        for (Node ne : n.getNeighbors())
            if (!ne.equals(inne) && !n.isProhibited(ne))
            {
                double neDist = EuclDist.d(n, ne);
                double angle = Math.acos(((inne.getX() - n.getX()) * (ne.getX() - n.getX())
                        + (inne.getY() - n.getY()) * (ne.getY() - n.getY())) / (nDist * neDist)); // cos(q)= a*b/|a||b|
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
    private Node faceChange(Node n, Node next, Node Lp, Node dstS, List<Double> Lf, List<Node> e0)
    {
        List<Double> intersection = lineCrossed(Lp, dstS, n, next);
        if (!intersection.isEmpty() &&
                EuclDist.d(intersection.get(0), intersection.get(1), dstS.getX(), dstS.getY())
                        < EuclDist.d(Lf.get(0), Lf.get(1), dstS.getX(), dstS.getY()))
        {
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
    private List<Double> lineCrossed(Node Lp, Node dstS, Node s, Node next)
    {
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
        if (m[0] != m[1])
        {
            // slopes not equal, compute intercept
            xint = (b[0] - b[1]) / (m[1] - m[0]);
            yint = m[1] * xint + b[1];
            // is intercept in both line segments?
            if ((xint <= Math.max(x1, x2)) && (xint >= Math.min(x1, x2)) &&
                    (yint <= Math.max(y1, y2)) && (yint >= Math.min(y1, y2)) &&
                    (xint <= Math.max(x3, x4)) && (xint >= Math.min(x3, x4)) &&
                    (yint <= Math.max(y3, y4)) && (yint >= Math.min(y3, y4)))
            {
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
    private boolean isNeighborOnLeftSide(Node ne, Node s, Node inne)
    {
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

    /* code from the GPSR Thesis Work
        private Node periInitForward(Node s, double dstX, double dstY)
    {
        // find bearing to dst
        double basebrg = bearing(dstX, dstY, s.getX(), s.getY());
        // find neighbor with greatest bearing <= brg
        Node minne = null;
        double brg, minbrg = 3 * Math.PI;

        for (Node ne : s.getNeighbors())
        {
            if (s.isProhibited(ne))
                continue;
            brg = bearing(ne.getX(), ne.getY(), s.getX(), s.getY()) - basebrg;

            //normilize brg
            if (brg < 0)
                brg += 2 * Math.PI;
            if (brg < 0)
                brg += 2 * Math.PI;
            if (brg < minbrg)
            {
                minbrg = brg;
                minne = ne;
            }
        }
        return minne;
    }

    private Node rightHandForward(Node s, Node inne)
    {
        double basebrg;

        // find bearing from mn to (x, y, z)
        basebrg = bearing(inne.getX(), inne.getY(), s.getX(), s.getY());

        Node minne = null;
        double brg, minbrg = 3 * Math.PI;

        for (Node ne : s.getNeighbors())
        {
            if (ne.equals(inne))
                continue;
            if (s.isProhibited(ne))
                continue;
            brg = bearing(s.getX(), s.getY(), ne.getX(), ne.getY()) - basebrg;

            //normilize brg
            if (brg < 0)
                brg += 2 * Math.PI;
            if (brg < 0)
                brg += 2 * Math.PI;
            if (brg < minbrg)
            {
                minbrg = brg;
                minne = ne;
            }
        }

        if (minne == null)
            return inne;
        else
            return minne;
    }

    private double bearing(double x1, double y1, double x2, double y2)
    {
        double brg;

        // XXX only deal with 2D for now
        // XXX check for (0, 0) args, a domain error
        brg = Math.atan2(y2 - y1, x2 - x1);
        if (brg < 0)
            brg += 2 * Math.PI;
        return brg;
    }
     */
}
