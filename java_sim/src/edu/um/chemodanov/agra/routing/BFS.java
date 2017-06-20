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

package edu.um.chemodanov.agra.routing;

import edu.um.chemodanov.agra.model.Node;
import edu.um.chemodanov.agra.model.Topology;

import java.util.*;

public class BFS implements BFSConstant
{
    /**
     * This method is a common shortest path algorithm (for min number of hops) that relies on global topo knowledge
     * We use it as a baseline for the geographic routing evaluation
     * @param src - source ndoe id
     * @param dst - destination node id
     * @param t - network topology
     * @return Resulting (shortest) path
     */
    public List<Node> breadthFirstSearch(int src, int dst, Topology t)
    {
        List<Node> nodes = t.getNodes();
        for (Node n : nodes)
        {
            n.setColor(WHITE);
            n.setPredecessor(NIL);
        }

        Queue<Node> q = new LinkedList<Node>();
        nodes.get(src).setColor(GRAY);
        q.add(nodes.get(src));
        bfs:
        {
            while (!q.isEmpty())
            {
                Node n = q.poll();
                n.setColor(BLACK);

                for (Node neighbor : n.getNeighbors())
                {
                    if (neighbor.getColor().equals(WHITE))
                    {
                        neighbor.setColor(GRAY);
                        neighbor.setPredecessor(n.getId());
                        q.add(neighbor);
                    }
                    if (neighbor.getId() == dst)
                        break bfs;
                }
            }
        }

        List<Node> path = new ArrayList<>();
        if (nodes.get(dst).getPredecessor() >= 0)
        {
            path.add(nodes.get(dst));
            int predecessor = nodes.get(dst).getPredecessor();

            while (predecessor != src &&
                    predecessor >= 0)
            {
                path.add(nodes.get(predecessor));
                predecessor = path.get(path.size() - 1).getPredecessor();
            }
        }
        if (!path.isEmpty())
        {
            path.add(nodes.get(src));
            Collections.reverse(path);
        }

        return path;
    }
}
