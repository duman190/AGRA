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

package edu.um.chemodanov.agra.util;

import edu.um.chemodanov.agra.model.Node;

public class EuclDist
{
    /**
     * Finds 2D Euclidean distance between two points
     * @param x1 - x coordinate of the source
     * @param y1 - y coordinate of the source
     * @param x2 - x coordinate of the destination
     * @param y2 - y coordinate of the destination
     * @return 2D Euclidean distance
     */
    public static double d(double x1, double y1, double x2, double y2)
    {
        return Math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    }

    /**
     * Finds 2D Euclidean distance between two nodes
     * @param src - source node
     * @param dst - destination node
     * @return 2D Euclidean distance
     */
    public static double d(Node src, Node dst)
    {
        return d(src.getX(), src.getY(), dst.getX(), dst.getY());
    }
}
