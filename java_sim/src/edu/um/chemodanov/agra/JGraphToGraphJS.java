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

package edu.um.chemodanov.agra;

import edu.um.chemodanov.agra.model.Node;
import edu.um.chemodanov.agra.model.Obstacle;
import edu.um.chemodanov.agra.model.Topology;
import edu.um.chemodanov.agra.util.EuclDist;

import java.util.Collections;
import java.util.List;

public class JGraphToGraphJS {
    private List<Node> route = Collections.EMPTY_LIST;
    private int resolution;

    public JGraphToGraphJS(int resolution) {
        this.resolution = resolution;
    }

    public void setRoute(List route) {
        this.route = route;
    }

    public String convertJGraphToJS(Topology t) {
        int windowSize = resolution;
        int padding = 10;
        int planeSize = t.getGridSize();

        int r = (windowSize) / (4 * planeSize);

        StringBuilder gJS = new StringBuilder("var svgContainer = d3.select(\"#topologyarea\").append(\"svg\")\n")
                .append("                                    .attr(\"width\",").append(windowSize).append(")\n")
                .append("                                    .attr(\"height\", ").append(windowSize).append(")\n");

        for (Obstacle o : t.getObstacles()) {
            double x = o.getX();
            double y = o.getY();
            double obstacle_r = o.getR() - 0.25;

            x = x * (windowSize - 2 * r) / planeSize + r;
            y = y * (windowSize - 2 * r) / planeSize + r;
            y = resolution + padding - y;
            obstacle_r = obstacle_r * (windowSize - 2 * r) / planeSize + r;

            gJS.append("\nvar circle").append(o.getId()).append(" = svgContainer.append(\"circle\"")
                    .append(")\n.attr(\"cx\", ").append(x)
                    .append(")\n.attr(\"cy\", ").append(y)
                    .append(")\n.attr(\"r\", ").append(obstacle_r);
            gJS.append(")\n.attr(\"stroke-width\", 1)\n.attr(\"stroke\", \"grey\"").append(")\n.style(\"fill\", \"grey\");");
        }

        t.initializeCircumscribedObstacles();
        for (Obstacle o : t.getCircumscribedObstacles()) {
            double x = o.getX();
            double y = o.getY();
            double obstacle_r = o.getR() - 0.25;

            x = x * (windowSize - 2 * r) / planeSize + r;
            y = y * (windowSize - 2 * r) / planeSize + r;
            y = resolution + padding - y;
            obstacle_r = obstacle_r * (windowSize - 2 * r) / planeSize + r;

            gJS.append("\nvar circle").append(o.getId()).append(" = svgContainer.append(\"circle\"")
                    .append(")\n.attr(\"cx\", ").append(x)
                    .append(")\n.attr(\"cy\", ").append(y)
                    .append(")\n.attr(\"r\", ").append(obstacle_r);
            gJS.append(")\n.attr(\"stroke-width\", 2)\n.attr(\"stroke\", \"black\"").append(")\n.style(\"fill\", \"none\");");
            //print center
            gJS.append("\nvar circle").append(o.getId()).append(" = svgContainer.append(\"circle\"")
                    .append(")\n.attr(\"cx\", ").append(x)
                    .append(")\n.attr(\"cy\", ").append(y)
                    .append(")\n.attr(\"r\", ").append(r / 2);
            gJS.append(")\n.attr(\"stroke-width\", 1)\n.attr(\"stroke\", \"black\"").append(")\n.style(\"fill\", \"black\");");

        }


        for (int i = 0; i < route.size() - 1; i++) {
            int x1 = route.get(i).getX();
            int x2 = route.get(i + 1).getX();
            int y1 = route.get(i).getY();
            int y2 = route.get(i + 1).getY();
            int fontSize = r;

            x1 = x1 * (windowSize - 2 * r) / planeSize + r;
            y1 = y1 * (windowSize - 2 * r) / planeSize + r;
            x2 = x2 * (windowSize - 2 * r) / planeSize + r;
            y2 = y2 * (windowSize - 2 * r) / planeSize + r;

            y1 = resolution + padding - y1;
            y2 = resolution + padding - y2;

            gJS.append("\nvar line").append(i + 1).append(" = svgContainer.append(\"line\"")
                    .append(")\n.attr(\"x1\", ").append(x1)
                    .append(")\n.attr(\"y1\", ").append(y1)
                    .append(")\n.attr(\"x2\", ").append(x2)
                    .append(")\n.attr(\"y2\", ").append(y2);

            gJS.append(")\n.attr(\"stroke-width\", 2)\n.attr(\"stroke\", \"red\");");


            gJS.append("\nvar circleText").append(i + 1).append(" = svgContainer.append(\"text\"")
                    .append(")\n.attr(\"x\", ").append((x2 + x1) / 2)
                    .append(")\n.attr(\"y\", ").append((y2 + y1) / 2)
                    .append(")\n.text(\"").append(" ")
                    .append(Double.valueOf(Math.round(EuclDist.d(route.get(i).getX(), route.get(i).getY(), route.get(i + 1).getX(), route.get(i + 1).getY()) * 10)) / 10)
                    .append(" m").append("\").attr(\"font-family\", \"sans-serif\"");

            gJS.append(").attr(\"font-size\", \"").append(fontSize + 2).append("px\").attr(\"fill\", \"red\");");
        }

        for (Node n : t.getNodes()) {
            if (n.isOn()) {
                int x = n.getX();
                int y = n.getY();
                int fontSize = r;

                x = x * (windowSize - 2 * r) / planeSize + r;
                y = y * (windowSize - 2 * r) / planeSize + r;
                y = resolution + padding - y;

                gJS.append("\nvar circle").append(n.getId()).append(" = svgContainer.append(\"circle\"")
                        .append(")\n.attr(\"cx\", ").append(x)
                        .append(")\n.attr(\"cy\", ").append(y)
                        .append(")\n.attr(\"r\", ").append(r);

                if (route.contains(n)) {
                    if (isNodeSrc(n))
                        gJS.append(")\n.attr(\"stroke-width\", 2)\n.attr(\"stroke\", \"black\"").append(")\n.style(\"fill\", \"aqua\");");
                    else if (isNodeDst(n))
                        gJS.append(")\n.attr(\"stroke-width\", 2)\n.attr(\"stroke\", \"black\"").append(")\n.style(\"fill\", \"red\");");
                    else
                        gJS.append(")\n.attr(\"stroke-width\", 2)\n.attr(\"stroke\", \"black\"").append(")\n.style(\"fill\", \"gold\");");
                }
//                else if (n.isBorder()) {
//                    if (n.getPredecessor() == -1)
//                        gJS.append(")\n.attr(\"stroke-width\", 1)\n.attr(\"stroke\", \"black\"").append(")\n.style(\"fill\", \"purple\");");
//                    else if (n.getLoop() != -1)
//                        gJS.append(")\n.attr(\"stroke-width\", 1)\n.attr(\"stroke\", \"black\"").append(")\n.style(\"fill\", \"green\");");
//                    else
//                        gJS.append(")\n.attr(\"stroke-width\", 1)\n.attr(\"stroke\", \"black\"").append(")\n.style(\"fill\", \"grey\");");
//                }
                else
                    gJS.append(")\n.attr(\"stroke-width\", 1)\n.attr(\"stroke\", \"black\"").append(")\n.style(\"fill\", \"white\");");

//                if (!n.isBorder())
                gJS.append("\nvar circleText").append(n.getId()).append(" = svgContainer.append(\"text\"")
                        .append(")\n.attr(\"x\", ").append(x - 1.8 * r / 2)
                        .append(")\n.attr(\"y\", ").append(y + r / 2)
                        .append(")\n.text(\"").append(n.getId()).append("\").attr(\"font-family\", \"sans-serif\"")
                        .append(").attr(\"font-size\", \"").append(fontSize)
                        .append("px\").attr(\"fill\", \"black\");");
//                else
//                {
//                    gJS.append("\nvar circleText").append(n.getId()).append(" = svgContainer.append(\"text\"")
//                            .append(")\n.attr(\"x\", ").append(x - 1.8 * r / 2)
//                            .append(")\n.attr(\"y\", ").append(y - r / 2)
//                            .append(")\n.append(\"tspan\"")
//                            .append(")\n.attr(\"x\", ").append(x - 1.8 * r / 2)
//                            .append(")\n.attr(\"dy\", ").append(4)//.append("em")
//                            .append(")\n.text(\"").append(n.getId()).append("\").attr(\"font-family\", \"sans-serif\"")
//                            .append(").attr(\"font-size\", \"").append(fontSize)
//                            .append("px\").attr(\"fill\", \"black\"")
//                            .append(")\n.append(\"tspan\"")
//                            .append(")\n.attr(\"x\", ").append(x - 1.75 * r / 2)
//                            .append(")\n.attr(\"dy\", ").append(6)//.append("em")
//                            .append(")\n.text(\"").append(n.getPredecessor()).append(",").append(n.getDist()).append("\").attr(\"font-family\", \"sans-serif\"")
//                            .append(").attr(\"font-size\", \"").append(3 * fontSize / 4)
//                            .append("px\").attr(\"fill\", \"black\");");
//                }
            }
        }

        return gJS.toString();
    }


    private boolean isNodeSrc(Node src) {
        if (!route.isEmpty())
            return route.get(0).equals(src);
        return false;
    }

    private boolean isNodeDst(Node dst) {
        if (!route.isEmpty()) {
            Node s = route.get(route.size() - 1);

            return s.equals(dst);
        }

        return false;
    }
}

