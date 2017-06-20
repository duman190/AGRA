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

package edu.um.chemodanov.agra.spark;

import edu.um.chemodanov.agra.JGraphToGraphJS;
import edu.um.chemodanov.agra.forwarding.*;
import edu.um.chemodanov.agra.model.Node;
import edu.um.chemodanov.agra.model.Topology;
import edu.um.chemodanov.agra.routing.BFS;
import org.apache.commons.lang3.StringUtils;
import spark.Request;
import spark.Response;
import spark.Route;

import java.util.*;

import static spark.Spark.*;

public class SparkLauncher
{

    public static void start(final Topology t, final int resolution)
    {
        final BFS bfs = new BFS();
        final GF gf = new GF();
        final GPGF gpgf = new GPGF();
        final ARPGF_Local arpgf = new ARPGF_Local();
        final ARGF_Local argf = new ARGF_Local();

        final JGraphToGraphJS gToJS = new JGraphToGraphJS(resolution);

        get(new Route("/topology.html")
        {
            @Override
            public Object handle(Request request, Response response)
            {
                StringBuilder str = new StringBuilder();

                str.append(" <!DOCTYPE html>\n");
                str.append("<html><head>");
                str.append("\n    <meta http-equiv=\"Content-Type\" content=\"text/html; charset=UTF-8\">"
                        + "    <title>AGRA Simulator v1.0</title>");
                str.append("<script type=\"text/javascript\" src=\"http://d3js.org/d3.v2.min.js\"></script>");
                str.append("</head><body>");

//                logic of working with user
                List<Node> route = new ArrayList<>();
                try
                {
                    if (!StringUtils.isEmpty(request.queryParams("bfs")))
                    {
                        if (!StringUtils.isEmpty(request.queryParams("srcID")) && !StringUtils.isEmpty(request.queryParams("dstID")))
                        {
                            int srcID = Integer.parseInt(request.queryParams("srcID"));
                            int dstID = Integer.parseInt(request.queryParams("dstID"));

                            if (!StringUtils.isEmpty(request.queryParams("radius")))
                            {
                                double radius = Double.parseDouble(request.queryParams("radius"));
                                for (Node s : t.getNodes())
                                    s.setR(radius);
                                t.reInitializeNeighbors();
                            }

                            route = bfs.breadthFirstSearch(srcID, dstID, t);

                            gToJS.setRoute(route);
                        }
                    } else if (!StringUtils.isEmpty(request.queryParams("gf")))
                    {
                        if (!StringUtils.isEmpty(request.queryParams("srcID")) && !StringUtils.isEmpty(request.queryParams("dstID")))
                        {
                            int srcID = Integer.parseInt(request.queryParams("srcID"));
                            int dstID = Integer.parseInt(request.queryParams("dstID"));

                            if (!StringUtils.isEmpty(request.queryParams("radius")))
                            {
                                double radius = Double.parseDouble(request.queryParams("radius"));
                                for (Node n : t.getNodes())
                                    n.setR(radius);
                                t.reInitializeNeighbors();
                            }

                            route = gf.greedyForwarding(srcID, dstID, t, false, 1000);

                            gToJS.setRoute(route);
                        }
                    } else if (!StringUtils.isEmpty(request.queryParams("gpsr")))
                    {
                        if (!StringUtils.isEmpty(request.queryParams("srcID")) && !StringUtils.isEmpty(request.queryParams("dstID")))
                        {
                            int srcID = Integer.parseInt(request.queryParams("srcID"));
                            int dstID = Integer.parseInt(request.queryParams("dstID"));

                            if (!StringUtils.isEmpty(request.queryParams("radius")))
                            {
                                double radius = Double.parseDouble(request.queryParams("radius"));
                                for (Node n : t.getNodes())
                                    n.setR(radius);
                                t.reInitializeNeighbors();
                            }

                            route = gf.greedyForwarding(srcID, dstID, t, true, 1000);

                            gToJS.setRoute(route);
                        }
                    } else if (!StringUtils.isEmpty(request.queryParams("gpgf")))
                    {
                        if (!StringUtils.isEmpty(request.queryParams("srcID")) && !StringUtils.isEmpty(request.queryParams("dstID")))
                        {
                            int srcID = Integer.parseInt(request.queryParams("srcID"));
                            int dstID = Integer.parseInt(request.queryParams("dstID"));

                            if (!StringUtils.isEmpty(request.queryParams("radius")))
                            {
                                double radius = Double.parseDouble(request.queryParams("radius"));
                                for (Node n : t.getNodes())
                                    n.setR(radius);
                                t.reInitializeNeighbors();
                            }

                            route = gpgf.greedyForwarding(srcID, dstID, t, 1000);

                            gToJS.setRoute(route);
                        }
                    } else if (!StringUtils.isEmpty(request.queryParams("arpgf")))
                    {
                        if (!StringUtils.isEmpty(request.queryParams("srcID")) && !StringUtils.isEmpty(request.queryParams("dstID")))
                        {
                            int srcID = Integer.parseInt(request.queryParams("srcID"));
                            int dstID = Integer.parseInt(request.queryParams("dstID"));

                            if (!StringUtils.isEmpty(request.queryParams("radius")))
                            {
                                double radius = Double.parseDouble(request.queryParams("radius"));
                                for (Node n : t.getNodes())
                                    n.setR(radius);
                                t.reInitializeNeighbors();
                            }

                            if (!StringUtils.isEmpty(request.queryParams("deg")))
                            {
                                double deg = Double.parseDouble(request.queryParams("deg"));
                                arpgf.setDeg(deg);
                            }

                            route = arpgf.potentialGreedyForwarding(srcID, dstID, t, 1000);

                            gToJS.setRoute(route);
                        }
                    } else if (!StringUtils.isEmpty(request.queryParams("argf")))
                    {
                        if (!StringUtils.isEmpty(request.queryParams("srcID")) && !StringUtils.isEmpty(request.queryParams("dstID")))
                        {
                            int srcID = Integer.parseInt(request.queryParams("srcID"));
                            int dstID = Integer.parseInt(request.queryParams("dstID"));

                            if (!StringUtils.isEmpty(request.queryParams("radius")))
                            {
                                double radius = Double.parseDouble(request.queryParams("radius"));
                                for (Node s : t.getNodes())
                                    s.setR(radius);
                                t.reInitializeNeighbors();
                            }

                            if (!StringUtils.isEmpty(request.queryParams("deg")))
                            {
                                double deg = Double.parseDouble(request.queryParams("deg"));
                                argf.setDeg(deg);
                            }

                            route = argf.potentialGreedyForwarding(srcID, dstID, t, 1000);

                            gToJS.setRoute(route);
                        }
                    } else if (!StringUtils.isEmpty(request.queryParams("gen_obstacle")))
                    {
                        if (!StringUtils.isEmpty(request.queryParams("x")) && !StringUtils.isEmpty(request.queryParams("y"))
                                && !StringUtils.isEmpty(request.queryParams("r")))
                        {
                            int x = Integer.parseInt(request.queryParams("x"));
                            int y = Integer.parseInt(request.queryParams("y"));
                            double r = Double.parseDouble(request.queryParams("r"));

                            t.generateObstacle(x, y, r);
                            t.reInitializeNeighbors();

                            gToJS.setRoute(Collections.EMPTY_LIST);
                        }
                    } else if (!StringUtils.isEmpty(request.queryParams("gen_obstacles")))
                    {
                        if (!StringUtils.isEmpty(request.queryParams("num")) && !StringUtils.isEmpty(request.queryParams("minr"))
                                && !StringUtils.isEmpty(request.queryParams("maxr")))
                        {
                            int num = Integer.parseInt(request.queryParams("num"));
                            double minr = Double.parseDouble(request.queryParams("minr"));
                            double maxr = Double.parseDouble(request.queryParams("maxr"));
                            t.generateObstacles(num, minr, maxr);
                            t.reInitializeNeighbors();

                            gToJS.setRoute(Collections.EMPTY_LIST);
                        }
                    } else if (!StringUtils.isEmpty(request.queryParams("clear_obstacles")))
                        t.clearAllObstacles();
                     else
                        gToJS.setRoute(Collections.EMPTY_LIST);

                } catch (Exception e)
                {
                    str.append(e);
                }
//                end

                str.append("<table width=\"100%\" height=\"100%\" border=\"2px\" cellspacing=\"0px\" cellpadding=\"4px\">"
                        + "<tr valign=\"top\">");
                str.append("<td align=\"left\" width=\"200\">");
                str.append(formOutput(request, t, route));
                str.append("</td>");
                str.append("<td align=\"center\" id=\"topologyarea\">"); //using id "topologyarea" to past here js content
                str.append("</td></tr></table>");
                str.append("<script type=\"text/javascript\">" + gToJS.convertJGraphToJS(t) + "</script>");
                str.append("</body>"
                        + "</html>");
                return str.toString();
            }
        });

    }

    private static String formOutput(Request request, Topology t, List<Node> route)
    {
        StringBuilder str = new StringBuilder();
        str.append("<form name=\"mainDashboard\" action=\"topology.html\">");
        str.append(formOutputInt(request, t, route));
        str.append("</form>");
        return str.toString();
    }

    private static String formOutputInt(Request request, Topology t, List<Node> route)
    {
        StringBuilder str = new StringBuilder();
        str.append("<table width=\"100%\" border=\"1px\" cellspacing=\"0px\" cellpadding=\"2px\">");
        str.append("<tr align=\"center\" valign=\"top\">");
        str.append("<tr><td align=\"center\">Area</td></tr>");
        str.append(createAreaForm(t));
        str.append("</tr>");
        str.append("<tr><td align=\"center\">Find Path</td></tr>");
        str.append(createRouteForm(request));
        str.append("</tr>");
        str.append("<tr><td align=\"center\">Generate Obstacle</td></tr>");
        str.append(createObstacleForm(request));
        str.append("</tr>");
        str.append("<tr><td align=\"center\">Console <input type=\"submit\" name=\"clear_logs\" value=\"Clear\"/></td></tr>");
        str.append("<td>");
        str.append("<table width=\"100%\" border=\"0px\" cellspacing=\"0px\" cellpadding=\"1px\" valign\"top\">");
        str.append(printLogs(route));
        str.append("</table></td>");
        str.append("</tr>");
        str.append("</table>");
        return str.toString();
    }

    private static String printLogs(List<Node> route)
    {
        StringBuilder log = new StringBuilder();

        if (route != null)
        {
            log.append("<tr><td align=\"center\">").append("Path with length ")
                    .append(route.size()).append(" hop(s) was found:</td></tr>");

            log.append("<tr><td>");
            for (Node s : route)
                log.append(s.getId()).append("->");

            log.delete(log.length() - 2, log.length()); // delete last "->"
            log.append("</td></tr>");
        }
        return log.toString();
    }

    private static String createAreaForm(Topology t)
    {
        StringBuilder str = new StringBuilder();
        str.append("<td>");
        str.append("<table width=\"100%\" border=\"0px\" cellspacing=\"0px\" cellpadding=\"1px\">");
        int num = numOfSensorsOn(t.getNodes());
        int filled = Math.round(num * 100 / t.getNodes().size());
        str.append("<tr><td align=\"center\">" + num + " sensors over " + t.getGridSize() + "x" + t.getGridSize() + " m2</td></tr>");
        str.append("<tr><td align=\"center\"> (" +
                filled + "% filled)</td></tr>");
        str.append("</table>");
        str.append("</td>");
        return str.toString();
    }

    private static int numOfSensorsOn(List<Node> nodes)
    {
        int num = 0;

        for (Node s : nodes)
            if (s.isOn())
                num++;

        return num;
    }


    private static String createRouteForm(Request request)
    {
        StringBuilder str = new StringBuilder();
        str.append("<td>");
        str.append("<table width=\"100%\" border=\"0px\" cellspacing=\"0px\" cellpadding=\"1px\">");
        str.append("<tr>");
        str.append("<td align=\"left\" width=\"90\">");
        str.append("Src ID:");
        str.append("</td>");
        str.append("<td>");
        str.append("<input type=\"text\" name=\"srcID\" value=\""
                + (StringUtils.isEmpty(request.queryParams("srcID")) ? "" : request.queryParams("srcID"))
                + "\" size=\"1\" />");
        str.append("</td>");
        str.append("<td align=\"left\" width=\"70\">");
        str.append("Dst ID:");
        str.append("</td>");
        str.append("<td>");
        str.append("<input type=\"text\" name=\"dstID\" value=\""
                + (StringUtils.isEmpty(request.queryParams("dstID")) ? "" : request.queryParams("dstID"))
                + "\" size=\"1\" />");
        str.append("</td>");
        str.append("</tr>");
        str.append("<tr>");
        str.append("<td align=\"left\" width=\"90\">");
        str.append("Radio range:");
        str.append("</td>");
        str.append("<td>");
        str.append("<input type=\"text\" name=\"radius\" value=\""
                + (StringUtils.isEmpty(request.queryParams("radius")) ? "" : request.queryParams("radius"))
                + "\" size=\"1\" />");
        str.append("</td>");
        str.append("<td align=\"left\" width=\"70\">");
        str.append("Degree:");
        str.append("</td>");
        str.append("<td>");
        str.append("<input type=\"text\" name=\"deg\" value=\""
                + (StringUtils.isEmpty(request.queryParams("deg")) ? "" : request.queryParams("deg"))
                + "\" size=\"1\" />");
        str.append("</td>");
        str.append("</tr>");
        str.append("</tr>");
        str.append("<tr>");
        str.append("<td align=\"center\" colspan=\"4\">");
        str.append("<input type=\"submit\" name=\"bfs\" value=\"Find Path with BFS (Optimal)\" style=\"width:100%\"/>");
        str.append("</td>");
        str.append("</tr>");
        str.append("<tr>");
        str.append("<td align=\"center\" colspan=\"4\">");
        str.append("<input type=\"submit\" name=\"gf\" value=\"Find Path with GF\" style=\"width:100%\"/>");
        str.append("</td>");
        str.append("</tr>");
        str.append("<tr>");
        str.append("<td align=\"center\" colspan=\"4\">");
        str.append("<input type=\"submit\" name=\"gpsr\" value=\"Find Path with GPSR\" style=\"width:100%\"/>");
        str.append("</td>");
        str.append("</tr>");
        str.append("<tr>");
        str.append("<td align=\"center\" colspan=\"4\">");
        str.append("<input type=\"submit\" name=\"gpgf\" value=\"Find Path with GPGF\" style=\"width:100%\"/>");
        str.append("</td>");
        str.append("</tr>");
        str.append("<tr>");
        str.append("<td align=\"center\" colspan=\"4\">");
        str.append("<input type=\"submit\" name=\"argf\" value=\"Find Path with ARGF\" style=\"width:100%\"/>");
        str.append("</td>");
        str.append("</tr>");
        str.append("<tr>");
        str.append("<td align=\"center\" colspan=\"4\">");
        str.append("<input type=\"submit\" name=\"arpgf\" value=\"Find Path with ARPGF\" style=\"width:100%\"/>");
        str.append("</td>");
        str.append("</tr>");
        str.append("</table>");
        str.append("</td>");
        str.append("</tr>");

        return str.toString();
    }

    private static String createObstacleForm(Request request)
    {
        StringBuilder str = new StringBuilder();
        str.append("<td>");
        str.append("<table width=\"100%\" border=\"0px\" cellspacing=\"0px\" cellpadding=\"1px\">");
        str.append("<tr>");
        str.append("<td align=\"left\" width=\"0\">");
        str.append("X:");
        str.append("</td>");
        str.append("<td>");
        str.append("<input type=\"text\" name=\"x\" value=\""
                + (StringUtils.isEmpty(request.queryParams("x")) ? "" : request.queryParams("x"))
                + "\" size=\"1\" />");
        str.append("</td>");
        str.append("<td align=\"left\" width=\"0\">");
        str.append("Y:");
        str.append("</td>");
        str.append("<td>");
        str.append("<input type=\"text\" name=\"y\" value=\""
                + (StringUtils.isEmpty(request.queryParams("y")) ? "" : request.queryParams("y"))
                + "\" size=\"1\" />");
        str.append("</td>");
        str.append("<td align=\"left\" width=\"0\">");
        str.append("R:");
        str.append("</td>");
        str.append("<td>");
        str.append("<input type=\"text\" name=\"r\" value=\""
                + (StringUtils.isEmpty(request.queryParams("r")) ? "" : request.queryParams("r"))
                + "\" size=\"1\" />");
        str.append("</td>");
        str.append("</tr>");
        str.append("<tr>");
        str.append("<td align=\"center\" colspan=\"6\">");
        str.append("<input type=\"submit\" name=\"gen_obstacle\" value=\"Generate Obstacle\" style=\"width:100%\"/>");
        str.append("</td>");
        str.append("</tr>");
        str.append("<tr>");
        str.append("<td align=\"left\" width=\"0\">");
        str.append("Num:");
        str.append("</td>");
        str.append("<td>");
        str.append("<input type=\"text\" name=\"num\" value=\""
                + (StringUtils.isEmpty(request.queryParams("num")) ? "" : request.queryParams("num"))
                + "\" size=\"1\" />");
        str.append("</td>");
        str.append("<td align=\"left\" width=\"0\">");
        str.append("minR:");
        str.append("</td>");
        str.append("<td>");
        str.append("<input type=\"text\" name=\"minr\" value=\""
                + (StringUtils.isEmpty(request.queryParams("minr")) ? "" : request.queryParams("minr"))
                + "\" size=\"1\" />");
        str.append("</td>");
        str.append("<td align=\"left\" width=\"0\">");
        str.append("maxR:");
        str.append("</td>");
        str.append("<td>");
        str.append("<input type=\"text\" name=\"maxr\" value=\""
                + (StringUtils.isEmpty(request.queryParams("maxr")) ? "" : request.queryParams("maxr"))
                + "\" size=\"1\" />");
        str.append("</td>");
        str.append("</tr>");
        str.append("<tr>");
        str.append("<td align=\"center\" colspan=\"6\">");
        str.append("<input type=\"submit\" name=\"gen_obstacles\" value=\"Generate Obstacles\" style=\"width:100%\"/>");
        str.append("</td>");
        str.append("</tr>");
        str.append("<tr>");
        str.append("<td align=\"center\" colspan=\"6\">");
        str.append("<input type=\"submit\" name=\"clear_obstacles\" value=\"Clear Obstacles\" style=\"width:100%\"/>");
        str.append("</td>");
        str.append("</tr>");
        str.append("</table>");
        str.append("</td>");
        str.append("</tr>");

        return str.toString();
    }

    private static double formatDouble(double d, int dz)
    {
        double dd = Math.pow(10, dz);
        return Math.round(d * dd) / dd;
    }
}


