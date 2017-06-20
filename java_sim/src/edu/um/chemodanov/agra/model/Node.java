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

package edu.um.chemodanov.agra.model;

import edu.um.chemodanov.agra.routing.BFSConstant;

import java.util.HashSet;
import java.util.Random;
import java.util.Set;

public class Node implements BFSConstant
{
    private int id;
    private int x;
    private int y;
    private double r;
    private boolean isOn = true;
    private boolean isBorder = false;
    private Set<Node> neighbors; // with asymmetrical links
    private Set<Node> staticNeighbors; //for internal use by topology only
    //this filed is used to limit graph connectivity for construction of the planarized graphs
    private Set<Node> prohibitedNeighbors;

    //routing info
    private String color = WHITE;
    private int predecessor = NIL;
    private int loop = NIL;
    private int component = 0;
    private int dist = 0;

    public Node(int id, int x, int y)
    {
        this.id = id;
        this.x = x;
        this.y = y;
        this.neighbors = new HashSet<>();
        this.staticNeighbors = new HashSet<>();
        this.prohibitedNeighbors = new HashSet<>();
        this.r = new Random().nextDouble()*4 + 1; //generate random radio range
    }

    public int getId()
    {
        return this.id;
    }

    public double getR(){ return  this.r;}

    public void setR(double r) {this.r =r;}

    public int getX()
    {
        return this.x;
    }

    public int getY()
    {
        return this.y;
    }

    public boolean isOn()
    {
        return this.isOn;
    }

    public Set<Node> getNeighbors()
    {
        return this.neighbors;
    }

    protected Set<Node> getStaticNeighbors()
    {
        return this.staticNeighbors;
    }

    protected void setNeighbors(Set<Node> neighbors)
    {
        this.neighbors = neighbors;
    }

    protected void setStaticNeighbors(Set<Node> staticNeighbors)
    {
        this.staticNeighbors = staticNeighbors;
    }

    public void clearProhibitedNeighbors()
    {
        prohibitedNeighbors.clear();
    }

    public boolean isProhibited(Node s)
    {
        return prohibitedNeighbors.contains(s);
    }

    public void addProhibitedNeighbor(Node s)
    {
        prohibitedNeighbors.add(s);
    }

    protected void setOff()
    {
        this.isOn = false;
    }

    protected void setOn()
    {
        this.isOn = true;
    }

    public boolean isBorder()
    {
        return this.isBorder;
    }

    public void setBorder(boolean isBorder)
    {
        this.isBorder = isBorder;
    }

    public String getColor()
    {
        return this.color;
    }

    public int getPredecessor()
    {
        return this.predecessor;
    }

    public void setColor(String color)
    {
        this.color = color;
    }

    public void setPredecessor(int predecessor)
    {
        this.predecessor = predecessor;
    }

    public int getLoop()
    {
        return this.loop;
    }

    public void setLoop(int loop)
    {
        this.loop = loop;
    }

    public int getDist()
    {
        return this.dist;
    }

    public void setDist(int dist)
    {
        this.dist = dist;
    }

    public int getComponent()
    {
        return this.component;
    }

    public void setComponent(int c)
    {
        this.component = c;
    }

    public String toString()
    {
        return "sensor" + id + " (" + x + "," + y + ") isOn=" + isOn;
    }

    public int hashCode()
    {
        return this.toString().hashCode();
    }

    public boolean equals(Object o)
    {
        if (o != null)
            return this.hashCode() == o.hashCode();

        return false;
    }
}
