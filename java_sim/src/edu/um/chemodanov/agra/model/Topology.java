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
import edu.um.chemodanov.agra.util.EuclDist;

import java.util.*;

public class Topology implements BFSConstant {
    private int size;
    private int nodesNum;
    private List<Node> nodes;
    private double pr = 0.95; //connectivity probability of two nodes in reciprocal radio range (i.e., to simulate asymmetrical links)
    private List<Obstacle> obstacles; //list of all complex concave obstacles
    private Set<Node> switchedOffNodes;
    private List<Obstacle> circumscribedObstacles; // list of circumscribed circles for complex obstacles (shape approximation)
    private Random rand = new Random();

    public Topology(int size, int nodesNum) {
        this.size = size;
        this.nodesNum = nodesNum;
        this.nodes = new ArrayList<>(nodesNum);
        this.switchedOffNodes = new HashSet<>();
        Random r = new Random();

        for (int i = 0; i < nodesNum; i++) {
            Node n = new Node(i, r.nextInt(size + 1), r.nextInt(size + 1));
            n.setR(0.1 * size);
            this.nodes.add(n);
        }

        this.obstacles = new ArrayList<>();
        this.circumscribedObstacles = new ArrayList<>();
    }

    public Topology(int size) {
        this.size = size;
        this.nodes = new ArrayList<>();
        this.switchedOffNodes = new HashSet<>();
        for (int i = 1; i <= size; i++)
            for (int j = 1; j <= size; j++) {
                Node n = new Node(nodes.size(), i, j);
                n.setR(0.1 * size);
                this.nodes.add(n);
            }

        this.nodesNum = nodes.size();
        this.obstacles = new ArrayList<>();
        this.circumscribedObstacles = new ArrayList<>();
    }

    public int getGridSize() {
        return this.size;
    }

    public List<Node> getNodes() {
        return this.nodes;
    }

    public Set<Node> getSwitchedOffNodes() {
        return this.switchedOffNodes;
    }

    public List<Obstacle> getObstacles() {
        return this.obstacles;
    }

    /**
     * Method to generate several circular obstacles that can overlap and create obstacles of complex concave shapes
     *
     * @param numObstacles - number of simple obstacles to generate
     * @param minR         - min obstacle radius
     * @param maxR         - max obstacle radius
     */
    public void generateObstacles(int numObstacles, double minR, double maxR) {
        Random r = new Random();
        int maxK = 100 * numObstacles;
        int k = 0;
        while (obstacles.size() != numObstacles && k < maxK) {
            double x = r.nextDouble() * size;// + 1;
            double y = r.nextDouble() * size;// + 1;
            double hr = minR + r.nextDouble() * (maxR - minR);
            if (x - hr > 1 && x + hr < size && y - hr > 1 && y + hr < size) {
                Obstacle o = new Obstacle(obstacles.size(), x, y, hr);
                obstacles.add(o);
            }
            k++;
        }
        downNodesWithinObstacle();
    }

    /**
     * Method to generate a single and simple circular obstacle that can overlap and create obstacles of complex concave shapes
     *
     * @param x - x coordinate of the obstacle center
     * @param y - y coordinate of the obstacle center
     * @param r - the obstacle's radius
     */
    public void generateObstacle(int x, int y, double r) {
        Obstacle o = new Obstacle(obstacles.size(), x, y, r);
        obstacles.add(o);

        downNodesWithinObstacle();
    }

    private void downNodesWithinObstacle() {
        for (Node n : nodes)
            for (Obstacle o : obstacles)
                if (EuclDist.d(n.getX(), n.getY(), o.getX(), o.getY()) <= o.getR()) {
                    n.setOff();
                    switchedOffNodes.add(n);
                }
    }

    public void reInitializeNeighbors() {
        for (Node n : nodes)
            if (n.isOn()) {
                List<Set<Node>> neighborsList = getNeighbors(n);
                n.setStaticNeighbors(neighborsList.get(0));
                n.setNeighbors(neighborsList.get(1));
            }
    }

    public void clearAllObstacles() {
        this.obstacles = new ArrayList<>();
        this.switchedOffNodes.clear();

        for (Node n : nodes) {
            n.setOn();
            n.setBorder(false);
        }
        reInitializeNeighbors();
    }

    private List<Set<Node>> getNeighbors(Node src) {
        Set<Node> staticNeighbors = new HashSet<>();
        Set<Node> neighbors = new HashSet<>();

        for (Node n : nodes)
            if (!n.equals(src) && n.isOn()
                    && EuclDist.d(src.getX(), src.getY(), n.getX(), n.getY()) <= src.getR()) {
                staticNeighbors.add(n);
                if (rand.nextDouble() <= pr)
                    neighbors.add(n);
            }

        List<Set<Node>> neighborsList = new ArrayList<>(2);
        neighborsList.add(staticNeighbors);
        neighborsList.add(neighbors);
        return neighborsList;
    }

    /**
     * @return list of obstacles of approximated shapes (using Equations 11 and 12 in AGRA paper)
     */
    public List<Obstacle> getCircumscribedObstacles() {
        return this.circumscribedObstacles;
    }

    /**
     * Method to inscribe of complex concave obstacles to circles (shape approximation)
     * Using Equations 11 and 12 in AGRA paper
     */
    public void initializeCircumscribedObstacles() {
        circumscribedObstacles = new ArrayList<>();
        double borderR = 1;
        List<Set<Node>> connectedNodes = findBorderNodes(borderR);

        //find simple and complex obstacles
        for (Set<Node> borderNodes : connectedNodes) {
            Set<Node> loops = new HashSet<>();
            estimateLoopsDistWithBFS(borderNodes); //find loops
            for (Node n : borderNodes) {

                if (n.getLoop() != NIL)
                    loops.add(n);
            }

            if (!loops.isEmpty())
                circumscribedObstacles.add(createCircumscribedObstacle(borderNodes));
        }
    }

    private List<Set<Node>> findBorderNodes(double borderR) {
        Map<Node, Set<Node>> neighbors = new HashMap<>();

        //estimate switched off neighbors
        for (Node n : switchedOffNodes)
            for (Node offS : switchedOffNodes)
                if (EuclDist.d(n.getX(), n.getY(), offS.getX(), offS.getY()) <= 1 && !n.equals(offS)) {
                    if (neighbors.containsKey(n))
                        neighbors.get(n).add(offS);
                    else {
                        Set<Node> sNeighbors = new HashSet<>();
                        sNeighbors.add(offS);
                        neighbors.put(n, sNeighbors);
                    }
                }

        //detecting connected components
        estimateOffComponents(switchedOffNodes, neighbors);
        Map<Integer, Set<Node>> connectedOffNodes = new HashMap<>();
        for (Node n : switchedOffNodes) {
            int c = n.getComponent();
            if (connectedOffNodes.containsKey(c))
                connectedOffNodes.get(c).add(n);
            else {
                Set<Node> interConnected = new HashSet<>();
                interConnected.add(n);
                connectedOffNodes.put(c, interConnected);
            }
        }

        //find border nodes for connected components
        List<Set<Node>> connectedNodes = new ArrayList<>(connectedOffNodes.size());
        for (Set<Node> offNodes : connectedOffNodes.values()) {
            Set<Node> borderNodes = new HashSet<>();
            for (Node n : nodes)
                if (n.isOn())
                    for (Node offS : offNodes)
                        if (EuclDist.d(n.getX(), n.getY(), offS.getX(), offS.getY()) <= borderR) {
                            borderNodes.add(n);
                            n.setBorder(true);
                        }
            connectedNodes.add(borderNodes);
        }

        return connectedNodes;
    }

    private void estimateOffComponents(Set<Node> offNodes, Map<Node, Set<Node>> neighbors) {
        for (Node n : offNodes) {
            n.setColor(WHITE);
            n.setComponent(0);
        }
        int comp = 0;
        for (Node n : offNodes)
            if (n.getColor().equals(WHITE)) {
                comp += 1;
                findOffComponentWithBFS(n, comp, neighbors);
            }
    }

    private void findOffComponentWithBFS(Node src, int comp, Map<Node, Set<Node>> neighbors) {
        Queue<Node> q = new LinkedList<>();
        src.setColor(GRAY);
        q.add(src);
        while (!q.isEmpty()) {
            Node n = q.poll();
            n.setColor(BLACK);
            n.setComponent(comp); //set a component
            for (Node neighbor : neighbors.get(n))
                if (neighbor.getColor().equals(WHITE)) {
                    neighbor.setColor(GRAY);
                    neighbor.setPredecessor(n.getId());
                    neighbor.setDist(n.getDist() + 1);
                    q.add(neighbor);
                }
        }
    }

    private void estimateLoopsDistWithBFS(Set<Node> connectedBorderNodes) {
        for (Node n : connectedBorderNodes) {
            n.setColor(WHITE);
            n.setPredecessor(NIL);
            n.setLoop(NIL);
            n.setDist(0);
        }

        if (!connectedBorderNodes.isEmpty()) {
            Node src = connectedBorderNodes.iterator().next();

            Queue<Node> q = new LinkedList<>();
            src.setColor(GRAY);
            q.add(src);
            Set<Node> loopNodes = new HashSet<>();
            while (!q.isEmpty()) {
                Node n = q.poll();
                n.setColor(BLACK);
                for (Node neighbor : n.getStaticNeighbors())
                    if (connectedBorderNodes.contains(neighbor))
                        if (neighbor.getColor().equals(WHITE)) {
                            neighbor.setColor(GRAY);
                            neighbor.setPredecessor(n.getId());
                            neighbor.setDist(n.getDist() + 1);
                            q.add(neighbor);
                        } else if (neighbor.getColor().equals(GRAY) &&
                                n.getPredecessor() != neighbor.getPredecessor() &&
                                !nodes.get(neighbor.getPredecessor()).getStaticNeighbors().contains(nodes.get(n.getPredecessor())) &&
                                !nodes.get(neighbor.getPredecessor()).getStaticNeighbors().contains(n)) // cycleDetected
                        {
                            if (loopNodes.isEmpty()) {
                                loopNodes.add(neighbor);
                                neighbor.setLoop(n.getId());
                            } else {
                                boolean valid = true;
                                for (Node loopS : loopNodes)
                                    if (loopNodes.contains(neighbor) ||
                                            loopS.getStaticNeighbors().contains(neighbor) ||
                                            loopS.getStaticNeighbors().contains(n))
                                        valid = false;

                                if (valid) {
                                    loopNodes.add(neighbor);
                                    neighbor.setLoop(n.getId());
                                }
                            }
                        }
            }
        }
    }

    private Obstacle createCircumscribedObstacle(Set<Node> borderNodes) {
        if (!borderNodes.isEmpty()) {
            double x = 0;
            double y = 0;
            for (Node n : borderNodes) {
                x += n.getX();
                y += n.getY();
            }

            x = x / borderNodes.size();
            y = y / borderNodes.size();

            double max = 0;
            for (Node n : borderNodes)
                if (EuclDist.d(n.getX(), n.getY(), x, y) > max)
                    max = EuclDist.d(n.getX(), n.getY(), x, y);
            return new Obstacle(circumscribedObstacles.size(), x, y, max);
        } else return null;
    }

    public String toString() {
        StringBuilder str = new StringBuilder();
        str.append("Topology: size=").append(size).append("x")
                .append(size).append(", nodes[\n");
        for (int i = 0; i < nodesNum; i++) {
            Node s = nodes.get(i);
            str.append(s).append("\n");
        }
        str.append("] obstacles[\n");
        for (int i = 0; i < obstacles.size(); i++) {
            Obstacle h = obstacles.get(i);
            str.append(h).append("\n");
        }
        return str.toString();
    }
}
