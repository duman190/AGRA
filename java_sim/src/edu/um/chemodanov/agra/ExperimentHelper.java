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

import edu.um.chemodanov.agra.forwarding.*;
import edu.um.chemodanov.agra.model.Node;
import edu.um.chemodanov.agra.model.Obstacle;
import edu.um.chemodanov.agra.model.Topology;
import edu.um.chemodanov.agra.routing.BFS;
import edu.um.chemodanov.agra.util.EuclDist;
import javafx.util.Pair;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.*;

public class ExperimentHelper {
    private static Random rand = new Random();

    /**
     * Experiment to detect the best repulsive field attenuation degree
     * over fixed network topology configuration
     *
     * @param trials       - number of trials
     * @param pairNum      - number of src-dst pairs
     * @param numObstacles - number of numObstacles to generate
     * @param deg          - array of different repulsive field attenuation degrees (for both local ARGF and ARPGF)
     */
    public void doExperiment1(int trials, int pairNum, int numObstacles, double[] deg) {
        System.out.println("Experiment 1 (attenuation degree tuning) has been started!");
        BFS bfs = new BFS();
        ARGF_Global argfGlobal = new ARGF_Global();
        ARGF_Local argfLocal = new ARGF_Local();
        ARPGF_Global arpgfGlobal = new ARPGF_Global();
        ARPGF_Local arpgfLocal = new ARPGF_Local();

        //conduct simulation
        int degSize = deg.length;
        int[] bfsPathNum = new int[trials];
        int[][] argfGPathNum = new int[degSize][trials];
        int[][] argfLPathNum = new int[degSize][trials];
        int[][] arpgfGPathNum = new int[degSize][trials];
        int[][] arpgfLPathNum = new int[degSize][trials];
        double[][] argfGPathStretch = new double[degSize][trials];
        double[][] argfLPathStretch = new double[degSize][trials];
        double[][] arpgfGPathStretch = new double[degSize][trials];
        double[][] arpgfLPathStretch = new double[degSize][trials];
        double[] coverage = new double[trials];
        int iter = 0;
        for (int j = 0; j < trials; j++) {
            System.out.println("Trial #" + j + " has been started!");
            Topology t = new Topology(100);
            for (Node n : t.getNodes())
                n.setR(rand.nextDouble() * 1 + 4);
            t.generateObstacles(numObstacles, 1, 10);
            System.out.println("Network and obstacles were created!");
            t.reInitializeNeighbors();
            System.out.println("Neighbors were initialized!");
            t.initializeCircumscribedObstacles();
            System.out.println("Circumscribed obstacles were created and they are:" + t.getCircumscribedObstacles());

            int maxLength = 100000; //arbitrary large number of hops!
            int size = t.getGridSize();

            //generate random pairs
            List<Integer> nodeIds = new ArrayList<>(size * size);
            for (Node n : t.getNodes())
                if (n.isOn())
                    nodeIds.add(n.getId());
            int nodesNum = nodeIds.size();
            System.out.println("Coverage ratio:" + Double.valueOf(nodesNum) / (size * size));
            Random r = new Random();
            List<Pair<Integer, Integer>> pairs = new ArrayList<>(size * size);
            while (pairs.size() < pairNum) {
                int src = nodeIds.get(r.nextInt(nodesNum));
                int dst = nodeIds.get(r.nextInt(nodesNum));

                if (src != dst)
                    pairs.add(new Pair<>(src, dst));
            }
            System.out.println("Pairs were generated. Total number of pairs=" + pairs.size());

            coverage[j] = Double.valueOf(nodesNum) / (size * size);
            System.out.println("Coverage ratio:" + coverage[j]);

            for (Pair<Integer, Integer> pair : pairs) {
                List<Node> pathBFS = bfs.breadthFirstSearch(pair.getKey(), pair.getValue(), t);

                if (!pathBFS.isEmpty()) {
                    bfsPathNum[j] += 1;

                    for (int i = 0; i < degSize; i++) {
                        double intDeg = deg[i];
                        ;
                        argfGlobal.setDeg(intDeg);
                        argfLocal.setDeg(intDeg);
                        arpgfGlobal.setDeg(intDeg);
                        arpgfLocal.setDeg(intDeg);
                        List<Node> pathARGFG = argfGlobal.potentialGreedyForwarding(pair.getKey(), pair.getValue(), t, maxLength);
                        List<Node> pathARGFL = argfLocal.potentialGreedyForwarding(pair.getKey(), pair.getValue(), t, maxLength);
                        List<Node> pathARPGFG = arpgfGlobal.potentialGreedyForwarding(pair.getKey(), pair.getValue(), t, maxLength);
                        List<Node> pathARPGFL = arpgfLocal.potentialGreedyForwarding(pair.getKey(), pair.getValue(), t, maxLength);
                        if (!pathARGFG.isEmpty() && pathARGFG.get(pathARGFG.size() - 1).getId() == pair.getValue()) {
                            argfGPathNum[i][j] += 1;
                            argfGPathStretch[i][j] += Double.valueOf(pathARGFG.size()) / Double.valueOf(pathBFS.size());
                        }
                        if (!pathARGFL.isEmpty() && pathARGFL.get(pathARGFL.size() - 1).getId() == pair.getValue()) {
                            argfLPathNum[i][j] += 1;
                            argfLPathStretch[i][j] += Double.valueOf(pathARGFL.size()) / Double.valueOf(pathBFS.size());
                        }
                        if (!pathARPGFG.isEmpty() && pathARPGFG.get(pathARPGFG.size() - 1).getId() == pair.getValue()) {
                            arpgfGPathNum[i][j] += 1;
                            arpgfGPathStretch[i][j] += Double.valueOf(pathARPGFG.size()) / Double.valueOf(pathBFS.size());
                        }
                        if (!pathARPGFL.isEmpty() && pathARPGFL.get(pathARPGFL.size() - 1).getId() == pair.getValue()) {
                            arpgfLPathNum[i][j] += 1;
                            arpgfLPathStretch[i][j] += Double.valueOf(pathARPGFL.size()) / Double.valueOf(pathBFS.size());
                        }
                    }
                }
                System.out.println("Trial #" + j + " with " + numObstacles + " numObstacles. All algorithms were used for pair[" + ++iter + "]. BFS found:" + bfsPathNum[j]);
            }
            for (int i = 0; i < degSize; i++) {
                argfGPathStretch[i][j] = argfGPathStretch[i][j] / argfGPathNum[i][j];
                argfLPathStretch[i][j] = argfLPathStretch[i][j] / argfLPathNum[i][j];
                arpgfGPathStretch[i][j] = arpgfGPathStretch[i][j] / arpgfGPathNum[i][j];
                arpgfLPathStretch[i][j] = arpgfLPathStretch[i][j] / arpgfLPathNum[i][j];
            }
        }
        System.out.println("Coverage ratio:" + Arrays.toString(coverage));
        System.out.println("BFS path num=" + Arrays.toString(bfsPathNum));
        System.out.println("ARGF G path num=" + Arrays.deepToString(argfGPathNum));
        System.out.println("ARGF L path num=" + Arrays.deepToString(argfLPathNum));
        System.out.println("ARPGF G path num=" + Arrays.deepToString(arpgfGPathNum));
        System.out.println("ARPGF L path num=" + Arrays.deepToString(arpgfLPathNum));
        System.out.println("ARGF G avg. path stretch=" + Arrays.deepToString(argfGPathStretch));
        System.out.println("ARGF L avg. path stretch=" + Arrays.deepToString(argfLPathStretch));
        System.out.println("ARPGF G avg. path stretch=" + Arrays.deepToString(arpgfGPathStretch));
        System.out.println("ARPGF L avg. path stretch=" + Arrays.deepToString(arpgfLPathStretch));
    }

    /**
     * Experiment with various number of obstacles to stress out local obstacle information overhead
     *
     * @param trials       - number of trials
     * @param numObstacles - array of different obstacle numbers to simulate different obstacle occupation scenarios
     * @param deg          - fixed repulsive field attenuation order degree (only for ARGF)
     */
    public void doExperiment2(int trials, int[] numObstacles, double deg) {
        int oSize = numObstacles.length;

        double[][] argfObstacleNum = new double[oSize][trials];
        double[][] arpgfObstacleNum = new double[oSize][trials];


        for (int i = 0; i < trials; i++) {
            System.out.println("Trial #" + (i + 1) + " has been started!");
            for (int j = 0; j < oSize; j++) {
                System.out.println("Number of obstacles " + numObstacles[j] + " ...");
                Topology t = new Topology(100);
                for (Node n : t.getNodes())
                    n.setR(rand.nextDouble() * 1 + 4);
                t.generateObstacles(numObstacles[j], 1, 10);
                System.out.println("Network and obstacles were created!");
                t.reInitializeNeighbors();
                System.out.println("Neighbors were initialized!");
                t.initializeCircumscribedObstacles();
                List<Obstacle> obstacles = t.getCircumscribedObstacles();
                System.out.println(t.getCircumscribedObstacles().size() + " circumscribed obstacles were created.");

                //calculate local obstacle information overhead
                int nodesNum = 0;
                for (Node n : t.getNodes())
                    if (n.isOn()) {
                        nodesNum++;
                        argfObstacleNum[j][i] += inObstacleRepulseZone(n, obstacles, n.getR(), deg).size();
                        arpgfObstacleNum[j][i] += inObstacleRepulseZone(n, obstacles, n.getR(), 1).size();
                    }

                if (numObstacles[j] == 50) { //store info for 40 % of occupation
                    List<Integer> argfHoleNumList = new ArrayList<>(nodesNum);
                    List<Integer> arpgfHoleNumList = new ArrayList<>(nodesNum);
                    for (Node n : t.getNodes())
                        if (n.isOn()) {
                            argfHoleNumList.add(inObstacleRepulseZone(n, obstacles, n.getR(), deg).size());
                            arpgfHoleNumList.add(inObstacleRepulseZone(n, obstacles, n.getR(), 1).size());
                        }
                    saveDataToFile("results/ARGF_obstacleNum_"+ numObstacles[j] +"_trial_" + i + ".txt", argfHoleNumList);
                    saveDataToFile("results/ARPGF_obstacleNum_"+ numObstacles[j] +"_trial_" + i + ".txt", arpgfHoleNumList);
                }

                argfObstacleNum[j][i] = argfObstacleNum[j][i] / nodesNum;
                arpgfObstacleNum[j][i] = arpgfObstacleNum[j][i] / nodesNum;
            }
        }
        System.out.println("ARGF avg. num of obstacles=" + Arrays.deepToString(argfObstacleNum));
        System.out.println("ARPGF avg. num of obstacles=" + Arrays.deepToString(arpgfObstacleNum));
    }

    /**
     * Experiment with various number of obstacles to simulate various obstacle occupation scenarios
     *
     * @param trials       - number of trials
     * @param pairNum      - number of src-dst pairs
     * @param numObstacles - array of obstacle numbers
     * @param deg          - fixed repulsive field attenuation order degree (only for ARGF)
     */
    public void doExperiment3(int trials, int pairNum, int[] numObstacles, double deg) {
        BFS bfs = new BFS();
        GF gf = new GF();
        GPGF gpgf = new GPGF();
        ARGF_Global argfGlobal = new ARGF_Global();
        argfGlobal.setDeg(deg);
        ARGF_Local argfLocal = new ARGF_Local();
        argfLocal.setDeg(deg);
        ARPGF_Local arpgfLocal = new ARPGF_Local();
        arpgfLocal.setDeg(1);
        ARPSR arpsr = new ARPSR();
        arpsr.setDeg(deg);
        int ttl = 100000; //unrestricted TTL

        int[][] bfsPathNum = new int[numObstacles.length][trials];
        double[][] avgDeg = new double[numObstacles.length][trials];
        int[][] gfPathNum = new int[numObstacles.length][trials];
        int[][] gpsrPathNum = new int[numObstacles.length][trials];
        int[][] gpgfPathNum = new int[numObstacles.length][trials];
        int[][] argfGPathNum = new int[numObstacles.length][trials];
        int[][] argfLPathNum = new int[numObstacles.length][trials];
        int[][] arpgfLPathNum = new int[numObstacles.length][trials];
        int[][] arpsrLPathNum = new int[numObstacles.length][trials];
        double[][] arpgfLHeaderSize = new double[numObstacles.length][trials];
        double[][] gpgfHeaderSize = new double[numObstacles.length][trials];
        double[][] gfPathStretch = new double[numObstacles.length][trials];
        double[][] gpsrPathStretch = new double[numObstacles.length][trials];
        double[][] gpgfPathStretch = new double[numObstacles.length][trials];
        double[][] argfGPathStretch = new double[numObstacles.length][trials];
        double[][] argfLPathStretch = new double[numObstacles.length][trials];
        double[][] arpgfLPathStretch = new double[numObstacles.length][trials];
        double[][] arpsrPathStretch = new double[numObstacles.length][trials];
        double[][] coverageRatio = new double[numObstacles.length][trials];

        int iter = 0;
        for (int i = 0; i < trials; i++) {
            System.out.println("Trial #" + (i + 1) + " has been started!");
            Topology t = new Topology(100);
            for (Node n : t.getNodes())
                n.setR(rand.nextDouble() * 1 + 4);

            for (int j = 0; j < numObstacles.length; j++) {
                t.generateObstacles(numObstacles[j], 1, 10);
                System.out.println("Network and obstacles were created!");
                t.reInitializeNeighbors();
                System.out.println("Neighbors were initialized!");
                //limit edges to build planarized graph (RNG)
                for (Node n : t.getNodes())
                    if (n.isOn()) {
                        n.clearProhibitedNeighbors();
                        Node u = n;
                        for (Node v : u.getNeighbors())
                            for (Node w : u.getNeighbors())
                                if (w.equals(v))
                                    continue;
                                else if (EuclDist.d(u, v) > Math.max(EuclDist.d(u, w), EuclDist.d(v, w))) {
                                    u.addProhibitedNeighbor(v);
                                    break;
                                }

                    }
                System.out.println("Planarized RNG graph was created!");
                t.initializeCircumscribedObstacles();
                System.out.println("Circumscribed obstacles were created and they are:" + t.getCircumscribedObstacles());

                int maxLengthGF = t.getNodes().size() - t.getSwitchedOffNodes().size();
                int totalDegree = 0;
                for (Node s : t.getNodes())
                    if (s.isOn())
                        totalDegree += s.getNeighbors().size();
                avgDeg[j][i] = Double.valueOf(totalDegree) / Double.valueOf(maxLengthGF);
                int size = t.getGridSize();

                // generate random pairs
                List<Integer> nodeIds = new ArrayList<>(size * size);
                for (Node s : t.getNodes())
                    if (s.isOn())
                        nodeIds.add(s.getId());
                int nodesNum = nodeIds.size();
                System.out.println("Coverage ratio:" + Double.valueOf(nodesNum) / (size * size));
                Random r = new Random();
                List<Pair<Integer, Integer>> pairs = new ArrayList<>(size * size);
                while (pairs.size() < pairNum) {
                    int src = nodeIds.get(r.nextInt(nodesNum));
                    int dst = nodeIds.get(r.nextInt(nodesNum));

                    if (src != dst) {
                        pairs.add(new Pair<>(src, dst));
                    }
                }
                System.out.println("Pairs were generated. Total number of pairs=" + pairs.size());

//        conduct simulation
                for (Pair<Integer, Integer> pair : pairs) {

                    List<Node> pathBFS = bfs.breadthFirstSearch(pair.getKey(), pair.getValue(), t);
                    if (!pathBFS.isEmpty() && pathBFS.size() - 1 <= ttl) {
                        bfsPathNum[j][i] += 1;
                        List<Node> pathGF = gf.greedyForwarding(pair.getKey(), pair.getValue(), t, false, ttl + 1);
                        List<Node> pathGPSR = gf.greedyForwarding(pair.getKey(), pair.getValue(), t, true, ttl + 1);
                        List<Integer> gpgfHSize = new ArrayList<>(1);
                        List<Node> pathGPGF = gpgf.greedyForwarding(pair.getKey(), pair.getValue(), t, ttl + 1, gpgfHSize);
                        List<Node> pathARGFG = argfGlobal.potentialGreedyForwarding(pair.getKey(), pair.getValue(), t, ttl + 1);
                        List<Node> pathARGFL = argfLocal.potentialGreedyForwarding(pair.getKey(), pair.getValue(), t, ttl + 1);
                        List<Integer> arpgfHSize = new ArrayList<>(1);
                        List<Node> pathARPGF = arpgfLocal.potentialGreedyForwarding(pair.getKey(), pair.getValue(), t, ttl + 1, arpgfHSize);
                        List<Node> pathARPSR = arpsr.potentialGreedyForwarding(pair.getKey(), pair.getValue(), t, ttl + 1);

                        if (!pathGF.isEmpty() && pathGF.get(pathGF.size() - 1).getId() == pair.getValue()) {
                            gfPathNum[j][i] += 1;
                            gfPathStretch[j][i] += Double.valueOf(pathGF.size()) / Double.valueOf(pathBFS.size());
                        }

                        if (!pathGPSR.isEmpty() && pathGPSR.get(pathGPSR.size() - 1).getId() == pair.getValue()) {
                            gpsrPathNum[j][i] += 1;
                            gpsrPathStretch[j][i] += Double.valueOf(pathGPSR.size()) / Double.valueOf(pathBFS.size());
                        }

                        if (!pathGPGF.isEmpty() && pathGPGF.get(pathGPGF.size() - 1).getId() == pair.getValue()) {
                            gpgfPathNum[j][i] += 1;
                            gpgfPathStretch[j][i] += Double.valueOf(pathGPGF.size()) / Double.valueOf(pathBFS.size());
                            gpgfHeaderSize[j][i] += gpgfHSize.isEmpty() ? 0 : gpgfHSize.get(0);
                        }

                        if (!pathARGFG.isEmpty() && pathARGFG.get(pathARGFG.size() - 1).getId() == pair.getValue()) {
                            argfGPathNum[j][i] += 1;
                            argfGPathStretch[j][i] += Double.valueOf(pathARGFG.size()) / Double.valueOf(pathBFS.size());
                        }

                        if (!pathARGFL.isEmpty() && pathARGFL.get(pathARGFL.size() - 1).getId() == pair.getValue()) {
                            argfLPathNum[j][i] += 1;
                            argfLPathStretch[j][i] += Double.valueOf(pathARGFL.size()) / Double.valueOf(pathBFS.size());
                        }

                        if (!pathARPGF.isEmpty() && pathARPGF.get(pathARPGF.size() - 1).getId() == pair.getValue()) {
                            arpgfLPathNum[j][i] += 1;
                            arpgfLPathStretch[j][i] += Double.valueOf(pathARPGF.size()) / Double.valueOf(pathBFS.size());
                            arpgfLHeaderSize[j][i] += arpgfHSize.isEmpty() ? 0 : arpgfHSize.get(0);
                        }

                        if (!pathARPSR.isEmpty() && pathARPSR.get(pathARPSR.size() - 1).getId() == pair.getValue()) {
                            arpsrLPathNum[j][i] += 1;
                            arpsrPathStretch[j][i] += Double.valueOf(pathARPSR.size()) / Double.valueOf(pathBFS.size());
                        }
                    }

                    System.out.println("All algorithms were used for pair[" + ++iter + "].");
                }
                coverageRatio[j][i] = Double.valueOf(nodesNum) / (size * size);
            }
            for (int j = 0; j < numObstacles.length; j++) {
                gfPathStretch[j][i] = gfPathStretch[j][i] / gfPathNum[j][i];
                gpsrPathStretch[j][i] = gpsrPathStretch[j][i] / gpsrPathNum[j][i];
                gpgfPathStretch[j][i] = gpgfPathStretch[j][i] / gpgfPathNum[j][i];
                argfGPathStretch[j][i] = argfGPathStretch[j][i] / argfGPathNum[j][i];
                argfLPathStretch[j][i] = argfLPathStretch[j][i] / argfLPathNum[j][i];
                arpgfLPathStretch[j][i] = arpgfLPathStretch[j][i] / arpgfLPathNum[j][i];
                arpsrPathStretch[j][i] = arpsrPathStretch[j][i] / arpsrLPathNum[j][i];
                //packet header sizes
                gpgfHeaderSize[j][i] = Double.valueOf(gpgfHeaderSize[j][i]) / Double.valueOf(gpgfPathNum[j][i]);
                arpgfLHeaderSize[j][i] = Double.valueOf(arpgfLHeaderSize[j][i]) / Double.valueOf(arpgfLPathNum[j][i]);
            }
        }

        System.out.println("Coverage ratio:" + Arrays.deepToString(coverageRatio));
        System.out.println("Avg. node degree:" + Arrays.deepToString(avgDeg));
        System.out.println("BFS path trials=" + Arrays.deepToString(bfsPathNum));
        System.out.println("GF path trials=" + Arrays.deepToString(gfPathNum));
        System.out.println("GPSR path trials=" + Arrays.deepToString(gpsrPathNum));
        System.out.println("GPGF path trials=" + Arrays.deepToString(gpgfPathNum));
        System.out.println("ARGF_G path trials=" + Arrays.deepToString(argfGPathNum));
        System.out.println("ARGF_L path trials=" + Arrays.deepToString(argfLPathNum));
        System.out.println("ARPGF path trials=" + Arrays.deepToString(arpgfLPathNum));
        System.out.println("ARPSR path trials=" + Arrays.deepToString(arpsrLPathNum));
        System.out.println("GF avg. path stretch=" + Arrays.deepToString(gfPathStretch));
        System.out.println("GPSR avg. path stretch=" + Arrays.deepToString(gpsrPathStretch));
        System.out.println("GPGF avg. path stretch=" + Arrays.deepToString(gpgfPathStretch));
        System.out.println("ARGF G avg. path stretch=" + Arrays.deepToString(argfGPathStretch));
        System.out.println("ARGF L avg. path stretch=" + Arrays.deepToString(argfLPathStretch));
        System.out.println("ARPGF avg. path stretch=" + Arrays.deepToString(arpgfLPathStretch));
        System.out.println("ARPSR avg. path stretch=" + Arrays.deepToString(arpsrPathStretch));

        System.out.println("GPGF packet header size=" + Arrays.deepToString(gpgfHeaderSize));
        System.out.println("ARPGF packet header size=" + Arrays.deepToString(arpgfLHeaderSize));
    }

    /**
     * Experiment with path length limitation based on TTL over configuration with fixed coverage ratio
     *
     * @param trials       - number of trials
     * @param pairNum      - number of src-dst pairs
     * @param numObstacles - number of obstacles
     * @param deg          - fixed repulsive field attenuation order degree (only for ARGF)
     * @param ttl          - array of different packet's TTL policies
     */
    public void doExperiment4(int trials, int pairNum, int numObstacles, double deg, int[] ttl) {
        BFS bfs = new BFS();
        GF gf = new GF();
        GPGF gpgf = new GPGF();
        ARGF_Global argfGlobal = new ARGF_Global();
        argfGlobal.setDeg(deg);
        ARGF_Local argfLocal = new ARGF_Local();
        argfLocal.setDeg(deg);
        ARPGF_Local arpgfLocal = new ARPGF_Local();
        arpgfLocal.setDeg(1);
        ARPSR arpsr = new ARPSR();
        arpsr.setDeg(deg);

        int[][] bfsPathNum = new int[ttl.length][trials];
        double[] avgDeg = new double[trials];
        int[][] gfPathNum = new int[ttl.length][trials];
        int[][] gpsrPathNum = new int[ttl.length][trials];
        int[][] gpgfPathNum = new int[ttl.length][trials];
        int[][] argfGPathNum = new int[ttl.length][trials];
        int[][] argfLPathNum = new int[ttl.length][trials];
        int[][] arpgfLPathNum = new int[ttl.length][trials];
        int[][] arpsrLPathNum = new int[ttl.length][trials];
        double[][] arpgfLHeaderSize = new double[ttl.length][trials];
        double[][] gpgfHeaderSize = new double[ttl.length][trials];
        double[][] gfPathStretch = new double[ttl.length][trials];
        double[][] gpsrPathStretch = new double[ttl.length][trials];
        double[][] gpgfPathStretch = new double[ttl.length][trials];
        double[][] argfGPathStretch = new double[ttl.length][trials];
        double[][] argfLPathStretch = new double[ttl.length][trials];
        double[][] arpgfLPathStretch = new double[ttl.length][trials];
        double[][] arpsrPathStretch = new double[ttl.length][trials];
        double[] coverageRatio = new double[trials];

        int iter = 0;
        for (int i = 0; i < trials; i++) {
            System.out.println("Trial #" + (i + 1) + " has been started!");
            Topology t = new Topology(100);
            for (Node n : t.getNodes())
                n.setR(rand.nextDouble() * 1 + 4);
            t.generateObstacles(numObstacles, 1, 10);
            System.out.println("Network and obstacles were created!");
            t.reInitializeNeighbors();
            System.out.println("Neighbors were initialized!");
            //limit edges to build planarized graph (RNG)
            for (Node n : t.getNodes())
                if (n.isOn()) {
                    n.clearProhibitedNeighbors();
                    Node u = n;
                    for (Node v : u.getNeighbors())
                        for (Node w : u.getNeighbors())
                            if (w.equals(v))
                                continue;
                            else if (EuclDist.d(u, v) > Math.max(EuclDist.d(u, w), EuclDist.d(v, w))) {
                                u.addProhibitedNeighbor(v);
                                break;
                            }

                }
            System.out.println("Planarized RNG graph was created!");
            t.initializeCircumscribedObstacles();
            System.out.println("Circumscribed obstacles were created and they are:" + t.getCircumscribedObstacles());

            int maxLengthGF = t.getNodes().size() - t.getSwitchedOffNodes().size();
            int totalDegree = 0;
            for (Node s : t.getNodes())
                if (s.isOn())
                    totalDegree += s.getNeighbors().size();
            avgDeg[i] = Double.valueOf(totalDegree) / Double.valueOf(maxLengthGF);
            int size = t.getGridSize();

//        generate random pairs
            List<Integer> nodeIds = new ArrayList<>(size * size);
            for (Node s : t.getNodes())
                if (s.isOn())
                    nodeIds.add(s.getId());
            int nodesNum = nodeIds.size();
            System.out.println("Coverage ratio:" + Double.valueOf(nodesNum) / (size * size));
            Random r = new Random();
            List<Pair<Integer, Integer>> pairs = new ArrayList<>(size * size);
            while (pairs.size() < pairNum) {
                int src = nodeIds.get(r.nextInt(nodesNum));
                int dst = nodeIds.get(r.nextInt(nodesNum));

                if (src != dst) {
                    pairs.add(new Pair<>(src, dst));
                }
            }
            System.out.println("Pairs were generated. Total number of pairs=" + pairs.size());

//        conduct simulation
            for (Pair<Integer, Integer> pair : pairs) {
                for (int j = 0; j < ttl.length; j++) {
                    List<Node> pathBFS = bfs.breadthFirstSearch(pair.getKey(), pair.getValue(), t);
                    if (!pathBFS.isEmpty() && pathBFS.size() - 1 <= ttl[j]) {
                        bfsPathNum[j][i] += 1;
                        List<Node> pathGF = gf.greedyForwarding(pair.getKey(), pair.getValue(), t, false, ttl[j] + 1);
                        List<Node> pathGPSR = gf.greedyForwarding(pair.getKey(), pair.getValue(), t, true, ttl[j] + 1);
                        List<Integer> gpgfHSize = new ArrayList<>(1);
                        List<Node> pathGPGF = gpgf.greedyForwarding(pair.getKey(), pair.getValue(), t, ttl[j] + 1, gpgfHSize);
                        List<Node> pathARGFG = argfGlobal.potentialGreedyForwarding(pair.getKey(), pair.getValue(), t, ttl[j] + 1);
                        List<Node> pathARGFL = argfLocal.potentialGreedyForwarding(pair.getKey(), pair.getValue(), t, ttl[j] + 1);
                        List<Integer> arpgfHSize = new ArrayList<>(1);
                        List<Node> pathARPGF = arpgfLocal.potentialGreedyForwarding(pair.getKey(), pair.getValue(), t, ttl[j] + 1, arpgfHSize);
                        List<Node> pathARPSR = arpsr.potentialGreedyForwarding(pair.getKey(), pair.getValue(), t, ttl[j] + 1);

                        if (!pathGF.isEmpty() && pathGF.get(pathGF.size() - 1).getId() == pair.getValue()) {
                            gfPathNum[j][i] += 1;
                            gfPathStretch[j][i] += Double.valueOf(pathGF.size()) / Double.valueOf(pathBFS.size());
                        }

                        if (!pathGPSR.isEmpty() && pathGPSR.get(pathGPSR.size() - 1).getId() == pair.getValue()) {
                            gpsrPathNum[j][i] += 1;
                            gpsrPathStretch[j][i] += Double.valueOf(pathGPSR.size()) / Double.valueOf(pathBFS.size());
                        }

                        if (!pathGPGF.isEmpty() && pathGPGF.get(pathGPGF.size() - 1).getId() == pair.getValue()) {
                            gpgfPathNum[j][i] += 1;
                            gpgfPathStretch[j][i] += Double.valueOf(pathGPGF.size()) / Double.valueOf(pathBFS.size());
                            gpgfHeaderSize[j][i] += gpgfHSize.isEmpty() ? 0 : gpgfHSize.get(0);
                        }

                        if (!pathARGFG.isEmpty() && pathARGFG.get(pathARGFG.size() - 1).getId() == pair.getValue()) {
                            argfGPathNum[j][i] += 1;
                            argfGPathStretch[j][i] += Double.valueOf(pathARGFG.size()) / Double.valueOf(pathBFS.size());
                        }

                        if (!pathARGFL.isEmpty() && pathARGFL.get(pathARGFL.size() - 1).getId() == pair.getValue()) {
                            argfLPathNum[j][i] += 1;
                            argfLPathStretch[j][i] += Double.valueOf(pathARGFL.size()) / Double.valueOf(pathBFS.size());
                        }

                        if (!pathARPGF.isEmpty() && pathARPGF.get(pathARPGF.size() - 1).getId() == pair.getValue()) {
                            arpgfLPathNum[j][i] += 1;
                            arpgfLPathStretch[j][i] += Double.valueOf(pathARPGF.size()) / Double.valueOf(pathBFS.size());
                            arpgfLHeaderSize[j][i] += arpgfHSize.isEmpty() ? 0 : arpgfHSize.get(0);
                        }

                        if (!pathARPSR.isEmpty() && pathARPSR.get(pathARPSR.size() - 1).getId() == pair.getValue()) {
                            arpsrLPathNum[j][i] += 1;
                            arpsrPathStretch[j][i] += Double.valueOf(pathARPSR.size()) / Double.valueOf(pathBFS.size());
                        }
                    }
                }
                System.out.println("All algorithms were used for pair[" + ++iter + "].");
            }
            coverageRatio[i] = Double.valueOf(nodesNum) / (size * size);

            for (int j = 0; j < ttl.length; j++) {
                gfPathStretch[j][i] = gfPathStretch[j][i] / gfPathNum[j][i];
                gpsrPathStretch[j][i] = gpsrPathStretch[j][i] / gpsrPathNum[j][i];
                gpgfPathStretch[j][i] = gpgfPathStretch[j][i] / gpgfPathNum[j][i];
                argfGPathStretch[j][i] = argfGPathStretch[j][i] / argfGPathNum[j][i];
                argfLPathStretch[j][i] = argfLPathStretch[j][i] / argfLPathNum[j][i];
                arpgfLPathStretch[j][i] = arpgfLPathStretch[j][i] / arpgfLPathNum[j][i];
                arpsrPathStretch[j][i] = arpsrPathStretch[j][i] / arpsrLPathNum[j][i];
                //packet header sizes
                gpgfHeaderSize[j][i] = Double.valueOf(gpgfHeaderSize[j][i]) / Double.valueOf(gpgfPathNum[j][i]);
                arpgfLHeaderSize[j][i] = Double.valueOf(arpgfLHeaderSize[j][i]) / Double.valueOf(arpgfLPathNum[j][i]);
            }
        }

        System.out.println("Coverage ratio:" + Arrays.toString(coverageRatio));
        System.out.println("Avg. node degree:" + Arrays.toString(avgDeg));
        System.out.println("BFS path trials=" + Arrays.deepToString(bfsPathNum));
        System.out.println("GF path trials=" + Arrays.deepToString(gfPathNum));
        System.out.println("GPSR path trials=" + Arrays.deepToString(gpsrPathNum));
        System.out.println("GPGF path trials=" + Arrays.deepToString(gpgfPathNum));
        System.out.println("ARGF_G path trials=" + Arrays.deepToString(argfGPathNum));
        System.out.println("ARGF_L path trials=" + Arrays.deepToString(argfLPathNum));
        System.out.println("ARPGF path trials=" + Arrays.deepToString(arpgfLPathNum));
        System.out.println("ARPSR path trials=" + Arrays.deepToString(arpsrLPathNum));
        System.out.println("GF avg. path stretch=" + Arrays.deepToString(gfPathStretch));
        System.out.println("GPSR avg. path stretch=" + Arrays.deepToString(gpsrPathStretch));
        System.out.println("GPGF avg. path stretch=" + Arrays.deepToString(gpgfPathStretch));
        System.out.println("ARGF G avg. path stretch=" + Arrays.deepToString(argfGPathStretch));
        System.out.println("ARGF L avg. path stretch=" + Arrays.deepToString(argfLPathStretch));
        System.out.println("ARPGF avg. path stretch=" + Arrays.deepToString(arpgfLPathStretch));
        System.out.println("ARPSR avg. path stretch=" + Arrays.deepToString(arpsrPathStretch));

        System.out.println("GPGF packet header size=" + Arrays.deepToString(gpgfHeaderSize));
        System.out.println("ARPGF packet header size=" + Arrays.deepToString(arpgfLHeaderSize));
    }

    /**
     * Experiment with radio range variation (to simulate different node degree) with the fixed TTL
     *
     * @param trials       - number of trials
     * @param pairNum      - number of src-dst pairs
     * @param numObstacles - number of obstacles
     * @param deg          - fixed repulsive field attenuation order degree (only for ARGF)
     * @param ttl          - fixed packet's TTL policy
     * @param maxR         - max radio range
     */
    public void doExperiment5(int trials, int pairNum, int numObstacles, double deg, int ttl, double maxR) {
        BFS bfs = new BFS();
        GF gf = new GF();
        GPGF gpgf = new GPGF();
        ARGF_Global argfGlobal = new ARGF_Global();
        argfGlobal.setDeg(deg);
        ARGF_Local argfLocal = new ARGF_Local();
        argfLocal.setDeg(deg);
        ARPGF_Local arpgfLocal = new ARPGF_Local();
        arpgfLocal.setDeg(1);
        ARPSR arpsr = new ARPSR();
        arpsr.setDeg(deg);

        int maxRows = 5;
        int[][] bfsPathNum = new int[maxRows][trials];
        double[][] avgDeg = new double[maxRows][trials];
        int[][] gfPathNum = new int[maxRows][trials];
        int[][] gpsrPathNum = new int[maxRows][trials];
        int[][] gpgfPathNum = new int[maxRows][trials];
        int[][] argfGPathNum = new int[maxRows][trials];
        int[][] argfLPathNum = new int[maxRows][trials];
        int[][] arpgfPathNum = new int[maxRows][trials];
        int[][] arpsrPathNum = new int[maxRows][trials];
        double[][] arpgfHeaderSize = new double[maxRows][trials];
        double[][] gpgfHeaderSize = new double[maxRows][trials];
        double[][] gfPathStretch = new double[maxRows][trials];
        double[][] gpsrPathStretch = new double[maxRows][trials];
        double[][] gpgfPathStretch = new double[maxRows][trials];
        double[][] argfGPathStretch = new double[maxRows][trials];
        double[][] argfLPathStretch = new double[maxRows][trials];
        double[][] arpgfPathStretch = new double[maxRows][trials];
        double[][] arpsrPathStretch = new double[maxRows][trials];
        double[] coverageRatio = new double[trials];

        int iter = 0;
        for (int i = 0; i < trials; i++) {
            System.out.println("Trial #" + (i + 1) + " has been started!");
            Topology t = new Topology(100);
            t.generateObstacles(numObstacles, 1, 10);
            System.out.println("Network and obstacles were created!");
            for (Node n : t.getNodes())
                n.setR(rand.nextDouble() * 4 + 1);
            t.reInitializeNeighbors();
            System.out.println("Neighbors were initialized!");

            int maxLengthGF = t.getNodes().size() - t.getSwitchedOffNodes().size();
            int size = t.getGridSize();

//        generate random pairs
            List<Integer> nodeIds = new ArrayList<>(size * size);
            for (Node n : t.getNodes())
                if (n.isOn())
                    nodeIds.add(n.getId());
            int nodesNum = nodeIds.size();
            System.out.println("Coverage ratio:" + Double.valueOf(nodesNum) / (size * size));
            Random r = new Random();
            List<Pair<Integer, Integer>> pairs = new ArrayList<>(size * size);
            while (pairs.size() < pairNum) {
                int src = nodeIds.get(r.nextInt(nodesNum));
                int dst = nodeIds.get(r.nextInt(nodesNum));

                if (src != dst)
                    pairs.add(new Pair<>(src, dst));
            }
            System.out.println("Pairs were generated. Total number of pairs=" + pairs.size());

//        conduct simulation
            int j = 0;
            for (int k = new Double(maxR).intValue(); k >= 1; k -= 1) {
                for (Node s : t.getNodes())
                    if (k > 1)
                        s.setR(k - rand.nextDouble());
                    else
                        s.setR(k);
                System.out.println("New radio range = " + k);

                //remove excess neighbors for radio range = k
                if (k != 5) {
                    for (Node s : t.getNodes())
                        if (s.isOn()) {
                            Set<Node> excessNeighbors = new HashSet<>();
                            for (Node nh : s.getNeighbors())
                                if (EuclDist.d(s.getX(), s.getY(), nh.getX(), nh.getY()) > k)
                                    excessNeighbors.add(nh);
                            s.getNeighbors().removeAll(excessNeighbors);
                        }
                    System.out.println("Neighbors were re-initialized!");
                }

                int totalDegree = 0;
                for (Node s : t.getNodes())
                    if (s.isOn())
                        totalDegree += s.getNeighbors().size();
                avgDeg[j][i] = Double.valueOf(totalDegree) / Double.valueOf(maxLengthGF);

                //limit edges to build planarized graph (RNG)
                for (Node s : t.getNodes())
                    if (s.isOn()) {
                        s.clearProhibitedNeighbors();
                        Node u = s;
                        for (Node v : u.getNeighbors())
                            for (Node w : u.getNeighbors())
                                if (w.equals(v))
                                    continue;
                                else if (EuclDist.d(u, v) > Math.max(EuclDist.d(u, w), EuclDist.d(v, w))) {
                                    u.addProhibitedNeighbor(v);
                                    break;
                                }

                    }
                System.out.println("Planarized RNG graph was created!");

                t.initializeCircumscribedObstacles();
                System.out.println("Circumscribed obstacles were created and they are:" + t.getCircumscribedObstacles());

                for (Pair<Integer, Integer> pair : pairs) {
                    List<Node> pathBFS = bfs.breadthFirstSearch(pair.getKey(), pair.getValue(), t);
                    if (!pathBFS.isEmpty() && pathBFS.size() <= ttl + 1) {
                        bfsPathNum[j][i] += 1;
                        List<Node> pathGF = gf.greedyForwarding(pair.getKey(), pair.getValue(), t, false, ttl + 1);
                        List<Node> pathGPSR = gf.greedyForwarding(pair.getKey(), pair.getValue(), t, true, ttl + 1);
                        List<Integer> gpgfHSize = new ArrayList<>(1);
                        List<Node> pathGPGF = gpgf.greedyForwarding(pair.getKey(), pair.getValue(), t, ttl + 1, gpgfHSize);
                        List<Node> pathARGFG = argfGlobal.potentialGreedyForwarding(pair.getKey(), pair.getValue(), t, ttl + 1);
                        List<Node> pathARGFL = argfLocal.potentialGreedyForwarding(pair.getKey(), pair.getValue(), t, ttl + 1);
                        List<Integer> arpgfHSize = new ArrayList<>(1);
                        List<Node> pathARPGF = arpgfLocal.potentialGreedyForwarding(pair.getKey(), pair.getValue(), t, ttl + 1, arpgfHSize);
                        List<Node> pathARPSR = arpsr.potentialGreedyForwarding(pair.getKey(), pair.getValue(), t, ttl + 1);

                        if (!pathGF.isEmpty() && pathGF.get(pathGF.size() - 1).getId() == pair.getValue()) {
                            gfPathNum[j][i] += 1;
                            gfPathStretch[j][i] += Double.valueOf(pathGF.size()) / Double.valueOf(pathBFS.size());
                        }

                        if (!pathGPSR.isEmpty() && pathGPSR.get(pathGPSR.size() - 1).getId() == pair.getValue()) {
                            gpsrPathNum[j][i] += 1;
                            gpsrPathStretch[j][i] += Double.valueOf(pathGPSR.size()) / Double.valueOf(pathBFS.size());
                        }

                        if (!pathGPGF.isEmpty() && pathGPGF.get(pathGPGF.size() - 1).getId() == pair.getValue()) {
                            gpgfPathNum[j][i] += 1;
                            gpgfPathStretch[j][i] += Double.valueOf(pathGPGF.size()) / Double.valueOf(pathBFS.size());
                            gpgfHeaderSize[j][i] += gpgfHSize.isEmpty() ? 0 : gpgfHSize.get(0);
                        }

                        if (!pathARGFG.isEmpty() && pathARGFG.get(pathARGFG.size() - 1).getId() == pair.getValue()) {
                            argfGPathNum[j][i] += 1;
                            argfGPathStretch[j][i] += Double.valueOf(pathARGFG.size()) / Double.valueOf(pathBFS.size());
                        }

                        if (!pathARGFL.isEmpty() && pathARGFL.get(pathARGFL.size() - 1).getId() == pair.getValue()) {
                            argfLPathNum[j][i] += 1;
                            argfLPathStretch[j][i] += Double.valueOf(pathARGFL.size()) / Double.valueOf(pathBFS.size());
                        }

                        if (!pathARPGF.isEmpty() && pathARPGF.get(pathARPGF.size() - 1).getId() == pair.getValue()) {
                            arpgfPathNum[j][i] += 1;
                            arpgfPathStretch[j][i] += Double.valueOf(pathARPGF.size()) / Double.valueOf(pathBFS.size());
                            arpgfHeaderSize[j][i] += arpgfHSize.isEmpty() ? 0 : arpgfHSize.get(0);
                        }

                        if (!pathARPSR.isEmpty() && pathARPSR.get(pathARPSR.size() - 1).getId() == pair.getValue()) {
                            arpsrPathNum[j][i] += 1;
                            arpsrPathStretch[j][i] += Double.valueOf(pathARPSR.size()) / Double.valueOf(pathBFS.size());
                        }
                    }
                    System.out.println("All algorithms were used for pair[" + ++iter + "]. BFS found:" + bfsPathNum[j][i]);
                }

                gfPathStretch[j][i] = gfPathStretch[j][i] / gfPathNum[j][i];
                gpsrPathStretch[j][i] = gpsrPathStretch[j][i] / gpsrPathNum[j][i];
                gpgfPathStretch[j][i] = gpgfPathStretch[j][i] / gpgfPathNum[j][i];
                argfGPathStretch[j][i] = argfGPathStretch[j][i] / argfGPathNum[j][i];
                argfLPathStretch[j][i] = argfLPathStretch[j][i] / argfLPathNum[j][i];
                arpgfPathStretch[j][i] = arpgfPathStretch[j][i] / arpgfPathNum[j][i];
                arpsrPathStretch[j][i] = arpsrPathStretch[j][i] / arpsrPathNum[j][i];
                //packet header sizes
                gpgfHeaderSize[j][i] = Double.valueOf(gpgfHeaderSize[j][i]) / Double.valueOf(gpgfPathNum[j][i]);
                arpgfHeaderSize[j][i] = Double.valueOf(arpgfHeaderSize[j][i]) / Double.valueOf(arpgfPathNum[j][i]);

                ++j;
            }
            coverageRatio[i] = Double.valueOf(nodesNum) / (size * size);
        }

        System.out.println("Coverage ratio:" + Arrays.toString(coverageRatio));
        System.out.println("Avg. node degree:" + Arrays.deepToString(avgDeg));
        System.out.println("BFS path trials=" + Arrays.deepToString(bfsPathNum));
        System.out.println("GF path trials=" + Arrays.deepToString(gfPathNum));
        System.out.println("GPSR path trials=" + Arrays.deepToString(gpsrPathNum));
        System.out.println("GPGF path trials=" + Arrays.deepToString(gpgfPathNum));
        System.out.println("ARGF_G path trials=" + Arrays.deepToString(argfGPathNum));
        System.out.println("ARGF_L path trials=" + Arrays.deepToString(argfLPathNum));
        System.out.println("ARPGF path trials=" + Arrays.deepToString(arpgfPathNum));
        System.out.println("ARPSR path trials=" + Arrays.deepToString(arpsrPathNum));
        System.out.println("GF avg. path stretch=" + Arrays.deepToString(gfPathStretch));
        System.out.println("GPSR avg. path stretch=" + Arrays.deepToString(gpsrPathStretch));
        System.out.println("GPGF avg. path stretch=" + Arrays.deepToString(gpgfPathStretch));
        System.out.println("ARGF G avg. path stretch=" + Arrays.deepToString(argfGPathStretch));
        System.out.println("ARGF L avg. path stretch=" + Arrays.deepToString(argfLPathStretch));
        System.out.println("ARPGF avg. path stretch=" + Arrays.deepToString(arpgfPathStretch));
        System.out.println("ARPSR avg. path stretch=" + Arrays.deepToString(arpsrPathStretch));

        System.out.println("GPGF packet header size=" + Arrays.deepToString(gpgfHeaderSize));
        System.out.println("ARPGF packet header size=" + Arrays.deepToString(arpgfHeaderSize));
    }


    /**
     * This method is used to retrieve local obstacles, i.e., which repulsion zones contain node n
     * (based on Equation 18 in the AGRA paper)
     *
     * @param n          - current node
     * @param obstacles  - Collection of the detected at the edge obstacles
     * @param radioRange - radio range of the node
     * @param deg        - repulsive field attenuation order degree
     * @return set of local obstacles for node n
     */
    private static Set<Obstacle> inObstacleRepulseZone(Node n, List<Obstacle> obstacles, double radioRange, double deg) {
        Set<Obstacle> currentObstacles = new HashSet<>();
        for (Obstacle o : obstacles)
            if (EuclDist.d(o.getX(), o.getY(), n.getX(), n.getY()) <= ((1 + 1 / deg) * o.getR() + radioRange))
                currentObstacles.add(o);
        return currentObstacles;
    }


    /**
     * internal method that is used to save data to specified file
     *
     * @param fileName - file name (with path) where to store data
     * @param data     - collection of data to store
     */
    private static void saveDataToFile(String fileName, Collection data) {
        try {
            PrintWriter output = new PrintWriter(new FileWriter(fileName));
            for (Object o : data)
                output.println(o);
            output.close();
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }
}
