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

import edu.um.chemodanov.agra.model.Topology;
import edu.um.chemodanov.agra.spark.SparkLauncher;

public class Main {

    public static void main(String[] args) {
        if (args.length >= 1 && isDouble(args[0])) {
            //modifiable variables
            int size = 20; //default value of the number of nodes (can be changed through arguments only for GUI Simulator)
            int resolution = 720; //default value of the screen height resolution (can be changed through arguments only for GUI Simulator)
            int numTrials = 50; //default value for a number of trials (can be changed through arguments)
            int numPairs = 1000; //default value for a number of src-dst pairs (can be changed through arguments)
            int ttlPolicy = 128; //default value for the packet's TTL (can be changed through arguments only for Experiment 5)

            //fixed variables (for experiments only)
            double[] degArray = {1, 2, 3, 4, 5};
            int[] obstacles = {10, 30, 50, 100};
            int[] ttl = {16, 32, 64, 128, 256};
            ExperimentHelper helper = new ExperimentHelper();

            //decide on scenario:
            switch ((int) Math.round(Double.valueOf(args[0]))) {
                case 0: //run GUI simulator
                    if (args.length > 1 && isDouble(args[1])) {
                        size = (int) Math.round(Double.valueOf(args[1]));
                        System.out.println("Size of area was set to " + size);
                    }
                    if (args.length > 2 && isDouble(args[2])) {
                        resolution = (int) Math.round(Double.valueOf(args[2]));
                        System.out.println("Resolution of area was set to " + resolution);
                    }
                    Topology t = new Topology(size);
                    t.reInitializeNeighbors();
                    System.out.println("Neighbors were initialized!");
                    SparkLauncher.start(t, resolution);
                    break;
                case 1: //run experiment number 1 (various attenuation degrees)
                    if (args.length > 1 && isDouble(args[1])) {
                        numTrials = (int) Math.round(Double.valueOf(args[1]));
                        System.out.println("Number of trials was set to " + numTrials);
                    }
                    if (args.length > 2 && isDouble(args[2])) {
                        numPairs = (int) Math.round(Double.valueOf(args[2]));
                        System.out.println("Number of src-dst pairs was set to " + numPairs);
                    }
                    helper.doExperiment1(numTrials, numPairs, 50, degArray);
                    break;
                case 2: //run experiment number 2 (local obstacle overhead estimation)
                    if (args.length > 1 && isDouble(args[1])) {
                        numTrials = (int) Math.round(Double.valueOf(args[1]));
                        System.out.println("Number of trials was set to " + numTrials);
                    }
                    helper.doExperiment2(numTrials, obstacles, 2);
                    break;
                case 3: //run experiment number 3 (evaluation of main stateless geo-routing approaches without TTL)
                    if (args.length > 1 && isDouble(args[1])) {
                        numTrials = (int) Math.round(Double.valueOf(args[1]));
                        System.out.println("Number of trials was set to " + numTrials);
                    }
                    if (args.length > 2 && isDouble(args[2])) {
                        numPairs = (int) Math.round(Double.valueOf(args[2]));
                        System.out.println("Number of src-dst pairs was set to " + numPairs);
                    }
                    helper.doExperiment3(numTrials, numPairs, obstacles, 2);
                    break;
                case 4: //run experiment number 4 (evaluation of main stateless geo-routing approaches with different TTLs)
                    if (args.length > 1 && isDouble(args[1])) {
                        numTrials = (int) Math.round(Double.valueOf(args[1]));
                        System.out.println("Number of trials was set to " + numTrials);
                    }
                    if (args.length > 2 && isDouble(args[2])) {
                        numPairs = (int) Math.round(Double.valueOf(args[2]));
                        System.out.println("Number of src-dst pairs was set to " + numPairs);
                    }
                    helper.doExperiment4(numTrials, numPairs, 100, 2, ttl);
                    break;
                case 5: //run experiment number 5 (evaluation of main stateless geo-routing approaches with fixed TTL under various node degrees)
                    if (args.length > 1 && isDouble(args[1])) {
                        numTrials = (int) Math.round(Double.valueOf(args[1]));
                        System.out.println("Number of trials was set to " + numTrials);
                    }
                    if (args.length > 2 && isDouble(args[2])) {
                        numPairs = (int) Math.round(Double.valueOf(args[2]));
                        System.out.println("Number of src-dst pairs was set to " + numPairs);
                    }
                    if (args.length > 3 && isDouble(args[3])) {
                        ttlPolicy = (int) Math.round(Double.valueOf(args[3]));
                        System.out.println("Packet's TTL was set to " + ttlPolicy);
                    }
                    helper.doExperiment5(numTrials, numPairs, 100, 2, ttlPolicy, 5);
                default:
                    System.out.println("Unknown scenario number: " + (int) Math.round(Double.valueOf(args[0]))
                            + ". Please enter a valid scenario number from 0 to 5!");
            }
        } else
            System.out.println("Not enough or wrong input arguments. Please specify at least a valid scenario number from 0 to 5!");
    }

    private static boolean isDouble(String str) {
        try {
            double d = Double.valueOf(str);
        } catch (NumberFormatException nfe) {
            return false;
        }
        return true;
    }
}
