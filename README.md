AGRA: AI-augmented Geographic Routing Approach  
for IoT-based Incident-Supporting Applications  
=================
In this project, we develop a novel Artificial Intelligent (AI)-augmented geographic routing approach, that relies on an area knowledge (e.g., obtained from the satellite imagery available at the edge cloud). In particular, we devise a new stateless greedy forwarding algorithm that uses such a geographical knowledge (i.e., in addition to geo-coordinates) to proactively avoid the local minimum problem by diverting traffic with an algorithm that emulates electrostatic repulsive forces. In our theoretical analysis, we show that our Greedy Forwarding achieves in the worst case a 3.291 path stretch approximation bound with respect to the shortest path, without assuming presence of symmetrical links or unit disk graphs.
Proposed algorithm is evaluted with both numerical (java) and event-driven (ns-3) simulations, and we establish the practicality of our approach in a real incident-supporting hierarchical cloud deployment to demonstrate improvement of application level throughput due to a reduced path stretch under severe node failures and high mobility challenges of disaster response scenarios.

Authors
=================
2017 VIMAN laboratory, Computer Science Department, University of Missouri-Columbia.

```
Updated June 19, 2017 by Dmitrii Chemodanov
```

All feedback appreciated to dycbt4@mail.missouri.edu 

License
=================
This project is licensed under the GNU General Public License - see the [LICENSE.md](LICENSE.md) file for details


How does AGRA work?
==================
AGRA first obtains a geographical knowledge of the physical obstacles (e.g., extracts this information from the satellite imagery available at the edge cloud using deep learning) and then propagates it from the gateway as shown in the figure below:

![Figure](/img/general.png)

**Figure 1**: Incident-Supporting Hierarchical Cloud setup: AGRA first extracts geographical knowledge of physical obstacles form the pre-uploaded at the edge cloud satellite imagery and then propagates this information from the gateway during during disaster-incident response activities within a lost infrastructure
region to enhance geographical routing.

Our proposed stateless greedy forwarding algorithm viz. "Attractive/Repulsive Greedy Forwarding" is then uses this geo-knowledge in addition to geo-coordinates of the nodes in order to proactively avoid local minimum situtations as shown in the figure below:

![Figure](/img/agra_example.jpg)

**Figure 2**: Attractive/Repulsive Greedy Forwarding (ARGF) example: ARGF starts in Attraction mode until it reaches the repulsion zone; at node b (located outside the repulsion zone), ARGF returns back to its Attraction mode. ARGF can again switch to the Repulsion mode with guaranteed progress towards destination. At node d, ARGF returns to the Attraction mode and continue forwarding in this mode to destination.

What is inside?
================
The source code for the Elsevier FGCS submission (SI on Information and Resource Management Systems for Internet of Things: Energy Management, Communication Protocols and Future Applications).

Java simulation is used to evaluate our novel repulsive based Greedy Forwarding with other related stateless greedy forwarding protocols in presence of obstacles with complex concave shapes

NS-3 simulation is used to evalute the application level throughput gained by those greedy forwarding protocols as well as by commonly used AODV and HWMP protocols under severe node failures and high mobility challenges of disaster response scenarios

Distribution
================
The distribution tree contains: 

* README

	- this file
    
* LICENSE

	- GNU license file

* img/ (folder with images used in README)
    
* java_sim/ (numerical simulation in presence of obstacles with complex concave shapes)	

    - build.xml (build file for ant)    
    
    - lib/      (java library dependencies)
    
    - result/   (empty folder to store some of the simulation results of experiment #2 in a form of txt files)
    
    - src/      (java source files)
        
        ```
        edu/um/chemodanov/agra/Main (java sim main file)
        ```
        
* ns3_sim/ (event-driven simulation of the realistic incident-supporting hierarchical cloud deployment)    
    
    - src/     (c++ source files for the AGRA protocol implemented as part of the GPSR protocol in NS-3, see - A. Fonseca, A. Camoes, T. Vazao, “Geographical routing implementation in NS3”, Proc. of ACM ICST, 2012.)
    
    - scratch/ (c++ source files of the node high failures and mobility scenarios)
    
        ```
        agra_failure_expr.cc  (main file for the high node failures simulation; the Joplin MO high school scene is used)
        ```
        
        ```
        agra_mobility_expr.cc (main file for the high node mobility simulation; the Joplin MO hospital scene is used)
        ```

COMPILATION AND RUN
============
Compiling and run of this software requires Ant, Java 1.7 and NS-3 (v.3.25 or higher) installed. These can be downloaded respectively from:  
http://jakarta.apache.org/ant/index.html 
http://java.sun.com/j2se/
http://www.nsnam.org/

## Java simulations
* navigate to *java_sim* folder

* clean

    ```
    ant clean 
    ```
    
* compile

    ```
    ant
    ```

* run GUI simulation
    
    ```
    java -jar target/agra.jar 0
    ```
    - visit User Web Interface at: http://localhost:4567/topology.html
    
    - optionally specify two more integer arguments: first X is to specify the area size (i.e., to place X by X number of nodes); and second Y is to specify the vertical resolution (i.e., to display graphics using Y by Y pixels)

* run any of 5 experiments
    
    ```
    java -jar target/agra.jar (from 1 to 5)
    
    ```
    - optionally specify three more integer arguments: first X is to specify the number of trials (default is 50); second Y is to specify the number of source-destination pairs (default is 1000); and third Z is to specify the packet's TTL policy (default is 128) for the experiment number 5 only!
    
## NS-3 simulations
* navigate to *ns-3_sim* folder

* copy everything to your NS-3 directory
    
* refer to NS-3 manual (see https://www.nsnam.org/docs/release/3.26/tutorial/html/getting-started.html#running-a-script) to run any of the following two simulations:
        
    ```
    agra_failure_expr.cc  (main file for the high node failures simulation; the Joplin MO high school scene is used)
    ```
        
    ```
    agra_mobility_expr.cc (main file for the high node mobility simulation; the Joplin MO hospital scene is used)
    ```    