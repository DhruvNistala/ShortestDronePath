# ShortestDronePath


* Project Abstract: 
	The purpose of this project is to compute the optimal path a drone can take given a sequence of points
 	with a minumum turning radius of 1, assuming the drone has a constant velocity. There are two possible
	ways to run this code: by plotting the optimal path through a given sequence of points (heading angles
	determined using Dijkstra's algorithm), and by plotting the Dubins path between a given pair of points
	with fixed heading angles, according to a CSC Dubins Path. The code is verified for various heading
	angles precisions (smallest step = 1 degree), and for multiple points. 

==============================

Author: Dhruv Nistala (Currently a senior at Westwood High School, Austin TX, Graduating Year 2021)

Email : dhruv.nistala@gmail.com 

==============================

* Acknowledgements: 
This is a high school project done during the summer of 2020 as an Intern/Researcher under the guidance of 
Prof. Swaroop Darbha from College of Engineering, A&M University at College Station, TX. 

=============================
Tools and Versions:
   Java   : Eclipse IDE ( Version: 2020-03 (4.15.0)) 
 
   Python : Pycharm  (Version 2020.1,  Python version 3.7 or later).

* Plots: Plotting can be done for two different purposes: 
	a. Plotting the entire sequence of points shows the optimal path a drone can take through many points.
	   The optimal path between two points is computed using a CSC Dubins path, and the optimal heading
           angles are determiend using Dijkstra's algorithm.

	b. By plotting Dubins only, one can view how the Dubins path is calculated between two points. 
           This is used for debugging purposes.

==============================
* FOLDERS:

   Java:
	
        Files:
		DroneShortestPath.java - main file of the project
		Bag.java - used in graph data structure
		DijkstraSP.java - computes shortest path through graph using Dijkstra's Algorithm
		Edge.java - used in graph data structure
		Graph.java - edge weighted directed graph
		IndexMinPQ.java - used in Dijkstra's Algorithm
	Steps after download:
		1. Create a new Java Project and download these files to the project's source folder.	
		2. Download the csv data files to the Java project folder.
		3. Execute the code in both files once, before using Python code.


    Python:

	Files:   
		PlotFinalPath.py - plots optimal path to all points
		PlotDubinsTwoPts.py - plots Dubins path calulation between two points
        Steps after download:
	1.	Create a new Python project and download these files to the project's source folder.
	2.	Identify the path to the output files of the Java code
	3.	Change the appropriate filepaths to match the generated output files generated by the Java code.

==============================
* Input files (for Java):
	PointsInput.csv      - Change the values in this file to match the desired points
                               and heading angle at those points. Ensure that distance between
                               consecutive points is < 4R.
	PlotDubinsOnly.csv:  - Change the values in this file to match the desired sequence of points.

==============================

* Output files (Generated by Java , input to Python) 
	inputAllDubinsTwoPts.txt - output file of optimal path thru all points from PointsInput.csv
	inputFinalPath.txt       - output file of Dubins path calculation of pair of points from PlotDubinsOnly.csv

====================================

References: 
 * The Dijkstra's Algorithm files are open-source code from princeton.edu 
 * from the book Algorithms, 4th Edition by Robert Sedgewick and Kevin Wayne.  
 
====================================

