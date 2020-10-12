/**
 * This is the main file of the project.
 * 
 * This class, provided an input file, will output the
 * shortest CSC Dubins Path configuration to traverse
 * these points in the given sequence. The optimal heading
 * angles configuration is computed using Dijkstra's algorithm.
 * 
 * Input file format is csv and includes the list of
 * points in order. This code can compute CSC Dubins path,
 * so the distance between adjacent points must be < 4R.
 * R is the minimum turning radius, and is currently defaulted
 * to 1. See PointsInput.csv for reference.
 * 
 * To test Dubins code, enter the start points, end points, and
 * defined heading angles in PointsDubinsOnly.csv.
 * 
 * For visual outputs, see Python files.
 * 
 * Author  : Dhruv Nistala, Westwood High School, Austin TX
 * Date    : June-September 2020
 * 
 * Project done under guidance of Professor Swaroop Darbha, 
 * Texas A&M University at College Station.
 * 	
 */

import java.util.*;

import java.io.*;
import java.math.*;

public class DroneShortestPath {

	final static double pi = Math.PI;
	static boolean DBG1 = true;
	static boolean DBG2 = true; //turn ONE of these on at a time ONLY, for debugging purposes
	static boolean DUBINS_ONLY = true; //for testing only dubins 
	static double R = 1; //***TO CHANGE RADIUS, CHANGE THIS VALUE***
	static PrintWriter pw;
	static PrintWriter pwAll;
		
	public static void main(String[] args) throws Exception {
		DBG1 = false;
		DBG2 = false;
		DUBINS_ONLY = false; //change this one to test Dubins subroutine for only two points, with heading angles predefined
		
		Scanner sc;
		Scanner scDubinsOnly;
		sc = new Scanner(new File("PointsInput.csv"));
		scDubinsOnly = new Scanner(new File("PointDubinsOnly.csv"));
		pw = new PrintWriter(new File("inputAllDubinsTwoPts.txt")); //for testing Dubins between 2 points
		pwAll = new PrintWriter(new File("inputFinalPath.txt")); //for showing final path for all points
		//format of inputFinalPath.txt[start point, start heading, start circle center, first tangent, end point, end heading, end circle center, second tangent]
		
		if (DUBINS_ONLY) {
			double startX = 0, startY = 0, stHead = 0, endX = 0, endY = 0, endHead = 0;
			while (scDubinsOnly.hasNext())
			{
				//read in list of points
				scDubinsOnly.nextLine();
				String[] parts = scDubinsOnly.next().split(",");
				startX = Double.parseDouble(parts[0]);
				startY = Double.parseDouble(parts[1]);
				stHead = Double.parseDouble(parts[2]);
				endX = Double.parseDouble(parts[3]);
				endY = Double.parseDouble(parts[4]);
				endHead = Double.parseDouble(parts[5]);
			}
			ArrayList<Pair> pointsPair = new ArrayList<Pair>();
			pointsPair.add(new Pair(startX, startY));
			pointsPair.add(new Pair(endX, endY));
			double pairHeadings[] = new double[2];
			pairHeadings[0] = stHead;
			pairHeadings[1] = endHead;
			pwAllPrint(pairHeadings, pointsPair);
		}
		else {
			ArrayList<Pair> points = new ArrayList<Pair>();
			
			while (sc.hasNext())
			{
				//read in list of points
				sc.nextLine();
				String[] parts = sc.next().split(",");
				double pointX = Double.parseDouble(parts[0]);
				double pointY = Double.parseDouble(parts[1]);
				
				points.add(new Pair(pointX, pointY));
			}
			
			int numHeadings = 6; //***CHANGE THIS VALUE TO CHANGE NUMBER OF POSSIBLE HEADING ANGLES FOR DUBINS COMPUTATION***
			int headingIncrements = 360/numHeadings;
			int numPoints = points.size();
				//creating graph
				Graph G = new Graph(numHeadings*numPoints+2);
				//creating dummy points edges
				for (int i = 1; i <= numHeadings; i++)
				{
					G.addEdge(new Edge(0, i, 0));
					G.addEdge(new Edge(G.V() - 1 - i,G.V() - 1, 0));
				}
				//creating edges between all coordinates' orientations
				for (int i = 0; i < points.size()-1; i++)
				{
					DBG2Print("----------------------------------------------\nADDING EDGE " + (i+1) + " now...");
					for (int j = 1; j <= numHeadings; j++)
					{
						for (int k = 1; k <= numHeadings; k++)
						{
							DBG2Print("\n---------------------------\nCreating new edge from " + (points.get(i)) + " to " + (points.get(i+1)) + " with headings " + headingIncrements * j + " and " + headingIncrements*k + "\n");
							//edge weight = Dubins distance between two points
							double weight = dubinsDist(points.get(i), points.get(i+1), new Heading(headingIncrements * j), new Heading(headingIncrements * k))[0];
							G.addEdge(new Edge((i)*numHeadings+j, (i+1)*numHeadings+k, weight));
						
						}
					}
				}
				DBG2Print(G.toString());
				
				

				
			
				DBG2Print("-------------------\nPrinting test best case: \n");
				
				
				
			
			//finding shortest path using Dijkstra Algorithm
			//establish source and terminus points
			int s = 0;
			int t = numHeadings*numPoints+1;
			String nodesOrder = "";
			DijkstraSP sp = new DijkstraSP(G, s); //compute shortest path
			if (sp.hasPathTo(t)) {
	            //System.out.printf("%d to %d (%.2f)  ", s, t, sp.distTo(t));
	            for (Edge e : sp.pathTo(t)) {
	            	nodesOrder += e.orderString();
	            }
	            System.out.println();
	        }
	        else {
	        	System.out.printf("%d to %d         no path\n", s, t);
	        }
			
			//parsing order of nodes determined by Dijkstra's algorithm
			String[] parseNodesReverse = nodesOrder.split(" ");
			int nodes[] = new int[parseNodesReverse.length-1];
			for (int i = 0; i < nodes.length; i++)
			{
				nodes[i] = Integer.parseInt(parseNodesReverse[nodes.length-i-1]);
				//System.out.println("node: " + nodes[i]);
			}
			
			double[] masterHeading = new double[points.size()];
			//tracing back to the point's original orientation, so can calculate distance one last time with proper heading.
			for (int i = 0; i < nodes.length; i++)
			{
				double headingAngle = (nodes[i] - i*numHeadings) * 360/numHeadings % 360;
				masterHeading[i] = headingAngle;
				//System.out.println("\npoint: " + points.get(i));
				//System.out.println("Heading angle: " + headingAngle);
			}
			
			//finally calculating the distance from each point to next point
			pwAllPrint(masterHeading, points);
			
		}
		
		
		

		pw.close();
		pwAll.close();

	}

	static double[] dubinsDist(Pair a, Pair b, Heading hA, Heading hB) throws FileNotFoundException {
		double D = dist(a, b);
		
		//CSC path calculated here
		if (D > 4 * R) {
			// c1 and c3 clockwise, direct tan. c2 and c4 anticlockwise, direct tangent.
			// c1-c4 and c2-c3 is transverse tan.
			boolean transverse, direct = false;
			Circle c1 = new Circle(a, true, hA);
			Circle c2 = new Circle(a, false, hA);
			Circle c3 = new Circle(b, true, hB);
			Circle c4 = new Circle(b, false, hB);

			pw.println("" + 1 + " " + R);
			pw.println(c1.startPoint.x + " " + c1.startPoint.y);
			pw.println(c3.startPoint.x + " " + c3.startPoint.y);
			pw.println("" + hA.degrees + " " + hB.degrees); //heading printout
			pw.println(c1.center.x + " " + c1.center.y);
			pw.println(c2.center.x + " " + c2.center.y);
			pw.println(c3.center.x + " " + c3.center.y);
			pw.println(c4.center.x + " " + c4.center.y);
			
			//getting possible paths between points
			double[] min = new double[1];
			min[0] = Integer.MAX_VALUE;
			double[] d13 = directTanDist(c1, c3);
			double[] d24 = directTanDist(c2, c4);
			double[] d14 = transverseTanDist(c1, c4);
			double[] d23 = transverseTanDist(c2, c3);

			if (d13[0] < min[0])
				min = d13;
			if (d24[0] < min[0])
				min = d24;
			if (d14[0] < min[0])
				min = d14;
			if (d23[0] < min[0])
				min = d23;

			return min; //min has configuration of path between 2 pts with smallest distance

		} else {
			//CCC path not yet coded. make sure distance between consecutive points is < 4R
			return new double[1];
		}

	}

	static double dist(Pair a, Pair b) {
		//calculates distance between two points
		double xDist = Math.abs(a.x - b.x);
		double yDist = Math.abs(a.y - b.y);
		double distance = Math.sqrt(Math.pow(xDist, 2) + Math.pow(yDist, 2));
		return distance;
	}

	static double[] directTanDist(Circle a, Circle b) {//calculates CSC path given same-direction circles
		double distance[] = new double[9];
		double theta;
		// finding angle between a and b centers, and creating potential tangent points.
		theta = findAngle(a.center, b.center);
		Pair at1 = new Pair(a.center.x + R * Math.sin(theta), a.center.y - R * Math.cos(theta));
		Pair bt1 = new Pair(b.center.x + R * Math.sin(theta), b.center.y - R * Math.cos(theta));
		Pair at2 = new Pair(a.center.x - R * Math.sin(theta), a.center.y + R * Math.cos(theta));
		Pair bt2 = new Pair(b.center.x - R * Math.sin(theta), b.center.y + R * Math.cos(theta));

		// if at1 to bt1 has circle on right, then is clockwise. Else, at2 to bt2 is
		// clockwise.
		// finding angle between at1 and bt1
		double angle = findAngle(at1, bt1);
		Pair testCW = createTestCenter(bt1, angle);

		 DBG1Print("Circle a: " + a.center.toString() + "\nCircle b: " + b.center.toString()); DBG1Print("at1: " + at1); DBG1Print("bt1: " + bt1);
		 DBG1Print("at2: " + at2); DBG1Print("bt2: " + bt2);
		 DBG1Print("angle from at1 to bt1: = " + (angle * 180 / pi));
		 DBG1Print("b center: " + b.center); DBG1Print("testCW: " + testCW);
		
		//storing information about circles and tangents
		distance[1] = a.center.x;
		distance[2] = a.center.y;
		distance[3] = b.center.x;
		distance[4] = b.center.y;
		
		//determining whether circles are clockwise, and choosing path accordingly.
		if (closeProxPts(testCW, b.center)) {// bt1 is clockwise.
			DBG1Print("bt1 is clockwise.");
			if (b.clockwise) {
				//pw.println("exit tangent point: " + at1);
				//pw.println("entry tangent point: " + bt1);
				DBG1Print("choose bt1.");
				distance[0] += calcArcLength(a, b, at1, bt1);
				distance[5] = at1.x;
				distance[6] = at1.y;
				distance[7] = bt1.x;
				distance[8] = bt1.y;
				pw.println(at1.x + " " + at1.y + " " + bt1.x + " " + bt1.y);
			} else {
				//pw.println("exit tangent point: " + at2);
				//pw.println("entry tangent point: " + bt2);
				DBG1Print("choose bt2.");
				distance[0] += calcArcLength(a, b, at2, bt2);
				distance[5] = at2.x;
				distance[6] = at2.y;
				distance[7] = bt2.x;
				distance[8] = bt2.y;
				pw.println(at2.x + " " + at2.y + " " + bt2.x + " " + bt2.y);
			}
		} else {// bt2 is clockwise.
			DBG1Print("bt2 is clockwise.");
			if (b.clockwise) {
				//pw.println("exit tangent point: " + at2);
				//pw.println("entry tangent point: " + bt2);
				DBG1Print("choose bt2.");
				distance[0] += calcArcLength(a, b, at2, bt2);
				distance[5] = at2.x;
				distance[6] = at2.y;
				distance[7] = bt2.x;
				distance[8] = bt2.y;
				pw.println(at2.x + " " + at2.y + " " + bt2.x + " " + bt2.y);
			} else {
				//pw.println("exit tangent point: " + at1);
				//pw.println("entry tangent point: " + bt1);
				DBG1Print("choose bt1.");
				distance[0] += calcArcLength(a, b, at1, bt1);
				distance[5] = at1.x;
				distance[6] = at1.y;
				distance[7] = bt1.x;
				distance[8] = bt1.y;
				pw.println(at1.x + " " + at1.y + " " + bt1.x + " " + bt1.y);
			}

		}

		DBG1Print("Distance: " + distance[0] + "\n");
		DBG2Print("Distance: " + distance[0] + "\n");

		return distance; //storage is in format [distance, startCircleX, startCircleY, endCircleX, endCircleY, first tangent X, first tangent Y, second tangent X, second tangent Y]
	}

	static double[] transverseTanDist(Circle a, Circle b) {//calculates CSC path for opposite-direction circles
		double distance[] = new double[9];
		double theta;

		// distance to intersection of tangent lines:
		double dist = dist(a.center, b.center) / 2;
		// finding midpoint
		Pair intersection = new Pair((0.5) * (a.center.x + b.center.x), (0.5) * (a.center.y + b.center.y));
		Pair at1, bt1, at2, bt2;

		// apAngle = angle from center to intersection point
		double angleIntersection = findAngle(a.center, intersection);
		// theta = polar angle between center and tangent points
		theta = Math.acos(R / dist);

		double addA = angleIntersection + theta;
		double addB = angleIntersection + theta + pi;
		double subA = angleIntersection - theta;
		double subB = angleIntersection - theta + pi;

		at1 = new Pair(a.center.x + R * Math.cos(addA), a.center.y + R * Math.sin(addA));
		bt1 = new Pair(b.center.x + R * Math.cos(addB), b.center.y + R * Math.sin(addB));
		at2 = new Pair(a.center.x + R * Math.cos(subA), a.center.y + R * Math.sin(subA));
		bt2 = new Pair(b.center.x + R * Math.cos(subB), b.center.y + R * Math.sin(subB));

		// if at1 to bt1 has circle on right, then bt1 is clockwise, and at1 is
		// anticlockwise.
		Pair testCW = createTestCenter(bt1, R);

		DBG1Print("angleIntersection: " + (angleIntersection * 180 / pi));
		DBG1Print("theta: " + (theta * 180 / pi));
		DBG1Print("addA: " + (addA) * 180 / pi);
		DBG1Print("Circle a: " + a.center.toString() + "\nCircle b: " + b.center.toString());
		DBG1Print("at1: " + at1);
		DBG1Print("bt1: " + bt1);
		DBG1Print("at2: " + at2);
		DBG1Print("bt2: " + bt2);
		DBG1Print("b center: " + b.center);
		DBG1Print("testCW: " + testCW);
		
		//storing information about circles and tangents
				distance[1] = a.center.x;
				distance[2] = a.center.y;
				distance[3] = b.center.x;
				distance[4] = b.center.y;
		
		//determining whether second circle is clockwise, and choosing path accordingly.
		if (closeProxPts(testCW, b.center)) {
			// bt1 is clockwise
			DBG1Print("bt1 is clockwise.");
			if (b.clockwise) {
				//pw.println("first tangent point: " + at1);
				//pw.println("second tangent point: " + bt1);
				DBG1Print("choose bt1.");
				distance[0] += calcArcLength(a, b, at1, bt1);
				distance[5] = at1.x;
				distance[6] = at1.y;
				distance[7] = bt1.x;
				distance[8] = bt1.y;
				pw.println(at1.x + " " + at1.y + " " + bt1.x + " " + bt1.y);
			} else {
				//pw.println("first tangent point: " + at2);
				//pw.println("second tangent point: " + bt2);
				DBG1Print("choose bt2.");
				distance[0] += calcArcLength(a, b, at2, bt2);
				distance[5] = at2.x;
				distance[6] = at2.y;
				distance[7] = bt2.x;
				distance[8] = bt2.y;
				pw.println(at2.x + " " + at2.y + " " + bt2.x + " " + bt2.y);

			}
		} else {
			// bt2 is clockwise
			DBG1Print("bt2 is clockwise.");
			if (b.clockwise) {
				//pw.println("first tangent point: " + at2);
				//pw.println("second tangent point: " + bt2);
				DBG1Print("choose bt2.");
				distance[0] += calcArcLength(a, b, at2, bt2);
				distance[5] = at2.x;
				distance[6] = at2.y;
				distance[7] = bt2.x;
				distance[8] = bt2.y;
				pw.println(at2.x + " " + at2.y + " " + bt2.x + " " + bt2.y);
			} else {
				//pw.println("first tangent point: " + at1);
				//pw.println("second tangent point: " + bt1);
				DBG1Print("choose bt1.");
				distance[0] += calcArcLength(a, b, at1, bt1);
				distance[5] = at1.x;
				distance[6] = at1.y;
				distance[7] = bt1.x;
				distance[8] = bt1.y;
				pw.println(at1.x + " " + at1.y + " " + bt1.x + " " + bt1.y);
			}
		}

		DBG1Print("Distance: " + distance[0] + "\n");
		DBG2Print("Distance: " + distance[0] + "\n");
		return distance; //storage is in format [distance, startCircleX, startCircleY, endCircleX, endCircleY, first tangent X, first tangent Y, second tangent X, second tangent Y]
	}

	static double calcArcLength(Circle a, Circle b, Pair at, Pair bt) {//calculates arc length along both circles and distance between tangent points
		double totalDistance = 0;
		double straight = dist(at, bt);
		totalDistance += straight;
		double stAngle;
		double atAngle;
		double endAngle;
		double btAngle;
		double aArc;
		double bArc;

		// solving arc of circle A
		// atAngle = angle from center to tangent point, stAngle = angle from center to
		// start point
		atAngle = findAngle(a.center, at);
		stAngle = findAngle(a.center, a.startPoint);

		aArc = R * getTurnAngle(stAngle, atAngle, a);

		/*
		 * DBGPrint("stAngle: " + stAngle); DBGPrint("atAngle: " + atAngle);
		 * DBGPrint("A arc: " + aArc);
		 */

		// solving arc of circle B
		// btAngle = angle from center to tangent point, stAngle = angle from center to
		// start point
		btAngle = findAngle(b.center, bt);
		endAngle = findAngle(b.center, b.startPoint);

		bArc = R * getTurnAngle(btAngle, endAngle, b);

		/*
		 * DBGPrint("btAngle: " + btAngle); DBGPrint("endAngle: " + endAngle);
		 * DBGPrint("B arc: " + bArc);
		 */

		totalDistance += aArc;
		totalDistance += bArc;

		return totalDistance;
	}

	static class Heading {//class that represents an angle, input in degrees, stored in radians (also degree value stored)
		double angle;
		double slope;
		double degrees;
		
		Heading(double degrees) {
			this.degrees = degrees;
			angle = degrees * pi / 180;
			slope = Math.tan(angle);
			if (closeProxVal(angle, pi / 2) || closeProxVal(angle, 3 * pi / 2)) {
				slope = Integer.MAX_VALUE;
			}
		}
	}

	static class Pair {//class representing x-y coordinate
		double x, y;

		Pair(double a, double b) {
			x = a;
			y = b;
		}

		public String toString() {
			return "(" + x + ", " + y + ")";
		}
	}

	static class Circle {//class representing circle, including start point and tangent point
		Pair center;
		double radius;
		boolean clockwise;
		Pair startPoint;
		Pair incomingTangentPoint;
		Heading h;

		Circle(Pair startPoint, boolean clockwise, Heading h) {
			this.startPoint = startPoint;
			this.h = h;
			this.clockwise = clockwise;
			double x = R * Math.sin(h.angle); // need to find sin because need to use inverse.;
			if (!clockwise)
				x = -x;
			double y;
			
			if (h.slope != 0)
				y = R * (-1 / h.slope) * x;
			else if (h.slope == Integer.MAX_VALUE)
				y = 0;
			else {
				y = R;// * Math.cos(h.angle);
				if (clockwise)
					y = -y;
			}
			

			center = new Pair(startPoint.x + x, startPoint.y + y);

		}
		public String toString()
		{
			return center.toString();
		}
	}

	static double sq(double a) {//square function
		return Math.pow(a, 2);
	}

	static double sqrt(double a) {//square root function
		return Math.pow(a, 0.5);
	}

	static double findSlope(Pair a, Pair b) {//finds slope between two points
		if (Math.abs(a.x - b.x) >= 0.0001) {
			return (b.y - a.y) / (b.x - a.x);
		}
		return Integer.MAX_VALUE; //vertical line
	}

	static double findAngle(Pair a, Pair b) {//finds angle between two points
		// order matters! b is supposed to be HEAD of vector.
		double slope = findSlope(a, b);
		double angle;
		if (slope == Integer.MAX_VALUE) {
			if (b.y - a.y > 0.0001) {
				angle = pi / 2;
			} else {
				angle = 3 * pi / 2;
			}
		}
		/*
		 * else if (closeProxVal(slope, 0)) { angle = 0; if (a.x - b.x > 0.0001) angle =
		 * pi; }
		 */
		else {
			angle = Math.atan(slope);
			if (a.x - b.x > 0.0001)
				angle += pi;
		}

		angle += 2 * pi;
		angle %= (2 * pi);

		return angle;
	}

	static Pair createTestCenter(Pair b, double angle) {//creates test point, used to see if circle is clockwise
		return new Pair(b.x + R * Math.cos(angle - pi / 2), b.y + R * Math.sin(angle - pi / 2));
	}

	static boolean closeProxPts(Pair a, Pair b) {//checks if pts are identical, accounts for rounding error with distance formula
		boolean same = Math.abs(b.x - a.x) < 0.001 && Math.abs(b.y - a.y) < 0.001;
		return same;
	}

	static boolean closeProxVal(double a, double b) {//checks if values are equal, accounts for rounding error with distance formula
		boolean same = Math.abs(b - a) < 0.0001;
		return same;
	}

	static void DBG1Print(String s) {//debug prints
		if (DBG1)
			System.out.println(s);
	}
	
	static void pwAllPrint(double[] masterHeading, ArrayList<Pair> points) throws FileNotFoundException
	{//outputs final path to file
		double totalDistance = 0;
		for (int i = 0; i < points.size() - 1; i++)
		{
			double[] printInformation = dubinsDist(points.get(i), points.get(i+1), new Heading(masterHeading[i]), new Heading(masterHeading[i+1]));
			double distance = printInformation[0];
			totalDistance += distance;
			System.out.println("Distance from " + points.get(i) + " with heading " + masterHeading[i] + " degrees to " + points.get(i+1) + " with heading " + masterHeading[i+1] + " degrees is " + distance);
			
			double[] finalInfo = new double[14];
			finalInfo[0] = points.get(i).x;
			finalInfo[1] = points.get(i).y;
			finalInfo[2] = masterHeading[i];
			finalInfo[3] = printInformation[1];
			finalInfo[4] = printInformation[2];
			finalInfo[5] = printInformation[5];
			finalInfo[6] = printInformation[6];
			finalInfo[7] = points.get(i+1).x;
			finalInfo[8] = points.get(i+1).y;
			finalInfo[9] = masterHeading[i+1];
			finalInfo[10] = printInformation[3];
			finalInfo[11] = printInformation[4];
			finalInfo[12] = printInformation[7];
			finalInfo[13] = printInformation[8];

		 pwAll.print(R + " "); for (double d : finalInfo) pwAll.print(d + " ");
		 pwAll.println();
		}
		System.out.println("The total distance of this optimal path is " + totalDistance);
	}
	
	static void DBG2Print(String s)
	{//debug prints
		if (DBG2)
			System.out.println(s);
	}
	
	static double getTurnAngle(double st, double ct, Circle c) {// st is start point angle, ct is tangent point angle. finds arc along circle
		double turnAngle;
		if (c.clockwise) {
			turnAngle = st - ct;
		} else {
			turnAngle = ct - st;
		}
		if (turnAngle < 0)
			turnAngle += 2 * pi;

		return turnAngle;
	}

}
