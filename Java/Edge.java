/**
 *  The {@code DirectedEdge} class represents a weighted edge in an 
 *  {@link EdgeWeightedDigraph}. Each edge consists of two integers
 *  (naming the two vertices) and a real-value weight. The data type
 *  provides methods for accessing the two endpoints of the directed edge and
 *  the weight.
 *  <p>
 *  For additional documentation, see <a href="https://algs4.cs.princeton.edu/44sp">Section 4.4</a> of
 *  <i>Algorithms, 4th Edition</i> by Robert Sedgewick and Kevin Wayne.
 *
 *  @author Robert Sedgewick
 *  @author Kevin Wayne
 */

//weighted, directed edge
public class Edge {
	 private final int v;
	 private final int w;
	 private final double weight;
	 
	 public Edge(int v, int w, double weight) {
	        if (v < 0) throw new IllegalArgumentException("Vertex names must be nonnegative integers");
	        if (w < 0) throw new IllegalArgumentException("Vertex names must be nonnegative integers");
	        if (Double.isNaN(weight)) throw new IllegalArgumentException("Weight is NaN");
	        this.v = v;
	        this.w = w;
	        this.weight = weight;
	    }
	 
	 //returns tail
	 public int from() {
	        return v;
	    }
	 
	 //returns head
	 public int to() {
	        return w;
	    }
	 
	 //returns weight
	 public double weight() {
	        return weight;
	    }
	 
	 public String toString() {
	        return v + "->" + w + " " + String.format("%5.2f", weight);
	    }
	 public String orderString()
	 {
		 return v + " ";
	 }
}
