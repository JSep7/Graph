/* Written by: Jaime Sepulveda
   Project 2 Graph Overview:
   Builds a weighted graph and runs Dijkstra's and Bellman-Ford
   algorithms to find the shortest paths from a source node.
   Outputs results to files for comparison.
   Class: COP3503C
*/

import java.io.*;
import java.util.*;

/*
 * A directed graph with nodes and edges. We are not going to allow
 * duplicate edges.
 */
public class Graph {
    /*
     * A node. Contains a name, a payload, and edges.
     */
    private class Node {
        public String name;
        public Object payload;

        // Map of edges, from name to weight.
        private HashMap<String, Integer> edges;

        /*
         * Connect this node to node to. Returns false if to is not in the
         * graph, or if they were already connected; returns true if they
         * are successfully connected.
         */
        public boolean connect(String to, Integer weight) {
            if (!nodes.containsKey(to)) {
                return false;
            }
            Integer result = edges.put(to, weight);
            return (result == null);
        }

        /*
         * Disconnect from node to. Returns true if it disconnects them,
         * false if they weren't connected.
         */
        public boolean disconnect(String to) {
            return (edges.remove(to) != null);
        }

        /*
         * Check whether we have an edge to node to.
         */
        public boolean connected(String to) {
            return edges.containsKey(to);
        }

        /*
         * Return the weight of the edge to node to, or null if they're
         * not connected.
         */
        public Integer edgeWeight(String to) {
            return edges.get(to);
        }

        /*
         * Change our data value.
         */
        public void changeNodeValue(Object value) {
            payload = value;
        }

        /*
         * Print our name, value and connections.
         */
        public void printNode() {
            System.out.printf("Node %s with value %s.\n", name,
                    payload.toString());
            for (String to : edges.keySet()) {
                Integer weight = edges.get(to);
                System.out.printf(" Connected to %s at weight %d.\n", to, weight);
            }
        }

        /*
         * Constructor. Needs the index name and the payload object.
         * If this is a simple graph, the payload can be null.
         */
        public Node(String newName, Object newPayload) {
            edges = new HashMap<String, Integer>();
            name = newName;
            payload = newPayload;
        }
    }

    /*
     * The collection for the nodes. We index the nodes by name.
     */
    HashMap<String, Node> nodes = new HashMap<String, Node>();

    /*
     * Check whether we have an edge between two nodes.
     */
    public boolean connected(String from, String to) {
        if (!nodes.containsKey(from) || !nodes.containsKey(to)) {
            return false;
        }
        return nodes.get(from).connected(to);
    }

    /*
     * Return the weight of the edge between two nodes, or null if they're
     * not connected.
     */
    public Integer edgeWeight(String from, String to) {
        if (!nodes.containsKey(from) || !nodes.containsKey(to)) {
            return null;
        }
        return nodes.get(from).edgeWeight(to);
    }

    /*
     * Connect nodes from and to. Returns false if either node is not in the graph,
     * or if they were already connected; returns true if they are successfully
     * connected.
     */
    public boolean connect(String from, String to, Integer weight) {
        if (!nodes.containsKey(from) || !nodes.containsKey(to)) {
            return false;
        }
        return nodes.get(from).connect(to, weight);
    }

    /*
     * Disconnect nodes from and to. Returns true if it disconnects them,
     * false if they weren't connected.
     */
    public boolean disconnect(String from, String to) {
        if (!nodes.containsKey(from) || !nodes.containsKey(to)) {
            return false;
        }
        return nodes.get(from).disconnect(to);
    }

    /*
     * Create a new node with associated data value. Returns true if the node is
     * successfully created, false if a node with that name already existed.
     */
    public boolean addNode(String name, Object value) {
        if (nodes.containsKey(name)) {
            return false;
        }
        Node n = new Node(name, value);
        nodes.put(name, n);
        return true;
    }

    // Returns true if and only if we have the named node.
    public boolean hasNode(String name) {
        return nodes.containsKey(name);
    }

    /*
     * Returns the data value of the named node, or null if there is no node
     * with that name.
     */
    public Object nodeValue(String name) {
        if (!nodes.containsKey(name)) {
            return null;
        }
        return nodes.get(name).payload;
    }

    /*
     * Change the data value of a node in a graph. Returns true if the value is
     * successfully changed, false if the node doesn't exist.
     */
    public boolean changeNodeValue(String name, Object value) {
        if (!nodes.containsKey(name)) {
            return false;
        }
        nodes.get(name).changeNodeValue(value);
        return true;
    }

    /*
     * Print the name, value and connections of a given node.
     */
    public void printNode(String name) {
        if (!nodes.containsKey(name)) {
            System.out.printf("No node named %s.\n", name);
            return;
        }
        nodes.get(name).printNode();
    }

    /*
     * Shortest‑path algorithms and the assignment driver
     */
    private static final int INF = Integer.MAX_VALUE / 4;

    /*
     * Tiny struct to hold distance and parent arrays
     */
    private static class PathResult {
        int[] dist, parent;

        PathResult(int[] d, int[] p) {
            dist = d;
            parent = p;
        }
    }

    /*
     * Dijkstra's algorithm using a min-heap.
     * Computes shortest distances from source.
     */
    public PathResult dijkstra(int src, int V) {
        int[] dist = new int[V + 1];        // Distance from src to each node
        int[] parent = new int[V + 1];      // Parent (previous) node in shortest path
        Arrays.fill(dist, INF);            // Init dist as infinity
        Arrays.fill(parent, -1);       // Parent set to none
        dist[src] = 0;


        // Min-heap if distances are the same it will use the lower node number
        PriorityQueue<int[]> pq = new PriorityQueue<>((a, b) -> { // Compiler did this for me
            if (a[0] != b[0]) {
                return a[0] - b[0];
            } else {
                return a[1] - b[1]; // Tie-break using node id
            }
        });
        pq.add(new int[]{0, src});


        while (!pq.isEmpty()) {
            int[] cur = pq.poll();
            int du = cur[0], u = cur[1];

            // Skip if already processed with better distance
            if (du != dist[u]) {
                continue;
            }

            // Explore all of u
            Node nu = nodes.get(String.valueOf(u));
            for (Map.Entry<String, Integer> e : nu.edges.entrySet()) {
                int v = Integer.parseInt(e.getKey());
                int w = e.getValue();
                int cand = du + w;

                // Updates if this is shorter or the same length but from a smaller node
                if (cand < dist[v] || (cand == dist[v] && u < parent[v])) {
                    dist[v] = cand;
                    parent[v] = u;
                    pq.add(new int[]{cand, v});
                }
            }
        }
        return new PathResult(dist, parent);
    }

    /*
     * Computes shortest paths from source `src`
     * to all vertices using Bellman-Ford algorithm.
     * Handles graphs with negative weights
     * if there isn't any negative cycles.
     */
    public PathResult bellmanFord(int src, int V, List<int[]> edges) {
        int[] dist = new int[V + 1];           // Distance from src to each vertex
        int[] parent = new int[V + 1];         // Parent (previous) node in shortest path
        Arrays.fill(dist, INF);                // Init dist as infinity
        Arrays.fill(parent, -1);          // Parent Set to none
        dist[src] = 0;
        parent[src] = 0;

        // Relax all edges V-1 times.
        for (int i = 1; i < V; ++i) {
            boolean changed = false;
            // Iterate over all the edges
            for (int[] e : edges) {
                int u = e[0], v = e[1], w = e[2];
                // Skip if node hasn't been reached yet.
                if (dist[u] == INF) {
                    continue;
                }
                int cand = dist[u] + w;
                // Update if shorter path found, or same with small parent
                if (cand < dist[v] || (cand == dist[v] && u < parent[v])) {
                    dist[v] = cand;
                    parent[v] = u;
                    changed = true;
                }
            }
            if (!changed) {
                break; // Early exit
            }
        }
        return new PathResult(dist, parent);
    }

    /*
     * Assignment driver
     * Reads input from a file, builds the graph,
     * runs Dijkstra's and Bellman-Ford algorithms,
     * then writes shortest path results to an output file.
     */
    public static void runAssignment(String inFile, String outFile)
            throws IOException {

        // Read's the graph size and the source
        BufferedReader br = new BufferedReader(new FileReader(inFile));
        int vertex   = Integer.parseInt(br.readLine().trim());   // Vertex count
        int source = Integer.parseInt(br.readLine().trim());   // Source vertex
        int edge   = Integer.parseInt(br.readLine().trim());   // Edge count

        // Builds the graph object
        Graph g = new Graph();
        for (int i = 1; i <= vertex; ++i) {
            g.addNode(Integer.toString(i), 0);
        }

        // Store the edges for BF
        List<int[]> edgeList = new ArrayList<>();

        for (int i = 0; i < edge; ++i) {
            StringTokenizer st = new StringTokenizer(br.readLine());
            int from = Integer.parseInt(st.nextToken());
            int to = Integer.parseInt(st.nextToken());
            int weight = Integer.parseInt(st.nextToken());

            // Add edge in both directions
            g.connect(Integer.toString(from), Integer.toString(to), weight);
            g.connect(Integer.toString(to),   Integer.toString(from), weight);

            // Store both directions for Bellman‑Ford
            edgeList.add(new int[]{from, to, weight});
            edgeList.add(new int[]{to,   from, weight});

        }
        br.close();

        // Run's the two algorithms
        PathResult dj = g.dijkstra(source, vertex);
        PathResult bf = g.bellmanFord(source, vertex, edgeList);

        // Write's the output file
        PrintWriter pw = new PrintWriter(new FileWriter(outFile));

        // Dijkstra block
        pw.println("Dijkstra");
        pw.println();
        pw.println(vertex);
        for (int v = 1; v <= vertex; ++v) {

            int distOut;
            int parentOut;

            if (v == source) { // Source Row -1 -1
                distOut = -1;
                parentOut = -1;
            } else {
                // Distance
                if (dj.dist[v] == INF) {
                    distOut = -1;
                } else{
                    distOut = dj.dist[v];
                }

                // Parent
                if (dj.parent[v] == -1){
                    parentOut = -1;
                } else{
                    parentOut = dj.parent[v];
                }
            }
            pw.printf("%d %d %d%n", v, distOut, parentOut);
        }
        pw.println();

        // Bellman–Ford block
        pw.println("Bellman-Ford");
        pw.println();
        pw.println(vertex);
        for (int v = 1; v <= vertex; ++v) {

            int distOut;
            int parentOut;

            if (v == source) { // Source row 0 0
                distOut   = 0;
                parentOut = 0;
            } else {
                // Distance
                if (bf.dist[v] == INF){
                    distOut = -1;
                } else{
                    distOut = bf.dist[v];
                }

                // Parent
                if (bf.parent[v] == -1) {
                    parentOut = -1;
                } else{
                    parentOut = bf.parent[v];
                }
            }

            pw.printf("%d %d %d%n", v, distOut, parentOut);
        }
        pw.close();
    }


    public static void main(String[] args) throws IOException {
        // Takes in the file and writes to the output
        String[] in = {
                "src/cop3503-asn2-input-1.txt",
                "src/cop3503-asn2-input-2.txt",
                "src/cop3503-asn2-input-3.txt",
        };

        String[] out = {
                "src/cop3503-asn2-output-Sepulveda-Jaime1.txt",
                "src/cop3503-asn2-output-Sepulveda-Jaime2.txt",
                "src/cop3503-asn2-output-Sepulveda-Jaime3.txt"
        };

        // Overrides any number of pairs
        if (args.length > 0) {
            if (args.length % 2 != 0) {
                System.err.println("Usage: java Graph [in1 out1] [in2 out2] ...");
                return;
            }
            int p = args.length / 2;
            in  = new String[p];
            out = new String[p];
            for (int i = 0, k = 0; i < p; ++i) {
                in[i]  = args[k++];
                out[i] = args[k++];
            }
        }

        // Run's each pair
        for (int i = 0; i < in.length; ++i){
            runAssignment(in[i], out[i]);
        }
    }
}