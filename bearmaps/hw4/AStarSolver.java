package bearmaps.hw4;

import java.util.*;
import java.util.LinkedList;


import bearmaps.proj2ab.ArrayHeapMinPQ;
import bearmaps.proj2ab.DoubleMapPQ;
import edu.princeton.cs.algs4.*;
public class AStarSolver<Vertex> implements ShortestPathsSolver<Vertex>{
    private double timeSpent;
    private ArrayHeapMinPQ<Vertex> pq; /** priority queue*/
    private HashSet<Vertex> visited; /** store all the visited Vertex*/
    private HashMap<Vertex,Double> distTo;
    private HashMap<Vertex,Double> AStarMap; /**to store estimated distance from vertex to to end*/
    private HashMap<Vertex,Vertex> edges; /**to keep track of the edges that are optimal */
    int numScouted; /**number of states explored*/
    private SolverOutcome outcome; /** to return the 1 of 3 possible outcomes*/
    private double solutionWeight; /**to return in solutionWeight()*/
    private LinkedList<Vertex> vertexSolution;
    List<WeightedEdge<Vertex>> neighbors; /**to store weighted edges from each vertices*/

    /**  Constructor which finds the solution, computing everything necessary for all other methods to return their
     * results in constant time.
     * timeout is passed in seconds*/
    public AStarSolver(AStarGraph<Vertex> input, Vertex start, Vertex end, double timeout){
        Stopwatch sw = new Stopwatch();
        pq = new ArrayHeapMinPQ<>();
        visited = new HashSet<>();
        AStarMap = new HashMap<>();
        distTo = new HashMap<>();
        edges = new HashMap<>();/**to keep track of edges you've gone through <to,from>*/
        vertexSolution = new LinkedList<>();
        numScouted = 0; /** number of vertex you have gone through*/
        Vertex currVertex; /** to keep track of current vertex*/
        /** source vertex added first with estimated goal as priority. later vertex needs heurisitic estimate
         * added into the priority*/
        pq.add(start, input.estimatedDistanceToGoal(start,end));
        distTo.put(start,0.0);
        do {
            currVertex = pq.getSmallest(); /** move to vertex with smallest distance*/
            if (sw.elapsedTime() >= timeout) {
                outcome = SolverOutcome.TIMEOUT; /**solver ran out of time*/
                solutionWeight = 0.0;
                timeSpent = sw.elapsedTime();
                vertexSolution.clear();
                return;
            }
            /**if vertex is at the desitnation*/
            if (currVertex.equals(end)) {
                /**store the latest weight from source*/
                solutionWeight = distTo.get(currVertex);
                /**Reverse the vertexSolution*/
                while(currVertex!=null){
                    vertexSolution.add(currVertex);
                    currVertex=edges.get(currVertex);
                }
                Collections.reverse(vertexSolution);
                outcome = SolverOutcome.SOLVED; /**completed all the work in time given */
                timeSpent = sw.elapsedTime();
                return;
            }

            /**relax neighbors of pq lowest priority*/
            /**deque is done, should increment numStatesExplored*/
            pq.removeSmallest();
            numScouted++; /**The total number of priority queue dequeue operations*/
            neighbors=input.neighbors(currVertex);
            for(WeightedEdge<Vertex> e: neighbors){
                relax(input,e,end);
            }
        }while(pq.size()>0);
        /**Unsolvable so reset everytime and set timeSpet to elasped time*/
        outcome = SolverOutcome.UNSOLVABLE; /**PQ became empty*/
        solutionWeight=0.0;
        solution().clear();
        timeSpent=sw.elapsedTime();
    }
    private void relax(AStarGraph<Vertex> input, WeightedEdge<Vertex> n,Vertex end){
        /**get vertex from,to and the weight*/
        Vertex from = n.from();
        Vertex to = n.to();
        double weight = n.weight();
        /**check if vertex of to has been reached before
         * if not, set it to infinity
         * */
        if(!distTo.containsKey(to)) distTo.put(to,Double.POSITIVE_INFINITY);
        /**if from vertex is unreached, then add to map with heurisitic distance*/
        if(!AStarMap.containsKey(to)){
            AStarMap.put(to,input.estimatedDistanceToGoal(to,end));
        }
        /**calulate distance with weight*/
        double dist = distTo.get(from)+weight;
        /**if new distance is closer then replace*/
        if(dist<distTo.get(to)){
            edges.put(to,from); /**store vertices from to to from to be used later in vertexSolution*/
            distTo.replace(to,dist); /**replace infinity with new distance*/
            double addedDist = distTo.get(to)+ AStarMap.get(to); /**distance of vertex to until vertex end*/
            if(!pq.contains(to)){
                pq.add(to,addedDist);
            }else{
                pq.changePriority(to,addedDist);
            }
        }
    }
    public SolverOutcome outcome(){
        return outcome;
    }
    public List<Vertex> solution(){
        if(outcome() == SolverOutcome.TIMEOUT || outcome == SolverOutcome.UNSOLVABLE) {
            throw new NoSuchElementException("No solution because outcome was TIMEOUT or UNSOLVABLE");
        }
        return vertexSolution;
    }
    public double solutionWeight(){
        if(outcome() == SolverOutcome.TIMEOUT || outcome == SolverOutcome.UNSOLVABLE) return 0;
            return solutionWeight;
    }
    public int numStatesExplored(){
        return numScouted;
    }
    public double explorationTime(){
        return timeSpent;
    }
}