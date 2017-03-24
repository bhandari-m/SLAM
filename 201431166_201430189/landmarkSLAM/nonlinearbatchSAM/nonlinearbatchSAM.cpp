/*
* Adapted from the Pose2SLAMExample_graph example provided with GTSAM.
* 
* 	- Reads a .graph file (GTSAM format) and constructs a pose graph
* 	- Optimizes the pose graphs
* 	- Saves the graph in graphviz format for visualization (can be converted to PDF as well)
*/


#include <gtsam/slam/dataset.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/geometry/Pose2.h>
#include <fstream>
#include <time.h> 


using namespace std;
using namespace gtsam;


int main(int argc, char **argv){



	clock_t t;
	t = clock();

	// Read in file and build graph
	NonlinearFactorGraph::shared_ptr graph;
	Values::shared_ptr initial;

	SharedDiagonal model = noiseModel::Diagonal::Sigmas((Vector(3) << 0.05, 0.05, 5.0*M_PI/180.0)); // why ?

	
	string graphFile="/home/Downloads/gtsam-3.2.1/examples/Data/example.graph";
	boost::tie(graph, initial) = load2D(graphFile, model);  //  ???

	// Printing the initial estimate.
	initial->print("Initial estimate: ");
	cout<< endl;
	cout<< endl;


	
	// Add a Gaussian prior on the first pose
	Pose2 priorMean(0.0, 0.0, 0.0);
	SharedDiagonal priorSigma = noiseModel::Diagonal::Sigmas((Vector(3) << 0.01, 0.01, 0.01));
	graph->push_back(PriorFactor<Pose2>(0, priorMean, priorSigma));

	// Perform single step optimization using Levenberg-Marquardt
	Values result = LevenbergMarquardtOptimizer(*graph, *initial).optimize(); // Smoothing of entire trajectory??? and observation(estimated) ??
	result.print("Result: ");
	cout<< endl;cout<< endl;

	// Perform singel step optimization using Gauss-Newton Solver.
	
	//Values result = GaussNewtonOptimizer(*graph, *initial).optimize();
	//result.print("Result: ");



	// Save the initial graph,as pdf. Render to PDF using "fdp inputGraph.dot -Tpdf > inputGraph.pdf"
	ofstream inputGr("inputGraph.dot");
	graph->saveGraph(inputGr, *initial);

	// // Save the final graph
	ofstream outputGr("outputGraph.dot");
	graph->saveGraph(outputGr, result);

	// Write g2o file
	string outputGraph = "outputGraph.g2o";
	writeG2o(*graph, result, outputGraph);

	t = clock() - t;
	printf ("It took  %f seconds.\n",((float)t)/CLOCKS_PER_SEC);

	return 0;
}
