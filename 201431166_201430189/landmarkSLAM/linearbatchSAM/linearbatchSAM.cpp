/**
  Incremental SAM example using odometry measurements and bearing-range (laser) measurements
 */

/**
 * A simple 2D planar iSAM example with landmarks
 *  - Data from the robot's run (odometry and range-bearing observations) are present in example.graph
 *  - We use a BetweenFactor to represent an odometry observation
 *  - We use a BearingRangeFactor to represent a range-bearing observation
 *  - We use ISAM2 (iSAM with the Bayes Tree graphical model) to represent the problem and for inference
 */

// As this is a planar SLAM example, we will use Pose2 variables (x, y, theta) to represent
// the robot positions and Point2 variables (x, y) to represent the landmark coordinates.
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use a RangeBearing factor for the range-bearing measurements to identified
// landmarks, and Between factors for the relative motion described by odometry measurements.
// Also, we will initialize the robot at the origin using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// common Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

// Utilities to read a .graph file and convert to a factor graph format
#include <gtsam/slam/dataset.h>

// We are using ISAM2 here
#include <gtsam/nonlinear/ISAM2.h>

// Utilities to compute GTSAM execution times
#include <gtsam/base/timing.h>
#include <fstream>
#include <time.h> 



using namespace std;
using namespace gtsam;


int main(int argc, char **argv){


  clock_t t;
  t = clock();
  
  // Input .graph filename
  string filename = "/home/Downloads/gtsam-3.2.1/examples/Data/example.graph";


  // Struct to hold ISAM2 parameters
  ISAM2Params params;

  // We're using the dogleg method here
  //params.relinearizeSkip = 10;
  params.optimizationParams = ISAM2DoglegParams();

  // Number of steps after which relinearization is to be performed (default = 10)
  
  params.enablePartialRelinearizationCheck = true;

  // Initialize an ISAM2 problem with the parameters specified
  ISAM2 isam2(params);

  // Load the dataset into a nonlinear factor graph structure
  std::pair<NonlinearFactorGraph::shared_ptr, Values::shared_ptr> data = load2D(filename);
  NonlinearFactorGraph dataset;
  dataset = *data.first;

  // For linearbatchSAM
  params.relinearizeSkip = dataset.size()-1;
  

  // TODO : For pdf purpose nad displaying o/ps
  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initial;
  SharedDiagonal model = noiseModel::Diagonal::Sigmas((Vector(3) << 0.05, 0.05, 5.0*M_PI/180.0));
  boost::tie(graph, initial) = load2D(filename, model);
  // Printing the initial estimate.
  initial->print("Initial estimate: ");





  // First time step (time from which to process the data file (set this to zero))
  int firstStep = 0;
  // Last time step (time up to which to process the data file (set this to -1))
  int lastStep = -1;

  // Looking for the first observation
  size_t nextObs = 0;
  bool havePreviousPose = false;
  Key firstPose;

  // As long as there is an observation to be added to the ISAM problem
  while(nextObs < dataset.size()){

    // If the observation is an odometry observation, it leads to a BetweenFactor
    if(BetweenFactor<Pose2>::shared_ptr observation = boost::dynamic_pointer_cast<BetweenFactor<Pose2> >(dataset[nextObs])){
      
      // Get the keys from the observation
      Key key1 = observation->key1();
      Key key2 = observation->key2();
      
      if((key1 >= firstStep && key1 < key2) || (key2 >= firstStep && key2 < key1)) {
        // This is an odometry starting at firstStep
        firstPose = std::min(key1, key2);
        break;
      }

      if((key2 >= firstStep && key1 < key2) || (key1 >= firstStep && key2 < key1)) {
        // This is an odometry connecting firstStep with a previous pose
        havePreviousPose = true;
        firstPose = std::max(key1, key2);
        break;
      }

    }

    ++ nextObs;

  }
  //Were not able to find the firstPose
  if (nextObs == dataset.size()){
    std::cout << "The supplied first step is past the end of the dataset." << std::endl;
    exit(1);
  }

  // If we didn't find an odometry linking to a previous pose, create a first pose and a prior
  if(!havePreviousPose){

    NonlinearFactorGraph newFactors;
    Values newVariables;

    SharedDiagonal priorNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.01, 0.01, 0.01));
    // newFactors.push_back(boost::make_shared<PriorFactor<Pose2> >(firstPose, Pose2(), priorNoise));
    newFactors.push_back(boost::make_shared<PriorFactor<Pose2> >(firstPose, Pose2(), noiseModel::Unit::Create(Pose2::Dim())));
    newVariables.insert(firstPose, Pose2());

    // Update the ISAM2 problem
    isam2.update(newFactors, newVariables);
  }


  // For each time step (running ISAM2)
  // for(size_t step = firstPose; nextObs < dataset.size() && (lastStep == -1 || step <= lastStep); ++step){
  
  for(size_t step = firstPose; nextObs < dataset.size(); ++step){ // ??

    // Structs to hold new variables and factors
    Values newVariables;
    NonlinearFactorGraph newFactors;

    // Collect the observations and the variables for the current step
    // Start timing
    // gttic_(Collect_observations);
    while(nextObs < dataset.size()){

      // Get the current factor from the dataset
      NonlinearFactor::shared_ptr obsFactor = dataset[nextObs];
      // obsFactor->print("");

      // If the factor corresponds to an odometry observation
      if(BetweenFactor<Pose2>::shared_ptr observation =  boost::dynamic_pointer_cast<BetweenFactor<Pose2> >(obsFactor)){

        // Stop collecting observations that are for future steps ??
        if(observation->key1() > step || observation->key2() > step){ // key1 key2 ???
          break;
        }

        // Require that one of the nodes is the current one
        if(observation->key1() != step && observation->key2() != step){
          throw runtime_error("Problem in data file, out-of-sequence observations!");
        }

        // Add a new factor
        newFactors.push_back(observation);

        // Initialize the new variable
        if(observation->key1() > observation->key2()){ // ???

          if(!newVariables.exists(observation->key1())){ // like checking in a map
            if(step == 1){ // ??
              newVariables.insert(observation->key1(), observation->measured().inverse()); // like storing in map
            }
            else{
              Pose2 previousPose = isam2.calculateEstimate<Pose2>(observation->key2());
              newVariables.insert(observation->key1(), previousPose * observation->measured().inverse());
            }
          }

        }

        else {

          if(!newVariables.exists(observation->key2())){
            
            if (step == 1){
              newVariables.insert(observation->key2(), observation->measured());
            }
            else{
              Pose2 previousPose = isam2.calculateEstimate<Pose2>(observation->key1());
              newVariables.insert(observation->key2(), previousPose * observation->measured());
            }

          }

        }

      }

      // If the factor corresponds to a range-bearing observation
      else if(BearingRangeFactor<Pose2, Point2>::shared_ptr observation = boost::dynamic_pointer_cast<BearingRangeFactor<Pose2, Point2> >(obsFactor)){

        Key poseKey = observation->keys()[0];
        Key lmKey = observation->keys()[1];

        // Add new factor
        newFactors.push_back(observation);

        // Initialize landmark
        if(!newVariables.exists(lmKey) && !isam2.getLinearizationPoint().exists(lmKey)){ // y check in both
          
          Pose2 pose;
          // ?? get value of pose (value ?? :-O)
          if(isam2.getLinearizationPoint().exists(poseKey)){
            pose = isam2.calculateEstimate<Pose2>(poseKey);
          }
          else{
            pose = newVariables.at<Pose2>(poseKey);
          }

          // getting the value of range and bearing
          Rot2 measuredBearing = observation->measured().first;
          double measuredRange = observation->measured().second;
          //inserting from where taking it (pose)  in new variables.
          newVariables.insert(lmKey,pose.transform_from(measuredBearing.rotate(Point2(measuredRange, 0.0))));

        }

        // // The below code snippet does not handle all corner cases.
        // if(!isam2.getLinearizationPoint().exists(lmKey)){
        //   Pose2 pose;
        //   if(isam2.getLinearizationPoint().exists(poseKey))
        //     pose = isam2.calculateEstimate<Pose2>(poseKey);
        //   else
        //     pose = newVariables.at<Pose2>(poseKey);
        //   Rot2 measuredBearing = observation->measured().first;
        //   double measuredRange = observation->measured().second;
        //   newVariables.insert(lmKey, 
        //     pose.transform_from(measuredBearing.rotate(Point2(measuredRange, 0.0))));
        // }

      }

      // Else, it is an unknown factor type
      else{
        throw std::runtime_error("Unknown factor type read from data file.");
      }

      ++ nextObs;
    }
    // Stop the timer
    // gttoc_(Collect_observations);

    isam2.update(newFactors, newVariables);

    if((step - firstPose) % params.relinearizeSkip == 0){
      Values estimate(isam2.calculateEstimate());
      estimate.print("");
    }


  } // Ending of the for loop.

  // TODO : 
  // Save the initial graph,as pdf. Render to PDF using "fdp inputGraph.dot -Tpdf > inputGraph.pdf"
  ofstream inputGr("inputGraph.dot");
  graph->saveGraph(inputGr, *initial);

  Values result(isam2.calculateEstimate());
  result.print("Result: ");

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

