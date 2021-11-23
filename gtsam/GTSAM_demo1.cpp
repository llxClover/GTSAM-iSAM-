#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
using namespace std;
using namespace gtsam;
    /**                           @5**********@4
     *                            *           *             
     *                            *           *
     *                            *           *
     *      **********@1**********@2**********@3
     * 
     */ 
int main(int argc, char** argv) 
{
    NonlinearFactorGraph graph;
 
    Values initials;

    // 给5个状态因子赋初值（相当于没有进行图优化前的状态因子）
    initials.insert(Symbol('x', 1), Pose2(0.2, -0.3, 0.2));
    initials.insert(Symbol('x', 2), Pose2(5.1, 0.3, -0.1));
    initials.insert(Symbol('x', 3), Pose2(9.9, -0.1, -M_PI_2 - 0.2));
    initials.insert(Symbol('x', 4), Pose2(10.2, -5.0, -M_PI + 0.1));
    initials.insert(Symbol('x', 5), Pose2(5.1, -5.1, M_PI_2 - 0.1));
    initials.print("\nInitial Values:\n"); 
    
    //固定第一个顶点，在gtsam中相当于添加一个先验因子（相当于起始位置x1）
    noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Sigmas(Vector3(1.0, 1.0, 0.1));
    graph.add(PriorFactor<Pose2>(Symbol('x', 1), Pose2(0, 0, 0), priorModel));
    
    //二元位姿因子（相当于状态约束）
    noiseModel::Diagonal::shared_ptr odomModel = noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.1));
    graph.add(BetweenFactor<Pose2>(Symbol('x', 1), Symbol('x', 2), Pose2(5, 0, 0), odomModel));
    graph.add(BetweenFactor<Pose2>(Symbol('x', 2), Symbol('x', 3), Pose2(5, 0, -M_PI_2), odomModel));
    graph.add(BetweenFactor<Pose2>(Symbol('x', 3), Symbol('x', 4), Pose2(5, 0, -M_PI_2), odomModel));
    graph.add(BetweenFactor<Pose2>(Symbol('x', 4), Symbol('x', 5), Pose2(5, 0, -M_PI_2), odomModel));
 
    //二元回环因子（相当于回环约束）
    noiseModel::Diagonal::shared_ptr loopModel = noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.1));
    graph.add(BetweenFactor<Pose2>(Symbol('x', 5), Symbol('x', 2), Pose2(5, 0, -M_PI_2), loopModel));
    graph.print("\nFactor Graph:\n"); 
 
 
    GaussNewtonParams parameters;
    parameters.setVerbosity("ERROR");
    parameters.setMaxIterations(20);
    parameters.setLinearSolverType("MULTIFRONTAL_QR");
    GaussNewtonOptimizer optimizer(graph, initials, parameters);
    Values results = optimizer.optimize();
    results.print("Final Result:\n");
 
    Marginals marginals(graph, results);
    cout << "x1 covariance:\n" << marginals.marginalCovariance(Symbol('x', 1)) << endl;
    cout << "x2 covariance:\n" << marginals.marginalCovariance(Symbol('x', 2)) << endl;
    cout << "x3 covariance:\n" << marginals.marginalCovariance(Symbol('x', 3)) << endl;
    cout << "x4 covariance:\n" << marginals.marginalCovariance(Symbol('x', 4)) << endl;
    cout << "x5 covariance:\n" << marginals.marginalCovariance(Symbol('x', 5)) << endl;
 
    return 0;
}

