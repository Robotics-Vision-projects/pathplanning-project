// Standard libraries
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
// Third party libraries
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
// RobWork libraries
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

using namespace std;
namespace fs = boost::filesystem;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

#define MAXTIME 60.
#define SPHERE_DIAMRETER  0.15f
#define SEED 10
#define EPSILON 0.08
#define PATHCOUNT 100


bool checkCollisions(Device::Ptr device, const State &state,
					 const CollisionDetector &detector, const Q &q) {
	State testState;
	CollisionDetector::QueryResult data;
	bool colFrom;

	testState = state;
	device->setQ(q,testState);
	colFrom = detector.inCollision(testState,&data);
	if (colFrom) {
		cerr << "Configuration in collision: " << q << endl;
		cerr << "Colliding frames: " << endl;
		FramePairSet fps = data.collidingFrames;
		for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
			cerr << (*it).first->getName() << " " << (*it).second->getName()
					<< endl;
		}
		return false;
	}
	return true;
}


bool printLuaScript(string dest_path, QPath& path)
{
    cout << "printing path of length " << path.size() << endl;

    ofstream luaScript(dest_path);
    if(luaScript.is_open()) {
        cout << "ERROR: failed to open/create rqt-path.lua";
        return false;
    }
    luaScript << "wc = rws.getRobWorkStudio():getWorkCell()\n"
                 "state = wc:getDefaultState()\n"
                 "device = wc:findDevice(\"KukaKr16\")\n"
                 "gripper = wc:findFrame(\"Tool\")\n"
                 "bottle = wc:findFrame(\"Bottle\")\n"
                 "table = wc:findFrame(\"Table\")\n"
                 "\n"
                 "function setQ(q) \n"
                 "qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])\n"
                 "device:setQ(qq,state)\n"
                 "rws.getRobWorkStudio():setState(state)\n"
                 "rw.sleep(0.1)\n"
                 "end\n"
                 "\n"
                 "function attach(obj, tool)\n"
                 "rw.gripFrame(obj, tool, state)\n"
                 "rws.getRobWorkStudio():setState(state)\n"
                 "rw.sleep(0.1)\n"
                 "end \n\n"
                 "setQ({-3.142, -0.827, -3.002, -3.143, 0.099, -1.573})\n"
                 "attach(bottle,gripper)\n";
    for (QPath::iterator it = ++path.begin(); it < path.end(); it++) {
        std::stringstream tmp;
        tmp << *it;
        luaScript << "setQ(" << tmp.str().substr(4) << ")" << endl;
    }
    luaScript << "attach(bottle,table)";
    luaScript.close();
    return true;
}

// searches for the shortest path with a given epsilon, and returns the
// shortest one after attemptNum attempts.
void searchBestPath(QPath& bestPath, QToQPlanner::Ptr& planner, Q& from, Q& to,
                    size_t attemptNum = 100)
{
    size_t shortest = SIZE_MAX;
    for(size_t i = 0; i < attemptNum; ++i)
    {
        QPath path;
        Timer t;
        t.resetAndResume();
        planner->query(from,to,path,MAXTIME);
        t.pause();
        if(t.getTime() < MAXTIME)
        {
            cout << "path length: " << path.size() << " time:" << t.getTime()
                    << endl;
            if(path.size() < shortest)
            {
                bestPath = path;
                shortest = bestPath.size();
                cout << "New best path length: " << bestPath.size() << endl;
            }
        }
        else
        {
            cout << "timeout" << endl;
        }
    }
}


int main(int argc, char** argv) {

    rw::math::Math::seed(SEED);

    // Get the path to the workcell, relative to the executable file.
    fs::path proj_path = fs::system_complete(
            fs::path(argv[0])).parent_path().parent_path();
    string wcFile = proj_path.string();
    wcFile.append("/Kr16WallWorkCell/Scene.wc.xml");

	const string deviceName = "KukaKr16";
    // Load both the workcell and the device.
	cout << "Trying to use workcell " << wcFile << " and device "
			<< deviceName << endl;
	WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
	Device::Ptr device = wc->findDevice(deviceName);

	if (device == NULL) {
		cerr << "Device: " << deviceName << " not found!" << endl;
		return 0;
	}

    State init_state(wc->getDefaultState());
    // Setting device to initial configuration state.
    device.get()->setQ(Q(6, -3.142, -0.827, -3.002, -3.143, 0.099, -1.573),
                       init_state);
    // gripping bottle with tool
    Kinematics::gripFrame(wc->findFrame("Bottle"), wc->findFrame("Tool"),
                          init_state);
    // starting state remains constant after this
    const State state(init_state);

	CollisionDetector detector(wc, 
			ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(&detector,
    													   device,state);

	QSampler::Ptr sampler = QSampler::makeConstrained(
			QSampler::makeUniform(device),constraint.getQConstraintPtr());
	QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
	QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler,
            metric, EPSILON, RRTPlanner::RRTConnect);

    // Get the state vectors for both the initial and end points of the path.
    Q from(6, -3.142, -0.827, -3.002, -3.143, 0.099, -1.573);
    Q to(6, 1.571, 0.006, 0.03, 0.153, 0.762, 4.49);
    // Exit the program if any collision is detected.
	if (!checkCollisions(device, state, detector, from))
		return 0;
	if (!checkCollisions(device, state, detector, to))
		return 0;

    cout << "Planning from " << from << " to " << to << endl;
    // Optimize the path between the 2 points, iterating 100 times.
    QPath path;
	Timer t;
    searchBestPath(path, planner, from, to, 100);

    string dest_path = proj_path.string();
    dest_path.append("/rqt-path.lua");
    printLuaScript(dest_path, path);

    // STATISTICS for epsilon
    std::vector<double> epsilons = {0.01, 0.02, 0.04, 0.06, 0.08, 0.12, 0.16,
                                    0.2, 0.3, 0.4, 0.5};
    ofstream pathStat(proj_path.string() + "/path-statistics.txt");
    size_t timeoutCount = 0;
    if(!pathStat.is_open())
    {
        cout << "ERROR: failed to open/create path-statistics.txt";
        return 0;
    }
    // For each epsilon, calculate PATHCOUNT times the path, and plot the
    // resulting statistics to a text file.
    for(auto epsilon : epsilons)
    {
        for(int i = 0; i < PATHCOUNT; ++i)
        {
            QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(
                    constraint, sampler, metric, epsilon,
                    RRTPlanner::RRTConnect);

            QPath path;
            Timer t;
            t.resetAndResume();
            planner->query(from,to,path,MAXTIME);
            t.pause();
            if(t.getTime() < MAXTIME)
            {
                pathStat << epsilon << " " << path.size() << " "<< t.getTime()
                        << endl;
            }
            else
            {
                ++timeoutCount;
            }
        }

    }
    pathStat.close();
    cout << "timeouts : " << timeoutCount << endl;
    // End the execution.
	cout << "Program done." << endl;
	return 0;
}
