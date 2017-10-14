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
#define EPSILON 0.16

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

int main(int argc, char** argv) {

    rw::math::Math::seed(SEED);

    // Get the path to the workcell, relative to the executable file.
    fs::path proj_path = fs::system_complete(
            fs::path(argv[0])).parent_path().parent_path();
    string wcFile = proj_path.string();
    wcFile.append("/Kr16WallWorkCell/Scene.wc.xml");

	const string deviceName = "KukaKr16";
	cout << "Trying to use workcell " << wcFile << " and device "
			<< deviceName << endl;

	WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
	Device::Ptr device = wc->findDevice(deviceName);

	if (device == NULL) {
		cerr << "Device: " << deviceName << " not found!" << endl;
		return 0;
	}

    State state = wc->getDefaultState();
    Kinematics::gripFrame(wc->findFrame("Bottle"), wc->findFrame("Tool"),
    					  state);

	CollisionDetector detector(wc, 
			ProximityStrategyFactory::makeDefaultCollisionStrategy());
    //rw::geometry::Cylinder cylinder(0.05f, 0.05f);
    detector.addGeometry(wc->findFrame("Bottle"),
    		rw::geometry::Geometry::makeSphere(SPHERE_DIAMRETER));
    PlannerConstraint constraint = PlannerConstraint::make(&detector,
    													   device,state);

	/** Most easy way: uses default parameters based on given device
		sampler: QSampler::makeUniform(device)
		metric: PlannerUtil::normalizingInfinityMetric(device->getBounds())
		extend: 0.05 */
	//QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, device,
	// RRTPlanner::RRTConnect);

	/* More complex way: allows more detailed definition of parameters and
	 * methods
	*/
	QSampler::Ptr sampler = QSampler::makeConstrained(
			QSampler::makeUniform(device),constraint.getQConstraintPtr());
	QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    double extend = EPSILON;
	QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler,
			metric, extend, RRTPlanner::RRTConnect);

    //Q from(6,-0.2,-0.6,1.5,0.0,0.6,1.2);
	//Q to(6,1.7,0.6,-0.8,0.3,0.7,-0.5); // Very difficult for planner
    //Q to(6,1.4,-1.3,1.5,0.3,1.3,1.6);
    Q from(6, -3.142, -0.827, -3.002, -3.143, 0.099, -1.573);
    Q to(6, 1.571, 0.006, 0.03, 0.153, 0.762, 4.49);

	if (!checkCollisions(device, state, detector, from))
		return 0;
	if (!checkCollisions(device, state, detector, to))
		return 0;

	cout << "Planning from " << from << " to " << to << endl;
	QPath path;
	Timer t;
	t.resetAndResume();
	planner->query(from,to,path,MAXTIME);
	t.pause();
	cout << "Path of length " << path.size() << " found in " << t.getTime()
			<< " seconds." << endl;
	if (t.getTime() >= MAXTIME) {
		cout << "Notice: max time of " << MAXTIME << " seconds reached."
				<< endl;
	}

    string dest_path = proj_path.string();
    dest_path.append("/rqt-path.lua");
    ofstream luaScript(dest_path);
    if(luaScript.is_open())
    {
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
					 "attach(bottle,gripper)";
        for (QPath::iterator it = path.begin()++; it < path.end(); it++) {
            std::stringstream tmp;
            tmp << *it;
            luaScript << "setQ(" << tmp.str().substr(4) << ")" << endl;
        }
        luaScript << "attach(bottle,table)";
    }
 
	cout << "Program done." << endl;
	return 0;
}
