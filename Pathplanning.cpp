
#include <iostream>
#include <rw/rw.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

#define MAXTIME 10.

bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
	State testState;
	CollisionDetector::QueryResult data;
	bool colFrom;

	testState = state;
	device->setQ(q,testState);
	colFrom = detector.inCollision(testState,&data);
	if (colFrom) {
		std::cerr << "Configuration in collision: " << q << std::endl;
		std::cerr << "Colliding frames: " << std::endl;
		FramePairSet fps = data.collidingFrames;
		for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
			std::cerr << (*it).first->getName() << " " << (*it).second->getName() << std::endl;
		}
		return false;
	}
	return true;
}

int main(int argc, char** argv) {
	std::string wcFile = argv[1];
	std::string deviceName = argv[2];
	std::cout << "Trying to use workcell: " << wcFile << " and device " << deviceName << std::endl;

	WorkCell::Ptr wc = WorkCellLoader::load(wcFile);
	Device::Ptr device = wc->findDevice(deviceName);
	if (device == NULL) {
		std::cerr << "Device: " << deviceName << " not found!" << std::endl;
		return 0;
	}

    State state = wc->getDefaultState();

	Q from(6,-3.1415,-0.82,-3,-3.14,0.1,-3.1415/2);
	Q to(6,3.1415/2,0.01,0,0.15,0.8,4.5);

    state = wc->getDefaultState();
    device->setQ(from, state);
    
    Frame* gripperframe = device->getEnd();
    if (gripperframe == NULL) {
        std::cout << "Frame: " << "PG70.TCP" << " not found!" << std::endl;
        //res.valid_path = false;
    }
    
    MovableFrame* bottleframe = (MovableFrame*)wc->findFrame("Bottle");
    if (bottleframe == NULL) {
        std::cout << "Frame: " << "Bottle" << " not found!" << std::endl;
    }
    device->setQ(from,state);
    std::cout << device->getQ(state) << std::endl;
    Kinematics::gripFrame(bottleframe, gripperframe, state);
    std::cout << device->getQ(state) << std::endl;
         
    CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
	PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);
    
	    Math::seed();
	QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
	QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
	double extend = 0.1;
	QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);
   
	if (!checkCollisions(device, state, detector, from))
		return 0;
	if (!checkCollisions(device, state, detector, to))
		return 0;

	std::cout << "Planning from " << from << " to " << to << std::endl;
	QPath path;
	Timer t;
	t.resetAndResume();
	planner->query(from,to,path,MAXTIME);
	t.pause();
	std::cout << "Path of length " << path.size() << " found in " << t.getTime() << " seconds." << std::endl;
	if (t.getTime() >= MAXTIME) {
		std::cout << "Notice: max time of " << MAXTIME << " seconds reached." << std::endl;
	}
	for (QPath::iterator it = path.begin(); it < path.end(); it++) {
		std::cout << *it << std::endl;
	}

	std::cout << "Program done." << std::endl;
	return 0;
}
