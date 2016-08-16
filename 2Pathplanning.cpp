

#include <iostream>
#include <rw/rw.hpp>
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
using namespace std;

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

const State state = wc->getDefaultState();

	
   static void gripFrame (bottleframe, gripperframe, state)  {
		WorkCell::Ptr wc = WorkCellLoader::load(wcFile);
	Device::Ptr device = wc->findDevice(ToolUnit);
	if (device == NULL) {
	std::cout << "Device: " << deviceName << " not found!" << std::endl;
	res.valid_path = false;
	}
	//State state = wc->getDefaultState();
	//const State state = wc->getDefaultState();
	//device->setQ(grip, state);
	

   Frame* gripperframe = device->getEnd();
	if (gripperframe == NULL) {
	std::cout << "Frame: " << "PG70.TCP" << " not found!" << std::endl;
	res.valid_path = false;
	}


	MovableFrame* bottleframe = (MovableFrame*)wc->findFrame("Bottle");
	if (bottleframe == NULL) {
	std::cout << "Frame: " << "Bottle" << " not found!" << std::endl;
	res.valid_path = false;
	}
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

	


	
	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
	PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);

	/** Most easy way: uses default parameters based on given device
		sampler: QSampler::makeUniform(device)
		metric: PlannerUtil::normalizingInfinityMetric(device->getBounds())
		extend: 0.05 */
	//QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, device, RRTPlanner::RRTConnect);

	/** More complex way: allows more detailed definition of parameters and methods */
	QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
	QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
	double extend = 0.1;
	QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

	//Q from(6,-0.2,-0.6,1.5,0.0,0.6,1.2);
	Q pick(6,3.1415,-0.82,-3,-3.14,0.1,-3.1415/2);
	//Q to(6,1.7,0.6,-0.8,0.3,0.7,-0.5); // Very difficult for planner
	//Q to(6,1.4,-1.3,1.5,0.3,1.3,1.6);
	Q place(6,3.1415/2,0.02,0,0.15,0.8,4.5);
	//Q aha(6,0.042,0.718,0.051,-0.141,0.747,-1.350);
    
    
    
    
 /*   //-------------------------------------------------------------
	WorkCell::Ptr wc = WorkCellLoader::load(wcFile);
	Device::Ptr device = wc->findDevice(Tool);
	if (device == NULL) {
	std::cout << "Device: " << Tool << " not found!" << std::endl;
	res.valid_path = false;
	}
	// const State state = wc->getDefaultState();
	State state = wc->getDefaultState();
	device->setQ(grip, state);
	
	
   Frame* gripperframe = device->getEnd();
	if (gripperframe == NULL) {
	std::cout << "Frame: " << "PG70.TCP" << " not found!" << std::endl;
	res.valid_path = false;
	}


	MovableFrame* bottleframe = (MovableFrame*)wc->findFrame("Bottle");
	if (bottleframe == NULL) {
	std::cout << "Frame: " << "Bottle" << " not found!" << std::endl;
	res.valid_path = false;
	}
*/	//------------------------------------------------------------------

 device->setQ(from, state);
 Kinematics::gripFrame(bottleframe, gripperframe, state);




    if (!checkCollisions(device, state, detector, pick))
		return 0;
	if (!checkCollisions(device, state, detector, place))
		return 0;
		

	std::cout << "Planning from " << pick << " to " << place << std::endl;
	QPath path;
	Timer t;
	t.resetAndResume();
	planner->query(pick,place,path,MAXTIME);
	t.pause();
	std::cout << "Path of length " << path.size() << " found in " << t.getTime() << " seconds." << std::endl;
	if (t.getTime() >= MAXTIME) {
		std::cout << "Notice: max time of " << MAXTIME << " seconds reached." << std::endl;
		}
		for (QPath::iterator it = path.begin(); it < path.end(); it++) {
		std::cout << *it << std::endl;
		}










/*	if (!checkCollisions(device, state, detector, from))
		return 0;
	if (!checkCollisions(device, state, detector, aha))
		return 0;
		

	std::cout << "Planning from " << from << " to " << aha << std::endl;
	QPath path;
	Timer t;
	t.resetAndResume();
	planner->query(from,aha,path,MAXTIME);
	t.pause();
	std::cout << "Path of length " << path.size() << " found in " << t.getTime() << " seconds." << std::endl;
	if (t.getTime() >= MAXTIME) {
		std::cout << "Notice: max time of " << MAXTIME << " seconds reached." << std::endl;
		}
		for (QPath::iterator it = path.begin(); it < path.end(); it++) {
		std::cout << *it << std::endl;
		}


	if (!checkCollisions(device, state, detector, aha))
		return 0;
	if (!checkCollisions(device, state, detector, to))
		return 0;
		

	std::cout << "Planning from " << aha << " to " << to << std::endl;
	//QPath path;
	//Timer t;
	t.resetAndResume();
	planner->query(aha,to,path,MAXTIME);
	t.pause();
	std::cout << "Path of length " << path.size() << " found in " << t.getTime() << " seconds." << std::endl;
	if (t.getTime() >= MAXTIME) {
		std::cout << "Notice: max time of " << MAXTIME << " seconds reached." << std::endl;
		}
		for (QPath::iterator it = path.begin(); it < path.end(); it++) {
		std::cout << *it << std::endl;
		}  */
	
	std::cout << "Program done." << std::endl;
	return 0;
}
