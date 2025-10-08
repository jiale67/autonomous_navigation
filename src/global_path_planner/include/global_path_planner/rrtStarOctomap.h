/*
*	File: rrtStarOctomap.h
*	---------------
*   RRT star planner class based on Octomap.
	Modified by Rjl on 2025.9.27
*/
#ifndef RRTSTATOCTOMAP_H
#define RRTSTAROCTOMAP_H
#include <global_path_planner/rrtOctomap.h>

namespace globalPlanner{
	template <std::size_t N>
	class rrtStarOctomap : public rrtOctomap<N>{
	private:
		ros::NodeHandle nh_;
		double rNeighborhood_; // region for rewiring
		double maxNeighbors_; // maximum value for neighboorhood
		std::unordered_map<KDTree::Point<N>, double, KDTree::PointHasher> distance_;

	public:
		// constructor using ros param:
		rrtStarOctomap(const ros::NodeHandle& nh);
	
		// constructor:
		rrtStarOctomap(const ros::NodeHandle& nh, double rNeighborhood, double maxNeighbors, std::vector<double> collisionBox, std::vector<double> envBox, double mapRes, double delQ=0.3, double dR=0.2, double connectGoalRatio=0.10, double timeout=1.0, bool visPath=true);
		
		void mapCB(const octomap_msgs::Octomap &msg);

		// get neighborhood points from the vector
		void getNeighborhood(const KDTree::Point<N>& q, std::vector<KDTree::Point<N>>& neighborhood);

		// get distance to start if connecting to q
		double getDistanceToStart(const KDTree::Point<N>& qNew, const KDTree::Point<N>& q);

		// get distance to start point at specific node:
		double getDistanceToStart(const KDTree::Point<N>& q);

		// record the distance to start based on parent point and current point:
		void updateDistanceToStart(const KDTree::Point<N>& qNew, const KDTree::Point<N>& qParent);

		// *** Core function: make plan based on all input ***
		virtual void makePlan(std::vector<KDTree::Point<N>>& plan);
		virtual void makePlan(nav_msgs::Path& plan);

		// return neighborhood radius:
		double getNeighborHoodRadius();
	};
	
	// ============function definition===================
	template <std::size_t N>
	rrtStarOctomap<N>::rrtStarOctomap(const ros::NodeHandle& nh) : rrtOctomap<N>(), nh_(nh){
		// Map Resolution for Collisiong checking
		if (not this->nh_.getParam("map_resolution", this->mapRes_)){
			this->mapRes_ = 0.2;
			cout << "[RRTPlanner]: No Map Resolition Parameter. Use default map resolution: 0.2." << endl;
		}

		// Visualize Path
		if (not this->nh_.getParam("vis_path", this->visPath_)){
			this->visPath_ = true;
			cout << "[RRTPlanner]: Visualize Path by default." << endl;
		}

		// Collision Box
		if (not this->nh_.getParam("collision_box", this->collisionBox_)){
			std::vector<double> defaultCollisionBox {1.0, 1.0, 0.6};
			this->collisionBox_ = defaultCollisionBox;
			cout << "[RRTPlanner]: No Collision Box Parameter. Use default collision Box: [1.0, 1.0, 0.6]." << endl;
		}

		// Environment Size (maximum)
		if (not this->nh_.getParam("env_box", this->envBox_)){
			std::vector<double> defaultEnvBox {-100, 100, -100, 100, 0, 1.5};
			cout << "[RRTPlanner]: No Environment Box Parameter. Use default env box: [-100, 100, -100, 100, 0, 1.5]." << endl;
		}

		// Incremental Distance (For RRT)
		if (not this->nh_.getParam("rrt_incremental_distance", this->delQ_)){
			this->delQ_ = 0.3;
			cout << "[RRTPlanner]: No RRT Incremental Distance Parameter. Use default value: 0.3m." << endl;
		}

		// Goal Reach Distance
		if (not this->nh_.getParam("goal_reach_distance", this->dR_)){
			this->dR_ = 0.4;
			cout << "[RRTPlanner]: No RRT Goal Reach Distance Parameter. Use default value: 0.4m." << endl;
		}

		// RRT Connect Goal Ratio
		if (not this->nh_.getParam("rrt_connect_goal_ratio", this->connectGoalRatio_)){
			this->connectGoalRatio_ = 0.2;
			cout << "[RRTPlanner]: No RRT Connect Goal Ratio Parameter. Use default value: 0.2." << endl;
		}

		// Time out:
		if (not this->nh_.getParam("timeout", this->timeout_)){
			this->timeout_ = 2.0;
			cout << "[RRTPlanner]: No Timeout Parameter. Use default value: 3.0s." << endl;
		}

		// ignore unknown voxel
		if (not this->nh_.getParam("ignore_unknown", this->ignoreUnknown_)){
			this->ignoreUnknown_ = false;
			cout << "[RRTPlanner]: No Ignore Unknown Parameter. Use default: false." << endl;
		}

		// maximum shortcut threshold
		if (not this->nh_.getParam("max_shortcut_dist", this->maxShortcutThresh_)){
			this->maxShortcutThresh_ = 5.0;
			cout << "[RRTPlanner]: No Max Shortcut Distance Threshold Parameter. Use default: 5.0." << endl;
		}


		// Neighborhood radius:
		if (not this->nh_.getParam("neighborhood_radius", this->rNeighborhood_)){
			this->rNeighborhood_ = 1.0;
			cout << "[RRTPlanner]: No Neighborhood Radius. Use default value: 1.0m." << endl;
		}

		// Maximum Number of Neighbors:
		if (not this->nh_.getParam("max_num_neighbors", this->maxNeighbors_)){
			this->maxNeighbors_ = 10;
			cout << "[RRTPlanner]: No Max Number of Neighbors Paramter. Use default: 10." << endl;
		}


		this->visRRT_ = false;		

		this->mapClient_ = this->nh_.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary");
		// this->updateMap();

		this->mapSub_ = this->nh_.subscribe("/octomap_full", 1, &rrtStarOctomap::mapCB, this);
		ros::Rate r (10);
		while (ros::ok() and this->map_ == NULL){
			cout << "[RRTPlanner]: Wait for Map..." << endl;
			ros::spinOnce();
			r.sleep();
		}
		cout << "[RRTPlanner]: Map Updated!" << endl;
		
		// Visualization:
		this->startVisModule();
		this->notUpdateSampleRegion_ = false;
	}

	template <std::size_t N>
	rrtStarOctomap<N>::rrtStarOctomap(const ros::NodeHandle& nh, double rNeighborhood, double maxNeighbors, std::vector<double> collisionBox, std::vector<double> envBox, double mapRes, double delQ, double dR, double connectGoalRatio, double timeout, bool visPath)
	: nh_(nh), rNeighborhood_(rNeighborhood), maxNeighbors_(maxNeighbors),  rrtOctomap<N>(collisionBox, envBox, mapRes, delQ, dR, connectGoalRatio, timeout, false, visPath){
		this->mapClient_ = this->nh_.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary");
		// this->updateMap();

		// Visualization:
		this->startVisModule();
	}

	template <std::size_t N>
	void rrtStarOctomap<N>::mapCB(const octomap_msgs::Octomap &msg){
    	octomap::OcTree* treePtr = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(msg));
   	 	this->map_ = std::shared_ptr<octomap::OcTree>(treePtr);		
   	 	double min_x, max_x, min_y, max_y, min_z, max_z;
		this->map_->getMetricMax(max_x, max_y, max_z);
		this->map_->getMetricMin(min_x, min_y, min_z);
		this->envLimit_[0] = min_x; this->envLimit_[1] = max_x; this->envLimit_[2] = min_y; this->envLimit_[3] = max_y; this->envLimit_[4] = min_z; this->envLimit_[5] = max_z;
		// this->updateSampleRegion();
	}

	template <std::size_t N>
	void rrtStarOctomap<N>::getNeighborhood(const KDTree::Point<N>& q, std::vector<KDTree::Point<N>>& neighborhood){
		this->ktree_.boundedRangeSearch(q, this->rNeighborhood_, this->maxNeighbors_, neighborhood);
	}

	template <std::size_t N>
	double rrtStarOctomap<N>::getDistanceToStart(const KDTree::Point<N>& qNew, const KDTree::Point<N>& q){
		return this->distance_[q] + KDTree::Distance(qNew, q);
	}

	template <std::size_t N>
	double rrtStarOctomap<N>::getDistanceToStart(const KDTree::Point<N>& q){
		return this->distance_[q];
	}

	template <std::size_t N>
	void rrtStarOctomap<N>::updateDistanceToStart(const KDTree::Point<N>& qNew, const KDTree::Point<N>& qParent){
		this->distance_[qNew] = this->getDistanceToStart(qNew, qParent);
	}


	template <std::size_t N>
	void rrtStarOctomap<N>::makePlan(std::vector<KDTree::Point<N>>& plan){
		if (not this->notUpdateSampleRegion_){
			this->updateSampleRegion();
		}
		if (this->visPath_){
			this->pathVisVec_.clear();
			this->pathVisMsg_.markers = this->pathVisVec_;
		}

		bool findPath = false;
		bool timeout = false;
		ros::Time startTime = ros::Time::now();
		double dT;
		int sampleNum = 0;
		KDTree::Point<N> qBack;

		cout << "[RRTPlanner]: Start planning!" << endl;
		double nearestDistance = std::numeric_limits<double>::max();  // if cannot find path to goal, find nearest way to goal
		KDTree::Point<N> nearestPoint = this->start_;
		double currentDistance = KDTree::Distance(nearestPoint, this->goal_);
		this->addVertex(this->start_);
		this->distance_[this->start_] = 0; // init distance to start
		while (ros::ok() and not findPath and not timeout){	
			ros::Time currentTime = ros::Time::now();
			dT = (currentTime - startTime).toSec();
			if (dT >= this->timeout_){
				timeout = true;
			}

			// 1. sample:
			KDTree::Point<N> qRand;
			double randomValue = randomNumber(0, 1);
			if (randomValue >= this->connectGoalRatio_){ // random sample trick
				this->randomConfig(qRand);
			}
			else{
				qRand = this->goal_;
			}

			// 2. find nearest neighbor:
			KDTree::Point<N> qNear;
			this->nearestVertex(qRand, qNear);

			// 3. new config by steering function:
			KDTree::Point<N> qNew;
			this->newConfig(qNear, qRand, qNew);


			// 4. Add new config to vertex and edge:
			if (this->hasNoEdge(qNear, qNew) and not this->checkCollisionLine(qNew, qNear)){
				// RRT* rewiring 1;
				KDTree::Point<N> qBestParent = qNear;
				double minDistance = this->getDistanceToStart(qNew, qNear);
				std::vector<KDTree::Point<N>> neighborhood;
				this->getNeighborhood(qNew, neighborhood);
				for (KDTree::Point<N> qNeighbor: neighborhood){
					double neighborDistance = this->getDistanceToStart(qNew, qNeighbor);
					if (neighborDistance < minDistance){
						if (not this->checkCollisionLine(qNew, qNeighbor)){
							minDistance = neighborDistance;
							qBestParent = qNeighbor;
						}
					}
				}
				this->addVertex(qNew);
				this->addEdge(qBestParent, qNew);
				this->updateDistanceToStart(qNew, qBestParent); // record the distance

				// RRT* rewiring 2:
				for (KDTree::Point<N> qNeighbor: neighborhood){
					double qNeighborDistance = this->getDistanceToStart(qNeighbor);
					double qNeighborDistanceNew = this->getDistanceToStart(qNeighbor, qNew);
					if (qNeighborDistanceNew < qNeighborDistance){
						if (not this->checkCollisionLine(qNeighbor, qNew)){
							this->addEdge(qNew, qNeighbor);
							this->updateDistanceToStart(qNeighbor, qNew);
						}
					}
				} 


				++sampleNum;

				// 5. check whether goal has been reached
				findPath = this->isReach(qNew);
				if (findPath){
					qBack = qNew;
				}
				else{
					currentDistance = KDTree::Distance(qNew, this->goal_);
					if (currentDistance < nearestDistance){
						nearestDistance = currentDistance;
						nearestPoint = qNew;
					}
				}

			}
		}
		cout << "[RRTPlanner]: Finish planning. with sample number: " << sampleNum << endl;

		// final step: back trace using the last one
		std::vector<KDTree::Point<N>> planRaw;
		if (findPath){
			this->backTrace(qBack, planRaw);
			cout << "[RRTPlanner]: path found! Time: " << dT << "s."<< endl;
		}
		else{
			this->backTrace(nearestPoint, planRaw);
			if (planRaw.size() == 1){
				plan = planRaw;
				cout << "[RRTPlanner]: TIMEOUT! Start position might not be feasible!!" << endl;
				return;
			}
			else{
				cout << "[RRTPlanner]: TIMEOUT!"<< "(>" << this->timeout_ << "s)" << ", Return closest path. Distance: " << nearestDistance << " m." << endl;
			}
		}
		this->shortcutWaypointPaths(planRaw, plan);

		// visualization
		if (this->visPath_){
			this->updatePathVisVec(plan);
			this->pathVisMsg_.markers = this->pathVisVec_;
		}
	}

	template <std::size_t N>
	void rrtStarOctomap<N>::makePlan(nav_msgs::Path& plan){
		std::vector<KDTree::Point<N>> planTemp;
		this->makePlan(planTemp);
		this->pathMsgConverter(planTemp, plan);
	}

	template <std::size_t N>
	double rrtStarOctomap<N>::getNeighborHoodRadius(){
		return this->rNeighborhood_;
	}


	// overload
	template <std::size_t N>
	std::ostream &operator<<(std::ostream &os, rrtStarOctomap<N> &rrtStarPlanner){
        os << "========================INFO========================\n";
        os << "[RRTPlanner]: RRT* planner with octomap\n";
        os << "[Connect Ratio]: " << rrtStarPlanner.getConnectGoalRatio() << "\n";
        // os << "[Start/Goal]:  " <<  rrtStarPlanner.getStart() << "=>" <<  rrtStarPlanner.getGoal() << "\n";
        std::vector<double> collisionBox = rrtStarPlanner.getCollisionBox();
        os << "[Collision Box]: " << collisionBox[0] << " " << collisionBox[1] << " " <<  collisionBox[2] << "\n";
        os << "[Map Res]: " << rrtStarPlanner.getMapRes() << "\n";
        os << "[Timeout]: " << rrtStarPlanner.getTimeout() << "\n";
        os << "[Neighborhood Radius]: " << rrtStarPlanner.getNeighborHoodRadius() << "\n";
        os << "====================================================";
        return os;
    }
}	


#endif