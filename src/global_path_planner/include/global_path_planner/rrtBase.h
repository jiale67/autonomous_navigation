/*
*	File: rrtBase.h
*	---------------
*   Base class for RRT planner.
*/
#ifndef RRTBASE_H
#define RRTBASE_H
#include <ros/ros.h>
#include <global_path_planner/KDTree.h>
#include <Eigen/Eigen>
#include <random>
#include <geometry_msgs/Pose.h>
#include <global_path_planner/utils.h>

using std::cout; using std::endl;

namespace globalPlanner{
	template <std::size_t N>
	class rrtBase{
	private:
		ros::NodeHandle nh_;

	protected:
		double delQ_; // incremental distance
		double dR_; // criteria for goal reaching
		KDTree::Point<N> start_;
		KDTree::Point<N> goal_;
		KDTree::Point<N> emptyToken_;
		KDTree::KDTree<N, int> ktree_; // KDTree
		std::unordered_map<KDTree::Point<N>, KDTree::Point<N>, KDTree::PointHasher> parent_; // for backtracking
		std::vector<double> collisionBox_; // (lx, ly, lz)
		std::vector<double> envBox_; // value of min max of x y and z
		double connectGoalRatio_;
		double timeout_;

	public:
		// Default constructor
		rrtBase(); 

		// Constructor
		rrtBase(KDTree::Point<N> start, KDTree::Point<N> goal, std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout);

		rrtBase(std::vector<double> start, std::vector<double> goal,  std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout);

		rrtBase(const ros::NodeHandle &nh, KDTree::Point<N> start, KDTree::Point<N> goal, std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout);

		rrtBase(const ros::NodeHandle &nh, std::vector<double> start, std::vector<double> goal,  std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout);

		rrtBase(std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout);


		virtual ~rrtBase();

		// load map based on different map representaiton
		virtual void updateMap() = 0;

		// collision checking function based on map and collision box:
		virtual bool checkCollision(const KDTree::Point<N>& q) = 0;

		// random sample in valid space (based on current map)
		virtual void randomConfig(KDTree::Point<N>& qRand) = 0;

		// Find the nearest vertex (node) in the tree
		void nearestVertex(const KDTree::Point<N>& qKey, KDTree::Point<N>& qNear);

		// Steer function: basd on delta
		void newConfig(const KDTree::Point<N>& qNear, const KDTree::Point<N>& qRand, KDTree::Point<N>& qNew);

		void backTrace(const KDTree::Point<N>& qGoal, std::vector<KDTree::Point<N>>& plan);

		bool isReach(const KDTree::Point<N>& q);

		// *** Core function: make plan based on all input ***
		virtual void makePlan(std::vector<KDTree::Point<N>>& plan) = 0;

		// add the new vertex to the RRT: add to KDTree
		void addVertex(const KDTree::Point<N>& qNew);

		// add the new edge to RRT: add to rrt 
		void addEdge(const KDTree::Point<N>& qNear, const KDTree::Point<N>& qNew);
		bool hasNoEdge(const KDTree::Point<N>& qNear, const KDTree::Point<N>& qNew);

		// update start position:
		void updateStart(const std::vector<double>& newStart);
		void updateStart(const Eigen::Vector3d& newStart);
		void updateStart(const KDTree::Point<N>& newStart);
		void updateStart(const geometry_msgs::Pose& newStart);

		// update goal position:
		void updateGoal(const std::vector<double>& newGoal);
		void updateGoal(const Eigen::Vector3d& newGoal);
		void updateGoal(const KDTree::Point<N>& newGoal);
		void updateGoal(const geometry_msgs::Pose& newGoal);

		void clearRRT();

		// return start point:
		KDTree::Point<N> getStart();

		// return goal point:
		KDTree::Point<N> getGoal();

		// return collision box:
		std::vector<double> getCollisionBox();

		// return env box:
		std::vector<double> getEnvBox();

		// return dR:
		double getReachRadius();

		// return parent dictionary (edge)
		std::unordered_map<KDTree::Point<N>, KDTree::Point<N>, KDTree::PointHasher> getParentDict(); 

		// return goal conenct ratio
		double getConnectGoalRatio();

		// return timeout
		double getTimeout();

	};

	// ===============Function Definition===============================
	template <std::size_t N>
	rrtBase<N>::rrtBase(){
		this->emptyToken_[0] = -11311; 
	};

	// Constructor:
	template <std::size_t N>
	rrtBase<N>::rrtBase(KDTree::Point<N> start, KDTree::Point<N> goal, std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout) 
	: collisionBox_(collisionBox), envBox_(envBox), delQ_(delQ), dR_(dR), connectGoalRatio_(connectGoalRatio), timeout_(timeout){
		// 确保起点和终点Z坐标为1（仅当N=3时）
		if constexpr (N == 3) {
			start[2] = 1.0;
			goal[2] = 1.0;
		}
		
		this->start_ = start;
		this->goal_ = goal;
		this->emptyToken_[0] = -11311; 
		this->parent_[start_] = this->emptyToken_; // set start parent to NULL
	}

	template <std::size_t N>
	rrtBase<N>::rrtBase(std::vector<double> start, std::vector<double> goal,  std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout)
	: collisionBox_(collisionBox), envBox_(envBox), delQ_(delQ), dR_(dR), connectGoalRatio_(connectGoalRatio), timeout_(timeout){
		// 确保起点和终点Z坐标为1（仅当N=3时）
		if constexpr (N == 3) {
			if (start.size() < 3) start.push_back(1.0);
			else start[2] = 1.0;
			
			if (goal.size() < 3) goal.push_back(1.0);
			else goal[2] = 1.0;
		}
		
		KDTree::Point<N> startp = KDTree::vec2Point<N>(start);
		KDTree::Point<N> goalp = KDTree::vec2Point<N>(goal);
		this->start_ = startp;
		this->goal_ = goalp;
		this->emptyToken_[0] = -11311; 
		this->parent_[start_] = this->emptyToken_; // set start parent to NULL
	}


	// Constructor:
	template <std::size_t N>
	rrtBase<N>::rrtBase(const ros::NodeHandle& nh, KDTree::Point<N> start, KDTree::Point<N> goal, std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout) 
	: nh_(nh), collisionBox_(collisionBox), envBox_(envBox), delQ_(delQ), dR_(dR), connectGoalRatio_(connectGoalRatio), timeout_(timeout){
		// 确保起点和终点Z坐标为1（仅当N=3时）
		if constexpr (N == 3) {
			start[2] = 1.0;
			goal[2] = 1.0;
		}
		
		this->start_ = start;
		this->goal_ = goal;
		this->emptyToken_[0] = -11311; 
		this->parent_[start_] = this->emptyToken_; // set start parent to NULL
	}

	template <std::size_t N>
	rrtBase<N>::rrtBase(const ros::NodeHandle& nh, std::vector<double> start, std::vector<double> goal,  std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout)
	: nh_(nh), collisionBox_(collisionBox), envBox_(envBox), delQ_(delQ), dR_(dR), connectGoalRatio_(connectGoalRatio), timeout_(timeout){
		// 确保起点和终点Z坐标为1（仅当N=3时）
		if constexpr (N == 3) {
			if (start.size() < 3) start.push_back(1.0);
			else start[2] = 1.0;
			
			if (goal.size() < 3) goal.push_back(1.0);
			else goal[2] = 1.0;
		}
		
		KDTree::Point<N> startp = KDTree::vec2Point<N>(start);
		KDTree::Point<N> goalp = KDTree::vec2Point<N>(goal);
		this->start_ = startp;
		this->goal_ = goalp;
		this->emptyToken_[0] = -11311; 
		this->parent_[start_] = this->emptyToken_; // set start parent to NULL
	}

	template <std::size_t N>
	rrtBase<N>::rrtBase(std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout)
	: collisionBox_(collisionBox), envBox_(envBox), delQ_(delQ), dR_(dR), connectGoalRatio_(connectGoalRatio), timeout_(timeout){
		cout << "[RRTPlanner]: Please update start and goal!" << endl;
		this->emptyToken_[0] = -11311; 
	}

	template <std::size_t N>
	void rrtBase<N>::nearestVertex(const KDTree::Point<N>& qKey, KDTree::Point<N>& qNear){
		this->ktree_.nearestNeighbor(qKey, qNear);
	}

	template <std::size_t N>
	rrtBase<N>::~rrtBase(){

	}

	template <std::size_t N>
	void rrtBase<N>::newConfig(const KDTree::Point<N>& qNear, const KDTree::Point<N>& qRand, KDTree::Point<N>& qNew){
		double distance = Distance(qNear, qRand);
		KDTree::Point<N> direction = qRand - qNear;
		qNew = qNear + (this->delQ_/distance) * direction;
	}

	template <std::size_t N>
	void rrtBase<N>::backTrace(const KDTree::Point<N>& qGoal, std::vector<KDTree::Point<N>>& plan){
		KDTree::Point<N> ptr = qGoal;
		while (ptr[0] != this->emptyToken_[0]){
			plan.push_back(ptr);
			ptr = this->parent_[ptr];
		}
		std::reverse(plan.begin(), plan.end());
	}

	template <std::size_t N>
	bool rrtBase<N>::isReach(const KDTree::Point<N>& q){
		return KDTree::Distance(q, this->goal_) <= this->dR_;
	}

	template <std::size_t N>
	void rrtBase<N>::addVertex(const KDTree::Point<N>& qNew){
		this->ktree_.insert(qNew);
	}

	template <std::size_t N>
	void rrtBase<N>::addEdge(const KDTree::Point<N>& qNear, const KDTree::Point<N>& qNew){
		this->parent_[qNew] = qNear;
	}

	template <std::size_t N>
	bool rrtBase<N>::hasNoEdge(const KDTree::Point<N>& qNear, const KDTree::Point<N>& qNew){
		if (qNear == qNew){
			return false;
		}
		return this->parent_[qNear] != qNew;
	}

	template <std::size_t N>
	void rrtBase<N>::updateStart(const std::vector<double>& newStart){
		KDTree::Point<N> newStartp = KDTree::vec2Point<N>(newStart);
		this->updateStart(newStartp);
	}

	template <std::size_t N>
	void rrtBase<N>::updateStart(const Eigen::Vector3d& newStart){
		KDTree::Point<N> newStartp = KDTree::eig2Point<N>(newStart);
		this->updateStart(newStartp);
	}
	
	template <std::size_t N>
	void rrtBase<N>::updateStart(const KDTree::Point<N>& newStart){
		// 确保起点Z坐标为1（仅当N=3时）
		KDTree::Point<N> adjustedStart = newStart;
		if constexpr (N == 3) {
			adjustedStart[2] = 1.0;
		}
		
		this->start_ = adjustedStart;
		this->parent_.clear();
		this->parent_[this->start_] = this->emptyToken_;
		if (not this->ktree_.empty()){
			this->clearRRT();
		}
	}

	template <std::size_t N>
	void rrtBase<N>::updateStart(const geometry_msgs::Pose& newStart){
		KDTree::Point<N> newStartp;
		newStartp[0] = newStart.position.x; 
		newStartp[1] = newStart.position.y; 
		// 确保起点Z坐标为1（仅当N=3时）
		if constexpr (N == 3) {
			newStartp[2] = 1.0;
		}
		else {
			newStartp[2] = newStart.position.z;
		}
		this->updateStart(newStartp);
	}

	template <std::size_t N>
	void rrtBase<N>::updateGoal(const std::vector<double>& newGoal){
		KDTree::Point<N> newGoalp = KDTree::vec2Point<N>(newGoal);
		this->updateGoal(newGoalp);
	}

	template <std::size_t N>
	void rrtBase<N>::updateGoal(const Eigen::Vector3d& newGoal){
		KDTree::Point<N> newGoalp = KDTree::eig2Point<N>(newGoal);
		this->updateGoal(newGoalp);
	}
	
	template <std::size_t N>
	void rrtBase<N>::updateGoal(const KDTree::Point<N>& newGoal){
		// 确保终点Z坐标为1（仅当N=3时）
		KDTree::Point<N> adjustedGoal = newGoal;
		if constexpr (N == 3) {
			adjustedGoal[2] = 1.0;
		}
		
		this->goal_ = adjustedGoal;
		this->parent_.clear();
		this->parent_[this->start_] = this->emptyToken_;
		if (not this->ktree_.empty()){
			this->clearRRT();
		}
	}

	template <std::size_t N>
	void rrtBase<N>::updateGoal(const geometry_msgs::Pose& newGoal){
		KDTree::Point<N> newGoalp;
		newGoalp[0] = newGoal.position.x; 
		newGoalp[1] = newGoal.position.y; 
		// 确保终点Z坐标为1（仅当N=3时）
		if constexpr (N == 3) {
			newGoalp[2] = 1.0;
		}
		else {
			newGoalp[2] = newGoal.position.z;
		}
		this->updateGoal(newGoalp);
	}

	template <std::size_t N>
	void rrtBase<N>::clearRRT(){
		this->ktree_.~KDTree();
		new (&this->ktree_) KDTree::KDTree<N, int> ();
	}

	template <std::size_t N>
	KDTree::Point<N> rrtBase<N>::getStart(){
		return this->start_;
	}

	template <std::size_t N>
	KDTree::Point<N> rrtBase<N>::getGoal(){
		return this->goal_;
	}

	template <std::size_t N>
	std::vector<double> rrtBase<N>::getCollisionBox(){
		return this->collisionBox_;
	}

	template <std::size_t N>
	std::vector<double> rrtBase<N>::getEnvBox(){
		return this->envBox_;
	}

	template <std::size_t N>
	double rrtBase<N>::getReachRadius(){
		return this->dR_;
	}

	template <std::size_t N>
	std::unordered_map<KDTree::Point<N>, KDTree::Point<N>, KDTree::PointHasher> rrtBase<N>::getParentDict(){
		return this->parent_;
	}

	template <std::size_t N>
	double rrtBase<N>::getConnectGoalRatio(){
		return this->connectGoalRatio_;
	}

	template <std::size_t N>
	double rrtBase<N>::getTimeout(){
		return this->timeout_;
	}
}

#endif