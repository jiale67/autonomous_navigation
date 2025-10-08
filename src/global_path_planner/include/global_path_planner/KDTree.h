/**
 * File: KDTree.h
 * ------------------------
 * An interface representing a kd-tree in some number of dimensions. The tree
 * can be constructed from a set of data and then queried for membership and
 * k nearest neighbors.
 */

#ifndef KDTREE_INCLUDED
#define KDTREE_INCLUDED


#include "Point.h"
#include "BoundedPQueue.h"
#include <stdexcept>
#include <cmath>
#include <vector>
#include <unordered_map>
#include <utility>
#include <algorithm>
#include <memory>


namespace KDTree{
    template <std::size_t N, typename ElemType>
    class KDTree {
    public:
        // Constructs an empty KDTree.
        KDTree();

        // Efficiently build a balanced KD-tree from a large set of points
        KDTree(std::vector<std::pair<Point<N>, ElemType>>& points);

        // Frees up all the dynamically allocated resources
        ~KDTree();

        // Deep-copies the contents of another KDTree into this one.
        KDTree(const KDTree& rhs);
        KDTree& operator=(const KDTree& rhs);

        // Returns the dimension of the points stored in this KDTree.
        std::size_t dimension() const;

        // Returns the number of elements in the kd-tree and whether the tree is empty
        std::size_t size() const;
        bool empty() const;

        // Returns whether the specified point is contained in the KDTree.
        bool contains(const Point<N>& pt) const;

        /*
         * Inserts the point pt into the KDTree, associating it with the specified value.
         * If the element already existed in the tree, the new value will overwrite the existing one.
         */
        void insert(const Point<N>& pt, const ElemType& value=ElemType());

        /*
         * Returns a reference to the value associated with point pt in the KDTree.
         * If the point does not exist, then it is added to the KDTree using the
         * default value of ElemType as its key.
         */
        ElemType& operator[](const Point<N>& pt);

        /*
         * Returns a reference to the key associated with the point pt. If the point
         * is not in the tree, this function throws an out_of_range exception.
         */
        ElemType& at(const Point<N>& pt);
        const ElemType& at(const Point<N>& pt) const;

        /*
         * Given a point v and an integer k, finds the k points in the KDTree
         * nearest to v and returns the most common value associated with those
         * points. In the event of a tie, one of the most frequent value will be chosen.
         */
        ElemType kNNValue(const Point<N>& key, std::size_t k) const;
        
        // Zhefan: This is to implement KNN and return k nearest neighbor coordinate
        void knn(const Point<N>& key, std::size_t k, std::vector<Point<N>>& knnVec) const;

        // Zhefan: This is to implement KNN and return nearest neighbor
        void nearestNeighbor(const Point<N>& key, Point<N>& nn) const;

        // Zhefan: This is to implement range search with predefined maximum neighbor: 1. first find neighbors 2. pich based on distance 
        void boundedRangeSearch(const Point<N>& key, double radius, int max_neighbor, std::vector<Point<N>>& neighborhood) const;


    private:
        struct Node {
            Point<N> point;
            Node *left;
            Node *right;
            int level;  // level of the node in the tree, starts at 0 for the root
            ElemType value;
            Node(const Point<N>& _pt, int _level, const ElemType& _value=ElemType()):
                point(_pt), left(NULL), right(NULL), level(_level), value(_value) {}
        };  

        // Root node of the KD-Tree
        Node* root_;

        // Number of points in the KD-Tree
        std::size_t size_;

        /*
         * Recursively build a subtree that satisfies the KD-Tree invariant using points in [start, end)
         * At each level, we split points into two halves using the median of the points as pivot
         * The root of the subtree is at level 'currLevel'
         * O(n) time partitioning algorithm is used to locate the median element
         */
        Node* buildTree(typename std::vector<std::pair<Point<N>, ElemType>>::iterator start,
                        typename std::vector<std::pair<Point<N>, ElemType>>::iterator end, int currLevel);

        /*
         * Returns the Node that contains Point pt if it is present in subtree 'currNode'
         * Returns the Node below which pt should be inserted if pt is not in the subtree
         */
        Node* findNode(Node* currNode, const Point<N>& pt) const;

        // Recursive helper method for kNNValue(pt, k)
        void nearestNeighborRecurse(const Node* currNode, const Point<N>& key, BoundedPQueue<ElemType>& pQueue) const;

        // Zhefan (ok): Recursive helper method for kNN(pt, k)
        void nearestNeighborRecurse(const Node* currNode, const Point<N>& key, BoundedPQueue<Point<N>>& pQueue) const;

        /*
         * Recursive helper method for copy constructor and assignment operator
         * Deep copies tree 'root' and returns the root of the copied tree
         */
        Node* deepcopyTree(Node* root);

        // Recursively free up all resources of subtree rooted at 'currNode'
        void freeResource(Node* currNode);
    };

    // ======================================================================================
    template <std::size_t N, typename ElemType>
    KDTree<N, ElemType>::KDTree() :
        root_(NULL), size_(0) { }

    template <std::size_t N, typename ElemType>
    typename KDTree<N, ElemType>::Node* KDTree<N, ElemType>::deepcopyTree(typename KDTree<N, ElemType>::Node* root) {
        if (root == NULL) return NULL;
        Node* newRoot = new Node(*root);
        newRoot->left = deepcopyTree(root->left);
        newRoot->right = deepcopyTree(root->right);
        return newRoot;
    }

    template <std::size_t N, typename ElemType>
    typename KDTree<N, ElemType>::Node* KDTree<N, ElemType>::buildTree(typename std::vector<std::pair<Point<N>, ElemType>>::iterator start,
                                                                       typename std::vector<std::pair<Point<N>, ElemType>>::iterator end, int currLevel) {
        if (start >= end) return NULL; // empty tree

        int axis = currLevel % N; // the axis to split on
        auto cmp = [axis](const std::pair<Point<N>, ElemType>& p1, const std::pair<Point<N>, ElemType>& p2) {
            return p1.first[axis] < p2.first[axis];
        };
        std::size_t len = end - start;
        auto mid = start + len / 2;
        std::nth_element(start, mid, end, cmp); // linear time partition

        // move left (if needed) so that all the equal points are to the right
        // The tree will still be balanced as long as there aren't many points that are equal along each axis
        while (mid > start && (mid - 1)->first[axis] == mid->first[axis]) {
            --mid;
        }

        Node* newNode = new Node(mid->first, currLevel, mid->second);
        newNode->left = buildTree(start, mid, currLevel + 1);
        newNode->right = buildTree(mid + 1, end, currLevel + 1);
        return newNode;
    }

    template <std::size_t N, typename ElemType>
    KDTree<N, ElemType>::KDTree(std::vector<std::pair<Point<N>, ElemType>>& points) {
        root_ = buildTree(points.begin(), points.end(), 0);
        size_ = points.size();
    }

    template <std::size_t N, typename ElemType>
    KDTree<N, ElemType>::KDTree(const KDTree& rhs) {
        root_ = deepcopyTree(rhs.root_);
        size_ = rhs.size_;
    }

    template <std::size_t N, typename ElemType>
    KDTree<N, ElemType>& KDTree<N, ElemType>::operator=(const KDTree& rhs) {
        if (this != &rhs) { // make sure we don't self-assign
            freeResource(root_);
            root_ = deepcopyTree(rhs.root_);
            size_ = rhs.size_;
        }
        return *this;
    }

    template <std::size_t N, typename ElemType>
    void KDTree<N, ElemType>::freeResource(typename KDTree<N, ElemType>::Node* currNode) {
        if (currNode == NULL) return;
        freeResource(currNode->left);
        freeResource(currNode->right);
        delete currNode;
        this->size_ = 0;
    }

    template <std::size_t N, typename ElemType>
    KDTree<N, ElemType>::~KDTree() {
        freeResource(root_);
    }

    template <std::size_t N, typename ElemType>
    std::size_t KDTree<N, ElemType>::dimension() const {
        return N;
    }

    template <std::size_t N, typename ElemType>
    std::size_t KDTree<N, ElemType>::size() const {
        return size_;
    }

    template <std::size_t N, typename ElemType>
    bool KDTree<N, ElemType>::empty() const {
        return size_ == 0;
    }

    template <std::size_t N, typename ElemType>
    typename KDTree<N, ElemType>::Node* KDTree<N, ElemType>::findNode(typename KDTree<N, ElemType>::Node* currNode, const Point<N>& pt) const {
        if (currNode == NULL || currNode->point == pt) return currNode;

        const Point<N>& currPoint = currNode->point;
        int currLevel = currNode->level;
        if (pt[currLevel%N] < currPoint[currLevel%N]) { // recurse to the left side
            return currNode->left == NULL ? currNode : findNode(currNode->left, pt);
        } else { // recurse to the right side
            return currNode->right == NULL ? currNode : findNode(currNode->right, pt);
        }
    }

    template <std::size_t N, typename ElemType>
    bool KDTree<N, ElemType>::contains(const Point<N>& pt) const {
        auto node = findNode(root_, pt);
        return node != NULL && node->point == pt;
    }

    template <std::size_t N, typename ElemType>
    void KDTree<N, ElemType>::insert(const Point<N>& pt, const ElemType& value) {
        auto targetNode = findNode(root_, pt);
        if (targetNode == NULL) { // this means the tree is empty
            root_ = new Node(pt, 0, value);
            size_ = 1;
        } else {
            if (targetNode->point == pt) { // pt is already in the tree, simply update its value
                targetNode->value = value;
            } else { // construct a new node and insert it to the right place (child of targetNode)
                int currLevel = targetNode->level;
                Node* newNode = new Node(pt, currLevel + 1, value);
                if (pt[currLevel%N] < targetNode->point[currLevel%N]) {
                    targetNode->left = newNode;
                } else {
                    targetNode->right = newNode;
                }
                ++size_;
            }
        }
    }

    template <std::size_t N, typename ElemType>
    const ElemType& KDTree<N, ElemType>::at(const Point<N>& pt) const {
        auto node = findNode(root_, pt);
        if (node == NULL || node->point != pt) {
            throw std::out_of_range("Point not found in the KD-Tree");
        } else {
            return node->value;
        }
    }

    template <std::size_t N, typename ElemType>
    ElemType& KDTree<N, ElemType>::at(const Point<N>& pt) {
        const KDTree<N, ElemType>& constThis = *this;
        return const_cast<ElemType&>(constThis.at(pt));
    }

    template <std::size_t N, typename ElemType>
    ElemType& KDTree<N, ElemType>::operator[](const Point<N>& pt) {
        auto node = findNode(root_, pt);
        if (node != NULL && node->point == pt) { // pt is already in the tree
            return node->value;
        } else { // insert pt with default ElemType value, and return reference to the new ElemType
            insert(pt);
            if (node == NULL) return root_->value; // the new node is the root
            else return (node->left != NULL && node->left->point == pt) ? node->left->value: node->right->value;
        }
    }

    template <std::size_t N, typename ElemType>
    void KDTree<N, ElemType>::nearestNeighborRecurse(const typename KDTree<N, ElemType>::Node* currNode, const Point<N>& key, BoundedPQueue<ElemType>& pQueue) const {
        if (currNode == NULL) return;
        const Point<N>& currPoint = currNode->point;

        // Add the current point to the BPQ if it is closer to 'key' that some point in the BPQ
        pQueue.enqueue(currNode->value, Distance(currPoint, key));

        // Recursively search the half of the tree that contains Point 'key'
        int currLevel = currNode->level;
        bool isLeftTree;
        if (key[currLevel%N] < currPoint[currLevel%N]) {
            nearestNeighborRecurse(currNode->left, key, pQueue);
            isLeftTree = true;
        } else {
            nearestNeighborRecurse(currNode->right, key, pQueue);
            isLeftTree = false;
        }

        if (pQueue.size() < pQueue.maxSize() || fabs(key[currLevel%N] - currPoint[currLevel%N]) < pQueue.worst()) {
            // Recursively search the other half of the tree if necessary
            if (isLeftTree) nearestNeighborRecurse(currNode->right, key, pQueue);
            else nearestNeighborRecurse(currNode->left, key, pQueue);
        }
    }

    // Zhefan: Helper function for knn
    template <std::size_t N, typename ElemType>
    void KDTree<N, ElemType>::nearestNeighborRecurse(const Node* currNode, const Point<N>& key, BoundedPQueue<Point<N>>& pQueue) const{
        if (currNode == NULL) return;
        const Point<N>& currPoint = currNode->point;

        // Zhefan: Add the current point to the BPQ if it is closer to 'key' that some point in the BPQ 
        pQueue.enqueue(currPoint, Distance(currPoint, key));


        // Recursively search the half of the tree that contains Point 'key'
        int currLevel = currNode->level;
        bool isLeftTree;
        if (key[currLevel%N] < currPoint[currLevel%N]) {
            nearestNeighborRecurse(currNode->left, key, pQueue);
            isLeftTree = true;
        } else {
            nearestNeighborRecurse(currNode->right, key, pQueue);
            isLeftTree = false;
        }

        if (pQueue.size() < pQueue.maxSize() || fabs(key[currLevel%N] - currPoint[currLevel%N]) < pQueue.worst()) {
            // Recursively search the other half of the tree if necessary
            if (isLeftTree) nearestNeighborRecurse(currNode->right, key, pQueue);
            else nearestNeighborRecurse(currNode->left, key, pQueue);
        }
    }

    template <std::size_t N, typename ElemType>
    ElemType KDTree<N, ElemType>::kNNValue(const Point<N>& key, std::size_t k) const {
        BoundedPQueue<ElemType> pQueue(k); // BPQ with maximum size k
        if (empty()) return ElemType(); // default return value if KD-tree is empty

        // Recursively search the KD-tree with pruning
        nearestNeighborRecurse(root_, key, pQueue);

        // Count occurrences of all ElemType in the kNN set
        std::unordered_map<ElemType, int> counter;
        while (!pQueue.empty()) {
            ++counter[pQueue.dequeueMin()];
        }

        // Return the most frequent element in the kNN set
        ElemType result;
        int cnt = -1;
        for (const auto &p : counter) {
            if (p.second > cnt) {
                result = p.first;
                cnt = p.second;
            }
        }
        return result;
    }


    // Zhefan: This is to implement KNN and return k nearest neighbor coordinate
    template <std::size_t N, typename ElemType>
    void KDTree<N, ElemType>::knn(const Point<N>& key, std::size_t k, std::vector<Point<N>>& knnVec) const {
        BoundedPQueue<Point<N>> pQueue(k); // BPQ with maximum size k
        if (empty()){std::cout << "Tree is empty" << std::endl;}; // default return value if KD-tree is empty

        // Recursively search the KD-tree with pruning
        nearestNeighborRecurse(root_, key, pQueue);

        // Use std::vector to store k nearest points from nearest to furthest:
        while (!pQueue.empty()){
            knnVec.push_back(pQueue.dequeueMin());
        }
    }

    // Zhefan: This is to implement single nearest neigbor
    template <std::size_t N, typename ElemType>
    void KDTree<N, ElemType>::nearestNeighbor(const Point<N>& key, Point<N>& nn) const{
        std::vector<Point<N>> knnVec;
        knn(key, 1, knnVec);
        nn = knnVec[0];
    }

    // Zhefan: This is to implement range search with predefined maximum neighbor: 1. first find neighbors 2. pich based on distance:
    template <std::size_t N, typename ElemType>
    void KDTree<N, ElemType>::boundedRangeSearch(const Point<N>& key, double radius, int max_neighbor, std::vector<Point<N>>& neighborhood) const{
        std::vector<Point<N>> knnVec;
        knn(key, max_neighbor, knnVec);
        for (Point<N> p: knnVec){
           double distance = Distance(p, key);
           if (distance <= radius){
                neighborhood.push_back(p);
           } 
           else{
                break;
           }
        }
    }


//--------------END------------------
}
#endif // KDTREE_INCLUDED
