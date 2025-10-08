/**
 * File: Point.h
 * -------------
 * A class representing a point in N-dimensional space.
 * Point is parameterized over an integer N.
 * This allows the compiler to verify the type being used correctly.
 */
#ifndef POINT_INCLUDED
#define POINT_INCLUDED

#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <eigen3/Eigen/Core>

namespace KDTree{
    template <std::size_t N>
    class Point {
    public:

        // Types representing iterators that can traverse and optionally modify the elements of the Point.
        typedef double* iterator;
        typedef const double* const_iterator;

        // Returns N, the dimension of the point.
        std::size_t size() const;

        // Queries or retrieves the value of the point at a particular point. The index is assumed to be in-range.
        double& operator[](std::size_t index);
        double operator[](std::size_t index) const;

        // Returns iterators delineating the full range of elements in the Point.
        iterator begin();
        iterator end();
        const_iterator begin() const;
        const_iterator end() const;


    private:
        double coords[N];
    };

    // Returns the Squared Euclidean distance between two points.
    template <std::size_t N>
    double Distance(const Point<N>& one, const Point<N>& two);

    // Returns whether two points are equal / not equal
    template <std::size_t N>
    bool operator==(const Point<N>& one, const Point<N>& two);

    template <std::size_t N>
    bool operator!=(const Point<N>& one, const Point<N>& two);
    

    template <std::size_t N>
    std::size_t Point<N>::size() const {
        return N;
    }

    template <std::size_t N>
    double& Point<N>::operator[] (std::size_t index) {
        return coords[index];
    }

    template <std::size_t N>
    double Point<N>::operator[] (std::size_t index) const {
        return coords[index];
    }

    template <std::size_t N>
    typename Point<N>::iterator Point<N>::begin() {
        return coords;
    }

    template <std::size_t N>
    typename Point<N>::const_iterator Point<N>::begin() const {
        return coords;
    }

    template <std::size_t N>
    typename Point<N>::iterator Point<N>::end() {
        return begin() + size();
    }

    template <std::size_t N>
    typename Point<N>::const_iterator Point<N>::end() const {
        return begin() + size();
    }




    //=========================================================

    template <std::size_t N>
    double Distance(const Point<N>& one, const Point<N>& two) {
        double result = 0.0;
        for (std::size_t i = 0; i < N; ++i)
            result += (one[i] - two[i]) * (one[i] - two[i]);
        result = sqrt(result);
        return result;
    }

    template <std::size_t N>
    bool operator==(const Point<N>& one, const Point<N>& two) {
        return std::equal(one.begin(), one.end(), two.begin());
    }

    template <std::size_t N>
    bool operator!=(const Point<N>& one, const Point<N>& two) {
        return !(one == two);
    }



    // Zhefan: cout
    template <std::size_t N>
    Point<N> operator+(const Point<N>& p1, const Point<N>& p2){
        Point<N> p_sum;
        for (size_t i=0; i<N; ++i){
            p_sum[i] = p1[i] + p2[i];
        }
        return p_sum;
    }

    template <std::size_t N>
    Point<N> operator-(const Point<N>& p1, const Point<N>& p2){
        Point<N> p_sum;
        for (size_t i=0; i<N; ++i){
            p_sum[i] = p1[i] - p2[i];
        }
        return p_sum;
    }

    template <std::size_t N>
    Point<N> operator*(const Point<N>& p1, double r){
        Point<N> p_result;
        for (size_t i=0; i<N; ++i){
            p_result[i] = r * p1[i];
        }
        return p_result;
    }

    template <std::size_t N>
    Point<N> operator*(double r, const Point<N>& p1){
        Point<N> p_result;
        for (size_t i=0; i<N; ++i){
            p_result[i] = r * p1[i];
        }
        return p_result;
    }


    // Zhefan: cout
    template <std::size_t N>
    std::ostream &operator<<(std::ostream &os, Point<N> const &p){
        os << "(";
        for (size_t i=0; i<N; ++i){
            os << p[i];
            if (i != N-1){
                os << " ";
            }
        }
        os << ")";
        return os;
    }

    struct PointHasher{
        template <std::size_t N>
        std::size_t operator()(const Point<N>& p) const{
            double sum = 0.0;
            for (size_t i=0; i<N; ++i){
                sum += (p[i]/N) * (p[i]/N) * 1000000;
            }
            std::size_t hash = std::size_t(sum);
            // std::cout << hash << std::endl;
            return hash;
        }   
    };


    // =============UTILS for conversion================
    template <std::size_t N>
    std::vector<double> point2Vec(const Point<N>& p){
        std::vector<double> result;
        for (size_t i=1; i<N; ++i){
            result.push_back(p[i]);
        }
        return result;
    }

    template <std::size_t N>
    Point<N> vec2Point(const std::vector<double>& vec){
        Point<N> result;
        for (size_t i=0; i<N; ++i){
            result[i] = vec[i];
        }
        return result;
    }

    template <std::size_t N>
    Eigen::Vector3d point2Eig(const Point<N>& p){
        Eigen::Vector3d pE (p[0], p[1], p[2]);
        return pE;
    }

    template <std::size_t N>
    Point<N> eig2Point(const Eigen::Vector3d& pE){
        Point<N> result;
        result[0] = pE(0);
        result[1] = pE(1);
        result[2] = pE(2);
        return result;
    }

}
#endif // POINT_INCLUDED
