#ifndef __ERL_UTILS_H_
#define __ERL_UTILS_H_

#include <chrono>
#include <random>
#include <vector>
#include <array>
#include <numeric>      // std::iota
#include <algorithm>    // std::sort
#include <iostream>
#include <Eigen/Dense>

namespace erl {
/**
 * @file erl_utils.h
 * @brief General utility functions.
 *
 * Includes probability and timing utility functions.
 */

/* Timing */
/**
 * Returns a high resolution clock at the current time.
 * @return A clock at the current time.
 */
inline std::chrono::high_resolution_clock::time_point tic() { return std::chrono::high_resolution_clock::now(); }

/**
 * Returns the wall time elapsed between the current time and the input clock.
 * @param t2 The input clock.
 * @return The wall time elapsed.
 */
inline double toc(const std::chrono::high_resolution_clock::time_point &t2) {
  return std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t2).count();
}

/**
 * Computes the sign of the input.
 * @tparam T The data type of the input.
 * @param val The input value.
 * @return The integer sign of the input.
 */
template<typename T>
inline int sgn(T val) { return (T(0) < val) - (val < T(0)); }


/* Probability */
/**
 * Random number generation class can generate psuedo-random numbers using standard libraries.
 */
class RndNumberGen {
 public:
  std::random_device rd_;
  std::default_random_engine gen_;
  RndNumberGen()
      : gen_(rd_()) {}

  /**
   * Generates psuedo-random integers on the interval [min_, max_].
   * @param min_ The minimum number.
   * @param max_ The maxmimum number.
   * @return A pseudo-random number between min_ and max_.
   */
  int uniform_int(int min_, int max_) {
    std::uniform_int_distribution<int> uid(min_, max_);
    return uid(gen_);
  }

  /**
   * Generates pseudo-random doubles on the interval [min_, max_].
   * @param min_ The minimum value.
   * @param max_ The maximum value.
   * @return A pseudo-random double between min_ and max_.
   */
  double uniform_double(double min_, double max_) {
    std::uniform_real_distribution<double> urd(min_, max_);
    return urd(gen_);
  }

  double gaussian(double mean, double stdev) {
    std::normal_distribution<double> grv(mean,stdev);
    return grv(gen_);
  }
};

/**
 * Compute the un-normalized pdf of a sample of a Gaussian variable, given a mean and covariance.
 * @param x The sample of a Gaussian distribution
 * @param mu The mean of the distribution.
 * @param cov The covariance of the distribution.
 * @return The resulting un-weighted pdf.
 */
inline double gaussian_pdf(const Eigen::VectorXd &x, const Eigen::VectorXd &mu, const Eigen::MatrixXd &cov) {
  Eigen::VectorXd exponential =  - 0.5 * (x - mu).transpose() * cov.inverse() * (x - mu);
  double pdf = std::exp(exponential[0]);
  //  std::cout <<"Computing Gaussian PDF for: " << x <<", " << mu <<" cov: " << cov;
  //  std::cout << "Exponential: " << exponential;
  //  std::cout <<"PDF: " << pdf;
  return pdf;
}

// TODO Fix the design of this..
class Probability
{

 public:
  std::random_device rd;
  std::mt19937 gen;

  Probability() {
    gen = std::mt19937(rd());
  }
};

struct normal_random_variable {
  normal_random_variable(Eigen::MatrixXd const &covar)
      : normal_random_variable(Eigen::VectorXd::Zero(covar.rows()), covar) {}

  normal_random_variable(Eigen::VectorXd const &mean, Eigen::MatrixXd const &covar)
      : mean(mean) {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
    transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
  }

  Eigen::VectorXd mean;
  Eigen::MatrixXd transform;

  Eigen::VectorXd operator()() const {
    static std::mt19937 gen{std::random_device{}()};
    static std::normal_distribution<> dist;

    return mean + transform * Eigen::VectorXd{mean.size()}.unaryExpr([&](double x) { return dist(gen); });
  }
};

static RndNumberGen gen_;

inline Eigen::VectorXd normal_dist(Eigen::VectorXd mean, Eigen::MatrixXd cov) {
  normal_random_variable sample{mean, cov};
  return sample().cast<double>();
}

inline Eigen::VectorXd normal_dist(Eigen::MatrixXd cov) {
  normal_random_variable sample{cov};
  return sample().cast<double>();
}

// Scalar normal dist
inline double normal_dist(double mean, double cov) {
  std::normal_distribution<double> dist = std::normal_distribution<double>(mean, cov);
  return dist(gen_.gen_);
}

/* STL Extensions */
/**
 * Prints a vector of data type T.
 * @tparam T The data type.
 * @param os The output stream.
 * @param v The vector of data.
 * @return The appended output strema.
 */
template<class T>
std::ostream &operator<<(std::ostream &os, const std::vector<T> &v) {
  os << "[";
  for (typename std::vector<T>::const_iterator ii = v.begin(); ii != v.end(); ++ii) {
    os << " " << *ii;
  }
  os << " ]";
  return os;
}
template<class T, size_t N>
std::ostream &operator<<(std::ostream &os, const std::array<T, N> &a) {
  os << "[";
  for (typename std::array<T, N>::const_iterator ii = a.begin(); ii != a.end(); ++ii) {
    os << " " << *ii;
  }
  os << " ]";
  return os;
}

template <typename T>
std::vector<size_t> sortAscendingIdx(const std::vector<T> &v)
{
  // initialize original index locations
  std::vector<size_t> idx(v.size());
  std::iota(idx.begin(), idx.end(), 0);
  // sort indices based on comparing values in v
  std::sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});
  return idx;
}

template <typename T>
std::vector<size_t> sortDescendingIdx(const std::vector<T> &v)
{
  // initialize original index locations
  std::vector<size_t> idx(v.size());
  std::iota(idx.begin(), idx.end(), 0);
  // sort indices based on comparing values in v
  std::sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) {return v[i1] > v[i2];});
  return idx;
}


template <typename T>
std::vector<T> operator+(const std::vector<T> &a, const std::vector<T> &b)
{
  std::vector<T> apb(a.size());
  std::transform (a.begin(), a.end(), b.begin(), apb.begin(), std::plus<T>());
  return apb;
}

template <typename T>
std::vector<T> operator-(const std::vector<T> &a, const std::vector<T> &b)
{
  std::vector<T> amb(a.size());
  std::transform (a.begin(), a.end(), b.begin(), amb.begin(), std::minus<T>());
  return amb;
}

template <typename T>
std::vector<T> operator*(const std::vector<T> &a, const std::vector<T> &b)
{
  std::vector<T> atb(a.size());
  std::transform (a.begin(), a.end(), b.begin(), atb.begin(), std::multiplies<T>());
  return atb;
}

template <typename T>
std::vector<T> operator/(const std::vector<T> &a, const std::vector<T> &b)
{
  std::vector<T> adb(a.size());
  std::transform (a.begin(), a.end(), b.begin(), adb.begin(), std::divides<T>());
  return adb;
}


template <typename T>
std::vector<T> operator+(const std::vector<T> &a, const T &b)
{
  std::vector<T> apb(a.size());
  std::transform(a.begin(), a.end(), apb.begin(), [&b](auto& ai) { return ai + b; });
  return apb;
}

template <typename T>
std::vector<T> operator+(const T &a, const std::vector<T> &b)
{
  std::vector<T> apb(b.size());
  std::transform(b.begin(), b.end(), apb.begin(), [&a](auto& bi) { return a + bi; });
  return apb;
}

template <typename T>
std::vector<T> operator-(const std::vector<T> &a, const T &b)
{
  std::vector<T> amb(a.size());
  std::transform(a.begin(), a.end(), amb.begin(), [&b](auto& ai) { return ai - b; });
  return amb;
}

template <typename T>
std::vector<T> operator-(const T &a, const std::vector<T> &b)
{
  std::vector<T> amb(b.size());
  std::transform(b.begin(), b.end(), amb.begin(), [&a](auto& bi) { return a - bi; });
  return amb;
}

template <typename T>
std::vector<T> operator*(const std::vector<T> &a, const T &b)
{
  std::vector<T> atb(a.size());
  std::transform(a.begin(), a.end(), atb.begin(), [&b](auto& ai) { return ai*b; });
  return atb;
}

template <typename T>
std::vector<T> operator*(const T &a, const std::vector<T> &b)
{
  std::vector<T> atb(b.size());
  std::transform(b.begin(), b.end(), atb.begin(), [&a](auto& bi) { return a*bi; });
  return atb;
}

template <typename T>
std::vector<T> operator/(const std::vector<T> &a, const T &b)
{
  std::vector<T> adb(a.size());
  std::transform(a.begin(), a.end(), adb.begin(), [&b](auto& ai) { return ai/b; });
  return adb;
}

template <typename T>
std::vector<T> operator/(const T &a, const std::vector<T> &b)
{
  std::vector<T> adb(b.size());
  std::transform(b.begin(), b.end(), adb.begin(), [&a](auto& bi) { return a/bi; });
  return adb;
}

template <typename T>
std::vector<T> nan_to_num( std::vector<T>&& a )
{
  std::vector<T> b(a.size());
  std::transform(a.begin(), a.end(), b.begin(), [](auto& ai){ return std::isnan(ai) ? T(0) : ai; });
  return b;
}



}
#endif


/*
//see https://en.cppreference.com/w/cpp/numeric/random/uniform_real_distribution
std::random_device rd;
std::mt19937 gen(rd());  //here you could set the seed, but std::random_device already does that
std::uniform_real_distribution<float> dis(-1.0, 1.0);
Eigen::MatrixXf A = Eigen::MatrixXf::NullaryExpr(3,3,[&](){return dis(gen);});
*/
  
