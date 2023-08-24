
#ifndef ERL_TYPES_H
#define ERL_TYPES_H

#include <Eigen/Core>
#include <vector>

///Set red font in printf funtion
#define ANSI_COLOR_RED "\x1b[1;31m"
///Set green font in printf funtion 
#define ANSI_COLOR_GREEN "\x1b[1;32m"
///Set yellow font in printf funtion 
#define ANSI_COLOR_YELLOW "\x1b[1;33m"
///Set blue font in printf funtion 
#define ANSI_COLOR_BLUE "\x1b[1;34m"
///Set magenta font in printf funtion 
#define ANSI_COLOR_MAGENTA "\x1b[1;35m"
///Set cyan font in printf funtion 
#define ANSI_COLOR_CYAN "\x1b[1;36m"
///Reset font color in printf funtion 
#define ANSI_COLOR_RESET "\x1b[0m"


///Pre-allocated std::vector for Eigen.
template <typename T> 
using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;
/*! \brief Rename the float type used in lib 

    Default is set to be double, but user can change it to float.
*/

typedef double decimal_t;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;


///Vector of type Vec2f.
typedef vec_E<Eigen::Vector2d> vec_Vec2f;
///Vector of type Vec2i.
typedef vec_E<Eigen::Vector2i> vec_Vec2i;
///Vector of type Vec3f.
typedef vec_E<Eigen::Vector3d> vec_Vec3f;
///Vector of type Vec3i.
typedef vec_E<Eigen::Vector3i> vec_Vec3i;
//


namespace erl
{
  template <typename T> 
  using aligned_vector = std::vector<T, Eigen::aligned_allocator<T>>;  
}

#endif
