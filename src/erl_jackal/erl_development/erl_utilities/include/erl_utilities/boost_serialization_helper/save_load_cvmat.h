#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <boost/serialization/array.hpp>
#include <boost/serialization/split_free.hpp>

namespace boost {
  namespace serialization {

    template<class Archive>
    void serialize(Archive & ar, cv::Mat& mat, const unsigned int version)
    {
      split_free(ar, mat, version);
    }

    /** Serialization support for cv::Mat */
    template<class Archive>
    void save(Archive &ar, const cv::Mat& m, const unsigned int /*version*/)
    {
      size_t elem_size = m.elemSize(); //CV_ELEM_SIZE(cvmat->type)
      size_t elem_type = m.type();

      ar << m.cols;
      ar << m.rows;
      ar << elem_size;
      ar << elem_type;

      const size_t data_size = m.cols * m.rows * elem_size;
      ar << make_array(m.ptr(), data_size);
    }

    /** Serialization support for cv::Mat */
    template<class Archive>
    void load(Archive &ar, cv::Mat& m, const unsigned int /*version*/)
    {
      int cols, rows;
      size_t elem_size, elem_type;

      std::cout << "load cv mat" << std::endl;

      ar >> cols;
      ar >> rows;
      ar >> elem_size;
      ar >> elem_type;

      m.create(rows, cols, elem_type);

      size_t data_size = m.cols * m.rows * elem_size;
      ar >> make_array(m.ptr(), data_size);
    }

    template<class Archive, class T, int n, int m>
    void serialize(Archive & ar, cv::Matx<T, n, m>& mat, const unsigned int version)
    {
      split_free(ar, mat, version);
    }

    template<class Archive, class T, int n, int m>
    void save(Archive &ar, const cv::Matx<T, n, m>& mat,
              const unsigned int /*version*/)
    {
      int cols = mat.cols;
      int rows = mat.rows;

      ar << cols;
      ar << rows;

      ar << make_array(&mat.val[0], cols * rows);
    }

    template<class Archive, class T, int n, int m>
    void load(Archive &ar, cv::Matx<T, n, m>& mat, const unsigned int /*version*/)
    {
      int cols, rows;

      ar >> cols;
      ar >> rows;

      ar >> make_array(&mat.val[0], cols * rows);
    }

    template<class Archive, class T, int S>
    void serialize(Archive &ar, cv::Vec<T, S>& vec, const unsigned int /*version*/)
    {
      ar & boost::serialization::base_object<cv::Matx<T, S, 1> >(vec);
    }

    template<class Archive, class T>
    void serialize(Archive &ar, cv::Point_<T>& pt, const unsigned int /*version*/)
    {
      ar & pt.x;
      ar & pt.y;
    }

    template<class Archive, class T>
    void serialize(Archive &ar, cv::Point3_<T>& pt, const unsigned int /*version*/)
    {
      ar & pt.x;
      ar & pt.y;
      ar & pt.z;
    }

    template<class Archive>
    void serialize(Archive &ar, cv::KeyPoint& kpt, const unsigned int /*version*/)
    {
      ar & kpt.angle;
      ar & kpt.class_id;
      ar & kpt.octave;
      ar & kpt.pt;
      ar & kpt.response;
      ar & kpt.size;
    }

  } //namespace serialization
} //namespace boost
