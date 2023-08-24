#ifndef BOOST_SERIALIZATION_IO_SAVELOAD_H
#define BOOST_SERIALIZATION_IO_SAVELOAD_H

// Eigen serialization
#include "save_load_eigen.h"

#include <boost/version.hpp>

// Boost serialization headers
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
//#include <boost/serialization/map.hpp>
//#include <boost/serialization/bitset.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/weak_ptr.hpp>
#include <boost/serialization/tracking.hpp>
#include <boost/serialization/tracking_enum.hpp>

//#if BOOST_VERSION < 105600
//  #include "boost_unordered_map_serialization.h"
//#else
//  #include <boost/serialization/unordered_map.hpp>
//#endif

// Boost filesystem header
#include <boost/filesystem.hpp>

// Boost ptr_container header
#include <boost/ptr_container/serialize_ptr_vector.hpp>

// Boost algorithm header
#include <boost/algorithm/string/predicate.hpp>

// Boost archive headers
#include <boost/archive/iterators/istream_iterator.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

// Boost iostreams headers
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filter/zlib.hpp>

#include <boost/serialization/export.hpp>

#include <fstream>

namespace
{

  struct SaveLoadException : public std::exception
  {
    SaveLoadException(const std::string &message)
    {
      message_ = message;
    }

    const char * what() const throw()
    {
      std::stringstream ss;
      ss << "Property : " << message_ << std::endl;

      return ss.str().c_str();
    }

    std::string message_;
  };

  template<class Archive, class Stream, class Obj>
  bool try_stream_next(Archive &ar, const Stream &/*s*/, Obj &o)
  {
    bool success = false;
    try
    {
      ar >> BOOST_SERIALIZATION_NVP(o);
      success = true;
    }
    catch (const boost::archive::archive_exception &e)
    {
      if (e.code != boost::archive::archive_exception::input_stream_error)
        throw;
    }
    return success;
  }

  bool check_path_file(const std::string& file)
  {
    boost::filesystem::path p(file);
    if (!boost::filesystem::is_directory(p.parent_path()) &&
        p.parent_path() != "")
    {
      throw SaveLoadException("ERROR! Directory not found: " + p.parent_path().string());
    }
    else if (!boost::filesystem::is_regular_file(p))
    {
      throw SaveLoadException("ERROR! File not found: " + p.filename().string() +
                              " in " + p.parent_path().string());
    }
    return true;
  }
} // namespace

namespace serialization
{
  namespace io = boost::iostreams;

  enum class SerializationType
  {
    TEXT = 0,
    BIN,
    CBIN,
    XML
  };

  /**
  * \brief save obj into binari file
  * \param obj & file name
  * \param compress - whether binari file should be compressed or not
  * \see loadBin()
  */
  template <class T>
  void saveBin(const T& o, const std::string& filename, const bool compress)
  {
    std::ofstream ofs(filename.c_str());
    { //Scope ensure out dies before ofs
      if (compress)
      {
        io::filtering_streambuf<io::output> out;
        out.push(io::zlib_compressor(io::zlib::best_speed));
        out.push(ofs);
        boost::archive::binary_oarchive oa(out);
        oa << o;
      }
      else
      {
        boost::archive::binary_oarchive oa(ofs);
        oa << o;
      }
    }
    ofs.close();
  }

  /**
  * @brief load obj from binari file
  * @param obj & file name
  * @param compress - whether binari file is compressed or not
  * @see saveBin()
  */
  template <class T>
  void loadBin(T& o, const std::string& filename, const bool compress)
  {
    check_path_file(filename);

    std::ifstream ifs(filename.c_str());
    { //Scope ensure in dies before ifs
      if (compress)
      {
        io::filtering_streambuf<io::input> in;
        in.push(io::zlib_decompressor());
        in.push(ifs);
        boost::archive::binary_iarchive ia(in);
        ia >> o;
        while (try_stream_next(ia, ifs, o));
      }
      else
      {
        boost::archive::binary_iarchive ia(ifs);
        ia >> o;
        while (try_stream_next(ia, ifs, o));
      }
    }
    ifs.close();
  }

  //////////////
  /**
  * @brief save obj into binari file
  * @param obj & file name
  * @see loadBin()
  */
  template <class T>
  void saveTxt(const T& o, const std::string& filename)
  {
    std::ofstream ofs(filename.c_str());

    { //Scope ensure out dies before ofs
      boost::archive::text_oarchive oa(ofs);
      oa << boost::serialization::make_nvp("o", o);
    }
    ofs.close();
  }

  /**
  * @brief load obj from binari file
  * @param obj & file name
  * @param compress - whether binari file is compressed or not
  * @see saveBin()
  */
  template <class T>
  void loadTxt(T& o, const std::string& filename)
  {
    check_path_file(filename);

    std::ifstream ifs(filename.c_str());
    { //Scope ensure in dies before ifs
      boost::archive::text_iarchive ia(ifs);
      ia >> boost::serialization::make_nvp("o", o);
      while (try_stream_next(ia, ifs, o));
    }
    ifs.close();
  }

  //////////////
  /**
  * @brief save obj into binari file
  * @param obj & file name
  * @see loadBin()
  */
  template <class T>
  void saveXml(const T& o, const std::string& filename)
  {
    std::ofstream ofs(filename.c_str());

    { //Scope ensure out dies before ofs
      boost::archive::xml_oarchive oa(ofs);
      oa << boost::serialization::make_nvp("o", o);
    }
    ofs.close();
  }

  /**
  * @brief load obj from binari file
  * @param obj & file name
  * @param compress - whether binari file is compressed or not
  * @see saveBin()
  */
  template <class T>
  void loadXml(T& o, const std::string& filename)
  {
    check_path_file(filename);

    std::ifstream ifs(filename.c_str());
    { //Scope ensure in dies before ifs
      boost::archive::xml_iarchive ia(ifs);
      ia >> boost::serialization::make_nvp("o", o);
      while (try_stream_next(ia, ifs, o));
    }
    ifs.close();
  }

  template <class T>
  void save(const T& t, const std::string& filename, const SerializationType type)
  {
    switch (type)
    {
    case SerializationType::TEXT:
    {
      saveTxt(t, filename);
      break;
    }
    case SerializationType::BIN:
    {
      saveBin(t, filename, false);
      break;
    }
    case SerializationType::CBIN:
    {
      saveBin(t, filename, true);
      break;
    }
    case SerializationType::XML:
    {
      saveXml(t, filename);
      break;
    }
    }
  }

  template <class T>
  void load(T& t, const std::string& filename, const SerializationType type)
  {
    switch (type)
    {
    case SerializationType::TEXT:
    {
      loadTxt(t, filename);
      break;
    }
    case SerializationType::BIN:
    {
      loadBin(t, filename, false);
      break;
    }
    case SerializationType::CBIN:
    {
      loadBin(t, filename, true);
      break;
    }
    case SerializationType::XML:
    {
      loadXml(t, filename);
      break;
    }
    }
  }

} //namespace serialization

BOOST_CLASS_TRACKING(std::vector< std::vector<int> >,    boost::serialization::track_always)
BOOST_CLASS_TRACKING(std::vector< std::vector<float> >,  boost::serialization::track_always)
BOOST_CLASS_TRACKING(std::vector< std::vector<double> >, boost::serialization::track_always)

#endif //BOOST_SERIALIZATION_IO_SAVELOAD_H
