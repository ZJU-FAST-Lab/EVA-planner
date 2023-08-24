#ifndef SERIALIZATION_BITSET_H_
#define SERIALIZATION_BITSET_H_

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/split_free.hpp>
#include <bitset>

namespace boost {
namespace serialization {

template<class Archive, size_t _Nb>
inline void save(Archive& ar, const std::bitset<_Nb>& t, const unsigned int)
{
  const std::string value = t.to_string();
  ar & BOOST_SERIALIZATION_NVP(value);
}

template<class Archive, size_t _Nb>
inline void load(Archive& ar, std::bitset<_Nb>& t, const unsigned int)
{
  std::string value;
  ar & BOOST_SERIALIZATION_NVP(value);

//  t = std::bitset<value.size()>(value);
}

template<class Archive, size_t _Nb>
inline void serialize(Archive& ar, std::bitset<_Nb>& t, const unsigned int file_version)
{
  boost::serialization::split_free(ar, t, file_version);
}

} // namespace serialization
} // namespace boost

#endif //SERIALIZATION_BITSET_H_
