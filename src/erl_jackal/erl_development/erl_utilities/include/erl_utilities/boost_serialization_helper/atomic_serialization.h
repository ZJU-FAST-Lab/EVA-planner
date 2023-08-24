#ifndef SERIALIZATION_ATOMIC_H_
#define SERIALIZATION_ATOMIC_H_

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/split_free.hpp>
#include <atomic>

namespace boost {
namespace serialization {

template<class Archive, class T>
inline void save(Archive& ar, const std::atomic<T>& t, const unsigned int) {
    // only the raw pointer has to be saved
    const T value = t.load();
    ar & BOOST_SERIALIZATION_NVP(value);
}

template<class Archive, class T>
inline void load(Archive& ar, std::atomic<T>& t, const unsigned int) {
    T value;
    ar & BOOST_SERIALIZATION_NVP(value);
    t.store(value);
}

template<class Archive, class T>
inline void serialize(Archive& ar, std::atomic<T>& t, const unsigned int file_version) {
    boost::serialization::split_free(ar, t, file_version);
}

template<class Archive, class T>
inline void save(Archive& ar, const std::__atomic_base<T>& t, const unsigned int) {
    // only the raw pointer has to be saved
    const T value = t.load();
    ar & BOOST_SERIALIZATION_NVP(value);
}

template<class Archive, class T>
inline void load(Archive& ar, std::__atomic_base<T>& t, const unsigned int) {
    T value;
    ar & BOOST_SERIALIZATION_NVP(value);
    t.store(value);
}

template<class Archive, class T>
inline void serialize(Archive& ar, std::__atomic_base<T> &t, const unsigned int file_version) {
    boost::serialization::split_free(ar, t, file_version);
}

} // namespace serialization
} // namespace boost

#endif //SERIALIZATION_ATOMIC_H_
