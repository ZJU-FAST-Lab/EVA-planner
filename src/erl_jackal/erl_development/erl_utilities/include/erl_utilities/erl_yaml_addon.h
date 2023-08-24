#ifndef __YAML_ADDON_H_
#define __YAML_ADDON_H_

#include <yaml-cpp/yaml.h>
namespace erl
{
// YAML
template<typename TP, std::size_t SIZE>
inline YAML::Emitter& operator << (YAML::Emitter& out, const std::array<TP, SIZE>& arr)
{
  out << YAML::BeginSeq;
  for( typename std::array<TP,SIZE>::const_iterator it = arr.begin(); it != arr.end(); ++it )
    out << *it;
  out << YAML::EndSeq;
  return out;
}

template<typename TP, std::size_t SIZE>
inline YAML::Emitter& operator << (YAML::Emitter& out, const std::vector<std::array<TP, SIZE>>& vc)
{
  out << YAML::BeginSeq;
  for( typename std::vector<std::array<TP,SIZE>>::const_iterator it = vc.begin(); 
       it != vc.end(); ++it )
    out << YAML::Flow << *it;
  out << YAML::EndSeq;
  return out;
}
}

//namespace YAML
//{
//template<typename TP, std::size_t SIZE>
//struct convert<std::array<TP,SIZE>>
//{
  //static Node encode(const std::array<TP,SIZE>& rhs) {
    //Node node;
    //for( std::size_t k = 0; k < SIZE; ++k )
      //node.push_back(rhs[k]);
    //return node;
  //}

  //static bool decode(const Node& node, std::array<TP,SIZE>& rhs)
  //{
    //if(!node.IsSequence() || node.size() != SIZE) {
      //return false;
    //}
    //for( std::size_t k = 0; k < SIZE; ++k )
      //rhs[k] = node[k].as<TP>();
    //return true;
  //}
//};

//}


#endif
