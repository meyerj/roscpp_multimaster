/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "roscpp_multimaster/param.h"
#include "roscpp_multimaster/master.h"
#include "roscpp_multimaster/xmlrpc_manager.h"
#include "roscpp_multimaster/this_node.h"
#include "roscpp_multimaster/names.h"
#include "roscpp_multimaster/master.h"

#include <ros/console.h>

#include <boost/lexical_cast.hpp>

#include <vector>
#include <map>

namespace ros
{

Parameters::Parameters(const MasterPtr& master)
: master_(master)
{
}

Parameters::~Parameters()
{
}

void Parameters::invalidateParentParams(const std::string& key)
{
  std::string ns_key = names::parentNamespace(key);
  while (ns_key != "" && ns_key != "/")
  {
    if (subscribed_params_.find(ns_key) != subscribed_params_.end())
    {
      // by erasing the key the parameter will be re-queried
      params_.erase(ns_key);
    }
    ns_key = names::parentNamespace(ns_key);
  }
}

void Parameters::set(const std::string& key, const XmlRpc::XmlRpcValue& v)
{
  std::string mapped_key = ros::names::resolve(key);

  XmlRpc::XmlRpcValue params, result, payload;
  params[0] = this_node::getName();
  params[1] = mapped_key;
  params[2] = v;

  {
    // Lock around the execute to the master in case we get a parameter update on this value between
    // executing on the master and setting the parameter in the params_ list.
    boost::mutex::scoped_lock lock(params_mutex_);

    if (master::execute("setParam", params, result, payload, true))
    {
      // Update our cached params list now so that if get() is called immediately after param::set()
      // we already have the cached state and our value will be correct
      if (subscribed_params_.find(mapped_key) != subscribed_params_.end())
      {
        params_[mapped_key] = v;
      }
      invalidateParentParams(mapped_key);
    }
  }
}

void Parameters::set(const std::string& key, const std::string& s)
{
  // construct xmlrpc_c::value object of the std::string and
  // call param::set(key, xmlvalue);
  XmlRpc::XmlRpcValue v(s);
  ros::param::set(key, v);
}

void Parameters::set(const std::string& key, const char* s)
{
  // construct xmlrpc_c::value object of the std::string and
  // call param::set(key, xmlvalue);
  std::string sxx = std::string(s);
  XmlRpc::XmlRpcValue v(sxx);
  ros::param::set(key, v);
}

void Parameters::set(const std::string& key, double d)
{
  XmlRpc::XmlRpcValue v(d);
  ros::param::set(key, v);
}

void Parameters::set(const std::string& key, int i)
{
  XmlRpc::XmlRpcValue v(i);
  ros::param::set(key, v);
}

void Parameters::set(const std::string& key, bool b)
{
  XmlRpc::XmlRpcValue v(b);
  ros::param::set(key, v);
}

template <class T>
  void Parameters::setImpl(const std::string& key, const std::vector<T>& vec)
{
  // Note: the XmlRpcValue starts off as "invalid" and assertArray turns it
  // into an array type with the given size
  XmlRpc::XmlRpcValue xml_vec;
  xml_vec.setSize(vec.size());

  // Copy the contents into the XmlRpcValue
  for(size_t i=0; i < vec.size(); i++) {
    xml_vec[i] = vec.at(i);
  }

  ros::param::set(key, xml_vec);
}

void Parameters::set(const std::string& key, const std::vector<std::string>& vec)
{
  setImpl(key, vec);
}

void Parameters::set(const std::string& key, const std::vector<double>& vec)
{
  setImpl(key, vec);
}

void Parameters::set(const std::string& key, const std::vector<float>& vec)
{
  setImpl(key, vec);
}

void Parameters::set(const std::string& key, const std::vector<int>& vec)
{
  setImpl(key, vec);
}

void Parameters::set(const std::string& key, const std::vector<bool>& vec)
{
  setImpl(key, vec);
}

template <class T>
  void Parameters::setImpl(const std::string& key, const std::map<std::string, T>& map)
{
  // Note: the XmlRpcValue starts off as "invalid" and assertArray turns it
  // into an array type with the given size
  XmlRpc::XmlRpcValue xml_value;
  const XmlRpc::XmlRpcValue::ValueStruct& xml_map = (const XmlRpc::XmlRpcValue::ValueStruct &)(xml_value);

  // Copy the contents into the XmlRpcValue
  for(typename std::map<std::string, T>::const_iterator it = map.begin(); it != map.end(); ++it) {
    xml_value[it->first] = it->second;
  }

  ros::param::set(key, xml_value);
}

void Parameters::set(const std::string& key, const std::map<std::string, std::string>& map)
{
  setImpl(key, map);
}

void Parameters::set(const std::string& key, const std::map<std::string, double>& map)
{
  setImpl(key, map);
}

void Parameters::set(const std::string& key, const std::map<std::string, float>& map)
{
  setImpl(key, map);
}

void Parameters::set(const std::string& key, const std::map<std::string, int>& map)
{
  setImpl(key, map);
}

void Parameters::set(const std::string& key, const std::map<std::string, bool>& map)
{
  setImpl(key, map);
}

bool Parameters::has(const std::string& key)
{
  XmlRpc::XmlRpcValue params, result, payload;
  params[0] = this_node::getName();
  params[1] = ros::names::resolve(key);
  //params[1] = key;
  // We don't loop here, because validateXmlrpcResponse() returns false
  // both when we can't contact the master and when the master says, "I
  // don't have that param."
  if (!master::execute("hasParam", params, result, payload, false))
  {
    return false;
  }

  return payload;
}

bool Parameters::del(const std::string& key)
{
  std::string mapped_key = ros::names::resolve(key);

  {
    boost::mutex::scoped_lock lock(params_mutex_);

    subscribed_params_.erase(mapped_key);
    params_.erase(mapped_key);
  }

  XmlRpc::XmlRpcValue params, result, payload;
  params[0] = this_node::getName();
  params[1] = mapped_key;
  // We don't loop here, because validateXmlrpcResponse() returns false
  // both when we can't contact the master and when the master says, "I
  // don't have that param."
  if (!master::execute("deleteParam", params, result, payload, false))
  {
    return false;
  }

  return true;
}

bool Parameters::getImpl(const std::string& key, XmlRpc::XmlRpcValue& v, bool use_cache)
{
  std::string mapped_key = ros::names::resolve(key);
  if (mapped_key.empty()) mapped_key = "/";

  if (use_cache)
  {
    boost::mutex::scoped_lock lock(params_mutex_);

    if (subscribed_params_.find(mapped_key) != subscribed_params_.end())
    {
      M_Param::iterator it = params_.find(mapped_key);
      if (it != params_.end())
      {
        if (it->second.valid())
        {
          ROS_DEBUG_NAMED("cached_parameters", "Using cached parameter value for key [%s]", mapped_key.c_str());

          v = it->second;
          return true;
        }
        else
        {
          ROS_DEBUG_NAMED("cached_parameters", "Cached parameter is invalid for key [%s]", mapped_key.c_str());
          return false;
        }
      }
    }
    else
    {
      // parameter we've never seen before, register for update from the master
      if (subscribed_params_.insert(mapped_key).second)
      {
        XmlRpc::XmlRpcValue params, result, payload;
        params[0] = this_node::getName();
        params[1] = master_.lock()->xmlRpcManager()->getServerURI();
        params[2] = mapped_key;

        if (!master::execute("subscribeParam", params, result, payload, false))
        {
          ROS_DEBUG_NAMED("cached_parameters", "Subscribe to parameter [%s]: call to the master failed", mapped_key.c_str());
          subscribed_params_.erase(mapped_key);
          use_cache = false;
        }
        else
        {
          ROS_DEBUG_NAMED("cached_parameters", "Subscribed to parameter [%s]", mapped_key.c_str());
        }
      }
    }
  }

  XmlRpc::XmlRpcValue params, result;
  params[0] = this_node::getName();
  params[1] = mapped_key;

  // We don't loop here, because validateXmlrpcResponse() returns false
  // both when we can't contact the master and when the master says, "I
  // don't have that param."
  bool ret = master::execute("getParam", params, result, v, false);

  if (use_cache)
  {
    boost::mutex::scoped_lock lock(params_mutex_);

    ROS_DEBUG_NAMED("cached_parameters", "Caching parameter [%s] with value type [%d]", mapped_key.c_str(), v.getType());
    params_[mapped_key] = v;
  }

  return ret;
}

bool Parameters::getImpl(const std::string& key, std::string& s, bool use_cache)
{
  XmlRpc::XmlRpcValue v;
  if (!getImpl(key, v, use_cache))
    return false;
  if (v.getType() != XmlRpc::XmlRpcValue::TypeString)
    return false;
  s = std::string(v);
  return true;
}

bool Parameters::getImpl(const std::string& key, double& d, bool use_cache)
{
  XmlRpc::XmlRpcValue v;
  if (!getImpl(key, v, use_cache))
  {
    return false;
  }

  if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    d = (int)v;
  }
  else if (v.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    return false;
  }
  else
  {
    d = v;
  }

  return true;
}

bool Parameters::getImpl(const std::string& key, float& f, bool use_cache)
{
  double d = static_cast<double>(f);
  bool result = getImpl(key, d, use_cache);
  if (result)
    f = static_cast<float>(d);
  return result;
}

bool Parameters::getImpl(const std::string& key, int& i, bool use_cache)
{
  XmlRpc::XmlRpcValue v;
  if (!getImpl(key, v, use_cache))
  {
    return false;
  }

  if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble)
  {
    double d = v;

    if (fmod(d, 1.0) < 0.5)
    {
      d = floor(d);
    }
    else
    {
      d = ceil(d);
    }

    i = d;
  }
  else if (v.getType() != XmlRpc::XmlRpcValue::TypeInt)
  {
    return false;
  }
  else
  {
    i = v;
  }

  return true;
}

bool Parameters::getImpl(const std::string& key, bool& b, bool use_cache)
{
  XmlRpc::XmlRpcValue v;
  if (!getImpl(key, v, use_cache))
    return false;
  if (v.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
    return false;
  b = v;
  return true;
}

bool Parameters::get(const std::string& key, std::string& s)
{
	return getImpl(key, s, false);
}

bool Parameters::get(const std::string& key, double& d)
{
	return getImpl(key, d, false);
}

bool Parameters::get(const std::string& key, float& f)
{
	return getImpl(key, f, false);
}

bool Parameters::get(const std::string& key, int& i)
{
	return getImpl(key, i, false);
}

bool Parameters::get(const std::string& key, bool& b)
{
	return getImpl(key, b, false);
}

bool Parameters::get(const std::string& key, XmlRpc::XmlRpcValue& v)
{
	return getImpl(key, v, false);
}

bool Parameters::getCached(const std::string& key, std::string& s)
{
	return getImpl(key, s, true);
}

bool Parameters::getCached(const std::string& key, double& d)
{
	return getImpl(key, d, true);
}

bool Parameters::getCached(const std::string& key, float& f)
{
	return getImpl(key, f, true);
}

bool Parameters::getCached(const std::string& key, int& i)
{
	return getImpl(key, i, true);
}

bool Parameters::getCached(const std::string& key, bool& b)
{
	return getImpl(key, b, true);
}

bool Parameters::getCached(const std::string& key, XmlRpc::XmlRpcValue& v)
{
	return getImpl(key, v, true);
}

template <class T> T xml_cast(XmlRpc::XmlRpcValue xml_value) 
{
  return static_cast<T>(xml_value);
}

template <class T> bool xml_castable(int XmlType) 
{
  return false;
}

template<> bool xml_castable<std::string>(int XmlType)
{
  return XmlType == XmlRpc::XmlRpcValue::TypeString;
}

template<> bool xml_castable<double>(int XmlType)
{
  return ( 
      XmlType == XmlRpc::XmlRpcValue::TypeDouble ||
      XmlType == XmlRpc::XmlRpcValue::TypeInt ||
      XmlType == XmlRpc::XmlRpcValue::TypeBoolean );
}

template<> bool xml_castable<float>(int XmlType)
{
  return ( 
      XmlType == XmlRpc::XmlRpcValue::TypeDouble ||
      XmlType == XmlRpc::XmlRpcValue::TypeInt ||
      XmlType == XmlRpc::XmlRpcValue::TypeBoolean );
}

template<> bool xml_castable<int>(int XmlType)
{
  return ( 
      XmlType == XmlRpc::XmlRpcValue::TypeDouble ||
      XmlType == XmlRpc::XmlRpcValue::TypeInt ||
      XmlType == XmlRpc::XmlRpcValue::TypeBoolean );
}

template<> bool xml_castable<bool>(int XmlType)
{
  return ( 
      XmlType == XmlRpc::XmlRpcValue::TypeDouble ||
      XmlType == XmlRpc::XmlRpcValue::TypeInt ||
      XmlType == XmlRpc::XmlRpcValue::TypeBoolean );
}

template<> double xml_cast(XmlRpc::XmlRpcValue xml_value)
{
  using namespace XmlRpc;
  switch(xml_value.getType()) {
    case XmlRpcValue::TypeDouble:
      return static_cast<double>(xml_value);
    case XmlRpcValue::TypeInt:
      return static_cast<double>(static_cast<int>(xml_value));
    case XmlRpcValue::TypeBoolean:
      return static_cast<double>(static_cast<bool>(xml_value));
    default:
     return 0.0;
  };
}

template<> float xml_cast(XmlRpc::XmlRpcValue xml_value)
{
  using namespace XmlRpc;
  switch(xml_value.getType()) {
    case XmlRpcValue::TypeDouble:
      return static_cast<float>(static_cast<double>(xml_value));
    case XmlRpcValue::TypeInt:
      return static_cast<float>(static_cast<int>(xml_value));
    case XmlRpcValue::TypeBoolean:
      return static_cast<float>(static_cast<bool>(xml_value));
    default:
      return 0.0f;
  };
}

template<> int xml_cast(XmlRpc::XmlRpcValue xml_value)
{
  using namespace XmlRpc;
  switch(xml_value.getType()) {
    case XmlRpcValue::TypeDouble:
      return static_cast<int>(static_cast<double>(xml_value));
    case XmlRpcValue::TypeInt:
      return static_cast<int>(xml_value);
    case XmlRpcValue::TypeBoolean:
      return static_cast<int>(static_cast<bool>(xml_value));
    default:
      return 0;
  };
}

template<> bool xml_cast(XmlRpc::XmlRpcValue xml_value)
{
  using namespace XmlRpc;
  switch(xml_value.getType()) {
    case XmlRpcValue::TypeDouble:
      return static_cast<bool>(static_cast<double>(xml_value));
    case XmlRpcValue::TypeInt:
      return static_cast<bool>(static_cast<int>(xml_value));
    case XmlRpcValue::TypeBoolean:
      return static_cast<bool>(xml_value);
    default:
      return false;
  };
}
  
template <class T>
  bool Parameters::getImpl(const std::string& key, std::vector<T>& vec, bool cached)
{
  XmlRpc::XmlRpcValue xml_array;
  if(!getImpl(key, xml_array, cached)) {
    return false;
  }

  // Make sure it's an array type
  if(xml_array.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return false;
  }

  // Resize the target vector (destructive)
  vec.resize(xml_array.size());

  // Fill the vector with stuff
  for (int i = 0; i < xml_array.size(); i++) {
    if(!xml_castable<T>(xml_array[i].getType())) {
      return false;
    }

    vec[i] = xml_cast<T>(xml_array[i]);
  }

  return true;
}

bool Parameters::get(const std::string& key, std::vector<std::string>& vec)
{
  return getImpl(key, vec, false);
}
bool Parameters::get(const std::string& key, std::vector<double>& vec)
{
  return getImpl(key, vec, false);
}
bool Parameters::get(const std::string& key, std::vector<float>& vec)
{
  return getImpl(key, vec, false);
}
bool Parameters::get(const std::string& key, std::vector<int>& vec)
{
  return getImpl(key, vec, false);
}
bool Parameters::get(const std::string& key, std::vector<bool>& vec)
{
  return getImpl(key, vec, false);
}

bool Parameters::getCached(const std::string& key, std::vector<std::string>& vec)
{
  return getImpl(key, vec, true);
}
bool Parameters::getCached(const std::string& key, std::vector<double>& vec)
{
  return getImpl(key, vec, true);
}
bool Parameters::getCached(const std::string& key, std::vector<float>& vec)
{
  return getImpl(key, vec, true);
}
bool Parameters::getCached(const std::string& key, std::vector<int>& vec)
{
  return getImpl(key, vec, true);
}
bool Parameters::getCached(const std::string& key, std::vector<bool>& vec)
{
  return getImpl(key, vec, true);
}

template <class T>
  bool Parameters::getImpl(const std::string& key, std::map<std::string, T>& map, bool cached)
{
  XmlRpc::XmlRpcValue xml_value;
  if(!getImpl(key, xml_value, cached)) {
    return false;
  }

  // Make sure it's a struct type
  if(xml_value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    return false;
  }

  // Fill the map with stuff
  for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = xml_value.begin();
      it != xml_value.end();
      ++it)
  {
    // Make sure this element is the right type
    if(!xml_castable<T>(it->second.getType())) {
      return false;
    }
    // Store the element
    map[it->first] = xml_cast<T>(it->second);
  }

  return true;
}

bool Parameters::get(const std::string& key, std::map<std::string, std::string>& map)
{
  return getImpl(key, map, false);
}
bool Parameters::get(const std::string& key, std::map<std::string, double>& map)
{
  return getImpl(key, map, false);
}
bool Parameters::get(const std::string& key, std::map<std::string, float>& map)
{
  return getImpl(key, map, false);
}
bool Parameters::get(const std::string& key, std::map<std::string, int>& map)
{
  return getImpl(key, map, false);
}
bool Parameters::get(const std::string& key, std::map<std::string, bool>& map)
{
  return getImpl(key, map, false);
}

bool Parameters::getCached(const std::string& key, std::map<std::string, std::string>& map)
{
  return getImpl(key, map, true);
}
bool Parameters::getCached(const std::string& key, std::map<std::string, double>& map)
{
  return getImpl(key, map, true);
}
bool Parameters::getCached(const std::string& key, std::map<std::string, float>& map)
{
  return getImpl(key, map, true);
}
bool Parameters::getCached(const std::string& key, std::map<std::string, int>& map)
{
  return getImpl(key, map, true);
}
bool Parameters::getCached(const std::string& key, std::map<std::string, bool>& map)
{
  return getImpl(key, map, true);
}


bool Parameters::search(const std::string& key, std::string& result_out)
{
  return search(this_node::getName(), key, result_out);
}

bool Parameters::search(const std::string& ns, const std::string& key, std::string& result_out)
{
  XmlRpc::XmlRpcValue params, result, payload;
  params[0] = ns;

  // searchParam needs a separate form of remapping -- remapping on the unresolved name, rather than the
  // resolved one.

  std::string remapped = key;
  M_string::const_iterator it = names::getUnresolvedRemappings().find(key);
  if (it != names::getUnresolvedRemappings().end())
  {
    remapped = it->second;
  }

  params[1] = remapped;
  // We don't loop here, because validateXmlrpcResponse() returns false
  // both when we can't contact the master and when the master says, "I
  // don't have that param."
  if (!master::execute("searchParam", params, result, payload, false))
  {
    return false;
  }

  result_out = (std::string)payload;

  return true;
}

void Parameters::update(const std::string& key, const XmlRpc::XmlRpcValue& v)
{
  std::string clean_key = names::clean(key);
  ROS_DEBUG_NAMED("cached_parameters", "Received parameter update for key [%s]", clean_key.c_str());

  boost::mutex::scoped_lock lock(params_mutex_);

  if (subscribed_params_.find(clean_key) != subscribed_params_.end())
  {
    params_[clean_key] = v;
  }
  invalidateParentParams(clean_key);
}

void Parameters::paramUpdateCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  result[0] = 1;
  result[1] = std::string("");
  result[2] = 0;

  update((std::string)params[1], params[2]);
}

void Parameters::init(const M_string& remappings)
{
  M_string::const_iterator it = remappings.begin();
  M_string::const_iterator end = remappings.end();
  for (; it != end; ++it)
  {
    const std::string& name = it->first;
    const std::string& param = it->second;

    if (name.size() < 2)
    {
      continue;
    }

    if (name[0] == '_' && name[1] != '_')
    {
      std::string local_name = "~" + name.substr(1);

      bool success = false;

      try
      {
        int32_t i = boost::lexical_cast<int32_t>(param);
        ros::param::set(names::resolve(local_name), i);
        success = true;
      }
      catch (boost::bad_lexical_cast&)
      {

      }

      if (success)
      {
        continue;
      }

      try
      {
        double d = boost::lexical_cast<double>(param);
        ros::param::set(names::resolve(local_name), d);
        success = true;
      }
      catch (boost::bad_lexical_cast&)
      {

      }

      if (success)
      {
        continue;
      }

      if (param == "true" || param == "True" || param == "TRUE")
      {
        ros::param::set(names::resolve(local_name), true);
      }
      else if (param == "false" || param == "False" || param == "FALSE")
      {
        ros::param::set(names::resolve(local_name), false);
      }
      else
      {
        ros::param::set(names::resolve(local_name), param);
      }
    }
  }

  master_.lock()->xmlRpcManager()->bind("paramUpdate", boost::bind(&Parameters::paramUpdateCallback, this, _1, _2));
}


namespace param
{

void set(const std::string& key, const XmlRpc::XmlRpcValue& v)
{
  Master::instance()->parameters()->set(key, v);
}

void set(const std::string& key, const std::string& s)
{
  Master::instance()->parameters()->set(key, s);
}

void set(const std::string& key, const char* s)
{
  Master::instance()->parameters()->set(key, s);
}

void set(const std::string& key, double d)
{
  Master::instance()->parameters()->set(key, d);
}

void set(const std::string& key, int i)
{
  Master::instance()->parameters()->set(key, i);
}

void set(const std::string& key, bool b)
{
  Master::instance()->parameters()->set(key, b);
}

void set(const std::string& key, const std::vector<std::string>& vec)
{
  Master::instance()->parameters()->set(key, vec);
}

void set(const std::string& key, const std::vector<double>& vec)
{
  Master::instance()->parameters()->set(key, vec);
}

void set(const std::string& key, const std::vector<float>& vec)
{
  Master::instance()->parameters()->set(key, vec);
}

void set(const std::string& key, const std::vector<int>& vec)
{
  Master::instance()->parameters()->set(key, vec);
}

void set(const std::string& key, const std::vector<bool>& vec)
{
  Master::instance()->parameters()->set(key, vec);
}

void set(const std::string& key, const std::map<std::string, std::string>& map)
{
  Master::instance()->parameters()->set(key, map);
}

void set(const std::string& key, const std::map<std::string, double>& map)
{
  Master::instance()->parameters()->set(key, map);
}

void set(const std::string& key, const std::map<std::string, float>& map)
{
  Master::instance()->parameters()->set(key, map);
}

void set(const std::string& key, const std::map<std::string, int>& map)
{
  Master::instance()->parameters()->set(key, map);
}

void set(const std::string& key, const std::map<std::string, bool>& map)
{
  Master::instance()->parameters()->set(key, map);
}

bool has(const std::string& key)
{
  return Master::instance()->parameters()->has(key);
}

bool del(const std::string& key)
{
  return Master::instance()->parameters()->del(key);
}

bool get(const std::string& key, std::string& s)
{
  return Master::instance()->parameters()->get(key, s);
}

bool get(const std::string& key, double& d)
{
  return Master::instance()->parameters()->get(key, d);
}

bool get(const std::string& key, float& f)
{
  return Master::instance()->parameters()->get(key, f);
}

bool get(const std::string& key, int& i)
{
  return Master::instance()->parameters()->get(key, i);
}

bool get(const std::string& key, bool& b)
{
  return Master::instance()->parameters()->get(key, b);
}

bool get(const std::string& key, XmlRpc::XmlRpcValue& v)
{
  return Master::instance()->parameters()->get(key, v);
}

bool getCached(const std::string& key, std::string& s)
{
  return Master::instance()->parameters()->getCached(key, s);
}

bool getCached(const std::string& key, double& d)
{
  return Master::instance()->parameters()->getCached(key, d);
}

bool getCached(const std::string& key, float& f)
{
  return Master::instance()->parameters()->getCached(key, f);
}

bool getCached(const std::string& key, int& i)
{
  return Master::instance()->parameters()->getCached(key, i);
}

bool getCached(const std::string& key, bool& b)
{
  return Master::instance()->parameters()->getCached(key, b);
}

bool getCached(const std::string& key, XmlRpc::XmlRpcValue& v)
{
  return Master::instance()->parameters()->getCached(key, v);
}

bool get(const std::string& key, std::vector<std::string>& vec)
{
  return Master::instance()->parameters()->get(key, vec);
}
bool get(const std::string& key, std::vector<double>& vec)
{
  return Master::instance()->parameters()->get(key, vec);
}
bool get(const std::string& key, std::vector<float>& vec)
{
  return Master::instance()->parameters()->get(key, vec);
}
bool get(const std::string& key, std::vector<int>& vec)
{
  return Master::instance()->parameters()->get(key, vec);
}
bool get(const std::string& key, std::vector<bool>& vec)
{
  return Master::instance()->parameters()->get(key, vec);
}

bool getCached(const std::string& key, std::vector<std::string>& vec)
{
  return Master::instance()->parameters()->getCached(key, vec);
}
bool getCached(const std::string& key, std::vector<double>& vec)
{
  return Master::instance()->parameters()->getCached(key, vec);
}
bool getCached(const std::string& key, std::vector<float>& vec)
{
  return Master::instance()->parameters()->getCached(key, vec);
}
bool getCached(const std::string& key, std::vector<int>& vec)
{
  return Master::instance()->parameters()->getCached(key, vec);
}
bool getCached(const std::string& key, std::vector<bool>& vec)
{
  return Master::instance()->parameters()->getCached(key, vec);
}

bool get(const std::string& key, std::map<std::string, std::string>& map)
{
  return Master::instance()->parameters()->getCached(key, map);
}
bool get(const std::string& key, std::map<std::string, double>& map)
{
  return Master::instance()->parameters()->getCached(key, map);
}
bool get(const std::string& key, std::map<std::string, float>& map)
{
  return Master::instance()->parameters()->getCached(key, map);
}
bool get(const std::string& key, std::map<std::string, int>& map)
{
  return Master::instance()->parameters()->getCached(key, map);
}
bool get(const std::string& key, std::map<std::string, bool>& map)
{
  return Master::instance()->parameters()->getCached(key, map);
}

bool getCached(const std::string& key, std::map<std::string, std::string>& map)
{
  return Master::instance()->parameters()->getCached(key, map);
}
bool getCached(const std::string& key, std::map<std::string, double>& map)
{
  return Master::instance()->parameters()->getCached(key, map);
}
bool getCached(const std::string& key, std::map<std::string, float>& map)
{
  return Master::instance()->parameters()->getCached(key, map);
}
bool getCached(const std::string& key, std::map<std::string, int>& map)
{
  return Master::instance()->parameters()->getCached(key, map);
}
bool getCached(const std::string& key, std::map<std::string, bool>& map)
{
  return Master::instance()->parameters()->getCached(key, map);
}


bool search(const std::string& key, std::string& result_out)
{
  return Master::instance()->parameters()->search(key, result_out);
}

bool search(const std::string& ns, const std::string& key, std::string& result_out)
{
  return Master::instance()->parameters()->search(ns, key, result_out);
}

void update(const std::string& key, const XmlRpc::XmlRpcValue& v)
{
  Master::instance()->parameters()->update(key, v);
}

void init(const M_string& remappings)
{
  Master::instance()->parameters()->init(remappings);
}

} // namespace param

} // namespace ros
