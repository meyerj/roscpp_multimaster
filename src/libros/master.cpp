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

#include "roscpp_multimaster/master.h"
#include "roscpp_multimaster/xmlrpc_manager.h"
#include "roscpp_multimaster/this_node.h"
#include "roscpp_multimaster/init.h"
#include "roscpp_multimaster/network.h"

#include "roscpp_multimaster/topic_manager.h"
#include "roscpp_multimaster/service_manager.h"
#include "roscpp_multimaster/connection_manager.h"
#include "roscpp_multimaster/param.h"

#include <ros/console.h>
#include <ros/assert.h>

#include "XmlRpc.h"
#include <boost/lexical_cast.hpp>

namespace ros
{

std::map<std::string,MasterPtr> g_master;
boost::mutex g_master_mutex;
const MasterPtr& Master::instance(const std::string &uri)
{
  if (g_master.find(uri) == g_master.end())
  {
    boost::mutex::scoped_lock lock(g_master_mutex);
    if (g_master.find(uri) == g_master.end())
    {
      MasterPtr master(new Master(uri));
      g_master[uri] = master;
      if (master->getURI() != uri)
      {
        g_master[master->getURI()] = master;
      }
    }
  }

  return g_master[uri];
}

Master::Master(const std::string& uri, const M_string& remappings)
  : port_(0)
  , uri_(uri)
  , nh_refcount_(0)
  , started_(false)
  , node_started_by_nh_(false)
{
  init(remappings);
}

Master::~Master()
{
  shutdown();
}

void Master::init(const M_string& remappings)
{
  if (uri_.empty())
  {
    M_string::const_iterator it = remappings.find("__master");
    if (it != remappings.end())
    {
      uri_ = it->second;
    }

    char *master_uri_env = NULL;
    #ifdef _MSC_VER
      _dupenv_s(&master_uri_env, NULL, "ROS_MASTER_URI");
    #else
      master_uri_env = getenv("ROS_MASTER_URI");
    #endif
    if (!master_uri_env)
    {
      ROS_FATAL( "ROS_MASTER_URI is not defined in the environment. Either " \
                 "type the following or (preferrably) add this to your " \
                 "~/.bashrc file in order set up your " \
                 "local machine as a ROS master:\n\n" \
                 "export ROS_MASTER_URI=http://localhost:11311\n\n" \
                 "then, type 'roscore' in another shell to actually launch " \
                 "the master program.");
      ROS_BREAK();
    }

    uri_ = master_uri_env;

#ifdef _MSC_VER
    // http://msdn.microsoft.com/en-us/library/ms175774(v=vs.80).aspx
    free(master_uri_env);
#endif
  }

  // Split URI into
  if (!network::splitURI(uri_, host_, port_))
  {
    ROS_FATAL( "Couldn't parse the master URI [%s] into a host:port pair.", uri_.c_str());
    ROS_BREAK();
  }
}

void Master::startNodeHandle()
{
  boost::mutex::scoped_lock lock(nh_refcount_mutex_);

  if (nh_refcount_ == 0 && !isStarted())
  {
    node_started_by_nh_ = true;
    start();
  }

  ++nh_refcount_;
}

void Master::shutdownNodeHandle()
{
  boost::mutex::scoped_lock lock(nh_refcount_mutex_);

  --nh_refcount_;

  if (nh_refcount_ == 0 && node_started_by_nh_)
  {
    shutdown();
  }

}

bool Master::isStarted() const
{
  return started_;
}

void Master::start()
{
  boost::mutex::scoped_lock lock(start_mutex_);
  if (started_)
  {
    return;
  }
  started_ = true;

  if (!ros::isStarted())
  {
    ros::start();
  }

  topicManager()->start();
  serviceManager()->start();
  connectionManager()->start();
//  PollManager::instance()->start();
  xmlRpcManager()->start();
}

void Master::shutdown()
{
  boost::mutex::scoped_lock lock(start_mutex_);
  if (!started_)
  {
    return;
  }
  started_ = false;

  topicManager()->shutdown();
  serviceManager()->shutdown();
//  PollManager::instance()->shutdown();
  connectionManager()->shutdown();
  xmlRpcManager()->shutdown();
}

const std::string& Master::getHost()
{
  return host_;
}

uint32_t Master::getPort()
{
  return port_;
}

const std::string& Master::getURI()
{
  return uri_;
}

void Master::setRetryTimeout(ros::WallDuration timeout)
{
  if (timeout < ros::WallDuration(0))
  {
    ROS_FATAL("retry timeout must not be negative.");
    ROS_BREAK();
  }
  retry_timeout_ = timeout;
}

bool Master::check()
{
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = getCallerId();
  return execute("getPid", args, result, payload, false);
}

bool Master::getTopics(master::V_TopicInfo& topics)
{
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = getCallerId();
  args[1] = ""; //TODO: Fix this

  if (!execute("getPublishedTopics", args, result, payload, true))
  {
    return false;
  }

  topics.clear();
  for (int i = 0; i < payload.size(); i++)
  {
    topics.push_back(master::TopicInfo(std::string(payload[i][0]), std::string(payload[i][1])));
  }

  return true;
}

bool Master::getNodes(V_string& nodes)
{
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = getCallerId();

  if (!execute("getSystemState", args, result, payload, true))
  {
    return false;
  }

  S_string node_set;
  for (int i = 0; i < payload.size(); ++i)
  {
    for (int j = 0; j < payload[i].size(); ++j)
    {
      XmlRpc::XmlRpcValue val = payload[i][j][1];
      for (int k = 0; k < val.size(); ++k)
      {
        std::string name = payload[i][j][1][k];
        node_set.insert(name);
      }
    }
  }

  nodes.insert(nodes.end(), node_set.begin(), node_set.end());

  return true;
}

bool Master::execute(const std::string& method, const XmlRpc::XmlRpcValue& request, XmlRpc::XmlRpcValue& response, XmlRpc::XmlRpcValue& payload, bool wait_for_master)
{
  ros::WallTime start_time = ros::WallTime::now();

  std::string master_host = getHost();
  uint32_t master_port = getPort();
  XmlRpc::XmlRpcClient *c = xmlRpcManager()->getXMLRPCClient(master_host, master_port, "/");
  bool printed = false;
  bool slept = false;
  bool ok = true;
  do
  {
    bool b = false;
    {
#if defined(__APPLE__)
      boost::mutex::scoped_lock lock(xmlrpc_call_mutex_);
#endif

      b = c->execute(method.c_str(), request, response);
    }

    ok = !ros::isShuttingDown() && !xmlRpcManager()->isShuttingDown();

    if (!b && ok)
    {
      if (!printed && wait_for_master)
      {
        ROS_ERROR("[%s] Failed to contact master at [%s:%d].  %s", method.c_str(), master_host.c_str(), master_port, wait_for_master ? "Retrying..." : "");
        printed = true;
      }

      if (!wait_for_master)
      {
        xmlRpcManager()->releaseXMLRPCClient(c);
        return false;
      }

      if (!retry_timeout_.isZero() && (ros::WallTime::now() - start_time) >= retry_timeout_)
      {
        ROS_ERROR("[%s] Timed out trying to connect to the master after [%f] seconds", method.c_str(), retry_timeout_.toSec());
        xmlRpcManager()->releaseXMLRPCClient(c);
        return false;
      }

      ros::WallDuration(0.05).sleep();
      slept = true;
    }
    else
    {
      if (!xmlRpcManager()->validateXmlrpcResponse(method, response, payload))
      {
        xmlRpcManager()->releaseXMLRPCClient(c);

        return false;
      }

      break;
    }

    ok = !ros::isShuttingDown() && !xmlRpcManager()->isShuttingDown();
  } while(ok);

  if (ok && slept)
  {
    ROS_INFO("Connected to master at [%s:%d]", master_host.c_str(), master_port);
  }

  xmlRpcManager()->releaseXMLRPCClient(c);

  return true;
}

const TopicManagerPtr& Master::topicManager()
{
  if (!topic_manager_)
  {
    boost::mutex::scoped_lock lock(topic_manager_mutex_);
    if (!topic_manager_)
    {
      topic_manager_.reset(new TopicManager(shared_from_this()));
    }
  }

  return topic_manager_;
}

const ServiceManagerPtr& Master::serviceManager()
{
  if (!service_manager_)
  {
    boost::mutex::scoped_lock lock(service_manager_mutex_);
    if (!service_manager_)
    {
      service_manager_.reset(new ServiceManager(shared_from_this()));
    }
  }

  return service_manager_;
}

const ParametersPtr& Master::parameters()
{
  if (!parameters_)
  {
    boost::mutex::scoped_lock lock(parameters_mutex_);
    if (!parameters_)
    {
      parameters_.reset(new Parameters(shared_from_this()));
    }
  }

  return parameters_;
}

const ConnectionManagerPtr& Master::connectionManager()
{
  if (!connection_manager_)
  {
    boost::mutex::scoped_lock lock(connection_manager_mutex_);
    if (!connection_manager_)
    {
      connection_manager_.reset(new ConnectionManager(shared_from_this()));
    }
  }

  return connection_manager_;
}

const XMLRPCManagerPtr& Master::xmlRpcManager()
{
  if (!xmlrpc_manager_)
  {
    boost::mutex::scoped_lock lock(xmlrpc_manager_mutex_);
    if (!xmlrpc_manager_)
    {
      xmlrpc_manager_.reset(new XMLRPCManager);
    }
  }

  return xmlrpc_manager_;
}

std::string Master::getCallerId()
{
  std::string caller_id = this_node::getName();
  if (uri_ != "http://localhost:11311") {
    caller_id += "_" + network::getHost() + "_" + boost::lexical_cast<std::string>(xmlRpcManager()->getServerPort());
  }
  return caller_id;
}

namespace master
{

void init(const M_string& remappings)
{
  Master::instance()->init(remappings);
}

const std::string& getHost()
{
  return Master::instance()->getHost();
}

uint32_t getPort()
{
  return Master::instance()->getPort();
}

const std::string& getURI()
{
  return Master::instance()->getURI();
}

void setRetryTimeout(ros::WallDuration timeout)
{
  Master::instance()->setRetryTimeout(timeout);
}

bool check()
{
  return Master::instance()->check();
}

bool getTopics(V_TopicInfo& topics)
{
  return Master::instance()->getTopics(topics);
}

bool getNodes(V_string& nodes)
{
  return Master::instance()->getNodes(nodes);
}

bool execute(const std::string& method, const XmlRpc::XmlRpcValue& request, XmlRpc::XmlRpcValue& response, XmlRpc::XmlRpcValue& payload, bool wait_for_master)
{
  return Master::instance()->execute(method, request, response, payload, wait_for_master);
}

} // namespace master

} // namespace ros
