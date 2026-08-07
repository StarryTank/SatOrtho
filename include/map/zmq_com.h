// 处理zmq收发的类

#ifndef ZMQ_COM_H_
#define ZMQ_COM_H_

#include <memory>

#include "key_frame.h"
#include "local_map.h"
#include "zmq.hpp"


namespace GVO_ATLAS {
class KeyFrameZmqPub {
 public:
  int Init(const std::string& address);
  void PubKeyFrame(const KeyFramePtr& kf);

 private:
  std::shared_ptr<zmq::context_t> keyframe_context_;
  std::shared_ptr<zmq::socket_t> keyframe_pub_;
};

typedef std::shared_ptr<KeyFrameZmqPub> KeyFrameZmqPubPtr;

class KeyFrameZmqSub {
 public:
  int Init(const std::string& address);
  int SubKeyFrame(KeyFramePtr& kf);

 private:
  std::shared_ptr<zmq::context_t> keyframe_context_;
  std::shared_ptr<zmq::socket_t> keyframe_sub_;
};
typedef std::shared_ptr<KeyFrameZmqSub> KeyFrameZmqSubPtr;

class LocalMapZmqPub {
 public:
  int Init(const std::string& address);
  void PubLocalMap(const LocalMapPtr& local_map);

 private:
  std::shared_ptr<zmq::context_t> local_map_context_;
  std::shared_ptr<zmq::socket_t> local_map_pub_;
};
typedef std::shared_ptr<LocalMapZmqPub> LocalMapZmqPubPtr;

class LocalMapZmqSub {
 public:
  int Init(const std::string& address);
  int SubLocalMap(LocalMapPtr& local_map);

 private:
  std::shared_ptr<zmq::context_t> local_map_context_;
  std::shared_ptr<zmq::socket_t> local_map_sub_;
};
typedef std::shared_ptr<LocalMapZmqSub> LocalMapZmqSubPtr;


}  // namespace GVO_ATLAS
#endif