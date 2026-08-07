#include "zmq_com.h"
#include "key_frame.h"
#include "local_map.h"
#include "spdlog.h"
#include "spdlog_lyf.h"
#include "zmq_IPC.h"

namespace GVO_ATLAS {
int KeyFrameZmqPub::Init(const std::string& address) {
  // 初始化zmq
  if (!address.empty()) {
    keyframe_context_.reset(new zmq::context_t(1));
    keyframe_pub_.reset(new zmq::socket_t(*keyframe_context_, ZMQ_PUB));
    keyframe_pub_->bind(address);
    SPDLOG_WARN("Success init keyframe_pub_ to address: {}", address);
    return 1;
  } else {
    return 0;
  }
}

void KeyFrameZmqPub::PubKeyFrame(const KeyFramePtr& kf) {
  if (keyframe_pub_ != nullptr) {
    // TODO：填入真实信息
    MsgHeader msg_header;
    msg_header.id_ = kf->GetId();
    msg_header.msg_type_ = DATA;
    msg_header.recver_id_ = 10;
    msg_header.sender_id_ = 5;

    std::string kf_str;
    kf->Pack(kf_str);
    Send(*keyframe_pub_, kf_str, msg_header);
  }
}

int KeyFrameZmqSub::Init(const std::string& address) {
  if (!address.empty()) {
    keyframe_context_.reset(new zmq::context_t(1));
    keyframe_sub_.reset(new zmq::socket_t(*keyframe_context_, ZMQ_SUB));
    keyframe_sub_->setsockopt(ZMQ_SUBSCRIBE, "", 0);
    keyframe_sub_->connect(address);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    SPDLOG_WARN("keyframe_sub_ has been connected to {}", address);
    return 1;
  }
  return 0;
}

int KeyFrameZmqSub::SubKeyFrame(KeyFramePtr& kf) {
  MsgHeader msg_header;
  msg_header.id_ = 0;
  msg_header.msg_type_ = DATA;
  msg_header.recver_id_ = 1;
  msg_header.sender_id_ = 1;

  std::string kf_str;
  KeyFramePtr cur_kf(new KeyFrame);

  if (Recv(*keyframe_sub_, kf_str, msg_header)) {
    SPDLOG_WARN("Success Recv KeyFrame: {}", msg_header.id_); 
    cur_kf->LoadFromStr(kf_str);
    kf = cur_kf;
    return 1;
  }
  return 0;
}



int LocalMapZmqPub::Init(const std::string& address) {
  if (!address.empty()) {
    local_map_context_.reset(new zmq::context_t(1));
    local_map_pub_.reset(new zmq::socket_t(*local_map_context_, ZMQ_PUB));

    int v_true =1;
    local_map_pub_->setsockopt(ZMQ_CONFLATE, &v_true, sizeof(v_true));
    
    local_map_pub_->bind(address);
    SPDLOG_WARN("Success init local_map_pub_ to address: {}", address);
    return 1;
  } else {
    return 0;
  }
}

void LocalMapZmqPub::PubLocalMap(const LocalMapPtr& local_map) {
  if (local_map_pub_ != nullptr) {
    // TODO：填入真实信息
    MsgHeader msg_header;
    msg_header.id_ = 0;
    msg_header.msg_type_ = DATA;
    msg_header.recver_id_ = 10;
    msg_header.sender_id_ = 5;
    std::string local_map_str;
    local_map->Pack(local_map_str);
    Send(*local_map_pub_, local_map_str, msg_header);
  }
}


int LocalMapZmqSub::Init(const std::string& address) {
  if (!address.empty()) {
    local_map_context_.reset(new zmq::context_t(1));
    local_map_sub_.reset(new zmq::socket_t(*local_map_context_, ZMQ_SUB));

    // 开启 conflate 模式，只接收最新的消息
    int v_true =1;
    local_map_sub_->setsockopt(ZMQ_CONFLATE, &v_true, sizeof(v_true));

    local_map_sub_->setsockopt(ZMQ_SUBSCRIBE, "", 0);
    local_map_sub_->connect(address);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    SPDLOG_WARN("local_map_sub_ has been connected to {}", address);
    return 1;
  }
  return 0;
}

int LocalMapZmqSub::SubLocalMap(LocalMapPtr& local_map) {
  MsgHeader msg_header;
  msg_header.id_ = 0;
  msg_header.msg_type_ = DATA;
  msg_header.recver_id_ = 1;
  msg_header.sender_id_ = 1;

  std::string local_map_str;
  LocalMapPtr cur_local_map(new LocalMap);

  if (Recv(*local_map_sub_, local_map_str, msg_header)) {
    SPDLOG_WARN("Success Recv LocalMap: {}", msg_header.id_); 
    cur_local_map->LoadFromStr(local_map_str);
    local_map = cur_local_map;
    return 1;
  }
  return 0;

}

}  // namespace GVO_ATLAS
