// 使用zmq从KFG中发送关键帧，该类主要实现对zmq::socket_t publisher的维护
// 对数据帧的打包

#ifndef ZMQ_PUBLISHER_H_
#define ZMQ_PUBLISHER_H_

#include "zmq.hpp"
#include <iostream>
#include <memory>
#include <opencv2/core/core.hpp>
#include <vector>

enum MSG_TYPE { KFDATA, FPOSE, CONTROL, DATA, IMAGE, GPS };

struct MsgHeader {
  int id_;
  MSG_TYPE msg_type_;
  int sender_id_;
  int recver_id_;
};

// 只负责给数据加帧头，加校验位，而后发送数据到指定位置
class ZMQSender {
public:
  // 指定id来初始化zmq
  // kf_dst --- Core模块的地址
  int Init(const std::string &dst_id);

  // 给数据包加帧头、计算crc校验位、发送数据
  // 帧头内容包括：起始位0xff, 帧id（4位int），消息类型（MSG_TYPE），
  // 发送方代码（写死为1），接收方代码（core为2， ins为3）
  // 数据长度（msg长度的1/4）
  int Send(const std::string &msg, MSG_TYPE msg_type, int suber_id);

private:
  // 帧头长度：24byte
  const int FRAME_HEADER_LEN = 24;

  // 发送KF数据
  std::unique_ptr<zmq::socket_t> publisher_;
  std::unique_ptr<zmq::context_t> context_;
};

// 只负责接收数据，验证校验位，而后去掉帧头，解析出数据域
class ZMQRecver {
public:
  // 连接到指定的发送端
  int Init(const std::string &src_id);
  // 接收发送端送来的数据，进行crc验证，而后取出数据部分，存放在msg中
  // 未接收到消息返回0，校验位不对返回-1，正确接收信息返回-1
  int Recv(std::string &msg, MSG_TYPE &msg_type);

private:
  std::unique_ptr<zmq::socket_t> recver_;
  std::unique_ptr<zmq::context_t> context_;
};

int Send(zmq::socket_t &socket, const std::string &msg,
         const MsgHeader &header);
int Recv(zmq::socket_t &socket, std::string &msg, MsgHeader &header);

#endif