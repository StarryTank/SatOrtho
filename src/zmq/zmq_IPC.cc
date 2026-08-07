#include "zmq_IPC.h"

// for crc
#include <unistd.h>  // usleep
#include <cstdint>   // Includes ::std::uint32_t
#include <iomanip>   // Includes ::std::hex
#include "CRC.h"

#define DEBUG_ 0
/******************************* ZMQSender *********************************/
// 指定id来初始化zmq
int ZMQSender::Init(const std::string &dst_id) {
  context_.reset(new zmq::context_t(1));
  publisher_.reset(new zmq::socket_t(*context_, ZMQ_PUB));
  publisher_->bind(dst_id);

  std::cout << "Create publisher to: " << dst_id << std::endl;
}

// 给数据包加帧头、计算crc校验位、发送数据
// 帧头内容包括：起始位0xff, 帧id（4位int），消息类型（MSG_TYPE），
// 发送方代码（写死为1），接收方代码（core为2， ins为3）
// 数据长度（msg长度的1/4）
int ZMQSender::Send(const std::string &data_msg, MSG_TYPE msg_type,
                    int suber_id) {
  // 创建帧头
  std::string header;
  {
    int send_id = 1;
    header.resize(24);
    // start
    int start_flag = 31415927;
    memcpy((char *)header.data(), (char *)&start_flag, 4);
    // id
    static int kf_id = 0;
    memcpy((char *)header.data() + 4, (char *)&kf_id, 4);
    ++kf_id;

    // msg_type
    memcpy((char *)header.data() + 8, (char *)&msg_type, 4);

    // send id
    memcpy((char *)header.data() + 12, (char *)&send_id, 4);

    // sub id
    memcpy((char *)header.data() + 16, (char *)&suber_id, 4);

    // data len
    int data_len = data_msg.size();
    memcpy((char *)header.data() + 20, (char *)&data_len, 4);
#if DEBUG_
    std::cout << "[header(send)]: ";
    for (int i = 0; i < header.size(); ++i) std::cout << int(header[i]) << " ";
    std::cout << std::endl;
#endif
  }

  std::string msg;
  msg = header + data_msg;
  // 计算校验码
  uint32_t crc = CRC::Calculate(msg.c_str(), msg.size(), CRC::CRC_32());

#if DEBUG_
  header = msg.substr(0, 24);
  std::cout << "[header(send after crc)]: ";
  for (int i = 0; i < header.size(); ++i) std::cout << int(header[i]) << " ";
  std::cout << std::endl;
#endif

  // 组建完整消息发送包
  zmq::message_t msg_send(msg.size() + 4);
  memcpy(msg_send.data(), msg.data(), msg.size());
  memcpy(msg_send.data() + msg.size(), (char *)&crc, 4);

  // 发送消息
  publisher_->send(msg_send);
  usleep(10);

  return 1;
}

/******************************* ZMQRecver *********************************/
// 连接到指定的发送端
int ZMQRecver::Init(const std::string &src_id) {
  context_.reset(new zmq::context_t(1));
  recver_.reset(new zmq::socket_t(*context_, ZMQ_SUB));
  recver_->connect(src_id);

  recver_->setsockopt(ZMQ_SUBSCRIBE, "", 0);

  return 1;
}

// 接收发送端送来的数据，进行crc验证，而后取出数据部分，存放在msg中
int ZMQRecver::Recv(std::string &data_msg, MSG_TYPE &msg_type) {
  zmq::message_t msg;
  int flag = recver_->recv(&msg);
#if DEBUG_
  std::cout << "[ZMQRecver::Recv]:flag = " << flag << std::endl;
#endif

  if (!flag) {
    return 0;
  }

  int len = msg.size();
  uint32_t crc32;
  memcpy(&crc32, msg.data() + (len - 4), 4);
  std::string str_msg;
  str_msg.resize(len - 4);
  memcpy((char *)str_msg.data(), msg.data(), len - 4);

  uint32_t crc32_data =
      CRC::Calculate(str_msg.c_str(), str_msg.size(), CRC::CRC_32());
  if (crc32_data != crc32) {
    std::cout << "CRC ERROR!!!\n";
    return -1;
  }

  // 解析出消息类型
  memcpy(&msg_type, str_msg.data() + 8, 4);

  // 打印帧头信息
#if DEBUG_
  std::string header = str_msg.substr(0, 24);
  std::cout << "[header(recv)]: ";
  for (int i = 0; i < header.size(); ++i) std::cout << int(header[i]) << " ";
  std::cout << std::endl;

  std::cout << "Frame header info:\n";
  int start_flag, frame_id, frame_type, send_id, recv_id, msg_len;
  memcpy(&start_flag, str_msg.data(), 4);
  memcpy(&frame_id, str_msg.data() + 4, 4);
  memcpy(&frame_type, str_msg.data() + 8, 4);
  memcpy(&send_id, str_msg.data() + 12, 4);
  memcpy(&recv_id, str_msg.data() + 16, 4);
  memcpy(&msg_len, str_msg.data() + 20, 4);

  std::cout << "start_flag = " << start_flag << "\n";
  std::cout << "frame_id = " << frame_id << "\n";
  std::cout << "frame_type = " << frame_type << "\n";
  std::cout << "send_id = " << send_id << "\n";
  std::cout << "recv_id = " << recv_id << "\n";
  std::cout << "msg_len = " << msg_len << "\n";
#endif
  data_msg = str_msg.substr(24);

  return 1;
}

int Send(zmq::socket_t &socket, const std::string &data_msg,
         const MsgHeader &msg_header) {
  // 创建帧头
  std::string header;
  {
    header.resize(24);
    // start
    int start_flag = 31415927;
    memcpy((char *)header.data(), (char *)&start_flag, 4);
    // id
    memcpy((char *)header.data() + 4, (char *)&msg_header.id_, 4);

    // msg_type
    memcpy((char *)header.data() + 8, (char *)&msg_header.msg_type_, 4);

    // send id
    memcpy((char *)header.data() + 12, (char *)&msg_header.sender_id_, 4);

    // sub id
    memcpy((char *)header.data() + 16, (char *)&msg_header.recver_id_, 4);

    // data len
    int data_len = data_msg.size();
    memcpy((char *)header.data() + 20, (char *)&data_len, 4);
#if DEBUG_
    std::cout << "[header(send)]: ";
    for (int i = 0; i < header.size(); ++i) std::cout << int(header[i]) << " ";
    std::cout << std::endl;
#endif
  }

  std::string msg;
  msg = header + data_msg;
  // 计算校验码
  // uint32_t crc = CRC::Calculate(msg.c_str(), msg.size(), CRC::CRC_32());
  // 仅对部分数据进行crc校验，具体来说，如果长度大于10000，在头部和尾部各取5000
  std::string crc_str;
  if (msg.size() <= 10000) {
    crc_str = msg;
  } else {
    crc_str = msg.substr(0, 5000) + msg.substr(msg.size() - 5000, 5000);
  }
#if DEBUG_
  std::cout << "crc_str_size = " << crc_str.size() << std::endl;
#endif

  uint32_t crc = CRC::Calculate(crc_str.c_str(), crc_str.size(), CRC::CRC_32());
#if DEBUG_
  header = msg.substr(0, 24);
  std::cout << "[header(send after crc)]: ";
  for (int i = 0; i < header.size(); ++i) std::cout << int(header[i]) << " ";
  std::cout << std::endl;
#endif

  // 组建完整消息发送包
  zmq::message_t msg_send(msg.size() + 4);
  memcpy(msg_send.data(), msg.data(), msg.size());
  memcpy(msg_send.data() + msg.size(), (char *)&crc, 4);

  // 发送消息
  bool flag = socket.send(msg_send);
  usleep(10);

  return flag;
}

int Recv(zmq::socket_t &socket, std::string &msg_data, MsgHeader &msg_header) {
  zmq::message_t msg;
  int flag = socket.recv(&msg);
#if DEBUG_
  std::cout << "[Recv]:flag = " << flag << std::endl;
#endif

  if (!flag) {
    return 0;
  }

  int len = msg.size();
  uint32_t crc32;
  memcpy(&crc32, msg.data() + (len - 4), 4);
  std::string str_msg;
  str_msg.resize(len - 4);
  memcpy((char *)str_msg.data(), msg.data(), len - 4);

  // uint32_t crc32_data = CRC::Calculate(str_msg.c_str(), str_msg.size(),
  // CRC::CRC_32());
  std::string crc_str;
  if (str_msg.size() <= 10000) {
    crc_str = str_msg;
  } else {
    crc_str =
        str_msg.substr(0, 5000) + str_msg.substr(str_msg.size() - 5000, 5000);
  }
  uint32_t crc32_data =
      CRC::Calculate(crc_str.c_str(), crc_str.size(), CRC::CRC_32());
  if (crc32_data != crc32) {
    std::cout << "CRC ERROR!!!\n";
    return -1;
  }

  // 解析出消息类型
  memcpy(&msg_header.id_, str_msg.data() + 4, 4);
  memcpy(&msg_header.msg_type_, str_msg.data() + 8, 4);
  memcpy(&msg_header.sender_id_, str_msg.data() + 12, 4);
  memcpy(&msg_header.recver_id_, str_msg.data() + 16, 4);

  // 打印帧头信息
#if DEBUG_
  std::string header = str_msg.substr(0, 24);
  std::cout << "[header(recv)]: ";
  for (int i = 0; i < header.size(); ++i) std::cout << int(header[i]) << " ";
  std::cout << std::endl;

  std::cout << "Frame header info:\n";
  int start_flag, frame_id, frame_type, send_id, recv_id, msg_len;
  memcpy(&start_flag, str_msg.data(), 4);
  memcpy(&frame_id, str_msg.data() + 4, 4);
  memcpy(&frame_type, str_msg.data() + 8, 4);
  memcpy(&send_id, str_msg.data() + 12, 4);
  memcpy(&recv_id, str_msg.data() + 16, 4);
  memcpy(&msg_len, str_msg.data() + 20, 4);

  std::cout << "start_flag = " << start_flag << "\n";
  std::cout << "frame_id = " << frame_id << "\n";
  std::cout << "frame_type = " << frame_type << "\n";
  std::cout << "send_id = " << send_id << "\n";
  std::cout << "recv_id = " << recv_id << "\n";
  std::cout << "msg_len = " << msg_len << "\n";
#endif
  msg_data = str_msg.substr(24);

  return 1;
}
