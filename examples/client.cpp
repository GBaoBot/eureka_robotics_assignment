// Minimal C++ client: RPC, map, wheels, state and camera streams.
// Build: g++ -std=c++17 -O2 client.cpp -o client -lzmq   (needs cppzmq + nlohmann-json headers)
// Run:   ./client [host]
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>
#include <zmq.hpp>
#include <zmq_addon.hpp>

using json = nlohmann::json;

struct Client {
  zmq::context_t ctx;
  zmq::socket_t rpc{ctx, zmq::socket_type::req}, cmd{ctx, zmq::socket_type::push};
  zmq::socket_t state{ctx, zmq::socket_type::sub}, camera{ctx, zmq::socket_type::sub};

  explicit Client(const std::string& host) {
    auto url = [&](int k) { return "tcp://" + host + ":" + std::to_string(5555 + k); };
    camera.set(zmq::sockopt::rcvhwm, 2);
    rpc.connect(url(0));
    cmd.connect(url(1));
    state.connect(url(2));
    camera.connect(url(3));
    state.set(zmq::sockopt::subscribe, "");
    camera.set(zmq::sockopt::subscribe, "");
  }

  // Returns the result; extra binary frames (e.g. the map PNG) go to `blobs`.
  json call(const std::string& method, json params = json::object(), std::vector<zmq::message_t>* blobs = nullptr) {
    rpc.send(zmq::buffer(json{{"method", method}, {"params", params}}.dump()));
    std::vector<zmq::message_t> frames;
    (void)zmq::recv_multipart(rpc, std::back_inserter(frames));
    json rep = json::parse(frames[0].to_string());
    if (!rep["ok"]) throw std::runtime_error(rep["error"].get<std::string>());
    if (blobs) std::move(frames.begin() + 1, frames.end(), std::back_inserter(*blobs));
    return rep["result"];
  }

  void send(const json& c) { cmd.send(zmq::buffer(c.dump()), zmq::send_flags::dontwait); }

  // Latest message on a stream, discarding older queued ones.
  std::vector<zmq::message_t> latest(zmq::socket_t& s) {
    std::vector<zmq::message_t> frames;
    (void)zmq::recv_multipart(s, std::back_inserter(frames));
    for (;;) {
      std::vector<zmq::message_t> next;
      if (!zmq::recv_multipart(s, std::back_inserter(next), zmq::recv_flags::dontwait)) return frames;
      frames = std::move(next);
    }
  }
};

int main(int argc, char** argv) {
  Client c(argc > 1 ? argv[1] : "localhost");
  for (auto& s : c.call("list_scenes")) std::cout << "scene " << s["id"] << " " << s["name"] << "\n";
  json scene = c.call("load_scene", {{"id", "01"}, {"seed", 42}});
  std::cout << "start " << scene["start"] << "\n";

  std::vector<zmq::message_t> blobs;
  json map = c.call("get_map", json::object(), &blobs);
  std::ofstream("map.png", std::ios::binary).write(blobs[0].data<char>(), blobs[0].size());
  std::cout << "map " << map << " -> map.png\n";

  json robot = c.call("get_robot_info"), cam = c.call("get_camera_info");
  double wheel_radius = robot["wheel_radius"], wheel_base = robot["wheel_base"], turn_rate = 1.0;
  double wheel_speed = turn_rate * wheel_base / 2 / wheel_radius;  // rad/s per wheel
  for (int i = 0; i < 30; ++i) {  // turn in place at 1 rad/s for 1.5 s; resend at 20 Hz (watchdog 250 ms)
    c.send({{"type", "wheels"}, {"left", -wheel_speed}, {"right", wheel_speed}});
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  json st = json::parse(c.latest(c.state)[0].to_string());
  std::cout << "state t=" << st["t"] << " pose " << st["pose"] << " ticks " << st["wheels"] << "\n";

  auto frames = c.latest(c.camera);
  json hdr = json::parse(frames[0].to_string());
  int W = hdr["width"], H = hdr["height"];
  std::vector<uint16_t> depth(W * H);
  std::memcpy(depth.data(), frames[2].data(), depth.size() * 2);  // little-endian uint16, row-major
  double z = depth[(H / 2) * W + W / 2] * cam["depth_scale"].get<double>();
  std::cout << "camera seq " << hdr["seq"] << ": jpeg " << frames[1].size() << " B, centre depth " << z << " m\n";
}
