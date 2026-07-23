// viper_queue.h

#ifndef VIPER_QUEUE_H_807067cf_90a1_4e0f_8166_a2c8ccd973c4
#define VIPER_QUEUE_H_807067cf_90a1_4e0f_8166_a2c8ccd973c4

#include <mutex>
#include <queue>
#include <condition_variable>
#include <cstdint>
#include <cstddef>

class viper_queue {

 private:

  mutable std::mutex mut;
  std::queue<uint8_t> data_queue;
  std::condition_variable data_cond;
  uint32_t* keep_on;
  std::size_t max_bytes_ = 1u << 20;   // backpressure cap; drop backlog if exceeded
  std::uint64_t dropped_ = 0;          // count of drop-backlog events

 public:
  viper_queue();

  ~viper_queue();

  viper_queue(viper_queue const& other);

  void init(uint32_t*);
  void set_max_bytes(std::size_t max_bytes);
  void push(uint8_t* data,uint32_t count);
  uint32_t wait_and_pop(uint8_t* buf,uint32_t max);
  bool empty() const;

};

#endif
