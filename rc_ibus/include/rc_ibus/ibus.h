//
// Created by myx on 2022/11/29.
//

#pragma once

#include <cstdint>
#include <rc_msgs/IbusData.h>

namespace rc_ibus
{
typedef struct
{
  int16_t ch[10];

  int16_t checksum_ibus;
} IBusData_t;

class IBus
{
public:
  IBus();
  ~IBus();
  void init(const char* serial);
  void getData(rc_msgs::IbusData* i_bus_data);
  void read();

private:
  IBusData_t i_bus_data_{};
  int port_{};
  int16_t buff_[32]{};
  uint8_t count = 0;
  uint8_t data_i = 0;
  bool is_update_ = false;
  void unpack(const uint8_t data);
};
}  // namespace rc_ibus
