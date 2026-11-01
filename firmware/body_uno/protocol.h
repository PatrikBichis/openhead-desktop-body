// GENERERAD av python/src/openhead_body/protocol.py (cd python && uv run python -m openhead_body.protocol --header). Redigera inte för hand.
#pragma once
#include <stdint.h>

#define ODM_MAGIC 20292
#define ODM_VERSION 1
#define ODM_TYPE_COMMAND 1
#define ODM_TYPE_STATE 2
#define ODM_TYPE_DIAG 3
#define ODM_N_JOINTS 14
#define ODM_NO_SERVO 255
#define ODM_FLAG_TORQUE 1
#define ODM_FLAG_ESTOP 2
#define ODM_FLAG_RESET 4
#define ODM_ST_WATCHDOG 1
#define ODM_ST_LOW_VOLTAGE 2
#define ODM_ST_SERVO_ERROR 4
#define ODM_ST_ESTOP 8
#define ODM_ST_TORQUE 16
#define ODM_ST_FALL 32
#define ODM_ST_IMU_OK 64
#define ODM_ANTENNA_OFF 255
#define ODM_IMU_GYRO_SCALE 1000
#define ODM_IMU_ACC_SCALE 100
#define ODM_FALL_FWD 0
#define ODM_FALL_BACK 1
#define ODM_FALL_LEFT 2
#define ODM_FALL_RIGHT 3
#define ODM_N_FALL_DIRS 4
#define ODM_REFLEX_KEEP 65535
#define ODM_OP_PING 1
#define ODM_OP_READ 2
#define ODM_OP_WRITE 3
#define ODM_OP_SCAN 4
#define ODM_OP_SET_IDS 5
#define ODM_OP_GET_IDS 6
#define ODM_OP_SET_PARAM 7
#define ODM_OP_GET_INFO 8
#define ODM_OP_SET_REFLEX 9
#define ODM_OP_GET_REFLEX 10
#define ODM_DS_OK 0
#define ODM_DS_TIMEOUT 255
#define ODM_DS_BUSY 254
#define ODM_DS_BAD_ARG 253
#define ODM_PARAM_WATCHDOG_MS 0
#define ODM_PARAM_MAX_STEP 1
#define ODM_PARAM_LOW_MV 2
#define ODM_PARAM_FALL_DEG 3
#define ODM_PARAM_FALL_HORIZON_MS 4
#define ODM_PARAM_FALL_LIMP_MS 5
#define ODM_N_PARAMS 6
#define ODM_FW_VERSION 2
static const uint8_t ODM_DEFAULT_IDS[14] = {20, 21, 22, 23, 24, 30, 31, 32, 33, 10, 11, 12, 13, 14};
static const char* const ODM_JOINT_NAMES[14] = {"left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle", "neck_pitch", "head_pitch", "head_yaw", "head_roll", "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle"};
static const char* const ODM_JOINT_LABELS[14] = {"V höft yaw", "V höft roll", "V höft pitch", "V knä", "V fotled", "Nacke pitch", "Huvud pitch", "Huvud yaw", "Huvud roll", "H höft yaw", "H höft roll", "H höft pitch", "H knä", "H fotled"};  // UTF-8
static const uint16_t ODM_PARAM_DEFAULTS[6] = {150, 40, 6000, 0, 150, 1500};
#define ODM_SIZE_COMMANDPACKET 39
#define ODM_SIZE_STATEPACKET 118
#define ODM_SIZE_DIAGPACKET 45

typedef struct __attribute__((packed)) {
  uint16_t magic;
  uint8_t version;
  uint8_t type;
  uint16_t seq;
  uint8_t flags;
  uint16_t targets[14];
  uint8_t antennas[2];
  uint16_t crc;
} CommandPacket;
static_assert(sizeof(CommandPacket) == 39, "CommandPacket");

typedef struct __attribute__((packed)) {
  uint16_t magic;
  uint8_t version;
  uint8_t type;
  uint16_t seq;
  uint16_t seq_echo;
  uint32_t t_ms;
  uint8_t status;
  uint16_t pos[14];
  int16_t vel[14];
  int16_t load[14];
  uint8_t feet;
  uint16_t bus_mv;
  uint8_t diag[4];
  int16_t imu[6];
  uint16_t crc;
} StatePacket;
static_assert(sizeof(StatePacket) == 118, "StatePacket");

typedef struct __attribute__((packed)) {
  uint16_t magic;
  uint8_t version;
  uint8_t type;
  uint16_t seq;
  uint8_t op;
  uint8_t id;
  uint8_t addr;
  uint8_t len;
  uint8_t status;
  uint8_t data[32];
  uint16_t crc;
} DiagPacket;
static_assert(sizeof(DiagPacket) == 45, "DiagPacket");
