/*
 * Copyright 2022 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "libs/camera/camera.h"

#include "camera.h"
#include "libs/base/check.h"
#include "libs/base/gpio.h"
#include "libs/pmic/pmic.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_csi.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_lpi2c.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_lpi2c_freertos.h"

#include "fsl_gpio.h"
#include "fsl_csi.h"
#include "fsl_mipi_csi2rx.h"
#include "fsl_camera.h"
#include "fsl_camera_receiver.h"
#include "fsl_camera_device.h"
#include "fsl_csi_camera_adapter.h"
#include "fsl_ov5640.h"
#include "fsl_pxp.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#ifdef CPU_MIMXRT1176CVM8A_cm4
#undef DEMO_CAMERA_BUFFER_COUNT
#define DEMO_CAMERA_BUFFER_COUNT 1

#undef DEMO_CAMERA_BUFFER_BPP
#define DEMO_CAMERA_BUFFER_BPP 1
#endif

#define DBG_OUTPUT(...)  printf(__VA_ARGS__)
// #define DBG_OUTPUT(...)

// #define DEBUG_LINE()  printf("D:%s:%d\n", __FILE__, __LINE__)

#ifdef CPU_MIMXRT1176CVM8A_cm4
uint8_t pxp_buffer[1];
#else
// __attribute__((section("NonCacheableCamera,\"aw\",%nobits @")))
// __attribute__((aligned(DEMO_CAMERA_BUFFER_ALIGN)))
// uint8_t
//     pxp_buffer[DEMO_CAMERA_HEIGHT][(DEMO_CAMERA_WIDTH + LINE_PADDING) * 3];
uint8_t pxp_buffer[1];
#endif

#if (__CORTEX_M == 7)
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/cm7/fsl_cache.h"
#elif (__CORTEX_M == 4)
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/cm4/fsl_cache.h"
#endif

// Camera MUX
// TODO: Interchange these values when the enclosure details are clear
#define MUX_BACK_CAMERA 0
#define MUX_FRONT_CAMERA 1

#include <cstring>
#include <memory>

status_t BOARD_Camera_I2C_SendSCCB(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subAddressSize, const uint8_t *txBuff, uint8_t txBuffSize)
{
  return coralmicro::CameraTask::GetSingleton()->Write(
        subAddress, txBuff, txBuffSize) ? kStatus_Success : !kStatus_Success;
}

status_t BOARD_Camera_I2C_ReceiveSCCB(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subAddressSize, uint8_t *rxBuff, uint8_t rxBuffSize)
{
  return coralmicro::CameraTask::GetSingleton()->Read(subAddress, &rxBuff[0]) ? kStatus_Success : !kStatus_Success;
}

void BOARD_PullCameraResetPin(bool pullUp)
{
  printf("BOARD_PullCameraResetPin:%d\n", pullUp);
  coralmicro::GpioSet((coralmicro::Gpio) coralmicro::Gpio::kCamReset, pullUp);
  coralmicro::GpioSet((coralmicro::Gpio) coralmicro::Gpio::kCamReset2, pullUp);
}

void BOARD_PullCameraPowerDownPin(bool pullUp)
{
  printf("BOARD_PullCameraPowerDownPin:%d\n", pullUp);
  coralmicro::GpioSet((coralmicro::Gpio) coralmicro::Gpio::kCamPwrDn, pullUp);
  coralmicro::GpioSet((coralmicro::Gpio) coralmicro::Gpio::kCamPwrDn2, pullUp);
}

namespace coralmicro {
namespace {
constexpr uint8_t kCameraAddress = 0x3c;
constexpr int kFramebufferCount = DEMO_CAMERA_BUFFER_COUNT;
constexpr uint8_t kModelIdHExpected = 0x56;
constexpr uint8_t kModelIdLExpected = 0x40;

constexpr float kRedCoefficient = .2126;
constexpr float kGreenCoefficient = .7152;
constexpr float kBlueCoefficient = .0722;
constexpr float kUint8Max = 255.0;

// Control whether the camera driver toggles board LEDs for status/debug.
// Set both to false to prevent any LED changes from the camera task.
static constexpr bool kCameraUseStatusLed = false;
static constexpr bool kCameraUseUserLed = false;

static void Rgb8888ToRgb(const uint8_t* in, uint8_t* out, int width, int height, int line_padding=LINE_PADDING) {
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
  // BGRA to RGB: swap red and blue channels
  out[(x * 3) + (y * width * 3) + 0] = in[(x * 4) + (y * (width + line_padding) * 4) + 2]; // R
  out[(x * 3) + (y * width * 3) + 1] = in[(x * 4) + (y * (width + line_padding) * 4) + 1]; // G
  out[(x * 3) + (y * width * 3) + 2] = in[(x * 4) + (y * (width + line_padding) * 4) + 0]; // B
    }
  }
}

static void Rgb888ToRgb(const uint8_t* in, uint8_t* out, int width, int height, int line_padding=LINE_PADDING) {
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      // BGRA - BGR
      out[(x * 3) + (y * width * 3) + 0] = in[(x * 3) + (y * (width + line_padding) * 3) + 0];
      out[(x * 3) + (y * width * 3) + 1] = in[(x * 3) + (y * (width + line_padding) * 3) + 1];
      out[(x * 3) + (y * width * 3) + 2] = in[(x * 3) + (y * (width + line_padding) * 3) + 2];
    }
  }
}

} // namespace

int CameraFormatBpp(CameraFormat fmt) {
  switch (fmt) {
    case CameraFormat::kRgb:
      return 3;
    case CameraFormat::kRaw:
    case CameraFormat::kY8:
      return 1;
  }
  return 0;
}

void ResizeNearestNeighbor(const uint8_t* src, int src_w, int src_h,
                           uint8_t* dst, int dst_w, int dst_h, int comps,
                           bool preserve_aspect) {
  int src_p = src_w * comps;
  int dst_p = dst_w * comps;
  float ratio_src = (float)src_w / src_h;
  float ratio_dst = (float)dst_w / dst_h;
  int scaled_w =
      preserve_aspect
          ? (ratio_dst > ratio_src ? src_w * (float)dst_h / src_h : dst_w)
          : dst_w;
  int scaled_h =
      preserve_aspect
          ? (ratio_dst > ratio_src ? dst_h : src_h * (float)dst_w / src_w)
          : dst_h;
  float ratio_x = (float)src_w / scaled_w;
  float ratio_y = (float)src_h / scaled_h;

  for (int y = 0; y < dst_h; y++) {
    if (y >= scaled_h) {
      std::memset(dst, 0, dst_p);
      dst += dst_p;
      continue;
    }

    int offset_y = static_cast<int>(y * ratio_y) * src_p;
    for (int x = 0; x < dst_w; x++) {
      int offset_x = static_cast<int>(x * ratio_x) * comps;
      const uint8_t* src_y = src + offset_y;
      for (int i = 0; i < comps; i++) {
        *dst++ = x < scaled_w ? src_y[offset_x + i] : 0;
      }
    }
  }
}

template <typename Callback>
void BayerInternal(const uint8_t* camera_raw, int width, int height,
                   CameraFilterMethod filter, Callback callback) {
  if (filter == CameraFilterMethod::kNearestNeighbor) {
    bool blue = true, green = false;
    for (int y = 2; y < height - 2; y++) {
      int start = green ? 3 : 2;
      for (int x = start; x < width - 2; x += 2) {
        int g1x = x + 1, g1y = y;
        int g2x = x + 2, g2y = y + 1;
        int r1x, r1y, r2x, r2y;
        int b1x, b1y, b2x, b2y;
        if (blue) {
          r1x = r2x = x + 1;
          r1y = r2y = y + 1;
          b1x = x;
          b1y = y;
          b2x = x + 2;
          b2y = y;
        } else {
          r1x = x;
          r1y = y;
          r2x = x + 2;
          r2y = y;
          b1x = b2x = x + 1;
          b1y = b2y = y + 1;
        }
        uint8_t r1 = camera_raw[r1x + (r1y * width)];
        uint8_t g1 = camera_raw[g1x + (g1y * width)];
        uint8_t b1 = camera_raw[b1x + (b1y * width)];
        uint8_t r2 = camera_raw[r2x + (r2y * width)];
        uint8_t g2 = camera_raw[g2x + (g2y * width)];
        uint8_t b2 = camera_raw[b2x + (b2y * width)];
        callback(x, y, r1, g1, b1);
        callback(x + 1, y, r2, g2, b2);
      }
      blue = !blue;
      green = !green;
    }
  } else if (filter == CameraFilterMethod::kBilinear) {
    int bayer_stride = width;

    size_t bayer_offset = 0;
    for (int y = 2; y < height - 2; y++) {
      bool odd_row = y & 1;
      int x = 1;
      size_t bayer_end = bayer_offset + (width - 2);

      if (odd_row) {
        uint8_t r = (static_cast<uint32_t>(camera_raw[bayer_offset + 1]) +
                     static_cast<uint32_t>(
                         camera_raw[bayer_offset + (bayer_stride * 2 + 1)]) +
                     1) >>
                    1;
        uint8_t b =
            (static_cast<uint32_t>(camera_raw[bayer_offset + bayer_stride]) +
             static_cast<uint32_t>(
                 camera_raw[bayer_offset + (bayer_stride + 2)]) +
             1) >>
            1;
        uint8_t g = camera_raw[bayer_offset + (bayer_stride + 1)];
        callback(x, y, r, g, b);
        bayer_offset += 1;
        ++x;
      }

      while (bayer_offset <= (bayer_end - 2)) {
        uint8_t r1 = 0, g1 = 0, b1 = 0, r2 = 0, g2 = 0, b2 = 0;
        uint8_t t0 = (static_cast<uint32_t>(camera_raw[bayer_offset]) +
                      static_cast<uint32_t>(camera_raw[bayer_offset + 2]) +
                      static_cast<uint32_t>(
                          camera_raw[bayer_offset + (bayer_stride * 2)]) +
                      static_cast<uint32_t>(
                          camera_raw[bayer_offset + (bayer_stride * 2 + 2)]) +
                      2) >>
                     2;
        g1 = (static_cast<uint32_t>(camera_raw[bayer_offset + 1]) +
              static_cast<uint32_t>(camera_raw[bayer_offset + bayer_stride]) +
              static_cast<uint32_t>(
                  camera_raw[bayer_offset + (bayer_stride + 2)]) +
              static_cast<uint32_t>(
                  camera_raw[bayer_offset + (bayer_stride * 2 + 1)]) +
              2) >>
             2;
        uint8_t t1 = (static_cast<uint32_t>(camera_raw[bayer_offset + 2]) +
                      static_cast<uint32_t>(
                          camera_raw[bayer_offset + (bayer_stride * 2 + 2)]) +
                      1) >>
                     1;
        uint8_t t2 = (static_cast<uint32_t>(
                          camera_raw[bayer_offset + (bayer_stride + 1)]) +
                      static_cast<uint32_t>(
                          camera_raw[bayer_offset + (bayer_stride + 3)]) +
                      1) >>
                     1;
        uint8_t t3 = camera_raw[bayer_offset + (bayer_stride + 1)];
        g2 = camera_raw[bayer_offset + (bayer_stride + 2)];
        if (odd_row) {
          r1 = t0;
          b1 = t3;

          r2 = t1;
          b2 = t2;
        } else {
          b1 = t0;
          r1 = t3;

          b2 = t1;
          r2 = t2;
        }
        callback(x, y, r1, g1, b1);
        callback(x + 1, y, r2, g2, b2);
        bayer_offset += 2;
        x += 2;
      }

      while (bayer_offset < bayer_end) {
        uint8_t t0 = (static_cast<uint32_t>(camera_raw[bayer_offset]) +
                      static_cast<uint32_t>(camera_raw[bayer_offset + 2]) +
                      static_cast<uint32_t>(
                          camera_raw[bayer_offset + (bayer_stride * 2)]) +
                      static_cast<uint32_t>(
                          camera_raw[bayer_offset + (bayer_stride * 2 + 2)]) +
                      2) >>
                     2;
        uint8_t g =
            (static_cast<uint32_t>(camera_raw[bayer_offset + 1]) +
             static_cast<uint32_t>(camera_raw[bayer_offset + bayer_stride]) +
             static_cast<uint32_t>(
                 camera_raw[bayer_offset + (bayer_stride + 2)]) +
             static_cast<uint32_t>(
                 camera_raw[bayer_offset + (bayer_stride * 2 + 1)]) +
             2) >>
            2;
        uint8_t t1 = camera_raw[bayer_offset + bayer_stride + 1];
        if (odd_row) {
          callback(x, y, t0, g, t1);
        } else {
          callback(x, y, t1, g, t0);
        }
        bayer_offset += 1;
        ++x;
      }

      bayer_offset += 2;
    }
  }
}

void RotateXY(CameraRotation rotation, int in_x, int in_y, int* out_x,
              int* out_y) {
  CHECK(out_x);
  CHECK(out_y);

  // Short-circuit for no rotation
  if (rotation == CameraRotation::k0) {
    *out_x = in_x;
    *out_y = in_y;
    return;
  }

  // Shift our coordinates so that the center of the image is 0,0
  in_x = in_x - (CameraTask::kWidth / 2);
  in_y = in_y - (CameraTask::kHeight / 2);

  // Simple rotation around origin
  switch (rotation) {
    case CameraRotation::k90:
      *out_x = -in_y;
      *out_y = in_x;
      break;
    case CameraRotation::k180:
      *out_x = -in_x;
      *out_y = -in_y;
      break;
    case CameraRotation::k270:
      *out_x = in_y;
      *out_y = -in_x;
      break;
    case CameraRotation::k0:
    default:
      CHECK(false);
  }

  // Undo coordinate space shift
  *out_x = *out_x + (CameraTask::kWidth / 2);
  *out_y = *out_y + (CameraTask::kHeight / 2);
  CHECK(*out_x >= 0);
  CHECK(*out_x < static_cast<int>(CameraTask::kWidth));
  CHECK(*out_y >= 0);
  CHECK(*out_y < static_cast<int>(CameraTask::kHeight));
}

void BayerToRgb(const uint8_t* camera_raw, uint8_t* camera_rgb, int width,
                int height, CameraFilterMethod filter,
                CameraRotation rotation) {
  std::memset(camera_rgb, 0, width * height * 3);
  BayerInternal(camera_raw, width, height, filter,
                [camera_rgb, width, height, rotation](int x, int y, uint8_t r,
                                                      uint8_t g, uint8_t b) {
                  int rot_x, rot_y;
                  RotateXY(rotation, x, y, &rot_x, &rot_y);
                  camera_rgb[(rot_x * 3) + (rot_y * width * 3) + 0] = r;
                  camera_rgb[(rot_x * 3) + (rot_y * width * 3) + 1] = g;
                  camera_rgb[(rot_x * 3) + (rot_y * width * 3) + 2] = b;
                });
}

void BayerToGrayscale(const uint8_t* camera_raw, uint8_t* camera_grayscale,
                      int width, int height, CameraFilterMethod filter,
                      CameraRotation rotation) {
  BayerInternal(camera_raw, width, height, filter,
                [camera_grayscale, width, height, rotation](
                    int x, int y, uint8_t r, uint8_t g, uint8_t b) {
                  int rot_x, rot_y;
                  RotateXY(rotation, x, y, &rot_x, &rot_y);
                  float r_f = static_cast<float>(r) / kUint8Max;
                  float g_f = static_cast<float>(g) / kUint8Max;
                  float b_f = static_cast<float>(b) / kUint8Max;
                  camera_grayscale[rot_x + (rot_y * width)] =
                      static_cast<uint8_t>(((kRedCoefficient * r_f * r_f) +
                                            (kGreenCoefficient * g_f * g_f) +
                                            (kBlueCoefficient * b_f * b_f)) *
                                           kUint8Max);
                });
}

void RgbToGrayscale(const uint8_t* camera_rgb, uint8_t* camera_grayscale,
                    int width, int height) {
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      float r_f =
          static_cast<float>(camera_rgb[(x * 3) + (y * width * 3) + 0]) /
          kUint8Max;
      float g_f =
          static_cast<float>(camera_rgb[(x * 3) + (y * width * 3) + 1]) /
          kUint8Max;
      float b_f =
          static_cast<float>(camera_rgb[(x * 3) + (y * width * 3) + 2]) /
          kUint8Max;
      camera_grayscale[x + (y * width)] = static_cast<uint8_t>(
          ((kRedCoefficient * r_f * r_f) + (kGreenCoefficient * g_f * g_f) +
           (kBlueCoefficient * b_f * b_f)) *
          kUint8Max);
    }
  }
}

bool CameraTask::GetFrame(const std::vector<CameraFrameFormat>& fmts) {
  if (!enabled_) {
    printf("Camera is not enabled, cannot capture frame.\r\n");
    return false;
  }

  // if (mode_ == CameraMode::kTrigger && !GpioGet(Gpio::kCameraTrigger)) {
  //   printf("Camera is in trigger mode but was never triggered\r\n");
  //   return false;
  // }

  bool ret = true;
  static uint8_t* raw = nullptr;
  int index = 0;

  // if (raw == nullptr)

  // GpioSet(Gpio::kCameraTrigger, false);
  index = GetFrame(&raw, true);

  if (!raw) {
    printf("No frame!!!\r\n");
    return false;
  }

  // if (mode_ == CameraMode::kTrigger) {
  //   GpioSet(Gpio::kCameraTrigger, false);
  // }

  for (const CameraFrameFormat& fmt : fmts) {
    DBG_OUTPUT("F%d:%dx%d\n", index, kWidth, kHeight);
    // std::memcpy(fmt.buffer, raw, kWidth * kHeight * 4);
    Rgb8888ToRgb(raw, fmt.buffer, fmt.width, fmt.height);

    ret = true;
    break;
  }

  GetSingleton()->ReturnFrame(index);
  return ret;
}

bool CameraTask::Read(uint16_t reg, uint8_t* val) {
  lpi2c_master_transfer_t transfer;
  transfer.flags = kLPI2C_TransferDefaultFlag;
  transfer.slaveAddress = kCameraAddress;
  transfer.direction = kLPI2C_Read;
  transfer.subaddress = static_cast<uint16_t>(reg);
  transfer.subaddressSize = sizeof(reg);
  transfer.data = val;
  transfer.dataSize = sizeof(*val);

  status_t status2 = LPI2C_RTOS_Transfer(i2c_handle2_, &transfer);
  status_t status1 = LPI2C_RTOS_Transfer(i2c_handle_, &transfer);

  DBG_OUTPUT("Rx|0x%04X=0x%02X|(s1: %ld)(s2:%ld)\n", reg, val[0], status1, status2);

  return status1 == kStatus_Success;
}

bool CameraTask::Write(uint16_t reg, uint8_t val) {
  return Write(reg, &val, sizeof(val));
}

bool CameraTask::Write(uint16_t reg, const uint8_t *val, int size) {
  lpi2c_master_transfer_t transfer;
  transfer.flags = kLPI2C_TransferDefaultFlag;
  transfer.slaveAddress = kCameraAddress;
  transfer.direction = kLPI2C_Write;
  transfer.subaddress = static_cast<uint16_t>(reg);
  transfer.subaddressSize = sizeof(reg);
  transfer.data = (void *)val;
  transfer.dataSize = size;

  status_t status1 = LPI2C_RTOS_Transfer(i2c_handle_, &transfer);
  status_t status2 = LPI2C_RTOS_Transfer(i2c_handle2_, &transfer);

  DBG_OUTPUT("Tx|0x%04X=0x%02X|(s1: %ld)(s2:%ld)\n", reg, val[0], status1, status2);

  return status1 == kStatus_Success;
}

void CameraTask::Init(lpi2c_rtos_handle_t* i2c_handle, lpi2c_rtos_handle_t* i2c_handle2) {
  QueueTask::Init();
  i2c_handle_ = i2c_handle;
  i2c_handle2_ = i2c_handle2;
  enabled_ = false;
  GetMotionDetectionConfigDefault(md_config_);
  md_config_.enable = false;

  // Init GPIO used by camera
  // GpioSetMode(Gpio::kCamReset, GpioMode::kOutput);
  GpioSetMode(Gpio::kCamReset2, GpioMode::kOutput);
  // GpioSetMode(Gpio::kCamPwrDn, GpioMode::kOutput);
  // GpioSetMode(Gpio::kCamPwrDn2, GpioMode::kOutput);
  // GpioSetMode(Gpio::kCamMux, GpioMode::kOutput);

  printf ("%s: i2c_Handle: 0x%x, i2c_handle2: 0x%x", __func__, i2c_handle, i2c_handle2);
}

void CameraTask::SwitchCamera(SwitchCameraId cameraId) {
  camera::Request req;
  req.type = camera::RequestType::kSwitchCamera;
  req.request.switchCameraId = cameraId;
  SendRequest(req);
}

int CameraTask::GetFrame(uint8_t** buffer, bool block) {
  camera::Request req;
  req.type = camera::RequestType::kFrame;
  req.request.frame.index = -1;
  camera::Response resp;

  do {
    resp = SendRequest(req);
  } while (block && resp.response.frame.index == -1);
  *buffer = IndexToFramebufferPtr(resp.response.frame.index);
  return resp.response.frame.index;
}

void CameraTask::ReturnFrame(int index) {
  camera::Request req;
  req.type = camera::RequestType::kFrame;
  req.request.frame.index = index;
  SendRequest(req);
}

bool CameraTask::Enable(CameraMode mode) {
  camera::Request req;
  req.type = camera::RequestType::kEnable;
  req.request.mode = mode;
  auto resp = SendRequest(req);
  enabled_ = resp.response.enable.success;
  return enabled_;
}

void CameraTask::Disable() {
  camera::Request req;
  req.type = camera::RequestType::kDisable;
  SendRequest(req);
}

bool CameraTask::SetPower(bool enable) {
  camera::Request req;
  req.type = camera::RequestType::kPower;
  req.request.power.enable = enable;
  camera::Response resp = SendRequest(req);
  return resp.response.power.success;
}

void CameraTask::ChangePattern(void)
{
  CameraTestPattern val = CameraTestPattern::kNone;

  if (test_pattern_ == CameraTestPattern::kNone)
    val = CameraTestPattern::kColorBar;
  else if (test_pattern_ == CameraTestPattern::kColorBar)
    val = CameraTestPattern::kWalkingOnes;
  else
    val = CameraTestPattern::kNone;

  SetTestPattern(val);
}

void CameraTask::SetTestPattern(CameraTestPattern pattern) {
  camera::Request req;
  req.type = camera::RequestType::kTestPattern;
  req.request.test_pattern.pattern = pattern;
  SendRequest(req);
}

void CameraTask::Trigger() {}

void CameraTask::DiscardFrames(int count) {
  camera::Request req;
  req.type = camera::RequestType::kDiscard;
  req.request.discard.count = count;
  SendRequest(req);
}

int CameraTask::DiscardOldFrames() {
  int discarded = 0;
  const int max_attempts = kFramebufferCount * 2;
  for (int i = 0; i < kFramebufferCount - 1 && discarded < max_attempts; ++i) {
    uint8_t* tmp = nullptr;
    int idx = GetFrame(&tmp, false);
    if (idx >= 0) {
      ReturnFrame(idx);
      ++discarded;
    } else {
      break;
    }
  }
  // Optionally: printf("Discarded %d old frames\n", discarded);
  return discarded;
}

void CameraTask::TaskInit() {
  printf("Camera %dx%d@%d %d bits per pixel\n",
    DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT, DEMO_CAMERA_FRAME_RATE, DEMO_CAMERA_BUFFER_BPP * 8);

  camera::PowerRequest req;
  req.enable = false;
  HandlePowerRequest(req);
}

void CameraTask::SetMotionDetectionRegisters() {
  if (md_config_.enable) {
  } else {
  }
}

bool CameraTask::VideoConvert(uint32_t in)
{
      pxp_ps_buffer_config_t psBufferConfig = {
#if (!(defined(FSL_FEATURE_PXP_HAS_NO_EXTEND_PIXEL_FORMAT) && FSL_FEATURE_PXP_HAS_NO_EXTEND_PIXEL_FORMAT)) || \
    (!(defined(FSL_FEATURE_PXP_V3) && FSL_FEATURE_PXP_V3))
        .pixelFormat = kPXP_PsPixelFormatRGB888, //kPXP_PsPixelFormatARGB8888,
#else
        .pixelFormat = kPXP_PsPixelFormatRGB888, /* Note: This is 32-bit per pixel */
#endif
        .swapByte    = false,
        .bufferAddrU = 0U,
        .bufferAddrV = 0U,
        .pitchBytes  = DEMO_CAMERA_WIDTH * DEMO_CAMERA_BUFFER_BPP,
    };

    /* Output config. */
    pxp_output_buffer_config_t outputBufferConfig = {
        .pixelFormat    = kPXP_OutputPixelFormatRGB888P,
        .interlacedMode = kPXP_OutputProgressive,
        .buffer1Addr    = 0U,
        .pitchBytes     = DEMO_BUFFER_WIDTH * 3,
#if DEMO_ROTATE_FRAME
        .width  = DEMO_BUFFER_HEIGHT,
        .height = DEMO_BUFFER_WIDTH,
#else
        .width       = DEMO_BUFFER_WIDTH,
        .height      = DEMO_BUFFER_HEIGHT,
#endif
    };

  /* Convert the camera input picture to RGB format. */
  psBufferConfig.bufferAddr = in;
  PXP_SetProcessSurfaceBufferConfig(DEMO_PXP, &psBufferConfig);

  outputBufferConfig.buffer0Addr = (uint32_t)pxp_buffer;
  PXP_SetOutputBufferConfig(DEMO_PXP, &outputBufferConfig);

  printf("pxp starting ...\r\n");

  PXP_Start(DEMO_PXP);

  printf("pxp waiting to complete ...\r\n");

  /* Wait for PXP process complete. */
  while (!(kPXP_CompleteFlag & PXP_GetStatusFlags(DEMO_PXP)));

  printf("pxp done\r\n");

  PXP_ClearStatusFlags(DEMO_PXP, kPXP_CompleteFlag);
}

void CameraTask::HandleSwitchCameraRequest(const SwitchCameraId cameraId) {
  bool discard = false;

  switch(cameraId) {
    case coralmicro::SwitchCameraId::kCameraBack:
        coralmicro::GpioSet((coralmicro::Gpio) Gpio::kCamMux, MUX_BACK_CAMERA);
        discard = true;
        printf("BACK camera selected\n");
        break;

    case coralmicro::SwitchCameraId::kCameraFront:
        coralmicro::GpioSet((coralmicro::Gpio) Gpio::kCamMux, MUX_FRONT_CAMERA);
        discard = true;
        printf("FRONT camera selected\n");
        break;

    default:
        printf("Invalid switchCameraId: %d,", cameraId);
        break;
  }

  if (discard) {
      uint32_t buffer;

      // Discard the old frames acquired
      for (int n=0; n<DEMO_CAMERA_BUFFER_COUNT; n++)
      {
        status_t status = CAMERA_RECEIVER_GetFullBuffer(&cameraReceiver, &buffer);

        if (status == kStatus_Success)
        {
          CAMERA_RECEIVER_SubmitEmptyBuffer(&cameraReceiver, (uint32_t)buffer);
        }
        else {
          break;
        }
      }
    }
}


camera::EnableResponse CameraTask::HandleEnableRequest(const CameraMode& mode) {
  camera::EnableResponse resp;
  status_t status;
  camera_config_t cameraConfig;

  // vTaskDelay(pdMS_TO_TICKS(1000));

  BOARD_InitPxp();
  BOARD_InitCamera();

  for(int n=0; n<10; n++)
  {
    uint8_t val;
    Read(0x3008, &val);
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  BOARD_PxpConfig();
  if (kCameraUseUserLed) {
    coralmicro::GpioSet((coralmicro::Gpio) coralmicro::Gpio::kUserLed, 1);
  }

  status = CAMERA_RECEIVER_Start(&cameraReceiver);
  printf("CAMERA_RECEIVER_Start = %ld\n", status);

  resp.success = (status == kStatus_Success);

  return resp;
}

bool CameraTask::Detect(void)
{
  uint8_t model_id_h = 0xff, model_id_l = 0xff;

  for (int i = 0; i < 10; ++i) {
      Read(0x300A, &model_id_h);
      Read(0x300B, &model_id_l);
      if (model_id_h == kModelIdHExpected && model_id_l == kModelIdLExpected) {
        return true;
      }
  }

  if (model_id_h != kModelIdHExpected || model_id_l != kModelIdLExpected) {
    printf("Camera model id not as expected!!!!!: 0x%02x%02x\r\n", model_id_h,
            model_id_l);
  }

  return false;
}

void CameraTask::HandleDisableRequest() {
  enabled_ = false;

  status_t status = CAMERA_RECEIVER_Stop(&cameraReceiver);
  printf("CAMERA_RECEIVER_Stop = %ld\n", status);
}

camera::PowerResponse CameraTask::HandlePowerRequest(
    const camera::PowerRequest& power) {
  camera::PowerResponse resp;
  resp.success = true;

  PmicTask::GetSingleton()->SetRailState(PmicRail::kCam2V8, power.enable);
  PmicTask::GetSingleton()->SetRailState(PmicRail::kCam1V8, power.enable);
  vTaskDelay(pdMS_TO_TICKS(10));


  if (power.enable) {
    coralmicro::GpioSet((coralmicro::Gpio) Gpio::kCamMux, 0);
    coralmicro::GpioSet((coralmicro::Gpio) Gpio::kCamPwrDn, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    coralmicro::GpioSet((coralmicro::Gpio) Gpio::kCamReset, 1);
    vTaskDelay(pdMS_TO_TICKS(40));
    coralmicro::GpioSet((coralmicro::Gpio) Gpio::kCamPwrDn2, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    coralmicro::GpioSet((coralmicro::Gpio) Gpio::kCamReset2, 1);
    vTaskDelay(pdMS_TO_TICKS(40));
    // Set MUX on front camera by default
    coralmicro::GpioSet((coralmicro::Gpio) Gpio::kCamMux, MUX_FRONT_CAMERA);
    // Init Cam on I2C1
    resp.success = CameraTask::Detect();
    printf ("%s: try I2C1: %s\n", __func__,
      resp.success ? "Success":"Failed");
    if (i2c_handle2_) {
      lpi2c_rtos_handle_t *old = i2c_handle_;
      i2c_handle_ = i2c_handle2_;
      // Init Cam on I2C2
      resp.success = CameraTask::Detect();
      printf ("%s: try I2C2: %s\n", __func__,
        resp.success ? "Success":"Failed");
      i2c_handle_ = old;
    }
  } else {
    // Power down: assert power-down and reset pins with same delays as enable
    coralmicro::GpioSet((coralmicro::Gpio)Gpio::kCamPwrDn, 1);
    vTaskDelay(pdMS_TO_TICKS(2));
    coralmicro::GpioSet((coralmicro::Gpio)Gpio::kCamReset, 0);
    vTaskDelay(pdMS_TO_TICKS(40));
    coralmicro::GpioSet((coralmicro::Gpio)Gpio::kCamPwrDn2, 1);
    vTaskDelay(pdMS_TO_TICKS(2));
    coralmicro::GpioSet((coralmicro::Gpio)Gpio::kCamReset2, 0);
    vTaskDelay(pdMS_TO_TICKS(40));
  }

  return resp;
}

camera::FrameResponse CameraTask::HandleFrameRequest(
    const camera::FrameRequest& frame) {
  camera::FrameResponse resp = {
    .index = -1
  };
  status_t status;
  uint32_t buffer;
  if (kCameraUseStatusLed) {
    coralmicro::GpioSet((coralmicro::Gpio) coralmicro::Gpio::kStatusLed, 0);
  }

  if (frame.index == -1) {  // GET
    // get new frame buffer
    int n = 40;
    bool state = true;

    DBG_OUTPUT ("CAMERA_RECEIVER_GetFullBuffer:waiting...\n");

    while(n--)
    {
      status = CAMERA_RECEIVER_GetFullBuffer(&cameraReceiver, &buffer);
      if (status == kStatus_Success)
      {
        break;
      }

      vTaskDelay(100);

      if (kCameraUseStatusLed) {
        coralmicro::GpioSet((coralmicro::Gpio) coralmicro::Gpio::kStatusLed, state);
        state = !state;
      }
    }

    DBG_OUTPUT("CAMERA_RECEIVER_GetFullBuffer = %ld\n", status);

    if (status == kStatus_Success) {
      // DBG_OUTPUT ("CAMERA_RECEIVER_GetFullBuffer:status = OK, invalidate %d bytes\n", sizeof(framebuffers[0]));
      // DCACHE_InvalidateByRange(buffer, sizeof(framebuffers[0]));

      if (kCameraUseStatusLed) {
        coralmicro::GpioSet((coralmicro::Gpio) coralmicro::Gpio::kStatusLed, 0);
      }

      resp.index = FramebufferPtrToIndex(reinterpret_cast<uint8_t*>(buffer));
    }
    else {
      //printf ("CAMERA_RECEIVER_GetFullBuffer:status = %ld\n", status);
      if (kCameraUseStatusLed) {
        coralmicro::GpioSet((coralmicro::Gpio) coralmicro::Gpio::kStatusLed, 1);
      }
    }
  } else {  // RETURN
    buffer = reinterpret_cast<uint32_t>(IndexToFramebufferPtr(frame.index));

    if (buffer) {
      status = CAMERA_RECEIVER_SubmitEmptyBuffer(&cameraReceiver, (uint32_t)buffer);
      DBG_OUTPUT ("CAMERA_RECEIVER_SubmitEmptyBuffer:status = %ld\n", status);
    }
  }

  if (kCameraUseStatusLed) {
    coralmicro::GpioSet((coralmicro::Gpio) coralmicro::Gpio::kStatusLed, 0);
  }

  // CamDumpRegistersOnly();

  uint32_t reg1 = 0x40810108;
  uint32_t reg2 = 0x4081010c;
  if (*(uint32_t*)reg1)
    printf ("%08lX=%08lX\n",reg1, *(uint32_t*)reg1);

  if (*(uint32_t*)reg2)
    printf ("%08lX=%08lX\n",reg2, *(uint32_t*)reg2);

  return resp;
}

void CameraTask::HandleTestPatternRequest(
  const camera::TestPatternRequest& test_pattern) {
  Write(0x503D, (uint8_t)test_pattern.pattern);
  test_pattern_ = test_pattern.pattern;
}

void CameraTask::HandleDiscardRequest(const camera::DiscardRequest& discard) {
  int discarded = 0;
  while (discarded < discard.count) {
    camera::FrameRequest request;
    request.index = -1;
    camera::FrameResponse resp = HandleFrameRequest(request);
    if (resp.index != -1) {
      // Return the frame, and increment the discard counter.
      discarded++;
      request.index = resp.index;
      HandleFrameRequest(request);
    }
  }
}

void CameraTask::GetMotionDetectionConfigDefault(
    CameraMotionDetectionConfig& config) {
  config.cb = nullptr;
  config.cb_param = nullptr;
  config.enable = true;
  config.x0 = 0;
  config.y0 = 0;
  config.x1 = kWidth - 1;
  config.y1 = kHeight - 1;
}

void CameraTask::SetMotionDetectionConfig(
    const CameraMotionDetectionConfig& config) {
  camera::Request req;
  req.type = camera::RequestType::kMotionDetectionConfig;
  req.request.motion_detection_config = config;
  SendRequest(req);
}

void CameraTask::HandleMotionDetectionConfig(
    const CameraMotionDetectionConfig& config) {
  md_config_ = config;
  SetMotionDetectionRegisters();
}

void CameraTask::HandleMotionDetectionInterrupt() {
  if (md_config_.cb) {
    // md_config_.cb(md_config_.cb_param);
  }
}

void CameraTask::SetMode(const CameraMode& mode) {
  mode_ = mode;
}

void CameraTask::RequestHandler(camera::Request* req) {
  camera::Response resp;
  resp.type = req->type;
  switch (req->type) {
    case camera::RequestType::kEnable:
      resp.response.enable = HandleEnableRequest(req->request.mode);
      break;
    case camera::RequestType::kDisable:
      HandleDisableRequest();
      break;
    case camera::RequestType::kPower:
      resp.response.power = HandlePowerRequest(req->request.power);
      break;
    case camera::RequestType::kFrame:
      resp.response.frame = HandleFrameRequest(req->request.frame);
      break;
    case camera::RequestType::kTestPattern:
      HandleTestPatternRequest(req->request.test_pattern);
      break;
    case camera::RequestType::kDiscard:
      HandleDiscardRequest(req->request.discard);
      break;
    case camera::RequestType::kMotionDetectionInterrupt:
      HandleMotionDetectionInterrupt();
      break;
    case camera::RequestType::kMotionDetectionConfig:
      HandleMotionDetectionConfig(req->request.motion_detection_config);
      break;
    case camera::RequestType::kSwitchCamera:
      HandleSwitchCameraRequest(req->request.switchCameraId);
      break;
  }
  if (req->callback) req->callback(resp);
}

}  // namespace coralmicro
