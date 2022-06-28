// Copyright 2022 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef APPS_OOBEE_JPEG_H_
#define APPS_OOBEE_JPEG_H_

namespace coral::micro {

unsigned long JpegCompressRgb(unsigned char* rgb, int width, int height,
                              int quality, unsigned char* buf,
                              unsigned long size);

struct JpegBuffer {
  unsigned char* data;
  unsigned long size;
};
JpegBuffer JpegCompressRgb(unsigned char* rgb, int width, int height,
                           int quality);

};  // namespace coral::micro

#endif  // APPS_OOBEE_JPEG_H_