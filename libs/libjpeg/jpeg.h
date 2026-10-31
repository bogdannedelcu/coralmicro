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

#ifndef LIBS_LIBJPEG_JPEG_H_
#define LIBS_LIBJPEG_JPEG_H_

#include <cstdint>
#include <vector>

namespace coralmicro {

// Converts an RGB image to JPEG format.
//
// @param rgb The image in RGB format.
// @param width The image's width.
// @param height The image's height.
// @param quality The quality of the image after compression (must be within
// [0-100]).
// @param buf The buffer to return the JPEG image data to.
// @param size The size allocated for buf.
// @return The size of the resulting JPEG buffer.
// EXP-s256 (2026-10-31): compress planar YCbCr 4:2:0 (Y w*h, Cb and Cr
// (w/2)*(h/2)) straight into `out` via raw_data_in -- the codec skips its
// RGB->YCbCr conversion and the chroma downsampling (PXP CSC2 already did
// both in hardware).  width and height must be multiples of 16.  Returns
// the JPEG size or 0 on error.
unsigned long JpegCompressYuv420(const unsigned char* y, const unsigned char* cb,
                                 const unsigned char* cr, int width, int height,
                                 int quality, unsigned char* out, unsigned long out_size);

unsigned long JpegCompressRgb(unsigned char* rgb, int width, int height,
                              int quality, unsigned char* buf,
                              unsigned long size);

// Represents a JPEG image buffer.
struct JpegBuffer {
  // The Jpeg image data.
  unsigned char* data;
  // The size of the image.
  unsigned long size;
};

// Converts an RGB image to JPEG format.
//
// @param rgb The image in RGB format.
// @param width The image's width.
// @param height The image's height.
// @param quality The quality of the image after compression (must be within
// [0-100]).
// @return A JPEG image buffer.
// Note: You have to deallocate data from JpegBuffer with `free()`.
JpegBuffer JpegCompressRgb(unsigned char* rgb, int width, int height,
                           int quality);

// Converts an RGB image to JPEG format.
//
// @param rgb The image in RGB format.
// @param width The image's width.
// @param height The image's height.
// @param quality The quality of the image after compression (must be within
// [0-100]).
// @param out the output vector to return the resulting JPEG image to.
void JpegCompressRgb(unsigned char* rgb, int width, int height, int quality,
                     std::vector<uint8_t>* out);

}  // namespace coralmicro

#endif  // LIBS_LIBJPEG_JPEG_H_
