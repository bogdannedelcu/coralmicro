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

#include "libs/libjpeg/jpeg.h"

#include <algorithm>

#include "third_party/nxp/rt1176-sdk/middleware/libjpeg/inc/jpeglib.h"

namespace coralmicro {
namespace {
constexpr size_t kVectorSizeIncrement = 10 * 1024;

struct vector_destination_mgr {
  struct jpeg_destination_mgr pub;

  std::vector<uint8_t>* out;
};

METHODDEF(void)
init_vector_destination(j_compress_ptr cinfo) { (void)cinfo; }

METHODDEF(boolean)
empty_vector_output_buffer(j_compress_ptr cinfo) {
  auto* dest = reinterpret_cast<vector_destination_mgr*>(cinfo->dest);

  auto size = dest->out->size();
  dest->out->resize(size + kVectorSizeIncrement);

  dest->pub.next_output_byte = dest->out->data() + size;
  dest->pub.free_in_buffer = kVectorSizeIncrement;

  return TRUE;
}

METHODDEF(void)
term_vector_destination(j_compress_ptr cinfo) {
  auto* dest = reinterpret_cast<vector_destination_mgr*>(cinfo->dest);

  dest->out->resize(dest->out->size() - dest->pub.free_in_buffer);
}

void jpeg_vector_dest(j_compress_ptr cinfo, std::vector<uint8_t>* out) {
  if (cinfo->dest == nullptr) {
    cinfo->dest = (struct jpeg_destination_mgr*)(*cinfo->mem->alloc_small)(
        (j_common_ptr)cinfo, JPOOL_PERMANENT, sizeof(vector_destination_mgr));
  }

  out->resize(kVectorSizeIncrement);

  auto* dest = reinterpret_cast<vector_destination_mgr*>(cinfo->dest);
  dest->pub.init_destination = init_vector_destination;
  dest->pub.empty_output_buffer = empty_vector_output_buffer;
  dest->pub.term_destination = term_vector_destination;
  dest->pub.next_output_byte = out->data();
  dest->pub.free_in_buffer = kVectorSizeIncrement;

  dest->out = out;
}

struct buf_destination_mgr {
  struct jpeg_destination_mgr pub;
  unsigned long size;
  unsigned long* out_size;
};

METHODDEF(void)
init_buf_destination(j_compress_ptr cinfo) { (void)cinfo; }

METHODDEF(boolean)
empty_buf_output_buffer(j_compress_ptr cinfo) {
  (void)cinfo;
  return FALSE;
}

METHODDEF(void)
term_buf_destination(j_compress_ptr cinfo) {
  auto* dest = reinterpret_cast<buf_destination_mgr*>(cinfo->dest);
  *dest->out_size = dest->size - dest->pub.free_in_buffer;
}

void jpeg_buf_dest(j_compress_ptr cinfo, unsigned char* buf, unsigned long size,
                   unsigned long* out_size) {
  if (cinfo->dest == nullptr) {
    cinfo->dest = (struct jpeg_destination_mgr*)(*cinfo->mem->alloc_small)(
        (j_common_ptr)cinfo, JPOOL_PERMANENT, sizeof(buf_destination_mgr));
  }

  auto* dest = reinterpret_cast<buf_destination_mgr*>(cinfo->dest);
  dest->pub.init_destination = init_buf_destination;
  dest->pub.empty_output_buffer = empty_buf_output_buffer;
  dest->pub.term_destination = term_buf_destination;
  dest->pub.next_output_byte = buf;
  dest->pub.free_in_buffer = size;

  dest->size = size;
  dest->out_size = out_size;
}


void JpegCompressImpl(struct jpeg_compress_struct* cinfo, unsigned char* rgb,
                      int quality) {
  jpeg_set_defaults(cinfo);
  jpeg_set_quality(cinfo, quality, TRUE);
  // EXP-s256 (2026-10-31): the integer "fast" DCT is ~2x cheaper than the
  // default islow on a Cortex-M7 and visually indistinguishable at q <= 90;
  // the encode was 116 ms / 640x480 frame with islow at -Os.
  cinfo->dct_method = JDCT_IFAST;

  jpeg_start_compress(cinfo, TRUE);

  JSAMPROW row_pointer[1];
  const int row_stride = cinfo->image_width * 3;
  while (cinfo->next_scanline < cinfo->image_height) {
    auto* line = &rgb[cinfo->next_scanline * row_stride];
 //   for (JDIMENSION j = 0; j < cinfo->image_width; ++j)
 //     std::swap(line[3 * j], line[3 * j + 2]);
    row_pointer[0] = line;
    jpeg_write_scanlines(cinfo, row_pointer, 1);
  }
  jpeg_finish_compress(cinfo);
  jpeg_destroy_compress(cinfo);
}
}  // namespace

unsigned long JpegCompressYuv420(const unsigned char* y, const unsigned char* cb,
                                 const unsigned char* cr, int width, int height,
                                 int quality, unsigned char* out, unsigned long out_size) {
  if ((width & 15) || (height & 15) || width <= 0 || height <= 0) return 0;
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);
  unsigned long used = 0;
  jpeg_buf_dest(&cinfo, out, out_size, &used);
  cinfo.image_width = width;
  cinfo.image_height = height;
  cinfo.input_components = 3;
  cinfo.in_color_space = JCS_YCbCr;
  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, quality, TRUE);
  cinfo.dct_method = JDCT_IFAST;
  cinfo.raw_data_in = TRUE;                   // we hand over MCU rows of planes
  // IJG 9 gotcha (EXP-s256, host-reproduced): with do_fancy_downsampling
  // left TRUE the encoder implements the 2x2 chroma subsampling *inside the
  // DCT* (16x16 chroma blocks, DCT_h/v_scaled_size = 16) and expects
  // full-resolution chroma rows.  In raw_data_in mode we already supply
  // 2x2-subsampled Cb/Cr (8 rows per iMCU row), so the 16-row reads ran off
  // the plane rows -> 16-row-period garbage stripes.  FALSE keeps 8x8 DCTs.
  cinfo.do_fancy_downsampling = FALSE;
  cinfo.comp_info[0].h_samp_factor = 2;       // Y 2x2, Cb/Cr 1x1 = 4:2:0
  cinfo.comp_info[0].v_samp_factor = 2;
  cinfo.comp_info[1].h_samp_factor = 1;
  cinfo.comp_info[1].v_samp_factor = 1;
  cinfo.comp_info[2].h_samp_factor = 1;
  cinfo.comp_info[2].v_samp_factor = 1;
  jpeg_start_compress(&cinfo, TRUE);
  JSAMPROW yrows[16], cbrows[8], crrows[8];
  JSAMPARRAY planes[3] = {yrows, cbrows, crrows};
  const int cw = width / 2;
  while (cinfo.next_scanline < cinfo.image_height) {
    const int row = (int)cinfo.next_scanline;
    for (int i = 0; i < 16; ++i) yrows[i] = (JSAMPROW)(y + (size_t)(row + i) * width);
    for (int i = 0; i < 8; ++i) {
      cbrows[i] = (JSAMPROW)(cb + (size_t)(row / 2 + i) * cw);
      crrows[i] = (JSAMPROW)(cr + (size_t)(row / 2 + i) * cw);
    }
    jpeg_write_raw_data(&cinfo, planes, 16);
  }
  jpeg_finish_compress(&cinfo);
  jpeg_destroy_compress(&cinfo);
  return used;
}

unsigned long JpegCompressRgb(unsigned char* rgb, int width, int height,
                              int quality, unsigned char* buf,
                              unsigned long size) {
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);

  unsigned long out_size;
  jpeg_buf_dest(&cinfo, buf, size, &out_size);

  cinfo.image_width = width;
  cinfo.image_height = height;
  cinfo.input_components = 3;
  cinfo.in_color_space = JCS_RGB;

  JpegCompressImpl(&cinfo, rgb, quality);
  return out_size;
}

JpegBuffer JpegCompressRgb(unsigned char* rgb, int width, int height,
                           int quality) {
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);

  JpegBuffer res{};
  jpeg_mem_dest(&cinfo, &res.data, &res.size);

  cinfo.image_width = width;
  cinfo.image_height = height;
  cinfo.input_components = 3;
  cinfo.in_color_space = JCS_RGB;

  JpegCompressImpl(&cinfo, rgb, quality);
  return res;
}

void JpegCompressRgb(unsigned char* rgb, int width, int height, int quality,
                     std::vector<uint8_t>* out) {
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);

  jpeg_vector_dest(&cinfo, out);

  cinfo.image_width = width;
  cinfo.image_height = height;
  cinfo.input_components = 3;
  cinfo.in_color_space = JCS_RGB;

  JpegCompressImpl(&cinfo, rgb, quality);
}

}  // namespace coralmicro
