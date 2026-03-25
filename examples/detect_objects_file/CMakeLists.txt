# Copyright 2022 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

add_executable_m7(detect_objects_file
    detect_objects_file.cc
    DATA
    #cat_300x300.rgb
    test_512x512.rgb
    #test_1024x1024.rgb
    ${PROJECT_SOURCE_DIR}/models/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite
    #${PROJECT_SOURCE_DIR}/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite
)

target_link_libraries(detect_objects_file
    libs_base-m7_freertos
)

target_compile_options(detect_objects_file PRIVATE -O2)
