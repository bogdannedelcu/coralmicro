/*
 * Copyright 2024 SentAI Project
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
 *
 * AIfES integration for SentAI runtime.
 * AIfES is licensed under AGPL-3.0 (see third_party/aifes/LICENSE).
 */

#ifndef LIBS_AIFES_AIFES_TASK_H_
#define LIBS_AIFES_AIFES_TASK_H_

#include <cstdint>
#include <cstddef>
#include <vector>
#include <string>

namespace coralmicro {

// Maximum supported layers in a model
constexpr int kAifesMaxLayers = 32;
// Maximum supported network structure (layer count)
constexpr int kAifesMaxStructure = 16;

// Training result structure
struct AifesTrainResult {
    float final_loss;
    float best_loss;
    float best_val_loss;
    int best_epoch;
    int total_epochs;
    float total_time_s;
    bool success;
    std::string log_path;
};

// Model info structure
struct AifesModelInfo {
    std::string name;
    int layer_count;
    int input_size;
    int output_size;
    int total_params;
    std::string dtype;
    bool loaded;
    bool trained;
};

// Training configuration (can be overridden from Python)
struct AifesTrainConfig {
    int epochs = 100;
    int batch_size = 4;
    float learning_rate = 0.01f;
    int optimizer = 0;  // 0=adam, 1=sgd
    int loss = 0;       // 0=mse, 1=crossentropy
    float sgd_momentum = 0.0f;
    float val_split = 0.1f;  // 10% validation split
    bool early_stopping = false;
    float early_stopping_target = 0.001f;
    int print_interval = 10;
    std::string log_path;
    
    // Weak-SIGReg: covariance regularization (arXiv:2603.05924)
    // Decorrelates hidden layer activations for stable training
    float sigreg_lambda = 0.0f;        // 0 = disabled, typical: 0.001-0.1
    float sigreg_lr = 0.001f;          // Learning rate for SIGReg gradient step
    int sigreg_interval = 1;           // Apply SIGReg every N epochs
    std::vector<int> sigreg_layers;    // Empty = all hidden layers, or specific indices
};

// C interface for MicroPython bindings
extern "C" {

// Load model from YAML file
// Returns: 0 on success, negative on error
int aifes_load_model(const char* yaml_path);

// Load pre-trained weights from binary file
// Returns: 0 on success, negative on error
int aifes_load_weights(const char* weights_path);

// Save trained weights to binary file
// Returns: 0 on success, negative on error
int aifes_save_weights(const char* weights_path);

// Train the model
// x_data: input data (flat array, row-major)
// y_data: target data (flat array, row-major)
// n_samples: number of samples
// input_size: size of each input
// output_size: size of each output
// sigreg_lambda: Weak-SIGReg covariance regularization (0 = disabled)
//   Layer selection is configured in YAML per-layer: [dense, 64, relu, sigreg]
// Returns: final training loss, or negative on error
float aifes_train(const float* x_data, const float* y_data,
                  int n_samples, int input_size, int output_size,
                  int epochs, int batch_size, float learning_rate,
                  int optimizer, int loss_fn, float val_split,
                  const char* log_path, float sigreg_lambda);

// Run inference on a single input
// input: input data (size = model input size)
// output: output buffer (size = model output size)
// Returns: 0 on success, negative on error
int aifes_predict(const float* input, float* output);

// Get model info
// Returns: 1 if model is loaded, 0 otherwise
int aifes_is_loaded(void);

// Get model name
const char* aifes_get_model_name(void);

// Get layer count
int aifes_get_layer_count(void);

// Get input size
int aifes_get_input_size(void);

// Get output size
int aifes_get_output_size(void);

// Get total parameters
int aifes_get_total_params(void);

// Get last train log path
const char* aifes_get_log_path(void);

// Get best epoch from last training
int aifes_get_best_epoch(void);

// Get best validation loss from last training
float aifes_get_best_val_loss(void);

// Unload model and free memory
void aifes_unload(void);

// ============== NEW: TPU-style set_input/invoke/output API ==============

// Set input data for inference (copies data into internal buffer)
// Returns: 0 on success, -1 if size mismatch, -2 if no model loaded
int aifes_set_input(const float* data, int size);

// Set input from TPU output tensor (auto-dequantizes int8/uint8)
// idx: TPU output tensor index
// Returns: 0 on success, negative on error
int aifes_from_tpu(int idx);

// Run inference using previously set input
// Returns: inference time in ms, negative on error
int aifes_invoke(void);

// Get output as array pointer (valid until next invoke)
// Returns: pointer to output buffer, NULL if no inference done
const float* aifes_get_output_data(void);

// Get output size (number of floats)
int aifes_get_output_count(void);

// Capture camera, resize, normalize to floats [0,1] and set as input
// grayscale: 0=RGB (w*h*3 floats), 1=grayscale (w*h floats)
// Returns: 0 on success, negative on error
int aifes_from_camera(int width, int height, int grayscale);

// Capture mic samples, normalize to floats [-1,1] and set as input
// samples: number of int16 samples to capture
// Returns: 0 on success, negative on error
int aifes_from_mic(int samples);

}  // extern "C"

// C++ class interface (for internal use)
class AifesTask {
 public:
    static AifesTask* GetSingleton();

    // Model management
    int LoadModel(const char* yaml_path);
    int LoadWeights(const char* weights_path);
    int SaveWeights(const char* weights_path);
    void Unload();

    // Training
    AifesTrainResult Train(const float* x_data, const float* y_data,
                           int n_samples, int input_size, int output_size,
                           const AifesTrainConfig& config);

    // Inference
    int Predict(const float* input, float* output);
    
    // New invoke-style API
    int SetInput(const float* data, int size);
    int FromTpu(int idx);
    int FromCamera(int width, int height, bool grayscale);
    int FromMic(int samples);
    int Invoke();
    const float* GetOutputData() const { return output_buffer_.data(); }
    int GetOutputCount() const { return static_cast<int>(output_buffer_.size()); }

    // Info getters
    bool IsLoaded() const { return model_loaded_; }
    const AifesModelInfo& GetInfo() const { return info_; }
    const AifesTrainResult& GetLastTrainResult() const { return last_result_; }

 private:
    AifesTask() = default;
    ~AifesTask();

    // Disable copy
    AifesTask(const AifesTask&) = delete;
    AifesTask& operator=(const AifesTask&) = delete;

    // Internal helpers
    int ParseYamlModel(const char* yaml_content, size_t len);
    void BuildModel();
    void FreeModel();
    int WriteTrainLog(const char* path, int epoch, float train_loss, 
                      float val_loss, int duration_ms);
    void ShuffleData(float* x, float* y, int n_samples, 
                     int input_size, int output_size);
    
    // Weak-SIGReg helpers (arXiv:2603.05924)
    // Computes hidden layer activations by layer-by-layer forward pass
    void ComputeHiddenActivations(const float* input, int n_samples,
                                   std::vector<std::vector<float>>& layer_outputs);
    // Computes SIGReg loss: ||Cov(H) - I||_F for given activations
    float ComputeSigregLoss(const std::vector<float>& activations, 
                            int n_samples, int dim);
    // Applies SIGReg gradient to weights (decorrelation step)
    float ApplySigregGradient(const float* x_data, int n_samples,
                               const AifesTrainConfig& config);

    // Model state
    bool model_loaded_ = false;
    bool model_built_ = false;
    AifesModelInfo info_;
    AifesTrainResult last_result_;

    // Network structure (parsed from YAML)
    uint32_t fnn_structure_[kAifesMaxStructure];
    int structure_len_ = 0;
    int activations_[kAifesMaxStructure];  // Activation function per layer
    bool sigreg_enabled_[kAifesMaxStructure];  // SIGReg enabled per hidden layer

    // Training defaults from YAML
    AifesTrainConfig yaml_defaults_;

    // Weights storage (flat array of F32)
    std::vector<float> weights_;
    std::vector<float> best_weights_;  // Best model weights

    // Working buffers
    std::vector<float> train_x_;
    std::vector<float> train_y_;
    std::vector<float> val_x_;
    std::vector<float> val_y_;
    
    // Inference buffers (for invoke-style API)
    std::vector<float> input_buffer_;
    std::vector<float> output_buffer_;
    bool input_set_ = false;
};

}  // namespace coralmicro

#endif  // LIBS_AIFES_AIFES_TASK_H_
