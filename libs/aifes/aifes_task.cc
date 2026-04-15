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

#include "libs/aifes/aifes_task.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>

#include "libs/base/filesystem.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

// AIfES includes
extern "C" {
#include "aifes.h"
#include "basic/express/aifes_express_f32_fnn.h"
}

// Extern declarations for TPU functions (defined in sentai_runtime.cc)
// Required for AIfES from_tpu() integration
extern "C" {
    int sentai_tpu_get_output_size(int idx);
    const void* sentai_tpu_get_output_data(int idx);
    int sentai_tpu_get_output_type(int idx);
    int sentai_tpu_output_quant(int idx, float* scale, int32_t* zero_point);
}

// Extern declarations for camera/mic functions (defined in sentai_runtime.cc)
// Required for AIfES from_camera() and from_mic() integration
extern "C" {
    // Capture camera, resize to w×h, output as RGB or grayscale uint8
    // Returns: bytes written (w*h*3 or w*h), negative on error
    int sentai_aifes_capture_camera(uint8_t* out, int w, int h, int grayscale);
    
    // Capture mic samples as int16 PCM
    // Returns: samples captured, negative on error
    int sentai_aifes_capture_mic(int16_t* out, int samples);
    
    // Check if mic is initialized
    int sentai_mic_is_initialized(void);
}

namespace coralmicro {

// Static singleton instance
static AifesTask* g_aifes_task = nullptr;

// Forward declaration for loss callback
static float g_current_loss = 0.0f;
static void aifes_loss_callback(float loss) {
    g_current_loss = loss;
}

// Helper: trim whitespace from string
static std::string Trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

// Helper: parse activation from string
static AIFES_E_activations ParseActivation(const std::string& act) {
    if (act == "relu") return AIfES_E_relu;
    if (act == "leaky_relu") return AIfES_E_leaky_relu;
    if (act == "elu") return AIfES_E_elu;
    if (act == "sigmoid") return AIfES_E_sigmoid;
    if (act == "softmax") return AIfES_E_softmax;
    if (act == "tanh") return AIfES_E_tanh;
    if (act == "softsign") return AIfES_E_softsign;
    if (act == "linear") return AIfES_E_linear;
    // Default to sigmoid
    return AIfES_E_sigmoid;
}

AifesTask* AifesTask::GetSingleton() {
    if (g_aifes_task == nullptr) {
        g_aifes_task = new AifesTask();
    }
    return g_aifes_task;
}

AifesTask::~AifesTask() {
    Unload();
}

void AifesTask::Unload() {
    FreeModel();
    model_loaded_ = false;
    model_built_ = false;
    info_ = AifesModelInfo{};
    structure_len_ = 0;
    weights_.clear();
    best_weights_.clear();
}

void AifesTask::FreeModel() {
    // Nothing dynamic to free - we use vectors
}

int AifesTask::ParseYamlModel(const char* yaml_content, size_t len) {
    // Simple YAML parser for our format
    // Expected format:
    // model:
    //   name: my_model
    //   dtype: f32
    //
    // backbone:
    //   - [input, 2]
    //   - [dense, 3, sigmoid]
    //
    // head:
    //   - [dense, 1, sigmoid]
    //
    // train:
    //   epochs: 100
    //   batch_size: 4
    //   learning_rate: 0.05
    //   optimizer: adam
    //   loss: mse

    std::string content(yaml_content, len);
    
    // Parse line by line
    enum Section { NONE, MODEL, BACKBONE, HEAD, TRAIN } section = NONE;
    
    // Layer info: neurons, activation, sigreg_enabled
    struct LayerInfo {
        int neurons;
        std::string activation;
        bool sigreg;
    };
    std::vector<LayerInfo> layers;
    int input_size = 0;
    
    size_t pos = 0;
    while (pos < content.size()) {
        size_t eol = content.find('\n', pos);
        if (eol == std::string::npos) eol = content.size();
        
        std::string line = Trim(content.substr(pos, eol - pos));
        pos = eol + 1;
        
        if (line.empty() || line[0] == '#') continue;
        
        // Check section headers
        if (line == "model:") { section = MODEL; continue; }
        if (line == "backbone:") { section = BACKBONE; continue; }
        if (line == "head:") { section = HEAD; continue; }
        if (line == "train:") { section = TRAIN; continue; }
        
        // Parse section content
        if (section == MODEL) {
            // name: xxx
            size_t colon = line.find(':');
            if (colon != std::string::npos) {
                std::string key = Trim(line.substr(0, colon));
                std::string val = Trim(line.substr(colon + 1));
                if (key == "name") info_.name = val;
                else if (key == "dtype") info_.dtype = val;
            }
        }
        else if (section == BACKBONE || section == HEAD) {
            // - [type, size, activation]
            if (line[0] != '-') continue;
            
            // Extract content between [ and ]
            size_t open = line.find('[');
            size_t close = line.find(']');
            if (open == std::string::npos || close == std::string::npos) continue;
            
            std::string inner = line.substr(open + 1, close - open - 1);
            
            // Split by comma
            std::vector<std::string> parts;
            size_t p = 0;
            while (p < inner.size()) {
                size_t comma = inner.find(',', p);
                if (comma == std::string::npos) comma = inner.size();
                parts.push_back(Trim(inner.substr(p, comma - p)));
                p = comma + 1;
            }
            
            if (parts.empty()) continue;
            
            std::string type = parts[0];
            
            if (type == "input" && parts.size() >= 2) {
                input_size = std::atoi(parts[1].c_str());
            }
            else if (type == "dense" && parts.size() >= 2) {
                int neurons = std::atoi(parts[1].c_str());
                std::string act = (parts.size() >= 3) ? parts[2] : "sigmoid";
                // Check for sigreg flag in remaining parts
                bool sigreg = false;
                for (size_t pi = 3; pi < parts.size(); pi++) {
                    if (parts[pi] == "sigreg" || parts[pi] == "SIGREG") {
                        sigreg = true;
                        break;
                    }
                }
                layers.push_back({neurons, act, sigreg});
            }
        }
        else if (section == TRAIN) {
            size_t colon = line.find(':');
            if (colon != std::string::npos) {
                std::string key = Trim(line.substr(0, colon));
                std::string val = Trim(line.substr(colon + 1));
                
                if (key == "epochs") yaml_defaults_.epochs = std::atoi(val.c_str());
                else if (key == "batch_size") yaml_defaults_.batch_size = std::atoi(val.c_str());
                else if (key == "learning_rate") yaml_defaults_.learning_rate = std::atof(val.c_str());
                else if (key == "optimizer") {
                    yaml_defaults_.optimizer = (val == "sgd") ? 1 : 0;
                }
                else if (key == "loss") {
                    yaml_defaults_.loss = (val == "crossentropy") ? 1 : 0;
                }
                else if (key == "val_split") yaml_defaults_.val_split = std::atof(val.c_str());
                else if (key == "early_stopping") yaml_defaults_.early_stopping = (val == "true");
                else if (key == "early_stopping_target") yaml_defaults_.early_stopping_target = std::atof(val.c_str());
                // SIGReg parameters
                else if (key == "sigreg" || key == "sigreg_lambda") {
                    yaml_defaults_.sigreg_lambda = std::atof(val.c_str());
                }
                else if (key == "sigreg_lr") {
                    yaml_defaults_.sigreg_lr = std::atof(val.c_str());
                }
                else if (key == "sigreg_interval") {
                    yaml_defaults_.sigreg_interval = std::atoi(val.c_str());
                }
                else if (key == "sigreg_layers") {
                    // Parse [0, 1, 2] or "all"
                    yaml_defaults_.sigreg_layers.clear();
                    if (val != "all" && val != "ALL") {
                        // Parse array: [0, 1, 2]
                        size_t open = val.find('[');
                        size_t close = val.find(']');
                        if (open != std::string::npos && close != std::string::npos) {
                            std::string inner = val.substr(open + 1, close - open - 1);
                            size_t p = 0;
                            while (p < inner.size()) {
                                size_t comma = inner.find(',', p);
                                if (comma == std::string::npos) comma = inner.size();
                                std::string num = Trim(inner.substr(p, comma - p));
                                if (!num.empty()) {
                                    yaml_defaults_.sigreg_layers.push_back(std::atoi(num.c_str()));
                                }
                                p = comma + 1;
                            }
                        }
                    }
                    // "all" or empty vector means all layers
                }
            }
        }
    }
    
    // Build FNN structure
    if (input_size == 0 || layers.empty()) {
        return -1;  // Invalid model
    }
    
    // Structure: [input, layer1, layer2, ..., output]
    structure_len_ = layers.size() + 1;
    if (structure_len_ > kAifesMaxStructure) {
        return -2;  // Too many layers
    }
    
    fnn_structure_[0] = input_size;
    for (size_t i = 0; i < layers.size(); i++) {
        fnn_structure_[i + 1] = layers[i].neurons;
        activations_[i] = static_cast<int>(ParseActivation(layers[i].activation));
        sigreg_enabled_[i] = layers[i].sigreg;
    }
    
    // Calculate info
    info_.layer_count = structure_len_;
    info_.input_size = input_size;
    info_.output_size = layers.back().neurons;
    
    // Calculate total params (weights + biases)
    info_.total_params = 0;
    for (int i = 0; i < structure_len_ - 1; i++) {
        info_.total_params += fnn_structure_[i] * fnn_structure_[i + 1];  // weights
        info_.total_params += fnn_structure_[i + 1];  // biases
    }
    
    // Allocate weights
    weights_.resize(info_.total_params);
    best_weights_.resize(info_.total_params);
    
    return 0;
}

int AifesTask::LoadModel(const char* yaml_path) {
    // Read YAML file
    std::vector<uint8_t> data;
    if (!LfsReadFile(yaml_path, &data)) {
        return -1;  // File not found
    }
    
    int result = ParseYamlModel(reinterpret_cast<char*>(data.data()), data.size());
    if (result != 0) {
        return result;
    }
    
    info_.loaded = true;
    info_.trained = false;
    model_loaded_ = true;
    
    return 0;
}

int AifesTask::LoadWeights(const char* weights_path) {
    if (!model_loaded_) {
        return -1;  // No model loaded
    }
    
    std::vector<uint8_t> data;
    if (!LfsReadFile(weights_path, &data)) {
        return -2;  // File not found
    }
    
    size_t expected = info_.total_params * sizeof(float);
    if (data.size() != expected) {
        return -3;  // Size mismatch
    }
    
    std::memcpy(weights_.data(), data.data(), expected);
    info_.trained = true;
    
    return 0;
}

int AifesTask::SaveWeights(const char* weights_path) {
    if (!model_loaded_ || !info_.trained) {
        return -1;  // No trained model
    }
    
    // Use best weights if available, otherwise current
    const float* w = best_weights_.empty() ? weights_.data() : best_weights_.data();
    
    if (!LfsWriteFile(weights_path, reinterpret_cast<const uint8_t*>(w), 
                      info_.total_params * sizeof(float))) {
        return -2;  // Write failed
    }
    
    return 0;
}

void AifesTask::ShuffleData(float* x, float* y, int n_samples, 
                            int input_size, int output_size) {
    // Fisher-Yates shuffle
    std::random_device rd;
    std::mt19937 gen(rd());
    
    std::vector<float> temp_x(input_size);
    std::vector<float> temp_y(output_size);
    
    for (int i = n_samples - 1; i > 0; i--) {
        std::uniform_int_distribution<> dist(0, i);
        int j = dist(gen);
        
        // Swap X[i] and X[j]
        std::memcpy(temp_x.data(), &x[i * input_size], input_size * sizeof(float));
        std::memcpy(&x[i * input_size], &x[j * input_size], input_size * sizeof(float));
        std::memcpy(&x[j * input_size], temp_x.data(), input_size * sizeof(float));
        
        // Swap Y[i] and Y[j]
        std::memcpy(temp_y.data(), &y[i * output_size], output_size * sizeof(float));
        std::memcpy(&y[i * output_size], &y[j * output_size], output_size * sizeof(float));
        std::memcpy(&y[j * output_size], temp_y.data(), output_size * sizeof(float));
    }
}

// ============== Weak-SIGReg Implementation (arXiv:2603.05924) ==============
// Covariance regularization for stable training: ||Cov(H) - I||_F
// Forces hidden layer activations to be decorrelated with unit variance.

// Helper: Apply activation function
static inline float ApplyActivation(float x, int act) {
    switch (act) {
        case AIfES_E_relu:
            return (x > 0) ? x : 0.0f;
        case AIfES_E_leaky_relu:
            return (x > 0) ? x : 0.01f * x;
        case AIfES_E_sigmoid:
            return 1.0f / (1.0f + expf(-x));
        case AIfES_E_tanh:
            return tanhf(x);
        case AIfES_E_softsign:
            return x / (1.0f + fabsf(x));
        case AIfES_E_linear:
        default:
            return x;
    }
}

void AifesTask::ComputeHiddenActivations(const float* input, int n_samples,
                                          std::vector<std::vector<float>>& layer_outputs) {
    // Compute activations for each hidden layer by manual forward pass
    // layer_outputs[l] contains activations for layer l, shape [n_samples * layer_size]
    
    if (!model_loaded_ || structure_len_ < 2) return;
    
    int n_hidden = structure_len_ - 2;  // Exclude input and output layers
    layer_outputs.resize(n_hidden);
    
    // Track weight offset in flat weights array
    int weight_offset = 0;
    
    // Current layer input (starts with network input)
    std::vector<float> curr_input(input, input + n_samples * fnn_structure_[0]);
    
    for (int layer = 0; layer < n_hidden; layer++) {
        int in_size = fnn_structure_[layer];
        int out_size = fnn_structure_[layer + 1];
        
        // Allocate output for this layer
        layer_outputs[layer].resize(n_samples * out_size);
        
        // Weights: [in_size x out_size], Biases: [out_size]
        const float* W = &weights_[weight_offset];
        const float* b = &weights_[weight_offset + in_size * out_size];
        int act = activations_[layer];
        
        // Forward pass: out = activation(input @ W + b)
        for (int s = 0; s < n_samples; s++) {
            for (int j = 0; j < out_size; j++) {
                float sum = b[j];
                for (int i = 0; i < in_size; i++) {
                    sum += curr_input[s * in_size + i] * W[i * out_size + j];
                }
                layer_outputs[layer][s * out_size + j] = ApplyActivation(sum, act);
            }
        }
        
        // Move to next layer
        weight_offset += in_size * out_size + out_size;
        curr_input = layer_outputs[layer];
    }
}

float AifesTask::ComputeSigregLoss(const std::vector<float>& activations,
                                    int n_samples, int dim) {
    // Compute Weak-SIGReg loss: ||Cov(H) - I||_F
    // H: [n_samples x dim] activations
    // Cov = (H - mean)^T @ (H - mean) / (N-1)
    // Loss = ||Cov - I||_F (Frobenius norm)
    
    if (n_samples < 2 || dim < 1) return 0.0f;
    
    // 1. Compute mean
    std::vector<float> mean(dim, 0.0f);
    for (int s = 0; s < n_samples; s++) {
        for (int j = 0; j < dim; j++) {
            mean[j] += activations[s * dim + j];
        }
    }
    for (int j = 0; j < dim; j++) {
        mean[j] /= n_samples;
    }
    
    // 2. Compute covariance matrix (only upper triangle needed for Frobenius norm)
    // Cov[i,j] = sum((h_i - mean_i) * (h_j - mean_j)) / (N-1)
    std::vector<float> cov(dim * dim, 0.0f);
    for (int s = 0; s < n_samples; s++) {
        for (int i = 0; i < dim; i++) {
            float hi = activations[s * dim + i] - mean[i];
            for (int j = 0; j < dim; j++) {
                float hj = activations[s * dim + j] - mean[j];
                cov[i * dim + j] += hi * hj;
            }
        }
    }
    
    float norm_factor = 1.0f / (n_samples - 1 + 1e-6f);
    for (int k = 0; k < dim * dim; k++) {
        cov[k] *= norm_factor;
    }
    
    // 3. Compute ||Cov - I||_F^2
    float loss = 0.0f;
    for (int i = 0; i < dim; i++) {
        for (int j = 0; j < dim; j++) {
            float target = (i == j) ? 1.0f : 0.0f;
            float diff = cov[i * dim + j] - target;
            loss += diff * diff;
        }
    }
    
    return sqrtf(loss);  // Frobenius norm
}

float AifesTask::ApplySigregGradient(const float* x_data, int n_samples,
                                      const AifesTrainConfig& config) {
    // Apply Weak-SIGReg gradient descent step
    // For each hidden layer:
    //   1. Compute hidden activations
    //   2. Compute covariance gradient: d||Cov-I||/dW
    //   3. Update weights: W -= lr * lambda * gradient
    //
    // The gradient of SIGReg w.r.t. weights is computed via chain rule:
    //   dL/dW = dL/dCov * dCov/dH * dH/dW
    //
    // For simplicity, we use finite differences on a subset of weights.
    // This is a practical approximation for embedded systems.
    
    if (!model_loaded_ || n_samples < 2) return 0.0f;
    
    // Compute hidden layer activations
    std::vector<std::vector<float>> layer_outputs;
    ComputeHiddenActivations(x_data, n_samples, layer_outputs);
    
    if (layer_outputs.empty()) return 0.0f;
    
    float total_loss = 0.0f;
    int n_layers = static_cast<int>(layer_outputs.size());
    int layers_processed = 0;
    
    // Helper: check if layer should be processed
    // Priority: 1) YAML per-layer sigreg flag, 2) config.sigreg_layers array
    auto shouldProcessLayer = [this, &config, n_layers](int layer_idx) -> bool {
        // Check YAML per-layer flag first
        if (sigreg_enabled_[layer_idx]) {
            return true;
        }
        // Check config override (from Python or train section)
        if (!config.sigreg_layers.empty()) {
            for (int idx : config.sigreg_layers) {
                if (idx == layer_idx) return true;
            }
            return false;  // Not in config list
        }
        // Default: if no YAML flags and no config list, apply to all
        // Check if ANY layer has sigreg flag set in YAML
        bool any_yaml_flag = false;
        for (int i = 0; i < n_layers; i++) {
            if (sigreg_enabled_[i]) {
                any_yaml_flag = true;
                break;
            }
        }
        return !any_yaml_flag;  // Apply to all only if no YAML flags set
    };
    
    // Track weight offset for each layer
    int weight_offset = 0;
    
    for (int l = 0; l < n_layers; l++) {
        int in_sz = fnn_structure_[l];
        int dim = fnn_structure_[l + 1];
        
        // Skip if this layer should not be processed
        if (!shouldProcessLayer(l)) {
            weight_offset += in_sz * dim + dim;  // Skip weights + biases
            continue;
        }
        
        // Compute current SIGReg loss for this layer
        float layer_loss = ComputeSigregLoss(layer_outputs[l], n_samples, dim);
        total_loss += layer_loss;
        layers_processed++;
        
        // Skip gradient step if loss is already low
        if (layer_loss < 0.01f) {
            weight_offset += in_sz * dim + dim;
            continue;
        }
        
        // Compute covariance and its gradient
        // Gradient of ||Cov - I||_F w.r.t. H is: 2 * H @ (Cov - I) / (N-1)
        // We'll use this to compute approximate weight gradient
        
        // Compute mean
        std::vector<float> mean(dim, 0.0f);
        for (int s = 0; s < n_samples; s++) {
            for (int j = 0; j < dim; j++) {
                mean[j] += layer_outputs[l][s * dim + j];
            }
        }
        for (int j = 0; j < dim; j++) mean[j] /= n_samples;
        
        // Compute covariance
        std::vector<float> cov(dim * dim, 0.0f);
        for (int s = 0; s < n_samples; s++) {
            for (int i = 0; i < dim; i++) {
                float hi = layer_outputs[l][s * dim + i] - mean[i];
                for (int j = 0; j < dim; j++) {
                    cov[i * dim + j] += hi * (layer_outputs[l][s * dim + j] - mean[j]);
                }
            }
        }
        float norm_factor = 1.0f / (n_samples - 1 + 1e-6f);
        for (int k = 0; k < dim * dim; k++) cov[k] *= norm_factor;
        
        // Compute grad_H = 2 * H_centered @ (Cov - I) / (N-1)
        // Then use this to update biases (simpler than full weight gradient)
        // For biases: db_j = -lr * lambda * sum_s(grad_H[s,j])
        
        float* W = &weights_[weight_offset];
        float* b = &weights_[weight_offset + in_sz * dim];
        
        // Compute gradient w.r.t. biases (approximation: decorrelate outputs)
        std::vector<float> grad_b(dim, 0.0f);
        for (int j = 0; j < dim; j++) {
            for (int k = 0; k < dim; k++) {
                float cov_diff = cov[j * dim + k] - ((j == k) ? 1.0f : 0.0f);
                grad_b[j] += cov_diff * 2.0f / (dim * layer_loss + 1e-6f);
            }
        }
        
        // Update biases
        float update_scale = config.sigreg_lr * config.sigreg_lambda;
        for (int j = 0; j < dim; j++) {
            b[j] -= update_scale * grad_b[j];
        }
        
        // Also apply small weight decay to off-diagonal correlation
        // This reduces correlation between output neurons
        // W_ij -= lr * lambda * sign(Cov_ij) for i != j
        for (int i = 0; i < in_sz && i < 16; i++) {  // Limit for speed
            for (int j1 = 0; j1 < dim; j1++) {
                for (int j2 = j1 + 1; j2 < dim && j2 < j1 + 8; j2++) {
                    float off_cov = cov[j1 * dim + j2];
                    if (fabsf(off_cov) > 0.1f) {
                        // Reduce weights that contribute to correlation
                        float sign = (off_cov > 0) ? 1.0f : -1.0f;
                        W[i * dim + j1] -= update_scale * sign * 0.01f;
                        W[i * dim + j2] += update_scale * sign * 0.01f;
                    }
                }
            }
        }
        
        weight_offset += in_sz * dim + dim;
    }
    
    return total_loss / (layers_processed + 1e-6f);
}

int AifesTask::WriteTrainLog(const char* path, int epoch, float train_loss, 
                             float val_loss, int duration_ms) {
    // Append to log file in CSV format
    // epoch,train_loss,val_loss,duration_ms
    
    FILE* f = fopen(path, epoch == 0 ? "w" : "a");
    if (!f) return -1;
    
    if (epoch == 0) {
        // Write header
        fprintf(f, "epoch,train_loss,val_loss,duration_ms\n");
    }
    
    fprintf(f, "%d,%.6f,%.6f,%d\n", epoch, train_loss, val_loss, duration_ms);
    fclose(f);
    
    return 0;
}

AifesTrainResult AifesTask::Train(const float* x_data, const float* y_data,
                                   int n_samples, int input_size, int output_size,
                                   const AifesTrainConfig& config) {
    AifesTrainResult result = {};
    result.success = false;
    
    if (!model_loaded_) {
        return result;
    }
    
    // Validate input/output sizes match model
    if (input_size != info_.input_size || output_size != info_.output_size) {
        return result;
    }
    
    TickType_t start_time = xTaskGetTickCount();
    
    // Split data into train/val sets
    int val_samples = static_cast<int>(n_samples * config.val_split);
    int train_samples = n_samples - val_samples;
    
    if (train_samples < 1 || (config.val_split > 0 && val_samples < 1)) {
        return result;
    }
    
    // Copy and shuffle data
    std::vector<float> x_train(train_samples * input_size);
    std::vector<float> y_train(train_samples * output_size);
    std::vector<float> x_val(val_samples * input_size);
    std::vector<float> y_val(val_samples * output_size);
    
    // First, create a shuffled copy
    std::vector<float> x_shuffled(n_samples * input_size);
    std::vector<float> y_shuffled(n_samples * output_size);
    std::memcpy(x_shuffled.data(), x_data, n_samples * input_size * sizeof(float));
    std::memcpy(y_shuffled.data(), y_data, n_samples * output_size * sizeof(float));
    ShuffleData(x_shuffled.data(), y_shuffled.data(), n_samples, input_size, output_size);
    
    // Split into train/val
    std::memcpy(x_train.data(), x_shuffled.data(), train_samples * input_size * sizeof(float));
    std::memcpy(y_train.data(), y_shuffled.data(), train_samples * output_size * sizeof(float));
    if (val_samples > 0) {
        std::memcpy(x_val.data(), &x_shuffled[train_samples * input_size], 
                    val_samples * input_size * sizeof(float));
        std::memcpy(y_val.data(), &y_shuffled[train_samples * output_size], 
                    val_samples * output_size * sizeof(float));
    }
    
    // Setup AIfES model
    AIFES_E_model_parameter_fnn_f32 nn;
    nn.layer_count = structure_len_;
    nn.fnn_structure = fnn_structure_;
    nn.fnn_activations = reinterpret_cast<AIFES_E_activations*>(activations_);
    nn.flat_weights = weights_.data();
    
    // Setup training parameters
    AIFES_E_training_parameter_fnn_f32 train_config;
    train_config.optimizer = (config.optimizer == 0) ? AIfES_E_adam : AIfES_E_sgd;
    train_config.sgd_momentum = config.sgd_momentum;
    train_config.loss = (config.loss == 0) ? AIfES_E_mse : AIfES_E_crossentropy;
    train_config.learn_rate = config.learning_rate;
    train_config.batch_size = config.batch_size;
    train_config.epochs = 1;  // We'll do epochs manually for validation
    train_config.epochs_loss_print_interval = 1;
    train_config.loss_print_function = aifes_loss_callback;
    train_config.early_stopping = AIfES_E_early_stopping_off;
    train_config.early_stopping_target_loss = 0.0f;
    
    // Weight initialization
    AIFES_E_init_weights_parameter_fnn_f32 init_weights;
    init_weights.init_weights_method = AIfES_E_init_glorot_uniform;
    init_weights.min_init_uniform = -2.0f;
    init_weights.max_init_uniform = 2.0f;
    
    // Create tensors
    uint16_t input_shape[] = {static_cast<uint16_t>(train_samples), 
                              static_cast<uint16_t>(input_size)};
    uint16_t target_shape[] = {static_cast<uint16_t>(train_samples), 
                               static_cast<uint16_t>(output_size)};
    uint16_t output_shape[] = {static_cast<uint16_t>(train_samples), 
                               static_cast<uint16_t>(output_size)};
    
    aitensor_t input_tensor = AITENSOR_2D_F32(input_shape, x_train.data());
    aitensor_t target_tensor = AITENSOR_2D_F32(target_shape, y_train.data());
    
    std::vector<float> output_data(train_samples * output_size);
    aitensor_t output_tensor = AITENSOR_2D_F32(output_shape, output_data.data());
    
    // Initialize weights on first epoch
    bool first_epoch = true;
    result.best_loss = 1e10f;
    result.best_val_loss = 1e10f;
    result.best_epoch = 0;
    
    // Training loop
    for (int epoch = 0; epoch < config.epochs; epoch++) {
        TickType_t epoch_start = xTaskGetTickCount();
        
        int8_t error;
        if (first_epoch) {
            error = AIFES_E_training_fnn_f32(&input_tensor, &target_tensor,
                                              &nn, &train_config, &init_weights,
                                              &output_tensor);
            first_epoch = false;
        } else {
            // Continue training without re-init
            init_weights.init_weights_method = AIfES_E_init_no_init;
            error = AIFES_E_training_fnn_f32(&input_tensor, &target_tensor,
                                              &nn, &train_config, &init_weights,
                                              &output_tensor);
        }
        
        if (error != 0) {
            printf("AIfES training error: %d\n", error);
            return result;
        }
        
        float train_loss = g_current_loss;
        
        // Apply Weak-SIGReg regularization (arXiv:2603.05924)
        float sigreg_loss = 0.0f;
        if (config.sigreg_lambda > 0.0f && 
            (epoch + 1) % config.sigreg_interval == 0) {
            sigreg_loss = ApplySigregGradient(x_train.data(), train_samples, config);
        }
        
        // Calculate validation loss
        float val_loss = 0.0f;
        if (val_samples > 0) {
            // Run inference on validation set
            uint16_t val_input_shape[] = {static_cast<uint16_t>(val_samples), 
                                          static_cast<uint16_t>(input_size)};
            uint16_t val_output_shape[] = {static_cast<uint16_t>(val_samples), 
                                           static_cast<uint16_t>(output_size)};
            
            aitensor_t val_input = AITENSOR_2D_F32(val_input_shape, x_val.data());
            std::vector<float> val_pred(val_samples * output_size);
            aitensor_t val_output = AITENSOR_2D_F32(val_output_shape, val_pred.data());
            
            AIFES_E_inference_fnn_f32(&val_input, &nn, &val_output);
            
            // Calculate MSE loss
            for (int i = 0; i < val_samples * output_size; i++) {
                float diff = val_pred[i] - y_val[i];
                val_loss += diff * diff;
            }
            val_loss /= (val_samples * output_size);
        }
        
        TickType_t epoch_end = xTaskGetTickCount();
        int duration_ms = (epoch_end - epoch_start) * portTICK_PERIOD_MS;
        
        // Track best model
        bool is_best = (val_samples > 0) ? (val_loss < result.best_val_loss) 
                                          : (train_loss < result.best_loss);
        if (is_best) {
            result.best_loss = train_loss;
            result.best_val_loss = val_loss;
            result.best_epoch = epoch;
            // Save best weights
            std::memcpy(best_weights_.data(), weights_.data(), 
                        info_.total_params * sizeof(float));
        }
        
        // Write to log
        if (!config.log_path.empty()) {
            WriteTrainLog(config.log_path.c_str(), epoch, train_loss, val_loss, duration_ms);
        }
        
        // Print progress
        if (epoch % config.print_interval == 0) {
            printf("Epoch %d/%d - loss: %.4f", epoch + 1, config.epochs, train_loss);
            if (val_samples > 0) {
                printf(" - val_loss: %.4f", val_loss);
            }
            if (config.sigreg_lambda > 0.0f) {
                printf(" - sigreg: %.3f", sigreg_loss);
            }
            if (is_best) {
                printf(" *");
            }
            printf("\n");
        }
        
        // Early stopping
        if (config.early_stopping && train_loss < config.early_stopping_target) {
            printf("Early stopping at epoch %d (loss %.4f < %.4f)\n", 
                   epoch + 1, train_loss, config.early_stopping_target);
            result.total_epochs = epoch + 1;
            break;
        }
        
        result.total_epochs = epoch + 1;
    }
    
    // Restore best weights
    if (!best_weights_.empty()) {
        std::memcpy(weights_.data(), best_weights_.data(), 
                    info_.total_params * sizeof(float));
    }
    
    TickType_t end_time = xTaskGetTickCount();
    result.total_time_s = (end_time - start_time) * portTICK_PERIOD_MS / 1000.0f;
    result.final_loss = result.best_loss;
    result.log_path = config.log_path;
    result.success = true;
    
    info_.trained = true;
    last_result_ = result;
    
    printf("\nTraining complete in %.1fs\n", result.total_time_s);
    printf("Best epoch: %d, best_loss: %.4f, best_val_loss: %.4f\n",
           result.best_epoch + 1, result.best_loss, result.best_val_loss);
    
    return result;
}

int AifesTask::Predict(const float* input, float* output) {
    if (!model_loaded_ || !info_.trained) {
        return -1;
    }
    
    // Setup model for inference
    AIFES_E_model_parameter_fnn_f32 nn;
    nn.layer_count = structure_len_;
    nn.fnn_structure = fnn_structure_;
    nn.fnn_activations = reinterpret_cast<AIFES_E_activations*>(activations_);
    nn.flat_weights = weights_.data();
    
    // Single sample inference
    uint16_t input_shape[] = {1, static_cast<uint16_t>(info_.input_size)};
    uint16_t output_shape[] = {1, static_cast<uint16_t>(info_.output_size)};
    
    aitensor_t input_tensor = AITENSOR_2D_F32(input_shape, const_cast<float*>(input));
    aitensor_t output_tensor = AITENSOR_2D_F32(output_shape, output);
    
    int8_t error = AIFES_E_inference_fnn_f32(&input_tensor, &nn, &output_tensor);
    return error;
}

// ============== NEW: TPU-style set_input/invoke/output API ==============

// Extern declarations for TPU functions (defined in sentai_runtime.cc)
extern "C" {
    int sentai_tpu_get_output_size(int idx);
    const void* sentai_tpu_get_output_data(int idx);
    int sentai_tpu_get_output_type(int idx);
    int sentai_tpu_output_quant(int idx, float* scale, int32_t* zero_point);
}

int AifesTask::SetInput(const float* data, int size) {
    if (!model_loaded_) {
        return -2;  // No model loaded
    }
    if (size != info_.input_size) {
        return -1;  // Size mismatch
    }
    
    input_buffer_.resize(size);
    std::memcpy(input_buffer_.data(), data, size * sizeof(float));
    input_set_ = true;
    
    // Pre-allocate output buffer
    output_buffer_.resize(info_.output_size);
    
    return 0;
}

int AifesTask::FromTpu(int idx) {
    if (!model_loaded_) {
        return -2;  // No model loaded
    }
    
    int size = sentai_tpu_get_output_size(idx);
    int type = sentai_tpu_get_output_type(idx);
    const void* data = sentai_tpu_get_output_data(idx);
    
    if (!data || size <= 0) {
        return -3;  // No TPU output
    }
    
    // Calculate number of elements
    int n_elements = 0;
    float scale = 1.0f;
    int32_t zero_point = 0;
    
    if (type == 1) {
        // float32
        n_elements = size / 4;
    } else if (type == 9 || type == 3) {
        // int8 or uint8
        sentai_tpu_output_quant(idx, &scale, &zero_point);
        n_elements = size;
    } else {
        return -4;  // Unsupported type
    }
    
    // Resize input buffer
    input_buffer_.resize(n_elements);
    
    // Dequantize into input buffer
    if (type == 1) {
        // float32 - direct copy
        std::memcpy(input_buffer_.data(), data, n_elements * sizeof(float));
    } else if (type == 9) {
        // int8 - dequantize
        const int8_t* idata = static_cast<const int8_t*>(data);
        for (int i = 0; i < n_elements; i++) {
            input_buffer_[i] = scale * (static_cast<float>(idata[i]) - static_cast<float>(zero_point));
        }
    } else {
        // uint8 - dequantize
        const uint8_t* udata = static_cast<const uint8_t*>(data);
        for (int i = 0; i < n_elements; i++) {
            input_buffer_[i] = scale * (static_cast<float>(udata[i]) - static_cast<float>(zero_point));
        }
    }
    
    input_set_ = true;
    
    // Check size match with model
    if (n_elements != info_.input_size) {
        printf("AIfES: TPU output size %d != model input size %d\n", 
               n_elements, info_.input_size);
        // Don't fail - user might want partial data
    }
    
    // Pre-allocate output buffer
    output_buffer_.resize(info_.output_size);
    
    return 0;
}

int AifesTask::FromCamera(int width, int height, bool grayscale) {
    if (!model_loaded_) {
        return -2;  // No model loaded
    }
    
    // Calculate expected size
    int n_elements = grayscale ? (width * height) : (width * height * 3);
    
    // Check if size matches model input
    if (n_elements != info_.input_size) {
        printf("AIfES: Camera %dx%d %s = %d floats, model expects %d\n",
               width, height, grayscale ? "gray" : "RGB", 
               n_elements, info_.input_size);
        return -5;  // Size mismatch
    }
    
    // Allocate temporary buffer for uint8 data
    std::vector<uint8_t> raw_buf(n_elements);
    
    // Capture camera frame
    int rc = sentai_aifes_capture_camera(raw_buf.data(), width, height, 
                                          grayscale ? 1 : 0);
    if (rc < 0) {
        printf("AIfES: Camera capture failed: %d\n", rc);
        return rc;
    }
    
    // Convert uint8 [0,255] to float [0,1]
    input_buffer_.resize(n_elements);
    for (int i = 0; i < n_elements; i++) {
        input_buffer_[i] = static_cast<float>(raw_buf[i]) / 255.0f;
    }
    
    input_set_ = true;
    output_buffer_.resize(info_.output_size);
    
    return 0;
}

int AifesTask::FromMic(int samples) {
    if (!model_loaded_) {
        return -2;  // No model loaded
    }
    
    // Check if size matches model input
    if (samples != info_.input_size) {
        printf("AIfES: Mic %d samples, model expects %d\n", 
               samples, info_.input_size);
        return -5;  // Size mismatch
    }
    
    // Check if mic is initialized
    if (!sentai_mic_is_initialized()) {
        printf("AIfES: Microphone not initialized\n");
        return -6;
    }
    
    // Allocate temporary buffer for int16 data
    std::vector<int16_t> raw_buf(samples);
    
    // Capture audio samples
    int rc = sentai_aifes_capture_mic(raw_buf.data(), samples);
    if (rc < 0) {
        printf("AIfES: Mic capture failed: %d\n", rc);
        return rc;
    }
    
    // Convert int16 [-32768,32767] to float [-1,1]
    input_buffer_.resize(samples);
    for (int i = 0; i < samples; i++) {
        input_buffer_[i] = static_cast<float>(raw_buf[i]) / 32768.0f;
    }
    
    input_set_ = true;
    output_buffer_.resize(info_.output_size);
    
    return 0;
}

int AifesTask::Invoke() {
    if (!model_loaded_ || !info_.trained) {
        return -1;  // No trained model
    }
    if (!input_set_) {
        return -2;  // No input set
    }
    
    TickType_t start = xTaskGetTickCount();
    
    int rc = Predict(input_buffer_.data(), output_buffer_.data());
    if (rc != 0) {
        return -3;  // Inference failed
    }
    
    TickType_t end = xTaskGetTickCount();
    int elapsed_ms = (end - start) * portTICK_PERIOD_MS;
    
    return elapsed_ms;
}

// C interface implementations
extern "C" {

int aifes_load_model(const char* yaml_path) {
    return AifesTask::GetSingleton()->LoadModel(yaml_path);
}

int aifes_load_weights(const char* weights_path) {
    return AifesTask::GetSingleton()->LoadWeights(weights_path);
}

int aifes_save_weights(const char* weights_path) {
    return AifesTask::GetSingleton()->SaveWeights(weights_path);
}

float aifes_train(const float* x_data, const float* y_data,
                  int n_samples, int input_size, int output_size,
                  int epochs, int batch_size, float learning_rate,
                  int optimizer, int loss_fn, float val_split,
                  const char* log_path, float sigreg_lambda) {
    AifesTrainConfig config;
    config.epochs = epochs;
    config.batch_size = batch_size;
    config.learning_rate = learning_rate;
    config.optimizer = optimizer;
    config.loss = loss_fn;
    config.val_split = val_split;
    config.sigreg_lambda = sigreg_lambda;
    if (log_path) {
        config.log_path = log_path;
    }
    
    AifesTrainResult result = AifesTask::GetSingleton()->Train(
        x_data, y_data, n_samples, input_size, output_size, config);
    
    return result.success ? result.final_loss : -1.0f;
}

int aifes_predict(const float* input, float* output) {
    return AifesTask::GetSingleton()->Predict(input, output);
}

int aifes_is_loaded(void) {
    return AifesTask::GetSingleton()->IsLoaded() ? 1 : 0;
}

const char* aifes_get_model_name(void) {
    return AifesTask::GetSingleton()->GetInfo().name.c_str();
}

int aifes_get_layer_count(void) {
    return AifesTask::GetSingleton()->GetInfo().layer_count;
}

int aifes_get_input_size(void) {
    return AifesTask::GetSingleton()->GetInfo().input_size;
}

int aifes_get_output_size(void) {
    return AifesTask::GetSingleton()->GetInfo().output_size;
}

int aifes_get_total_params(void) {
    return AifesTask::GetSingleton()->GetInfo().total_params;
}

const char* aifes_get_log_path(void) {
    return AifesTask::GetSingleton()->GetLastTrainResult().log_path.c_str();
}

int aifes_get_best_epoch(void) {
    return AifesTask::GetSingleton()->GetLastTrainResult().best_epoch;
}

float aifes_get_best_val_loss(void) {
    return AifesTask::GetSingleton()->GetLastTrainResult().best_val_loss;
}

void aifes_unload(void) {
    AifesTask::GetSingleton()->Unload();
}

// ============== NEW: TPU-style set_input/invoke/output API ==============

int aifes_set_input(const float* data, int size) {
    return AifesTask::GetSingleton()->SetInput(data, size);
}

int aifes_from_tpu(int idx) {
    return AifesTask::GetSingleton()->FromTpu(idx);
}

int aifes_invoke(void) {
    return AifesTask::GetSingleton()->Invoke();
}

const float* aifes_get_output_data(void) {
    return AifesTask::GetSingleton()->GetOutputData();
}

int aifes_get_output_count(void) {
    return AifesTask::GetSingleton()->GetOutputCount();
}

int aifes_from_camera(int width, int height, int grayscale) {
    return AifesTask::GetSingleton()->FromCamera(width, height, grayscale != 0);
}

int aifes_from_mic(int samples) {
    return AifesTask::GetSingleton()->FromMic(samples);
}

}  // extern "C"

}  // namespace coralmicro
