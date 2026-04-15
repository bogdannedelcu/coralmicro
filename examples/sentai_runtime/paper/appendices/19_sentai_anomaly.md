# Appendix: sentai.anomaly

The `sentai.anomaly` namespace implements online anomaly detection based on incremental covariance estimation and Mahalanobis distance, together with a simple CUSUM change detector. It is suitable for learning a baseline normal distribution and then scoring embeddings or scalar signals during deployment.

## Functions

- `sentai.anomaly.init(dim)`: Initializes the anomaly model.
- `sentai.anomaly.observe(vector)`: Updates the normal model with one observation.
- `sentai.anomaly.observe_tpu(idx)`: Updates the model from a TPU output tensor.
- `sentai.anomaly.score(vector)`: Returns the anomaly score for a vector.
- `sentai.anomaly.score_tpu(idx)`: Scores a TPU output tensor.
- `sentai.anomaly.threshold(val)`: Gets or sets the anomaly threshold.
- `sentai.anomaly.is_anomaly(vector)`: Tests whether a vector exceeds the threshold.
- `sentai.anomaly.is_anomaly_tpu(idx)`: Tests a TPU output tensor.
- `sentai.anomaly.cusum_init(threshold, drift)`: Initializes the CUSUM detector.
- `sentai.anomaly.cusum_observe(value)`: Feeds one scalar sample into CUSUM.
- `sentai.anomaly.cusum_score()`: Returns current positive and negative cumulative sums.
- `sentai.anomaly.cusum_reset()`: Clears CUSUM accumulators.
- `sentai.anomaly.save(path)`: Saves the learned model.
- `sentai.anomaly.load(path)`: Loads a saved model.
- `sentai.anomaly.stats()`: Returns model statistics.
- `sentai.anomaly.info()`: Alias for `stats()`.
- `sentai.anomaly.clear()`: Clears all statistics.

## Example

```python
import sentai

sentai.tpu.load('/models/mobilenet_features.tflite')
sentai.camera.init()
sentai.anomaly.init(1280)

for _ in range(30):
    sentai.camera.to_tensor()
    sentai.tpu.invoke()
    sentai.anomaly.observe_tpu(0)

sentai.camera.to_tensor()
sentai.tpu.invoke()
print(sentai.anomaly.score_tpu(0))
```