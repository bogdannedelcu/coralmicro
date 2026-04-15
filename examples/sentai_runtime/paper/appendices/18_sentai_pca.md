# Appendix: sentai.pca

The `sentai.pca` namespace provides principal-component analysis for high-dimensional embeddings. Its role is to reduce vector dimension before downstream use in classifiers, clustering methods, or lightweight anomaly detectors, especially when raw feature vectors are too large for repeated on-device processing.

## Functions

- `sentai.pca.init(in_dim, out_dim)`: Initializes the PCA model.
- `sentai.pca.add(vector)`: Adds a training vector.
- `sentai.pca.from_tpu(idx)`: Adds a TPU output tensor as training data.
- `sentai.pca.fit()`: Computes the principal components.
- `sentai.pca.transform(vector)`: Projects a vector into the reduced space.
- `sentai.pca.transform_tpu(idx)`: Projects a TPU output tensor directly.
- `sentai.pca.inverse(reduced)`: Reconstructs an approximate original vector.
- `sentai.pca.explained_variance()`: Returns component variances.
- `sentai.pca.save(path)`: Saves the fitted PCA model.
- `sentai.pca.load(path)`: Loads a PCA model.
- `sentai.pca.info()`: Returns model metadata.
- `sentai.pca.clear()`: Clears stored training vectors.

## Example

```python
import sentai

sentai.tpu.load('/models/mobilenet_features.tflite')
sentai.camera.init()
sentai.pca.init(1280, 32)
sentai.camera.to_tensor()
sentai.tpu.invoke()
sentai.pca.from_tpu(0)
sentai.pca.fit()
print(sentai.pca.transform_tpu(0)[:5])
```