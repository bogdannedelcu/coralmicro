# Appendix: sentai.kmeans

The `sentai.kmeans` namespace implements compact k-means clustering for embedding vectors. It is mainly useful for few-shot or unsupervised classification workflows in which a TPU feature extractor produces dense embeddings and the runtime clusters them directly on device.

## Functions

- `sentai.kmeans.init(k, dim)`: Initializes the clustering state.
- `sentai.kmeans.add(cluster_idx, vector)`: Adds a labeled vector to a cluster.
- `sentai.kmeans.add_from_tpu(cluster_idx, tpu_idx)`: Adds a TPU output directly.
- `sentai.kmeans.compute()`: Computes centroids from labeled samples.
- `sentai.kmeans.fit(max_iter)`: Runs unsupervised Lloyd iterations.
- `sentai.kmeans.predict(vector)`: Predicts the closest centroid.
- `sentai.kmeans.from_tpu(tpu_idx)`: Predicts from a TPU output tensor.
- `sentai.kmeans.distances(vector)`: Returns distances to all centroids.
- `sentai.kmeans.save(path)`: Saves centroids to flash.
- `sentai.kmeans.load(path)`: Loads centroids from flash.
- `sentai.kmeans.centroid(idx)`: Returns one centroid vector.
- `sentai.kmeans.info()`: Returns configuration and sample-count metadata.
- `sentai.kmeans.clear()`: Clears training vectors while keeping centroids.

## Example

```python
import sentai

sentai.tpu.load('/models/mobilenet_features.tflite')
sentai.camera.init()
sentai.kmeans.init(3, 1280)

sentai.camera.to_tensor()
sentai.tpu.invoke()
sentai.kmeans.add_from_tpu(0, 0)
sentai.kmeans.compute()
print(sentai.kmeans.info())
```