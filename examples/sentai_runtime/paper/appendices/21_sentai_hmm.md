# Appendix: sentai.hmm

The `sentai.hmm` namespace implements a discrete hidden Markov model with Baum-Welch training and Viterbi decoding. It is suitable for compact sequence-classification tasks in which observations can be discretized, such as simple activity recognition or discretized feature streams extracted from sensors.

## Functions

- `sentai.hmm.init(n_states, n_obs)`: Initializes the HMM.
- `sentai.hmm.set_transition(i, j, prob)`: Sets a transition probability.
- `sentai.hmm.set_emission(state, obs, prob)`: Sets an emission probability.
- `sentai.hmm.set_prior(state, prob)`: Sets an initial-state probability.
- `sentai.hmm.add_seq(obs_list)`: Adds a training sequence.
- `sentai.hmm.train(max_iter)`: Runs Baum-Welch training.
- `sentai.hmm.viterbi(obs_list)`: Decodes the most likely state sequence.
- `sentai.hmm.predict(obs)`: Predicts the most likely next state.
- `sentai.hmm.log_likelihood(obs_list)`: Scores a sequence.
- `sentai.hmm.from_tpu(idx, n_bins)`: Discretizes a TPU output tensor.
- `sentai.hmm.from_imu(n_bins)`: Discretizes IMU magnitude.
- `sentai.hmm.save(path)`: Saves the HMM model.
- `sentai.hmm.load(path)`: Loads a saved model.
- `sentai.hmm.info()`: Returns model metadata.
- `sentai.hmm.clear()`: Clears parameters and sequences.

## Example

```python
import sentai

sentai.imu.init()
sentai.hmm.init(3, 8)
seq = [sentai.hmm.from_imu(8) for _ in range(100)]
sentai.hmm.add_seq(seq)
sentai.hmm.train(20)
print(sentai.hmm.predict(sentai.hmm.from_imu(8)))
```