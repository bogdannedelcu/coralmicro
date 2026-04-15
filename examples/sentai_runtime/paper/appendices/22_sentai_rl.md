# Appendix: sentai.rl

The `sentai.rl` namespace collects three reinforcement-learning mechanisms with different complexity levels: tabular Q-learning, multi-armed bandits, and a compact DQN. It is intended for online adaptation of thresholds, policies, or configuration choices in small embedded experiments.

## Functions

- `sentai.rl.q_init(n_states, n_actions)`: Initializes a tabular Q-table.
- `sentai.rl.q_update(s, a, r, s_next, alpha, gamma)`: Updates one Q-value.
- `sentai.rl.q_action(s, epsilon)`: Selects an action with an epsilon-greedy policy.
- `sentai.rl.q_value(s, a)`: Returns one Q-value.
- `sentai.rl.q_save(path)`: Saves the Q-table.
- `sentai.rl.q_load(path)`: Loads the Q-table.
- `sentai.rl.q_clear()`: Clears the Q-table.
- `sentai.rl.mab_init(n_arms)`: Initializes a multi-armed bandit.
- `sentai.rl.mab_pull(arm, reward)`: Reports the reward of a selected arm.
- `sentai.rl.mab_select(strategy)`: Selects the next arm.
- `sentai.rl.mab_stats()`: Returns per-arm statistics.
- `sentai.rl.mab_clear()`: Clears bandit statistics.
- `sentai.rl.dqn_init(state_dim, n_actions, hidden)`: Initializes the DQN.
- `sentai.rl.dqn_observe(state, action, reward, next_state, done)`: Adds one replay transition.
- `sentai.rl.dqn_action(state, epsilon)`: Selects an action from the DQN.
- `sentai.rl.dqn_train(batch_size)`: Trains on a replay batch.
- `sentai.rl.dqn_save(path)`: Saves DQN weights.
- `sentai.rl.dqn_load(path)`: Loads DQN weights.
- `sentai.rl.dqn_clear()`: Clears DQN state.
- `sentai.rl.info()`: Returns a summary of all three subsystems.

## Example

```python
import sentai

sentai.rl.mab_init(4)
for _ in range(20):
    arm = sentai.rl.mab_select(1)
    reward = 1.0 if arm == 0 else 0.2
    sentai.rl.mab_pull(arm, reward)
print(sentai.rl.mab_stats())
```