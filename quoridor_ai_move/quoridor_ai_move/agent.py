from __future__ import annotations
from pathlib import Path

import numpy as np
import tensorflow as tf

try:
    from .encoder import NUM_ACTIONS
    from .model import build_qnet, masked_argmax
except ImportError:
    from encoder import NUM_ACTIONS
    from model import build_qnet, masked_argmax


class DQNAgent:
    def __init__(
        self,
        lr: float = 1e-3,
        gamma: float = 0.99,
        strategy: tf.distribute.Strategy | None = None,
        filters: int = 64,
        n_blocks: int = 4,
        tau: float = 0.005,          # soft update rate (0 = hard copy, like before)
        grad_clip: float = 1.0,      # max global gradient norm
    ):
        self.gamma = gamma
        self.tau = tau
        self.grad_clip = grad_clip
        self.strategy = strategy or tf.distribute.get_strategy()
        with self.strategy.scope():
            self.q_net = build_qnet(filters=filters, n_blocks=n_blocks)
            self.target_net = build_qnet(filters=filters, n_blocks=n_blocks)
            self.target_net.set_weights(self.q_net.get_weights())
            self.optimizer = tf.keras.optimizers.Adam(
                learning_rate=lr,
                clipnorm=grad_clip   # gradient clipping built into optimizer
            )
            # Build the optimizer eagerly inside the strategy scope so its
            # slot variables (Adam momentum/velocity) are colocated with the
            # model weights. Otherwise the lazy build() fires inside the
            # @tf.function _train_step on the first call, outside any scope,
            # and MultiWorkerMirroredStrategy raises "Need to be inside
            # with strategy.scope()".
            self.optimizer.build(self.q_net.trainable_variables)

    def select_action(self, state: np.ndarray, mask: np.ndarray, epsilon: float) -> int:
        legal = np.flatnonzero(mask > 0.5)
        if legal.size == 0:
            return -1
        if np.random.random() < epsilon:
            return int(np.random.choice(legal))
        q = self.q_net(state[None, ...], training=False).numpy()[0]
        q_masked = np.where(mask > 0.5, q, -1e9)
        return int(np.argmax(q_masked))

    @tf.function
    def _train_step(self, s, a, r, s2, done, mask2):
        # Target Q using frozen target network
        next_q = self.target_net(s2, training=False)
        neg_inf = tf.fill(tf.shape(next_q), tf.constant(-1e9, dtype=next_q.dtype))
        next_q = tf.where(mask2 > 0.5, next_q, neg_inf)
        max_next = tf.reduce_max(next_q, axis=-1)
        max_next = tf.where(tf.reduce_sum(mask2, axis=-1) > 0.5,
                            max_next, tf.zeros_like(max_next))
        target = r + self.gamma * (1.0 - done) * max_next

        with tf.GradientTape() as tape:
            q = self.q_net(s, training=True)
            a_onehot = tf.one_hot(a, NUM_ACTIONS, dtype=q.dtype)
            q_a = tf.reduce_sum(q * a_onehot, axis=-1)
            # Huber loss instead of MSE — much more robust to outlier rewards
            loss = tf.reduce_mean(
                tf.keras.losses.huber(
                    tf.stop_gradient(target), q_a, delta=1.0
                )
            )
        grads = tape.gradient(loss, self.q_net.trainable_variables)
        # clipnorm is set on the optimizer, but clip here too as a safety net
        grads, _ = tf.clip_by_global_norm(grads, self.grad_clip)
        self.optimizer.apply_gradients(zip(grads, self.q_net.trainable_variables))
        return loss

    def train_on_batch(self, batch):
        s, a, r, s2, done, mask2 = batch
        s   = tf.convert_to_tensor(s)
        a   = tf.convert_to_tensor(a)
        r   = tf.convert_to_tensor(r)
        s2  = tf.convert_to_tensor(s2)
        done   = tf.convert_to_tensor(done)
        mask2  = tf.convert_to_tensor(mask2)
        return float(self._train_step(s, a, r, s2, done, mask2).numpy())

    def update_target(self, hard: bool = False):
        if hard or self.tau >= 1.0:
            # Hard copy — same as before
            self.target_net.set_weights(self.q_net.get_weights())
        else:
            # Soft update: target = τ*online + (1-τ)*target
            for online_var, target_var in zip(
                self.q_net.weights, self.target_net.weights
            ):
                target_var.assign(
                    self.tau * online_var + (1.0 - self.tau) * target_var
                )

    def save(self, directory: str):
        Path(directory).mkdir(parents=True, exist_ok=True)
        self.q_net.save_weights(str(Path(directory) / "qnet.weights.h5"))

    def load(self, directory: str):
        p = Path(directory) / "qnet.weights.h5"
        if p.exists():
            self.q_net.load_weights(str(p))
            self.target_net.set_weights(self.q_net.get_weights())
            return True
        return False