"""Q-network architecture: a small AlphaZero-style ResNet trunk feeding
a fully-connected head that outputs one Q-value per action."""
from __future__ import annotations

import tensorflow as tf
from tensorflow.keras import layers, Model

# from .encoder import BOARD_N, NUM_ACTIONS, STATE_CHANNELS

try:
    from .encoder import BOARD_N, NUM_ACTIONS, STATE_CHANNELS
except ImportError:
    from .encoder import BOARD_N, NUM_ACTIONS, STATE_CHANNELS

def build_qnet(filters: int = 64, n_blocks: int = 4, dropout: float = 0.1) -> Model:
    reg = tf.keras.regularizers.l2(1e-4)
    inp = layers.Input(shape=(BOARD_N, BOARD_N, STATE_CHANNELS), name="state")
    x = layers.Conv2D(filters, 3, padding="same", use_bias=False,
                      kernel_regularizer=reg)(inp)
    x = layers.BatchNormalization()(x)
    x = layers.ReLU()(x)
    for _ in range(n_blocks):
        x = _residual_block(x, filters, reg)
    x = layers.Flatten()(x)
    x = layers.Dense(256, activation="relu", kernel_regularizer=reg)(x)
    x = layers.Dropout(dropout)(x)   # ADDED
    q = layers.Dense(NUM_ACTIONS, activation=None, name="q")(x)
    return Model(inp, q, name="quoridor_qnet")

def _residual_block(x, filters: int, reg=None):
    shortcut = x
    x = layers.Conv2D(filters, 3, padding="same", use_bias=False,
                      kernel_regularizer=reg)(x)
    x = layers.BatchNormalization()(x)
    x = layers.ReLU()(x)
    x = layers.Conv2D(filters, 3, padding="same", use_bias=False,
                      kernel_regularizer=reg)(x)
    x = layers.BatchNormalization()(x)
    x = layers.Add()([x, shortcut])
    x = layers.ReLU()(x)
    return x


@tf.function
def masked_argmax(q_values: tf.Tensor, mask: tf.Tensor) -> tf.Tensor:
    """Argmax over q_values where mask==1. mask and q_values are (B, A)."""
    # Replace masked-out entries with -inf so they can never win the argmax.
    neg_inf = tf.fill(tf.shape(q_values), tf.constant(-1e9, dtype=q_values.dtype))
    masked = tf.where(mask > 0.5, q_values, neg_inf)
    return tf.argmax(masked, axis=-1, output_type=tf.int32)
