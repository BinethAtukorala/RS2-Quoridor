"""Q-network architecture: a small AlphaZero-style ResNet trunk feeding
a fully-connected head that outputs one Q-value per action."""
from __future__ import annotations

import tensorflow as tf
from tensorflow.keras import layers, Model

# from .encoder import BOARD_N, NUM_ACTIONS, STATE_CHANNELS

try:
    from .encoder import BOARD_N, NUM_ACTIONS, STATE_CHANNELS
except ImportError:
    from encoder import BOARD_N, NUM_ACTIONS, STATE_CHANNELS


def _residual_block(x, filters: int):
    # Standard pre-activation residual block: two 3x3 convs with batch norm,
    # added back to the input so gradients flow through deeper stacks.
    shortcut = x
    x = layers.Conv2D(filters, 3, padding="same", use_bias=False)(x)
    x = layers.BatchNormalization()(x)
    x = layers.ReLU()(x)
    x = layers.Conv2D(filters, 3, padding="same", use_bias=False)(x)
    x = layers.BatchNormalization()(x)
    x = layers.Add()([x, shortcut])
    x = layers.ReLU()(x)
    return x


def build_qnet(filters: int = 64, n_blocks: int = 4) -> Model:
    # Input: encoded board tensor from encoder.encode_state.
    inp = layers.Input(shape=(BOARD_N, BOARD_N, STATE_CHANNELS), name="state")
    # Stem conv brings the input up to `filters` channels before the resnet.
    x = layers.Conv2D(filters, 3, padding="same", use_bias=False)(inp)
    x = layers.BatchNormalization()(x)
    x = layers.ReLU()(x)
    # Residual trunk -- depth/width controlled by callers.
    for _ in range(n_blocks):
        x = _residual_block(x, filters)
    # Dense head produces one scalar Q-value for each of the NUM_ACTIONS slots.
    x = layers.Flatten()(x)
    x = layers.Dense(256, activation="relu")(x)
    q = layers.Dense(NUM_ACTIONS, activation=None, name="q")(x)
    return Model(inp, q, name="quoridor_qnet")


@tf.function
def masked_argmax(q_values: tf.Tensor, mask: tf.Tensor) -> tf.Tensor:
    """Argmax over q_values where mask==1. mask and q_values are (B, A)."""
    # Replace masked-out entries with -inf so they can never win the argmax.
    neg_inf = tf.fill(tf.shape(q_values), tf.constant(-1e9, dtype=q_values.dtype))
    masked = tf.where(mask > 0.5, q_values, neg_inf)
    return tf.argmax(masked, axis=-1, output_type=tf.int32)
