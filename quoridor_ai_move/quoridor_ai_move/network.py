"""AlphaZero dual-head ResNet.

Body: small Conv2D stem + N residual blocks.
Policy head: 1x1 conv -> flatten -> Dense(NUM_ACTIONS), trained against MCTS visit counts.
Value  head: 1x1 conv -> flatten -> Dense(1) tanh, trained against game outcome (-1..+1).

Defaults are intentionally smaller than the DQN net (32 filters, 3 blocks)
because the 5x5 state space doesn't need the capacity, and a smaller net
trains faster per step and overfits the buffer less.
"""
from __future__ import annotations

import tensorflow as tf
from tensorflow.keras import layers, regularizers

try:
    from .encoder import BOARD_N, NUM_ACTIONS, STATE_CHANNELS
except ImportError:
    from encoder import BOARD_N, NUM_ACTIONS, STATE_CHANNELS

L2 = 1e-4


def _conv_bn(x, filters, k=3):
    x = layers.Conv2D(filters, k, padding="same", use_bias=False,
                      kernel_regularizer=regularizers.l2(L2))(x)
    x = layers.BatchNormalization()(x)
    return x


def _residual(x, filters):
    y = _conv_bn(x, filters); y = layers.ReLU()(y)
    y = _conv_bn(y, filters)
    return layers.ReLU()(layers.Add()([x, y]))


def build_az_net(filters: int = 32, n_blocks: int = 3) -> tf.keras.Model:
    inp = layers.Input(shape=(BOARD_N, BOARD_N, STATE_CHANNELS))
    x = _conv_bn(inp, filters); x = layers.ReLU()(x)
    for _ in range(n_blocks):
        x = _residual(x, filters)

    # Policy head.
    p = layers.Conv2D(2, 1, use_bias=False,
                      kernel_regularizer=regularizers.l2(L2))(x)
    p = layers.BatchNormalization()(p); p = layers.ReLU()(p)
    p = layers.Flatten()(p)
    policy_logits = layers.Dense(NUM_ACTIONS, name="policy",
                                 kernel_regularizer=regularizers.l2(L2))(p)

    # Value head.
    v = layers.Conv2D(1, 1, use_bias=False,
                      kernel_regularizer=regularizers.l2(L2))(x)
    v = layers.BatchNormalization()(v); v = layers.ReLU()(v)
    v = layers.Flatten()(v)
    v = layers.Dense(64, activation="relu",
                     kernel_regularizer=regularizers.l2(L2))(v)
    value = layers.Dense(1, activation="tanh", name="value",
                         kernel_regularizer=regularizers.l2(L2))(v)

    return tf.keras.Model(inp, [policy_logits, value], name="az_net")
