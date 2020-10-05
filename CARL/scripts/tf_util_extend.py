import tensorflow as tf
import numpy as np
import os

xavier_initializer = tf.contrib.layers.xavier_initializer()

def fc_net(input, net_count, layers_sizes, activation, reuse=None, flatten=False): # build fully connected network
    curr_tf = input
    for i, size in enumerate(layers_sizes):
        with tf.variable_scope(str(net_count), reuse=reuse):
            curr_tf = tf.layers.dense(inputs=curr_tf,
                                    units=size,
                                    kernel_initializer=xavier_initializer,
                                    activation = activation if i < len(layers_sizes)-1 else None)
        net_count += 1

    if flatten:
        curr_tf = tf.contrib.layers.flatten(curr_tf)

    return curr_tf, net_count

def conv1d_net(input, net_count, kernals_sizes, filters_sizes, activation, reuse=None, flatten=False): # build 1D convolutional network
    curr_tf = input
    for i, size in enumerate(kernals_sizes):
        with tf.variable_scope(str(net_count), reuse=reuse):
            curr_tf = tf.layers.conv1d(inputs=curr_tf,
                                       filters=filters_sizes[i],
                                       kernel_size=size,
                                       kernel_initializer=xavier_initializer,
                                       activation = activation if i < len(kernals_sizes)-1 else None)
            net_count += 1

    if flatten:
        curr_tf = tf.contrib.layers.flatten(curr_tf)

    return curr_tf, net_count

def conv2d_net(input, net_count, kernals_sizes, filters_sizes, activation, reuse=None, flatten=False): # build 2D convolutional network
    curr_tf = input
    for i, size in enumerate(kernals_sizes):
        with tf.variable_scope(str(net_count), reuse=reuse):
            curr_tf = tf.layers.conv2d(inputs=curr_tf,
                                       filters=filters_sizes[i],
                                       kernel_size=[size, size],
                                       kernel_initializer=xavier_initializer,
                                       activation = activation if i < len(kernals_sizes)-1 else None)
            net_count += 1

    if flatten:
        curr_tf = tf.contrib.layers.flatten(curr_tf)

    return curr_tf, net_count
