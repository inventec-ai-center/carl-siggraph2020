import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import tensorflow as tf
from tensorflow.python import pywrap_tensorflow
from sklearn.decomposition import PCA
import sys

sys.path.append('../DeepMimic/')
sys.path.append('scripts/')

import tf_util_extend as TFUtilExtend
import net_builder as NetBuilder


Gating_Net_Name = 'fc_3layers_512units_branch_inputs'
Num_Primitives = 8


def build_net_gating(s_tf, g_tf, w_dim, reuse):
    input_tfs = [s_tf, g_tf]
    h = NetBuilder.build_net(Gating_Net_Name, input_tfs, reuse)
    w_tf = tf.layers.dense(inputs=h, units=w_dim, activation=tf.nn.sigmoid, kernel_initializer=TFUtilExtend.xavier_initializer)
    return w_tf


def build_normalizer(filename):
    reader = pywrap_tensorflow.NewCheckpointReader(filename)
    s_mean = reader.get_tensor('agent/resource/s_norm/mean')
    s_std  = reader.get_tensor('agent/resource/s_norm/std')
    g_mean = reader.get_tensor('agent/resource/g_norm/mean')
    g_std  = reader.get_tensor('agent/resource/g_norm/std')
    return s_mean, s_std, g_mean, g_std


def get_vars(file_name):
    varlist=[]
    reader = pywrap_tensorflow.NewCheckpointReader(file_name)
    var_to_shape_map = reader.get_variable_to_shape_map()
    for key in sorted(var_to_shape_map):
        if ("gating" in key or "generator" in key) and 'Adam' not in key:
            varlist.append(key)
    return varlist


def calc_weights(states, goals, model_file):
    tf.reset_default_graph()
    s_dim = states.shape[1]
    g_dim = goals.shape[1]
    w_dim = Num_Primitives

    s_tf  = tf.placeholder(tf.float32, shape=([None, s_dim]), name="s")
    g_tf = tf.placeholder(tf.float32, shape=([None, g_dim]), name="g")

    s_mean, s_std, g_mean, g_std = build_normalizer(model_file)
    with tf.variable_scope('agent/main/actor/gating'):
        s_norm_tf = (s_tf - s_mean) / s_std
        g_norm_tf = (g_tf - g_mean) / g_std
        w_tf = build_net_gating(s_norm_tf, g_norm_tf, w_dim, reuse=False)

    with tf.Session() as sess:
        vars_restore = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES)
        saver = tf.train.Saver(vars_restore)
        saver.restore(sess, model_file)
        weights = sess.run(w_tf, feed_dict={s_tf:states, g_tf:goals})
        return weights


states = np.load('output/dog3d_gan_control_adapter_speed_control/states.npy')
goals = np.load('output/dog3d_gan_control_adapter_speed_control/goals.npy')
weights_reference = np.load('output/dog3d_gan_control_adapter_speed_control/weights.npy')
weights_gan = calc_weights(states, goals, 'data/policies/dog3d/model_drl_finetuning_speed_control.ckpt')

print(weights_reference.shape)
print(weights_gan.shape)

CANTER_SPEED = 3.5
TROT_SPEED = 2
PACE_SPEED = 1

tolerance = 0.2

mask_canter = (goals[:,-1] >= CANTER_SPEED-tolerance)  & (goals[:, -1] <= CANTER_SPEED+tolerance)
mask_trot   = (goals[:,-1] >= TROT_SPEED-tolerance)    & (goals[:, -1] <= TROT_SPEED+tolerance)
mask_pace   = (goals[:,-1] >= PACE_SPEED-tolerance)    & (goals[:, -1] <= PACE_SPEED+tolerance)

print('Canter samples: ', np.where(mask_canter)[0].shape)
print('Trot   samples: ', np.where(mask_trot)[0].shape)
print('Pace   samples: ', np.where(mask_pace)[0].shape)

num_samples = np.min([np.where(mask_canter)[0].shape, np.where(mask_trot)[0].shape, np.where(mask_pace)[0].shape])

indices_canter = np.where(mask_canter)[0][:num_samples]
indices_trot   = np.where(mask_trot)[0][:num_samples]
indices_pace   = np.where(mask_pace)[0][:num_samples]

def plot_2d_action_dist(weights, weights_gt, exp_name, exp_color):
    pca = PCA(n_components=2).fit(weights_gt)
    weights_2d = pca.transform(weights)
    weights_2d_canter = weights_2d[indices_canter]
    weights_2d_trot = weights_2d[indices_trot]
    weights_2d_pace = weights_2d[indices_pace]

    weights_gt_2d = pca.transform(weights_gt)
    weights_gt_2d_canter = weights_gt_2d[indices_canter]
    weights_gt_2d_trot = weights_gt_2d[indices_trot]
    weights_gt_2d_pace = weights_gt_2d[indices_pace]

    s = 0.1
    alpha = 0.2
    gt_color = 'C3'
    y_scale = -1

    fig = plt.figure(figsize=(9,3))
    ax = fig.add_subplot(131)

    ax.set_title('Pace')
    ax.scatter(weights_gt_2d_pace[:,0], weights_gt_2d_pace[:,1] * y_scale, color=gt_color, s=s, alpha=alpha)
    ax.scatter(weights_2d_pace[:,0], weights_2d_pace[:,1] * y_scale, color=exp_color, s=s, alpha=alpha)
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_xticks([])
    ax.set_yticks([])

    ax = fig.add_subplot(132)
    ax.set_title('Trot')
    ax.scatter(weights_gt_2d_trot[:,0], weights_gt_2d_trot[:,1] * y_scale, color=gt_color, s=s, alpha=alpha)
    ax.scatter(weights_2d_trot[:,0],  weights_2d_trot[:,1] * y_scale, color=exp_color, s=s, alpha=alpha)
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_xticks([])
    ax.set_yticks([])

    ax = fig.add_subplot(133)
    ax.set_title('Canter')
    ax.scatter(weights_gt_2d_canter[:,0], weights_gt_2d_canter[:,1] * y_scale, color=gt_color, s=s, alpha=alpha)
    ax.scatter(weights_2d_canter[:,0], weights_2d_canter[:,1] * y_scale, color=exp_color, s=s, alpha=alpha)
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_xticks([])
    ax.set_yticks([])

    c0_patch = mpatches.Patch(color=gt_color, label='Reference')
    c1_patch = mpatches.Patch(color=exp_color, label=exp_name)
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles=[c0_patch, c1_patch], loc='right', bbox_to_anchor=(1.23, 0.5), bbox_transform=fig.transFigure)

    plt.savefig("action_dist.png", dpi=300, bbox_inches="tight")

plot_2d_action_dist(weights_gan, weights_reference, 'GAN Control Adapter', 'C0')
