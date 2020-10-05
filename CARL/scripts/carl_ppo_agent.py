import os
import sys
import glob
import time
import numpy as np
import copy as copy
import tensorflow as tf
from tensorflow.python import pywrap_tensorflow

from env.env import Env
from learning.ppo_agent import PPOAgent
from learning.solvers.mpi_solver import MPISolver
import learning.tf_util as TFUtil
import tf_util_extend as TFUtilExtend
import learning.rl_util as RLUtil
from util.logger import Logger
import util.mpi_util as MPIUtil
import mpi_util_extend as MPIUtilExtend
import util.math_util as MathUtil
from env.env import Env
import net_builder as NetBuilder


class CarlPPOAgent(PPOAgent):
    NAME = "CarlPPO"
    PRIMITIVES_KEY = 'Primitives'
    PRETRAINING_KEY = 'Pretraining'
    ACTOR_PRIMITIVES_NET_KEY = 'ActorPrimitivesNet'
    ACTOR_GATING_NET_KEY = 'ActorGatingNet'
    GATING_REGULARIZATION_LAMBDA_KEY = "GatingRegularizationLambda"
    RECORD_OUTPUT_ITERS_KEY = "RecordOutputIters"
    ENABLE_RECORD_DATA_KEY = 'EnableRecordData'
    MAX_RECORD_SAMPLE_COUNT_KEY = 'MaxRecordSampleCount'
    CONTROL_TYPE_KEY = 'ControlType'
    INIT_TOTAL_SAMPLES_KEY = "InitTotalSamples"
    MAX_ITERATION_KEY = 'MaxIteration'

    def __init__(self, world, id, json_data, seed):
        self._enable_record_data = False
        self._enable_update_normalizer = True
        self._records = [] # (s, g, g2, w) in a trajectory
        self._trajectories = [] # history trajectories
        self._max_record_sample_count = 0
        self._record_smaple_count = 0
        self._max_iteration = -1
        self._init_total_sample_count = 0
        self.control_type = 'speed'
        self.record_output_iters = 10
        self.train_mean_reward = 0.0
        self.train_pathlen = 0.0
        self.random_seed = seed

        super().__init__(world, id, json_data)
        return

    def _load_params(self, json_data):
        super()._load_params(json_data)

        if (self.RECORD_OUTPUT_ITERS_KEY in json_data):
            self.record_output_iters = json_data[self.RECORD_OUTPUT_ITERS_KEY]

        if (self.MAX_RECORD_SAMPLE_COUNT_KEY in json_data):
            self._max_record_sample_count = json_data[self.MAX_RECORD_SAMPLE_COUNT_KEY]

        if (self.CONTROL_TYPE_KEY in json_data):
            self.control_type = json_data[self.CONTROL_TYPE_KEY]

        if (self.INIT_TOTAL_SAMPLES_KEY in json_data):
            self._init_total_sample_count = int(json_data[self.INIT_TOTAL_SAMPLES_KEY])

        if (self.ENABLE_RECORD_DATA_KEY in json_data):
            self._enable_record_data = bool(json_data[self.ENABLE_RECORD_DATA_KEY])

        if (self.MAX_ITERATION_KEY in json_data):
            self._max_iteration = json_data[self.MAX_ITERATION_KEY]
        return

    def _build_graph(self, json_data):
        with self.sess.as_default(), self.graph.as_default():
            if self.random_seed is not None and self.random_seed != 0:
                tf.set_random_seed(self.random_seed)
            with tf.variable_scope(self.tf_scope):
                self._build_nets(json_data)

                with tf.variable_scope(self.SOLVER_SCOPE):
                    self._build_losses(json_data)
                    self._build_solvers(json_data)

                self._initialize_vars()
                self._build_saver()
        return

    def _build_nets(self, json_data):
        assert self.ACTOR_PRIMITIVES_NET_KEY in json_data
        assert self.ACTOR_GATING_NET_KEY in json_data
        assert self.CRITIC_NET_KEY in json_data

        actor_num_primitives = json_data[self.PRIMITIVES_KEY]
        actor_primitives_net_name = json_data[self.ACTOR_PRIMITIVES_NET_KEY]
        actor_gating_net_name = json_data[self.ACTOR_GATING_NET_KEY]
        critic_net_name = json_data[self.CRITIC_NET_KEY]
        actor_init_output_scale = 1 if (self.ACTOR_INIT_OUTPUT_SCALE_KEY not in json_data) else json_data[self.ACTOR_INIT_OUTPUT_SCALE_KEY]
        self.enable_pretraining = json_data[self.PRETRAINING_KEY]

        s_size = self.get_state_size()
        g_size = self.get_goal_size()
        a_size = self.get_action_size()

        # setup input tensors
        self.s_tf = tf.placeholder(tf.float32, shape=[None, s_size], name="s")
        self.a_tf = tf.placeholder(tf.float32, shape=[None, a_size], name="a")
        self.tar_val_tf = tf.placeholder(tf.float32, shape=[None], name="tar_val")
        self.adv_tf = tf.placeholder(tf.float32, shape=[None], name="adv")
        self.g_tf = tf.placeholder(tf.float32, shape=([None, g_size] if self.has_goal() else None), name="g")
        self.old_logp_tf = tf.placeholder(tf.float32, shape=[None], name="old_logp")
        self.exp_mask_tf = tf.placeholder(tf.float32, shape=[None], name="exp_mask")

        with tf.variable_scope('main'):
            with tf.variable_scope('actor'):
                self.a_mean_tf, self.a_std_tf = self._build_net_actor(actor_primitives_net_name, actor_gating_net_name, actor_num_primitives, actor_init_output_scale)
            with tf.variable_scope('critic'):
                self.critic_tf = self._build_net_critic(critic_net_name)
            with tf.variable_scope('generator'):
                self.generator = self._build_net_gating(actor_gating_net_name, actor_num_primitives, actor_init_output_scale)

        if (self.a_mean_tf != None):
            Logger.print('Built actor net: %s (primitives) + %s (gating)' % (actor_primitives_net_name, actor_gating_net_name))

        if (self.critic_tf != None):
            Logger.print('Built critic net: ' + critic_net_name)

        self.norm_a_std_tf = self.a_std_tf
        norm_a_noise_tf = self.norm_a_std_tf * tf.random_normal(shape=tf.shape(self.a_mean_tf))
        norm_a_noise_tf *= tf.expand_dims(self.exp_mask_tf, axis=-1)
        self.sample_a_tf = self.a_mean_tf + norm_a_noise_tf * self.a_norm.std_tf
        self.sample_a_logp_tf = TFUtil.calc_logp_gaussian(x_tf=norm_a_noise_tf, mean_tf=None, std_tf=self.norm_a_std_tf)
        return

    def _enable_pretraining(self):
        return self.enable_pretraining

    def _build_losses(self, json_data):
        actor_weight_decay = 0 if (self.ACTOR_WEIGHT_DECAY_KEY not in json_data) else json_data[self.ACTOR_WEIGHT_DECAY_KEY]
        critic_weight_decay = 0 if (self.CRITIC_WEIGHT_DECAY_KEY not in json_data) else json_data[self.CRITIC_WEIGHT_DECAY_KEY]
        gating_regularization_lambda = 0 if (self.GATING_REGULARIZATION_LAMBDA_KEY not in json_data) else json_data[self.GATING_REGULARIZATION_LAMBDA_KEY]

        norm_val_diff = self.val_norm.normalize_tf(self.tar_val_tf) - self.val_norm.normalize_tf(self.critic_tf)
        self.critic_loss_tf = 0.5 * tf.reduce_mean(tf.square(norm_val_diff))

        if (critic_weight_decay != 0):
            self.critic_loss_tf += critic_weight_decay * self._weight_decay_loss('main/critic')

        norm_tar_a_tf = self.a_norm.normalize_tf(self.a_tf)
        self._norm_a_mean_tf = self.a_norm.normalize_tf(self.a_mean_tf)

        self.logp_tf = TFUtil.calc_logp_gaussian(norm_tar_a_tf, self._norm_a_mean_tf, self.norm_a_std_tf)
        ratio_tf = tf.exp(self.logp_tf - self.old_logp_tf)
        actor_loss0 = self.adv_tf * ratio_tf
        actor_loss1 = self.adv_tf * tf.clip_by_value(ratio_tf, 1.0 - self.ratio_clip, 1 + self.ratio_clip)
        self.actor_loss_tf = -tf.reduce_mean(tf.minimum(actor_loss0, actor_loss1))

        norm_a_bound_min = self.a_norm.normalize(self.a_bound_min)
        norm_a_bound_max = self.a_norm.normalize(self.a_bound_max)
        a_bound_loss = TFUtil.calc_bound_loss(self._norm_a_mean_tf, norm_a_bound_min, norm_a_bound_max)
        self.actor_loss_tf += a_bound_loss

        self.regularization_loss_tf = None
        if gating_regularization_lambda > 0:
            vars_generator = []
            vars_gating = []
            for var in tf.trainable_variables():
                if 'bias' in var.name: continue ## Ignore bias
                if 'generator' in var.name:
                    vars_generator.append(var)
                elif 'gating' in var.name:
                    vars_gating.append(var)
            self.regularization_loss_tf = 0
            for i in range(0, len(vars_gating)):
                l1_loss = tf.reduce_mean(tf.keras.losses.MAE(vars_generator[i], vars_gating[i]))
                self.regularization_loss_tf += l1_loss
            self.actor_loss_tf += self.regularization_loss_tf * gating_regularization_lambda

        if (actor_weight_decay != 0):
            self.actor_loss_tf += actor_weight_decay * self._weight_decay_loss('main/actor')

        # for debugging
        self.clip_frac_tf = tf.reduce_mean(tf.to_float(tf.greater(tf.abs(ratio_tf - 1.0), self.ratio_clip)))

        return

    def _build_solvers(self, json_data):
        actor_stepsize = 0.001 if (self.ACTOR_STEPSIZE_KEY not in json_data) else json_data[self.ACTOR_STEPSIZE_KEY]
        actor_momentum = 0.9 if (self.ACTOR_MOMENTUM_KEY not in json_data) else json_data[self.ACTOR_MOMENTUM_KEY]
        critic_stepsize = 0.01 if (self.CRITIC_STEPSIZE_KEY not in json_data) else json_data[self.CRITIC_STEPSIZE_KEY]
        critic_momentum = 0.9 if (self.CRITIC_MOMENTUM_KEY not in json_data) else json_data[self.CRITIC_MOMENTUM_KEY]

        critic_vars = self._tf_vars('main/critic')
        critic_opt = tf.train.MomentumOptimizer(learning_rate=critic_stepsize, momentum=critic_momentum)
        self.critic_grad_tf = tf.gradients(self.critic_loss_tf, critic_vars)
        self.critic_solver = MPISolver(self.sess, critic_opt, critic_vars)

        self._actor_stepsize_tf = tf.get_variable(dtype=tf.float32, name='actor_stepsize', initializer=actor_stepsize, trainable=False)
        self._actor_stepsize_ph = tf.get_variable(dtype=tf.float32, name='actor_stepsize_ph', shape=[])
        self._actor_stepsize_update_op = self._actor_stepsize_tf.assign(self._actor_stepsize_ph)

        actor_vars = self._tf_vars('main/actor')
        if not self.enable_pretraining:
            vars_to_remove = []
            for var in actor_vars:
                if 'main/actor/primitives/' in var.name:
                    vars_to_remove.append(var)
            for rem in vars_to_remove:
                actor_vars.remove(rem)

        # for var in actor_vars:
        #     Logger.print(var)

        actor_opt = tf.train.MomentumOptimizer(learning_rate=self._actor_stepsize_tf, momentum=actor_momentum)
        self.actor_grad_tf = tf.gradients(self.actor_loss_tf, actor_vars)
        self.actor_solver = MPISolver(self.sess, actor_opt, actor_vars)
        return

    def _build_net_actor(self, primitives_net_name, gating_net_name, num_primitives, init_output_scale):
        with tf.variable_scope('primitives'):
            primitives_mean_tf, primitives_std_tf = self._build_net_primitives(primitives_net_name, num_primitives, init_output_scale)
        with tf.variable_scope('gating'):
            w_tf = self._build_net_gating(gating_net_name, num_primitives, init_output_scale)

        self.primitives_mean_tf = primitives_mean_tf
        self.primitives_std_tf = primitives_std_tf
        self.gating_weights_tf = w_tf

        # normalize the gating weights
        w_scale = 1.0 / tf.reduce_sum(w_tf, axis=1, keepdims=True)  # (?, 1)

        std_op_tf = tf.stack(primitives_std_tf, axis=0)             # (num_primitives, ?, action_dim)
        mean_op_tf = tf.stack(primitives_mean_tf, axis=0)           # (num_primitives, ?, action_dim)
        w_op_tf = tf.expand_dims(tf.transpose(w_tf * w_scale), 2)   # (num_primitives, ?, 1)

        # build composite variance
        norm_a_std_tf = tf.reduce_sum(w_op_tf / std_op_tf, axis=0)                # (?, action_dim)
        norm_a_std_tf = 1.0 / norm_a_std_tf                                       # (?, action_dim)
        # build composite mean
        norm_a_mean_tf = tf.reduce_sum(w_op_tf / std_op_tf * mean_op_tf, axis=0)  # (?, action_dim)
        norm_a_mean_tf *= norm_a_std_tf                                           # (?, action_dim)

        a_mean_tf = self.a_norm.unnormalize_tf(norm_a_mean_tf)
        a_std_tf = norm_a_std_tf
        return a_mean_tf, a_std_tf

    def _build_net_primitives(self, net_name, num_primitives, init_output_scale):
        input_tfs = []
        norm_s_tf = self.s_norm.normalize_tf(self.s_tf)
        input_tfs += [norm_s_tf]
        h = NetBuilder.build_net(net_name, input_tfs)
        a_dim = self.get_action_size()
        batch_size = tf.shape(norm_s_tf)[0]
        primitives_mean_tf = []
        primitives_std_tf = []
        for i in range(0, num_primitives):
            h2 = tf.layers.dense(inputs=h, units=256, activation=tf.nn.relu, name='primitive_%d_dense' % (i),
                                 kernel_initializer=TFUtil.xavier_initializer)
            norm_mean_tf = tf.layers.dense(inputs=h2, units=a_dim, activation=None, name='primitive_%d_dense_mean' % (i),
                                    kernel_initializer=tf.random_uniform_initializer(minval=-init_output_scale, maxval=init_output_scale))
            norm_std_tf = self.exp_params_curr.noise * tf.ones([batch_size, a_dim])
            primitives_mean_tf.append(norm_mean_tf)
            primitives_std_tf.append(norm_std_tf)
        return primitives_mean_tf, primitives_std_tf

    def _build_net_gating(self, net_name, num_primitives, init_output_scale):
        input_tfs = []
        norm_s_tf = self.s_norm.normalize_tf(self.s_tf)
        input_tfs += [norm_s_tf]
        if (self.has_goal()):
            norm_g_tf = self.g_norm.normalize_tf(self.g_tf)
            input_tfs += [norm_g_tf]
        h = NetBuilder.build_net(net_name, input_tfs)
        w_tf = tf.layers.dense(inputs=h, units=num_primitives, activation=tf.nn.sigmoid,
                               kernel_initializer=TFUtil.xavier_initializer)
        return w_tf

    def _train_step(self):
        adv_eps = 1e-5

        start_idx = self.replay_buffer.buffer_tail
        end_idx = self.replay_buffer.buffer_head
        assert(start_idx == 0)
        assert(self.replay_buffer.get_current_size() <= self.replay_buffer.buffer_size) # must avoid overflow
        assert(start_idx < end_idx)

        idx = np.array(list(range(start_idx, end_idx)))
        end_mask = self.replay_buffer.is_path_end(idx)
        end_mask = np.logical_not(end_mask)

        vals = self._compute_batch_vals(start_idx, end_idx)
        new_vals = self._compute_batch_new_vals(start_idx, end_idx, vals)

        valid_idx = idx[end_mask]
        exp_idx = self.replay_buffer.get_idx_filtered(self.EXP_ACTION_FLAG).copy()
        num_valid_idx = valid_idx.shape[0]
        num_exp_idx = exp_idx.shape[0]
        exp_idx = np.column_stack([exp_idx, np.array(list(range(0, num_exp_idx)), dtype=np.int32)])

        local_sample_count = valid_idx.size
        global_sample_count = int(MPIUtil.reduce_sum(local_sample_count))
        mini_batches = int(np.ceil(global_sample_count / self.mini_batch_size))

        adv = new_vals[exp_idx[:,0]] - vals[exp_idx[:,0]]
        new_vals = np.clip(new_vals, self.val_min, self.val_max)

        adv_mean = np.mean(adv)
        adv_std = np.std(adv)
        adv = (adv - adv_mean) / (adv_std + adv_eps)
        adv = np.clip(adv, -self.norm_adv_clip, self.norm_adv_clip)

        critic_loss = 0
        actor_loss = 0
        regularization_loss = 0
        actor_clip_frac = 0
        gating_weights_sum = 0
        primitives_action_mean = [0] * len(self.primitives_mean_tf)
        primitives_action_std = [0] * len(self.primitives_std_tf)
        action_mean = 0
        action_std = 0

        for _ in range(self.epochs):
            np.random.shuffle(valid_idx)
            np.random.shuffle(exp_idx)

            for b in range(mini_batches):
                batch_idx_beg = b * self._local_mini_batch_size
                batch_idx_end = batch_idx_beg + self._local_mini_batch_size

                critic_batch = np.array(range(batch_idx_beg, batch_idx_end), dtype=np.int32)
                actor_batch = critic_batch.copy()
                critic_batch = np.mod(critic_batch, num_valid_idx)
                actor_batch = np.mod(actor_batch, num_exp_idx)
                shuffle_actor = (actor_batch[-1] < actor_batch[0]) or (actor_batch[-1] == num_exp_idx - 1)

                critic_batch = valid_idx[critic_batch]
                actor_batch = exp_idx[actor_batch]
                critic_batch_vals = new_vals[critic_batch]
                actor_batch_adv = adv[actor_batch[:,1]]

                critic_s = self.replay_buffer.get('states', critic_batch)
                critic_g = self.replay_buffer.get('goals', critic_batch) if self.has_goal() else None
                curr_critic_loss = self._update_critic(critic_s, critic_g, critic_batch_vals)

                actor_s = self.replay_buffer.get("states", actor_batch[:,0])
                actor_g = self.replay_buffer.get("goals", actor_batch[:,0]) if self.has_goal() else None
                actor_a = self.replay_buffer.get("actions", actor_batch[:,0])
                actor_logp = self.replay_buffer.get("logps", actor_batch[:,0])
                curr_actor_loss, curr_actor_clip_frac, curr_regularization_loss = self._update_actor(actor_s, actor_g, actor_a, actor_logp, actor_batch_adv)

                gating_weights_sum += np.mean(np.sum(self._eval_gating_weights(actor_s, actor_g), axis=1))
                action_mean_batch, action_std_batch = self._eval_action_mean_std(actor_s, actor_g)
                action_mean += np.mean(action_mean_batch)
                action_std += np.mean(action_std_batch)
                primitives_mean_batch, primitives_std_batch = self._eval_primitives_action_mean_std(actor_s, actor_g)
                for i in range(0, len(primitives_mean_batch)):
                    primitives_action_mean[i] += np.mean(primitives_mean_batch[i])
                    primitives_action_std[i] += np.mean(primitives_std_batch[i])

                critic_loss += curr_critic_loss
                actor_loss += np.abs(curr_actor_loss)
                regularization_loss += curr_regularization_loss
                actor_clip_frac += curr_actor_clip_frac

                if (shuffle_actor):
                    np.random.shuffle(exp_idx)

        total_batches = mini_batches * self.epochs
        critic_loss /= total_batches
        actor_loss /= total_batches
        regularization_loss /= total_batches
        actor_clip_frac /= total_batches
        gating_weights_sum /= total_batches
        action_mean /= total_batches
        action_std /= total_batches
        primitives_action_mean = [primitive_mean / total_batches for primitive_mean in primitives_action_mean]
        primitives_action_std = [primitive_std / total_batches for primitive_std in primitives_action_std]

        critic_loss = MPIUtil.reduce_avg(critic_loss)
        actor_loss = MPIUtil.reduce_avg(actor_loss)
        actor_clip_frac = MPIUtil.reduce_avg(actor_clip_frac)

        critic_stepsize = self.critic_solver.get_stepsize()
        actor_stepsize = self.update_actor_stepsize(actor_clip_frac)

        self.logger.log_tabular('Critic_Loss', critic_loss)
        self.logger.log_tabular('Critic_Stepsize', critic_stepsize)
        self.logger.log_tabular('Actor_Loss', actor_loss)
        self.logger.log_tabular('Actor_L1_Loss', regularization_loss)
        self.logger.log_tabular('Actor_Stepsize', actor_stepsize)
        self.logger.log_tabular('Gating_Weights', gating_weights_sum)
        for i, primitive_mean in enumerate(primitives_action_mean):
            self.logger.log_tabular('Primitive%d_Mean' % i, primitive_mean)
        for i, primitive_std in enumerate(primitives_action_std):
            self.logger.log_tabular('Primitive%d_Std' % i, primitive_std)
        self.logger.log_tabular('Action_Mean', action_mean)
        self.logger.log_tabular('Action_Std', action_std)
        self.logger.log_tabular('Clip_Frac', actor_clip_frac)
        self.logger.log_tabular('Adv_Mean', adv_mean)
        self.logger.log_tabular('Adv_Std', adv_std)

        self.replay_buffer.clear()
        return

    def _train(self):
        with self.sess.as_default(), self.graph.as_default():
            samples = self.replay_buffer.total_count
            self._total_sample_count = int(MPIUtil.reduce_sum(samples)) + self._init_total_sample_count
            end_training = False

            if (self.replay_buffer_initialized):
                if (self._valid_train_step()):
                    prev_iter = self.iter
                    iters = self._get_iters_per_update()
                    avg_train_return = MPIUtil.reduce_avg(self.train_return)
                    avg_train_mean_reward = MPIUtil.reduce_avg(self.train_mean_reward)
                    avg_train_pathlen = MPIUtil.reduce_avg(self.train_pathlen)
                    avg_train_pathlen /= 30 # policy is executed in 30Hz

                    for _ in range(iters):
                        curr_iter = self.iter
                        wall_time = time.time() - self.start_time
                        wall_time /= 60 * 60 # store time in hours

                        has_goal = self.has_goal()
                        s_mean = np.mean(self.s_norm.mean)
                        s_std = np.mean(self.s_norm.std)
                        g_mean = np.mean(self.g_norm.mean) if has_goal else 0
                        g_std = np.mean(self.g_norm.std) if has_goal else 0

                        self.logger.log_tabular("Iteration", self.iter)
                        self.logger.log_tabular("Wall_Time", wall_time)
                        self.logger.log_tabular("Samples", self._total_sample_count)
                        self.logger.log_tabular("Train_Path_Length", avg_train_pathlen)
                        self.logger.log_tabular("Train_Mean_Reward", avg_train_mean_reward)
                        self.logger.log_tabular("Train_Return", avg_train_return)
                        self.logger.log_tabular("Test_Return", self.avg_test_return)
                        self.logger.log_tabular("State_Mean", s_mean)
                        self.logger.log_tabular("State_Std", s_std)
                        self.logger.log_tabular("Goal_Mean", g_mean)
                        self.logger.log_tabular("Goal_Std", g_std)
                        self._log_exp_params()

                        self._update_iter(self.iter + 1)

                        train_start_time = time.time()
                        self._train_step()
                        train_time = time.time() - train_start_time

                        if self.iter == 1:
                            iteration_time = time.time() - self.start_time
                        else:
                            iteration_time = time.time() - self.iter_start_time
                        self.iter_start_time = time.time()

                        self.logger.log_tabular("Train_time", train_time)
                        self.logger.log_tabular("Simulation_time",  iteration_time - train_time)

                        Logger.print("Agent " + str(self.id))
                        self.logger.print_tabular()
                        Logger.print("")

                        if (self._enable_output() and curr_iter % self.int_output_iters == 0):
                            self.logger.dump_tabular()


                    if (prev_iter // self.int_output_iters != self.iter // self.int_output_iters):
                        end_training = self.enable_testing()

            else:

                Logger.print("Agent " + str(self.id))
                Logger.print("Samples: " + str(self._total_sample_count))
                Logger.print("")

                if (self._total_sample_count >= self.init_samples):
                    self.replay_buffer_initialized = True
                    end_training = self.enable_testing()

            if self._need_normalizer_update and self._enable_update_normalizer:
                self._update_normalizers()
                self._need_normalizer_update = self.normalizer_samples > self._total_sample_count

            if end_training:
                self._init_mode_train_end()
        return

    def _update_actor(self, s, g, a, logp, adv):
        feed = {
            self.s_tf: s,
            self.g_tf: g,
            self.a_tf: a,
            self.adv_tf: adv,
            self.old_logp_tf: logp
        }

        if self.regularization_loss_tf is not None:
            loss, grads, clip_frac, reg_loss = self.sess.run([self.actor_loss_tf, self.actor_grad_tf,
                                                            self.clip_frac_tf, self.regularization_loss_tf], feed)
        else:
            loss, grads, clip_frac = self.sess.run([self.actor_loss_tf, self.actor_grad_tf, self.clip_frac_tf], feed)
            reg_loss = 0

        self.actor_solver.update(grads)
        return loss, clip_frac, reg_loss

    def _eval_primitives_action_mean_std(self, s, g):
        with self.sess.as_default(), self.graph.as_default():
            s = np.reshape(s, [-1, self.get_state_size()])
            g = np.reshape(g, [-1, self.get_goal_size()]) if self.has_goal() else None

            feed = {
                self.s_tf : s
            }

            primitives_mean = []
            for a_mean_tf in self.primitives_mean_tf:
                mean = a_mean_tf.eval(feed)
                primitives_mean.append(mean)

            primitives_std = []
            for a_std_tf in self.primitives_std_tf:
                std = a_std_tf.eval(feed)
                primitives_std.append(std)

        return primitives_mean, primitives_std

    def _eval_action_mean_std(self, s, g):
        with self.sess.as_default(), self.graph.as_default():
            s = np.reshape(s, [-1, self.get_state_size()])
            g = np.reshape(g, [-1, self.get_goal_size()]) if self.has_goal() else None

            feed = {
                self.s_tf : s,
                self.g_tf : g
            }

            a_mean = self.a_mean_tf.eval(feed)
            a_std = self.a_std_tf.eval(feed)
            norm_a_mean = self.a_norm.normalize(a_mean)
            norm_a_std = a_std
        return norm_a_mean, norm_a_std

    def _eval_gating_weights(self, s, g):
        with self.sess.as_default(), self.graph.as_default():
            s = np.reshape(s, [-1, self.get_state_size()])
            g = np.reshape(g, [-1, self.get_goal_size()]) if self.has_goal() else None

            feed = {
                self.s_tf : s,
                self.g_tf : g
            }
            w = self.gating_weights_tf.eval(feed)
        return w

    def _log_primitives_means_stds(self, s, g):
        primitives_mean, primitives_std = self._eval_primitives_action_mean_std(s, g)
        num_primitives = len(primitives_mean)
        means = []
        stds = []
        for i in range(0, num_primitives):
            means.extend(primitives_mean[i].tolist()[0])
            stds.extend(primitives_std[i].tolist()[0])
        self.world.env.log_primitives_means_stds(self.id, num_primitives, means, stds)
        return

    def _log_gating_weights(self, s, g):
        weights = self._eval_gating_weights(s, g)
        self.world.env.log_gating_weights(self.id, weights[0])
        return

    def _record_data(self, s, g):
        w = self._eval_gating_weights(s, g)
        g_target = self._record_goal_target()
        record = [s, g_target, w[0]]
        self._records.append(record)
        return

    def save_records(self, filename):
        def numpy_1d_array_to_string(arr):
            line = ''
            arr = list(arr)
            for i in range(0, len(arr)):
                if i < len(arr) - 1:
                    line += '{:.10}'.format(arr[i]) + ' '
                else:
                    line += '{:.10}'.format(arr[i])
            return line

        trajectories_list = MPIUtilExtend.gather(self._trajectories)
        if MPIUtil.is_root_proc():
            trajectories = trajectories_list[0]
            for i in range(1, len(trajectories_list)):
                trajectories += trajectories_list[i]

            with open(filename, 'w') as fout:
                for records in trajectories:
                    for record in records:
                        line = ''
                        for entry in record:
                            line += numpy_1d_array_to_string(entry) + '\n'
                        fout.write(line)
                    fout.write('\n')
                    self._record_smaple_count += len(records)

            _root_sample_count = np.array(self._record_smaple_count, dtype=np.float32)
            MPIUtil.bcast(_root_sample_count)
        else:
            _root_sample_count = np.array([0], dtype=np.float32)
            MPIUtil.bcast(_root_sample_count)
            self._record_smaple_count = int(_root_sample_count[0])

        Logger.print('Record Sample Count: ' + str(self._record_smaple_count))
        self._trajectories.clear()
        return

    def load_model(self, policy_model_path, control_adapter_model_path):
        with self.sess.as_default(), self.graph.as_default():
            try:
                self.saver.restore(self.sess, policy_model_path)
            except:
                # manually restore primitive network from the checkpoint
                reader = pywrap_tensorflow.NewCheckpointReader(policy_model_path)
                var_to_shape_map = reader.get_variable_to_shape_map()
                for key in var_to_shape_map:
                    if '_norm/' in key:
                        continue
                    if not self._enable_pretraining() and ('/gating/' in key or '/critic/' in key):
                        continue
                    scope = key[0:key.rfind('/')]
                    var_name = key[key.rfind('/')+1:]
                    with tf.variable_scope(scope, reuse=tf.AUTO_REUSE):
                        var = tf.get_variable(var_name)
                        tensor = reader.get_tensor(key)
                        if var.shape[0] == tensor.shape[0]:
                            self.sess.run(var.assign(tensor))
                        else:
                            assert(False)
                        Logger.print('{:<65} restored'.format(var.name))

            self._load_normalizers()
            Logger.print('Policy model successfully loaded from: ' + policy_model_path)

            # restore gating network from control adapter model
            if control_adapter_model_path:
                reader = pywrap_tensorflow.NewCheckpointReader(control_adapter_model_path)
                var_to_shape_map = reader.get_variable_to_shape_map()

                params = dict()
                for key in var_to_shape_map.keys():
                    if 'generator' in key and 'Adam' not in key:
                        params[key] = reader.get_tensor(key)
                params = sorted(params)

                Logger.print('-'*25 + ' Loading Gating Netowrk ' + '-'*25)
                var_list = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='agent/main/actor/gating')
                var_names = []
                for var in var_list:
                    var_names.append(var.name)

                mapping = []
                for key in params:
                    # name = key.replace(':0', '').replace('control_adapter/generator', 'agent/main/actor/gating').replace('/output/', '/') + ':0'
                    name = key.replace(':0', '').replace('generator', 'agent/main/actor/gating').replace('/5/', '/') + ':0'
                    if name in var_names:
                        idx = var_names.index(name)
                        mapping.append(idx)
                    else:
                        assert(False)
                for i in range(len(params)):
                    var = var_list[i]
                    key = params[mapping[i]]
                    self.sess.run(var.assign(reader.get_tensor(key)))
                    Logger.print('{:<50} -> {:<30}'.format(key, var.name))

                Logger.print('-'*25 + ' Loading Generator Netowrk ' + '-'*25)
                var_list = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='agent/main/generator')
                var_names = []
                for var in var_list:
                    var_names.append(var.name)

                # Make a copy of the gating network for regularization in DRL fine-tuning training
                mapping = []
                for key in params:
                    # name = key.replace(':0', '').replace('control_adapter/generator', 'agent/main/generator').replace('/output/', '/') + ':0'
                    name = key.replace(':0', '').replace('generator', 'agent/main/generator').replace('/5/', '/') + ':0'
                    if name in var_names:
                        idx = var_names.index(name)
                        mapping.append(idx)
                    else:
                        assert(False)
                for i in range(len(params)):
                    var = var_list[i]
                    key = params[mapping[i]]
                    self.sess.run(var.assign(reader.get_tensor(key)))
                    Logger.print('{:<50} -> {:<30}'.format(key, var.name))

                Logger.print('Control adapter model successfully loaded from: ' + control_adapter_model_path)
        return

    def _get_records_output_path(self):
        assert(self.int_output_dir != '')
        file_path = self.int_output_dir + ('/agent{:d}_records/agent{:d}_records_{:010d}.txt').format(self.id, self.id, self.iter)
        return file_path

    def _build_saver(self):
        vars = self._get_saver_vars()
        vars_restore = []
        for var in vars:
            if 'discriminator' not in var.name and 'generator' not in var.name:
                vars_restore.append(var)
        self.saver = tf.train.Saver(vars_restore, max_to_keep=0)

    def update(self, timestep):
        if self.need_new_action():
            self._update_new_action()

        if (self._mode == self.Mode.TRAIN and self.enable_training):
            self._update_counter += timestep

            while self._update_counter >= self.update_period:
                self._train()
                self._update_exp_params()
                self.world.env.set_sample_count(self._total_sample_count)
                self._update_counter -= self.update_period

                if self._enable_record_data and self._record_smaple_count > self._max_record_sample_count:
                    self._end_record_data()
        return

    def has_goal_target(self):
        g = self._record_goal_target()
        return not (g.shape[0] == 1 and g[0] == 0)

    def enable_evaluation(self):
        return True

    def _record_goal_target(self):
        g = self.world.env.record_goal_target(self.id)
        return g

    def _end_path(self):
        s = self._record_state()
        g = self._record_goal()
        r = self._record_reward()

        self.path.rewards.append(r)
        self.path.states.append(s)
        self.path.goals.append(g)
        self.path.terminate = self.world.env.check_terminate(self.id)

        if self._enable_record_data:
            self._record_data(s, g)
            self._trajectories.append(self._records)
            self._records.clear()
        return

    def _update_new_action(self):
        s = self._record_state()
        g = self._record_goal()

        if not (self._is_first_step()):
            r = self._record_reward()
            self.path.rewards.append(r)

        a, logp = self._decide_action(s=s, g=g)
        assert len(np.shape(a)) == 1
        assert len(np.shape(logp)) <= 1

        flags = self._record_flags()
        self._apply_action(a)

        self.path.states.append(s)
        self.path.goals.append(g)
        self.path.actions.append(a)
        self.path.logps.append(logp)
        self.path.flags.append(flags)

        if self._enable_record_data:
            self._record_data(s, g)

        if self._enable_draw():
            self._log_val(s, g)
            self._log_gating_weights(s, g)
            self._log_primitives_means_stds(s, g)
        return

    def _update_iter(self, iter):
        super()._update_iter(iter)

        if (self._enable_record_data and self.iter % self.record_output_iters == 0):
            records_output_path = self._get_records_output_path()
            records_output_dir = os.path.dirname(records_output_path)
            if MPIUtil.is_root_proc() and not os.path.exists(records_output_dir):
                os.makedirs(records_output_dir)
            self.save_records(records_output_path)
        return

    def _store_path(self, path):
        path_id = super()._store_path(path)

        valid_path = path_id != MathUtil.INVALID_IDX

        if valid_path:
            self.train_mean_reward = np.average(path.rewards)
            self.train_pathlen = path.get_pathlen()

        return path_id

    def _end_record_data(self):
        if MPIUtil.is_root_proc():
            records_output_path = self._get_records_output_path()
            records_output_dir = os.path.dirname(records_output_path)
            if not os.path.exists(records_output_dir):
                os.makedirs(records_output_dir)

            trajectories = []
            record_files = glob.glob(records_output_dir + '/*.txt')
            for record_file in record_files:
                print('Read file ' + record_file)
                _trajectories = self._parse_record_data(record_file)
                trajectories += _trajectories

            states, goals, weights = [], [], []
            if self.control_type == 'speed':
                for records in trajectories:
                    for i in range(len(records)):
                        states += [list(records[i][0])]
                        goals += [list(records[i][1])]
                        weights += [list(records[i][2])]
            elif self.control_type == 'heading':
                n_previous_steps = 1
                for records in trajectories:
                    for i in range(n_previous_steps, len(records)):
                        state_curr = records[i][0]
                        goal_curr = records[i][1]
                        goal_prev = records[i-n_previous_steps][1]
                        weight_curr = records[i][2]

                        theta_curr = np.arctan2(-goal_curr[2], goal_curr[0])
                        theta_prev = np.arctan2(-goal_prev[2], goal_prev[0])
                        theta_delta = theta_curr - theta_prev
                        goal_curr_new = np.concatenate([goal_prev, [theta_delta]], axis=-1)

                        states += [list(state_curr)]
                        goals += [list(goal_curr_new)]
                        weights += [list(weight_curr)]
            else:
                Logger.print('Unsupported control type')
                assert(0)

            states = np.array(states, dtype=np.float32)
            goals = np.array(goals, dtype=np.float32)
            weights = np.array(weights, dtype=np.float32)

            output_path = self._get_output_path()
            output_dir = os.path.dirname(output_path)
            if not os.path.exists(records_output_dir):
                os.makedirs(records_output_dir)

            np.save(output_dir + '/states', states)
            np.save(output_dir + '/goals', goals)
            np.save(output_dir + '/weights', weights)

            print('Saved record data with %d samples' % (states.shape[0]))
        return

    def _parse_record_data(self, filename):
        trajectories = []
        with open(filename) as fin:
            lines = fin.readlines()
            i = 0
            records = []
            while i < len(lines):
                if lines[i] == '\n':
                    trajectories += [records]
                    records = []
                    i += 1
                else:
                    state = np.fromstring(lines[i], dtype=np.float32, sep=' ')
                    goal = np.fromstring(lines[i + 1], dtype=np.float32, sep=' ')
                    weight = np.fromstring(lines[i + 2], dtype=np.float32, sep=' ')

                    record = [state, goal, weight]
                    records += [record]
                    i += 3

        return trajectories

    def isDone(self):
        if self._enable_record_data:
            return self._record_smaple_count > self._max_record_sample_count
        elif self._enable_training and self._max_iteration > 0:
            return self.iter > self._max_iteration
        else:
            return False
