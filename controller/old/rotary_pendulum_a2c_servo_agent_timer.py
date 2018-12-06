from environment_servo_agent_timer import Env
from keras.optimizers import Adam, SGD, RMSprop
from keras.models import Sequential
from keras import backend as K
from keras.layers import Dense
from datetime import datetime
import tensorflow as tf
import numpy as np
import threading
import random
import pylab
import math
import time
import sys
import os

K.clear_session()
EPISODES = 1000
ACTION_TIME = 5 / 1000
SKIPPING = 4

history_size = 4

class A2CAgent:
    def __init__(self, state_size, action_size):
        self.load_model = False
        if os.path.exists("./save2/pendulum_actor_servo.h5") or \
                os.path.exists("./save2/pendulum_actor_servo.h5"):
            self.load_model = True

        # 상태와 행동의 크기 정의
        self.state_size = state_size * history_size
        self.action_size = action_size
        self.value_size = 1

        # 액터-크리틱 하이퍼파라미터
        self.discount_factor = 0.99
        self.actor_lr = 1e-4
        self.critic_lr = 1e-4
        # self.epsilon = 0.99
        # self.epsilon_decator = 0.995
        # self.epsilon_min = 0.1

        # 정책신경망과 가치신경망 생성
        self.actor = self.build_actor()
        self.critic = self.build_critic()
        self.actor_updater = self.actor_optimizer()
        self.critic_updater = self.critic_optimizer()

        if self.load_model:
            self.actor.load_weights("./save2/pendulum_actor_servo.h5")
            self.critic.load_weights("./save2/pendulum_critic_servo.h5")

    # actor: 상태를 받아 각 행동의 확률을 계산
    def build_actor(self):
        actor = Sequential()
        actor.add(Dense(48, input_shape=[self.state_size,], activation='relu', kernel_initializer='he_uniform'))
        actor.add(Dense(48, activation='relu', kernel_initializer='he_uniform'))
        actor.add(Dense(self.action_size, activation='softmax', kernel_initializer='he_uniform'))
        actor.summary()
        return actor

    # critic: 상태를 받아서 상태의 가치를 계산
    def build_critic(self):
        critic = Sequential()
        critic.add(Dense(48, input_shape=[self.state_size,], activation='relu', kernel_initializer='he_uniform'))
        critic.add(Dense(48, activation='relu', kernel_initializer='he_uniform'))
        critic.add(Dense(48, activation='relu', kernel_initializer='he_uniform'))
        critic.add(Dense(self.value_size, activation='linear', kernel_initializer='he_uniform'))
        critic.summary()
        return critic

    # 정책신경망의 출력을 받아 확률적으로 행동을 선택
    def get_action(self, state):
        policy = self.actor.predict(state, batch_size=1).flatten()
        action = np.random.choice(self.action_size, 1, p=policy)[0]
        # if random.random() > self.epsilon:
        #     action = np.argmax(policy)
        # else:
        #     action = int(random.randrange(0, self.action_size))
        return action

    # 정책신경망을 업데이트하는 함수
    def actor_optimizer(self):
        action = K.placeholder(shape=[None, self.action_size])
        advantage = K.placeholder(shape=[None, ])

        action_prob = K.sum(action * self.actor.output, axis=1)
        cross_entropy = K.log(action_prob) * advantage
        loss = -K.sum(cross_entropy)

        optimizer = Adam(lr=self.actor_lr)
        updates = optimizer.get_updates(self.actor.trainable_weights, [], loss)
        train = K.function([self.actor.input, action, advantage], [], updates=updates)

        return train

    # 가치신경망을 업데이트하는 함수
    def critic_optimizer(self):
        target = K.placeholder(shape=[None, ])

        loss = K.mean(K.square(target - self.critic.output))

        optimizer = Adam(lr=self.critic_lr)
        updates = optimizer.get_updates(self.critic.trainable_weights, [], loss)
        train = K.function([self.critic.input, target], [], updates=updates)

        return train

    # 각 타임스텝마다 정책신경망과 가치신경망을 업데이트
    def train_model(self, states, actions, rewards, next_states, dones):
        # if self.epsilon > self.epsilon_min:
        #     self.epsilon *= self.epsilon_decator

        for i in range(len(states)):
            if i % SKIPPING == 0 or dones[i]:
                value = self.critic.predict(states[i])[0]
                next_value = self.critic.predict(next_states[i])[0]

                act = np.zeros([1, self.action_size])
                act[0][actions[i]] = 1

                # 벨만 기대 방정식를 이용한 어드벤티지와 업데이트 타깃
                if dones[i]:
                    advantage = rewards[i] - value
                    target = [rewards[i]]
                else:
                    advantage = (rewards[i] + self.discount_factor * next_value) - value
                    target = rewards[i] + self.discount_factor * next_value
                self.actor_updater([states[i], act, advantage])
                self.critic_updater([states[i], target])

        # for i in range(len(states)):
        #     value = self.critic.predict(states[i])[0]
        #     next_value = self.critic.predict(next_states[i])[0]
        #
        #     act = np.zeros([1, self.action_size])
        #     act[0][actions[i]] = 1
        #
        #     # 벨만 기대 방정식를 이용한 어드벤티지와 업데이트 타깃
        #     if dones[i]:
        #         advantage = rewards[i] - value
        #         target = [rewards[i]]
        #     else:
        #         advantage = (rewards[i] + self.discount_factor * next_value) - value
        #         target = rewards[i] + self.discount_factor * next_value
        #     self.actor_updater([states[i], act, advantage])
        #     self.critic_updater([states[i], target])


if __name__ == "__main__":
    time.sleep(5)
    isReady = False

    print(str(datetime.now()) + ' started!', flush=False)
    env = Env()

    # def timer_thread():
    #     global isReady
    #     print("***** Timer thread started! ({0} sec) *****".format(ACTION_TIME))
    #     while True:
    #         isReady = True
    #         time.sleep(ACTION_TIME)
    #
    # timer_thread = threading.Thread(target=timer_thread)
    # timer_thread.daemon = True
    # timer_thread.start()

    state_size = env.state_space_shape[0]
    action_size = env.action_space_shape[0]

    episode_start_num = 0
    if os.path.exists("./save/lastest_episode_num.txt"):
        with open("./save/lastest_episode_num.txt", 'r') as f:
            episode_start_num = int(f.readline())

    # 액터-크리틱(A2C) 에이전트 생성
    agent = A2CAgent(state_size, action_size)

    best_score = -999999

    # if os.path.exists("./save/last_epsilon_value.txt"):
    #     with open("./save/last_epsilon_value.txt", 'r') as f:
    #         agent.epsilon = float(f.readline())

    scores, episodes = [], []
    try:
        for episode in range(episode_start_num, EPISODES):
            done = False
            score = 0
            step = 0
            # states, actions, next_states, rewards, dones, is_swing_up_modes = [], [], [], [], [], []
            historys, actions, next_historys, rewards, dones = [], [], [], [], []

            state = env.reset()
            # state = np.reshape(state, [1, state_size])

            history = np.stack((state, state, state, state), axis=1)
            history = np.reshape(history, (1, state_size, history_size))

            while not done:
                # if isReady:
                #     isReady = False
                step += 1

                flat_history = np.reshape(history, (1, state_size * history_size))

                # print("state:", state)
                action_index = agent.get_action(flat_history)

                next_state, reward, done, info = env.step(action_index)
                pendulum_rad = math.acos(next_state[0])
                # next_state = np.reshape(next_state, [1, state_size])

                print("|| ep: {0:4d} || step: {1:3d} || action: {2:2d} || p rad: {3:1.3f} "
                      "|| reward: {4:4.2f} || done: {5} || time: {6}"
                      .format(episode, step, action_index, pendulum_rad, reward, done,
                              datetime.utcnow().strftime('%S.%f')[:-1]))#, agent.epsilon), flush=False)

                sys.stdout.flush()

                next_state = np.reshape(next_state, (1, state_size, 1))
                next_history = np.append(next_state, history[:, :, :history_size - 1], axis=2)

                flat_next_history = np.reshape(next_history, (1, state_size * history_size))

                # states.append(state)
                historys.append(flat_history)
                actions.append(action_index)
                rewards.append(reward)
                # next_states.append(next_state)
                next_historys.append(flat_next_history)
                dones.append(done)

                score += reward
                # state = next_state
                history = next_history

                if done:
                    env.wait()
                    if step > 5:
                        agent.train_model(historys, actions, rewards, next_historys, dones)
                        scores.append(score)
                        episodes.append(episode)
                        pylab.plot(episodes, scores, 'b')
                        pylab.savefig("./save/pendulum_a2c_servo.png")

                    print(flush=False)
                    print("********** || episode: {0:4d} is Done!!! || score: {1:6.2f} "
                          "|| step: {2:3d} || info: {3} || **********"
                          .format(episode, score, step, info), flush=False)
                    print("----------------------------------------------------------------------"
                          "----------------------------------------------------------------------", flush=False)
                    print(flush=False)

                    # epsilon_file = open("./save/last_epsilon_value.txt", 'w')
                    # epsilon_file.write(str(agent.epsilon) + "\n")
                    # epsilon_file.close()

                    if episode % 50 == 0:
                        agent.actor.save_weights("./save/pendulum_actor_servo.h5")
                        agent.critic.save_weights("./save/pendulum_critic_servo.h5")
                        sys.stdout.flush()

                        f = open("./save/last_episode_num.txt", 'w')
                        f.write(str(episode) + "\n")
                        f.close()

                    if best_score < score:
                        best_score = score
                        agent.actor.save_weights("./save/pendulum_actor_servo.h5")
                        agent.critic.save_weights("./save/pendulum_critic_servo.h5")
                        sys.stdout.flush()

                        f = open("./save/last_episode_num.txt", 'w')
                        f.write(str(episode) + "\n")
                        f.close()

                    # 이전 10개 에피소드의 점수 평균이 500보다 크면 학습 중단
                    if np.mean(scores[-min(10, len(scores)):]) > 500:
                        agent.actor.save_weights("./save/pendulum_actor_servo.h5")
                        agent.critic.save_weights("./save/pendulum_critic_servo.h5")
                        sys.stdout.flush()

                        f = open("./save/last_episode_num.txt", 'w')
                        f.write(str(episode) + "\n")
                        env.close()
                        sys.exit()
                # else:
                #     time.sleep(0.0001)

        env.close()
        sys.exit()

    except KeyboardInterrupt as e:
        env.close()
        sys.exit()
