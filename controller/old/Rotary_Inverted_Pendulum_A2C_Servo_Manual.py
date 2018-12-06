from Environment_Servo_Manual import Env
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

EPISODES = 1000000


class A2CAgent:
    def __init__(self, state_size, action_size):
        self.load_model = False
        if os.path.exists("./save/pendulum_actor_servo_manual.h5"):
            self.load_model = True

        # 상태와 행동의 크기 정의
        self.state_size = state_size
        self.action_size = action_size
        self.value_size = 1

        # 액터-크리틱 하이퍼파라미터
        self.discount_factor = 0.99
        self.actor_lr = 0.001
        self.critic_lr = 0.005

        # 정책신경망과 가치신경망 생성
        self.actor = self.build_actor()
        self.critic = self.build_critic()
        self.actor_updater = self.actor_optimizer()
        self.critic_updater = self.critic_optimizer()

        if self.load_model:
            self.actor.load_weights("./save/pendulum_actor_servo_manual.h5")
            self.critic.load_weights("./save/pendulum_critic_servo_manual.h5")

    # actor: 상태를 받아 각 행동의 확률을 계산
    def build_actor(self):
        actor = Sequential()
        actor.add(Dense(24, input_dim=self.state_size, activation='relu', kernel_initializer='he_uniform'))
        actor.add(Dense(self.action_size, activation='softmax', kernel_initializer='he_uniform'))
        actor.summary()
        return actor

    # critic: 상태를 받아서 상태의 가치를 계산
    def build_critic(self):
        critic = Sequential()
        critic.add(Dense(24, input_dim=self.state_size, activation='relu', kernel_initializer='he_uniform'))
        critic.add(Dense(24, activation='relu', kernel_initializer='he_uniform'))
        critic.add(Dense(self.value_size, activation='linear', kernel_initializer='he_uniform'))
        critic.summary()
        return critic

    # 정책신경망의 출력을 받아 확률적으로 행동을 선택
    def get_action(self, state):
        policy = self.actor.predict(state, batch_size=1).flatten()
        action = np.random.choice(self.action_size, 1, p=policy)[0]
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
        for i in range(len(states)):
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


if __name__ == "__main__":
    print("\n***** The learning started at {0} *****".format(datetime.now()))
    env = Env()

    # 환경으로부터 상태와 행동의 크기를 받아옴
    state_size = env.state_space_shape[0]
    action_size = env.action_space_shape[0]

    # 액터-크리틱(A2C) 에이전트 생성
    agent = A2CAgent(state_size, action_size)

    episode_start_num = 0
    if os.path.exists("./save/latest_episode_num.txt"):
        with open("./save/latest_episode_num.txt", 'r') as f:
            episode_start_num = int(f.readline())

    scores, episodes = [], []
    states, actions, next_states, rewards, dones = [], [], [], [], []
    try:
        for episode in range(episode_start_num, EPISODES):
            done = False
            score = 0
            step = 0
            states.clear(), actions.clear(), next_states.clear(), rewards.clear(), dones.clear()

            state = env.reset()
            state = np.reshape(state, [1, state_size])

            while not done:
                step += 1

                action_index = agent.get_action(state)

                next_state, reward, done, info = env.step(action_index)
                pendulum_rad = math.acos(next_state[3])
                next_state = np.reshape(next_state, [1, state_size])

                print("|| e: {0:4d} || step: {1:3d} || action index: {2:2d} || P rad: {3:1.5f} "
                      "|| reward: {4:.5f} || done: {5} || time: {6} ||".format(
                        episode, step, action_index, pendulum_rad,
                        reward, done, datetime.utcnow().strftime('%S.%f')[:-1]
                        ), flush=True)

                states.append(state)
                actions.append(action_index)
                rewards.append(reward)
                next_states.append(next_state)
                dones.append(done)

                score += reward
                state = next_state

                if done:
                    if step >= 50:
                        agent.train_model(states, actions, rewards, next_states, dones)
                        scores.append(score)
                        episodes.append(episode)
                        pylab.plot(episodes, scores, 'b')
                        pylab.savefig("./save/pendulum_a2c_servo_manual.png")

                    print("\n********** || episode: {0:4d} is Done!!! || score: {1:6.5f} "
                          "|| step: {2:3d} || info: {3} || **********\n"
                          .format(episode, score, step, info), flush=True)
                    print("---------------------------------------------------------------"
                          "---------------------------------------------------------------\n")

                    if episode != 0 and episode % 50 == 0:
                        agent.actor.save_weights("./save/pendulum_actor_servo_manual.h5")
                        agent.critic.save_weights("./save/pendulum_critic_servo_manual.h5")

                        f = open("./save/latest_episode_num.txt", 'w')
                        f.write(str(episode) + "\n")
                        f.close()

                    # 이전 10개 에피소드의 점수 평균이 -30보다 크면 학습 중단
                    # if np.mean(scores[-min(10, len(scores)):]) > -30:
                    #     agent.actor.save_weights("./save/pendulum_actor.h5")
                    #     agent.critic.save_weights("./save/pendulum_critic.h5")
                    #     env.close()
                    #     sys.exit()

        env.close()
        sys.exit()

    except KeyboardInterrupt:
        print("\nMain thread KeyboardInterrupted")
        env.close()
        sys.exit()
