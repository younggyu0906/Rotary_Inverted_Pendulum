import os
import sys
import math
import pylab
import random
import numpy as np
from datetime import datetime
from collections import deque
from keras.layers import Dense
from keras.optimizers import Adam
from keras.models import Sequential
from environment_servo_2 import Env
import _pickle as pickle
from keras.layers import LeakyReLU
import time
import threading

EPISODES = 1000
history_size = 4
MEMORY_LENGTH = 5000
init_epsilon = 0.1
ACTION_TIME = 0.005

# DQN 에이전트
class DQNAgent:
    def __init__(self, state_size, action_size):
        self.load_model = False

        # 상태와 행동의 크기 정의
        self.state_size = state_size #  * history_size
        self.action_size = action_size

        # DQN 하이퍼파라미터
        self.discount_factor = 0.99
        self.learning_rate = 0.00025
        self.epsilon = 1.0
        self.epsilon_decay = 0.9995
        self.epsilon_min = 0.001
        self.batch_size = 128
        self.train_start = 2500

        # 리플레이 메모리, 최대 크기 2000
        self.memory = deque(maxlen=MEMORY_LENGTH)

        # 모델과 타깃 모델 생성
        self.model = self.build_model()
        self.target_model = self.build_model()

        # 타깃 모델 초기화
        self.update_target_model()

        if os.path.exists("./save/rotary_pendulum_dqn_2.h5") or \
                os.path.exists("./save/rotary_pendulum_dqn_2.h5"):
            self.load_model = True
            self.memory_load("./save/dqn_memory_2.pickle")

        if self.load_model:
            self.model.load_weights("./save/rotary_pendulum_dqn_2.h5")

    def memory_dump(self, file_name):
        with open(file_name, 'wb') as memory_file:
            pickle.dump(self.memory, memory_file)

    def memory_load(self, file_name):
        with open(file_name, 'rb') as memory_file:
            self.memory = pickle.load(memory_file)

    # 상태가 입력, 큐함수가 출력인 인공신경망 생성
    def build_model(self):
        model = Sequential()
        model.add(Dense(48, input_dim=self.state_size, activation='relu', kernel_initializer='he_uniform'))
        model.add(Dense(48, activation='relu', kernel_initializer='he_uniform'))
        model.add(Dense(24, activation='relu', kernel_initializer='he_uniform'))
        model.add(Dense(self.action_size, activation='linear', kernel_initializer='he_uniform'))
        model.summary()
        model.compile(loss='mse', optimizer=Adam(lr=self.learning_rate))
        return model

    # 타깃 모델을 모델의 가중치로 업데이트
    def update_target_model(self):
        self.target_model.set_weights(self.model.get_weights())

    # 입실론 탐욕 정책으로 행동 선택
    def get_action(self, state):
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        else:
            q_value = self.model.predict(state)
        return np.argmax(q_value[0])

    # 샘플 <s, a, r, s'>을 리플레이 메모리에 저장
    def append_sample(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    # 리플레이 메모리에서 무작위로 추출한 배치로 모델 학습
    def train_model(self):
        # 메모리에서 배치 크기만큼 무작위로 샘플 추출
        mini_batch = random.sample(self.memory, self.batch_size)

        states = np.zeros((self.batch_size, self.state_size))
        next_states = np.zeros((self.batch_size, self.state_size))
        actions, rewards, dones = [], [], []

        for i in range(self.batch_size):
            states[i] = mini_batch[i][0]
            actions.append(mini_batch[i][1])
            rewards.append(mini_batch[i][2])
            next_states[i] = mini_batch[i][3]
            dones.append(mini_batch[i][4])

        # 현재 상태에 대한 모델의 큐함수
        # 다음 상태에 대한 타깃 모델의 큐함수
        target = self.model.predict(states)
        target_val = self.target_model.predict(next_states)

        # 벨만 최적 방정식을 이용한 업데이트 타깃
        for i in range(self.batch_size):
            if dones[i]:
                target[i][actions[i]] = rewards[i]
            else:
                target[i][actions[i]] = rewards[i] + self.discount_factor * (np.amax(target_val[i]))

        self.model.fit(states, target, batch_size=self.batch_size, epochs=1, verbose=0)


if __name__ == "__main__":
    print(str(datetime.now()) + ' started!', flush=True)

    env = Env()

    state_size = env.state_space_shape[0]
    action_size = env.action_space_shape[0]

    episode_start_num = 0
    if os.path.exists("./save/last_episode_num_2.txt"):
        with open("./save/last_episode_num_2.txt", 'r') as f:
            episode_start_num = int(f.readline())

    # DQN 에이전트 생성
    agent = DQNAgent(state_size, action_size)

    scores, episodes = [], []
    best_score = -999999

    for episode in range(episode_start_num, EPISODES):
        done = False
        score = 0
        step = 0

        # env 초기화
        state = env.reset()
        state = np.reshape(state, [1, state_size])

        # history = np.stack((state, state, state, state), axis=1)
        # history = np.reshape(history, (1, state_size, history_size))
        last_state = 0
        last_action_index = 1
        stop_action = False
        cnt = 5

        while not done:
            step += 1

            # 현재 상태로 행동을 선택
            # flat_history = np.reshape(history, (1, state_size * history_size))

            action_index = agent.get_action(state)

            # 선택한 행동으로 환경에서 한 타임스텝 진행
            next_state, reward, done, info = env.step(action_index)

            next_state = np.reshape(next_state, [1, state_size])
            # next_state = np.reshape(next_state, (1, state_size, 1))
            # next_history = np.append(next_state, history[:, :, :history_size - 1], axis=2)
            # flat_next_history = np.reshape(next_history, (1, state_size * history_size))

            # 리플레이 메모리에 샘플 <s, a, r, s'> 저장
            if not done and not step == 1:
                agent.append_sample(state, action_index, reward, next_state, done)

            score += reward
            # history = next_history
            state = next_state

            if len(agent.memory) >= agent.train_start and agent.epsilon > agent.epsilon_min:
                agent.epsilon *= agent.epsilon_decay

            if done:
                env.wait()

                if len(agent.memory) >= agent.train_start and step > 5:
                    for _ in range(step):
                        agent.train_model()

                if episode % 4 == 0:
                    # 각 에피소드마다 타깃 모델을 모델의 가중치로 업데이트
                    agent.update_target_model()

                if step > 5:
                    # 에피소드마다 학습 결과 출력
                    scores.append(score)
                    episodes.append(episode)
                    pylab.plot(episodes, scores, 'b')
                    pylab.savefig("./save/rotary_pendulum_dqn_2.png")

                print("episode:{0}  score:{1}  step:{2}  info:{3}  memory length:{4}  epsilon:{5:10.8f}".format(
                    episode, score, step, info, len(agent.memory), agent.epsilon
                ), flush=True)

                agent.model.save_weights("./save/rotary_pendulum_dqn_2.h5")
                sys.stdout.flush()

                f = open("./save/last_episode_num_2.txt", 'w')
                f.write(str(episode) + "\n")
                f.close()

                agent.memory_dump("./save/dqn_memory_2.pickle")

                if len(agent.memory) >= MEMORY_LENGTH - 1 and best_score < score:
                    best_score = score

                    agent.model.save_weights("./save/rotary_pendulum_dqn_good_2.h5")
                    sys.stdout.flush()

                    f = open("./save/last_episode_num_good_2.txt", 'w')
                    f.write(str(episode) + "\n")
                    f.close()

                    pylab.plot(episodes, scores, 'b')
                    pylab.savefig("./save/rotary_pendulum_dqn_good_2.png")

                    agent.memory_dump("./save/dqn_memory_good_2.pickle")
