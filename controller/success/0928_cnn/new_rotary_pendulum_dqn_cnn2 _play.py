import os
import sys
import math
import pylab
import random
import numpy as np
from datetime import datetime
from collections import deque
from keras.layers import Dense, Dropout, Flatten, Conv2D, MaxPooling2D
from keras.optimizers import Adam
from keras.models import Sequential
from environment_servo_agent_timer import Env
import _pickle as pickle
from keras.layers import LeakyReLU
import time
import math
import threading

EPISODES = 100000
history_size = 10
MEMORY_LENGTH = 20000
init_epsilon = 0.3
ACTION_TIME = 0.005
train_iteration_num = 100

# DQN 에이전트
class DQNAgent:
    def __init__(self, state_shape, action_size):
        self.load_model = True

        # 상태와 행동의 크기 정의
        self.state_shape = state_shape
        self.action_size = action_size

        # DQN 하이퍼파라미터
        self.discount_factor = 0.99
        self.learning_rate = 0.001
        self.epsilon = 0.3
        self.epsilon_decay = 0.98
        self.epsilon_min = 0.01
        self.batch_size = 64 #64
        # self.train_start = 1000

        # 리플레이 메모리, 최대 크기 2000
        self.memory = deque(maxlen=MEMORY_LENGTH)

        # 모델과 타깃 모델 생성
        self.model = self.build_model()
        self.target_model = self.build_model()

        # 타깃 모델 초기화
        self.update_target_model()

        if os.path.exists("./save/save_cnn/rotary_pendulum_dqn_cnn.h5"):
            self.load_model = True
            self.memory_load("./save/save_cnn/dqn_memory_cnn.pickle")

        if self.load_model:
            self.model.load_weights("./save/save_cnn/rotary_pendulum_dqn_cnn.h5")

    def memory_dump(self, file_name):
        with open(file_name, 'wb') as memory_file:
            pickle.dump(self.memory, memory_file)

    def memory_load(self, file_name):
        with open(file_name, 'rb') as memory_file:
            self.memory = pickle.load(memory_file)

    # 상태가 입력, 큐함수가 출력인 인공신경망 생성
    def build_model(self):
        model = Sequential()
        model.add(Conv2D(32, kernel_size=(4, 4),
                         activation='relu',
                         input_shape=self.state_shape))
        model.add(Conv2D(64, (1, 4), activation='relu'))
        model.add(Flatten())
        model.add(Dense(128, activation='relu'))
        model.add(Dense(self.action_size, activation='linear'))
        model.summary()
        model.compile(loss='mse', optimizer=Adam(lr=self.learning_rate))
        return model

    # 타깃 모델을 모델의 가중치로 업데이트
    def update_target_model(self):
        self.target_model.set_weights(self.model.get_weights())

    # 입실론 탐욕 정책으로 행동 선택
    def get_action(self, state):
        q_value = self.model.predict(state)
        return np.argmax(q_value[0])

    # 샘플 <s, a, r, s'>을 리플레이 메모리에 저장
    def append_sample(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    # 리플레이 메모리에서 무작위로 추출한 배치로 모델 학습<=== pub
    def train_model(self):
        # 메모리에서 배치 크기만큼 무작위로 샘플 추출
        mini_batch = random.sample(self.memory, self.batch_size)

        states = np.zeros(
            (self.batch_size, self.state_shape[0], self.state_shape[1], self.state_shape[2])
        )

        next_states = np.zeros(
            (self.batch_size, self.state_shape[0], self.state_shape[1], self.state_shape[2])
        )

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
    state_shape = [state_size, history_size, 1]
    action_size = env.action_space_shape[0]

    episode_start_num = 0
    if os.path.exists("./save/save_cnn/last_episode_num_cnn.txt"):
        with open("./save/save_cnn/last_episode_num_cnn.txt", 'r') as f:
            episode_start_num = int(f.readline())

    # DQN 에이전트 생성
    agent = DQNAgent(state_shape, action_size)

    scores, episodes = [], []
    best_score = -999999

    for episode in range(episode_start_num, EPISODES):
        kp_theta = 2.0
        kd_theta = -2.0
        kp_alpha = -30.0
        kd_alpha = 2.5

        done = False
        score = 0
        step = 0

        # env 초기화
        previousTime = time.perf_counter()
        state, theta_n_k1, theta_dot_k1, alpha_n_k1, alpha_dot_k1 = env.reset()

        state = [state[0] * 100, state[1], state[2] * 100, state[3]]

        state = np.reshape(state, [1, state_size, 1, 1])
        history = np.zeros([1, state_size, history_size, 1])
        for i in range(history_size):
            history = np.delete(history, 0, axis=2)
            history = np.append(history, state, axis=2)
        history = np.reshape(history, [1, state_size, history_size, 1])

        while not done:
            step += 1

            # 현재 상태로 행동을 선택
            action_index = agent.get_action(history)

            prior_history = history
            while True:
                currentTime = time.perf_counter()
                if currentTime - previousTime >= 5 / 1000:
                    break

            previousTime = time.perf_counter()
            # 선택한 행동으로 환경에서 한 타임스텝 진행
            next_state, reward, done, info = env.step(action_index)

            next_state = [next_state[0] * 100, next_state[1], next_state[2] * 100, next_state[3]]

            next_state = np.reshape(next_state, (1, state_size, 1, 1))
            history = np.delete(history, 0, axis=2)
            history = np.append(history, values=next_state, axis=2)

            history = np.reshape(history, [1, state_size, history_size, 1])

            score += reward

            if done:
                env.wait()

                # 에피소드마다 학습 결과 출력
                scores.append(score)
                episodes.append(episode)
                pylab.plot(episodes, scores, 'b')
                pylab.savefig("./save/save_cnn/rotary_pendulum_dqn_cnn_play.png")

                # print()
                print("episode:{0}  score:{1}  step:{2}  info:{3}".format(
                    episode, score, step, info
                ), flush=True)
