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
from environment_servo_agent_timer import Env
import _pickle as pickle
from keras.layers import LeakyReLU
import time
import math
import threading

EPISODES = 100
history_size = 2
ACTION_TIME = 0.005

# DQN 에이전트
class DQNAgent:
    def __init__(self, state_size, action_size):
        self.load_model = True

        # 상태와 행동의 크기 정의
        self.state_size = state_size * history_size
        self.action_size = action_size

        self.learning_rate = 0.00025

        # 모델과 타깃 모델 생성
        self.model = self.build_model()

        if os.path.exists("./save/rotary_pendulum_dqn.h5") or \
                os.path.exists("./save/rotary_pendulum_dqn.h5"):
            self.load_model = True
            self.memory_load("./save/dqn_memory.pickle")

        if self.load_model:
            self.model.load_weights("./save/rotary_pendulum_dqn.h5")

    # 상태가 입력, 큐함수가 출력인 인공신경망 생성
    def build_model(self):
        model = Sequential()
        model.add(Dense(48, input_dim=self.state_size, activation='relu', kernel_initializer='he_uniform'))
        # model.add(LeakyReLU(alpha=0.3))
        model.add(Dense(48, activation='relu', kernel_initializer='he_uniform'))
        # model.add(LeakyReLU(alpha=0.3))
        model.add(Dense(48, activation='relu', kernel_initializer='he_uniform'))
        # model.add(LeakyReLU(alpha=0.3))
        model.add(Dense(self.action_size, activation='linear', kernel_initializer='he_uniform'))
        model.summary()
        model.compile(loss='mse', optimizer=Adam(lr=self.learning_rate))
        return model

    # 입실론 탐욕 정책으로 행동 선택
    def get_action(self, state):
        q_value = self.model.predict(state)
        return np.argmax(q_value[0])


if __name__ == "__main__":
    print(str(datetime.now()) + ' started!', flush=True)

    env = Env()

    state_size = env.state_space_shape[0] - 2
    action_size = env.action_space_shape[0]

    # DQN 에이전트 생성
    agent = DQNAgent(state_size, action_size)

    scores, episodes = [], []
    best_score = -999999

    for episode in range(0, EPISODES):
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

        # state = np.reshape(state, [1, state_size])

        state_for_manual_balance = state

        # sin_pen_rad = math.sin(state[0])
        # cos_pen_rad = math.cos(state[0])
        # state = [sin_pen_rad, cos_pen_rad, state[1], state[3]]
        state = [50.0*state[0], 50.0*state[2]]
        # state = np.reshape(state, [1, state_size])
        # print(state)

        history = np.stack((state, state), axis=1)
        history = np.reshape(history, (1, state_size, history_size))
        last_state = 0
        last_action_index = 1
        stop_action = False
        max_cnt = 5
        cnt = max_cnt

        while not done:
            step += 1

            # 현재 상태로 행동을 선택
            flat_history = np.reshape(history, (1, state_size * history_size))
            action_index = agent.get_action(flat_history)

            # 선택한 행동으로 환경에서 한 타임스텝 진행
            while True:
                currentTime = time.perf_counter()
                if currentTime - previousTime >= 5 / 1000:
                    break

            previousTime = time.perf_counter()
            next_state, reward, done, info = env.step(action_index)

            state_for_manual_balance = next_state

            next_state = [50.0*next_state[0], 50.0*next_state[2]]

            next_state = np.reshape(next_state, (1, state_size, 1))
            next_history = np.append(next_state, history[:, :, :history_size - 1], axis=2)
            flat_next_history = np.reshape(next_history, (1, state_size * history_size))

            score += reward
            history = next_history
            # state = next_state

            if done:
                env.wait()

                if step > 5:
                    # 에피소드마다 학습 결과 출력
                    scores.append(score)
                    episodes.append(episode)
                    pylab.plot(episodes, scores, 'b')
                    pylab.savefig("./save/rotary_pendulum_dqn.png")

                # print()
                print("episode:{0}  score:{1}  step:{2}  info:{3}".format(
                    episode, score, step, info
                ), flush=True)

                agent.model.save_weights("./save/rotary_pendulum_dqn_good.h5")
                sys.stdout.flush()

                f = open("./save/last_episode_num_good.txt", 'w')
                f.write(str(episode) + "\n")
                f.close()

                pylab.plot(episodes, scores, 'b')
                pylab.savefig("./save/rotary_pendulum_dqn_good.png")

