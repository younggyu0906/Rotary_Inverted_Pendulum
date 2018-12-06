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
        self.load_model = False

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
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        else:
            q_value = self.model.predict(state)
        return np.argmax(q_value[0])

    # 샘플 <s, a, r, s'>을 리플레이 메모리에 저장
    def append_sample(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    # 리플레이 메모리에서 무작위로 추출한 배치로 모델 학습<=== pub
    def train_model(self):
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

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

        agent.epsilon = init_epsilon
        done = False
        score = 0
        step = 0

        # env 초기화
        previousTime = time.perf_counter()
        state, theta_n_k1, theta_dot_k1, alpha_n_k1, alpha_dot_k1 = env.reset()

        # state = np.reshape(state, [1, state_size])

        state_for_manual_balance = state
        state = [state[0] * 100 , state[1], state[2] * 100, state[3]]

        # sin_pen_rad = math.sin(state[0])
        # cos_pen_rad = math.cos(state[0])
        # state = [sin_pen_rad, cos_pen_rad, state[1], state[3]]

        theta_dot = (50.0 * -state[2]) - (50.0 * -state[2]) + (0.7612 * theta_dot_k1)
        _theta_dot_k1 = theta_dot
        alpha_dot = (50.0 * -state[0]) - (50.0 * -state[0]) + (0.7612 * alpha_dot_k1)
        _alpha_dot_k1 = alpha_dot

        # state = [50.0*state[0], 50.0*state[2], theta_dot, alpha_dot]
        state = np.reshape(state, [1, state_size, 1, 1])
        history = np.zeros([1, state_size, history_size, 1])
        for i in range(history_size):
            history = np.delete(history, 0, axis=2)
            history = np.append(history, state, axis=2)
        history = np.reshape(history, [1, state_size, history_size, 1])

        # for i in range(history_size - 1):
        #     for j in range(state_size):
        #         state[j].append(state[j][0])
        history = np.reshape(history, [1, state_size, history_size, 1])

        last_state = 0
        last_action_index = 1
        stop_action = False
        max_cnt = 5
        cnt = max_cnt

        while not done:
            step += 1

            # 현재 상태로 행동을 선택
            if len(agent.memory) >= MEMORY_LENGTH - 1:
                action_index = agent.get_action(history)
            else:
                if np.random.rand() <= agent.epsilon:
                    action_index = random.randrange(agent.action_size)
                else:
                    # transfer function = 50s/(s+50)
                    # z-transform at 1ms = (50z - 50)/(z-0.9512)
                    alpha = state_for_manual_balance[0]
                    theta = state_for_manual_balance[2]

                    theta_n = -theta
                    theta_dot = (50.0 * theta_n) - (50.0 * theta_n_k1) + (0.7612 * theta_dot_k1)  # 5ms
                    theta_n_k1 = theta_n
                    theta_dot_k1 = theta_dot

                    # transfer function = 50s/(s+50)
                    # z-transform at 1ms = (50z - 50)/(z-0.9512)
                    alpha_n = -alpha
                    alpha_dot = (50.0 * alpha_n) - (50.0 * alpha_n_k1) + (0.7612 * alpha_dot_k1)  # 5ms
                    alpha_n_k1 = alpha_n
                    alpha_dot_k1 = alpha_dot

                    # multiply by proportional and derivative gains
                    motorVoltage = (theta * kp_theta) + (theta_dot * kd_theta) + (alpha * kp_alpha) + (
                                alpha_dot * kd_alpha)

                    # set the saturation limit to +/- 15V
                    if motorVoltage > 15.0:
                        motorVoltage = 15.0
                    elif motorVoltage < -15.0:
                        motorVoltage = -15.0

                    # invert for positive CCW
                    motorVoltage = -motorVoltage

                    # convert the analog value to the PWM duty cycle that will produce the same average voltage
                    motorPWM = motorVoltage * (625.0 / 15.0)

                    motorPWM = int(motorPWM)

                    motorPWM += -(motorPWM % 40)

                    if motorPWM < 0:
                        action_index = 0
                    elif 0 < motorPWM:
                        action_index = 2
                    else:
                        action_index = 1

            prior_history = history
            while True:
                currentTime = time.perf_counter()
                if currentTime - previousTime >= 5 / 1000:
                    break

            previousTime = time.perf_counter()
            # 선택한 행동으로 환경에서 한 타임스텝 진행
            next_state, reward, done, info = env.step(action_index)
            # print(next_state, reward)

            _theta_dot = (50.0 * -next_state[2]) - (50.0 * -next_state[2]) + (0.7612 * _theta_dot_k1)
            _theta_dot_k1 = _theta_dot
            _alpha_dot = (50.0 * -next_state[0]) - (50.0 * -next_state[0]) + (0.7612 * _alpha_dot_k1)
            _alpha_dot_k1 = _alpha_dot

            state_for_manual_balance = next_state
            next_state = [next_state[0] * 100, next_state[1], next_state[2] * 100, next_state[3]]
            # next_state = [50.0 * next_state[0], 50.0 * next_state[2], _theta_dot, _alpha_dot]
            # state = np.reshape(state, [1, state_size, 1, 1])

            next_state = np.reshape(next_state, (1, state_size, 1, 1))
            history = np.delete(history, 0, axis=2)
            history = np.append(history, values=next_state, axis=2)

            history = np.reshape(history, [1, state_size, history_size, 1])
            # 리플레이 메모리에 샘플 <s, a, r, s'> 저장
            if not(done and step == 1):
                # agent.append_sample(state, action_index, reward, next_state, done)
                agent.append_sample(prior_history, action_index, reward, history, done)

            score += reward

            if agent.epsilon > agent.epsilon_min:
                agent.epsilon *= agent.epsilon_decay

            if done:
                env.wait()

                # if len(agent.memory) >= MEMORY_LENGTH / 5 and step > 5:#agent.train_start:
                if len(agent.memory) >= MEMORY_LENGTH:
                    tn = 0
                    for i in range(train_iteration_num):
                        agent.train_model()
                        tn += 1
                    print("train iteration num -", tn)

                # if episode % 4 == 0:
                #     # 각 에피소드마다 타깃 모델을 모델의 가중치로 업데이트
                agent.update_target_model()

                if len(agent.memory) >= MEMORY_LENGTH and step > 5:
                    # 에피소드마다 학습 결과 출력
                    scores.append(score)
                    episodes.append(episode)
                    pylab.plot(episodes, scores, 'b')
                    pylab.savefig("./save/save_cnn/rotary_pendulum_dqn_cnn.png")


                # print()
                print("episode:{0}  score:{1}  step:{2}  info:{3}  memory length:{4}  epsilon:{5:10.8f}".format(
                    episode, score, step, info, len(agent.memory), agent.epsilon
                ), flush=True)

                agent.model.save_weights("./save/save_cnn/rotary_pendulum_dqn_cnn.h5")
                sys.stdout.flush()

                f = open("./save/save_cnn/last_episode_num_cnn.txt", 'w')
                f.write(str(episode) + "\n")
                f.close()

                agent.memory_dump("./save/save_cnn/dqn_memory_cnn.pickle")

                if len(agent.memory) >= MEMORY_LENGTH - 1 and best_score < score:
                    best_score = score

                    agent.model.save_weights("./save/save_cnn/rotary_pendulum_dqn_cnn.h5")
                    sys.stdout.flush()

                    f = open("./save/save_cnn/last_episode_num_cnn.txt", 'w')
                    f.write(str(episode) + "\n")
                    f.close()

                    pylab.plot(episodes, scores, 'b')
                    pylab.savefig("./save/save_cnn/rotary_pendulum_dqn_cnn.png")

                    agent.memory_dump("./save/save_cnn/dqn_memory_cnn.pickle")

                if len(scores) > 30 and np.mean(scores[-min(5, len(scores)):]) > 7000:
                    best_score = score

                    agent.model.save_weights("./save/save_cnn/rotary_pendulum_dqn_cnn.h5")
                    sys.stdout.flush()

                    f = open("./save/save_cnn/last_episode_num_cnn.txt", 'w')
                    f.write(str(episode) + "\n")
                    f.close()

                    pylab.plot(episodes, scores, 'b')
                    pylab.savefig("./save/save_cnn/rotary_pendulum_dqn_cnn.png")

                    agent.memory_dump("./save/save_cnn/dqn_memory_cnn.pickle")
                    env.close()
                    sys.exit()
