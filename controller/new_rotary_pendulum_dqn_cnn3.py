import os
import sys
import pylab
import random
import numpy as np
from datetime import datetime
from collections import deque
from keras.layers import Dense, Flatten, Conv2D
from keras.optimizers import Adam
from keras.models import Sequential
from environment_servo_agent_timer import Env
import _pickle as pickle
import time

ACTION_TIME = 0.005

EPISODES = 100000
HISTORY_LENGTH = 10
MEMORY_LENGTH = 20000

TRAIN_ITERATION_N = 100

INITIAL_EPSILON = 0.3
EPSILON_DECATOR = 0.98
EPSILON_MINIMUM = 0.001

DISCOUNT_FACTOR = 0.99
LEARNING_RATE = 0.001

BATCH_SIZE = 64

SAVED_MODEL_PATH = "./save/save_cnn/rotary_pendulum_dqn_cnn3.h5"
SAVED_MEMORY_PATH = "./save/save_cnn/dqn_memory_cnn3.pickle"
GRAPH_PATH = "./save/save_cnn/rotary_pendulum_dqn_cnn3.png"
LAST_EPISODE_N_PATH = "./save/save_cnn/last_episode_num_cnn3.txt"


# DQN 에이전트
class DQNAgent:
    def __init__(self, state_size, action_size):
        self.load_model = False

        # 상태와 행동의 크기 정의
        self.state_size = state_size
        self.action_size = action_size

        self.epsilon = INITIAL_EPSILON

        # 리플레이 메모리, 최대 크기 2000
        self.memory = deque(maxlen=MEMORY_LENGTH)

        # 모델과 타깃 모델 생성
        self.model = self.build_model()
        self.target_model = self.build_model()

        # 타깃 모델 초기화
        self.update_target_model()

        if os.path.exists(SAVED_MODEL_PATH):
            self.load_model = True

        if self.load_model:
            self.model.load_weights(SAVED_MODEL_PATH)
            self.memory_load(SAVED_MEMORY_PATH)

    def memory_dump(self, file_name):
        with open(file_name, 'wb') as memory_file:
            pickle.dump(self.memory, memory_file)

    def memory_load(self, file_name):
        with open(file_name, 'rb') as memory_file:
            self.memory = pickle.load(memory_file)

    # 상태가 입력, 큐함수가 출력인 인공신경망 생성
    def build_model(self):
        model = Sequential()
        model.add(
            Conv2D(32, kernel_size=(4, 4),
                   activation='relu',
                   input_shape=[self.state_size, HISTORY_LENGTH, 1]
            )
        )
        model.add(Conv2D(64, (1, 4), activation='relu'))
        model.add(Flatten())
        model.add(Dense(128, activation='relu'))
        model.add(Dense(self.action_size, activation='linear'))
        model.summary()
        model.compile(loss='mse', optimizer=Adam(lr=LEARNING_RATE))
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
        # 메모리에서 배치 크기만큼 무작위로 샘플 추출
        mini_batch = random.sample(self.memory, BATCH_SIZE)

        states = np.zeros(
            (BATCH_SIZE, self.state_size, HISTORY_LENGTH, 1)
        )

        next_states = np.zeros(
            (BATCH_SIZE, self.state_size, HISTORY_LENGTH, 1)
        )

        actions, rewards, dones = [], [], []

        for i in range(BATCH_SIZE):
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
        for i in range(BATCH_SIZE):
            if dones[i]:
                target[i][actions[i]] = rewards[i]
            else:
                target[i][actions[i]] = rewards[i] + DISCOUNT_FACTOR * (np.amax(target_val[i]))

        self.model.fit(states, target, batch_size=BATCH_SIZE, epochs=1, verbose=0)


if __name__ == "__main__":
    print(str(datetime.now()) + ' started!', flush=True)

    env = Env()

    state_size = env.state_space_shape[0]
    action_size = env.action_space_shape[0]

    # Reload episode number of last learning
    episode_start_num = 0
    if os.path.exists(LAST_EPISODE_N_PATH):
        with open(LAST_EPISODE_N_PATH, 'r') as f:
            episode_start_num = int(f.readline())

    # DQN 에이전트 생성
    agent = DQNAgent(state_size, action_size)

    scores, episodes = [], []

    for episode in range(episode_start_num, EPISODES):
        agent.epsilon = INITIAL_EPSILON
        done = False
        score = 0
        step = 0

        success_cnt = 0

        # env 초기화
        previous_time = time.perf_counter()

        # state = [pendulum_radian, pendulum_velocity, motor_radian, motor_velocity]
        state, theta_n_k1, theta_dot_k1, alpha_n_k1, alpha_dot_k1 = env.reset()
        state_for_manual_balance = state
        state = [state[0] * 100 , state[1], state[2] * 100, state[3]]

        state = np.reshape(state, [1, state_size, 1, 1])
        history = np.zeros([1, state_size, HISTORY_LENGTH, 1])
        for i in range(HISTORY_LENGTH):
            history = np.delete(history, 0, axis=2)
            history = np.append(history, state, axis=2)
        history = np.reshape(history, [1, state_size, HISTORY_LENGTH, 1])

        while not done:
            step += 1

            # balance using network's output
            if len(agent.memory) >= MEMORY_LENGTH - 1:
                action_index = agent.get_action(history)
            # manual balance
            else:
                if np.random.rand() <= agent.epsilon:   # random action for noise
                    action_index = random.randrange(agent.action_size)
                else:
                    kp_theta = 2.0
                    kd_theta = -2.0
                    kp_alpha = -30.0
                    kd_alpha = 2.5

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
                    motor_voltage = (theta * kp_theta) + (theta_dot * kd_theta) + (alpha * kp_alpha) + (
                                alpha_dot * kd_alpha)

                    # set the saturation limit to +/- 15V
                    if motor_voltage > 15.0:
                        motor_voltage = 15.0
                    elif motor_voltage < -15.0:
                        motor_voltage = -15.0

                    # invert for positive CCW
                    motor_voltage = -motor_voltage

                    # convert the analog value to the PWM duty cycle that will produce the same average voltage
                    motorPWM = motor_voltage * (625.0 / 15.0)
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
                current_time = time.perf_counter()
                if current_time - previous_time >= 5 / 1000:
                    break

            previous_time = time.perf_counter()
            # 선택한 행동으로 환경에서 한 타임스텝 진행
            next_state, reward, done, info = env.step(action_index)

            state_for_manual_balance = next_state
            next_state = [next_state[0] * 100, next_state[1], next_state[2] * 100, next_state[3]]

            next_state = np.reshape(next_state, (1, state_size, 1, 1))
            history = np.delete(history, 0, axis=2)
            history = np.append(history, values=next_state, axis=2)

            history = np.reshape(history, [1, state_size, HISTORY_LENGTH, 1])
            # 리플레이 메모리에 샘플 <s, a, r, s'> 저장
            if not(done and step == 1):
                # agent.append_sample(state, action_index, reward, next_state, done)
                agent.append_sample(prior_history, action_index, reward, history, done)

            score += reward

            if agent.epsilon > EPSILON_MINIMUM:
                agent.epsilon *= EPSILON_DECATOR

            if done:
                env.wait()

                if len(agent.memory) >= MEMORY_LENGTH and step > 5:
                    # Train model
                    tn = 0
                    for i in range(TRAIN_ITERATION_N):
                        agent.train_model()
                        tn += 1
                    print("train iteration num -", tn)

                    # Update target model
                    agent.update_target_model()

                    # Graph's x: episodes, y: scores
                    scores.append(score)
                    episodes.append(episode)
                    pylab.plot(episodes, scores, 'b')
                    pylab.savefig(GRAPH_PATH)

                    print("episode:{0}  score:{1}  step:{2}  info:{3}  memory length:{4}  epsilon:{5:10.8f}".format(
                        episode, score, step, info, len(agent.memory), agent.epsilon
                    ), flush=True)

                    # Save results
                    agent.model.save_weights(SAVED_MODEL_PATH)
                    agent.memory_dump(SAVED_MEMORY_PATH)
                    pylab.plot(episodes, scores, 'b')
                    pylab.savefig(GRAPH_PATH)

                    f = open(LAST_EPISODE_N_PATH, 'w')
                    f.write(str(episode) + "\n")
                    f.close()

                    sys.stdout.flush()

                    # if serial successes are persisted, system will be shutted down.
                    if step >= 12000:
                        success_cnt += 1
                    else:
                        success_cnt = 0

                    if success_cnt >= 10:
                        env.close()
                        sys.exit()
