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
import threading

EPISODES = 1000
history_size = 4
MEMORY_LENGTH = 20000
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
        self.epsilon = 0.1
        self.epsilon_decay = 0.9
        self.epsilon_min = 0.00001
        self.batch_size = 4096 #64
        self.train_start = 1000

        # 리플레이 메모리, 최대 크기 2000
        self.memory = deque(maxlen=MEMORY_LENGTH)

        # 모델과 타깃 모델 생성
        self.model = self.build_model()
        self.target_model = self.build_model()

        # 타깃 모델 초기화
        self.update_target_model()

        if os.path.exists("./save/rotary_pendulum_dqn.h5") or \
                os.path.exists("./save/rotary_pendulum_dqn.h5"):
            self.load_model = True
            self.memory_load("./save/dqn_memory.pickle")

        if self.load_model:
            self.model.load_weights("./save/rotary_pendulum_dqn.h5")

    def memory_dump(self, file_name):
        with open(file_name, 'wb') as memory_file:
            pickle.dump(self.memory, memory_file)

    def memory_load(self, file_name):
        with open(file_name, 'rb') as memory_file:
            self.memory = pickle.load(memory_file)

    # 상태가 입력, 큐함수가 출력인 인공신경망 생성
    def build_model(self):
        model = Sequential()
        model.add(Dense(48, input_dim=self.state_size, activation='linear', kernel_initializer='he_uniform'))
        model.add(LeakyReLU(alpha=0.3))
        model.add(Dense(48, activation='linear', kernel_initializer='he_uniform'))
        model.add(LeakyReLU(alpha=0.3))
        model.add(Dense(24, activation='linear', kernel_initializer='he_uniform'))
        model.add(LeakyReLU(alpha=0.3))
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
        # policy = self.model.predict(state)[0]
        # e_x = np.exp(policy - np.max(policy))
        # softmax = e_x / e_x.sum()
        # action = np.random.choice(self.action_size, 1, p=softmax)[0]
        # return action

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

    def timer_thread():
        global isReady
        print("***** Timer thread started! ({0} sec) *****".format(ACTION_TIME))
        while True:
            isReady = True
            time.sleep(ACTION_TIME)

    timer_thread = threading.Thread(target=timer_thread)
    timer_thread.daemon = True
    timer_thread.start()

    env = Env()

    state_size = env.state_space_shape[0] -1
    action_size = env.action_space_shape[0]

    episode_start_num = 0
    if os.path.exists("./save/last_episode_num.txt"):
        with open("./save/last_episode_num.txt", 'r') as f:
            episode_start_num = int(f.readline())

    # DQN 에이전트 생성
    agent = DQNAgent(state_size, action_size)

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
        state, theta_n_k1, theta_dot_k1, alpha_n_k1, alpha_dot_k1 = env.reset()
        state_for_mr = state
        state = state[:3]
        state = np.reshape(state, [1, state_size])

        # history = np.stack((state, state, state, state), axis=1)
        # history = np.reshape(history, (1, state_size, history_size))
        last_state = 0
        last_action_index = 1
        stop_action = False
        cnt = 5

        while not done:
            if isReady:
                isReady = False
                step += 1

                # 현재 상태로 행동을 선택
                # flat_history = np.reshape(history, (1, state_size * history_size))
                if len(agent.memory) >= MEMORY_LENGTH - 1:
                    action_index = agent.get_action(state)
                    print(action_index, end=" ")
                else:
                    # transfer function = 50s/(s+50)
                    # z-transform at 1ms = (50z - 50)/(z-0.9512)
                    theta = state_for_mr[3]
                    alpha = state_for_mr[0] / 100
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

                    # if motorPWM == 0:
                    #     action_index = 3
                    # elif motorPWM <= -120:
                    #     action_index = 0
                    # elif -120 < motorPWM <= -80:
                    #     action_index = 1
                    # elif -80 < motorPWM <= -40:
                    #     action_index = 2
                    # elif 120 <= motorPWM:
                    #     action_index = 6
                    # elif 80 <= motorPWM < 120:
                    #     action_index = 5
                    # else:
                    #     action_index = 4
                    # print(motorVoltage, motorPWM, "theta:", theta, "alpha:", alpha, motorPWM, state[0][0], action_index)

                # else:
                #     if -0.4 < state[0][0] < 0.4 or cnt == 0:
                #         action_index = 1
                #         cnt = 5
                #     else:
                #         if state[0][0] < last_state and cnt > 0:
                #             action_index = 0
                #             cnt -= 1
                #         elif state[0][0] > last_state and cnt > 0:
                #             action_index = 2
                #             cnt -= 1
                #         else:
                #             action_index = last_action_index
                #             cnt = 5
                if step == 1:
                    action_index = 2

                last_action_index = action_index
                last_state = state[0][0]
                # print(state[0], action_index, cnt)

                # 선택한 행동으로 환경에서 한 타임스텝 진행
                next_state, reward, done, info = env.step(action_index)
                state_for_mr = next_state
                next_state = next_state[:3]

                # print("episode:{0}  step:{1}  action: {2}  reward:{3:.2f}  done:{4}  epsilon:{5:2f}  time:{6}"
                #       .format(episode, step, action_index, reward, done, agent.epsilon,
                #               datetime.utcnow().strftime('%S.%f')[:-1]), flush=False)

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

                if len(agent.memory) >= MEMORY_LENGTH-1 and agent.epsilon > agent.epsilon_min:
                    agent.epsilon *= agent.epsilon_decay

                if done:
                    env.wait()

                    if len(agent.memory) >= MEMORY_LENGTH/2 and step > 5:#agent.train_start:
                        agent.train_model()

                    if episode % 4 == 0:
                        # 각 에피소드마다 타깃 모델을 모델의 가중치로 업데이트
                        agent.update_target_model()

                    if step > 5:
                        # 에피소드마다 학습 결과 출력
                        scores.append(score)
                        episodes.append(episode)
                        pylab.plot(episodes, scores, 'b')
                        pylab.savefig("./save/rotary_pendulum_dqn.png")


                    print()
                    print("episode:{0}  score:{1}  step:{2}  info:{3}  memory length:{4}  epsilon:{5:10.8f}".format(
                        episode, score, step, info, len(agent.memory), agent.epsilon
                    ), flush=True)

                    agent.model.save_weights("./save/rotary_pendulum_dqn.h5")
                    sys.stdout.flush()

                    f = open("./save/last_episode_num.txt", 'w')
                    f.write(str(episode) + "\n")
                    f.close()

                    agent.memory_dump("./save/dqn_memory.pickle")

                    if len(agent.memory) >= MEMORY_LENGTH - 1 and best_score < score:
                        best_score = score

                        agent.model.save_weights("./save/rotary_pendulum_dqn_good.h5")
                        sys.stdout.flush()

                        f = open("./save/last_episode_num_good.txt", 'w')
                        f.write(str(episode) + "\n")
                        f.close()

                        pylab.plot(episodes, scores, 'b')
                        pylab.savefig("./save/rotary_pendulum_dqn_good.png")

                        agent.memory_dump("./save/dqn_memory_good.pickle")


                    # if episode % 50 == 0:
                    #     agent.model.save_weights("./save/rotary_pendulum_dqn.h5")
                    #     sys.stdout.flush()
                    #
                    #     f = open("./save/last_episode_num.txt", 'w')
                    #     f.write(str(episode) + "\n")
                    #     f.close()
                    #
                    #     agent.memory_dump("./save/dqn_memory.pickle")
                    #
                    # if best_score < score:
                    #     best_score = score
                    #     agent.model.save_weights("./save/rotary_pendulum_dqn.h5")
                    #     sys.stdout.flush()
                    #
                    #     f = open("./save/last_episode_num.txt", 'w')
                    #     f.write(str(episode) + "\n")
                    #     f.close()
                    #
                    #     agent.memory_dump("./save/dqn_memory.pickle")

                    # # 이전 10개 에피소드의 점수 평균이 500보다 크면 학습 중단
                    # if np.mean(scores[-min(10, len(scores)):]) > 500:
                    #     agent.model.save_weights("./save/rotary_pendulum_dqn.h5")
                    #     sys.stdout.flush()
                    #
                    #     f = open("./save/last_episode_num.txt", 'w')
                    #     f.write(str(episode) + "\n")
                    #     env.close()
                    #     sys.exit()
            else:
                time.sleep(0.0001)
