#!/usr/bin/env python3

# Copyright (C) 2021  Shivam Pandey
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
import gym
from stable_baselines.common import make_vec_env
from stable_baselines import A2C, ACKTR
from stable_baselines.common.evaluation import evaluate_policy

from agent.policy_network import PolicyNetV1

from rl_pid.envs.cartpole_v2 import CartPoleEnvV2

if __name__ == "__main__":
    # Create and wrap the environment
    # env = make_vec_env(CartPoleEnvV2, n_envs=10)

    env = make_vec_env(CartPoleEnvV2, n_envs=10)
    # env = make_vec_env(lambda:gym.make("CartPole-v2"), n_envs=10)

    model = A2C(PolicyNetV1, env, learning_rate=1e-3, n_steps=5, verbose=1)
    # model = DQN(PolicyNetV1, env, learning_rate=1e-3, verbose=1)
    # model = ACKTR(PolicyNetV1, env, verbose=1)

    # Train the agent
    model.learn(total_timesteps=int(2e6))

    model.save("a2c_cartpole")

    del model

    model = A2C.load("a2c_cartpole")

    # Evaluate the agent
    mean_reward, std_reward = evaluate_policy(model, CartPoleEnvV2(), n_eval_episodes=100)
    print(mean_reward, std_reward)

    obs = env.reset()
    while True:
        action, _states = model.predict(obs)
        obs, rewards, dones, info = env.step(action)
        env.render()
