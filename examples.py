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

from rl_pid.envs.cartpole_v2 import CartPoleEnvV2

if __name__ == "__main__":
    env = CartPoleEnvV2()
    for i_episode in range(2000):
        observation = env.reset()
        for t in range(100):
            env.render()
            print("observation:- ", observation)
            action = env.action_space.sample()
            print("action:- ", action)
            observation, reward, done, info = env.step(action)
            if done:
                print("Episode finished after {} timesteps".format(t + 1))
                break
    env.close()
