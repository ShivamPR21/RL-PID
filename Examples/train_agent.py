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

from stable_baselines.common import make_vec_env
from stable_baselines import A2C

from agent.policy_network import PolicyNetV1

from rl_pid.envs.cartpole_v2 import CartPoleEnvV2

if __name__ == "__main__":
    # Create and wrap the environment
    env = make_vec_env(CartPoleEnvV2, n_envs=4)

    model = A2C(PolicyNetV1, env, verbose=1)
    # Train the agent
    model.learn(total_timesteps=100000)
