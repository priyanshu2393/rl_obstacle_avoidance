# Reinforcement Learning Obstacle Avoidance

This README provides a comprehensive overview of the Reinforcement Learning (RL) Obstacle Avoidance System implemented in this repository.

## Table of Contents

- [Introduction](#introduction)
- [TD3 Algorithm Implementation](#td3-algorithm-implementation)
- [Environment Setup](#environment-setup)
- [Training Procedures](#training-procedures)
- [Testing Procedures](#testing-procedures)
- [Contributing](#contributing)
- [License](#license)

## Introduction

The Reinforcement Learning Obstacle Avoidance System is designed to navigate an agent through an environment filled with obstacles using the Twin Delayed Deep Deterministic Policy Gradient (TD3) algorithm. 

## TD3 Algorithm Implementation

Twin Delayed DDPG (TD3) is an off-policy actor-critic algorithm. It addresses the shortcomings of the original DDPG by using two main strategies:

1. **Twin Q-networks**: To reduce overestimation bias, TD3 maintains two Q-value functions and updates the main network only if both networks agree.
2. **Policy delay**: The policy is updated less frequently than the Q-value functions to stabilize learning.

This implementation can be found in the `td3.py` file within the `RL` directory.

## Environment Setup

To set up the environment for running the RL Obstacle Avoidance System, follow these steps:

1. **Clone the repository**:
   ```bash
   git clone https://github.com/priyanshu2393/rl_obstacle_avoidance.git
   cd rl_obstacle_avoidance
   ```

2. **Create a virtual environment** (optional but recommended):
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows use `venv\Scripts\activate`
   ```

3. **Install the required packages**:
   ```bash
   pip install -r requirements.txt
   ```

## Training Procedures

To train the TD3 agent:

1. Navigate to the `RL` directory:
   ```bash
   cd RL
   ```

2. Run the training script:
   ```bash
   python train.py
   ```

3. Adjust hyperparameters in `config.py` as needed before starting the training.

The training logs will be saved in the `logs` directory.

## Testing Procedures

To test the trained agent:

1. Ensure you are in the `RL` directory.

2. Execute the testing script:
   ```bash
   python test.py
   ```

The agent's performance will be evaluated and metrics will be displayed.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request for any enhancements or fixes.

## License

This project is licensed under the MIT License. See the LICENSE file for more information.