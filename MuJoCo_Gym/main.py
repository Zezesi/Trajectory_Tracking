import gymnasium as gym
from stable_baselines3 import DDPG, DQN, PPO, SAC, TD3, A2C
import os
import argparse


# Create directories to store models and logs
model_dir = "models"
log_dir = "logs"
os.makedirs(model_dir, exist_ok=True)
os.makedirs(log_dir, exist_ok=True)

class total_model:
    def __init__(self,env,rl_algo)->None:
        self.env=env
        self.rl_algo=rl_algo

    def train(self):
        if self.rl_algo == 'SAC':
            model = SAC('MlpPolicy', self.env, verbose=1, device='cpu', tensorboard_log=log_dir, learning_rate=0.01)
        elif self.rl_algo == 'TD3':
            model = TD3('MlpPolicy', self.env, verbose=1, device='cpu', tensorboard_log=log_dir)
        elif self.rl_algo == 'A2C':
            model = A2C('MlpPolicy', self.env, verbose=1, device='cpu', tensorboard_log=log_dir)
        elif self.rl_algo == 'DQN':
            model = DQN('MlpPolicy', self.env, verbose=1, device='cpu', tensorboard_log=log_dir)
        elif self.rl_algo == 'PPO':
            model = PPO('MlpPolicy', self.env, verbose=1, device='cpu', tensorboard_log=log_dir)
        else:
            print('Algorithm not supported!')
            return

        TIMESTEPS = 25000
        iters = 0
        while True:
            iters += 1

            model.learn(total_timesteps=TIMESTEPS, reset_num_timesteps=False)
            model.save(f"{model_dir}/{self.rl_algo}_{TIMESTEPS * iters}")

    def test(self,infer_model):
        if self.rl_algo == 'SAC':
            model = SAC.load(infer_model, env=self.env)
        elif self.rl_algo == 'TD3':
            model = TD3.load(infer_model, env=self.env)
        elif self.rl_algo == 'A2C':
            model = A2C.load(infer_model, env=self.env)
        elif self.rl_algo == 'DQN':
            model = DQN.load(infer_model, env=self.env)
        elif self.rl_algo == 'PPO':
            model = PPO.load(infer_model, env=self.env)
        else:
            print('Algorithm not supported!')
            return

        # First observation(state) in the env
        obs = self.env.reset()[0]
        extra_steps = 500
        while True:
            action, _ = model.predict(obs)
            obs, _, done, _, _ = self.env.step(action)
            if done:
                extra_steps -= 1
                if extra_steps < 0:
                    break





if __name__ == '__main__':


    parser = argparse.ArgumentParser(description='Train or Test model.')
    parser.add_argument('gymenv', help='Gymnasium environment i.e Humanoid-v4, Reacher-v4...') # first letter must be in uppercase
    parser.add_argument('rl_algo', help='RL algorithm i.e SAC, TD3...')
    parser.add_argument('-t', '--train', action='store_true')
    parser.add_argument('-s', '--test', metavar='path_to_model')
    args = parser.parse_args()
    if args.train:
       gymenv = gym.make(args.gymenv, render_mode='human')

    if args.test:
       gymenv = gym.make(args.gymenv, render_mode='human')

    model=total_model(gymenv,args.rl_algo)

    if args.train:
        model.train()

    if args.test:
        if os.path.isfile(args.test):
            model.test(infer_model=args.test)
        else:
            print(f'{args.test} not found')
