import numpy as np
import torch as th
import torch.nn.functional as F
import gymnasium as gym
from gymnasium import spaces

from stable_baselines3 import DQN
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.evaluation import evaluate_policy

import kuka_dynamics

# ------------------------------------------------------------
# 1. JOINT LIMITS AND PICK-PLACE SETTINGS
# ------------------------------------------------------------
LOW = kuka_dynamics.JOINT_LOW
HIGH = kuka_dynamics.JOINT_HIGH
GRIP_DISTANCE = 10.0
SUCCESS_DISTANCE = 24.0


# ------------------------------------------------------------
# 2. DISCRETE PICK-AND-PLACE ENVIRONMENT (เหมือนเดิมทุกอย่าง)
# ------------------------------------------------------------
class DiscreteJointReachEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, max_steps=360, dt=0.05, max_velocity=0.65):
        super().__init__()
        self.max_steps = max_steps
        self.dt = dt
        self.max_velocity = max_velocity
        self.action_space = spaces.Discrete(12)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(18,), dtype=np.float32)

    def _obs(self):
        return kuka_dynamics.make_observation(self.q, self.goal, self.prev_v)

    def _sample_reachable_position(self):
        for _ in range(5000):
            position = self.np_random.uniform(
                low=np.array([-180.0, -180.0, 10.0]),
                high=np.array([180.0, 180.0, 80.0]),
            ).astype(np.float32)
            if kuka_dynamics.solve_ik(position, self.q) is not None:
                return position
        raise RuntimeError("Could not sample a reachable object/tray position")

    def _joint_goal(self, position):
        goal = kuka_dynamics.solve_ik(position, self.q)
        if goal is None:
            raise ValueError(f"Position is outside the arm workspace: {position}")
        return np.asarray(goal, dtype=np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.steps = 0
        self.q = self.np_random.uniform(LOW * 0.35, HIGH * 0.35).astype(np.float32)
        options = options or {}

        object_position = options.get("object_position")
        tray_position = options.get("tray_position")
        self.object_position = (
            np.asarray(object_position, dtype=np.float32)
            if object_position is not None
            else self._sample_reachable_position()
        )
        self.tray_position = (
            np.asarray(tray_position, dtype=np.float32)
            if tray_position is not None
            else self._sample_reachable_position()
        )
        for name, position in (("object_position", self.object_position), ("tray_position", self.tray_position)):
            if position.shape != (3,) or not np.isfinite(position).all():
                raise ValueError(f"{name} must contain three finite XYZ coordinates")
            if kuka_dynamics.solve_ik(position, self.q) is None:
                raise ValueError(f"{name} is outside the arm workspace: {position}")
        if object_position is None and tray_position is None:
            while np.linalg.norm(self.object_position - self.tray_position) <= SUCCESS_DISTANCE:
                self.tray_position = self._sample_reachable_position()

        self.prev_v = np.zeros(6, dtype=np.float32)
        self.is_holding = False
        self.has_picked = False
        self.goal = self._joint_goal(self.object_position)
        return self._obs(), {"object_position": self.object_position.copy(), "tray_position": self.tray_position.copy()}

    def step(self, action):
        self.steps += 1
        velocity = np.zeros(6, dtype=np.float32)
        joint_idx = int(action) // 2
        direction = 1.0 if int(action) % 2 == 0 else -1.0
        velocity[joint_idx] = direction * self.max_velocity

        old_dist = float(np.linalg.norm(self.goal - self.q))
        self.q = np.clip(self.q + velocity * self.dt, LOW, HIGH).astype(np.float32)
        new_dist = float(np.linalg.norm(self.goal - self.q))
        end_effector = np.asarray(kuka_dynamics.forward_kinematics(self.q)[-1])
        target = self.tray_position if self.is_holding else self.object_position
        target_distance = float(np.linalg.norm(end_effector - target))

        smooth_penalty = float(np.mean((velocity - self.prev_v) ** 2))
        speed_penalty = float(np.mean(velocity ** 2))
        reward = 12 * (old_dist - new_dist) - 0.035 * smooth_penalty - 0.004 * speed_penalty - 0.015
        success = False
        if self.is_holding:
            self.object_position = end_effector.copy()
            if target_distance <= SUCCESS_DISTANCE:
                self.object_position = self.tray_position.copy()
                self.is_holding = False
                success = True
                reward += 20.0
        elif target_distance <= GRIP_DISTANCE:
            self.object_position = end_effector.copy()
            self.is_holding = True
            self.has_picked = True
            self.goal = self._joint_goal(self.tray_position)
            reward += 8.0

        self.prev_v = velocity.copy()
        terminated = success
        truncated = self.steps >= self.max_steps and not terminated
        info = {
            "distance": new_dist,
            "target_distance": target_distance,
            "reached": success,
            "phase": "place" if self.is_holding else ("complete" if success else "pick"),
            "object_position": self.object_position.copy(),
            "tray_position": self.tray_position.copy(),
        }
        return self._obs(), float(reward), terminated, truncated, info


# ------------------------------------------------------------
# 3. DOUBLE DQN (DDQN) — subclass ของ SB3 DQN โดย override train()
#
#    DQN  : y = r + γ · max_a' Q_target(s', a')
#    DDQN : y = r + γ · Q_target(s', argmax_a' Q_online(s', a'))
#           (online net เลือก action, target net ประเมินค่า)
# ------------------------------------------------------------
class DDQN(DQN):
    def train(self, gradient_steps: int, batch_size: int = 100) -> None:
        self.policy.set_training_mode(True)
        self._update_learning_rate(self.policy.optimizer)

        losses = []
        for _ in range(gradient_steps):
            replay_data = self.replay_buffer.sample(batch_size, env=self._vec_normalize_env)

            # SB3 เวอร์ชันใหม่มี discounts (รองรับ n-step); เวอร์ชันเก่าใช้ gamma ปกติ
            discounts = getattr(replay_data, "discounts", None)
            if discounts is None:
                discounts = self.gamma

            with th.no_grad():
                # 1) online network เลือก action ที่ดีที่สุดใน s'
                next_actions = self.q_net(replay_data.next_observations).argmax(dim=1, keepdim=True)
                # 2) target network ประเมินค่าของ action นั้น
                next_q_values = self.q_net_target(replay_data.next_observations).gather(1, next_actions)
                # TD target
                target_q_values = replay_data.rewards + (1 - replay_data.dones) * discounts * next_q_values

            current_q_values = self.q_net(replay_data.observations)
            current_q_values = th.gather(current_q_values, dim=1, index=replay_data.actions.long())

            loss = F.smooth_l1_loss(current_q_values, target_q_values)
            losses.append(loss.item())

            self.policy.optimizer.zero_grad()
            loss.backward()
            th.nn.utils.clip_grad_norm_(self.policy.parameters(), self.max_grad_norm)
            self.policy.optimizer.step()

        self._n_updates += gradient_steps
        self.logger.record("train/n_updates", self._n_updates, exclude="tensorboard")
        self.logger.record("train/loss", np.mean(losses))


# ------------------------------------------------------------
# 4. TRAINING + EVALUATION
# ------------------------------------------------------------
if __name__ == "__main__":
    env = DiscreteJointReachEnv()
    check_env(env, warn=True)
    print("Pick-and-place environment ready:", env.observation_space.shape, env.action_space.n)

    model = DDQN(
        "MlpPolicy",
        env,
        learning_rate=1e-3,
        buffer_size=100_000,
        learning_starts=1000,
        batch_size=128,
        tau=1.0,                     # hard update (แก้จากเดิมที่ tau=0.01 + interval 1000 ทำให้ target ขยับช้าเกินไป)
        gamma=0.98,
        train_freq=4,
        gradient_steps=1,
        target_update_interval=1000,
        exploration_fraction=0.3,
        exploration_initial_eps=1.0,
        exploration_final_eps=0.05,
        policy_kwargs=dict(net_arch=[256, 256]),
        verbose=1,
        device="auto",
    )

    print("Training DDQN for movable object/tray positions...")
    model.learn(total_timesteps=200_000, progress_bar=True)
    model.save("ddqn_6dof_arm_pick_place")
    print("Model saved to ddqn_6dof_arm_pick_place.zip")

    # --------------------------------------------------------
    # Evaluation
    # --------------------------------------------------------
    eval_env = Monitor(DiscreteJointReachEnv())

    mean_reward, std_reward = evaluate_policy(
        model, eval_env, n_eval_episodes=100, deterministic=True
    )
    print(f"Mean reward: {mean_reward:.2f} +/- {std_reward:.2f}")

    eval_episodes = 100
    success_count = 0
    final_distances = []
    episode_lengths = []

    print(f"Starting evaluation over {eval_episodes} episodes...")
    for episode in range(eval_episodes):
        obs, info = eval_env.reset()
        done = False
        steps = 0
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = eval_env.step(action)
            done = terminated or truncated
            steps += 1

        if info.get("reached", False):
            success_count += 1
        final_distances.append(info.get("distance", 0.0))
        episode_lengths.append(steps)

    success_rate = success_count / eval_episodes * 100
    print("\n" + "=" * 40)
    print("DDQN EVALUATION RESULTS")
    print("=" * 40)
    print(f"Success Rate:           {success_rate:.1f}% ({success_count}/{eval_episodes})")
    print(f"Average Final Distance: {np.mean(final_distances):.4f} (joint space)")
    print(f"Best Final Distance:    {np.min(final_distances):.4f} (joint space)")
    print(f"Average Episode Length: {np.mean(episode_lengths):.1f} steps")
    print("=" * 40)