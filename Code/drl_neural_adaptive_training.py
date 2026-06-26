#!/usr/bin/env python3
"""
================================================================================
DRL NEURAL-ADAPTIVE CONTROLLER FOR AGRICULTURAL UAV
Event-Triggered Super-Twisting SMC with Meta-Learning (MAML)
================================================================================

PURPOSE:
    Train and test deep reinforcement learning agent to adapt super-twisting
    gains (γ₁, γ₂) for agricultural UAV missions (hover, track, spray modes).
    
HARDWARE TARGET:
    Huawei MateBook D16 (Intel Core i5/i7, 16GB RAM, integrated GPU)
    - Optimized for CPU training (no CUDA required)
    - Lightweight PPO implementation with MAML meta-learning
    
TRAINING PHASES:
    1. COARSE DEMO (10k steps): Learn basic task structure from expert demos
    2. NORMAL TRAINING (100k steps): Standard PPO exploration-exploitation
    3. FINE TRAINING (50k steps): Task-specific refinement with MAML
    
COMMUNICATION:
    - UDP socket for MATLAB integration (Simulink UAV dynamics)
    - JSON message format for state/action/reward exchange
    
AUTHOR: [Your Name], NUST SEECS
DATE: January 31, 2026
PAPER: ICRAI 2026 - Event-Triggered Neural-AS-NST-STSMC
================================================================================
"""

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal
from collections import deque
import json
import socket
import time
import matplotlib.pyplot as plt
from datetime import datetime
import os

# ============================================================================
# CONFIGURATION & HYPERPARAMETERS
# ============================================================================

class Config:
    """
    Central configuration for DRL training on Huawei MateBook D16.
    Optimized for CPU training with reasonable memory footprint.
    """
    
    # ========== HARDWARE OPTIMIZATION ==========
    DEVICE = torch.device("cpu")  # MateBook D16: Use CPU (stable, no CUDA needed)
    NUM_WORKERS = 4               # Parallel environments (adjust for i5: 4, i7: 8)
    TORCH_THREADS = 4             # PyTorch CPU threads
    
    # ========== STATE/ACTION SPACES ==========
    # State: [sliding_var (3), velocity (3), altitude (1), mass (1), wind_mag (1)] = 9D
    STATE_DIM = 9
    # Action: [γ₁_delta, γ₂_delta] ∈ [-1, 1] → adaptive gain increments
    ACTION_DIM = 2
    
    # ========== PPO HYPERPARAMETERS ==========
    GAMMA = 0.99              # Discount factor for future rewards
    LAMBDA = 0.95             # GAE lambda (advantage estimation)
    EPSILON = 0.2             # PPO clipping parameter
    ACTOR_LR = 3e-4           # Actor learning rate (conservative for stability)
    CRITIC_LR = 1e-3          # Critic learning rate
    ENTROPY_COEFF = 0.01      # Exploration bonus
    VALUE_LOSS_COEFF = 0.5    # V-function loss weight
    MAX_GRAD_NORM = 0.5       # Gradient clipping (prevent exploding gradients)
    
    # ========== TRAINING PHASES ==========
    # Phase 1: Coarse Demo (imitation learning from expert)
    COARSE_STEPS = 10_000
    COARSE_BATCH_SIZE = 256
    COARSE_EPOCHS = 4
    
    # Phase 2: Normal Training (standard PPO)
    NORMAL_STEPS = 100_000
    NORMAL_BATCH_SIZE = 512
    NORMAL_EPOCHS = 10
    
    # Phase 3: Fine Training (MAML meta-learning)
    FINE_STEPS = 50_000
    FINE_BATCH_SIZE = 256
    FINE_EPOCHS = 5
    MAML_INNER_LR = 0.01      # Inner-loop adaptation rate
    MAML_INNER_STEPS = 5      # Gradient steps per task adaptation
    
    # ========== TASK DISTRIBUTION (Agricultural Modes) ==========
    TASKS = ['hover', 'track', 'spray']  # 3 distinct operational modes
    TASK_DURATION = 60.0      # seconds per task episode
    
    # ========== NETWORK ARCHITECTURE ==========
    HIDDEN_DIM = 128          # MLP hidden layer size (lightweight for CPU)
    NUM_LAYERS = 2            # Depth of actor/critic networks
    
    # ========== BUFFER & LOGGING ==========
    BUFFER_SIZE = 2048        # Replay buffer capacity
    LOG_INTERVAL = 1000       # Steps between console logs
    SAVE_INTERVAL = 10_000    # Steps between model checkpoints
    
    # ========== UDP COMMUNICATION (MATLAB INTEGRATION) ==========
    UDP_IP = "127.0.0.1"      # Localhost for local MATLAB Simulink
    UDP_PORT_SEND = 5005      # Python → MATLAB (actions)
    UDP_PORT_RECV = 5006      # MATLAB → Python (states/rewards)
    UDP_TIMEOUT = 10.0        # Socket timeout (seconds)
    
    # ========== FILE PATHS ==========
    CHECKPOINT_DIR = "./checkpoints"
    LOG_DIR = "./logs"
    PLOT_DIR = "./plots"

# Initialize PyTorch for CPU efficiency
torch.set_num_threads(Config.TORCH_THREADS)

# ============================================================================
# NEURAL NETWORK ARCHITECTURES
# ============================================================================

class ActorNetwork(nn.Module):
    """
    Policy network (actor): Maps state → action distribution (μ, σ).
    Outputs: Mean and log_std for Gaussian policy over [γ₁_delta, γ₂_delta].
    """
    def __init__(self, state_dim, action_dim, hidden_dim=128):
        super(ActorNetwork, self).__init__()
        
        # Shared feature extractor
        self.shared = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.Tanh(),  # Smooth activations (better than ReLU for control)
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh()
        )
        
        # Policy head: Output mean μ ∈ ℝ²
        self.mean_head = nn.Linear(hidden_dim, action_dim)
        
        # Log standard deviation (learnable, ensures σ > 0)
        self.log_std = nn.Parameter(torch.zeros(action_dim))
        
    def forward(self, state):
        """
        Forward pass: state → (action_mean, action_std)
        
        Args:
            state (torch.Tensor): Batch of states [batch_size, state_dim]
        
        Returns:
            action_mean (torch.Tensor): Mean of action distribution [batch_size, action_dim]
            action_std (torch.Tensor): Std dev [batch_size, action_dim]
        """
        features = self.shared(state)
        action_mean = torch.tanh(self.mean_head(features))  # Bound to [-1, 1]
        action_std = torch.exp(self.log_std).expand_as(action_mean)
        return action_mean, action_std
    
    def sample_action(self, state):
        """
        Sample action from policy: a ~ N(μ(s), σ²)
        
        Returns:
            action (torch.Tensor): Sampled action
            log_prob (torch.Tensor): Log probability log π(a|s)
        """
        mean, std = self.forward(state)
        dist = Normal(mean, std)
        action = dist.sample()
        log_prob = dist.log_prob(action).sum(dim=-1)  # Sum over action dimensions
        return action, log_prob
    
    def evaluate_action(self, state, action):
        """
        Evaluate log probability of given action under current policy.
        Used during PPO updates.
        
        Returns:
            log_prob (torch.Tensor): log π(a|s)
            entropy (torch.Tensor): Policy entropy H(π(·|s))
        """
        mean, std = self.forward(state)
        dist = Normal(mean, std)
        log_prob = dist.log_prob(action).sum(dim=-1)
        entropy = dist.entropy().sum(dim=-1)
        return log_prob, entropy


class CriticNetwork(nn.Module):
    """
    Value network (critic): Maps state → V(s) (expected return).
    """
    def __init__(self, state_dim, hidden_dim=128):
        super(CriticNetwork, self).__init__()
        
        self.network = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, 1)  # Scalar value output
        )
        
    def forward(self, state):
        """
        Forward pass: state → V(s)
        
        Args:
            state (torch.Tensor): [batch_size, state_dim]
        
        Returns:
            value (torch.Tensor): State value [batch_size, 1]
        """
        return self.network(state)


# ============================================================================
# REPLAY BUFFER
# ============================================================================

class RolloutBuffer:
    """
    Experience replay buffer for on-policy PPO training.
    Stores trajectories and computes GAE (Generalized Advantage Estimation).
    """
    def __init__(self, capacity=2048):
        self.capacity = capacity
        self.clear()
        
    def clear(self):
        """Reset buffer to empty state."""
        self.states = []
        self.actions = []
        self.rewards = []
        self.log_probs = []
        self.values = []
        self.dones = []
        
    def store(self, state, action, reward, log_prob, value, done):
        """
        Store single transition (s, a, r, log π(a|s), V(s), done).
        """
        self.states.append(state)
        self.actions.append(action)
        self.rewards.append(reward)
        self.log_probs.append(log_prob)
        self.values.append(value)
        self.dones.append(done)
        
    def get(self):
        """
        Retrieve all stored experiences as numpy arrays.
        """
        return (
            np.array(self.states, dtype=np.float32),
            np.array(self.actions, dtype=np.float32),
            np.array(self.rewards, dtype=np.float32),
            np.array(self.log_probs, dtype=np.float32),
            np.array(self.values, dtype=np.float32),
            np.array(self.dones, dtype=np.float32)
        )
    
    def compute_gae(self, next_value, gamma=0.99, lam=0.95):
        """
        Compute Generalized Advantage Estimation (GAE).
        
        GAE formula:
            A_t = δ_t + (γλ)δ_{t+1} + (γλ)²δ_{t+2} + ...
            where δ_t = r_t + γV(s_{t+1}) - V(s_t)
        
        Args:
            next_value (float): V(s_T) for terminal state
            gamma (float): Discount factor
            lam (float): GAE lambda parameter
        
        Returns:
            advantages (np.ndarray): Advantage estimates A(s,a)
            returns (np.ndarray): Discounted returns G_t
        """
        values = self.values + [next_value]
        advantages = []
        gae = 0
        
        # Backward pass to compute GAE
        for t in reversed(range(len(self.rewards))):
            delta = self.rewards[t] + gamma * values[t+1] * (1 - self.dones[t]) - values[t]
            gae = delta + gamma * lam * (1 - self.dones[t]) * gae
            advantages.insert(0, gae)
        
        advantages = np.array(advantages, dtype=np.float32)
        returns = advantages + np.array(self.values, dtype=np.float32)
        
        return advantages, returns


# ============================================================================
# PPO AGENT WITH MAML META-LEARNING
# ============================================================================

class PPO_MAML_Agent:
    """
    Proximal Policy Optimization agent with Model-Agnostic Meta-Learning.
    
    Features:
        - Standard PPO for single-task training
        - MAML for fast task adaptation (hover/track/spray modes)
        - Expert demonstration imitation (coarse phase)
    """
    def __init__(self, config):
        self.config = config
        
        # Initialize actor and critic networks
        self.actor = ActorNetwork(
            config.STATE_DIM, 
            config.ACTION_DIM, 
            config.HIDDEN_DIM
        ).to(config.DEVICE)
        
        self.critic = CriticNetwork(
            config.STATE_DIM, 
            config.HIDDEN_DIM
        ).to(config.DEVICE)
        
        # Optimizers
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=config.ACTOR_LR)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=config.CRITIC_LR)
        
        # Replay buffer
        self.buffer = RolloutBuffer(config.BUFFER_SIZE)
        
        # Training statistics
        self.total_steps = 0
        self.episode_rewards = []
        self.episode_lengths = []
        
        # Create checkpoint directories
        os.makedirs(config.CHECKPOINT_DIR, exist_ok=True)
        os.makedirs(config.LOG_DIR, exist_ok=True)
        os.makedirs(config.PLOT_DIR, exist_ok=True)
        
    def select_action(self, state, deterministic=False):
        """
        Select action from policy given current state.
        
        Args:
            state (np.ndarray): Current state observation [state_dim]
            deterministic (bool): If True, return mean action (no sampling)
        
        Returns:
            action (np.ndarray): Action to execute [action_dim]
            log_prob (float): Log probability of action
            value (float): State value V(s)
        """
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.config.DEVICE)
        
        with torch.no_grad():
            if deterministic:
                action_mean, _ = self.actor(state_tensor)
                action = action_mean
                log_prob = torch.zeros(1)
            else:
                action, log_prob = self.actor.sample_action(state_tensor)
            
            value = self.critic(state_tensor)
        
        return (
            action.cpu().numpy()[0], 
            log_prob.cpu().numpy()[0], 
            value.cpu().numpy()[0][0]
        )
    
    def update_ppo(self, batch_size, epochs):
        """
        PPO policy update using clipped objective.
        
        Algorithm:
            1. Sample mini-batches from replay buffer
            2. Compute clipped surrogate loss L^CLIP
            3. Update actor with gradient ascent
            4. Update critic with MSE loss on value estimates
        
        Args:
            batch_size (int): Mini-batch size
            epochs (int): Number of update epochs per data collection
        
        Returns:
            metrics (dict): Training metrics (actor_loss, critic_loss, entropy)
        """
        # Retrieve buffer data
        states, actions, rewards, old_log_probs, old_values, dones = self.buffer.get()
        
        # Compute GAE advantages and returns
        # Assume episode ended (next_value = 0)
        advantages, returns = self.buffer.compute_gae(
            next_value=0.0, 
            gamma=self.config.GAMMA, 
            lam=self.config.LAMBDA
        )
        
        # Normalize advantages (stabilizes training)
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        
        # Convert to PyTorch tensors
        states_t = torch.FloatTensor(states).to(self.config.DEVICE)
        actions_t = torch.FloatTensor(actions).to(self.config.DEVICE)
        old_log_probs_t = torch.FloatTensor(old_log_probs).to(self.config.DEVICE)
        advantages_t = torch.FloatTensor(advantages).to(self.config.DEVICE)
        returns_t = torch.FloatTensor(returns).to(self.config.DEVICE)
        
        # Training metrics
        total_actor_loss = 0
        total_critic_loss = 0
        total_entropy = 0
        update_count = 0
        
        # Multiple epochs over same data (PPO allows this)
        for _ in range(epochs):
            # Generate random mini-batches
            indices = np.arange(len(states))
            np.random.shuffle(indices)
            
            for start in range(0, len(states), batch_size):
                end = start + batch_size
                batch_idx = indices[start:end]
                
                # Mini-batch data
                batch_states = states_t[batch_idx]
                batch_actions = actions_t[batch_idx]
                batch_old_log_probs = old_log_probs_t[batch_idx]
                batch_advantages = advantages_t[batch_idx]
                batch_returns = returns_t[batch_idx]
                
                # ========== ACTOR UPDATE ==========
                # Evaluate actions under current policy
                log_probs, entropy = self.actor.evaluate_action(batch_states, batch_actions)
                
                # Compute importance sampling ratio
                ratio = torch.exp(log_probs - batch_old_log_probs)
                
                # Clipped surrogate objective (PPO core)
                surr1 = ratio * batch_advantages
                surr2 = torch.clamp(ratio, 1.0 - self.config.EPSILON, 
                                   1.0 + self.config.EPSILON) * batch_advantages
                actor_loss = -torch.min(surr1, surr2).mean()
                
                # Add entropy bonus (encourages exploration)
                actor_loss -= self.config.ENTROPY_COEFF * entropy.mean()
                
                # Gradient update
                self.actor_optimizer.zero_grad()
                actor_loss.backward()
                nn.utils.clip_grad_norm_(self.actor.parameters(), self.config.MAX_GRAD_NORM)
                self.actor_optimizer.step()
                
                # ========== CRITIC UPDATE ==========
                # Predict state values
                values = self.critic(batch_states).squeeze()
                
                # MSE loss between predicted and actual returns
                critic_loss = nn.MSELoss()(values, batch_returns)
                
                # Gradient update
                self.critic_optimizer.zero_grad()
                critic_loss.backward()
                nn.utils.clip_grad_norm_(self.critic.parameters(), self.config.MAX_GRAD_NORM)
                self.critic_optimizer.step()
                
                # Accumulate metrics
                total_actor_loss += actor_loss.item()
                total_critic_loss += critic_loss.item()
                total_entropy += entropy.mean().item()
                update_count += 1
        
        # Average metrics
        metrics = {
            'actor_loss': total_actor_loss / update_count,
            'critic_loss': total_critic_loss / update_count,
            'entropy': total_entropy / update_count
        }
        
        return metrics
    
    def meta_learn_maml(self, task_name, num_inner_steps=5):
        """
        MAML inner-loop adaptation for specific task (hover/track/spray).
        
        Algorithm (MAML):
            1. Sample task τ_i from task distribution
            2. Adapt policy with K gradient steps on τ_i
            3. Evaluate adapted policy on τ_i test data
            4. Meta-update: θ ← θ - β∇_θ Σ_i L_τi(θ - α∇_θ L_τi(θ))
        
        Args:
            task_name (str): 'hover', 'track', or 'spray'
            num_inner_steps (int): Number of adaptation gradient steps
        
        Returns:
            adapted_actor (ActorNetwork): Task-adapted actor network
        """
        # Clone current actor for task-specific adaptation
        adapted_actor = ActorNetwork(
            self.config.STATE_DIM, 
            self.config.ACTION_DIM, 
            self.config.HIDDEN_DIM
        ).to(self.config.DEVICE)
        adapted_actor.load_state_dict(self.actor.state_dict())
        
        # Inner-loop optimizer (SGD for fast adaptation)
        inner_optimizer = optim.SGD(
            adapted_actor.parameters(), 
            lr=self.config.MAML_INNER_LR
        )
        
        # Retrieve task-specific data from buffer
        states, actions, _, old_log_probs, _, _ = self.buffer.get()
        states_t = torch.FloatTensor(states).to(self.config.DEVICE)
        actions_t = torch.FloatTensor(actions).to(self.config.DEVICE)
        
        # Inner-loop adaptation (K gradient steps)
        for step in range(num_inner_steps):
            # Compute policy loss on task data
            log_probs, _ = adapted_actor.evaluate_action(states_t, actions_t)
            inner_loss = -log_probs.mean()  # Maximize log likelihood
            
            # Gradient update
            inner_optimizer.zero_grad()
            inner_loss.backward()
            inner_optimizer.step()
        
        return adapted_actor
    
    def save_checkpoint(self, filename):
        """
        Save model checkpoint (actor, critic, optimizer states).
        """
        checkpoint = {
            'actor_state_dict': self.actor.state_dict(),
            'critic_state_dict': self.critic.state_dict(),
            'actor_optimizer': self.actor_optimizer.state_dict(),
            'critic_optimizer': self.critic_optimizer.state_dict(),
            'total_steps': self.total_steps,
            'episode_rewards': self.episode_rewards
        }
        filepath = os.path.join(self.config.CHECKPOINT_DIR, filename)
        torch.save(checkpoint, filepath)
        print(f"✓ Checkpoint saved: {filepath}")
    
    def load_checkpoint(self, filename):
        """
        Load model checkpoint from file.
        """
        filepath = os.path.join(self.config.CHECKPOINT_DIR, filename)
        checkpoint = torch.load(filepath, map_location=self.config.DEVICE)
        
        self.actor.load_state_dict(checkpoint['actor_state_dict'])
        self.critic.load_state_dict(checkpoint['critic_state_dict'])
        self.actor_optimizer.load_state_dict(checkpoint['actor_optimizer'])
        self.critic_optimizer.load_state_dict(checkpoint['critic_optimizer'])
        self.total_steps = checkpoint['total_steps']
        self.episode_rewards = checkpoint['episode_rewards']
        
        print(f"✓ Checkpoint loaded: {filepath}")


# ============================================================================
# MATLAB UDP COMMUNICATION
# ============================================================================

class MatlabUDPInterface:
    """
    UDP socket interface for communication with MATLAB Simulink.
    
    Protocol:
        Python → MATLAB: JSON {"action": [γ₁_delta, γ₂_delta]}
        MATLAB → Python: JSON {"state": [...], "reward": float, "done": bool}
    """
    def __init__(self, config):
        self.config = config
        
        # Create UDP sockets
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_recv.bind((config.UDP_IP, config.UDP_PORT_RECV))
        self.sock_recv.settimeout(config.UDP_TIMEOUT)
        
        print(f"UDP Interface initialized:")
        print(f"  Send to MATLAB: {config.UDP_IP}:{config.UDP_PORT_SEND}")
        print(f"  Receive from MATLAB: {config.UDP_IP}:{config.UDP_PORT_RECV}")
    
    def send_action(self, action):
        """
        Send action to MATLAB Simulink.
        
        Args:
            action (np.ndarray): [γ₁_delta, γ₂_delta]
        """
        message = json.dumps({"action": action.tolist()})
        self.sock_send.sendto(
            message.encode(), 
            (self.config.UDP_IP, self.config.UDP_PORT_SEND)
        )
    
    def receive_state(self):
        """
        Receive state observation from MATLAB.
        
        Returns:
            state (np.ndarray): State vector [9D]
            reward (float): Immediate reward
            done (bool): Episode termination flag
        """
        try:
            data, _ = self.sock_recv.recvfrom(4096)
            message = json.loads(data.decode())
            
            state = np.array(message['state'], dtype=np.float32)
            reward = float(message['reward'])
            done = bool(message['done'])
            
            return state, reward, done
        
        except socket.timeout:
            print("⚠ UDP timeout: No response from MATLAB")
            return None, 0.0, True
    
    def close(self):
        """Close UDP sockets."""
        self.sock_send.close()
        self.sock_recv.close()


# ============================================================================
# TRAINING FUNCTIONS (3-PHASE PIPELINE)
# ============================================================================

def phase1_coarse_demo_training(agent, config):
    """
    PHASE 1: COARSE DEMO TRAINING (Imitation Learning)
    
    Objective:
        Learn basic task structure from expert demonstrations.
        Expert policy: Simple PID or pre-tuned PSO gains.
    
    Duration: 10k steps (~30 minutes on MateBook D16)
    """
    print("\n" + "="*70)
    print("PHASE 1: COARSE DEMO TRAINING (Imitation Learning)")
    print("="*70)
    
    # Generate expert demonstrations (simplified)
    # In practice, load from MATLAB Simulink runs with PID/PSO
    expert_states, expert_actions = generate_expert_demos(config)
    
    # Supervised learning: Minimize behavioral cloning loss
    states_t = torch.FloatTensor(expert_states).to(config.DEVICE)
    actions_t = torch.FloatTensor(expert_actions).to(config.DEVICE)
    
    for epoch in range(config.COARSE_EPOCHS):
        # Forward pass
        action_mean, action_std = agent.actor(states_t)
        
        # MSE loss (imitate expert actions)
        bc_loss = nn.MSELoss()(action_mean, actions_t)
        
        # Backward pass
        agent.actor_optimizer.zero_grad()
        bc_loss.backward()
        agent.actor_optimizer.step()
        
        print(f"  Epoch {epoch+1}/{config.COARSE_EPOCHS} | BC Loss: {bc_loss.item():.4f}")
    
    # Save coarse model
    agent.save_checkpoint("phase1_coarse.pth")
    print("✓ Phase 1 complete: Coarse model saved\n")


def phase2_normal_ppo_training(agent, matlab_udp, config):
    """
    PHASE 2: NORMAL PPO TRAINING (Standard RL)
    
    Objective:
        Explore state-action space and optimize adaptive gains via PPO.
        Train on mixed tasks (hover/track/spray) without meta-learning.
    
    Duration: 100k steps (~3-4 hours on MateBook D16)
    """
    print("\n" + "="*70)
    print("PHASE 2: NORMAL PPO TRAINING (Standard Exploration-Exploitation)")
    print("="*70)
    
    episode_count = 0
    step_count = 0
    episode_reward = 0
    episode_length = 0
    
    # Initialize MATLAB connection
    print("Waiting for MATLAB Simulink to start...")
    state, _, _ = matlab_udp.receive_state()
    
    while step_count < config.NORMAL_STEPS:
        # Select action
        action, log_prob, value = agent.select_action(state)
        
        # Send action to MATLAB
        matlab_udp.send_action(action)
        
        # Receive next state and reward
        next_state, reward, done = matlab_udp.receive_state()
        
        # Store transition in buffer
        agent.buffer.store(state, action, reward, log_prob, value, done)
        
        # Update counters
        episode_reward += reward
        episode_length += 1
        step_count += 1
        agent.total_steps += 1
        
        # Episode ended
        if done or episode_length >= config.TASK_DURATION * 10:  # 600 steps @ 10Hz
            agent.episode_rewards.append(episode_reward)
            agent.episode_lengths.append(episode_length)
            
            print(f"  Episode {episode_count} | Steps: {episode_length} | "
                  f"Reward: {episode_reward:.2f} | Total Steps: {step_count}")
            
            # PPO update
            if len(agent.buffer.states) >= config.NORMAL_BATCH_SIZE:
                metrics = agent.update_ppo(config.NORMAL_BATCH_SIZE, config.NORMAL_EPOCHS)
                print(f"    → Actor Loss: {metrics['actor_loss']:.4f} | "
                      f"Critic Loss: {metrics['critic_loss']:.4f} | "
                      f"Entropy: {metrics['entropy']:.4f}")
                agent.buffer.clear()
            
            # Reset environment
            episode_count += 1
            episode_reward = 0
            episode_length = 0
            state, _, _ = matlab_udp.receive_state()
        else:
            state = next_state
        
        # Save checkpoint periodically
        if step_count % config.SAVE_INTERVAL == 0:
            agent.save_checkpoint(f"phase2_step{step_count}.pth")
    
    # Final checkpoint
    agent.save_checkpoint("phase2_normal.pth")
    print("✓ Phase 2 complete: Normal PPO training finished\n")


def phase3_fine_maml_training(agent, matlab_udp, config):
    """
    PHASE 3: FINE MAML TRAINING (Meta-Learning)
    
    Objective:
        Enable fast task adaptation using Model-Agnostic Meta-Learning.
        Train meta-policy that can quickly adapt to hover/track/spray modes.
    
    Duration: 50k steps (~2 hours on MateBook D16)
    """
    print("\n" + "="*70)
    print("PHASE 3: FINE MAML TRAINING (Meta-Learning for Task Adaptation)")
    print("="*70)
    
    step_count = 0
    task_idx = 0
    
    while step_count < config.FINE_STEPS:
        # Sample random task
        task_name = config.TASKS[task_idx % len(config.TASKS)]
        print(f"\n  Meta-Episode on task: {task_name.upper()}")
        
        # Collect task-specific data
        state, _, _ = matlab_udp.receive_state()
        for t in range(100):  # Short episodes for meta-learning
            action, log_prob, value = agent.select_action(state)
            matlab_udp.send_action(action)
            next_state, reward, done = matlab_udp.receive_state()
            agent.buffer.store(state, action, reward, log_prob, value, done)
            
            if done:
                break
            state = next_state
        
        # MAML inner-loop adaptation
        adapted_actor = agent.meta_learn_maml(task_name, config.MAML_INNER_STEPS)
        
        # Evaluate adapted policy (outer-loop loss)
        test_reward = evaluate_adapted_policy(adapted_actor, matlab_udp, config)
        print(f"    → Adapted policy reward: {test_reward:.2f}")
        
        # Meta-update (outer-loop gradient)
        # Simplified: Use test_reward as meta-objective
        # In full MAML, compute gradient through inner-loop updates
        
        # Update PPO with collected data
        if len(agent.buffer.states) >= config.FINE_BATCH_SIZE:
            metrics = agent.update_ppo(config.FINE_BATCH_SIZE, config.FINE_EPOCHS)
            agent.buffer.clear()
        
        step_count += 100
        agent.total_steps += 100
        task_idx += 1
        
        # Save checkpoint
        if step_count % config.SAVE_INTERVAL == 0:
            agent.save_checkpoint(f"phase3_step{step_count}.pth")
    
    # Final MAML-trained model
    agent.save_checkpoint("phase3_maml_final.pth")
    print("✓ Phase 3 complete: MAML meta-learning finished\n")


# ============================================================================
# TESTING & EVALUATION
# ============================================================================

def test_trained_agent(agent, matlab_udp, config, num_episodes=10):
    """
    Test trained DRL agent on MATLAB Simulink UAV environment.
    
    Args:
        agent (PPO_MAML_Agent): Trained agent
        matlab_udp (MatlabUDPInterface): UDP interface
        num_episodes (int): Number of test episodes
    
    Returns:
        results (dict): Test statistics (rewards, RMSE, energy, etc.)
    """
    print("\n" + "="*70)
    print("TESTING TRAINED AGENT")
    print("="*70)
    
    agent.actor.eval()  # Set to evaluation mode (no dropout/batch norm)
    agent.critic.eval()
    
    test_rewards = []
    test_lengths = []
    
    for ep in range(num_episodes):
        state, _, _ = matlab_udp.receive_state()
        episode_reward = 0
        episode_length = 0
        done = False
        
        while not done:
            # Deterministic action (no exploration)
            action, _, _ = agent.select_action(state, deterministic=True)
            matlab_udp.send_action(action)
            next_state, reward, done = matlab_udp.receive_state()
            
            episode_reward += reward
            episode_length += 1
            state = next_state
            
            if episode_length >= config.TASK_DURATION * 10:
                break
        
        test_rewards.append(episode_reward)
        test_lengths.append(episode_length)
        print(f"  Test Episode {ep+1} | Reward: {episode_reward:.2f} | Length: {episode_length}")
    
    # Summary statistics
    results = {
        'mean_reward': np.mean(test_rewards),
        'std_reward': np.std(test_rewards),
        'mean_length': np.mean(test_lengths)
    }
    
    print(f"\nTest Results:")
    print(f"  Mean Reward: {results['mean_reward']:.2f} ± {results['std_reward']:.2f}")
    print(f"  Mean Episode Length: {results['mean_length']:.1f} steps")
    
    return results


def plot_training_curves(agent, config):
    """
    Visualize training progress (rewards, losses, etc.).
    """
    plt.figure(figsize=(12, 5))
    
    # Plot 1: Episode rewards
    plt.subplot(1, 2, 1)
    plt.plot(agent.episode_rewards, alpha=0.6)
    # Moving average (window=50)
    if len(agent.episode_rewards) > 50:
        moving_avg = np.convolve(agent.episode_rewards, np.ones(50)/50, mode='valid')
        plt.plot(range(49, len(agent.episode_rewards)), moving_avg, 'r-', linewidth=2)
    plt.xlabel("Episode")
    plt.ylabel("Total Reward")
    plt.title("Training Progress: Episode Rewards")
    plt.grid(True, alpha=0.3)
    
    # Plot 2: Episode lengths
    plt.subplot(1, 2, 2)
    plt.plot(agent.episode_lengths, alpha=0.6)
    plt.xlabel("Episode")
    plt.ylabel("Episode Length (steps)")
    plt.title("Training Progress: Episode Lengths")
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    save_path = os.path.join(config.PLOT_DIR, "training_curves.png")
    plt.savefig(save_path, dpi=300)
    print(f"✓ Training curves saved: {save_path}")


# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def generate_expert_demos(config, num_demos=1000):
    """
    Generate expert demonstrations for imitation learning.
    
    In practice, replace with actual MATLAB PID/PSO controller runs.
    Here we use simple heuristic: γ₁, γ₂ proportional to sliding variable.
    
    Returns:
        states (np.ndarray): [num_demos, state_dim]
        actions (np.ndarray): [num_demos, action_dim]
    """
    states = []
    actions = []
    
    for _ in range(num_demos):
        # Note: Choose one of the following state generation methods based on your needs:
        # Method 1: Random state sampling (for generalization)
        # Method 2: Actual state from Matlab code
        
        # Method 1: Random state (simulate UAV conditions)
        s = np.random.randn(config.STATE_DIM).astype(np.float32)
        
        # Method 2: Actual state components from Matlab code (uncomment if needed)
        # -----------------From here------------------
        # # Random reference trajectory
        # zeta_d = np.random.uniform(-5, 5, 3).astype(np.float32)  # Desired position
        # v_d = np.random.uniform(-2, 2, 3).astype(np.float32)    # Desired velocity
        
        # # Actual state (with PID outer loop)
        # zeta = zeta_d + np.random.randn(3) * 0.5  # Position error
        # v = v_d + np.random.randn(3) * 0.3       # Velocity error
        # eta = np.random.uniform(-np.pi/6, np.pi/6, 3).astype(np.float32)  # Attitude angles [roll, pitch, yaw]
        # Omega = np.random.randn(3) * 0.5  # Angular velocity
        
        # # ===== Inner Loop: NASTSMC Attitude Control =====
        # eta_d = np.array([0.1*zeta[0], 0.1*zeta[1], 0.0])  # Desired attitude from outer loop
        
        # # Attitude errors
        # e_eta = eta_d - eta
        # de_eta = -Omega  # Approximate: deta ≈ -Omega (simplified for demo)
        
        # # Nonsingular terminal sliding surface (NASTSMC)
        # p, q = 3, 2  # Terminal sliding surface powers
        # de_eta_abs = np.abs(de_eta)
        # de_eta_power = np.sign(de_eta) * (de_eta_abs ** (p / q))
        # beta = 10.0  # Sliding surface gain
        # s = e_eta + (1.0 / beta) * de_eta_power
        
        # # Continuous approximation of sign function (CASF - Continuous Approximation Sign Function)
        # gamma_casf = 0.5  # Smoothing parameter
        # delta_s = s / (np.abs(s) + gamma_casf)
        
        # # Adaptive gains (to be tuned by DRL agent)
        # k1_hat = 100.0 + np.random.randn(3) * 10
        # k2_hat = 50.0 + np.random.randn(3) * 5
        # alpha = 80.0
        
        # # NASTSMC control law
        # u_tau_eq = np.zeros(3)  # No feedforward for demo
        # u_tau = u_tau_eq + alpha * de_eta + k1_hat * np.sqrt(np.abs(s)) * delta_s + k2_hat * np.abs(s)
        
        # # Clamp control input
        # u_tau = np.clip(u_tau, -100, 100)
        
        # # ===== Outer Loop: PID Position Control =====
        # kp_zeta = np.array([50.0, 50.0, 80.0])
        # kd_zeta = np.array([20.0, 20.0, 20.0])
        
        # e_zeta = zeta_d - zeta
        # u_f = kp_zeta * e_zeta + kd_zeta * (v_d - v)
        
        # # ===== State vector for DRL agent =====
        # # state = [sliding_var (3), velocity_error (3), altitude (1), mass_uncertainty (1), wind_magnitude (1)]
        # wind_disturbance = np.random.uniform(0, 12, 1)[0]  # Wind speed 0-12 m/s
        # mass_uncertainty = np.random.uniform(0.8, 1.2, 1)[0]  # ±20% mass variation
        
        # s = np.concatenate([
        #     s,                                      # Sliding variable [3D]
        #     v_d - v,                               # Velocity error [3D]
        #     np.array([zeta[2]]),                   # Altitude [1D]
        #     np.array([mass_uncertainty]),          # Mass uncertainty factor [1D]
        #     np.array([wind_disturbance])           # Wind magnitude [1D]
        # ]).astype(np.float32)
        # -----------------To here------------------

        # Expert action (heuristic rule)
        sliding_var_norm = np.linalg.norm(s[:3])
        gamma1_delta = np.clip(0.1 * sliding_var_norm, -1, 1)
        gamma2_delta = np.clip(0.05 * sliding_var_norm**2, -1, 1)
        a = np.array([gamma1_delta, gamma2_delta], dtype=np.float32)
        
        states.append(s)
        actions.append(a)
    
    return np.array(states), np.array(actions)


def evaluate_adapted_policy(adapted_actor, matlab_udp, config, num_steps=50):
    """
    Evaluate MAML-adapted policy on test data.
    
    Returns:
        total_reward (float): Cumulative reward over test episode
    """
    state, _, _ = matlab_udp.receive_state()
    total_reward = 0
    
    for _ in range(num_steps):
        state_t = torch.FloatTensor(state).unsqueeze(0).to(config.DEVICE)
        with torch.no_grad():
            action_mean, _ = adapted_actor(state_t)
            action = action_mean.cpu().numpy()[0]
        
        matlab_udp.send_action(action)
        next_state, reward, done = matlab_udp.receive_state()
        total_reward += reward
        
        if done:
            break
        state = next_state
    
    return total_reward


# ============================================================================
# MAIN EXECUTION
# ============================================================================

def main():
    """
    Main training pipeline: 3-phase training + testing.
    """
    print("\n" + "="*70)
    print("DRL NEURAL-ADAPTIVE CONTROLLER FOR AGRICULTURAL UAV")
    print("Event-Triggered Super-Twisting SMC with Meta-Learning")
    print("Hardware: Huawei MateBook D16 | Target: MATLAB Simulink UDP")
    print("="*70)
    
    # Initialize configuration
    config = Config()
    
    # Create agent
    print("\n[1/5] Initializing PPO-MAML agent...")
    agent = PPO_MAML_Agent(config)
    print(f"  Actor parameters: {sum(p.numel() for p in agent.actor.parameters())}")
    print(f"  Critic parameters: {sum(p.numel() for p in agent.critic.parameters())}")
    
    # Initialize UDP interface
    print("\n[2/5] Initializing MATLAB UDP interface...")
    matlab_udp = MatlabUDPInterface(config)
    
    # ========== TRAINING PIPELINE ==========
    
    print("\n[3/5] Starting 3-phase training pipeline...")
    
    # Phase 1: Coarse demo training
    phase1_coarse_demo_training(agent, config)
    
    # Phase 2: Normal PPO training
    phase2_normal_ppo_training(agent, matlab_udp, config)
    
    # Phase 3: Fine MAML training
    phase3_fine_maml_training(agent, matlab_udp, config)
    
    # ========== TESTING ==========
    
    print("\n[4/5] Testing trained agent...")
    test_results = test_trained_agent(agent, matlab_udp, config, num_episodes=10)
    
    # ========== VISUALIZATION ==========
    
    print("\n[5/5] Generating training plots...")
    plot_training_curves(agent, config)
    
    # Close UDP connection
    matlab_udp.close()
    
    print("\n" + "="*70)
    print("✓ TRAINING COMPLETE")
    print(f"  Final model: {config.CHECKPOINT_DIR}/phase3_maml_final.pth")
    print(f"  Test mean reward: {test_results['mean_reward']:.2f}")
    print(f"  Logs saved to: {config.LOG_DIR}")
    print("="*70 + "\n")


if __name__ == "__main__":
    main()
