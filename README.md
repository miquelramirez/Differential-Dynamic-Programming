# Differential-Dynamic-Programming

A friend of mine directed me to these two papers (one, two) by a group working on autonomous helicopters, and I wanted to wrap my head around Differential Dynamic Programming (DDP) by writing some code.

# What is RL?

Reinforcement learning (RL) problems can be described as Markov decision processes (MDPs) can be defind by (S, A, T, H, s(0), R), where:

* S - set of states
* A - set of actions (inputs)
* T - dynamics model (set of probability distributions)
* H - horizon (number of timesteps considered)
* s(0) - initial state
* R - reward function (R: S x A -> R) 

# What is LQR

Linear quadratic regulators (LQRs) are a class of MDPs whose dynamic model is given by:

s(t+1) = A(t)s(t) + B(t)u(t) + w(t)

where:

* t = 0, ..., H
* $A(t) \in R^{nxn}$
* $B(t) \in R^{nxp}$
* $w(t)$ is a random variable (zero mean with finate variance)

To calculate the reward for being in $$s(t)$$ and taking action $$u(t)$$:

$-s(t)^{T}Q(t)s(t) - u(t)^{T}$

# What is DDP?

Differential dynamic programming can be used to find the optimal policy for LQR control problems. 


