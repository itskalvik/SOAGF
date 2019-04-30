### Introduction
There are several applications where having a large-scale team of robots is advantageous compared to using a  single robot. Tasks can be divided into several sub-tasks which can be distributed among the team of robots, the robots can be reconfigured to solve a complex task which would otherwise require a specialised robot and some tasks could be completed much faster when compared to using a single robot. One such use case would be disaster rescue where robots could be used to find survivors and alert authorities of their location. Another such application is exploration and mapping. Having multiple robots exploring and mapping different regions in space can greatly reduce the amount of time needed to complete the task. 

The task of assigning paths to a team of robots to form a target formation is non-trivial. Given a set of paths, there are several things to consider such as collisions within the cluster, the optimality of the paths taken by each robot and assignments of robots. Matthew Turpin, et al proposed CAPT, a method for computing optimal paths for a set of robots from an initial to a target position. They assume that the robots are interchangeable in the target formation. Agarwal and Akella improve on CAPT by allowing for variable scale and translation of the target formation, thereby not requiring exact positions for every robot in the target formation. Although, Agarwal and Akella assume that the target location of the cluster is not fixed, their solution does allow for the location and scale of the target formation to be constrained to a specific region in space.

In this project, I implemented a 2D variant of the assignment and goal formation algorithm proposed in "Simultaneous Optimization of Assignments and Goal Formations for Multiple Robots" by Agarwal and Akella, and tried to extend their work by presenting an iterative algorithm which can handle variable rotations of the goal formation.

### Paper Overview
The papers considers $n$ identical spherical robots with equal radius $R$. The initial positions of the robots are given by  \(\mathbf{p}_i = (\mathit{p_{ix}},\mathit{p_{iy}}, \mathit{p_{iz}})^\top, \mathbf{i} = 1, . . . , \mathit{n}\) and the initial formation is the set of initial positions $\textbf{P} = (\textbf{p}_i^\top)$. Similarly, the required goal formation's shape is given by  $\textbf{S} = (\textbf{s}_i^\top)$, in which each point is given by  $\mathbf{s}_j = (\mathit{s_{jx}},\mathit{s_{jy}}, \mathit{s_{jz}})^\top, \mathbf{j} = 1, . . . , \mathit{n}$ and  the computed goal formation is given by  $\textbf{Q} = (\textbf{q}_j\top)$ and $\mathbf{q}_j = (\mathit{q_{jx}},\mathit{q_{jy}}, \mathit{q_{jz}})^\top, \mathbf{j} = 1, . . . , \mathit{n}$. A translation vector $\textbf{d} = (\mathit{d_{x}},\mathit{d_{y}}, \mathit{d_{z}})^\top$ is defined such that $\textbf{q}_1 = \textbf{d}$ and a scalar $\alpha$ represents the scale of the goal formation. The proposed algorithm simultaneously computes the optimal assignments and formation parameters (scale and translation) for the goal formation for a team of robots from an initial position $\textbf{P}$ to the goal formation $\textbf{Q}$. 

Given the cost function  $\mathit{c}^{\alpha d}_{ij}$
$$
\begin{aligned}
\mathit{c}^{\alpha d}_{ij} &= ||\mathbf{p}_i - \mathbf{q}_j||^2_2 \\
& = (\mathbf{p}_i - \alpha \mathbf{s}_j - \mathbf{d})^\top(\mathbf{p}_i - \alpha \mathbf{s}_j - \mathbf{d}) \\
& = \mathbf{p}_i^\top\mathbf{p}_i + \alpha^2\mathbf{s}_j^\top\mathbf{s}_j - 2\alpha\mathbf{p}_i^\top\mathbf{s}_j + 2\alpha\mathbf{s}_j^\top\mathbf{d} \\ 
& \quad \  - 2\mathbf{p}_i^\top\mathbf{d} + \mathbf{d}^\top\mathbf{d}
\end{aligned}
$$


{% include lib/mathjax.html %}
