# Differential Algebraic Representation (DAR) framework for KUKA LWR IV+

## Compiling procedure:
    - mkdir build
    - cd ./build
    - cmake ..
    - make

## Running procedure:
    - sudo ./executable

## Physical paramters

| Parameter        | Joint(1)       | Joint(2)       |
|------------------|----------------|----------------|
| $\ell$           | $0.5 \mathrm{~m}$ | $0.5 \mathrm{~m}$ |
| $\tilde{\ell}$   | $0.25 \mathrm{~m}$ | $0.25 \mathrm{~m}$ |
| $\mathrm{m}$     | $2.6 \mathrm{~kg}$ | $2.6 \mathrm{~kg}$ |


## Output regulation

The DAR framework has been used to design a controller for addressing the output regulation problem.

The periodic trajectory is

$\begin{cases}
\omega_{\theta 1} & = \phantom{-}0.4 \cos (t)  \\ 
\omega_{\theta 2} & =-0.4 \cos (t)
\end{cases}$

Internal model matrices

$\Phi =
\begin{bmatrix}
    0 & I & 0 & 0 & 0 \\
    0 & 0 & I & 0 & 0 \\
    0 & 0 & 0 & I & 0 \\
    0 & 0 & 0 & 0 & I \\
    0 & -4f^5I & 0 & -5f^3I & 0
\end{bmatrix}
\quad
\Gamma =
\begin{bmatrix}
    0 \\ 0 \\ 0 \\ 0 \\ I
\end{bmatrix}$

Control gain

$\mathbf{K}=
\begin{bmatrix}
    -501.1559 & -51.6762 & -61.0883 & -13.9976 & -969.4690 & -58.9046 & -1045.9322 & 1.2262 & -3376.5894 & -224.8351 & -674.4890 & -3.7868 & -1028.4645 & 75.5330 \\
    60.5655 & -192.7508 & -12.5232 & -26.7909 & 194.7745 & -364.1018 & 374.1509 & -376.6959 & 624.2392 & -1274.0779 & 228.6800 & -244.2496 & 171.1357 & -390.1254
\end{bmatrix}
$

## Observer design

The DAR framework ahs been used to design a full-state observer.

Several gains for the observer have been designed, corresponding to different decay ratios $\tau_2$

$\tau_2=1 \quad K_{o,\theta} =
\begin{bmatrix}
37.7 & -0.2 \\
3.1 & 33.4
\end{bmatrix} \quad K_{0,\omega} =
\begin{bmatrix}
1177 & -54.3 \\
-22.6 & 1113
\end{bmatrix}$

$\tau_2=10 \quad K_{o,\theta} =
\begin{bmatrix}
40.5 & -0.4 \\
2.5 & 37
\end{bmatrix} \quad K_{0,\omega} =
\begin{bmatrix}
1221.7 & -71.2 \\
-27.2 & 1170
\end{bmatrix}$

$\tau_2=20 \quad K_{o,\theta} =
\begin{bmatrix}
37.8 & \phantom{-}0.2 \\
2.2 & 41.6
\end{bmatrix} \quad K_{0,\omega} =
\begin{bmatrix}
1810.5 & -125.7 \\
-96 & 1214.4
\end{bmatrix}$

## Saturation 

The DAR framewrok has been used to design a controller able to cope with saturation constraints in the joint actuators.

## Anti-windup

The DAR framewrok has been used to design an anti-windup compensatore to cope with the presence of saturation constraints in the joint actuators