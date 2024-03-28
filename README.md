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

$$
\begin{cases}
\omega_{\theta 1} & = 0.4 \cos (t)  \\ 
\omega_{\theta 2} & =-0.4 \cos (t)
\end{cases}
$$

Internal model matrices considering two harmonics

$$\Phi =
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
\end{bmatrix}$$

Internal model matrices considering three harmonics

$$\Phi =
\begin{bmatrix}
    0 & I & 0 & 0 & 0 & 0 & 0 \\
    0 & 0 & I & 0 & 0 & 0 & 0 \\
    0 & 0 & 0 & I & 0 & 0 & 0 \\
    0 & 0 & 0 & 0 & I & 0 & 0 \\
    0 & 0 & 0 & 0 & 0 & I & 0 \\
    0 & 0 & 0 & 0 & 0 & 0 & I \\
    0 & -36f^7I & 0 & -49f^5I & 0 & -14f^3I & 0
\end{bmatrix}
\quad
\Gamma =
\begin{bmatrix}
    0 \\ 0 \\ 0 \\ 0 \\ 0 \\ 0 \\ I
\end{bmatrix}$$

Control gain

$$\mathbf{K}=
\begin{bmatrix}
    -501.1559 & -51.6762 & -61.0883 & -13.9976 & -969.4690 & -58.9046 & -1045.9322 & 1.2262 & -3376.5894 & -224.8351 & -674.4890 & -3.7868 & -1028.4645 & 75.5330 \\
    60.5655 & -192.7508 & -12.5232 & -26.7909 & 194.7745 & -364.1018 & 374.1509 & -376.6959 & 624.2392 & -1274.0779 & 228.6800 & -244.2496 & 171.1357 & -390.1254
\end{bmatrix}
$$

## Observer design

The DAR framework ahs been used to design a full-state observer.

Several gains for the observer have been designed, corresponding to different decay ratios $\tau_2$

$$\tau_2=1 \quad K_{o,\theta} =
\begin{bmatrix}
37.7 & -0.2 \\
3.1 & 33.4
\end{bmatrix} \quad K_{0,\omega} =
\begin{bmatrix}
1177 & -54.3 \\
-22.6 & 1113
\end{bmatrix}$$

$$\tau_2=10 \quad K_{o,\theta} =
\begin{bmatrix}
40.5 & -0.4 \\
2.5 & 37
\end{bmatrix} \quad K_{0,\omega} =
\begin{bmatrix}
1221.7 & -71.2 \\
-27.2 & 1170
\end{bmatrix}$$

$$\tau_2=20 \quad K_{o,\theta} =
\begin{bmatrix}
37.8 & 0.2 \\
2.2 & 41.6
\end{bmatrix} \quad K_{0,\omega} =
\begin{bmatrix}
1810.5 & -125.7 \\
-96 & 1214.4
\end{bmatrix}$$

## Saturation 

The DAR framewrok has been used to design a controller able to cope with saturation constraints in the joint actuators.

## Anti-windup

The DAR framewrok has been used to design an anti-windup compensatore to cope with the presence of saturation constraints in the joint actuators


|                   |                      | Internal   | Model      |
|-------------------|----------------------|------------|------------|
| Optimization      | Decay ratio $\tau_2$ | IM2        | IM3        |
|                   | $\tau_2=0.5$         |$\checkmark$|$\checkmark$|
| OPT1              | $\tau_2=1$           |$\checkmark$|$\checkmark$|
|                   | $\tau_2=1.5$         |$\checkmark$|$\checkmark$|
|                   | $\tau_2=0.5$         |$\checkmark$|$\checkmark$|
| OPT2              | $\tau_2=1$           |$\checkmark$|$\checkmark$|
|                   | $\tau_2=1.5$         |$\checkmark$|$\checkmark$|

## IM2 DC 0.5 OPT 1

$$\eta(0) = 
\begin{bmatrix}
    0.07 & -0.11 & -0.05 & 0.04 & -0.10 & 0.07 & 0.05 & -0.04 & 0.10 & -0.07
\end{bmatrix}^T$$

$$E1 = 
\begin{bmatrix}
    -1.22 \times 10^{-6} & -9.59 \times 10^{-7} \\
    -3.00 \times 10^{-6} & -2.41 \times 10^{-6} \\
    -1.08 \times 10^{-6} & -3.66 \times 10^{-7} \\
    -9.91 \times 10^{-6} & -9.77 \times 10^{-6} \\
    7.20 \times 10^{-5} & 3.92 \times 10^{-5} \\
    0.00022 & 0.00022 \\
    -0.00078 & -0.00024 \\
    -0.00153 & -0.00197 \\
    0.00394 & -2.26 \times 10^{-5} \\
    0.00418 & 0.00959 \\
\end{bmatrix}$$

$$K = 
\begin{bmatrix}
    -627.03 & -75.32 & -95.96 & -21.19 & -1421.04 & -50.64 & -1629.36 & 37.63 & -4915.79 & -205.74 & -1042.28 & 17.64 & -1486.00 & -73.17 \\
    151.44 & -268.39 & -10.15 & -48.99 & 583.93 & -675.21 & 980.21 & -764.13 & 1910.31 & -2343.39 & 603.41 & -490.60 & 539.16 & -710.69 \\
\end{bmatrix}$$

## IM2 DC 1 OPT 1

$$\eta(0) = 
\begin{bmatrix}
    0.02 & -0.03 & -0.03 & 0.02 & -0.03 & 0.02 & 0.03 & -0.02 & 0.03 & -0.02 \\
\end{bmatrix}^T$$

$$E1 = 
\begin{bmatrix}
    -5.62 \times 10^{-7} & -4.85 \times 10^{-7} \\
    -6.21 \times 10^{-7} & -2.90 \times 10^{-7} \\
    -3.17 \times 10^{-6} & -1.44 \times 10^{-6} \\
    -2.02 \times 10^{-5} & -1.96 \times 10^{-5} \\
    6.87 \times 10^{-5} & 3.22 \times 10^{-5} \\
    2.41 \times 10^{-4} & 2.44 \times 10^{-4} \\
    -6.78 \times 10^{-4} & -1.73 \times 10^{-4} \\
    -1.49 \times 10^{-3} & -1.92 \times 10^{-3} \\
    3.54 \times 10^{-3} & -1.72 \times 10^{-4} \\
    4.11 \times 10^{-3} & 8.99 \times 10^{-3} \\
\end{bmatrix}$$

$$K = 
\begin{bmatrix}
    -900.93 & -136.17 & -116.94 & -26.20 & -5578.43 & -251.31 & -10234.08 & -291.45 & -15360.69 & -886.18 & -5434.65 & -196.42 & -3521.00 & -264.00 \\
    145.71 & -379.55 & -11.66 & -54.79 & 2236.31 & -2301.96 & 4642.54 & -4176.21 & 5476.13 & -6426.05 & 2311.47 & -2240.48 & 1047.85 & -1497.09 \\
\end{bmatrix}$$

##  IM2 DC 1.5 OPT 1

$$\eta(0) = 
\begin{bmatrix}
    0.01 & -0.01 & -0.02 & 0.01 & -0.01 & 0.00 & 0.02 & -0.01 & 0.01 & -0.00 \\
\end{bmatrix}^T$$

$$E1 = 
\begin{bmatrix}
    -1.84 \times 10^{-7} & -2.40 \times 10^{-7} \\
    8.23 \times 10^{-7} & 9.09 \times 10^{-7} \\
    -4.21 \times 10^{-6} & -1.72 \times 10^{-6} \\
    -2.57 \times 10^{-5} & -2.45 \times 10^{-5} \\
    6.54 \times 10^{-5} & 2.60 \times 10^{-5} \\
    2.51 \times 10^{-4} & 2.56 \times 10^{-4} \\
    -6.14 \times 10^{-4} & -1.22 \times 10^{-4} \\
    -1.49 \times 10^{-3} & -1.90 \times 10^{-3} \\
    3.30 \times 10^{-3} & -2.95 \times 10^{-4} \\
    4.22 \times 10^{-3} & 8.91 \times 10^{-3} \\
\end{bmatrix}$$

$$k = 
\begin{bmatrix}
    -1212.94 & -208.28 & -134.00 & -30.98 & -17376.43 & -1068.73 & -33308.19 & -1919.71 & -36679.94 & -2855.19 & -14047.65 & -974.86 & -6115.78 & -632.37 \\
    131.25 & -492.06 & -13.38 & -58.21 & 6738.73 & -6473.14 & 13271.45 & -12387.88 & 12114.48 & -13978.46 & 4998.01 & -5319.45 & 1490.10 & -2402.84 \\
\end{bmatrix}

