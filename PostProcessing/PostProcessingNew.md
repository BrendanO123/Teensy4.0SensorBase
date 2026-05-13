# Measurements and Constants
## Measurements
| Variable | Sensor | Explanation |
|----|---|---|
| $\alpha = \frac{d\theta}{dt}$ | Gryo (IMU) | The time rate of change of the yaw heading |
| $\theta$ | IMU DMP | The yaw heading |
| $\psi$ | Stepper | The angle of the lidar module as it sweeps side to side |
| $Po$ | GPS | The position of the boat |
| $V$ | GPS | The speed and direction of the boat |
| $a$ | IMU | The acceleration on the boat |
| $pr$ | IMU | The pressure of the air |
| $T$ | IMU | The outside temperature |
accuracy for accel, quat, and gps

# Filtering
In order to reduce noise in our data and ensure optimal estimates of the system, we will use an **Extended Kalman Filter (EKF)** to transform our data points and provide accurate confidence values for the points, which we will use later. The EKF relies on a model of how the system changes over time, which it then uses to predict sensor measurements. It then treats both the prediction and measurement, with their respective errors, as normally distributed random variables, and calculates their intersection probability distribution to find the optimal prediction and confidence.

## EKF Basics
### Initialization
| Variable | Explanation | Initial State |
|---------|-----|----------|
| $\widehat{x}_0$ | Initial state of measured variable | From sensor |
| $P_0$ | Initial Variance | $1$ or $\mathbf I$ (the identity matrix) if a vector variable |
| $Q$ | The "Process Noise" variance (prediction model variance) | Constant Parameter |
| $R$ | The measurement noise variance | Constant Parameter |
| $f()$ | Transformation Function | |
| z | The Measurement from the sensor |
| K | The Kalman Gain Constant |


### Prediction
$$\widehat{x}_1^{-} = f(\widehat{x}_0)$$
$$\widehat{P}_1^{-} = A^2\widehat{P}_0 + Q$$
$$A = f'(\widehat{x}_0)$$

### Correction
$$K_1 = \frac{\widehat{P}_1^{-}}{\widehat{P}_0 + R}$$
$$\widehat{x}_1 = \widehat{x}_1^{-}(1-K_1) + z_1K_1$$
$$\widehat{P}_1 = (1-K_1)\widehat{P}_1^{-}$$

### Iteration
$$\widehat{x}_0 = \widehat{x}_1$$
$$\widehat{P}_0 = \widehat{P}_1$$

## Prediction Step of EFK

| Variable | Heading | Pan | Position | Distance |
|-------------|---------|-----|----------|----------|
| First Order | $\large{\widehat{\alpha}_1^{-}=\frac{d\widehat{\alpha}}{dt}dt+\widehat{\alpha}_0}$ | $\large{\widehat{\psi}_1^{-}=\frac{d\widehat{\psi}}{dt}dt+\widehat{\psi}_0}$ | $\large{\widehat{a}_1^{-}=\frac{d\widehat{a}}{dt}dt+\widehat{a}_0}$ | $\large{\widehat{D}_1^{-}=\frac{d\widehat{D}}{dt}dt+\widehat{D}_0}$ |
| | A: $\large{\frac{d\widehat{\alpha}}{dt}}$; ($\alpha = \frac{d\theta}{dt}$) | A: $\large{\frac{d\widehat{\psi}}{dt}}$ | A: $\large{\frac{d\widehat{a}}{dt}}$ | |
| | | | | |
| Second Order | $\large{\widehat{\theta}_1^{-}=\widehat{\alpha}_0dt+\widehat{\theta}_0}$ | | $\large{\widehat{V}_1^{-} = \begin{bmatrix}cos(\widehat{\alpha}_0dt), sin(\widehat{\alpha}_0dt) \cr -sin(\widehat{\alpha}_0dt), cos(\widehat{\alpha}_0dt)\end{bmatrix}\vec{\widehat{V}_0} + \widehat{a}_0dt(\frac{\vec{\widehat{V}_0}}{\lvert\lvert\vec{\widehat{V}_0}\rvert\rvert})}$ | |
| | A: $\large{\widehat{\alpha}}$ | | A: $\large{\begin{bmatrix}cos(\widehat{\alpha}_0dt), sin(\widehat{\alpha}_0dt) \cr -sin(\widehat{\alpha}_0dt), cos(\widehat{\alpha}_0dt)\end{bmatrix}}$ | |
| | | | |
| Third Order | | | $\large{\widehat{P}_1^{-}=\widehat{V}_0dt+\widehat{P}_0}$ | |
| | | | | $A: \widehat{V}_0$ |