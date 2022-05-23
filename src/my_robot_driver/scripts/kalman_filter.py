import numpy as np

# offsets of each variable in the state vector
iX = 0
iV = 1
NUMVARS = iV + 1


class KF:
    def __init__(self, initial_x: float, initial_v: float, accel_variance: float) -> None:
        # mean of state GRV
        self._x = np.array([initial_x, initial_v])
        self._accel_variance = accel_variance

        # covariance of state GRV
        self._P = np.eye(2)
    
    def predict(self, dt: float) -> None:
        # x = F * x
        # P = F * P * F.T + G * Gt * a
        F = np.array([[1, dt], [0,1]])
        new_x = F.dot(self._x)

        G = np.array([0.5*dt**2, dt].reshape(2, 1))
        new_P = F.dot(self._P).dot(F.T) + G.dot(G.T) * self._accel_variance 

        self._P = new_P
        self._x = new_x

    def update(self, meas_value: float, meas_variance: float):
        H = np.array([1, 0]).reshape(1, 2)

        z = np.array([meas_value])
        R = np.array([meas_variance])

        y = z - H.dot(self._x)
        S = H.dot(self._P).dot(H.T) + R

        K = self._P.dot(H.T).dot(np.linalg.inv(S))

        new_x = self._x + K.dot(y)
        new_P = (np.eye(2) - K.dot(H)).dot(self._P)

        self._P = new_P
        self._x = new_x

    @property
    def cov(self) -> np.array:
        return self._P

    @property
    def mean(self) -> np.array:
        return self._x

    @property
    def pos(self) -> float:
        return self._x[0]
    
    @property
    def vel(self) -> float:
        return self._x[1]