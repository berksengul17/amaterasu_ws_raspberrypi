import numpy as np

class Kalman:
    def __init__(self):
        self.Xk = np.vstack((0.0, 0.0))  # State Matrix
        self.yaw = 0  # Initial Yaw
        self.Pk = np.array([0.1, 0.1]) * np.identity(2)  # Pk
        self.Q11 = 0.002  # Q11
        self.Q22 = 0.005  # Q22
        self.R = 0.03  # R

    def update_yaw_angle(self, input_vector_variable, dt, measuredYaw):
        self.yaw, self.Xk, self.Pk = self.update(
            self.Xk,
            measuredYaw,
            self.Pk,
            self.Q11,
            self.Q22,
            self.R,
            input_vector_variable,
            dt
        )

    def update(self, Xk_prev, measurement, Pk_prev, error,
               driftError, MeasurementUncertainty, input_vector_variable, dt):
        # State Transition Matrix
        StateTransitionMatrix = np.array([[1, -dt], [0, 1]])  # A

        # Control Matrix
        ControlMatrix = np.vstack((input_vector_variable, 0.0))  # B

        # Process Noise
        ProcessNoise = dt * (np.array([error, driftError]) * np.identity(2))  # Q

        # Observation Matrix
        ObservationMatrix = np.array([1.0, 0.0])  # H

        # Prediction
        DynamicModel = np.matmul(StateTransitionMatrix, Xk_prev) + dt * ControlMatrix
        PredictorCovariance = np.matmul(
            np.matmul(StateTransitionMatrix, Pk_prev),
            StateTransitionMatrix.T
        ) + ProcessNoise

        # Measurement
        Innovation = self.normalize_angle(measurement - np.matmul(ObservationMatrix, DynamicModel))
        InnovationCovariance = np.matmul(
            np.matmul(ObservationMatrix, PredictorCovariance),
            ObservationMatrix.T
        ) + MeasurementUncertainty

        KalmanGain = np.matmul(
            PredictorCovariance, np.vstack((1.0, 0.0))
        ) / InnovationCovariance

        # Update
        StateUpdate = DynamicModel + KalmanGain * Innovation
        CovarianceUpdate = np.matmul(
            np.identity(2) - np.matmul(KalmanGain, ObservationMatrix.reshape((1, 2))),
            PredictorCovariance
        )

        # Normalize the updated yaw angle
        StateUpdate[0, 0] = self.normalize_angle(StateUpdate[0, 0])

        return StateUpdate[0, 0], StateUpdate, CovarianceUpdate
    
    def normalize_angle(self, angle):
        """Normalize angle to the range -180° to 180°."""
        return (angle + 180) % 360 - 180


    @property
    def yaw(self):
        return self._yaw

    @yaw.setter
    def yaw(self, yaw):
        self._yaw = yaw
