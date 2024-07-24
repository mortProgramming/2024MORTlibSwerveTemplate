package frc.robot.library.Hardware.Motor;

import edu.wpi.first.math.controller.PIDController;

public class PIDMotor extends Motor {

    public PIDController controller;

    public PIDMotor(MotorTypeEnum motorType, int motorID) {
        super(motorType, motorID);

        controller = new PIDController(0, 0, 0);
        controller.enableContinuousInput(-180, 180);
    }

    public void setPIDValues(double kP, double kI, double kD) {
        controller = new PIDController(kP, kI, kD);
    }

    public void setPIDEnableContinuousInput(double minimum, double maximum) {
        controller.enableContinuousInput(minimum, maximum);
    }

    public void setPIDDisableContinuousInput() {
        controller.disableContinuousInput();
    }

    public void setPIDTolerance(double position, double velocity) {
        controller.setTolerance(position, velocity);
    }

    public void setPositionD(double position, double setpoint) {
        super.setVoltage(controller.calculate(position, setpoint));
    }

    public PIDController getPIDController() {
        return controller;
    }

}