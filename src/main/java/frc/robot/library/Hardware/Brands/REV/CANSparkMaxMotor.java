package frc.robot.library.Hardware.Brands.REV;

import com.revrobotics.CANSparkMax;
import frc.robot.library.Hardware.Motor.MotorIntf;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkPIDController;

public class CANSparkMaxMotor implements MotorIntf {

    public int ID;
    public CANSparkLowLevel.MotorType brushType;

    public CANSparkMax motor;
    public SparkPIDController controller;

    // CANSparkLowLevel.MotorType.kBrushless
    public CANSparkMaxMotor(int ID, CANSparkLowLevel.MotorType brushType) {
        this.ID = ID;
        this.brushType = brushType;

        motor = new CANSparkMax(ID, brushType);
        controller = motor.getPIDController();

        controller.setP(0, 0);
        controller.setI(0, 0);
        controller.setD(0, 0);
    }

    public void setCurrentLimit(double limit) {
        motor.setSecondaryCurrentLimit(limit);
    }

    public void setDirection(boolean direction) {
        motor.setInverted(direction);
    }

    public void setPIDValues(double kP, double kI, double kD) {
        controller.setP(kP);
        controller.setI(kI);
        controller.setD(kD);
    }

    public void setPercent(double percent) {
        motor.set(percent);
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    public void setPositionD(double positon, double setpoint) {
        // do later
    }

    public void setCanivore(String canivore) {
        System.out.println("Why are you here?");
    }

    

    public double getPositionD() {
        return motor.getEncoder().getPosition() * 360;
    }

    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    public double getVelocityD() {
        return motor.getEncoder().getVelocity() * 360;
    }

    public double getVelocity1() {
        return motor.getEncoder().getVelocity();
    }

    public CANSparkMax getMotor() {
        return motor;
    }
 
}
