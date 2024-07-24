package frc.robot.library.Hardware.Motor;

import frc.robot.library.Hardware.Brands.CTRE.TalonFXMotor;
import frc.robot.library.Hardware.Brands.REV.CANSparkMaxMotor;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Motor implements MotorIntf {

    public MotorTypeEnum motorType;
    public int ID;
    public CANSparkLowLevel.MotorType brushType;
    public boolean direction;

    public MotorIntf motor;
    
    public Motor(MotorTypeEnum motorType, int ID) {
        this(motorType, ID, MotorType.kBrushless);
    }
    
    public Motor(MotorTypeEnum motorType, int ID, CANSparkLowLevel.MotorType brushType) {
        this.motorType = motorType;
        this.ID = ID;
        this.brushType = brushType;

        switch (motorType) {
            case NEO:
                motor = new CANSparkMaxMotor(ID, brushType);
                break;

            case NEO550:
                motor = new CANSparkMaxMotor(ID, brushType);
                break;

            case FALCON:
                motor = new TalonFXMotor(ID);
                break;

            case KRAKEN:
                motor = new TalonFXMotor(ID);
                break;
        }
    }

    public void setCurrentLimit(double limit) {
        motor.setCurrentLimit(limit);
    }

    public void setDirection(boolean direction) {
        motor.setDirection(direction);
    }

    public void setPIDValues(double kP, double kI, double kD) {
        motor.setPIDValues(kP, kI, kD);
    }

    public void setPercent(double percent) {
        motor.setPercent(percent);
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    public void setPositionD(double position, double setpoint) {
        motor.setPositionD(position, setpoint);
    }

    public void setCanivore(String canivore) {
        motor.setCanivore(canivore);
    }


    
    public double getPositionD() {
        return motor.getPositionD();
    }

    public double getPosition() {
        return motor.getPosition();
    }

    public double getVelocityD() {
        return motor.getVelocityD();
    }

    public double getVelocity1() {
        return motor.getVelocity1();
    }

    public MotorIntf getMotor() {
        return motor;
    }

    public MotorTypeEnum getMotorType() {
        return motorType;
    }

}