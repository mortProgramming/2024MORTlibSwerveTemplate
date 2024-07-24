package frc.robot.library.Hardware.Brands.CTRE;

import frc.robot.library.Hardware.Motor.MotorIntf;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class TalonFXMotor implements MotorIntf {

    public int ID;

    public TalonFX motor;

    public TalonFXConfiguration config;

    public TalonFXMotor(int ID) {
        this.ID = ID;

        motor = new TalonFX(ID);
        config = new TalonFXConfiguration();

        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;

        motor.getConfigurator().apply(config);
    }

    public void setCurrentLimit(double limit) {
        config.CurrentLimits.SupplyCurrentLimit = limit;
        motor.getConfigurator().apply(config);
    }

    public void setDirection(boolean direction) {
        motor.setInverted(direction);
    }

    public void setPIDValues(double kP, double kI, double kD) {
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        motor.getConfigurator().apply(config);
    }

    public void setPercent(double percent) {
        motor.set(percent);
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    public void setPositionD(double position, double setpoint) {
        //do
    }



    public double getPositionD() {
        return motor.getPosition().getValueAsDouble() * 360;
    }

    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    public double getVelocityD() {
        return motor.getVelocity().getValueAsDouble() * 360;
    }

    public double getVelocity1() {
        return motor.getVelocity().getValueAsDouble();
    }

    public TalonFX getMotor() {
        return motor;
    }
 
}