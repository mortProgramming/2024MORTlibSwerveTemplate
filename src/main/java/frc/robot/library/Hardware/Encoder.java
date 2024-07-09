package frc.robot.library.Hardware;

import frc.robot.library.Hardware.ctre.CANCoderEncoder;

import edu.wpi.first.math.geometry.Rotation2d;

public class Encoder implements EncoderIntf {

    public EncoderTypeEnum encoderType;
    public int ID;

    public EncoderIntf encoder;
    
    public Encoder(EncoderTypeEnum encoderType, int ID) {
        this.encoderType = encoderType;
        this.ID = ID;

        switch (encoderType) {
            case CANCODER:
                encoder = new CANCoderEncoder(ID);
                break;
        }
    }

    public Rotation2d getPosition() {
        return encoder.getPosition();
    }

    public double getPositionD() {
        return encoder.getPositionD();
    }

    public double getPositionR() {
        return encoder.getPositionR();
    }

    public EncoderIntf getEncoder() {
        return encoder;
    }

    public EncoderTypeEnum getMotorType() {
        return encoderType;
    }

}