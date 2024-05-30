package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.ClimberConstants.ClimberPosition.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants.ClimberPosition;
import frc.robot.wrappers.GenericPID;

public class Climber extends SubsystemBase {
    private CANSparkMax leaderMotor;
    private CANSparkMax followerMotor;
    private GenericPID climberPID;

    /** Constructs a climber
     *  @param rightMotorID The ID of the right climbing motor from the perspective of the robot
     *  @param rightMotorID The ID of the left climbing motor from the perspective of the robot */
    public Climber(int rightMotorID, int leftMotorID) {
        leaderMotor = new CANSparkMax(leftMotorID, MotorType.kBrushless);
        leaderMotor.restoreFactoryDefaults();
        leaderMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, leftMotorID);

        followerMotor = new CANSparkMax(rightMotorID, MotorType.kBrushless);
        followerMotor.restoreFactoryDefaults();
        followerMotor.follow(leaderMotor, true);
        followerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        followerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        followerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

        climberPID = new GenericPID(leaderMotor, ControlType.kPosition, .15);
        climberPID.setRatio(CLIMBER_RATIO);
        climberPID.setInputRange(MINIMUM.getHeight(), MAXIMUM.getHeight());
    }

    /** @return How far the climber is extended in inches */
    public double getHeight() { return climberPID.getMeasurement(); }
    /** @return Whether the climber is at it's setpoint */
    public boolean atSetpoint() { return climberPID.atSetpoint(.5); } 

    /** Moves the climber to a given height in inches
     *  @param pos The desired position of the climber */
    public void goToPosition(ClimberPosition pos) {
        climberPID.activate(pos.getHeight());
    }
    /** Stops the climber from moving */
    public void stop() { 
        climberPID.pause(); 
        leaderMotor.stopMotor(); 
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Height", getHeight());
        SmartDashboard.putBoolean("Climber at Setpoint", atSetpoint());
    }
}
