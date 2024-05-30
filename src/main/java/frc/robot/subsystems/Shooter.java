package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.ShooterPosition.*;
import static frc.robot.Constants.ShooterConstants.SHOOTER_PIVOT_RATIO;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants.ShooterPosition;
import frc.robot.wrappers.GenericPID;

public class Shooter extends SubsystemBase{
    private CANSparkFlex shooterLeader;
    private CANSparkFlex shooterFollower;
    private GenericPID shooterPID;
    private GenericPID shooterFollowerPID;

    private CANSparkMax pivotLeader;
    private CANSparkMax pivotFollower;
    private GenericPID pivotPID;

    /** Constructs a shooter
     *  @param shooterRightID The ID of the right shooter motor from the perspective of the robot
     *  @param shooterLeftID The ID of the left shooter motor from the perspective of the robot
     *  @param pivotRightID The ID of the right shooter pivoting motor from the perspective of the robot
     *  @param pivotLeftID The ID of the left shooter pivoting motor from the perspective of the robot
     */
    public Shooter(int shooterRightID, int shooterLeftID, int pivotRightID, int pivotLeftID) {
        shooterLeader = new CANSparkFlex(shooterRightID, MotorType.kBrushless);
        shooterLeader.restoreFactoryDefaults();
        shooterLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        shooterFollower = new CANSparkFlex(shooterLeftID, MotorType.kBrushless);
        shooterFollower.restoreFactoryDefaults();
        shooterFollower.follow(shooterLeader, true);
        shooterFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        shooterFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        shooterFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

        shooterPID = new GenericPID(shooterLeader, ControlType.kVelocity, .00022, .0000005, 0);
        shooterPID.setInputRange(0, 6000);
        shooterFollowerPID = new GenericPID(shooterFollower, ControlType.kVelocity, .00022, .0000005, 0);

        pivotLeader = new CANSparkMax(pivotLeftID, MotorType.kBrushless);
        pivotLeader.restoreFactoryDefaults();
        pivotLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);

        pivotFollower = new CANSparkMax(pivotRightID, MotorType.kBrushless);
        pivotFollower.restoreFactoryDefaults();
        pivotFollower.follow(pivotLeader, true);
        pivotFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        pivotFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        pivotFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

        pivotPID = new GenericPID(pivotLeader, ControlType.kPosition, .2);
        pivotPID.setRatio(SHOOTER_PIVOT_RATIO);
        pivotPID.setInputRange(MINIMUM.getAngle(), MAXIMUM.getAngle());
        pivotPID.activate(POINT_BLANK.getAngle());
    }

    /** @return Whether the shooter has reached its target speed */
    public boolean atTargetSpeed() { return shooterPID.atSetpoint(200); }
    /** @return Whether the shooter has reached its target speed or surpassed it */
    public boolean atOrAboveTargetSpeed() { return shooterPID.getMeasurement() >= shooterPID.getSetpoint() || atTargetSpeed(); }
    /** @return Whether the shooter has reached its target position */
    public boolean atTargetPosition() { return pivotPID.atSetpoint(2); }
    /** @return The RPM of the shooter */
    public double getRPM() { return shooterPID.getMeasurement(); }
    /** @return The position of the shooter's pivot */
    public double getPosition() { return pivotPID.getMeasurement(); }
    /** @return Whether the shooter is ready to shoot */
    public boolean readyToShoot() { return atOrAboveTargetSpeed() && atTargetPosition() && shooterPID.getSetpoint() > 0;}

    /** Activates the shooter PID
     *  @param RPM The speed of the shooter in RPM */
    public void setSpeed(double RPM) {
        if (RPM == 0) {
            shooterPID.pause();
            shooterLeader.stopMotor();
        }
        else
            shooterPID.activate(RPM);
    }

    public Command increase() {
        return this.runOnce(() -> pivotPID.activate(pivotPID.getSetpoint()+1));
    }
    public Command decrease() {
        return this.runOnce(() -> pivotPID.activate(pivotPID.getSetpoint()-1));
    }
    
    /** Stops the shooter */
    public void stopShooter() {
        shooterPID.pause();
        shooterLeader.stopMotor();
    }
    
    /** Activates the shooter PID
     *  @param pos The desired position of the shooter */
    public void goToPosition(ShooterPosition pos) {
        pivotPID.activate(pos.getAngle()); 
    }

    /** Activates the shooter PID
     *  @param angle The desired position of the shooter in degrees */
    public void goToAngle(double angle ) {
        pivotPID.activate(angle); 
    }

    /** Stops the shooter */
    public void stopPivot() {
        pivotPID.pause();
        pivotLeader.stopMotor();
    }

    @Override // Called every 20ms
    public void periodic() {
        SmartDashboard.putNumber("Shooter RPM", getRPM());
        SmartDashboard.putNumber("Shooter Differential", shooterFollowerPID.getMeasurement() - shooterPID.getMeasurement());
        SmartDashboard.putBoolean("Shooter at Target Speed", atTargetSpeed());

        SmartDashboard.putNumber("Shooter Pivot Position", getPosition());
        SmartDashboard.putBoolean("Shooter at Target Position", atTargetPosition());

        SmartDashboard.putBoolean("Ready to Shoot", readyToShoot());
    }
}
