package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.INTAKE_PIVOT_RATIO;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants.IntakePosition;
import frc.robot.wrappers.GenericPID;
import frc.robot.wrappers.LimitSwitch;

public class Intake extends SubsystemBase{
    private CANSparkMax intakeMotor;
    private CANSparkMax rotationMotor;
    private GenericPID rotationPID;
    private LimitSwitch limitSwitch;

    /** Constructs an Intake
     *  @param intakeID The ID of the intake motor
     *  @param rotateID The ID of the rotate motor
     *  @param limPort  Limelight port
     */
    public Intake(int intakeID, int rotateID, int limPort){
        intakeMotor = new CANSparkMax(intakeID, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setInverted(true);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        
        rotationMotor = new CANSparkMax(rotateID, MotorType.kBrushless);
        rotationMotor.restoreFactoryDefaults();
        rotationMotor.setIdleMode(IdleMode.kBrake);
        rotationMotor.setInverted(false);
        rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
        rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        rotationPID = new GenericPID(rotationMotor, ControlType.kPosition, 0.075);
        rotationPID.setRatio(INTAKE_PIVOT_RATIO);

        limitSwitch = new LimitSwitch(limPort);
        limitSwitch.setInverted(true);
    }
    
    /** Intake stopped when holding a piece
     * @param speed
     */
    public void intake(double speed){  
        if(limitSwitch.isNotPressed()) 
            intakeMotor.set(-speed);
        else
            stopIntake();
    }
    
    /** Sets the intake to eject */
    public void eject(double speed){
        intakeMotor.set(speed);
    }

    /** Stops the rotation of the intake */
    public void stopRotate(){
        rotationPID.pause();
        rotationMotor.set(0);
    }

    /** Stops the Intake */ 
    public void stopIntake(){
        intakeMotor.set(0);
    }

    /** Intake goes to specified position */
    public void goTo(IntakePosition pos) {
        rotationPID.activate(pos.getAngle());
    }

    /** Manually rotates the Intake */
    public void manualRotate(double speed){
        rotationMotor.set(speed);
    }

    /** Intake is holding a piece */
    public boolean holdingPiece(){
        return limitSwitch.isPressed();
    }

    /** Intake at setpoint */
    public boolean intakeAtSetPoint(){
        return rotationPID.atSetpoint(3);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Position Degrees", rotationPID.getMeasurement());
        SmartDashboard.putBoolean("Intake Has Piece", holdingPiece());
    }
}
