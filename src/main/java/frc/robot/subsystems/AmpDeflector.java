package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.wrappers.GenericPID;

import static frc.robot.Constants.AmpDeflectorConstants.DEFLECTOR_RATIO;
import static frc.robot.Constants.AmpDeflectorConstants.DeflectorPosition;
import static frc.robot.Constants.AmpDeflectorConstants.DeflectorPosition.*;

public class AmpDeflector extends SubsystemBase{

    private CANSparkMax rotationMotor;
    private GenericPID ampPID;

    public AmpDeflector(int rotationID){
        rotationMotor = new CANSparkMax(rotationID, MotorType.kBrushless);
        rotationMotor.restoreFactoryDefaults();
        rotationMotor.setInverted(true);
        rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
        rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);

        ampPID = new GenericPID(rotationMotor, ControlType.kPosition, 0.035);
        ampPID.activate(0);
        ampPID.setRatio(DEFLECTOR_RATIO);
        ampPID.setInputRange(MINIMUM.getAngle(), MAXIMUM.getAngle());
    }

    public double getPostion() { return ampPID.getMeasurement();}

    public void goToPosition(DeflectorPosition pos) { ampPID.activate(pos.getAngle()); }
    public void goToPositionDeg(double pos) { ampPID.activate(pos); }

    public void stop(){
        ampPID.pause();
    }

    public Command deflectorOut() {
        return this.runEnd(() -> goToPosition(AMP_SHOT), () -> goToPosition(MINIMUM));
    }

    @Override // Called every 20ms
    public void periodic(){
        SmartDashboard.putNumber("Deflector Position", getPostion());
    }
}
