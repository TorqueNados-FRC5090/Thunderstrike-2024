package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.ShooterPosition;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class AimShooterAtSpeaker extends Command {
    private final Shooter shooter;
    private final Limelight limelight;
    public AimShooterAtSpeaker(Limelight limelight, Shooter shooter) {
        this.limelight = limelight;
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void initialize() { }
    
    @Override
    public void execute() {
        if (limelight.hasValidTarget()) {
            shooter.goToAngle(-0.7634 * limelight.getTargetY() + 14.723);
            shooter.setSpeed(5000);
        }
        else {
            shooter.goToPosition(ShooterPosition.POINT_BLANK);
            shooter.setSpeed(3500);
        }
    }

    @Override
    public void end(boolean interrupted) {}
    
    @Override
    public boolean isFinished(){
        // This command should turn off the LED at the end of the game
        return shooter.readyToShoot();
    }
}