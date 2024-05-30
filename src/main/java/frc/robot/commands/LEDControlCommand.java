package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.LEDConstants.LEDColor;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Candle;

public class LEDControlCommand extends Command {
    private final Intake intake;
    private final Shooter shooter;
    private final Limelight shooterLimelight;
    private final Candle candle;
    public LEDControlCommand(Candle candle, RobotContainer robot) {
        intake = robot.intake;
        shooter = robot.shooter;
        shooterLimelight = robot.shooterLimelight;
        this.candle = candle;

        addRequirements(candle);
    }

    @Override
    public void initialize() {
    // This command should turn on the LED at the start of the game
        
    }
    
    @Override
    public void execute() {
        // This command should change colors for if there is an object
        if (intake.holdingPiece() && shooterLimelight.getTargetY() <= -15.5)
            candle.setAll(LEDColor.RED);
        else if (intake.holdingPiece() && shooter.readyToShoot() && shooterLimelight.hasValidTarget() && Math.abs(shooterLimelight.getTargetX()) < 5)
            candle.setAll(LEDColor.GREEN);
        else if (intake.holdingPiece())
            candle.setAll(LEDColor.ORANGE);
        else candle.setAll(LEDColor.BLUE);
    }
    @Override
    public boolean isFinished(){
        // This command should turn off the LED at the end of the game
        return false;
    }
}