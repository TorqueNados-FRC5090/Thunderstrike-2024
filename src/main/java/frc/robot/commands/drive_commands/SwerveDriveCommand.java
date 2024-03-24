package frc.robot.commands.drive_commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.DriveConstants.*;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

import java.util.function.DoubleSupplier;

/** Drives the robot */
public class SwerveDriveCommand extends Command {
    // Declare variables that will be initialized by the constructor
    private final SwerveDrivetrain drivetrain;
    private final DoubleSupplier inputY;
    private final DoubleSupplier inputX;
    private final DoubleSupplier inputRot;
    private final SlewRateLimiter slewX = new SlewRateLimiter(TRANSLATION_SLEW);
    private final SlewRateLimiter slewY = new SlewRateLimiter(TRANSLATION_SLEW);
    private final SlewRateLimiter slewRot = new SlewRateLimiter(ROTATION_SLEW);

    /** 
     * Constructs a DriveCommand
     *  
     * @param drivetrain The robot's drivetrain
     * @param translationInputX The left/right translation instruction
     * @param translationInputY The forward/back translation instruction
     * @param rotation The rotational instruction
    */
    public SwerveDriveCommand(SwerveDrivetrain drivetrain, DoubleSupplier translationInputX, DoubleSupplier translationInputY, DoubleSupplier rotationInput) {

        // Initialize internal variables with values passed through params
        this.drivetrain = drivetrain;
        this.inputX = translationInputX;
        this.inputY = translationInputY;
        this.inputRot = rotationInput;
        
        // Tell the CommandBase that this command uses the drivetrain
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double x = slewX.calculate(drivetrain.deadbandAndSquare(inputX.getAsDouble(), TRANSLATION_DEADBAND));
        double y = slewY.calculate(drivetrain.deadbandAndSquare(inputY.getAsDouble(), TRANSLATION_DEADBAND));
        double rot = slewRot.calculate(drivetrain.deadbandAndSquare(inputRot.getAsDouble(), ROTATION_DEADBAND));

        drivetrain.drive(x, y, rot);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Has no end condition
    }
}
