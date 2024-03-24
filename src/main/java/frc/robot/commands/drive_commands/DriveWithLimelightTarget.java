package frc.robot.commands.drive_commands;

// Imports
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.DoubleSupplier;

/** Drives the robot while locking heading to a limelight's target */
public class DriveWithLimelightTarget extends Command {
    // Declare variables that will be initialized by the constructor
    private final SwerveDrivetrain drivetrain;
    private final Limelight limelight;
    private final DoubleSupplier inputX;
    private final DoubleSupplier inputY;
    private final DoubleSupplier inputRot;
    private final boolean ends;
    private final PIDController headingController;

    private final SlewRateLimiter slewX = new SlewRateLimiter(TRANSLATION_SLEW);
    private final SlewRateLimiter slewY = new SlewRateLimiter(TRANSLATION_SLEW);
    private final SlewRateLimiter slewRot = new SlewRateLimiter(ROTATION_SLEW);
    
    /** Constructs a DriveWithLimelightTarget command
     *  @param drivetrain The robot's drivetrain 
     *  @param limelight The limelight that the robot will get targets from
     *  @param translationInputX The joystick that will be used to control horizontal movement
     *  @param translationInputY The joystick that will be used to control forward and backward movement
     *  @param backupRotationInput The joystick that will be used to control rotation if limelight has no target */
    public DriveWithLimelightTarget(
            SwerveDrivetrain drivetrain, Limelight limelight,
            DoubleSupplier translationInputX, DoubleSupplier translationInputY, DoubleSupplier backupRotationInput, boolean ends) {

        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.inputX = translationInputX;
        this.inputY = translationInputY;
        this.inputRot = backupRotationInput;
        this.ends = ends;
        this.headingController = new PIDController(.008, 0, .0025);
        
        addRequirements(drivetrain);
    }

    @Override // Configure the PID controller
    public void initialize() {
        headingController.setTolerance(1); // Allow for 1 degree of rotational error
        headingController.enableContinuousInput(-180, 180); // -180 and 180 are the same heading
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Get the x and y values to drive with
        double x = slewX.calculate(drivetrain.deadbandAndSquare(inputX.getAsDouble(), TRANSLATION_DEADBAND));
        double y = slewY.calculate(drivetrain.deadbandAndSquare(inputY.getAsDouble(), TRANSLATION_DEADBAND));
        double rotation = slewRot.calculate(drivetrain.deadbandAndSquare(inputRot.getAsDouble(), ROTATION_DEADBAND));

        // Calculate the rotation instruction if limelight has a target
        if (limelight.hasValidTarget()) {
            rotation = headingController.calculate(limelight.getTargetX(), 0);

            drivetrain.drive(x, y, rotation);
        }
        else {
            drivetrain.drive(x, y, rotation);
        }        
    }

    // Command ends when robot is done rotating
    @Override
    public boolean isFinished() {
        return ends && headingController.atSetpoint();
    }

    // Runs when the command ends
    @Override
    public void end(boolean interrupted) {}
}
