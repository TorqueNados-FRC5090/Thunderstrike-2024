package frc.robot.commands.drive_commands;

// Imports
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import java.util.function.DoubleSupplier;

/** Drives the robot while locking heading to a limelight's target */
public class DriveWithLimelightTarget extends Command {
    // Declare variables that will be initialized by the constructor
    private final SwerveDrivetrain drivetrain;
    private final Limelight limelight;
    private final DoubleSupplier inputX;
    private final DoubleSupplier inputY;
    private final DoubleSupplier inputRot;
    private final PIDController headingController;
    
    /** Constructs a DriveWithLimelightTarget command
     *  @param drivetrain The robot's drivetrain 
     *  @param limelight The limelight that the robot will get targets from
     *  @param translationInputX The joystick that will be used to control horizontal movement
     *  @param translationInputY The joystick that will be used to control forward and backward movement
     *  @param backupRotationInput The joystick that will be used to control rotation if limelight has no target */
    public DriveWithLimelightTarget(
            SwerveDrivetrain drivetrain, Limelight limelight,
            DoubleSupplier translationInputX, DoubleSupplier translationInputY, DoubleSupplier backupRotationInput) {

        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.inputX = translationInputX;
        this.inputY = translationInputY;
        this.inputRot = backupRotationInput;
        this.headingController = new PIDController(.009, 0, .00315);
        
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
        double x = inputX.getAsDouble();
        double y = inputY.getAsDouble();
        double rotation = inputRot.getAsDouble();

        // Calculate the rotation instruction if limelight has a target
        if (limelight.hasValidTarget()) {
            x = drivetrain.preprocessX(x);
            y = drivetrain.preprocessY(y);
            rotation = headingController.calculate(limelight.getTargetX(), 0);

            drivetrain.sendDrive(x, y, rotation, true);
        }
        else {
            drivetrain.drive(x, y, rotation);
        }        
    }

    // Command ends when robot is done rotating
    @Override
    public boolean isFinished() {
        return headingController.atSetpoint();
    }

    // Runs when the command ends
    @Override
    public void end(boolean interrupted) {}
}
