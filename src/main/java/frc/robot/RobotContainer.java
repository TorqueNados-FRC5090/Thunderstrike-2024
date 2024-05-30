package frc.robot;

// Import constants
import static frc.robot.Constants.ControllerPorts.*;
import static frc.robot.Constants.LEDPorts.*;
import static frc.robot.Constants.IntakeIDs.*;
import static frc.robot.Constants.ShooterIDs.*;
import static frc.robot.Constants.ClimberIDs.*;
import static frc.robot.Constants.AmpDeflectorIDs.*;

import frc.robot.Constants.ClimberConstants.ClimberPosition;
import frc.robot.Constants.IntakeConstants.IntakePosition;
import frc.robot.Constants.ShooterConstants.ShooterPosition;
// Command imports
import frc.robot.commands.*;
import frc.robot.commands.drive_commands.*;
import frc.robot.commands.intake_commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// Subsystem imports
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.*;
import edu.wpi.first.wpilibj.DriverStation;
// Other imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_PORT);

    public final SwerveDrivetrain drivetrain = new SwerveDrivetrain();
    public final Intake intake = new Intake(INTAKE_DRIVER_ID, INTAKE_ROTATOR_ID, INTAKE_LIMIT_ID);
    public final Shooter shooter = new Shooter(SHOOTER_RIGHT_ID, SHOOTER_LEFT_ID, SHOOTER_PIVOT_RIGHT_ID, SHOOTER_PIVOT_LEFT_ID);
    public final Climber climber = new Climber(CLIMBER_RIGHT_ID, CLIMBER_LEFT_ID);
    public final Candle candleLEDS = new Candle(CANDLE_ID);
    public final AmpDeflector deflector = new AmpDeflector(AMP_DEFLECTOR_ID);
    public final Limelight shooterLimelight = new Limelight("limelight-shooter");
    public final Limelight intakeLimelight = new Limelight("limelight-intake");
    
    private final AutonContainer auton = new AutonContainer(this);
    private final SendableChooser<Command> autonChooser = auton.buildAutonChooser();    

    /** Constructs a RobotContainer */
    public RobotContainer() {
        SmartDashboard.putData("Auton Selector", autonChooser);
        
        setDriverControls();
        setOperatorControls();
        setDefaultCommands();
    }

    /** Use this to pass the autonomous command to the main {@link Robot} class.
     *  @return the command to run in autonomous */
    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    public boolean onRedAlliance() { 
        return DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red);
    }

    /** Configures a set of commands that will run by default without human operation */
    private void setDefaultCommands() {
        // Set the intake to always be intaking by default
        intake.setDefaultCommand(new IntakePiece(intake));
        // Set the LEDs to intdicate the robot's state
        candleLEDS.setDefaultCommand(new LEDControlCommand(candleLEDS, this));

    }

    /** Configures a set of control bindings for the robot's operator */
    private void setDriverControls() {
        // Drives the robot with the joysticks
        drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, 
        () -> driverController.getLeftX(),
        () -> driverController.getLeftY(),
        () -> driverController.getRightX()));

        // PRESS START -> Reset the heading of the robot so the currently faced direction becomes 0
        driverController.leftBumper().onTrue(new InstantCommand(() -> drivetrain.resetHeading()));
        
        // HOLD LT -> Activate the automatic intake
        driverController.leftTrigger().whileTrue(new IntakeAutoPickup(intake)
            .alongWith(new DriveWithLimelightTarget(drivetrain, intakeLimelight,
                () -> driverController.getLeftX(), () -> driverController.getLeftY(), () -> driverController.getRightX()).repeatedly())
                .until(() -> intake.holdingPiece())
            .onlyIf(() -> shooter.getPosition() >= ShooterPosition.POINT_BLANK.getAngle()-1));
        
        // HOLD RT -> Drive in robot centric mode
        driverController.rightTrigger().onTrue(new InstantCommand(() -> drivetrain.setFieldCentric(false)))
            .onFalse(new InstantCommand(() -> drivetrain.setFieldCentric(true)));
        // HOLD X -> Lock the drivetrain for anti-defense
        driverController.x().whileTrue(new LockDrivetrain(drivetrain));
        // HOLD A -> Lock robot heading straight
        driverController.a().whileTrue(new DriveWithHeading(drivetrain,
            () -> driverController.getLeftX(), () -> driverController.getLeftY(), 0));
    }

    /** Configures a set of control bindings for the robot's operator */
    private void setOperatorControls() {
        // Hold LT -> Prep the shooter for a point-blank shot
        operatorController.leftTrigger()
            .onTrue(new SetShooterState(shooter, ShooterPosition.POINT_BLANK, 4000))
            .onFalse(new SetShooterState(shooter, ShooterPosition.POINT_BLANK, 0));

        // Hold RB -> Prep the shooter for a shot using limelight
        operatorController.rightBumper()
            .whileTrue(new AimShooterAtSpeaker(shooterLimelight, shooter).repeatedly()
                .alongWith(new DriveWithLimelightTarget(drivetrain, shooterLimelight,
                    () -> driverController.getLeftX(), () -> driverController.getLeftY(), () -> driverController.getRightX()).repeatedly()))
            .onFalse(new SetShooterState(shooter, ShooterPosition.POINT_BLANK, 0));

        // Hold LB -> Prep shooter for amp shot
        operatorController.leftBumper()
            .onTrue(new SetShooterState(shooter, ShooterPosition.AMP_SHOT, 1350))
            .whileTrue(deflector.deflectorOut())
            .onFalse(new SetShooterState(shooter, ShooterPosition.POINT_BLANK, 0));

        // HOLD RT -> Drive the intake outward for piece ejection
        operatorController.rightTrigger().whileTrue(new Eject(intake).onlyIf(() -> shooter.getRPM() > 500));

        // HOLD Y -> Raise the climber, release to climb
        operatorController.y().onTrue(
            new SequentialCommandGroup(
                new SetIntakePosition(intake, IntakePosition.CLIMB),
                new ParallelCommandGroup(
                    new SetShooterState(shooter, ShooterPosition.MINIMUM, 0),
                    new SetClimberPosition(climber, ClimberPosition.MAXIMUM))))
        .onFalse(new SetClimberPosition(climber, ClimberPosition.MINIMUM));
    }
}