package frc.robot.commands;

import static frc.robot.Constants.SwerveConstants.MAX_TRANSLATION_SPEED;
import static frc.robot.Constants.SwerveConstants.ModuleConstants.WHEEL_DIAMETER;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants.ShooterPosition;
import frc.robot.RobotContainer;
import frc.robot.commands.drive_commands.DriveWithLimelightTarget;
import frc.robot.commands.intake_commands.Eject;
import frc.robot.commands.intake_commands.IntakeAutoPickup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

/** A container that stores various procedures for the autonomous portion of the game */
public class AutonContainer {
    private final SwerveDrivetrain drivetrain;
    private final Shooter shooter;
    private final Intake intake;
    private final Limelight shooterLimelight;

    /** Constructs an AutonContainer object */ 
    public AutonContainer(RobotContainer robot) {
        drivetrain = robot.drivetrain;
        shooter = robot.shooter;
        intake = robot.intake;
        shooterLimelight = robot.shooterLimelight;

        registerNamedCommands();

        AutoBuilder.configureHolonomic(
            drivetrain::getPoseMeters, 
            drivetrain::setOdometry,
            drivetrain::getChassisSpeeds,
            drivetrain::driveRobotRelative,
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    MAX_TRANSLATION_SPEED, // Max module speed, in m/s
                    WHEEL_DIAMETER, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> robot.onRedAlliance(),
            drivetrain);
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("RevShooter", new SetShooterState(shooter, ShooterPosition.POINT_BLANK, 5000));
        NamedCommands.registerCommand("Shoot Preload", shootPreload());
        NamedCommands.registerCommand("Intake", new IntakeAutoPickup(intake).withTimeout(3));
        NamedCommands.registerCommand("Take Longshot", new DriveWithLimelightTarget(drivetrain, shooterLimelight, () -> 0, () -> 0, () -> 0)
            .alongWith(new AimShooterAtSpeaker(shooterLimelight, shooter)).withTimeout(.9)
            .andThen(new Eject(intake).withTimeout(.1)));
        NamedCommands.registerCommand("Shoot", new Eject(intake).withTimeout(.15));
    }

    public SendableChooser<Command> buildAutonChooser() {
        SendableChooser<Command> chooser = new SendableChooser<Command>();
        chooser.setDefaultOption("Do Nothing", doNothing());
        chooser.addOption("Shoot Preload ONLY", shootPreload());
        chooser.addOption("Amp Drive Late and Shoot", getPPAuto("Amp Drive Late and Shoot Preload"));
        chooser.addOption("Amp Four Piece", getPPAuto("Amp Four Piece Long Shot"));
        chooser.addOption("Center Three Piece (Under Stage)", getPPAuto("Center Three Piece (Under Stage)"));
        chooser.addOption("Center Four Piece (Near Pieces Only)", getPPAuto("Center Four Piece (Near Pieces Only)"));
        chooser.addOption("Center Five Piece!!", getPPAuto("Center Five Piece!!"));
        chooser.addOption("Center 5.5 Piece", getPPAuto("Center Five and a Half Piece"));
        chooser.addOption("Stage Drive Late and Shoot", getPPAuto("Stage Drive Late and Shoot Preload"));
        chooser.addOption("Stage 3.5 Piece", getPPAuto("Stage Three Piece and Claim Center"));
        return chooser;
    }

    /** Auton that does nothing */
    public Command doNothing() {
        return new WaitCommand(0);
    }

    /** Shoots the piece that comes preloaded in our robot */
    public Command shootPreload() {
        return new SequentialCommandGroup(
            new SetShooterState(shooter, ShooterPosition.POINT_BLANK, 5000).withTimeout(.5),
            new Eject(intake).withTimeout(.15));
    }

    /** Returns a command that executes a pathplanner auto 
     *  @param AutoName The name of the auto in the GUI */
    public Command getPPAuto(String AutoName) {
        drivetrain.setOdometry(PathPlannerAuto.getStaringPoseFromAutoFile(AutoName));
        return AutoBuilder.buildAuto(AutoName);
    }
}