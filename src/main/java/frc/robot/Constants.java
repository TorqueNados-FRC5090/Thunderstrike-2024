package frc.robot;
// Imports
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/* 
 * To access numbers in this file, import or statically import one of its subclasses:
 * example:
 * import static frc.robot.Constants.ControllerPorts.*;
 * import frc.robot.Constants.DriveConstants;
 */
public final class Constants {

    /* -------------- IDs -------------- */

    /** USB ports used by controllers. */
    public static final class ControllerPorts {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    public static final class LEDPorts {
        public static final int CANDLE_ID = 2;
    }

    /** IDs used by the swerve drivetrain.
        3X for turning, 4X for driving, 5X for abs encoders. */
    public static final class SwerveIDs {
        // Front left module
        public static final int FL_TURN_ID = 30;
        public static final int FL_DRIVE_ID = 40;
        public static final int FL_ENCODER_ID = 50;
        // Front right module
        public static final int FR_TURN_ID = 31;
        public static final int FR_DRIVE_ID = 41;
        public static final int FR_ENCODER_ID = 51;
        // Rear left module
        public static final int RL_TURN_ID = 32;
        public static final int RL_DRIVE_ID = 42;
        public static final int RL_ENCODER_ID = 52;
        // Rear right module
        public static final int RR_TURN_ID = 33;
        public static final int RR_DRIVE_ID = 43;
        public static final int RR_ENCODER_ID = 53;
    }

    /** IDs used by the intake */
    public static final class IntakeIDs {
        public static final int INTAKE_DRIVER_ID = 10;
        public static final int INTAKE_ROTATOR_ID = 11;
        public static final int INTAKE_LIMIT_ID = 0;
    }

    /** IDs used by the shooter */
    public static final class ShooterIDs {
        public static final int SHOOTER_LEFT_ID = 12;
        public static final int SHOOTER_RIGHT_ID = 13;
        public static final int SHOOTER_PIVOT_LEFT_ID = 14;
        public static final int SHOOTER_PIVOT_RIGHT_ID = 15;   
    }

    public static final class ClimberIDs {
        public static final int CLIMBER_LEFT_ID = 16;
        public static final int CLIMBER_RIGHT_ID = 17;
    }

    public static final class AmpDeflectorIDs {
        public static final int AMP_DEFLECTOR_ID = 18;
    }

    /* -------------- SUBSYTEM CONSTANTS -------------- */

    public static final class IntakeConstants {
        /** Converts intake pivot motor revolutions to degrees */
        public static final double INTAKE_PIVOT_RATIO = .1125;
        public static enum IntakePosition {
            PICKUP(-214), 
            CLIMB(-90),
            SHOOT(0);


            private double angle;
            IntakePosition(double angle) { this.angle = angle; }
            /** @return The angle in degrees associated with this position */
            public double getAngle() { return angle; }
        }
    }

    /** Any constants needed by the shooter */
    public static final class ShooterConstants {
        /** Converts shooter pivot motor revolutions to degrees */
        public static final double SHOOTER_PIVOT_RATIO = 1.3889;
        public static enum ShooterPosition {
            MINIMUM(-24), 
            POINT_BLANK(6),
            AMP_SHOT(11),
            LONG_SHOT(22),
            MAXIMUM(26);

            private double angle;
            ShooterPosition(double angle) { this.angle = angle; } 
            /** @return The angle in degrees associated with this position */
            public double getAngle() { return angle; }
        }
    }

    public static final class ClimberConstants {
        /** Converts climber motor revolutions to inches */
        public static final double CLIMBER_RATIO = 6.498;
        public static enum ClimberPosition {
            MINIMUM(0), 
            MAXIMUM(14.4);

            private double height;
            ClimberPosition(double angle) { this.height = angle; } 
            /** The height in inches associated with this position */
            public double getHeight() { return height; }
        }
    }

    public static final class LEDConstants {
        public static enum LEDColor {
            RED(255, 0, 0),
            GREEN(25, 255, 0),
            BLUE(0, 10, 181),
            YELLOW(255, 100, 0),
            PURPLE(162, 18, 184),
            PINK(255, 166, 238),
            LIGHT_BLUE(125, 212, 255),
            ORANGE(180, 20, 0),
            WHITE(255, 255, 255);

            private int red;
            private int green;
            private int blue;
             
            LEDColor(int red, int green, int blue) {
                this.red = red;
                this.green = green;
                this.blue = blue;
            }

            public int getRed() { return red; }
            public int getGreen() { return green; }
            public int getBlue() { return blue; }
        }

        public static enum LEDStrip {
            CANDLE(0, 8),
            SHOOTER(8, 60),
            INTAKE(68, 31);

            private int startingIndex;
            private int stripLength;

            LEDStrip(int startingIndex, int stripLength){
                this.startingIndex = startingIndex;
                this.stripLength = stripLength;
            }

            public int getStartingIndex() { return startingIndex; }
            public int getStripLength() { return stripLength; }
        }
    }

    public static final class AmpDeflectorConstants {
        /** Converts deflector motor revolutions to degrees */
        public static final double DEFLECTOR_RATIO = 25.0 / 360.0;
        public static enum DeflectorPosition {
            MINIMUM(0), 
            AMP_SHOT(165),
            MAXIMUM(180);

            private double degrees;
            DeflectorPosition(double angle) { this.degrees = angle; } 
            /** The angle in degrees associated with this position */
            public double getAngle() { return degrees; }
        }
    }

    /** Turning a module to absolute 0 minus its offset will point it forward */
    public static final class SwerveModuleOffsets {
        public static final double FL_OFFSET = 112;
        public static final double FR_OFFSET = 76.9;
        public static final double RL_OFFSET = 46.8;
        public static final double RR_OFFSET = 95.5;
    }

    /** Whether or not each swerve component should be inverted/reversed */
    public static final class SwerveInversions {
        // Whether each driving motor should be inverted
        public static final boolean INVERT_FL_DRIVE = false;
        public static final boolean INVERT_RL_DRIVE = false;
        public static final boolean INVERT_FR_DRIVE = false;
        public static final boolean INVERT_RR_DRIVE = false;

        // Whether each turning motor should be inverted
        public static final boolean INVERT_FL_TURN = true;
        public static final boolean INVERT_FR_TURN = true;
        public static final boolean INVERT_RL_TURN = true;
        public static final boolean INVERT_RR_TURN = true;
    }

    /** Constants related to swerve calculations */
    public static final class SwerveConstants {
        /** The distance between the left and right wheels */
        public static final double TRACK_WIDTH = Units.inchesToMeters(22.75);
        /** The distance between the front and rear wheels */
        public static final double WHEEL_BASE = Units.inchesToMeters(22.75);

        /** An array containing the position of each module as a {@link Translation2d} object */
        public static final Translation2d[] MODULE_TRANSLATIONS = {
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
        };

        // Kinematics are used to calculate how each module needs to move
        // in order to move the robot as a whole in a certain way

        /** Standard kinematics with center of rotation located at the center of the robot */
        public static final SwerveDriveKinematics SWERVE_KINEMATICS =
            new SwerveDriveKinematics(MODULE_TRANSLATIONS);

        /** The max speed the robot is able to drive in m/sec */
        public static final double MAX_TRANSLATION_SPEED = 5;
        /** The max speed the robot is allowed to spin in rads/sec */
        public static final double MAX_ROTATION_SPEED = 2.5 * Math.PI; // 1.25 rotations/sec
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
            new TrapezoidProfile.Constraints(
                MAX_ROTATION_SPEED, Math.PI * 2);

        public static final class ModuleConstants {
            /** The ratio of the drive motors on the workhorse chassis */
            public static final double DRIVE_RATIO_SLOW = 1 / ((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0));
            /** The ratio of the drive motors on the robot */
            public static final double DRIVE_RATIO_FAST = 1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
            /** The ratio of the turning motors */
            public static final double TURN_RATIO = 1 / ((14.0 / 50.0) * (10.0 / 60.0));
            
            /** The diameter of the wheels measured in meters */
            public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.94);

            /** Drive motor revolutions * DRIVE_REVS_TO_M = distance in meters */
            public static final double DRIVE_REVS_TO_M = ((WHEEL_DIAMETER * Math.PI) / DRIVE_RATIO_FAST);

            // 1 RPM * DRIVE_REVS_TO_M = speed in m/min. Divide by 60 to find m/sec
            /** Drive motor RPM * DRIVE_RPM_TO_MPS = speed in m/sec */
            public static final double DRIVE_RPM_TO_MPS = DRIVE_REVS_TO_M / 60.0;

            /** Turning motor revolutions * TURNING_REVS_TO_DEG = Turning motor total degrees turned */
            public static final double TURNING_REVS_TO_DEG =  360.0 / TURN_RATIO;
        }
    
        /** Enum representing the four possible positions a module can occupy */
        public enum ModulePosition {
            FRONT_LEFT,
            FRONT_RIGHT,
            REAR_LEFT,
            REAR_RIGHT
        }
    }

    public static final class DriveConstants {
            /** Higher values make the robot drive more aggressively */
            public static final double TRANSLATION_SLEW = 4;
            /** Higher values make the robot spin more aggressively */
            public static final double ROTATION_SLEW = 6;
    
            /** Translation instructions closer to 0 than the deadband will be set to 0 */
            public static final double TRANSLATION_DEADBAND = .05;
            /** Rotation instructions closer to 0 than the deadband will be set to 0 */
            public static final double ROTATION_DEADBAND = .1;
    }
}
