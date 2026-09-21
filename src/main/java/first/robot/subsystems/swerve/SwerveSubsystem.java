package first.robot.subsystems.swerve;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.hardware.imu.OnboardIMU.MountOrientation;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.smartdashboard.Field2d;

/**
 * The SwerveSubsystem class represents a swerve drive system for a robot.
 * It manages four swerve modules (front-left, front-right, back-left,
 * back-right)
 * and uses kinematics to calculate the desired states for each module based on
 * the desired chassis speeds.
 * 
 * <p>
 * Features:
 * <ul>
 * <li>Initialization of swerve modules with specific configurations.</li>
 * <li>Calculation of swerve module states using kinematics.</li>
 * <li>Periodic updates to set the desired state for each module.</li>
 * </ul>
 * 
 * <p>
 * Usage:
 * <ul>
 * <li>Instantiate the subsystem to initialize the swerve modules and
 * kinematics.</li>
 * <li>Set the desired chassis speeds to control the robot's movement.</li>
 * <li>The periodic method automatically updates the module states.</li>
 * </ul>
 * 
 * <p>
 * Dependencies:
 * <ul>
 * <li>SwerveModule: Represents an individual swerve module.</li>
 * <li>SwerveModuleConfig: Configuration for each swerve module.</li>
 * <li>SwerveDriveKinematics: Handles kinematics calculations for swerve
 * drive.</li>
 * <li>ChassisVelocities: Represents the desired speeds for the robot chassis.</li>
 * </ul>
 */
public class SwerveSubsystem extends SubsystemBase {

    SwerveModule frontLeft;
    SwerveModule frontRight;
    SwerveModule backLeft;
    SwerveModule backRight;

    ChassisVelocities desiredSpeeds = new ChassisVelocities();

    SwerveDriveKinematics kinematics;
    SwerveDrivePoseEstimator poseEstimator;

    OnboardIMU imu = new OnboardIMU(MountOrientation.FLAT);

    public Field2d field = new Field2d();

    /**
     * The SwerveSubsystem class is responsible for configuring and managing the
     * swerve drive modules
     * and kinematics for a robot. It initializes four swerve modules (front-left,
     * front-right, back-left,
     * and back-right) with their respective configurations, including motor IDs,
     * encoder ports, angle offsets,
     * and positions on the robot. The subsystem also sets up the swerve drive
     * kinematics using the locations
     * of the modules.
     * 
     * <p>
     * Key functionalities:
     * <ul>
     * <li>Defines configurations for each swerve module, including drive motor ID,
     * angle motor ID,
     * absolute encoder port, angle offset, and position (x, y).</li>
     * <li>Creates instances of SwerveModule for each module using the defined
     * configurations.</li>
     * <li>Initializes the SwerveDriveKinematics object to handle the kinematics of
     * the swerve drive system.</li>
     * </ul>
     * 
     * <p>
     * Usage:
     * This subsystem is typically used in conjunction with a higher-level control
     * system to manage
     * robot movement and orientation using swerve drive principles.
     */
    public SwerveSubsystem() {
        SwerveModuleConfig frontLeftConfig = new SwerveModuleConfig()
                .driveMotorID(2)
                .angleMotorID(1)
                .absoluteEncoderPort(3)
                .angleOffset(253)
                .invertAngleMotor(true)
                .x(0.5)
                .y(0.5);

        SwerveModuleConfig backLeftConfig = new SwerveModuleConfig()
                .driveMotorID(4)
                .angleMotorID(3)
                .absoluteEncoderPort(2)
                .angleOffset(338)
                .invertAngleMotor(true)
                .x(-0.5)
                .y(0.5);

        SwerveModuleConfig backRightConfig = new SwerveModuleConfig()
                .driveMotorID(6)
                .angleMotorID(5)
                .absoluteEncoderPort(1)
                .angleOffset(123)
                .invertAngleMotor(true)
                .x(-0.5)
                .y(-0.5);

        SwerveModuleConfig frontRightConfig = new SwerveModuleConfig()
                .driveMotorID(8)
                .angleMotorID(7)
                .absoluteEncoderPort(0)
                .angleOffset(242)
                .invertAngleMotor(true)
                .x(0.5)
                .y(-0.5);

        frontLeft = new SwerveModule("Front Left", frontLeftConfig);
        frontRight = new SwerveModule("Front Right", frontRightConfig);
        backLeft = new SwerveModule("Back Left", backLeftConfig);
        backRight = new SwerveModule("Back Right", backRightConfig);

        kinematics = new SwerveDriveKinematics(frontLeft.location, frontRight.location, backLeft.location,
                backRight.location);

        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                getYaw(),
                getModulePositions(),
                new Pose2d()); // x,y,heading in radians; Vision measurement std dev, higher=less weight
    }

    public void setDesiredSpeeds(ChassisVelocities speeds) {
        this.desiredSpeeds = speeds;
    }

    public void setDesiredSpeeds(double vx, double vy, double omega) {
        this.desiredSpeeds = new ChassisVelocities(vx, vy, omega);
    }

    public void setDesiredSpeedsFieldOriented(double vx, double vy, double omega, double robotAngle) {
        this.desiredSpeeds = new ChassisVelocities(vx, vy, omega).toRobotRelative(Rotation2d.fromDegrees(robotAngle));
    }

    public Rotation2d getYaw() {
        return imu.getRotation2d();
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    @Override
    public void periodic() {
        SwerveModuleVelocity[] moduleStates = kinematics.toSwerveModuleVelocities(this.desiredSpeeds);

        frontLeft.setDesiredVelocity(moduleStates[0]);
        frontRight.setDesiredVelocity(moduleStates[1]);
        backLeft.setDesiredVelocity(moduleStates[2]);
        backRight.setDesiredVelocity(moduleStates[3]);
    }

    public Pose2d getPose() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPose'");
    }

    public void setEnabled(boolean driveEnabled) {
        // TODO: implement
        return;
    }

    public Command getAutonomousCommand(String selected) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAutonomousCommand'");
    }
}
