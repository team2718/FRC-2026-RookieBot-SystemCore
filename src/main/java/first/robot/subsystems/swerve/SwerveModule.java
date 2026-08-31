package first.robot.subsystems.swerve;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RotationsPerSecond;

import org.wpilib.hardware.discrete.AnalogInput;
import org.wpilib.hardware.rotation.DutyCycleEncoder;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.system.RobotController;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * Represents a single module in a swerve drive system, responsible for
 * controlling
 * both the drive and angle motors of the module. This class provides
 * functionality
 * to set the desired state of the module and retrieve its current angle.
 * 
 * <p>
 * Key Features:
 * <ul>
 * <li>Uses cosine compensation to adjust speed based on angular
 * difference.</li>
 * <li>Supports continuous input for angle PID control.</li>
 * <li>Handles conversion factors for drive and angle motors.</li>
 * </ul>
 * 
 * <p>
 * Dependencies:
 * <ul>
 * <li>{@link TalonFX} for drive motor control.</li>
 * <li>{@link SparkMax} for angle motor control.</li>
 * <li>{@link DutyCycleEncoder} for absolute encoder readings.</li>
 * <li>{@link PIDController} for angle control.</li>
 * </ul>
 * 
 * <p>
 * Usage:
 * 
 * <pre>
 * SwerveModuleConfig config = new SwerveModuleConfig(...);
 * SwerveModule module = new SwerveModule(config);
 * SwerveModuleState desiredState = new SwerveModuleState(...);
 * module.setDesiredState(desiredState);
 * </pre>
 * 
 * <p>
 * Configuration:
 * <ul>
 * <li>Adjust {@code DRIVE_CONVERSION_FACTOR} and
 * {@code ANGLE_CONVERSION_FACTOR} based on motor setup.</li>
 * <li>Set {@code USE_COSINE_COMPENSATION} to enable or disable speed
 * adjustment.</li>
 * </ul>
 * 
 * @author Your Name
 * @version 1.0
 */
public class SwerveModule {
    private static final boolean USE_COSINE_COMPENSATION = true;

    private static final double WHEEL_RADIUS_METERS = Meters.convertFrom(2, Inches);
    private static final double DRIVE_CONVERSION_FACTOR = 8.0;
    private final SwerveModuleConfig config;
    private final TalonFX driveMotor;
    private final SparkMax angleMotor;
    private final AnalogInput absoluteEncoder;
    final Translation2d location;
    private final String localName;

    final PIDController angleController = new PIDController(0.15, 0.0, 0.0);

    /**
     * Represents a single swerve module in a swerve drive system.
     * This class encapsulates the functionality for controlling the drive motor,
     * angle motor, and absolute encoder of the module, as well as its physical
     * location
     * and angle offset.
     *
     * @param config The configuration object containing parameters for the swerve
     *               module,
     *               including motor IDs, encoder port, angle offset, and physical
     *               location.
     *               - {@code driveMotorID}: The ID of the drive motor.
     *               - {@code angleMotorID}: The ID of the angle motor.
     *               - {@code absoluteEncoderPort}: The port for the absolute
     *               encoder.
     *               - {@code angleOffset}: The offset for the angle of the module.
     *               - {@code x}: The x-coordinate of the module's location.
     *               - {@code y}: The y-coordinate of the module's location.
     */
    public SwerveModule(String localName, SwerveModuleConfig config) {
        // Drive Motor
        this.driveMotor = new TalonFX(config.driveMotorID, CANBus.systemcore(0));
        TalonFXConfiguration driveMotorConfiguration = new TalonFXConfiguration();
        driveMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        driveMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotorConfiguration.Slot0.kV = 0.12;
        driveMotorConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.6;
        this.driveMotor.getConfigurator().apply(driveMotorConfiguration);

        // Angle Motor
        this.angleMotor = new SparkMax(0, config.angleMotorID, MotorType.kBrushless);
        SparkMaxConfig angleMotorConfig = new SparkMaxConfig();
        angleMotorConfig.limitSwitch.forwardLimitSwitchTriggerBehavior(LimitSwitchConfig.Behavior.kKeepMovingMotor);
        angleMotorConfig.limitSwitch.reverseLimitSwitchTriggerBehavior(LimitSwitchConfig.Behavior.kKeepMovingMotor);
        angleMotorConfig.smartCurrentLimit(20);
        angleMotorConfig.inverted(config.invertAngleMotor);
        angleMotorConfig.idleMode(IdleMode.kBrake);
        angleMotorConfig.closedLoopRampRate(0.3);
        this.angleMotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.absoluteEncoder = new AnalogInput(config.absoluteEncoderPort);
        this.config = config;
        this.location = new Translation2d(config.x, config.y);

        this.localName = localName;

        this.angleController.enableContinuousInput(0, 360);
    }

    /**
     * Sets the desired state for the swerve module, including speed and angle.
     * The method optimizes the desired state to minimize unnecessary rotation and
     * adjusts
     * the speed using cosine compensation if enabled.
     *
     * @param swerveModuleState The desired state of the swerve module, including
     *                          speed and angle.
     *                          The state is optimized based on the current angle of
     *                          the module.
     */
    public void setDesiredVelocity(SwerveModuleVelocity desiredModuleState) {
        desiredModuleState = desiredModuleState.optimize(getModuleRotation());

        if (USE_COSINE_COMPENSATION) {
            desiredModuleState.velocity *= desiredModuleState.angle.minus(getModuleRotation()).getCos();
        }

        SmartDashboard.putNumber(localName + "/speed_output",
                desiredModuleState.velocity / (2 * Math.PI * WHEEL_RADIUS_METERS) * DRIVE_CONVERSION_FACTOR);

        driveMotor.setControl(
                new VelocityVoltage(
                        desiredModuleState.velocity / (2 * Math.PI * WHEEL_RADIUS_METERS) * DRIVE_CONVERSION_FACTOR));

        angleMotor.setVoltage(
                angleController.calculate(getAbsolutePositionDegrees(),
                        (desiredModuleState.angle.getDegrees() + 360) % 360.0));

        SmartDashboard.putNumber(localName + "/angle_raw", getRawAbsolutePositionsDegrees());
        SmartDashboard.putNumber(localName + "/angle", getAbsolutePositionDegrees());
        SmartDashboard.putNumber(localName + "/angle_setpoint", (desiredModuleState.angle.getDegrees() + 360) % 360.0);

        SmartDashboard.putNumber(localName + "/speed", driveMotor.getVelocity().getValue().in(RotationsPerSecond));
        SmartDashboard.putNumber(localName + "/speed_setpoint", desiredModuleState.velocity);
    }

    /**
     * Retrieves the raw absolute position of the encoder in degrees.
     *
     * @return The raw absolute position of the encoder in degrees (0 to 360).
     */
    private double getRawAbsolutePositionsDegrees() {
        return (config.invertAbsoluteEncoder ? -1.0 : 1.0)
                * (absoluteEncoder.getVoltage() / RobotController.getVoltage3V3())
                * 360.0;

        // absoluteEncoder.getAverageVoltage() is broken! HAL error
    }

    /**
     * Calculates the absolute position of the swerve module in degrees.
     *
     * @return The absolute position of the swerve module in degrees (0 to 360).
     */
    private double getAbsolutePositionDegrees() {
        return (getRawAbsolutePositionsDegrees() - config.angleOffset + 360) % 360;
    }

    private Rotation2d getModuleRotation() {
        return Rotation2d.fromDegrees(getAbsolutePositionDegrees());
    }

    public SwerveModuleVelocity getVelocity() {
        return new SwerveModuleVelocity(
                driveMotor.getVelocity().getValueAsDouble() / DRIVE_CONVERSION_FACTOR,
                getModuleRotation());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveMotor.getPosition().getValueAsDouble() / DRIVE_CONVERSION_FACTOR,
                getModuleRotation());
    }
}
