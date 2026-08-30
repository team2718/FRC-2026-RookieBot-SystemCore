package frc.robot.subsystems.swerve;

public class SwerveModuleConfig {
    public int driveMotorID;
    public int angleMotorID;
    public int absoluteEncoderPort;
    public double angleOffset;
    public double x;
    public double y;
    public boolean invertAbsoluteEncoder = false;
    public boolean invertAngleMotor = false;

    /**
     * Represents the configuration for a swerve module in the robot's drivetrain.
     * This class is intended to encapsulate the settings and parameters required
     * to initialize and operate a swerve module.
     */
    public SwerveModuleConfig() {}

    /**
     * Sets the ID of the drive motor for this swerve module configuration.
     *
     * @param driveMotorID The ID of the drive motor.
     * @return The current instance of {@code SwerveModuleConfig} for method chaining.
     */
    public SwerveModuleConfig driveMotorID(int driveMotorID) {
        this.driveMotorID = driveMotorID;
        return this;
    }

    /**
     * Sets the ID of the angle motor for the swerve module.
     *
     * @param angleMotorID The ID of the angle motor.
     * @return The current instance of {@code SwerveModuleConfig} for method chaining.
     */
    public SwerveModuleConfig angleMotorID(int angleMotorID) {
        this.angleMotorID = angleMotorID;
        return this;
    }

    /**
     * Sets the port number for the absolute encoder.
     * 
     * @param absoluteEncoderPort The port number of the absolute encoder.
     * @return The current instance of {@code SwerveModuleConfig} for method chaining.
     */
    public SwerveModuleConfig absoluteEncoderPort(int absoluteEncoderPort) {
        this.absoluteEncoderPort = absoluteEncoderPort;
        return this;
    }

    /**
     * Sets the angle offset for the swerve module configuration.
     * The angle offset is used to adjust the initial position of the module's angle.
     *
     * @param angleOffset The angle offset in degrees.
     * @return The updated SwerveModuleConfig instance.
     */
    public SwerveModuleConfig angleOffset(double angleOffset) {
        this.angleOffset = angleOffset;
        return this;
    }

    /**
     * Sets the x-coordinate for the swerve module configuration.
     *
     * @param x The x-coordinate value to set.
     * @return The current instance of {@code SwerveModuleConfig} for method chaining.
     */
    public SwerveModuleConfig x(double x) {
        this.x = x;
        return this;
    }

    /**
     * Sets the y-coordinate for the swerve module configuration.
     *
     * @param y The y-coordinate value to set.
     * @return The current instance of {@code SwerveModuleConfig} for method chaining.
     */
    public SwerveModuleConfig y(double y) {
        this.y = y;
        return this;
    }

    /**
     * Sets whether to invert the absolute encoder reading.
     *
     * @param invertAbsoluteEncoder True to invert the absolute encoder, false otherwise.
     * @return The current instance of {@code SwerveModuleConfig} for method chaining.
     */
    public SwerveModuleConfig invertAbsoluteEncoder(boolean invertAbsoluteEncoder) {
        this.invertAbsoluteEncoder = invertAbsoluteEncoder;
        return this;
    }

    public SwerveModuleConfig invertAngleMotor(boolean invertAngleMotor) {
        this.invertAngleMotor = invertAngleMotor;
        return this;
    }
}