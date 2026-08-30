package frc.robot;

import org.wpilib.command2.Commands;
import org.wpilib.command2.button.RobotModeTriggers;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.system.Timer;

import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RobotCode {

    PowerDistribution pdh = new PowerDistribution(0);

    XboxController driverXboxController = new XboxController(0);

    SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    String selectedAuto = Robot.kMoveAuto;

    private final Timer matchTimer = new Timer();

    public void init() {


        matchTimer.reset();

        // Start match time on autonomous start
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> {
            matchTimer.reset();
            matchTimer.start();
        }));

        // Start match time on teleop start
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {
            matchTimer.reset();
            matchTimer.start();
        }));

        // Stop match time on end of match
        RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> {
            matchTimer.stop();
            matchTimer.reset();
        }));
    }

    public void periodic() {

        //// SmartDashboard ////
        SwerveModulePosition[] swerveAngles = swerveSubsystem.getModulePositions();
        SmartDashboard.putString("Front Left Angle", swerveAngles[0].angle.toString());
        SmartDashboard.putString("Front Right Angle", swerveAngles[1].angle.toString());
        SmartDashboard.putString("Back Left Angle", swerveAngles[2].angle.toString());
        SmartDashboard.putString("Back Right Angle", swerveAngles[3].angle.toString());

        //// Always run these ////

        if (matchTimer.isRunning() && DriverStation.isDisabled()) {
            matchTimer.stop();
        }

        SmartDashboard.putNumber("PDH Total Current", pdh.getTotalCurrent());

        if (DriverStation.isAutonomous()) {
            SmartDashboard.putNumber("Match Time", (20) - matchTimer.get());
        }

        if (DriverStation.isTeleop()) {
            SmartDashboard.putNumber("Match Time", (140 + 20) - matchTimer.get());
        }

        //// Autonomous ////

        if (DriverStation.isAutonomous()) {
            if (selectedAuto.equals(Robot.kMoveAuto)) {
                if (matchTimer.get() < 3) {
                    swerveSubsystem.setDesiredSpeeds(0.3, 0, 0);
                } else {
                    swerveSubsystem.setDesiredSpeeds(0.0, 0, 0);
                }
            }

            if (selectedAuto.equals(Robot.kStillAuto)) {
                swerveSubsystem.setDesiredSpeeds(0.0, 0, 0);
            }

            // Always keep arm up and out of the way in auto
            return;
        }

        //// Teleop ////

        // Swerve driver control
        swerveSubsystem.setDesiredSpeeds(driverXboxController.getLeftY(), driverXboxController.getLeftX(),
                2.0 * driverXboxController.getRightX());
    }

   

    public void setAuto(String m_autoSelected) {
        this.selectedAuto = m_autoSelected;
    }
}
