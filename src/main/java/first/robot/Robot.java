package first.robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.RobotModeTriggers;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.epilogue.Epilogue;
import org.wpilib.epilogue.Logged;
import org.wpilib.framework.TimedRobot;
import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.system.DataLogManager;
import org.wpilib.system.Timer;

import first.robot.subsystems.swerve.SwerveSubsystem;

// 2027 example: https://github.com/wpilibsuite/allwpilib/tree/v2027.0.0-alpha-6/wpilibjExamples/src/main/java/org/wpilib/examples/rapidreactcommandbot

// Main changes:
// - Main.java moved up one directory to first package
// - Everything else moved to first.robot package
// - XboxController replaced with the Gamepad class

@Logged(name = "Robot")
public class Robot extends TimedRobot {

    PowerDistribution pdh = new PowerDistribution(0);

    // 2027: XboxController (and all other bespoke controller classes) have been
    // replaced with the Gamepad class
    Gamepad driverController = new Gamepad(0);

    SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private final Timer matchTimer = new Timer();

    private Command selectedAuto;
    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    enum AutoMode {
        MoveAuto, StillAuto
    }

    public Robot() {
        configureBindings();

        // Initialize data logging.
        DataLogManager.start();
        Epilogue.bind(this);

        // Add Autos to chooser
        autoChooser.setDefaultOption("Move Auto", AutoMode.MoveAuto.name());
        autoChooser.addOption("Still Auto", AutoMode.StillAuto.name());

        // Setup Timer
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

    public void configureBindings() {
        // Configure your button bindings here
    }

    @Override
    public void robotPeriodic() {
        // Run the command scheduler.
        CommandScheduler.getInstance().run();

        //// SmartDashboard ////
        SwerveModulePosition[] swerveAngles = swerveSubsystem.getModulePositions();
        SmartDashboard.putString("Front Left Angle", swerveAngles[0].angle.toString());
        SmartDashboard.putString("Front Right Angle", swerveAngles[1].angle.toString());
        SmartDashboard.putString("Back Left Angle", swerveAngles[2].angle.toString());
        SmartDashboard.putString("Back Right Angle", swerveAngles[3].angle.toString());

        //// Always run these

        if (matchTimer.isRunning() && isDisabled()) {
            matchTimer.stop();
        }

        SmartDashboard.putNumber("PDH Total Current", pdh.getTotalCurrent());

        if (isAutonomous()) {
            SmartDashboard.putNumber("Match Time", (20) - matchTimer.get());
        }

        if (isTeleop()) {
            SmartDashboard.putNumber("Match Time", (140 + 20) - matchTimer.get());
        }

        // Swerve driver control
        swerveSubsystem.setDesiredSpeeds(driverController.getLeftY(), driverController.getLeftX(),
                2.0 * driverController.getRightX());
    }

    @Override
    public void autonomousInit() {
        switch (autoChooser.getSelected()) {
            case "Move Auto":
                selectedAuto = Commands.none();
                break;
            case "Still Auto":
                selectedAuto = Commands.none();
                break;
            default:
                selectedAuto = null;
        }

        if (selectedAuto != null) {
            CommandScheduler.getInstance().schedule(selectedAuto);
        }
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (selectedAuto != null) {
            selectedAuto.cancel();
        }
    }

    @Override
    public void utilityInit() {
        // Cancels all running commands at the start of utility mode.
        CommandScheduler.getInstance().cancelAll();
    }
}
