package first.robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.RobotModeTriggers;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.epilogue.Logged;
import org.wpilib.framework.TimedRobot;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.AngularVelocity;
import static org.wpilib.units.Units.*;
import org.wpilib.system.DataLogManager;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.Selectable;
import org.wpilib.units.measure.AngularVelocity;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import first.robot.subsystems.swerve.SwerveSubsystem;
import first.robot.utils.ShooterTree;

// 2027 example: https://github.com/wpilibsuite/allwpilib/tree/v2027.0.0-alpha-6/wpilibjExamples/src/main/java/org/wpilib/examples/rapidreactcommandbot

// Main changes:
// - Main.java moved up one directory to first package
// - Everything else moved to first.robot package
// - XboxController replaced with the Gamepad class

public class Robot extends TimedRobot {

    // PowerDistribution pdh = new PowerDistribution(CANPort.CAN_S0);

    // 2027: XboxController (and all other bespoke controller classes) have been
    // replaced with the Gamepad class
    Gamepad driverController = new Gamepad(0);

    SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    TalonFX shooterMotor, intakeMotor, portalMotor;

    private final Timer matchTimer = new Timer();

    private Command selectedAuto;
    private final Selectable<String> autoChooser = new Selectable<>();

    private final double TRIGGER_EPSILON = 0.2;

    // Motor spinny speed requests
    private final NeutralOut stopRequest = new NeutralOut();
    // Idk if it should be 8.5V, that's what Google suggested
    private final VoltageOut runIntake = new VoltageOut(8.5);
    private final VoltageOut runOuttake = new VoltageOut(-8.5);
    // Aaand some weird inversion stuff for the intake and portal
    private final VoltageOut portalIntake = runOuttake;
    private final VoltageOut portalOuttake = runIntake;
    private final VoltageOut runPortal = new VoltageOut(5);


    enum AutoMode {
        MoveAuto, StillAuto
    }

    public Robot() {
        configureBindings();

        // Initialize data logging.
        DataLogManager.start();
        // Epilogue.bind(this);

        // Add Autos to chooser
        autoChooser.addDefault("Move Auto", AutoMode.MoveAuto.name());
        autoChooser.add("Still Auto", AutoMode.StillAuto.name());

        // Setup mah motors
        configureMotors();

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
        // Haha who needs commands and bindings when you can just
        // use if-elses? T-T
    }

    private void configureMotors() {
        // TODO: Actually configure motors and IDs!!!
        // I don't know what ANY of these values should really be
        //  I just copied them from SwerveModule.java lol

        // Setup configs for shooterMotor, intakeMotor, and portalMotor
        shooterMotor = new TalonFX(9,  new CANBus(CANPort.CAN_S0));
        TalonFXConfiguration shooterMotorConfiguration = new TalonFXConfiguration();
        shooterMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        shooterMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterMotorConfiguration.Slot0.kV = 0.12;
        shooterMotorConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.6;
        shooterMotor.getConfigurator().apply(shooterMotorConfiguration);

        intakeMotor = new TalonFX(10,  new CANBus(CANPort.CAN_S0));
        TalonFXConfiguration intakeMotorConfiguration = new TalonFXConfiguration();
        intakeMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        intakeMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // I think the below line is necessary cause of the intake motor positioning
        intakeMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeMotorConfiguration.Slot0.kV = 0.12;
        intakeMotorConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.6;
        intakeMotor.getConfigurator().apply(intakeMotorConfiguration);

        portalMotor = new TalonFX(11,  new CANBus(CANPort.CAN_S0));
        TalonFXConfiguration portalMotorConfiguration = new TalonFXConfiguration();
        portalMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        portalMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        portalMotorConfiguration.Slot0.kV = 0.12;
        portalMotorConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.6;
        portalMotor.getConfigurator().apply(portalMotorConfiguration);
    }

    @Override
    public void robotPeriodic() {
        // Run the command scheduler.
        CommandScheduler.getInstance().run();

        //// Telemetry ////
        SwerveModulePosition[] swerveAngles = swerveSubsystem.getModulePositions();
        Telemetry.log("Front Left Angle", swerveAngles[0].angle.toString());
        Telemetry.log("Front Right Angle", swerveAngles[1].angle.toString());
        Telemetry.log("Back Left Angle", swerveAngles[2].angle.toString());
        Telemetry.log("Back Right Angle", swerveAngles[3].angle.toString());

        //// Always run these

        if (matchTimer.isRunning() && isDisabled()) {
            matchTimer.stop();
        }

        // Telemetry.log("PDH Total Current", pdh.getTotalCurrent());

        if (isAutonomous()) {
            Telemetry.log("Match Time", (20) - matchTimer.get());
        }

        if (isTeleop()) {
            Telemetry.log("Match Time", (140 + 20) - matchTimer.get());
        }

        // Swerve driver control
        swerveSubsystem.setDesiredSpeeds(-driverController.getLeftY(), -driverController.getLeftX(),
                -2.0 * driverController.getRightX());
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

    // TELEOP
    // This seemed to be better than using an if statement in robotPeriodic()?
    @Override
    public void teleopPeriodic() {
        // Shoot
        if (driverController.getRightTrigger() > TRIGGER_EPSILON) {
            // Find the optimal speed and run the shooter motor
            double hubDistMeters = Units.feetToMeters(4.0); // Arbitrary value assuming we're right against the hub;
            // Maybe one day we'll get vision :')
            // (note; measurements yielded 48.765; rounding will hopefully be fine?)
            double shooterRPM = ShooterTree.getShooterRPM(hubDistMeters);
            AngularVelocity shooterVel = RPM.of(shooterRPM);
            shooterMotor.setControl(new VelocityVoltage(shooterVel));

            // Run the portal and intake motors to feed balls
            // (do we need to have a delay before this?)
            portalMotor.setControl(runPortal);
            intakeMotor.setControl(runPortal);

        } else {
            shooterMotor.setControl(stopRequest);
            
            // Intake / Outtake
            if (driverController.getLeftTrigger() > TRIGGER_EPSILON) {
                // Needs to run intakeMotor and (reversed) portalMotor
                intakeMotor.setControl(runIntake);
                portalMotor.setControl(portalIntake);
            } else if (driverController.getLeftBumperButtonPressed()) {
                // Run both motors the opposite way to unjam
                intakeMotor.setControl(runOuttake);
                portalMotor.setControl(portalOuttake);
            } else {
                intakeMotor.setControl(stopRequest);
                portalMotor.setControl(stopRequest);
            }
        } 
    }

    @Override
    public void utilityInit() {
        // Cancels all running commands at the start of utility mode.
        CommandScheduler.getInstance().cancelAll();
    }
}
