package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.generated.TunerConstants;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * Test suite for CommandSwerveDrivetrain.
 *
 * These tests use WPILib's HAL simulation to test the drivetrain
 * without requiring actual hardware.
 */
class CommandSwerveDrivetrainTest {

    private CommandSwerveDrivetrain drivetrain;

    @BeforeEach
    void setUp() {
        // Initialize HAL for simulation
        assert HAL.initialize(500, 0);

        // Create the drivetrain using the factory method
        drivetrain = TunerConstants.createDrivetrain();

        // Reset the command scheduler
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().enable();
    }

    @AfterEach
    void tearDown() {
        // Clean up command scheduler
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();

        // Close the drivetrain to release resources
        if (drivetrain != null) {
            drivetrain.close();
        }

        // Shutdown HAL
        HAL.shutdown();
    }

    @Test
    void testDrivetrainCreation() {
        assertNotNull(drivetrain, "Drivetrain should be created successfully");
    }

    @Test
    void testApplyRequestReturnsCommand() {
        // Create a simple idle request
        SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

        // Get a command from applyRequest
        Command command = drivetrain.applyRequest(() -> idleRequest);

        assertNotNull(command, "applyRequest should return a non-null Command");
        assertTrue(command.getRequirements().contains(drivetrain),
            "Command should require the drivetrain subsystem");
    }

    @Test
    void testSysIdQuasistaticForward() {
        Command command = drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward);

        assertNotNull(command, "sysIdQuasistatic forward should return a non-null Command");
    }

    @Test
    void testSysIdQuasistaticReverse() {
        Command command = drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse);

        assertNotNull(command, "sysIdQuasistatic reverse should return a non-null Command");
    }

    @Test
    void testSysIdDynamicForward() {
        Command command = drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward);

        assertNotNull(command, "sysIdDynamic forward should return a non-null Command");
    }

    @Test
    void testSysIdDynamicReverse() {
        Command command = drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse);

        assertNotNull(command, "sysIdDynamic reverse should return a non-null Command");
    }

    @Test
    void testPeriodicRunsWithoutError() {
        // Simulate disabled state
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();

        // periodic() should run without throwing
        assertDoesNotThrow(() -> drivetrain.periodic(),
            "periodic() should run without error when disabled");
    }

    @Test
    void testPeriodicWithBlueAlliance() {
        // Set up blue alliance
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();

        // periodic() should handle blue alliance perspective
        assertDoesNotThrow(() -> drivetrain.periodic(),
            "periodic() should handle blue alliance without error");
    }

    @Test
    void testPeriodicWithRedAlliance() {
        // Set up red alliance
        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();

        // periodic() should handle red alliance perspective
        assertDoesNotThrow(() -> drivetrain.periodic(),
            "periodic() should handle red alliance without error");
    }

    @Test
    void testAddVisionMeasurement() {
        Pose2d visionPose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(45));
        double timestamp = 0.0;

        // Should not throw when adding vision measurement
        assertDoesNotThrow(() -> drivetrain.addVisionMeasurement(visionPose, timestamp),
            "addVisionMeasurement should accept pose without error");
    }

    @Test
    void testAddVisionMeasurementWithStdDevs() {
        Pose2d visionPose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(45));
        double timestamp = 0.0;

        // Create standard deviation matrix [x, y, theta]
        Matrix<N3, N1> stdDevs = new Matrix<>(edu.wpi.first.math.Nat.N3(), edu.wpi.first.math.Nat.N1());
        stdDevs.set(0, 0, 0.1); // x std dev
        stdDevs.set(1, 0, 0.1); // y std dev
        stdDevs.set(2, 0, 0.1); // theta std dev

        // Should not throw when adding vision measurement with std devs
        assertDoesNotThrow(() -> drivetrain.addVisionMeasurement(visionPose, timestamp, stdDevs),
            "addVisionMeasurement with stdDevs should accept pose without error");
    }

    @Test
    void testSamplePoseAtReturnsOptional() {
        // samplePoseAt should return an Optional (may be empty if buffer is empty)
        var result = drivetrain.samplePoseAt(0.0);

        assertNotNull(result, "samplePoseAt should return a non-null Optional");
    }

    @Test
    void testFieldCentricRequest() {
        // Create a field-centric drive request
        SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
            .withVelocityX(0.5)
            .withVelocityY(0.0)
            .withRotationalRate(0.0);

        Command command = drivetrain.applyRequest(() -> fieldCentric);

        assertNotNull(command, "Field centric request should create a valid command");
        assertTrue(command.getRequirements().contains(drivetrain),
            "Field centric command should require drivetrain");
    }

    @Test
    void testRobotCentricRequest() {
        // Create a robot-centric drive request
        SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
            .withVelocityX(0.5)
            .withVelocityY(0.0)
            .withRotationalRate(0.0);

        Command command = drivetrain.applyRequest(() -> robotCentric);

        assertNotNull(command, "Robot centric request should create a valid command");
        assertTrue(command.getRequirements().contains(drivetrain),
            "Robot centric command should require drivetrain");
    }

    @Test
    void testCommandSchedulerIntegration() {
        SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
        Command command = drivetrain.applyRequest(() -> idleRequest);

        // Verify command can be scheduled (subsystem requirement check)
        assertDoesNotThrow(() -> CommandScheduler.getInstance().schedule(command),
            "Command should be schedulable without error");

        // Verify the command requires the drivetrain
        assertTrue(command.getRequirements().contains(drivetrain),
            "Command should require the drivetrain subsystem");

        // Cancel any running commands to clean up
        CommandScheduler.getInstance().cancelAll();
    }

    @Test
    void testMultipleRequestsCanBeCreated() {
        SwerveRequest.Idle idle = new SwerveRequest.Idle();
        SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();
        SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();

        Command idleCommand = drivetrain.applyRequest(() -> idle);
        Command fieldCommand = drivetrain.applyRequest(() -> fieldCentric);
        Command robotCommand = drivetrain.applyRequest(() -> robotCentric);

        assertNotNull(idleCommand, "Idle command should be created");
        assertNotNull(fieldCommand, "Field centric command should be created");
        assertNotNull(robotCommand, "Robot centric command should be created");

        // All commands should require the same subsystem
        assertTrue(idleCommand.getRequirements().contains(drivetrain));
        assertTrue(fieldCommand.getRequirements().contains(drivetrain));
        assertTrue(robotCommand.getRequirements().contains(drivetrain));
    }

    @Test
    void testGetCharacterizationReturnsNonNull() {
        SwerveCharacterization characterization = drivetrain.getCharacterization();

        assertNotNull(characterization, "getCharacterization should return a non-null instance");
    }

    @Test
    void testCharacterizationRoutineTypeSwitching() {
        SwerveCharacterization characterization = drivetrain.getCharacterization();

        // Default should be translation
        assertEquals(SwerveCharacterization.RoutineType.TRANSLATION,
            characterization.getActiveRoutineType(),
            "Default routine type should be TRANSLATION");

        // Switch to steer
        characterization.setActiveRoutine(SwerveCharacterization.RoutineType.STEER);
        assertEquals(SwerveCharacterization.RoutineType.STEER,
            characterization.getActiveRoutineType(),
            "Routine type should be STEER after switching");

        // Switch to rotation
        characterization.setActiveRoutine(SwerveCharacterization.RoutineType.ROTATION);
        assertEquals(SwerveCharacterization.RoutineType.ROTATION,
            characterization.getActiveRoutineType(),
            "Routine type should be ROTATION after switching");
    }

    @Test
    void testSysIdCommandsRequireDrivetrain() {
        Command quasistatic = drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward);
        Command dynamic = drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward);

        assertTrue(quasistatic.getRequirements().contains(drivetrain),
            "SysId quasistatic command should require drivetrain");
        assertTrue(dynamic.getRequirements().contains(drivetrain),
            "SysId dynamic command should require drivetrain");
    }
}
