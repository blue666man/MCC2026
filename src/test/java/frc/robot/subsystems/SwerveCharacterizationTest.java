package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicReference;

/**
 * Test suite for SwerveCharacterization.
 *
 * These tests verify the SysId routine management independently
 * from the full drivetrain hardware.
 */
class SwerveCharacterizationTest {

    private TestSubsystem testSubsystem;
    private SwerveCharacterization characterization;
    private AtomicReference<SwerveRequest> lastRequest;

    /** Minimal subsystem for testing characterization without hardware */
    private static class TestSubsystem extends SubsystemBase {
        // Empty subsystem for testing
    }

    @BeforeEach
    void setUp() {
        assert HAL.initialize(500, 0);

        testSubsystem = new TestSubsystem();
        lastRequest = new AtomicReference<>();

        // Create characterization with a consumer that captures the request
        characterization = new SwerveCharacterization(
            request -> lastRequest.set(request),
            testSubsystem
        );

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().enable();
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        HAL.shutdown();
    }

    @Test
    void testDefaultActiveRoutineIsTranslation() {
        assertEquals(SwerveCharacterization.RoutineType.TRANSLATION,
            characterization.getActiveRoutineType(),
            "Default active routine should be TRANSLATION");
    }

    @Test
    void testSetActiveRoutineToSteer() {
        characterization.setActiveRoutine(SwerveCharacterization.RoutineType.STEER);

        assertEquals(SwerveCharacterization.RoutineType.STEER,
            characterization.getActiveRoutineType(),
            "Active routine should be STEER after setting");
    }

    @Test
    void testSetActiveRoutineToRotation() {
        characterization.setActiveRoutine(SwerveCharacterization.RoutineType.ROTATION);

        assertEquals(SwerveCharacterization.RoutineType.ROTATION,
            characterization.getActiveRoutineType(),
            "Active routine should be ROTATION after setting");
    }

    @Test
    void testSetActiveRoutineBackToTranslation() {
        characterization.setActiveRoutine(SwerveCharacterization.RoutineType.ROTATION);
        characterization.setActiveRoutine(SwerveCharacterization.RoutineType.TRANSLATION);

        assertEquals(SwerveCharacterization.RoutineType.TRANSLATION,
            characterization.getActiveRoutineType(),
            "Active routine should be TRANSLATION after switching back");
    }

    @Test
    void testQuasistaticForwardReturnsCommand() {
        Command command = characterization.quasistatic(SysIdRoutine.Direction.kForward);

        assertNotNull(command, "quasistatic forward should return a non-null Command");
    }

    @Test
    void testQuasistaticReverseReturnsCommand() {
        Command command = characterization.quasistatic(SysIdRoutine.Direction.kReverse);

        assertNotNull(command, "quasistatic reverse should return a non-null Command");
    }

    @Test
    void testDynamicForwardReturnsCommand() {
        Command command = characterization.dynamic(SysIdRoutine.Direction.kForward);

        assertNotNull(command, "dynamic forward should return a non-null Command");
    }

    @Test
    void testDynamicReverseReturnsCommand() {
        Command command = characterization.dynamic(SysIdRoutine.Direction.kReverse);

        assertNotNull(command, "dynamic reverse should return a non-null Command");
    }

    @Test
    void testGetTranslationRoutineReturnsNonNull() {
        assertNotNull(characterization.getTranslationRoutine(),
            "getTranslationRoutine should return a non-null SysIdRoutine");
    }

    @Test
    void testGetSteerRoutineReturnsNonNull() {
        assertNotNull(characterization.getSteerRoutine(),
            "getSteerRoutine should return a non-null SysIdRoutine");
    }

    @Test
    void testGetRotationRoutineReturnsNonNull() {
        assertNotNull(characterization.getRotationRoutine(),
            "getRotationRoutine should return a non-null SysIdRoutine");
    }

    @Test
    void testAllRoutinesAreDifferent() {
        SysIdRoutine translation = characterization.getTranslationRoutine();
        SysIdRoutine steer = characterization.getSteerRoutine();
        SysIdRoutine rotation = characterization.getRotationRoutine();

        assertNotSame(translation, steer, "Translation and steer routines should be different");
        assertNotSame(translation, rotation, "Translation and rotation routines should be different");
        assertNotSame(steer, rotation, "Steer and rotation routines should be different");
    }

    @Test
    void testQuasistaticCommandRequiresSubsystem() {
        Command command = characterization.quasistatic(SysIdRoutine.Direction.kForward);

        assertTrue(command.getRequirements().contains(testSubsystem),
            "Quasistatic command should require the subsystem");
    }

    @Test
    void testDynamicCommandRequiresSubsystem() {
        Command command = characterization.dynamic(SysIdRoutine.Direction.kForward);

        assertTrue(command.getRequirements().contains(testSubsystem),
            "Dynamic command should require the subsystem");
    }

    @Test
    void testSwitchingRoutineChangesQuasistaticCommand() {
        Command translationCmd = characterization.quasistatic(SysIdRoutine.Direction.kForward);

        characterization.setActiveRoutine(SwerveCharacterization.RoutineType.STEER);
        Command steerCmd = characterization.quasistatic(SysIdRoutine.Direction.kForward);

        // Commands should be different objects since they come from different routines
        assertNotSame(translationCmd, steerCmd,
            "Commands from different routines should be different objects");
    }

    @Test
    void testSwitchingRoutineChangesDynamicCommand() {
        Command translationCmd = characterization.dynamic(SysIdRoutine.Direction.kForward);

        characterization.setActiveRoutine(SwerveCharacterization.RoutineType.ROTATION);
        Command rotationCmd = characterization.dynamic(SysIdRoutine.Direction.kForward);

        assertNotSame(translationCmd, rotationCmd,
            "Commands from different routines should be different objects");
    }
}
