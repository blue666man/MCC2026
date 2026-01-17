package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.function.Consumer;

/**
 * Encapsulates SysId characterization routines for a swerve drivetrain.
 *
 * This class manages three types of characterization:
 * - Translation: For finding PID gains for drive motors
 * - Steer: For finding PID gains for steer motors
 * - Rotation: For finding PID gains for the FieldCentricFacingAngle HeadingController
 */
public class SwerveCharacterization {

    /** Available characterization routine types */
    public enum RoutineType {
        TRANSLATION,
        STEER,
        ROTATION
    }

    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
        new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
        new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
        new SwerveRequest.SysIdSwerveRotation();

    private final SysIdRoutine m_sysIdRoutineTranslation;
    private final SysIdRoutine m_sysIdRoutineSteer;
    private final SysIdRoutine m_sysIdRoutineRotation;

    private SysIdRoutine m_activeRoutine;

    /**
     * Creates a new SwerveCharacterization with the given control consumer and subsystem.
     *
     * @param setControl Consumer that applies a SwerveRequest to the drivetrain
     * @param subsystem The subsystem to use for the SysId routines (typically the drivetrain)
     */
    public SwerveCharacterization(Consumer<SwerveRequest> setControl, Subsystem subsystem) {
        m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,        // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                null,        // Use default timeout (10 s)
                state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                output -> setControl.accept(m_translationCharacterization.withVolts(output)),
                null,
                subsystem
            )
        );

        m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,        // Use default ramp rate (1 V/s)
                Volts.of(7), // Use dynamic voltage of 7 V
                null,        // Use default timeout (10 s)
                state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                volts -> setControl.accept(m_steerCharacterization.withVolts(volts)),
                null,
                subsystem
            )
        );

        /*
         * SysId routine for characterizing rotation.
         * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
         * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
         */
        m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                /* This is in radians per second², but SysId only supports "volts per second" */
                Volts.of(Math.PI / 6).per(Second),
                /* This is in radians per second, but SysId only supports "volts" */
                Volts.of(Math.PI),
                null, // Use default timeout (10 s)
                state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                output -> {
                    /* output is actually radians per second, but SysId only supports "volts" */
                    setControl.accept(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                    /* also log the requested output for SysId */
                    SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                },
                null,
                subsystem
            )
        );

        // Default to translation routine
        m_activeRoutine = m_sysIdRoutineTranslation;
    }

    /**
     * Sets which characterization routine to use for subsequent quasistatic/dynamic calls.
     *
     * @param type The routine type to activate
     */
    public void setActiveRoutine(RoutineType type) {
        m_activeRoutine = switch (type) {
            case TRANSLATION -> m_sysIdRoutineTranslation;
            case STEER -> m_sysIdRoutineSteer;
            case ROTATION -> m_sysIdRoutineRotation;
        };
    }

    /**
     * Gets the currently active routine type.
     *
     * @return The active routine type
     */
    public RoutineType getActiveRoutineType() {
        if (m_activeRoutine == m_sysIdRoutineTranslation) {
            return RoutineType.TRANSLATION;
        } else if (m_activeRoutine == m_sysIdRoutineSteer) {
            return RoutineType.STEER;
        } else {
            return RoutineType.ROTATION;
        }
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the active routine.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command quasistatic(SysIdRoutine.Direction direction) {
        return m_activeRoutine.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the active routine.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command dynamic(SysIdRoutine.Direction direction) {
        return m_activeRoutine.dynamic(direction);
    }

    /**
     * Gets the translation characterization routine directly.
     *
     * @return The translation SysIdRoutine
     */
    public SysIdRoutine getTranslationRoutine() {
        return m_sysIdRoutineTranslation;
    }

    /**
     * Gets the steer characterization routine directly.
     *
     * @return The steer SysIdRoutine
     */
    public SysIdRoutine getSteerRoutine() {
        return m_sysIdRoutineSteer;
    }

    /**
     * Gets the rotation characterization routine directly.
     *
     * @return The rotation SysIdRoutine
     */
    public SysIdRoutine getRotationRoutine() {
        return m_sysIdRoutineRotation;
    }
}
