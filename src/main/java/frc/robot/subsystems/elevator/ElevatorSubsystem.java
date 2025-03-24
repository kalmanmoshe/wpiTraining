package frc.robot.subsystems.elevator;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Motors.TalonFXMotorCreator;

class ElevatorConstants {
    public static final int PRIMARY_ELEVATOR_MOTOR_ID = 1;
    public static final int SECONDARY_ELEVATOR_MOTOR_ID = 2;
    public static final int HEIGHT_LEVEL_1 = 10;
    public static final int HEIGHT_LEVEL_2 = 30;
    public static final int HEIGHT_LEVEL_3 = 80;
    public static final int HEIGHT_LEVEL_4 = 110;
}

public class ElevatorSubsystem extends SubsystemBase {
    /**
     * The motor that controls the elevator.
     */
    private TalonFX primaryElevatorMotor;
    /**
     * The motor that follows the primary elevator motor.
     */
    private TalonFX secondaryElevatorMotor;
    /**
     * Creates a new ElevatorSubsystem.
     */
    public ElevatorSubsystem() {
        this.primaryElevatorMotor = TalonFXMotorCreator.createTalonFX(ElevatorConstants.PRIMARY_ELEVATOR_MOTOR_ID);
        this.secondaryElevatorMotor = TalonFXMotorCreator.createSlaveTalonint(ElevatorConstants.SECONDARY_ELEVATOR_MOTOR_ID, ElevatorConstants.PRIMARY_ELEVATOR_MOTOR_ID, false);
    }
    private void setSpeed(double speed) {
        this.primaryElevatorMotor.set(speed);
    }
    private void disable() {
        this.primaryElevatorMotor.disable();
    }
    /**
     * Moves the elevator to the specified height level.
     * 
     * @param level The height level to move the elevator to.
     */
    private void setPosition(double rotations){
        this.primaryElevatorMotor.setPosition(rotations);
    }
}

