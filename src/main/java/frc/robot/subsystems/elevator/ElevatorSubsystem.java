package frc.robot.subsystems.elevator;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private static final class ElevatorSubsystemConstants{
        public static final int MOTOR_ID = 1; 
        public static final int SLAVE_MOTOR_ID = 2;
        public static final int heightLevle_1=10;
        public static final int heightLevle_2=30;
        public static final int heightLevle_3=80;
        public static final int heightLevle_4=110;
    }
    private TalonFX motor;
    private TalonFX slaveMotor;

    public ElevatorSubsystem() {
        this.motor = new TalonFX(ElevatorSubsystemConstants.MOTOR_ID);
        this.slaveMotor = new TalonFX(ElevatorSubsystemConstants.SLAVE_MOTOR_ID);
    }
    private void setSpeed(double speed) {
        this.motor.set(speed);
        this.slaveMotor.set(speed);
    }
    private void stop() {
        this.motor.set(0);
        this.slaveMotor.set(0);
    }
    private void setHeight(double rotations){
        this.motor.setPosition(rotations);
        this.slaveMotor.setPosition(rotations);
    }
}