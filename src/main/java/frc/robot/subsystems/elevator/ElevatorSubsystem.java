package frc.robot.subsystems.elevator;
import frc.DELib25.Sensors.BeamBreak;
import frc.DELib25.Subsystems.ServoSubsystem.ServoSubsystemConfiguration;
import frc.DELib25.Subsystems.ServoSubsystem.Base.Motor.ServoSubsystemTalon;


public class ElevatorSubsystem extends ServoSubsystemTalon {
    private BeamBreak elevatorMagnet;
    private boolean magnetState = false;
    public ElevatorSubsystem(ServoSubsystemConfiguration config) {
        super(config);
        elevatorMagnet = new BeamBreak(0);
    }
    public boolean getMagnetState() {
        return magnetState;
    }


}
