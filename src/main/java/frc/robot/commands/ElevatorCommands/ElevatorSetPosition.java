package frc.robot.commands.ElevatorCommands;

import frc.DELib25.Subsystems.ServoSubsystem.Base.Motor.ServoSubsystemTalon;
import frc.DELib25.Subsystems.ServoSubsystem.Commands.ServoSubsystemSetPosition;

public class ElevatorSetPosition extends ServoSubsystemSetPosition{
    public ElevatorSetPosition(ServoSubsystemTalon servoSubsystemTalon, double Position) {
        super(servoSubsystemTalon, Position);
    }
}
