package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.BooleanUtil.StableBoolean;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorMagneticHoming extends Command {
    ElevatorSubsystem elevator;
    StableBoolean isAtResetPoint;
    boolean skipSetPosition = false;
    private Timer timer;
    public ElevatorMagneticHoming(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        isAtResetPoint = new StableBoolean(0.2);
        addRequirements(elevator);
    }
    /**
     * Prepares the elevator for homing by checking its position.
     * Skips positioning if already near the reset point.
     */
    @Override
    public void initialize() {
        skipSetPosition = elevator.getPosition() < 20;
        if (!skipSetPosition) {
            elevator.setMotionMagicPosition(20);
        }
    }
    /**
     * Executes the elevator homing process using a magnetic sensor.
     * Ensures the reset process is completed after a 1-second delay.
     */
    @Override
    public void execute() {
        if (elevator.isAtSetpoint() || skipSetPosition){
            elevator.setPrecentOutput(-0.1);
        }
        if (isAtResetPoint.get(elevator.getMagnetState())) {
            timer.reset();
            if(timer.hasElapsed(1)){
                elevator.resetPosition(0);
                elevator.disableMotors();
            }
            else{
                elevator.disableMotors();
            }
        }
    }
    
}
