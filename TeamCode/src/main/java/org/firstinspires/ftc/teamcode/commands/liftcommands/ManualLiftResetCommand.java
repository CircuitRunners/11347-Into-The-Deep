package org.firstinspires.ftc.teamcode.commands.liftcommands;


import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.Slides;

public class ManualLiftResetCommand extends CommandBase {
    private final Slides lift;
    private final GamepadEx manipulator;


    public ManualLiftResetCommand(Slides lift, GamepadEx manipulator){
        addRequirements(lift);

        this.lift = lift;
        this.manipulator = manipulator;
    }

    @Override
    public void execute(){
        if(manipulator.getButton(GamepadKeys.Button.DPAD_LEFT)){
            lift.setLiftPower(-0.3);
        } else {
            lift.brake_power();
        }
    }


    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        lift.brake_power();
        lift.resetLiftPosition();
    }
}