package org.firstinspires.ftc.teamcode.commands.presets

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.commands.liftcommands.ProfiledLiftCommand
import org.firstinspires.ftc.teamcode.subsystems.SlidesPID
import org.firstinspires.ftc.teamcode.subsystems.SlidesPID.SlidePositions
import org.firstinspires.ftc.teamcode.subsystems.ArmCorrected
class LiftToRestingCommand(lift: SlidesPID, preset: Presets, arm: ArmCorrected) : ParallelCommandGroup() {

    enum class Presets {
        DOWN,
    }

    init {
        addCommands(
            ParallelCommandGroup(
                SequentialCommandGroup(
                    // Change this ms to change when the arm comes up
                    WaitCommand(800),
                    InstantCommand({
                        arm.toRestPos();
                    }),
                ),
                InstantCommand({

                }),
                when (preset) {
                    Presets.DOWN ->
                        ProfiledLiftCommand(lift, SlidePositions.STAGE_0.position, false)

                }
            )
        )
        addRequirements(lift)
    }
}
