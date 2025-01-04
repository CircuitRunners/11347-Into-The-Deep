package org.firstinspires.ftc.teamcode.commands.presets

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.commands.liftcommands.ProfiledLiftCommand
import org.firstinspires.ftc.teamcode.subsystems.SlidesPID
import org.firstinspires.ftc.teamcode.subsystems.SlidesPID.SlidePositions
import org.firstinspires.ftc.teamcode.subsystems.ArmCorrected
import org.firstinspires.ftc.teamcode.subsystems.Slides
import org.firstinspires.ftc.teamcode.subsystems.Diffy

class LiftToScoringCommand(diffy: Diffy, lift: Slides, preset: Presets, arm: ArmCorrected) : ParallelCommandGroup() {

    enum class Presets {
        DOWN,
        AUTO,
        SHORT,
        MID,
        HIGH
    }

    init {
        addCommands(
            ParallelCommandGroup(
                SequentialCommandGroup(
                    // Change this ms to change when the arm comes up
                    WaitCommand(1000),
                    InstantCommand({
                        arm.toBasketPos();
                    }),
                        WaitCommand(200),
                        InstantCommand({
                            diffy.centerDiffy();
                        }),
                ),

                when (preset) {
                    Presets.DOWN ->
                        ProfiledLiftCommand(lift, SlidePositions.STAGE_0.position, false)
                    Presets.AUTO ->
                        ProfiledLiftCommand(lift, SlidePositions.AUTO.position, true)
                    Presets.SHORT ->
                        ProfiledLiftCommand(lift, SlidePositions.STAGE_1.position, true)
                    Presets.MID ->
                        ProfiledLiftCommand(lift, SlidePositions.STAGE_2.position, false)
                    Presets.HIGH ->
                        ProfiledLiftCommand(lift, SlidePositions.STAGE_3.position, true)

                }
            )
        )
        addRequirements(lift)
    }
}
