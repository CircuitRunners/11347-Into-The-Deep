package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Diffy extends SubsystemBase{

    //TODO implement analog encoders for axons ⬇️⬇️⬇️
    //get our analog input from the hardwareMap AnalogInput analogInput = hardwareMap.get(AnalogInput.class, "myanaloginput");
    // get the voltage of our analog line // divide by 3.3 (the max voltage) to get a value between 0 and 1 // multiply by 360 to convert it to 0 to 360 degrees double position = analogInput.getVoltage() / 3.3 * 360;

    CRServo leftDiffyServo, rightDiffyServo;
    public Diffy (HardwareMap hardwareMap) {
            leftDiffyServo = hardwareMap.get(CRServo.class, "leftDiffyServo");
            rightDiffyServo = hardwareMap.get(CRServo.class, "rightDiffyServo");
        }


    public void rotateDiffy (double power) {
        leftDiffyServo.setPower(-power);
        rightDiffyServo.setPower(power);      

    }
    
    public void moveDiffy (double power) {
        leftDiffyServo.setPower(power);
        rightDiffyServo.setPower(power);

    }

    public void diffYBrake () {
        leftDiffyServo.setPower(0);
        rightDiffyServo.setPower(0);
    }
}
