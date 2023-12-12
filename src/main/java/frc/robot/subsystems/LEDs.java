package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.collectorStates;

public class LEDs extends SubsystemBase{
    private final AddressableLED led;

    private final AddressableLEDBuffer ledBuffer;

    private boolean animating = false;
    private int animSpot = 0;
    private double lastAnimUpdate = 0;

    /**
     * constructs the leds class
     */
    public LEDs() {
        // create an addressable led object and a buffer for it
        led = new AddressableLED(LEDConstants.PWMPort);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.length);

        // set the length of the strip and give it data
        led.setLength(LEDConstants.length);
        led.setData(ledBuffer);
        led.start();
    }

    @Override
    public void periodic() {
        updateAnimation();

        led.setData(ledBuffer);
    }

    /**
     * set the color of the entire strip to a rgb color
     */
    public void setColorRGB(int r, int g, int b) {
        // loop through the buffer and set each led to the desired color
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
    }

    /**
     * creates a command to set the entire strip to a rgb color
     * @return the generated command
     */
    public Command setColorRGBCommand(int r, int g, int b) {
        return this.runOnce(() -> setColorRGB(r, g, b));
    }

    /**
     * set the color of each led individually
     */
    public void setIndividualColors(int[] r, int[] g, int[] b) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r[i], g[i], b[i]);
        }
    }

    /**
     * create a command to show a rgb color for a specified time
     * @param time the amount of time
     * @return the generated command
     */
    public Command showColorTime(int r, int g, int b, double time) {
        return runOnce(() -> setColorRGB(r, g, b)).andThen(Commands.waitSeconds(time));
    }

    private void updateAnimation() {
        if (animating) {
            if (Timer.getFPGATimestamp() - lastAnimUpdate >= 0.5) {
                for (int i = 0; i < LEDConstants.length; i++) {
                    if (i == animSpot || i == LEDConstants.length-animSpot) {
                        ledBuffer.setRGB(i, 0, 0, 200);

                    }
                }
                animSpot += 1;
                lastAnimUpdate = Timer.getFPGATimestamp();
                if (animSpot > LEDConstants.length/2) {
                    animating = false;
                }
            }
        }
    }

    public void setColorByState(Constants.collectorStates state) {
        if (state.equals(collectorStates.nothing)) {
            setColorRGB(0, 200, 0);
        }
        else if (state.equals(collectorStates.collecting)) {
            setColorRGB(102, 0, 204);
            // a purple color because cube
        }
        else if (state.equals(collectorStates.moving)) {
            setColorRGB(255, 204, 0);
            // yellow to show working on moving
        }
        else if (state.equals(collectorStates.shooting)) {
            animating = true;
        }
    }

    public Command update(Supplier<Constants.collectorStates> state) {
        return this.run(() -> setColorByState(state.get()));
    }

    public Command update() {
        return Commands.none();
    }
}
