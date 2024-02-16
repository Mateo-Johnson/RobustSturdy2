package frc.robot.lights;

import edu.wpi.first.wpilibj.PWM;

public class Lights {

    public static PWM redLED;
    public static PWM greenLED;
    public static PWM blueLED;
    public static double[] rgbValues;

    public static void makeLightsColors(double[] rgbValues) {
        redLED = new PWM(1); //THESE ARE PLACEHOLDER PORTS
        greenLED = new PWM(2);
        blueLED = new PWM(3);

        redLED.setRaw((int) (rgbValues[0])); 
        greenLED.setRaw((int) (rgbValues[1]));
        blueLED.setRaw((int) (rgbValues[2]));
    }

    //SOLID COLOR
    public static void solidColor(double[] rgbValues) {
        makeLightsColors(rgbValues);
    }

    //COLOR CYCLE
    public static void colorCycle(double[][] colors, int durationInMillis) {
        int numSteps = colors.length;
        for (int i = 0; i < numSteps; i++) {
            makeLightsColors(colors[i]);
            delay(durationInMillis / numSteps);
        }
    }

    //COLOR FLASH
    public static void colorFlash(double[] rgbValues1, double[] rgbValues2, int flashDurationInMillis) {
        makeLightsColors(rgbValues1);
        delay(flashDurationInMillis / 2);
        makeLightsColors(rgbValues2);
        delay(flashDurationInMillis / 2);
    }

    //COLOR PULSE
    public static void colorPulse(double[] rgbValues, int pulseDurationInMillis) {
        double[] originalColor = rgbValues.clone();
        for (int i = 0; i <= 255; i++) {
            double factor = Math.abs(Math.sin(Math.toRadians(i))) * 255;
            double[] newColor = {originalColor[0] * factor / 255, originalColor[1] * factor / 255, originalColor[2] * factor / 255};
            makeLightsColors(newColor);
            delay(pulseDurationInMillis / 255);
        }
    }

    //COLOR FLICKER
    public static void colorFlicker(double[] rgbValues, int flickerDurationInMillis) {
        while (true) {
            double flickerRed = Math.random() * rgbValues[0];
            double flickerGreen = Math.random() * rgbValues[1];
            double flickerBlue = Math.random() * rgbValues[2];
            double[] flickerColor = {flickerRed, flickerGreen, flickerBlue};
            makeLightsColors(flickerColor);
            delay(flickerDurationInMillis);
        }
    }

    //COLOR STROBE
    public static void colorStrobe(double[] rgbValues, int strobeDurationInMillis) {
        while (true) {
            makeLightsColors(rgbValues);
            delay(strobeDurationInMillis / 2);
            makeLightsColors(new double[]{0, 0, 0}); // Turn off
            delay(strobeDurationInMillis / 2);
        }
    }

    //COLOR FADE
    public static void colorFade(double[] startColor, double[] endColor, int fadeDurationInMillis) {
        int numSteps = 100; // Adjust according to the desired smoothness of fade
        for (int i = 0; i <= numSteps; i++) {
            double[] currentColor = {
                startColor[0] + (endColor[0] - startColor[0]) * i / numSteps,
                startColor[1] + (endColor[1] - startColor[1]) * i / numSteps,
                startColor[2] + (endColor[2] - startColor[2]) * i / numSteps
            };
            makeLightsColors(currentColor);
            delay(fadeDurationInMillis / numSteps);
        }
    }

    //COLOR BURST
    public static void colorBurst(double[] rgbValues, double[] burstColor, int burstDurationInMillis) {
        makeLightsColors(burstColor);
        delay(burstDurationInMillis);
        makeLightsColors(rgbValues);
    }

    //COLOR SPARKLE
    public static void colorSparkle(double[] rgbValues, int sparkleDurationInMillis) {
        while (true) {
            int ledIndex = (int) (Math.random() * rgbValues.length);
            double[] sparkleColor = {Math.random() * rgbValues[0], Math.random() * rgbValues[1], Math.random() * rgbValues[2]};
            makeLightsColors(sparkleColor);
            delay(sparkleDurationInMillis);
            makeLightsColors(rgbValues);
        }
    }

    //COLOR WAVE
    public static void colorWave(double[][] colors, int waveDurationInMillis) {
        int numColors = colors.length;
        for (int i = 0; i < numColors; i++) {
            makeLightsColors(colors[i]);
            delay(waveDurationInMillis / numColors);
        }
    }

    private static void delay(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
