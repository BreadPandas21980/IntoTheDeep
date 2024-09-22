package org.firstinspires.ftc.teamcode.util;


public class RateLimiter {

    public double input;
    public double maxRateOfChange;
    public long lastTime = System.nanoTime();
    public double lastOutput;
    public double output;

    public RateLimiter(double input, double maxRateOfChange) {
        this.input = input;
        this.maxRateOfChange = maxRateOfChange;
    }

    public double findOutput() {
        long currentTime =  System.nanoTime();
        double loopTime = (double)(currentTime - lastTime) / 1.0E9; //in seconds
        lastTime = currentTime;
        double maxChange = loopTime * maxRateOfChange;
        output += constrain(input - lastOutput, -maxChange, maxChange);
        lastOutput = output;
        return output;
    }

    public double constrain(double inputToBeCapped, double min, double max) {
        if(inputToBeCapped > max) {
            inputToBeCapped = max;
        }
        if(inputToBeCapped < min) {
            inputToBeCapped = min;
        }
        return inputToBeCapped;
    }
}
