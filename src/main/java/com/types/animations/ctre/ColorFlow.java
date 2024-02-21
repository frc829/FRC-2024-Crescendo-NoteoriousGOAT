package com.types.animations.ctre;

import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

public class ColorFlow {

    public final int r;
    public final int g;
    public final int b;
    public final int w;
    public final double speed;
    public final int numLed;
    public final String direction;
    public final int ledOffset;
    public final ColorFlowAnimation ctreVersion;

    public ColorFlow(
            int r,
            int g,
            int b,
            int w,
            double speed,
            int numLed,
            String direction,
            int ledOffset) {
        this.r = r;
        this.g = g;
        this.b = b;
        this.w = w;
        this.speed = speed;
        this.numLed = numLed;
        this.direction = direction;
        this.ledOffset = ledOffset;
        if (direction == "Forward") {
            ctreVersion = new ColorFlowAnimation(r, g, b, w, speed, numLed, Direction.Forward, ledOffset);
        } else {
            ctreVersion = new ColorFlowAnimation(r, g, b, w, speed, numLed, Direction.Backward, ledOffset);
        }
    }

}
