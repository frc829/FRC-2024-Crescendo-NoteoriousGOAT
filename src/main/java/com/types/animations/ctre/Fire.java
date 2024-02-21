package com.types.animations.ctre;

import com.ctre.phoenix.led.FireAnimation;

public class Fire {
    public final double brightness;
    public final double cooling;
    public final double sparking;
    public final double speed;
    public final int numLed;
    public final int ledOffset;
    public final boolean reverse;
    public final FireAnimation ctreVersion;

    public Fire(
            double brightness,
            double cooling,
            double sparking,
            double speed,
            int numLed,
            boolean reverse,
            int ledOffset) {
        this.brightness = brightness;
        this.cooling = cooling;
        this.sparking = sparking;
        this.speed = speed;
        this.numLed = numLed;
        this.reverse = reverse;
        this.ledOffset = ledOffset;
        ctreVersion = new FireAnimation(brightness, speed, numLed, sparking, cooling, reverse, ledOffset);
    }

}
