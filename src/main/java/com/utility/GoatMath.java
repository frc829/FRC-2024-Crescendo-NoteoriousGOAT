package com.utility;

public class GoatMath {
    private GoatMath(){
    }

    public static final double round(double value, int decimalPoints){
        double sign = Math.signum(value);
        value = Math.abs(value);
        value *= Math.pow(10, decimalPoints);
        value = Math.floor(value);
        value /= Math.pow(10, decimalPoints);
        return sign * value;

    }
}
