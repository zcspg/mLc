function X = norm_roll(roll)
    while(roll < -180)
        roll = roll + 360;
    end
    while(roll > 180)
        roll = roll - 360;
    end
    X = roll;