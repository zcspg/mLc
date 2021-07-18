function X = norm_yaw(yaw)
    while(yaw < 0)
        yaw = yaw + 360;
    end
    while(yaw >= 360)
        yaw = yaw - 360;
    end
    X = yaw;