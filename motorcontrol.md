# DC motor control

## Set_duty

```javascript
function set_duty(float duty){
    set mode to control_duty
    clip the pwm within -max to max

    then convert to direction + absolute duty cycle

    if pwm < min pwm:
        duty -> 0
        brake

    else 
        apply 
}
```