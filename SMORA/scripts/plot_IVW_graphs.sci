// Display mode
mode(0);

// Display warning for floating point exception
ieee(1);

FILENAMES = ["statistics_online_SMORA_L.txt", "statistics_online_SMORA_XL.txt"];
tests = length(length(FILENAMES));

for i = 1:tests
    FILENAME = FILENAMES(i);
    Array = csvRead(FILENAME, ";",[],[],[],[],[],[2]);
    pwm = Array(:,1);
    samples = Array(:,2);
    voltage = Array(:,3);
    current = Array(:,4);
    velocity = Array(:,5);
    
    subplot(tests, 2, i*2-1)
    ax2=gca();
    plot(voltage, current)
    ax2.margins(3)=0.15;
    ax2.margins(4)=0.15;
    //ax2.axes_visible(1)="off";//hide labels ans tics of y axis 
    //xtitle(sprintf("PWM: %d, AVG VELOCITY: %7.3f", PWM, avg_velocity), "us", "degrees")
    xtitle(FILENAME);
    xlabel("Voltage (V)");
    ylabel("Current (mA)");
    
    subplot(tests, 2, i*2)
    ax1=gca(); 
    plot(voltage, velocity)
    ax1.margins(3)=0.15;
    ax1.margins(4)=0.15;
    //ax1.axes_visible(1)="off";//hide labels ans tics of y axis
    //xtitle(sprintf("CURRENT - MEAN:%7.3f, DEV:%7.3f", avg_current, deviation_current), "us", "mA")
    xtitle(FILENAME);
    xlabel("Voltage (v)");
    ylabel("Velocity (°/s)");
end
