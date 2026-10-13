package com.marswars.bt.monitor;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.marswars.bt.core.PortInfo;
import com.marswars.bt.core.PortType;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.junit.jupiter.api.Test;

class NetworkParameterStoreTest {
    @Test
    void publishesTypedDefaultsUnderTuning() {
        NetworkParameterStore store = new NetworkParameterStore();
        PortInfo wait = PortInfo.input("wait_msec", PortType.INT, "3000", "");
        PortInfo speed = PortInfo.input("speed", PortType.DOUBLE, "1.5", "");
        PortInfo flag = PortInfo.input("enabled", PortType.BOOLEAN, "true", "");
        PortInfo name = PortInfo.input("target", PortType.STRING, "HUB", "");

        assertEquals(3000, store.value("Autos/Test/wait_msec", wait));
        assertEquals(1.5, store.value("Autos/Test/speed", speed));
        assertEquals(true, store.value("Autos/Test/enabled", flag));
        assertEquals("HUB", store.value("Autos/Test/target", name));

        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        assertEquals(
                3000.0, nt.getDoubleTopic("/Tuning/Autos/Test/wait_msec").subscribe(-1).get());
        assertEquals("HUB", nt.getStringTopic("/Tuning/Autos/Test/target").subscribe("").get());

        // Untuned entry follows an edited XML default...
        store.value("Autos/Test/wait_msec", PortInfo.input("wait_msec", PortType.INT, "2500", ""));
        assertEquals(
                2500.0, nt.getDoubleTopic("/Tuning/Autos/Test/wait_msec").subscribe(-1).get());

        // ...but a value tuned on the dashboard is kept.
        nt.getEntry("/Tuning/Autos/Test/speed").setDouble(4.0);
        store.value("Autos/Test/speed", PortInfo.input("speed", PortType.DOUBLE, "2.0", ""));
        assertEquals(4.0, nt.getDoubleTopic("/Tuning/Autos/Test/speed").subscribe(-1).get());
    }
}
