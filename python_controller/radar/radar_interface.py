from python_controller.models import RadarObservation


class UnconnectedRadar:
    """No sensor or distance is fabricated. Stage 2 must supply a real adapter."""
    def observe(self, now):
        return RadarObservation(None,None,now,False)
