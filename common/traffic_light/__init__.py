"""Traffic-light extension; no camera opener, model or motor publisher."""
from .color_classifier import classify_color
from .plugin import TrafficLightPlugin
from .schemas import ColorEvidence, ColorThresholds

__all__ = ["TrafficLightPlugin", "classify_color", "ColorEvidence", "ColorThresholds"]
