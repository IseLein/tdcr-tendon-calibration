"""
Utility functions for TDCR calibration.
"""


def draw_progress_bar(current: float, target: float, width: int = 30) -> str:
    """
    Draw a text-based progress bar.

    Args:
        current: Current value
        target: Target value
        width: Width of the progress bar in characters

    Returns:
        Formatted progress bar string
    """
    percentage = min(100, int((current / target) * 100))
    filled = int((percentage / 100) * width)
    bar = "=" * filled + ">" + " " * (width - filled - 1)
    return f"[{bar}] {percentage}%"
