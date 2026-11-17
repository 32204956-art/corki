def relative_height_ratio(target, frame_height):
    """Dimensionless RGB proxy; never a physical distance measurement.

    A future range-based controller can inject its observation-to-control proxy
    into FollowController without changing camera capture or UDP commands.
    """
    return (target.bbox[3]-target.bbox[1])/frame_height
