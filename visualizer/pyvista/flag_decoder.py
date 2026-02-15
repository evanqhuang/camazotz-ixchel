"""Navigation flag decoding utilities."""

FLAG_NAMES = {
    0x01: 'ENCODER_ESTIMATED',
    0x02: 'IMU_ESTIMATED',
    0x04: 'DEPTH_VIRTUAL',
    0x08: 'DEPTH_UNVERIFIED',
    0x10: 'NAV_CRITICAL',
    0x20: 'ENCODER_LOST',
    0x40: 'IMU_LOST',
    0x80: 'SENSOR_CONFLICT',
}


def decode_flags(flag: int) -> list[str]:
    """Decode a flag bitfield into list of active flag names."""
    return [name for bit, name in FLAG_NAMES.items() if flag & bit]


def flag_severity(flag: int) -> str:
    """Determine severity level of a flag value.

    Returns:
        'ok', 'warning', or 'critical'
    """
    if flag == 0x00:
        return 'ok'

    critical_mask = 0x10 | 0x20 | 0x40 | 0x80
    if flag & critical_mask:
        return 'critical'

    return 'warning'


def flag_color(flag: int) -> str:
    """Get color name/hex for a flag value based on severity."""
    severity = flag_severity(flag)

    if severity == 'ok':
        return '#00ff00'
    elif severity == 'warning':
        return '#ffd700'
    else:
        return '#ff0000'
