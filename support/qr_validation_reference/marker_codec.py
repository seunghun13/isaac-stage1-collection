"""Scene marker wire format and independent, offline RGB decoder.

The decoder accepts image pixels plus static marker geometry only.  It neither
accepts nor guesses an expected epoch, camera, tick, timestamp or GT state.
"""
import binascii
import struct

import numpy as np

SCHEMA = "mro_scene_marker_v1"
MAGIC = 0xA7
WARMUP_TICK = 65535
DEFAULT_LAYOUT = {"x": 12, "y": 12, "cell_px": 8, "cols": 12, "rows": 10}


def border_pattern():
    """Asymmetric fixed perimeter: also supplies dark and bright references."""
    grid = np.zeros((10, 12), dtype=np.uint8)
    grid[0] = [1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1]
    grid[-1] = [0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0]
    grid[1:-1, 0] = [1, 0, 1, 1, 0, 0, 1, 0]
    grid[1:-1, -1] = [0, 0, 1, 0, 1, 1, 0, 1]
    return grid


def encode_marker(epoch, tick, camera_id):
    """Return 10x12 uint8 cells. Tick 65535 is reserved for warmup."""
    if not 0 <= int(epoch) <= 0xFFFFFFFF:
        raise ValueError("epoch must fit uint32")
    if not 0 <= int(tick) <= 0xFFFF:
        raise ValueError("tick must fit uint16")
    if not 1 <= int(camera_id) <= 255:
        raise ValueError("camera_id must be 1..255")
    payload = struct.pack(">BBIH", MAGIC, int(camera_id), int(epoch), int(tick))
    encoded = payload + struct.pack(">H", binascii.crc_hqx(payload, 0xFFFF))
    bits = np.unpackbits(np.frombuffer(encoded, dtype=np.uint8), bitorder="big")
    cells = border_pattern()
    cells[1:-1, 1:-1] = bits.reshape(8, 10)
    return cells


def encoding_metadata():
    return {
        "schema": SCHEMA,
        "layout": dict(DEFAULT_LAYOUT),
        "magic": MAGIC,
        "byte_order": "big endian; bits MSB first; interior row-major",
        "payload": ["magic uint8", "camera_id uint8", "epoch uint32", "scene_tick uint16", "CRC16 uint16"],
        "crc": "CRC-16/CCITT-FALSE: polynomial 0x1021, init 0xffff, no xorout; over first 8 bytes",
        "fixed_border": border_pattern().tolist(),
        "warmup_tick": WARMUP_TICK,
        "valid_recorded_tick_range": [0, WARMUP_TICK - 1],
        "decoder": {
            "sample": "central half of each projected cell; RGB channel average then median",
            "minimum_reference_contrast_8bit": 60,
            "max_dark_normalized": 0.28,
            "min_bright_normalized": 0.72,
            "max_cell_interquartile_range_normalized": 0.30,
            "on_ambiguity": "reject; no expected-code nearest match or bit correction",
        },
    }


def decode_marker(rgb, layout=None):
    """Decode image independently, retaining reasons and pixel-level margins.

    Values between the dark and bright thresholds, a nonuniform center, an
    incorrect fixed perimeter, and a CRC mismatch all fail closed.
    """
    layout = dict(DEFAULT_LAYOUT if layout is None else layout)
    result = {"schema": SCHEMA, "valid": False, "reason": None}
    image = np.asarray(rgb)
    if image.ndim != 3 or image.shape[2] < 3 or image.dtype != np.uint8:
        return dict(result, reason="expected uint8 HxWx3 RGB image")
    try:
        x, y, size, cols, rows = (int(layout[k]) for k in ("x", "y", "cell_px", "cols", "rows"))
    except (KeyError, TypeError, ValueError):
        return dict(result, reason="invalid static layout")
    if cols != 12 or rows != 10 or size < 4 or x < 0 or y < 0:
        return dict(result, reason="unsupported static layout")
    if x + cols * size > image.shape[1] or y + rows * size > image.shape[0]:
        return dict(result, reason="marker ROI outside image")
    inset = max(1, size // 4)
    medians, spreads = np.empty((rows, cols)), np.empty((rows, cols))
    for row in range(rows):
        for col in range(cols):
            patch = image[y + row * size + inset:y + (row + 1) * size - inset,
                          x + col * size + inset:x + (col + 1) * size - inset, :3]
            values = patch.mean(axis=2)
            medians[row, col] = np.median(values)
            spreads[row, col] = np.percentile(values, 75) - np.percentile(values, 25)
    fixed = border_pattern()
    border = np.ones((rows, cols), dtype=bool)
    border[1:-1, 1:-1] = False
    dark = float(np.median(medians[border & (fixed == 0)]))
    bright = float(np.median(medians[border & (fixed == 1)]))
    contrast = bright - dark
    result.update(dark_reference=dark, bright_reference=bright, contrast=contrast)
    if contrast < 60:
        return dict(result, reason="insufficient or inverted reference contrast")
    normalized = (medians - dark) / contrast
    ambiguous = ((normalized > 0.28) & (normalized < 0.72)) | (spreads / contrast > 0.30)
    result["min_threshold_margin"] = float(np.min(np.abs(normalized - 0.5)))
    result["ambiguous_cells"] = np.argwhere(ambiguous).tolist()
    if np.any(ambiguous):
        return dict(result, reason="ambiguous or nonuniform cells")
    cells = (normalized >= 0.5).astype(np.uint8)
    bad_border = border & (cells != fixed)
    if np.any(bad_border):
        return dict(result, reason="fixed border/orientation mismatch", bad_border_cells=np.argwhere(bad_border).tolist())
    encoded = np.packbits(cells[1:-1, 1:-1].reshape(-1), bitorder="big").tobytes()
    magic, camera_id, epoch, tick, observed_crc = struct.unpack(">BBIHH", encoded)
    computed_crc = binascii.crc_hqx(encoded[:8], 0xFFFF)
    result.update(magic=magic, camera_id=camera_id, epoch=epoch, tick=tick,
                  crc_observed=observed_crc, crc_computed=computed_crc)
    if magic != MAGIC:
        return dict(result, reason="magic mismatch")
    if observed_crc != computed_crc:
        return dict(result, reason="CRC mismatch")
    if camera_id == 0:
        return dict(result, reason="reserved camera id")
    return dict(result, valid=True, reason="decoded independently", warmup=(tick == WARMUP_TICK))


def _test_image(cells, dark=25, bright=225):
    """Synthetic decoder unit-test fixture only; never used in capture."""
    layout = DEFAULT_LAYOUT
    image = np.full((270, 480, 3), 100, dtype=np.uint8)
    pixels = np.repeat(np.repeat(cells, layout["cell_px"], axis=0), layout["cell_px"], axis=1)
    image[12:92, 12:108] = np.where(pixels[..., None], bright, dark)
    return image


if __name__ == "__main__":
    for camera_id in (1, 2, 3):
        for tick in (0, 1, 4, 1796, WARMUP_TICK):
            cells = encode_marker(0x12345678, tick, camera_id)
            decoded = decode_marker(_test_image(cells))
            assert decoded["valid"] and decoded["tick"] == tick and decoded["epoch"] == 0x12345678
            assert decoded["camera_id"] == camera_id
    base = encode_marker(0x12345678, 4, 1)
    damaged = base.copy(); damaged[5, 5] ^= 1
    assert decode_marker(_test_image(damaged))["reason"] == "CRC mismatch"
    assert not decode_marker(_test_image(np.rot90(base, 2)))["valid"]
    uncertain = _test_image(base); uncertain[52:60, 52:60] = 125
    assert decode_marker(uncertain)["reason"] == "ambiguous or nonuniform cells"
    assert not decode_marker(_test_image(base, 100, 140))["valid"]
    for delta in (1, 4):
        actual = decode_marker(_test_image(encode_marker(0x12345678, 40-delta, 1)))
        assert actual["valid"] and actual["tick"] != 40
    print("marker codec tests passed: independent decoding; CRC, orientation, ambiguity, low contrast rejection; 1/4 tick distinguishability")
