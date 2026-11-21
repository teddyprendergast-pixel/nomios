"""Low-level SolidWorks (.SLDASM/.SLDPRT) stream extraction and binary STL I/O."""

from __future__ import annotations

import re
import struct
import zlib
from pathlib import Path
from typing import Dict

import numpy as np


def unpack_sld_streams(filepath: Path) -> Dict[str, bytes]:
    """Decompress internal G3/OPC deflate streams from a SolidWorks file."""
    data = filepath.read_bytes()
    streams: Dict[str, bytes] = {}

    # SolidWorks G3 containers store internal streams with local zip-like headers
    # starting with 0x14 0x00 0x06 0x00 0x08 0x00 and nibble-swapped stream names.
    for match in re.finditer(b"\x14\x00\x06\x00\x08\x00", data):
        idx = match.start()
        _, _, _, _, _, csize, usize, namelen = struct.unpack_from("<HHHIIIII", data, idx)
        if namelen > 200 or idx + 26 + namelen + csize > len(data):
            continue

        name_raw = data[idx + 26 : idx + 26 + namelen]
        name = bytes(((b & 0x0F) << 4) | (b >> 4) for b in name_raw).decode("ascii", errors="ignore")
        payload = data[idx + 26 + namelen : idx + 26 + namelen + csize]

        try:
            decompressed = zlib.decompress(payload, -15)
            if len(decompressed) == usize:
                streams[name] = decompressed
        except zlib.error:
            pass

    return streams


def extract_tessellation_tris(chunk: bytes) -> np.ndarray:
    """Extract (N, 3, 3) float32 triangle vertices in meters from a SolidWorks display list."""
    all_tris = []
    pos = 0
    face_sig = b"\x04\x00\x00\x00\x08\x00\x00\x00\x02\x00\x00\x00"

    while True:
        idx = chunk.find(face_sig, pos)
        if idx == -1 or idx + 16 > len(chunk):
            break

        strip_count = struct.unpack_from("<I", chunk, idx + 12)[0]
        v_hdr_pos = idx + 16 + 4 * strip_count
        if 0 < strip_count < 10000 and v_hdr_pos + 16 <= len(chunk):
            vhdr = struct.unpack_from("<4I", chunk, v_hdr_pos)
            # Vertex buffer header: stride=12 bytes, tag=100, type=2 (float32), count
            if vhdr[0] == 12 and vhdr[1] == 100 and vhdr[2] == 2:
                v_cnt = vhdr[3]
                strips = struct.unpack_from(f"<{strip_count}I", chunk, idx + 16)
                if sum(strips) == v_cnt and v_hdr_pos + 16 + 12 * v_cnt <= len(chunk):
                    verts = np.frombuffer(
                        chunk, dtype=np.float32, count=v_cnt * 3, offset=v_hdr_pos + 16
                    ).reshape((v_cnt, 3))

                    n_hdr_pos = v_hdr_pos + 16 + 12 * v_cnt
                    norms = None
                    if n_hdr_pos + 16 + 12 * v_cnt <= len(chunk):
                        nhdr = struct.unpack_from("<4I", chunk, n_hdr_pos)
                        if nhdr == (12, 100, 2, v_cnt):
                            norms = np.frombuffer(
                                chunk, dtype=np.float32, count=v_cnt * 3, offset=n_hdr_pos + 16
                            ).reshape((v_cnt, 3))

                    v_offset = 0
                    for s_len in strips:
                        for i in range(s_len - 2):
                            if i % 2 == 0:
                                tri = (verts[v_offset + i], verts[v_offset + i + 1], verts[v_offset + i + 2])
                            else:
                                tri = (verts[v_offset + i], verts[v_offset + i + 2], verts[v_offset + i + 1])

                            # Flip winding if it disagrees with stored vertex normals
                            if norms is not None:
                                fn = np.cross(tri[1] - tri[0], tri[2] - tri[0])
                                avg_n = norms[v_offset + i] + norms[v_offset + i + 1] + norms[v_offset + i + 2]
                                if np.dot(fn, avg_n) < 0:
                                    tri = (tri[0], tri[2], tri[1])
                            all_tris.append(tri)
                        v_offset += s_len

                    pos = n_hdr_pos + 16 + (12 * v_cnt if norms is not None else 0)
                    continue

        pos = idx + 4

    if not all_tris:
        return np.zeros((0, 3, 3), dtype=np.float32)
    return np.array(all_tris, dtype=np.float32)


def read_binary_stl(filepath: Path) -> np.ndarray:
    """Read a binary STL file into an (N, 3, 3) float32 triangle array."""
    data = filepath.read_bytes()
    num_tris = struct.unpack_from("<I", data, 80)[0]
    dtype = np.dtype([
        ("normal", "<f4", (3,)),
        ("v0", "<f4", (3,)),
        ("v1", "<f4", (3,)),
        ("v2", "<f4", (3,)),
        ("attr", "<u2"),
    ])
    records = np.frombuffer(data, dtype=dtype, count=num_tris, offset=84)
    return np.stack([records["v0"], records["v1"], records["v2"]], axis=1).astype(np.float32)


def write_binary_stl(filepath: Path, tris: np.ndarray, header_text: str = "Nomios SolidWorks Binary STL") -> None:
    """Write an (N, 3, 3) float32 triangle array to a standard binary STL file."""
    filepath.parent.mkdir(parents=True, exist_ok=True)
    hdr = header_text.encode("ascii", errors="ignore")[:80]
    hdr = hdr + b"\x00" * (80 - len(hdr))
    num_tris = len(tris)

    v0, v1, v2 = tris[:, 0, :], tris[:, 1, :], tris[:, 2, :]
    normals = np.cross(v1 - v0, v2 - v0)
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.where(norms > 1e-12, normals / np.maximum(norms, 1e-12), 0.0).astype(np.float32)

    dtype = np.dtype([
        ("normal", "<f4", (3,)),
        ("v0", "<f4", (3,)),
        ("v1", "<f4", (3,)),
        ("v2", "<f4", (3,)),
        ("attr", "<u2"),
    ])
    records = np.empty(num_tris, dtype=dtype)
    records["normal"] = normals
    records["v0"] = v0
    records["v1"] = v1
    records["v2"] = v2
    records["attr"] = 0

    with open(filepath, "wb") as f:
        f.write(hdr)
        f.write(struct.pack("<I", num_tris))
        f.write(records.tobytes())
