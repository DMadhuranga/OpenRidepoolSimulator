#!/usr/bin/env python3
"""Generate map/times.npy (the all-pairs travel-time matrix) from map/edges.csv.

times.npy is derived data: an all-pairs shortest-path solve over the directed,
weighted graph in edges.csv, stored as a dense uint16 matrix.  For Chicago that
is 44095 x 44095 = 3.9 GB built from a 1.7 MB edge list, which is why the input
archives ship edges.csv and leave this file to be generated.

Usage:
    python3 data/gen_times_npy.py data/chicago
    python3 data/gen_times_npy.py data/chicago --jobs 16
    python3 data/gen_times_npy.py data/chicago --verify /elsewhere/times.npy

Requires numpy and scipy.
"""
import argparse
import multiprocessing as mp
import os
import sys
import time

import numpy as np

try:
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import dijkstra
except ImportError:
    sys.exit("error: scipy is required (pip install scipy)")

UINT16_MAX = 65535

# Per-worker state, set once by _init.
_G = _OUT_PATH = _N = _OFFSET = None


def node_count(city_dir, edges):
    """Matrix dimension: the node id space, not just the ids that have edges.

    The simulator indexes the matrix by node id, so an isolated node still
    occupies a row.  nodes.csv is authoritative when present.
    """
    from_edges = int(max(edges[:, 0].max(), edges[:, 1].max()))
    nodes_csv = os.path.join(city_dir, "map", "nodes.csv")
    if os.path.exists(nodes_csv):
        with open(nodes_csv) as f:
            from_nodes = sum(1 for line in f if line.strip())
        if from_nodes != from_edges:
            print(f"  note: nodes.csv has {from_nodes} rows, edges.csv tops out "
                  f"at {from_edges}; using {max(from_nodes, from_edges)}")
        return max(from_nodes, from_edges)
    return from_edges


def load_graph(city_dir):
    edges_csv = os.path.join(city_dir, "map", "edges.csv")
    if not os.path.exists(edges_csv):
        sys.exit(f"error: {edges_csv} not found")
    edges = np.loadtxt(edges_csv, delimiter=",", dtype=np.int64)
    if edges.ndim != 2 or edges.shape[1] < 3:
        sys.exit(f"error: expected 'origin,dest,weight' rows in {edges_csv}")
    n = node_count(city_dir, edges)
    # edges.csv is 1-based; the simulator subtracts 1 (network.cpp).
    g = coo_matrix((edges[:, 2], (edges[:, 0] - 1, edges[:, 1] - 1)),
                   shape=(n, n)).tocsr()
    return g, n, len(edges)


def _init(g, out_path, n, offset):
    global _G, _OUT_PATH, _N, _OFFSET
    _G, _OUT_PATH, _N, _OFFSET = g, out_path, n, offset


def _solve_chunk(bounds):
    """Dijkstra for rows [lo, hi), written straight into the memmap."""
    lo, hi = bounds
    dist = dijkstra(_G, indices=list(range(lo, hi)), directed=True)
    finite = np.isfinite(dist)
    unreachable = int(dist.size - finite.sum())
    overflow = int((dist[finite] > UINT16_MAX).sum())
    np.clip(dist, 0, UINT16_MAX, out=dist)
    out = np.memmap(_OUT_PATH, dtype=np.uint16, mode="r+",
                    offset=_OFFSET, shape=(_N, _N))
    out[lo:hi] = dist.astype(np.uint16)
    out.flush()
    del out
    return hi - lo, unreachable, overflow


def _progress(done, n, t0):
    elapsed = time.time() - t0
    rate = done / elapsed if elapsed else 0
    eta = (n - done) / rate if rate else 0
    sys.stdout.write(f"\r  {done}/{n} rows  {rate:.0f} rows/s  "
                     f"eta {eta / 60:.1f} min   ")
    sys.stdout.flush()


def allocate(out_path, n):
    """Write the .npy header, then size the file so workers can fill it in
    place -- the 3.9 GB is never held in memory."""
    with open(out_path, "wb") as f:
        np.lib.format.write_array_header_2_0(
            f, {"descr": "<u2", "fortran_order": False, "shape": (n, n)})
        header_len = f.tell()
    with open(out_path, "r+b") as f:
        f.truncate(header_len + n * n * 2)
    return header_len


def generate(city_dir, jobs, chunk):
    g, n, n_edges = load_graph(city_dir)
    out_path = os.path.join(city_dir, "map", "times.npy")
    print(f"edges={n_edges}  nodes={n}  output={out_path}  "
          f"({n * n * 2 / 1e9:.2f} GB)  jobs={jobs}")

    offset = allocate(out_path, n)
    bounds = [(lo, min(lo + chunk, n)) for lo in range(0, n, chunk)]

    t0 = time.time()
    done = unreachable = overflow = 0
    if jobs == 1:
        _init(g, out_path, n, offset)
        results = (_solve_chunk(b) for b in bounds)
        for d, u, o in results:
            done += d; unreachable += u; overflow += o
            _progress(done, n, t0)
    else:
        with mp.Pool(jobs, initializer=_init,
                     initargs=(g, out_path, n, offset)) as pool:
            for d, u, o in pool.imap_unordered(_solve_chunk, bounds):
                done += d; unreachable += u; overflow += o
                _progress(done, n, t0)
    print()
    if unreachable:
        print(f"  warning: {unreachable} unreachable pairs stored as {UINT16_MAX}")
    if overflow:
        print(f"  warning: {overflow} distances exceeded {UINT16_MAX}, clipped")
    print(f"done in {time.time() - t0:.1f}s")
    return out_path


def verify(city_dir, reference, samples):
    """Spot-check the generated matrix against a reference times.npy."""
    mine = np.load(os.path.join(city_dir, "map", "times.npy"), mmap_mode="r")
    theirs = np.load(reference, mmap_mode="r")
    if mine.shape != theirs.shape:
        sys.exit(f"shape mismatch: {mine.shape} vs {theirs.shape}")
    rng = np.random.default_rng(0)
    bad = 0
    for r in rng.integers(0, mine.shape[0], samples):
        d = np.abs(np.asarray(mine[r], dtype=np.int64)
                   - np.asarray(theirs[r], dtype=np.int64))
        if d.max():
            bad += 1
            print(f"  row {r}: {int((d != 0).sum())} differing cells, "
                  f"max |diff| {int(d.max())}")
    print(f"verify: {samples - bad}/{samples} sampled rows match exactly "
          f"({mine.shape[0]} x {mine.shape[1]} matrix)")
    return bad == 0


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("city_dir", help="a DATAROOT directory, e.g. data/chicago")
    p.add_argument("--jobs", type=int, default=min(8, os.cpu_count() or 1),
                   help="parallel workers (default: min(8, cores))")
    p.add_argument("--chunk", type=int, default=250,
                   help="source rows per task (default: 250)")
    p.add_argument("--verify", metavar="REFERENCE_NPY",
                   help="after generating, compare sampled rows with this file")
    p.add_argument("--samples", type=int, default=20,
                   help="rows to sample when verifying (default: 20)")
    p.add_argument("--verify-only", action="store_true",
                   help="skip generation, just verify what is already there")
    args = p.parse_args()

    if not args.verify_only:
        generate(args.city_dir, max(1, args.jobs), args.chunk)
    if args.verify and not verify(args.city_dir, args.verify, args.samples):
        sys.exit(1)


if __name__ == "__main__":
    main()
